#include "imureader.hpp"
#include "i2c_mutex.hpp"

#include <unistd.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <gpiod.h>

IMUReader::IMUReader(unsigned           i2c_bus,
                     unsigned           i2c_address,
                     const char*        gpiochip,
                     unsigned           rdy_line,
                     icm20948::settings settings)
    : imu_(i2c_bus, i2c_address, settings),
      running_(false),
      gpiochip_path_(gpiochip),
      rdy_line_(rdy_line),
      chip_(nullptr),
      request_(nullptr),
      gpio_fd_(-1),
      stop_fd_(-1)
{
}

IMUReader::~IMUReader()
{
    stop();
}

bool IMUReader::init()
{
    return imu_.init();
}

void IMUReader::setCallback(SampleCallback cb)
{
    callback_ = std::move(cb);
}

bool IMUReader::start()
{
    if (running_) return false;

    chip_ = gpiod_chip_open(gpiochip_path_);
    if (!chip_) return false;

    gpiod_line_settings* line_settings = gpiod_line_settings_new();
    gpiod_line_settings_set_direction(line_settings, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(line_settings, GPIOD_LINE_EDGE_RISING);

    gpiod_line_config* line_cfg = gpiod_line_config_new();
    unsigned int offset = static_cast<unsigned int>(rdy_line_);
    gpiod_line_config_add_line_settings(line_cfg, &offset, 1, line_settings);

    gpiod_request_config* req_cfg = gpiod_request_config_new();
    gpiod_request_config_set_consumer(req_cfg, "imu-rdy");

    request_ = gpiod_chip_request_lines(chip_, req_cfg, line_cfg);

    gpiod_request_config_free(req_cfg);
    gpiod_line_config_free(line_cfg);
    gpiod_line_settings_free(line_settings);

    if (!request_) {
        gpiod_chip_close(chip_);
        return false;
    }

    gpio_fd_ = gpiod_line_request_get_fd(request_);
    stop_fd_ = eventfd(0, EFD_NONBLOCK);

    running_ = true;
    worker_thread_ = std::thread(&IMUReader::worker, this);
    return true;
}

void IMUReader::stop()
{
    running_ = false;

    if (stop_fd_ >= 0) {
        uint64_t val = 1;
        write(stop_fd_, &val, sizeof(val));
    }

    if (worker_thread_.joinable()) worker_thread_.join();

    if (request_) {
        gpiod_line_request_release(request_);
        request_ = nullptr;
    }
    if (chip_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }
    if (stop_fd_ >= 0) {
        close(stop_fd_);
        stop_fd_ = -1;
    }
}

void IMUReader::worker()
{
    int epoll_fd = epoll_create1(0);

    epoll_event gpio_ev{};
    gpio_ev.events  = EPOLLIN | EPOLLET;
    gpio_ev.data.fd = gpio_fd_;
    epoll_ctl(epoll_fd, EPOLL_CTL_ADD, gpio_fd_, &gpio_ev);

    epoll_event stop_ev{};
    stop_ev.events  = EPOLLIN;
    stop_ev.data.fd = stop_fd_;
    epoll_ctl(epoll_fd, EPOLL_CTL_ADD, stop_fd_, &stop_ev);

    gpiod_edge_event_buffer* buffer = gpiod_edge_event_buffer_new(1);

    epoll_event fired{};
    while (running_) {
        int n = epoll_wait(epoll_fd, &fired, 1, -1);
        if (n <= 0 || !running_) break;
        if (fired.data.fd == stop_fd_) break;

        if (fired.data.fd == gpio_fd_) {
            int got = gpiod_line_request_read_edge_events(request_, buffer, 1);
            if (got > 0) {
                gpiod_edge_event* event = gpiod_edge_event_buffer_get_event(buffer, 0);
                if (gpiod_edge_event_get_event_type(event) == GPIOD_EDGE_EVENT_RISING_EDGE) {
                    icm20948::IMUSample sample{};

                    // FIX: hold the shared I2C bus mutex for the full read_sample()
                    // call so the ADS1115 thread cannot interleave its own
                    // ioctl(I2C_SLAVE) + read between our register writes.
                    bool ok = false;
                    {
                        std::lock_guard<std::mutex> lk(i2c1_mutex());
                        ok = imu_.read_sample(sample);

                        if (ok) imu_.check_DRDY_INT();
                    }

                    if (ok && callback_) {
                        callback_(sample.ax, sample.ay, sample.az,
                                  sample.gx, sample.gy, sample.gz,
                                  sample.mx, sample.my);
                    }
                }
            }
        }
    }

    gpiod_edge_event_buffer_free(buffer);
    close(epoll_fd);
}