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
    /// Ensure the worker thread and GPIO resources are cleaned up.
    stop();
}

bool IMUReader::init()
{
    /// Initialise the underlying IMU driver.
    return imu_.init();
}

void IMUReader::setCallback(SampleCallback cb)
{
    /// Register the callback for decoded IMU samples.
    callback_ = std::move(cb);
}

bool IMUReader::start()
{
    /// Ignore duplicate starts.
    if (running_) return false;

      /// Open the GPIO chip exposing the DRDY line.
    chip_ = gpiod_chip_open(gpiochip_path_);
    if (!chip_) return false;

      /// Configure the DRDY GPIO as a rising-edge input.
    gpiod_line_settings* line_settings = gpiod_line_settings_new();
    gpiod_line_settings_set_direction(line_settings, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(line_settings, GPIOD_LINE_EDGE_RISING);

    gpiod_line_config* line_cfg = gpiod_line_config_new();
    unsigned int offset = static_cast<unsigned int>(rdy_line_);
    gpiod_line_config_add_line_settings(line_cfg, &offset, 1, line_settings);

      /// Tag the line request for easier debugging.
    gpiod_request_config* req_cfg = gpiod_request_config_new();
    gpiod_request_config_set_consumer(req_cfg, "imu-rdy");

      /// Request ownership of the DRDY line.
    request_ = gpiod_chip_request_lines(chip_, req_cfg, line_cfg);

    gpiod_request_config_free(req_cfg);
    gpiod_line_config_free(line_cfg);
    gpiod_line_settings_free(line_settings);

    if (!request_) {
        gpiod_chip_close(chip_);
        return false;
    }

      /// Get the GPIO event fd and create an eventfd for thread shutdown.
    gpio_fd_ = gpiod_line_request_get_fd(request_);
    stop_fd_ = eventfd(0, EFD_NONBLOCK);

      /// Launch the background event loop.
    running_ = true;
    worker_thread_ = std::thread(&IMUReader::worker, this);
    return true;
}

void IMUReader::stop()
{
    /// Signal the worker loop to exit.
    running_ = false;

      /// Wake epoll_wait() so the worker can notice shutdown.
    if (stop_fd_ >= 0) {
        uint64_t val = 1;
        write(stop_fd_, &val, sizeof(val));
    }

      /// Wait for the worker thread to finish before releasing resources.
    if (worker_thread_.joinable()) worker_thread_.join();

      /// Release GPIO request and chip handles.
    if (request_) {
        gpiod_line_request_release(request_);
        request_ = nullptr;
    }
    if (chip_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }

      /// Close the shutdown event fd.
    if (stop_fd_ >= 0) {
        close(stop_fd_);
        stop_fd_ = -1;
    }
}

void IMUReader::worker()
{
     /// epoll waits on both the GPIO event fd and the shutdown event fd.
    int epoll_fd = epoll_create1(0);

    epoll_event gpio_ev{};
    gpio_ev.events  = EPOLLIN | EPOLLET;
    gpio_ev.data.fd = gpio_fd_;
    epoll_ctl(epoll_fd, EPOLL_CTL_ADD, gpio_fd_, &gpio_ev);

    epoll_event stop_ev{};
    stop_ev.events  = EPOLLIN;
    stop_ev.data.fd = stop_fd_;
    epoll_ctl(epoll_fd, EPOLL_CTL_ADD, stop_fd_, &stop_ev);

      /// Single-slot buffer is enough since events are handled one at a time.
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

                    /// FIX: hold the shared I2C bus mutex for the full read_sample()
                    /// call so the ADS1115 thread cannot interleave its own
                    /// ioctl(I2C_SLAVE) + read between our register writes.
                    bool ok = false;
                    {
                        std::lock_guard<std::mutex> lk(i2c1_mutex());
                        ok = imu_.read_sample(sample);

                        // Optional DRDY status check after the sample read.

                        if (ok) {
                        imu_.check_DRDY_INT();
                                }
                    }
                  
                    /// Forward decoded sample to the client callback
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
