#include "ads1115rpi.h"
#include "i2c_mutex.hpp"

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string>
#include <stdexcept>
#include <thread>
#include <chrono>

ADS1115settings makeDefaultADS1115Settings()
{
    ADS1115settings s{};
    s.i2c_bus      = 1;
    s.address      = ADS1115settings::DEFAULT_ADS1115_ADDRESS;
    s.samplingRate = ADS1115settings::FS128HZ;
    s.pgaGain      = ADS1115settings::FSR2_048;
    s.channel      = ADS1115settings::AIN0;
    s.drdy_chip    = 0;
    s.drdy_gpio    = ADS1115settings::DEFAULT_ALERT_RDY_TO_GPIO;
    return s;
}

void ADS1115rpi::start(ADS1115settings settings)
{
    ads1115settings = settings;

    char gpioFilename[20];
    snprintf(gpioFilename, 19, "/dev/i2c-%d", settings.i2c_bus);
    fd_i2c = open(gpioFilename, O_RDWR);
    if (fd_i2c < 0)
        throw std::invalid_argument("Could not open I2C.");

    if (ioctl(fd_i2c, I2C_SLAVE, settings.address) < 0)
        throw std::invalid_argument("Could not access I2C address.");

    i2c_writeWord(reg_lo_thres, 0x0000);
    i2c_writeWord(reg_hi_thres, 0x8000);

    unsigned r = (0b10000000 << 8);
    r = r | (1 << 2) | (1 << 3);
    r = r | (settings.samplingRate << 5);
    r = r | (settings.pgaGain << 9);
    r = r | (settings.channel << 12) | 1 << 14;
    i2c_writeWord(reg_config, r);

    const std::string chipPath =
        "/dev/gpiochip" + std::to_string(settings.drdy_chip);
    const std::string consumername =
        "gpioconsumer_" + std::to_string(settings.drdy_chip)
        + "_" + std::to_string(settings.drdy_gpio);

    gpiod::line_config line_cfg;
    line_cfg.add_line_settings(
        settings.drdy_gpio,
        gpiod::line_settings()
            .set_direction(gpiod::line::direction::INPUT)
            .set_edge_detection(gpiod::line::edge::RISING)
            .set_bias(gpiod::line::bias::PULL_UP));

    chip    = std::make_shared<gpiod::chip>(chipPath);
    auto builder = chip->prepare_request();
    builder.set_consumer(consumername);
    builder.set_line_config(line_cfg);
    request = std::make_shared<gpiod::line_request>(builder.do_request());

    running = true;
    thr     = std::thread(&ADS1115rpi::worker, this);
}

void ADS1115rpi::setChannel(ADS1115settings::Input channel)
{
    unsigned r = i2c_readWord(reg_config);
    r = r & (~(3 << 12));
    r = r | (channel << 12);
    i2c_writeWord(reg_config, r);
    ads1115settings.channel = channel;
}

void ADS1115rpi::dataReady()
{
    int ret = i2c_readConversion();
    if (ret < 0)
        return;  // silent skip — retries already logged inside i2c_readConversion

    float v = static_cast<float>(ret) / static_cast<float>(0x7fff)
              * fullScaleVoltage();
    if (adsCallbackInterface)
        adsCallbackInterface(v);
}

void ADS1115rpi::worker()
{
    // Pure interrupt-driven — no polling fallback.
    // If ALERT/RDY stops firing, log a warning every 5 s and wait.
    static constexpr int WARN_EVERY = 10;  // 10 × 500 ms = 5 s
    int misses = 0;

    while (running)
    {
        bool edgeSeen = request->wait_edge_events(
            std::chrono::milliseconds(ISR_TIMEOUT_MS));

        if (!running) break;

        if (edgeSeen) {
            misses = 0;
            gpiod::edge_event_buffer buffer;
            request->read_edge_events(buffer, 1);
            dataReady();
        } else {
            ++misses;
            if (misses % WARN_EVERY == 0)
                fprintf(stderr,
                    "ADS1115: no ALERT/RDY edge for ~%d s (misses=%d) "
                    "— check DRDY wiring\n",
                    static_cast<int>(misses * ISR_TIMEOUT_MS / 1000), misses);
        }
    }
}

void ADS1115rpi::stop()
{
    if (!running) return;
    running = false;
    if (thr.joinable())  thr.join();
    if (request)         request->release();
    if (chip)            chip->close();
    if (fd_i2c >= 0)   { close(fd_i2c); fd_i2c = -1; }
}

void ADS1115rpi::i2c_selectDevice()
{
    ioctl(fd_i2c, I2C_SLAVE, ads1115settings.address);
}

void ADS1115rpi::i2c_writeWord(uint8_t reg, unsigned data)
{
    std::lock_guard<std::mutex> lk(i2c1_mutex());
    i2c_selectDevice();
    uint8_t tmp[3];
    tmp[0] = reg;
    tmp[1] = static_cast<uint8_t>((data & 0xff00) >> 8);
    tmp[2] = static_cast<uint8_t>(data & 0x00ff);
    if (write(fd_i2c, tmp, 3) < 0)
        throw std::invalid_argument("ADS1115: could not write to i2c.");
}

unsigned ADS1115rpi::i2c_readWord(uint8_t reg)
{
    std::lock_guard<std::mutex> lk(i2c1_mutex());
    i2c_selectDevice();
    uint8_t tmp[2];
    tmp[0] = reg;
    write(fd_i2c, tmp, 1);
    if (read(fd_i2c, tmp, 2) < 0)
        throw std::invalid_argument("ADS1115: could not read from i2c.");
    return (static_cast<unsigned>(tmp[0]) << 8) | static_cast<unsigned>(tmp[1]);
}

int ADS1115rpi::i2c_readConversion()
{
    static constexpr int MAX_RETRIES = 5;

    // I2C_RDWR is an atomic kernel transaction — it carries the device address
    // inside msgs[].addr and is serialized by the kernel I2C bus driver.
    // No userspace mutex or I2C_SLAVE ioctl needed here.

    uint8_t reg    = 0;
    uint8_t buf[2] = {};

    struct i2c_msg msgs[2];
    msgs[0].addr  = ads1115settings.address;
    msgs[0].flags = 0;
    msgs[0].len   = 1;
    msgs[0].buf   = &reg;

    msgs[1].addr  = ads1115settings.address;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = 2;
    msgs[1].buf   = buf;

    struct i2c_rdwr_ioctl_data txn;
    txn.msgs  = msgs;
    txn.nmsgs = 2;

    for (int attempt = 0; attempt < MAX_RETRIES; ++attempt) {
        if (ioctl(fd_i2c, I2C_RDWR, &txn) == 2)
            return (static_cast<int>(buf[0]) << 8) | static_cast<int>(buf[1]);
    }

    return -1;  // caller skips silently
}