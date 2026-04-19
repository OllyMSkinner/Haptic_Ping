#pragma once

#include <atomic>
#include <functional>
#include <thread>

#include "Icm20948driver.hpp"
#include <gpiod.h>
#include <future>

// Threaded IMU reader driven by the data-ready GPIO.

class IMUReader
{
public:
    // Called for each decoded IMU sample.
    using SampleCallback = std::function<void(float ax, float ay, float az,
                                              float gx, float gy, float gz,
                                              float mx, float my)>;

    IMUReader(unsigned           i2c_bus,
              unsigned           i2c_address = ICM20948_I2C_ADDR,
              const char*        gpiochip    = "/dev/gpiochip0",
              unsigned           rdy_line    = 27,
              icm20948::settings settings    = icm20948::settings());

    ~IMUReader();

    bool init();
    void setCallback(SampleCallback cb);
    bool start();
    void stop();

private:
    // Worker loop waiting on DRDY and dispatching samples.
    void worker();

    // Low-level IMU driver instance.
    icm20948::ICM20948_I2C imu_;

    // Client callback for new samples.
    SampleCallback         callback_;

    // Background reader thread.
    std::thread            worker_thread_;

    // Run flag shared with the worker thread.
    std::atomic_bool       running_;

    // GPIO chip path and DRDY line number.
    const char*         gpiochip_path_;
    unsigned            rdy_line_;

    // libgpiod handles for the DRDY line.
    gpiod_chip*         chip_;
    gpiod_line_request* request_;

    // FDs used by the event loop.
    int                 gpio_fd_;
    int                 stop_fd_;
};
