#pragma once

#include <atomic>
#include <functional>
#include <thread>

#include "Icm20948driver.hpp"
#include <gpiod.h>
#include <future>

class IMUReader
{
public:
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
    icm20948::ICM20948_I2C& getIMU();
private:
    void worker();

    icm20948::ICM20948_I2C imu_;
    SampleCallback         callback_;
    std::thread            worker_thread_;
    std::atomic_bool       running_;

    const char*         gpiochip_path_;
    unsigned            rdy_line_;
    gpiod_chip*         chip_;
    gpiod_line_request* request_;
    int                 gpio_fd_;
    int                 stop_fd_;
};
icm20948::ICM20948_I2C& IMUReader::getIMU()
{
    return imu_;
}