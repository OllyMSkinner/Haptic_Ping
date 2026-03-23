#ifndef IMU_PUBLISHER_HPP
#define IMU_PUBLISHER_HPP

#include <atomic>
#include <functional>
#include <thread>

#include "../imudriver/icm20948_i2c.hpp"

namespace icm20948
{
    class IMUPublisher
    {
    public:
        using SampleCallback = std::function<void(const IMUSample&)>;

        IMUPublisher(unsigned i2c_bus,
                     unsigned i2c_address = ICM20948_I2C_ADDR,
                     const char* gpiochip = "/dev/gpiochip0",
                     unsigned rdy_line = 17);

        ~IMUPublisher();

        bool init();
        void registerCallback(SampleCallback cb);
        bool start();
        void stop();

    private:
        void worker();

        ICM20948_I2C imu_;
        SampleCallback callback_;
        std::thread worker_thread_;
        std::atomic_bool running_;

        const char* gpiochip_path_;
        unsigned rdy_line_;
    };
}

#endif