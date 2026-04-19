#pragma once
#include <mutex>

// Shared mutex for the I2C-1 bus.
// IMUReader (ICM20948) and ADS1115rpi run on separate threads and both
// access /dev/i2c-1. Without this lock, their ioctl(I2C_SLAVE) and
// read/write sequences can interleave, causing transactions to target
// the wrong device and triggering the ADC retry failures seen in logs.

inline std::mutex& i2c1_mutex()
{
    static std::mutex m;
    return m;
}
