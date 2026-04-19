// Shared mutex protecting access to the /dev/i2c-1 bus.
//
// SOLID principles:
//   S - This helper provides one shared synchronisation point for I2C bus access.
//   D - Centralising the bus lock in one place avoids duplicating global mutex
//       definitions across multiple modules and makes cross-thread I2C ownership
//       easier to reason about.

#pragma once
#include <mutex>

// Shared mutex for the I2C-1 bus.
// IMUReader (ICM20948) and ADS1115rpi run on separate threads and both
// access /dev/i2c-1. Without this lock, their ioctl(I2C_SLAVE) and
// read/write sequences can interleave, causing transactions to target
// the wrong device and triggering the ADC retry failures seen in logs.

inline std::mutex& i2c1_mutex()
{
    // Function-local static storage gives one shared mutex instance for the
    // whole process while keeping the global namespace clean.
    static std::mutex m;
    return m;
}
