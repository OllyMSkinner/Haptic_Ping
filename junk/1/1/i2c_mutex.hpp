#pragma once
#include <mutex>

// Single mutex protecting the shared I2C-1 bus.
// Both IMUReader (ICM20948) and ADS1115rpi run on separate threads and
// both use /dev/i2c-1.  Without this lock the threads interleave their
// ioctl(I2C_SLAVE) + read/write pairs, addressing the wrong device and
// causing the ADC retry storm seen in the terminal output.
//
// Usage — take the lock for the entire logical transaction:
//   {
//       std::lock_guard<std::mutex> lk(i2c_bus_mutex);
//       ioctl(fd, I2C_SLAVE, addr);
//       read(fd, buf, len);
//   }

inline std::mutex& i2c1_mutex()
{
    static std::mutex m;
    return m;
}
