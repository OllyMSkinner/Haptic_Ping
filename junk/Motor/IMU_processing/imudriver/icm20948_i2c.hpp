#ifndef ICM20948_I2C_HPP
#define ICM20948_I2C_HPP

#define ICM20948_INT_ENABLE_1_BANK          0
#define ICM20948_INT_ENABLE_1_ADDR          0x11

#define ICM20948_INT_STATUS_1_BANK          0
#define ICM20948_INT_STATUS_1_ADDR          0x1A

#include <cstdint>

#include "mraa/common.hpp"
#include "mraa/i2c.hpp"

#include "../imu_settings/icm20948_defs.hpp"
#include "../imu_settings/icm20948_utils.hpp"

namespace icm20948
{
    struct IMUSample
    {
        float ax;
        float ay;
        float az;

        float gx;
        float gy;
        float gz;

        float mx;
        float my;
        float mz;
    };

    class ICM20948_I2C
    {
    private:
        mraa::I2c _i2c;
        unsigned _i2c_bus, _i2c_address;
        uint8_t _current_bank;
        float _accel_scale_factor, _gyro_scale_factor, _magn_scale_factor;

        bool _write_byte(const uint8_t bank, const uint8_t reg, const uint8_t byte);
        bool _read_byte(const uint8_t bank, const uint8_t reg, uint8_t &byte);
        bool _write_bit(const uint8_t bank, const uint8_t reg, const uint8_t bit_pos, const bool bit);
        bool _read_bit(const uint8_t bank, const uint8_t reg, const uint8_t bit_pos, bool &bit);
        bool _read_block_bytes(const uint8_t bank, const uint8_t start_reg, uint8_t *bytes, const int length);
        bool _write_mag_byte(const uint8_t mag_reg, const uint8_t byte);
        bool _read_mag_byte(const uint8_t mag_reg, uint8_t &byte);
        bool _enable_data_ready_interrupt();

        bool _set_bank(uint8_t bank);
        bool _set_accel_sample_rate_div();
        bool _set_accel_range_dlpf();
        bool _set_gyro_sample_rate_div();
        bool _set_gyro_range_dlpf();

        bool _magnetometer_init();
        bool _magnetometer_enable();
        bool _magnetometer_set_mode();
        bool _magnetometer_configured();
        bool _magnetometer_set_readout();

        bool _chip_i2c_master_reset();

    public:
        // Contains linear acceleration in m/s^2
        float accel[3];
        // Contains angular velocities in rad/s
        float gyro[3];
        // Contains magnetic field strength in uTesla
        float magn[3];

        // Sensor settings
        icm20948::settings settings;

        // Constructor
        ICM20948_I2C(
            unsigned i2c_bus,
            unsigned i2c_address = ICM20948_I2C_ADDR,
            icm20948::settings settings = icm20948::settings()
        );

        bool init();
        bool reset();
        bool wake();
        bool set_settings();
        bool read_accel_gyro();
        bool read_magn();

        // Read all currently needed values and package them into one sample
        bool read_sample(IMUSample& sample);
    };
}

#endif