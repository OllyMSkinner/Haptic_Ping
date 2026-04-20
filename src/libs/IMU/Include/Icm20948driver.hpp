/** This header defines the register map, configuration types, and public driver
* interface for the ICM-20948 IMU and its onboard magnetometer.
*
* SOLID principles:
*   S - This file centralises device-specific constants, configuration structures,
*       and the public interface for the IMU driver in one place.
*   O - Configuration is represented through settings types, allowing the driver to
*       be reconfigured without changing client code that uses the interface.
*   I - Configuration is grouped into smaller settings types (accelerometer,
*       gyroscope, magnetometer) so callers interact with the parts of the
*       device configuration they actually need. */

#pragma once

#include <cstdint>
#include <string>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include "yaml-cpp/yaml.h"

/// Register addresses and constants for the ICM-20948 IMU and onboard magnetometer.
///  Register map

#define ICM20948_I2C_ADDR       0x69
#define ICM20948_MAGN_I2C_ADDR  0x0C

/// Bank-select register and encoded bank values.
/// These constants isolate device-specific magic numbers from the driver logic.
#define ICM20948_REG_BANK_SEL_ADDR          0x7F
#define ICM20948_REG_BANK_SEL_BANK0_VALUE   0x00
#define ICM20948_REG_BANK_SEL_BANK1_VALUE   0x10
#define ICM20948_REG_BANK_SEL_BANK2_VALUE   0x20
#define ICM20948_REG_BANK_SEL_BANK3_VALUE   0x30

/// Bank 0
/// Output and control registers used for identity, power, interrupts,
/// and reading decoded sensor data.
#define ICM20948_WHO_AM_I_ADDR              0x00
#define ICM20948_USER_CTRL_ADDR             0x03
#define ICM20948_PWR_MGMT_1_ADDR            0x06
#define ICM20948_INT_PIN_CFG_ADDR           0x0F
#define ICM20948_INT_ENABLE_1_ADDR          0x11
#define ICM20948_I2C_MST_STATUS_ADDR        0x17
#define ICM20948_ACCEL_XOUT_H_ADDR          0x2D
#define ICM20948_ACCEL_XOUT_L_ADDR          0x2E
#define ICM20948_ACCEL_YOUT_H_ADDR          0x2F
#define ICM20948_ACCEL_YOUT_L_ADDR          0x30
#define ICM20948_ACCEL_ZOUT_H_ADDR          0x31
#define ICM20948_ACCEL_ZOUT_L_ADDR          0x32
#define ICM20948_GYRO_XOUT_H_ADDR           0x33
#define ICM20948_GYRO_XOUT_L_ADDR           0x34
#define ICM20948_GYRO_YOUT_H_ADDR           0x35
#define ICM20948_GYRO_YOUT_L_ADDR           0x36
#define ICM20948_GYRO_ZOUT_H_ADDR           0x37
#define ICM20948_GYRO_ZOUT_L_ADDR           0x38
#define ICM20948_TEMP_OUT_H_ADDR            0x39
#define ICM20948_TEMP_OUT_L_ADDR            0x3A
#define ICM20948_EXT_SLV_SENS_DATA_00_ADDR  0x3B

/// Bank IDs for the corresponding registers above.
/// Keeping bank IDs alongside register definitions reduces the chance of
/// mismatched bank/register access in the implementation.
#define ICM20948_WHO_AM_I_BANK              0
#define ICM20948_USER_CTRL_BANK             0
#define ICM20948_PWR_MGMT_1_BANK            0
#define ICM20948_INT_PIN_CFG_BANK           0
#define ICM20948_INT_ENABLE_1_BANK          0
#define ICM20948_I2C_MST_STATUS_BANK        0
#define ICM20948_ACCEL_OUT_BANK             0
#define ICM20948_GYRO_OUT_BANK              0
#define ICM20948_EXT_SLV_SENS_DATA_00_BANK  0


/// Bank 2
/// Configuration registers for sample rate, scale, and digital filtering.
#define ICM20948_GYRO_SMPLRT_DIV_ADDR       0x00
#define ICM20948_GYRO_CONFIG_1_ADDR         0x01
#define ICM20948_ACCEL_SMPLRT_DIV_1_ADDR    0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2_ADDR    0x11
#define ICM20948_ACCEL_CONFIG_1_ADDR        0x14

/// Bank IDs for accel/gyro configuration registers.
#define ICM20948_GYRO_SMPLRT_DIV_BANK       2
#define ICM20948_GYRO_CONFIG_1_BANK         2
#define ICM20948_ACCEL_SMPLRT_DIV_1_BANK    2
#define ICM20948_ACCEL_SMPLRT_DIV_2_BANK    2
#define ICM20948_ACCEL_CONFIG_1_BANK        2

/// Bank 3
/// Registers for the internal I2C master used to communicate with the
/// onboard magnetometer through the main IMU device.
#define ICM20948_I2C_MST_CTRL_ADDR          0x01
#define ICM20948_I2C_SLV0_ADDR_ADDR         0x03
#define ICM20948_I2C_SLV0_REG_ADDR          0x04
#define ICM20948_I2C_SLV0_CTRL_ADDR         0x05
#define ICM20948_I2C_SLV4_ADDR_ADDR         0x13
#define ICM20948_I2C_SLV4_REG_ADDR          0x14
#define ICM20948_I2C_SLV4_CTRL_ADDR         0x15
#define ICM20948_I2C_SLV4_DO_ADDR           0x16
#define ICM20948_I2C_SLV4_DI_ADDR           0x17

/// Bank IDs for the internal I2C master interface.
#define ICM20948_I2C_MST_CTRL_BANK          3
#define ICM20948_I2C_SLV0_ADDR_BANK         3
#define ICM20948_I2C_SLV0_REG_BANK          3
#define ICM20948_I2C_SLV0_CTRL_BANK         3
#define ICM20948_I2C_SLV4_ADDR_BANK         3
#define ICM20948_I2C_SLV4_REG_BANK          3
#define ICM20948_I2C_SLV4_CTRL_BANK         3
#define ICM20948_I2C_SLV4_DO_BANK           3
#define ICM20948_I2C_SLV4_DI_BANK           3

/// AK09916 magnetometer control register and expected WHO_AM_I value.
#define AK09916_CNTL2_ADDR                  0x31
#define ICM20948_BANK0_WHO_AM_I_VALUE       0xEA


/// Interrupt status registers.
/// These support event-driven sampling by allowing higher-level code to
/// check whether new data is ready before attempting a read.
#define ICM20948_INT_STATUS_BANK            0
#define ICM20948_INT_STATUS_1_BANK          0

#define ICM20948_INT_STATUS_ADDR            0x19
#define ICM20948_INT_STATUS_1_ADDR          0x1A


///  Settings types

namespace icm20948
{
    /// Accelerometer full-scale range.
    /// Using an enum keeps configuration values readable and avoids scattering
    /// raw register meanings through application code.
    typedef enum {
        ACCEL_2G  = 0,
        ACCEL_4G,
        ACCEL_8G,
        ACCEL_16G
    } accel_scale;

    /// Accelerometer digital low-pass filter config.
    typedef enum {
        ACCEL_DLPF_246HZ   = 0,
        ACCEL_DLPF_246HZ_2,
        ACCEL_DLPF_111_4HZ,
        ACCEL_DLPF_50_4HZ,
        ACCEL_DLPF_23_9HZ,
        ACCEL_DLPF_11_5HZ,
        ACCEL_DLPF_5_7HZ,
        ACCEL_DLPF_473HZ
    } accel_dlpf_config;

    /// Accelerometer runtime configuration.
    /// Grouping these parameters in one struct improves clarity and keeps the
    /// public interface cleaner than passing many separate constructor arguments.
    typedef struct accel_settings {
        uint16_t          sample_rate_div;
        accel_scale       scale;
        bool              dlpf_enable;
        accel_dlpf_config dlpf_config;

        accel_settings(uint16_t          sample_rate_div = 0,
                       accel_scale       scale           = ACCEL_2G,
                       bool              dlpf_enable     = true,
                       accel_dlpf_config dlpf_config     = ACCEL_DLPF_246HZ)
            : sample_rate_div(sample_rate_div), scale(scale),
              dlpf_enable(dlpf_enable), dlpf_config(dlpf_config) {}
    } accel_settings;

    /// Gyroscope full-scale range.
    typedef enum {
        GYRO_250DPS  = 0,
        GYRO_500DPS,
        GYRO_1000DPS,
        GYRO_2000DPS
    } gyro_scale;

    /// Gyroscope digital low-pass filter config.
    typedef enum {
        GYRO_DLPF_196_6HZ = 0,
        GYRO_DLPF_151_8HZ,
        GYRO_DLPF_119_5HZ,
        GYRO_DLPF_51_2HZ,
        GYRO_DLPF_23_9HZ,
        GYRO_DLPF_11_6HZ,
        GYRO_DLPF_5_7HZ,
        GYRO_DLPF_361_4HZ
    } gyro_dlpf_config;

    /// Gyroscope runtime configuration.
    typedef struct gyro_settings {
        uint8_t          sample_rate_div;
        gyro_scale       scale;
        bool             dlpf_enable;
        gyro_dlpf_config dlpf_config;

        gyro_settings(uint8_t          sample_rate_div = 0,
                      gyro_scale       scale           = GYRO_250DPS,
                      bool             dlpf_enable     = true,
                      gyro_dlpf_config dlpf_config     = GYRO_DLPF_196_6HZ)
            : sample_rate_div(sample_rate_div), scale(scale),
              dlpf_enable(dlpf_enable), dlpf_config(dlpf_config) {}
    } gyro_settings;

    /// Magnetometer operating mode.
    typedef enum {
        MAGN_SHUTDOWN  = 0,
        MAGN_SINGLE    = 1,
        MAGN_10HZ      = 2,
        MAGN_20HZ      = 4,
        MAGN_50HZ      = 6,
        MAGN_100HZ     = 8,
        MAGN_SELF_TEST = 16
    } magn_mode;

    /// Magnetometer runtime configuration.
    typedef struct magn_settings {
        magn_mode mode;
        explicit magn_settings(magn_mode mode = MAGN_100HZ) : mode(mode) {}
    } magn_settings;

    /// Aggregate device configuration.
    /// This gives client code a single configuration object, which improves
    /// readability and makes the driver easier to construct and reconfigure.
    typedef struct settings {
        accel_settings accel;
        gyro_settings  gyro;
        magn_settings  magn;

        settings(accel_settings accel = accel_settings(),
                 gyro_settings  gyro  = gyro_settings(),
                 magn_settings  magn  = magn_settings())
            : accel(accel), gyro(gyro), magn(magn) {}

        /// Construct settings from a YAML config node.
        /// This supports configuration-driven setup without hard-coding all
        /// parameters in application code.
        explicit settings(YAML::Node config_file_node);
    } settings;

    /// Helpers for converting config enums to scale factors / readable strings.
    /// These utility functions separate presentation and conversion logic from
    /// the main driver code, improving maintainability.
    float       accel_scale_factor(accel_scale scale);
    std::string accel_scale_to_str(accel_scale scale);
    std::string accel_dlpf_config_to_str(accel_dlpf_config config);

    float       gyro_scale_factor(gyro_scale scale);
    std::string gyro_scale_to_str(gyro_scale scale);
    std::string gyro_dlpf_config_to_str(gyro_dlpf_config config);

    std::string magn_mode_to_str(magn_mode mode);

    /// One fully decoded IMU sample.
    /// This struct provides a clean data-transfer object so callers can work
    /// with physical units rather than raw register bytes.
    struct IMUSample {
        float ax, ay, az;
        float gx, gy, gz;
        float mx, my, mz;
    };

    /** ICM20948_I2C encapsulates the device driver for the IMU over Linux I2C.
    *
    * SOLID principles:
    *   S - This class focuses on IMU communication, configuration, and conversion
    *       of raw register data into decoded sensor values.
    *   O - Client code interacts through configuration objects and high-level
    *       methods, allowing internal implementation details to evolve without
    *       forcing changes on users of the class. */


    class ICM20948_I2C
    {
    public:
        /// Most recent decoded sensor values.
        /// These cached arrays provide direct access to the latest successful
        /// reading while keeping raw register handling internal to the class.
        float accel[3];
        float gyro[3];
        float magn[3];

        /// Active driver settings.
        /// Storing settings here keeps configuration tied to the device instance.
        icm20948::settings settings;

        ICM20948_I2C(unsigned           i2c_bus,
                     unsigned           i2c_address = ICM20948_I2C_ADDR,
                     icm20948::settings settings    = icm20948::settings());
        ~ICM20948_I2C();

        /// Public API for device lifecycle and sensor access.
        /// Higher-level code interacts through these methods rather than
        /// directly manipulating device registers.
        bool init();
        bool reset();
        bool wake();
        bool set_settings();
        bool read_accel_gyro();
        bool read_magn();
        bool read_sample(IMUSample& sample);
        bool check_DRDY_INT();

    private:
        /// Linux I2C file descriptor and device identity.
        /// These implementation details are kept private so the transport layer
        /// remains hidden from the rest of the application.
        int       _i2c_fd;
        unsigned  _i2c_bus;
        unsigned  _i2c_address;

        /// Cached register bank to avoid redundant bank switches.
        /// This is a performance and consistency optimisation internal to the driver.
        uint8_t   _current_bank;

        /// Cached conversion factors from raw counts to physical units.
        /// Keeping these values private prevents duplication of conversion logic.
        float     _accel_scale_factor;
        float     _gyro_scale_factor;
        float     _magn_scale_factor;

        /// Device setup helpers.
        /// These methods isolate configuration subtasks, which keeps the public
        /// interface small and the implementation more maintainable.
        bool _enable_data_ready_interrupt();
        bool _set_bank(uint8_t bank);
        bool _set_accel_sample_rate_div();
        bool _set_accel_range_dlpf();
        bool _set_gyro_sample_rate_div();
        bool _set_gyro_range_dlpf();

        /// Magnetometer setup via the ICM-20948 internal I2C master.
        /// This complexity is hidden from callers so they can treat the driver
        /// as a single IMU abstraction.
        bool _magnetometer_init();
        bool _magnetometer_enable();
        bool _magnetometer_set_mode();
        bool _magnetometer_configured();
        bool _magnetometer_set_readout();
        bool _chip_i2c_master_reset();

        /// Low-level register access helpers.
        /// These centralise raw byte/bit operations so direct register handling
        /// is not scattered across the driver implementation.
        bool _write_byte(uint8_t bank, uint8_t reg, uint8_t byte);
        bool _read_byte(uint8_t bank, uint8_t reg, uint8_t& byte);
        bool _write_bit(uint8_t bank, uint8_t reg, uint8_t bit_pos, bool bit);
        bool _read_bit(uint8_t bank, uint8_t reg, uint8_t bit_pos, bool& bit);
        bool _read_block_bytes(uint8_t bank, uint8_t start_reg, uint8_t* bytes, int length);
        bool _write_mag_byte(uint8_t mag_reg, uint8_t byte);
        bool _read_mag_byte(uint8_t mag_reg, uint8_t& byte);


    };

} 
