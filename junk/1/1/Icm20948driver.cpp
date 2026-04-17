#include "Icm20948driver.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cerrno>
#include <cstdio>
#include <cstring>

#define G2MSQR            9.80665f
#define DEG2RAD           0.017453293f
#define MAGN_SCALE_FACTOR 0.149975574f

namespace icm20948
{
    float accel_scale_factor(accel_scale scale)
    {
        switch (scale) {
            case ACCEL_2G:  return 1 / 16384.0f;
            case ACCEL_4G:  return 1 / 8192.0f;
            case ACCEL_8G:  return 1 / 4096.0f;
            case ACCEL_16G: return 1 / 2048.0f;
            default:
                throw std::invalid_argument(
                    "Invalid argument in accel_scale_factor(): " + std::to_string(scale));
        }
    }

    std::string accel_scale_to_str(accel_scale scale)
    {
        switch (scale) {
            case ACCEL_2G:  return "2G";
            case ACCEL_4G:  return "4G";
            case ACCEL_8G:  return "8G";
            case ACCEL_16G: return "16G";
            default:        return "<invalid accelerometer scale>";
        }
    }

    std::string accel_dlpf_config_to_str(accel_dlpf_config config)
    {
        switch (config) {
            case ACCEL_DLPF_246HZ:   return "246Hz";
            case ACCEL_DLPF_246HZ_2: return "246Hz";
            case ACCEL_DLPF_111_4HZ: return "111.4Hz";
            case ACCEL_DLPF_50_4HZ:  return "50.4Hz";
            case ACCEL_DLPF_23_9HZ:  return "23.9Hz";
            case ACCEL_DLPF_11_5HZ:  return "11.5Hz";
            case ACCEL_DLPF_5_7HZ:   return "5.7Hz";
            case ACCEL_DLPF_473HZ:   return "473Hz";
            default:                 return "<invalid accelerometer DLPF config>";
        }
    }

    float gyro_scale_factor(gyro_scale scale)
    {
        switch (scale) {
            case GYRO_250DPS:  return 1 / 131.0f;
            case GYRO_500DPS:  return 1 / 65.5f;
            case GYRO_1000DPS: return 1 / 32.8f;
            case GYRO_2000DPS: return 1 / 16.4f;
            default:
                throw std::invalid_argument(
                    "Invalid argument in gyro_scale_factor(): " + std::to_string(scale));
        }
    }

    std::string gyro_scale_to_str(gyro_scale scale)
    {
        switch (scale) {
            case GYRO_250DPS:  return "250DPS";
            case GYRO_500DPS:  return "500DPS";
            case GYRO_1000DPS: return "1000DPS";
            case GYRO_2000DPS: return "2000DPS";
            default:           return "<invalid gyroscope scale>";
        }
    }

    std::string gyro_dlpf_config_to_str(gyro_dlpf_config config)
    {
        switch (config) {
            case GYRO_DLPF_196_6HZ: return "196.6Hz";
            case GYRO_DLPF_151_8HZ: return "151.8Hz";
            case GYRO_DLPF_119_5HZ: return "119.5Hz";
            case GYRO_DLPF_51_2HZ:  return "51.2Hz";
            case GYRO_DLPF_23_9HZ:  return "23.9Hz";
            case GYRO_DLPF_11_6HZ:  return "11.6Hz";
            case GYRO_DLPF_5_7HZ:   return "5.7Hz";
            case GYRO_DLPF_361_4HZ: return "361.4Hz";
            default:                return "<invalid gyroscope DLPF config>";
        }
    }

    std::string magn_mode_to_str(magn_mode mode)
    {
        switch (mode) {
            case MAGN_SHUTDOWN:  return "Shutdown";
            case MAGN_SINGLE:    return "Single";
            case MAGN_10HZ:      return "10Hz";
            case MAGN_20HZ:      return "20Hz";
            case MAGN_50HZ:      return "50Hz";
            case MAGN_100HZ:     return "100Hz";
            case MAGN_SELF_TEST: return "Self-test";
            default:             return "<invalid magnetometer mode>";
        }
    }

    settings::settings(YAML::Node node)
    {
        for (auto it = node.begin(); it != node.end(); ++it) {
            const std::string key = it->first.as<std::string>();

            if (key == "accelerometer") {
                for (auto a = it->second.begin(); a != it->second.end(); ++a) {
                    const std::string ak = a->first.as<std::string>();
                    if (ak == "sample_rate_div")
                        accel.sample_rate_div = static_cast<uint16_t>(a->second.as<unsigned>());
                    else if (ak == "scale")
                        accel.scale = static_cast<accel_scale>(a->second.as<unsigned>());
                    else if (ak == "dlpf") {
                        for (auto d = a->second.begin(); d != a->second.end(); ++d) {
                            const std::string dk = d->first.as<std::string>();
                            if (dk == "enable")
                                accel.dlpf_enable = static_cast<bool>(d->second.as<int>());
                            else if (dk == "cutoff")
                                accel.dlpf_config = static_cast<accel_dlpf_config>(d->second.as<unsigned>());
                        }
                    }
                }
            } else if (key == "gyroscope") {
                for (auto g = it->second.begin(); g != it->second.end(); ++g) {
                    const std::string gk = g->first.as<std::string>();
                    if (gk == "sample_rate_div")
                        gyro.sample_rate_div = static_cast<uint8_t>(g->second.as<unsigned>());
                    else if (gk == "scale")
                        gyro.scale = static_cast<gyro_scale>(g->second.as<unsigned>());
                    else if (gk == "dlpf") {
                        for (auto d = g->second.begin(); d != g->second.end(); ++d) {
                            const std::string dk = d->first.as<std::string>();
                            if (dk == "enable")
                                gyro.dlpf_enable = static_cast<bool>(d->second.as<int>());
                            else if (dk == "cutoff")
                                gyro.dlpf_config = static_cast<gyro_dlpf_config>(d->second.as<unsigned>());
                        }
                    }
                }
            } else if (key == "magnetometer") {
                for (auto m = it->second.begin(); m != it->second.end(); ++m) {
                    if (m->first.as<std::string>() == "mode") {
                        switch (m->second.as<unsigned>()) {
                            case 0:  magn.mode = MAGN_SHUTDOWN;  break;
                            case 1:  magn.mode = MAGN_SINGLE;    break;
                            case 2:  magn.mode = MAGN_10HZ;      break;
                            case 3:  magn.mode = MAGN_20HZ;      break;
                            case 4:  magn.mode = MAGN_50HZ;      break;
                            case 5:  magn.mode = MAGN_100HZ;     break;
                            case 6:  magn.mode = MAGN_SELF_TEST; break;
                            default:
                                throw std::runtime_error(
                                    "Invalid mode chosen for magnetometer in YAML config");
                        }
                    }
                }
            }
        }
    }

    ICM20948_I2C::ICM20948_I2C(unsigned i2c_bus,
                               unsigned i2c_address,
                               icm20948::settings settings)
        : _i2c_fd(-1),
          _i2c_bus(i2c_bus),
          _i2c_address(i2c_address),
          _current_bank(0),
          _accel_scale_factor(0.0f),
          _gyro_scale_factor(0.0f),
          _magn_scale_factor(MAGN_SCALE_FACTOR),
          settings(settings)
    {
        char bus_path[32];
        std::snprintf(bus_path, sizeof(bus_path), "/dev/i2c-%u", i2c_bus);

        _i2c_fd = ::open(bus_path, O_RDWR);
        if (_i2c_fd < 0) {
            throw std::runtime_error(std::string("ICM20948: cannot open ") + bus_path +
                                     ": " + std::strerror(errno));
        }

        ::ioctl(_i2c_fd, I2C_TIMEOUT, 20);

        if (::ioctl(_i2c_fd, I2C_SLAVE, i2c_address) < 0) {
            ::close(_i2c_fd);
            _i2c_fd = -1;
            throw std::runtime_error(std::string("ICM20948: I2C_SLAVE failed: ") +
                                     std::strerror(errno));
        }

        accel[0] = accel[1] = accel[2] = 0.0f;
        gyro[0]  = gyro[1]  = gyro[2]  = 0.0f;
        magn[0]  = magn[1]  = magn[2]  = 0.0f;
    }

bool ICM20948_I2C::init()
{
    uint8_t device_id = 0;
    bool success = true;

    success &= _set_bank(0);
    success &= _read_byte(ICM20948_WHO_AM_I_BANK, ICM20948_WHO_AM_I_ADDR, device_id);
    success &= (device_id == ICM20948_BANK0_WHO_AM_I_VALUE);
    success &= reset();
    success &= wake();
    success &= set_settings();
    success &= _enable_data_ready_interrupt();

    bool magn_ok = false;
    for (int i = 0; i < 3; ++i) {
        magn_ok = _magnetometer_init();
        if (magn_ok) break;
    }
    if (!magn_ok) {
        std::cerr << "[IMU] Magnetometer init failed, continuing without it\n";
        // Disable I2C master so the failed mag init does not corrupt
        // subsequent accel/gyro reads via the main I2C bus
        _write_bit(ICM20948_USER_CTRL_BANK,
                   ICM20948_USER_CTRL_ADDR, 5, false);
    }

    return success;
}

    bool ICM20948_I2C::reset()
    {
        bool success = _write_bit(ICM20948_PWR_MGMT_1_BANK,
                                  ICM20948_PWR_MGMT_1_ADDR, 7, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        if (success) {
            bool still_resetting = true;
            while (still_resetting && success) {
                std::this_thread::sleep_for(std::chrono::milliseconds(25));
                success &= _read_bit(ICM20948_PWR_MGMT_1_BANK,
                                     ICM20948_PWR_MGMT_1_ADDR, 7, still_resetting);
            }
        }

        if (success) _current_bank = 0;
        return success;
    }

    bool ICM20948_I2C::wake()
    {
        bool success = _write_bit(ICM20948_PWR_MGMT_1_BANK,
                                  ICM20948_PWR_MGMT_1_ADDR, 6, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        return success;
    }

    bool ICM20948_I2C::set_settings()
    {
        bool success = true;
        success &= _set_accel_sample_rate_div();
        success &= _set_accel_range_dlpf();
        success &= _set_gyro_sample_rate_div();
        success &= _set_gyro_range_dlpf();
        return success;
    }

    bool ICM20948_I2C::_enable_data_ready_interrupt()
    {
        return _write_byte(ICM20948_INT_ENABLE_1_BANK,
                           ICM20948_INT_ENABLE_1_ADDR, 0x01);
    }

    bool ICM20948_I2C::read_accel_gyro()
    {
        uint8_t buf[12] = {};
        if (!_read_block_bytes(ICM20948_ACCEL_OUT_BANK,
                               ICM20948_ACCEL_XOUT_H_ADDR, buf, 12)) {
            return false;
        }

        const int16_t accel_x_raw = static_cast<int16_t>((buf[0] << 8) | buf[1]);
        const int16_t accel_y_raw = static_cast<int16_t>((buf[2] << 8) | buf[3]);
        const int16_t accel_z_raw = static_cast<int16_t>((buf[4] << 8) | buf[5]);
        const int16_t gyro_x_raw  = static_cast<int16_t>((buf[6] << 8) | buf[7]);
        const int16_t gyro_y_raw  = static_cast<int16_t>((buf[8] << 8) | buf[9]);
        const int16_t gyro_z_raw  = static_cast<int16_t>((buf[10] << 8) | buf[11]);

        accel[0] = static_cast<float>(accel_x_raw) * _accel_scale_factor * G2MSQR;
        accel[1] = static_cast<float>(accel_y_raw) * _accel_scale_factor * G2MSQR;
        accel[2] = static_cast<float>(accel_z_raw) * _accel_scale_factor * G2MSQR;

        gyro[0] = static_cast<float>(gyro_x_raw) * _gyro_scale_factor * DEG2RAD;
        gyro[1] = static_cast<float>(gyro_y_raw) * _gyro_scale_factor * DEG2RAD;
        gyro[2] = static_cast<float>(gyro_z_raw) * _gyro_scale_factor * DEG2RAD;

        return true;
    }

    bool ICM20948_I2C::read_magn()
    {
        uint8_t buf[6] = {};
        if (!_read_block_bytes(ICM20948_EXT_SLV_SENS_DATA_00_BANK,
                               ICM20948_EXT_SLV_SENS_DATA_00_ADDR, buf, 6)) {
            return false;
        }

        const int16_t mag_x_raw = static_cast<int16_t>((buf[1] << 8) | buf[0]);
        const int16_t mag_y_raw = static_cast<int16_t>((buf[3] << 8) | buf[2]);
        const int16_t mag_z_raw = static_cast<int16_t>((buf[5] << 8) | buf[4]);

        magn[0] = static_cast<float>(mag_x_raw) * _magn_scale_factor;
        magn[1] = static_cast<float>(mag_y_raw) * _magn_scale_factor;
        magn[2] = static_cast<float>(mag_z_raw) * _magn_scale_factor;

        return true;
    }

    bool ICM20948_I2C::read_sample(IMUSample& sample)
    {
        if (!read_accel_gyro()) return false;

        if (!read_magn()) magn[0] = magn[1] = magn[2] = 0.0f;

        sample.ax = accel[0]; sample.ay = accel[1]; sample.az = accel[2];
        sample.gx = gyro[0];  sample.gy = gyro[1];  sample.gz = gyro[2];
        sample.mx = magn[0];  sample.my = magn[1];  sample.mz = magn[2];

        return true;
    }

    bool ICM20948_I2C::_set_bank(uint8_t bank)
    {
        if (_current_bank == bank) return true;

        uint8_t val = 0;
        switch (bank) {
            case 0: val = ICM20948_REG_BANK_SEL_BANK0_VALUE; break;
            case 1: val = ICM20948_REG_BANK_SEL_BANK1_VALUE; break;
            case 2: val = ICM20948_REG_BANK_SEL_BANK2_VALUE; break;
            case 3: val = ICM20948_REG_BANK_SEL_BANK3_VALUE; break;
            default:
                throw std::runtime_error(
                    "Invalid bank number in _set_bank(): " + std::to_string(bank));
        }

        uint8_t buf[2] = { ICM20948_REG_BANK_SEL_ADDR, val };
        if (::write(_i2c_fd, buf, 2) == 2) {
            _current_bank = bank;
            return true;
        }
        return false;
    }

    bool ICM20948_I2C::_set_accel_sample_rate_div()
    {
        uint8_t lsb = settings.accel.sample_rate_div & 0xff;
        uint8_t msb = (settings.accel.sample_rate_div >> 8) & 0x0f;
        bool ok = true;
        ok &= _write_byte(ICM20948_ACCEL_SMPLRT_DIV_1_BANK,
                          ICM20948_ACCEL_SMPLRT_DIV_1_ADDR, msb);
        ok &= _write_byte(ICM20948_ACCEL_SMPLRT_DIV_2_BANK,
                          ICM20948_ACCEL_SMPLRT_DIV_2_ADDR, lsb);
        return ok;
    }

    bool ICM20948_I2C::_set_accel_range_dlpf()
    {
        uint8_t byte = 0;
        byte |= static_cast<uint8_t>(!!static_cast<uint8_t>(settings.accel.dlpf_enable));
        byte |= static_cast<uint8_t>(static_cast<uint8_t>(settings.accel.scale) << 1);
        byte |= static_cast<uint8_t>(static_cast<uint8_t>(settings.accel.dlpf_config) << 3);

        bool ok = _write_byte(ICM20948_ACCEL_CONFIG_1_BANK,
                              ICM20948_ACCEL_CONFIG_1_ADDR, byte);
        if (ok) _accel_scale_factor = accel_scale_factor(settings.accel.scale);
        return ok;
    }

    bool ICM20948_I2C::_set_gyro_sample_rate_div()
    {
        return _write_byte(ICM20948_GYRO_SMPLRT_DIV_BANK,
                           ICM20948_GYRO_SMPLRT_DIV_ADDR,
                           settings.gyro.sample_rate_div);
    }

    bool ICM20948_I2C::_set_gyro_range_dlpf()
    {
        uint8_t byte = 0;
        byte |= static_cast<uint8_t>(!!static_cast<uint8_t>(settings.gyro.dlpf_enable));
        byte |= static_cast<uint8_t>(static_cast<uint8_t>(settings.gyro.scale) << 1);
        byte |= static_cast<uint8_t>(static_cast<uint8_t>(settings.gyro.dlpf_config) << 3);

        bool ok = _write_byte(ICM20948_GYRO_CONFIG_1_BANK,
                              ICM20948_GYRO_CONFIG_1_ADDR, byte);
        if (ok) _gyro_scale_factor = gyro_scale_factor(settings.gyro.scale);
        return ok;
    }

    bool ICM20948_I2C::_magnetometer_init()
    {
        bool ok = true;
        ok &= _magnetometer_enable();
        if (!ok) { std::cerr << "Failed on _magnetometer_enable()\n"; return false; }
        ok &= _magnetometer_set_mode();
        if (!ok) { std::cerr << "Failed on _magnetometer_set_mode()\n"; return false; }
        ok &= _magnetometer_configured();
        if (!ok) { std::cerr << "Failed on _magnetometer_configured()\n"; return false; }
        ok &= _magnetometer_set_readout();
        if (!ok) { std::cerr << "Failed on _magnetometer_set_readout()\n"; return false; }
        return ok;
    }

    bool ICM20948_I2C::_magnetometer_enable()
    {
        bool ok = _write_bit(ICM20948_INT_PIN_CFG_BANK,
                             ICM20948_INT_PIN_CFG_ADDR, 1, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        ok &= _write_byte(ICM20948_I2C_MST_CTRL_BANK,
                          ICM20948_I2C_MST_CTRL_ADDR, 0x4D);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        ok &= _write_bit(ICM20948_USER_CTRL_BANK,
                         ICM20948_USER_CTRL_ADDR, 5, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        return ok;
    }

    bool ICM20948_I2C::_magnetometer_set_mode()
    {
        bool ok = _write_mag_byte(AK09916_CNTL2_ADDR,
                                  static_cast<uint8_t>(MAGN_SHUTDOWN));
        ok &= _write_mag_byte(AK09916_CNTL2_ADDR,
                              static_cast<uint8_t>(settings.magn.mode));
        return ok;
    }

    bool ICM20948_I2C::_magnetometer_configured()
    {
        uint8_t mag_id = 0;
        for (int i = 0; i < 5; ++i) {
            if (_read_mag_byte(0x01, mag_id)) return true;
            _chip_i2c_master_reset();
            std::cerr << "Magnetometer not configured properly, resetting chip I2C master\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        return false;
    }

    bool ICM20948_I2C::_magnetometer_set_readout()
    {
        bool ok = _write_byte(ICM20948_I2C_SLV0_ADDR_BANK,
                              ICM20948_I2C_SLV0_ADDR_ADDR, 0x8C);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        ok &= _write_byte(ICM20948_I2C_SLV0_REG_BANK,
                          ICM20948_I2C_SLV0_REG_ADDR, 0x11);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        ok &= _write_byte(ICM20948_I2C_SLV0_CTRL_BANK,
                          ICM20948_I2C_SLV0_CTRL_ADDR, 0x89);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        return ok;
    }

    bool ICM20948_I2C::_chip_i2c_master_reset()
    {
        return _write_bit(ICM20948_USER_CTRL_BANK,
                          ICM20948_USER_CTRL_ADDR, 1, true);
    }

    bool ICM20948_I2C::_write_byte(uint8_t bank, uint8_t reg, uint8_t byte)
    {
        bool ok = _set_bank(bank);
        uint8_t buf[2] = { reg, byte };
        ok &= (::write(_i2c_fd, buf, 2) == 2);
        return ok;
    }

    bool ICM20948_I2C::_read_byte(uint8_t bank, uint8_t reg, uint8_t& byte)
    {
        bool ok = _set_bank(bank);
        if (!ok) return false;

        uint8_t reg_buf = reg;
        if (::write(_i2c_fd, &reg_buf, 1) != 1) return false;
        if (::read(_i2c_fd, &byte, 1) != 1) return false;
        return true;
    }

    bool ICM20948_I2C::_write_bit(uint8_t bank, uint8_t reg,
                                  uint8_t bit_pos, bool bit)
    {
        uint8_t existing = 0;
        if (!_read_byte(bank, reg, existing)) return false;

        uint8_t updated =
            static_cast<uint8_t>(
                (existing & ~static_cast<uint8_t>(1U << bit_pos)) |
                (static_cast<uint8_t>(!!bit) << bit_pos));

        return _write_byte(bank, reg, updated);
    }

    bool ICM20948_I2C::_read_bit(uint8_t bank, uint8_t reg,
                                 uint8_t bit_pos, bool& bit)
    {
        uint8_t existing = 0;
        if (!_read_byte(bank, reg, existing)) return false;
        bit = ((existing >> bit_pos) & 1U) != 0U;
        return true;
    }

    bool ICM20948_I2C::_read_block_bytes(uint8_t bank, uint8_t start_reg,
                                         uint8_t* bytes, int length)
    {
        if (!_set_bank(bank)) return false;
        uint8_t reg_buf = start_reg;
        if (::write(_i2c_fd, &reg_buf, 1) != 1) return false;
        return ::read(_i2c_fd, bytes, length) == length;
    }

    bool ICM20948_I2C::_write_mag_byte(uint8_t mag_reg, uint8_t byte)
    {
        bool ok = true;
        ok &= _write_byte(ICM20948_I2C_SLV4_ADDR_BANK,
                          ICM20948_I2C_SLV4_ADDR_ADDR, ICM20948_MAGN_I2C_ADDR);
        ok &= _write_byte(ICM20948_I2C_SLV4_REG_BANK,
                          ICM20948_I2C_SLV4_REG_ADDR, mag_reg);
        ok &= _write_byte(ICM20948_I2C_SLV4_DO_BANK,
                          ICM20948_I2C_SLV4_DO_ADDR, byte);
        ok &= _write_byte(ICM20948_I2C_SLV4_CTRL_BANK,
                          ICM20948_I2C_SLV4_CTRL_ADDR, 0x80);

        bool done = false;
        for (int i = 0; i < 20; ++i) {
            _read_bit(ICM20948_I2C_MST_STATUS_BANK,
                      ICM20948_I2C_MST_STATUS_ADDR, 6, done);
            if (done) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        if (!done) std::cerr << "Could not write to magnetometer (acknowledgement error)\n";
        return ok && done;
    }

    bool ICM20948_I2C::_read_mag_byte(uint8_t mag_reg, uint8_t& byte)
    {
        bool ok = true;
        ok &= _write_byte(ICM20948_I2C_SLV4_ADDR_BANK,
                          ICM20948_I2C_SLV4_ADDR_ADDR,
                          static_cast<uint8_t>(ICM20948_MAGN_I2C_ADDR | 0x80));
        ok &= _write_byte(ICM20948_I2C_SLV4_REG_BANK,
                          ICM20948_I2C_SLV4_REG_ADDR, mag_reg);
        ok &= _write_byte(ICM20948_I2C_SLV4_CTRL_BANK,
                          ICM20948_I2C_SLV4_CTRL_ADDR, 0x80);

        bool done = false;
        for (int i = 0; i < 20; ++i) {
            _read_bit(ICM20948_I2C_MST_STATUS_BANK,
                      ICM20948_I2C_MST_STATUS_ADDR, 6, done);
            if (done) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        if (!done) std::cerr << "Could not read from magnetometer (acknowledgement error)\n";
        if (!done) return false;

        return _read_byte(ICM20948_I2C_SLV4_DI_BANK,
                          ICM20948_I2C_SLV4_DI_ADDR, byte);
    }

    bool ICM20948_I2C::check_DRDY_INT(){
        uint8_t int_status;
        
        if (!_read_byte(ICM20948_INT_STATUS_1_BANK,ICM20948_INT_STATUS_1_ADDR,int_status)){
            std::cerr << "Unable to read INT_STATUS" << std::endl;
            return false;
        }

        return (int_status & 0x20) != 0;
    }

} // namespace icm20948