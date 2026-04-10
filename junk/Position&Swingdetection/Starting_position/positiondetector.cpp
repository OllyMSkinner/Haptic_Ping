#include "positiondetector.h"

#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <algorithm>
#include <sstream>
#include <string>

#include <fcntl.h>
#include <unistd.h>

#include "icm20948_i2c.hpp"

// Confirm upright after ~200 ms of stability
static int stabilitySamplesRequired = 20;

namespace PositionDetectorName {

// ==========================================================
// Core Upright Detection Logic (UPDATED: 9-axis detection)
// ==========================================================
void PositionDetector::CheckPosition(
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float mx, float my, float mz)
{

    const bool accelOK =
        (std::fabs(ax + 7.8f) < 9.8f) &&
        (std::fabs(ay - 5.4f) < -3.0f) &&
        (std::fabs(az + 1.6f) < 1.0f);

    const bool gyroStable =
        (std::fabs(gx - 0.1f) < 1.0f) &&
        (std::fabs(gy - 0.5f) < 0.05f) &&
        (std::fabs(gz - 0.2f) < 0.3f);

    const bool magnetOK =
        (std::fabs(mx - 38.5f) < -27.1f) &&
        (std::fabs(my + 6.3f) < 16.5f) &&
        (std::fabs(mz + 19.9f) < 28.5f);

    const bool isUpright = accelOK && gyroStable && magnetOK;

    std::cout << "AX=" << ax
              << " AY=" << ay
              << " AZ=" << az
              << " | GX=" << gx
              << " GY=" << gy
              << " GZ=" << gz
              << " | MX=" << mx
              << " MY=" << my
              << " MZ=" << mz
              << " | Upright=" << isUpright
              << " | Counter=" << Counter
              << " | Confirmed=" << UprightConfirmed
              << std::endl;

    if (isUpright) {
        Counter++;

        if (Counter >= stabilitySamplesRequired && !UprightConfirmed) {
            UprightConfirmed = true;

            std::cout << "[EVENT] Upright position detected!" << std::endl;

            if (callback) {
                callback->OnUprightDetected();
            }
        }
    }
    else {
        Counter = 0;
        UprightConfirmed = false;
    }
}

// ==========================================================
// Mode 1: Direct IMU (Pi 5 standalone)
// ==========================================================
bool PositionDetector::RunFromIMU(unsigned bus,
                                  unsigned addr,
                                  std::atomic_bool& runFlag,
                                  int samplePeriodMs)
{
    icm20948::ICM20948_I2C imu(bus, addr);

    if (!imu.init()) {
        std::cerr << "ICM20948 init() failed (bus=" << bus
                  << ", addr=0x" << std::hex << addr << std::dec << ")\n";
        return false;
    }

    const int targetConfirmMs = 200;
    stabilitySamplesRequired =
        std::max(1, targetConfirmMs / std::max(1, samplePeriodMs));

    std::cout << "PositionDetector streaming IMU (direct I2C)\n";

    while (runFlag.load()) {

        const bool ok_ag = imu.read_accel_gyro();
        const bool ok_m  = imu.read_magn();

        if (ok_ag) {

            float mx = ok_m ? imu.magn[0] : 0.0f;
            float my = ok_m ? imu.magn[1] : 0.0f;
            float mz = ok_m ? imu.magn[2] : 0.0f;

            CheckPosition(
                imu.accel[0],
                imu.accel[1],
                imu.accel[2],
                imu.gyro[0],
                imu.gyro[1],
                imu.gyro[2],
                mx, my, mz);
        }
        else {
            Counter = 0;
            UprightConfirmed = false;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(samplePeriodMs));
    }

    return true;
}

// ==========================================================
// Mode 2: Bluetooth RFCOMM (Pi 5 receiver)
// ==========================================================
bool PositionDetector::RunFromRFCOMM(const char* devPath,
                                     std::atomic_bool& runFlag,
                                     int samplePeriodMs)
{
    const int targetConfirmMs = 200;
    stabilitySamplesRequired =
        std::max(1, targetConfirmMs / std::max(1, samplePeriodMs));

    std::cout << "PositionDetector reading from RFCOMM: "
              << devPath << "\n";

    int fd = ::open(devPath, O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        std::perror("open(rfcomm)");
        return false;
    }

    std::string buffer;
    buffer.reserve(4096);

    char tmp[512];

    while (runFlag.load()) {

        ssize_t n = ::read(fd, tmp, sizeof(tmp));

        if (n > 0) {
            buffer.append(tmp, n);

            size_t pos = 0;
            while (true) {

                size_t nl = buffer.find('\n', pos);
                if (nl == std::string::npos) break;

                std::string line = buffer.substr(pos, nl - pos);

                if (!line.empty() && line.back() == '\r')
                    line.pop_back();

                pos = nl + 1;

                if (line.rfind("ts_", 0) == 0)
                    continue;

                std::stringstream ss(line);
                std::string token;

                float ax, ay, az, gx, gy, gz, mx, my, mz;

                try {

                    std::getline(ss, token, ','); // ts

                    std::getline(ss, token, ','); ax = std::stof(token);
                    std::getline(ss, token, ','); ay = std::stof(token);
                    std::getline(ss, token, ','); az = std::stof(token);

                    std::getline(ss, token, ','); gx = std::stof(token);
                    std::getline(ss, token, ','); gy = std::stof(token);
                    std::getline(ss, token, ','); gz = std::stof(token);

                    std::getline(ss, token, ','); mx = std::stof(token);
                    std::getline(ss, token, ','); my = std::stof(token);
                    std::getline(ss, token, ','); mz = std::stof(token);

                }
                catch (...) {
                    continue;
                }

                CheckPosition(ax, ay, az, gx, gy, gz, mx, my, mz);
            }

            if (pos > 0)
                buffer.erase(0, pos);
        }
        else if (n < 0) {
            std::perror("read(rfcomm)");
            Counter = 0;
            UprightConfirmed = false;
            break;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(samplePeriodMs));
    }

    ::close(fd);
    return true;
}

} // namespace PositionDetectorName
