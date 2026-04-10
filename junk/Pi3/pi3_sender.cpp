#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <string>

#include <fcntl.h>
#include <unistd.h>

#include "icm20948_i2c.hpp"

static volatile std::sig_atomic_t g_run = 1;
static void on_sigint(int) { g_run = 0; }

static int open_with_retry(const char* path, int flags, int retries = 50, int ms = 100) {
    for (int i = 0; i < retries; ++i) {
        int fd = ::open(path, flags);
        if (fd >= 0) return fd;
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }
    return -1;
}

int main(int argc, char** argv)
{
    std::signal(SIGINT, on_sigint);

    unsigned bus = 1;
    unsigned addr = ICM20948_I2C_ADDR;
    const char* out_dev = "/dev/rfcomm0";
    const char* csv_path = "pi3_imu_log.csv";

    if (argc >= 2) bus = static_cast<unsigned>(std::stoul(argv[1]));
    if (argc >= 3) addr = static_cast<unsigned>(std::stoul(argv[2], nullptr, 16));
    if (argc >= 4) out_dev = argv[3];
    if (argc >= 5) csv_path = argv[4];

    std::cout << "Initialising IMU (bus=" << bus << ", addr=0x"
              << std::hex << addr << std::dec << ")...\n";

    icm20948::ICM20948_I2C imu(bus, addr);
    if (!imu.init()) {
        std::cerr << "ICM20948 init() failed\n";
        return 1;
    }

    std::cout << "IMU init OK\n";

    int bt_fd = open_with_retry(out_dev, O_WRONLY | O_NOCTTY);
    if (bt_fd < 0) {
        std::cerr << "Failed to open " << out_dev << "\n";
        return 1;
    }

    std::cout << "Bluetooth connected: " << out_dev << "\n";

    int csv_fd = ::open(csv_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (csv_fd >= 0) {
        const char* hdr = "ts_ms,ax,ay,az,gx,gy,gz,mx,my,mz\n";
        ::write(csv_fd, hdr, std::strlen(hdr));
    }

    const char* hdr = "ts_ms,ax,ay,az,gx,gy,gz,mx,my,mz\n";
    ::write(bt_fd, hdr, std::strlen(hdr));

    std::cout << "Streaming IMU data...\n";

    auto t0 = std::chrono::steady_clock::now();

    float last_mx = 0, last_my = 0, last_mz = 0;

    while (g_run) {

        bool ok_ag = imu.read_accel_gyro();

        bool ok_m = false;
        for (int i = 0; i < 3 && !ok_m; ++i) {
            ok_m = imu.read_magn();
            if (!ok_m)
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        auto now = std::chrono::steady_clock::now();
        uint64_t ts_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();

        if (!ok_ag) {
            std::cerr << "Warning: accel/gyro read failed\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        if (ok_m) {
            last_mx = imu.magn[0];
            last_my = imu.magn[1];
            last_mz = imu.magn[2];
        } else {
            std::cerr << "Warning: magnetometer read failed, using last value\n";
        }

        float mx = last_mx;
        float my = last_my;
        float mz = last_mz;

        char line[256];
        int n = std::snprintf(
            line, sizeof(line),
            "%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            static_cast<unsigned long long>(ts_ms),
            imu.accel[0], imu.accel[1], imu.accel[2],
            imu.gyro[0],  imu.gyro[1],  imu.gyro[2],
            mx, my, mz
        );

        if (n > 0) {
            if (::write(bt_fd, line, static_cast<size_t>(n)) < 0) {
                std::cerr << "Bluetooth write failed\n";
                break;
            }

            if (csv_fd >= 0)
                ::write(csv_fd, line, static_cast<size_t>(n));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (csv_fd >= 0) ::close(csv_fd);
    ::close(bt_fd);

    std::cout << "Sender stopped\n";
    return 0;
}