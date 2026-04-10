#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <csignal>
#include <cmath>
#include <stdexcept>
#include <atomic>

#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>

#include "positiondetector.h"

static volatile std::sig_atomic_t g_run = 1;
static void on_sigint(int) { g_run = 0; }

// ========= Simple LED controller via gpiochip =========
class LED {
public:
    LED(const char* chipDev, int gpioLine) {
        chip_fd = open(chipDev, O_RDWR);
        if (chip_fd < 0) {
            throw std::runtime_error("Failed to open GPIO chip device");
        }

        std::memset(&req, 0, sizeof(req));
        req.lineoffsets[0] = static_cast<__u32>(gpioLine);
        req.flags = GPIOHANDLE_REQUEST_OUTPUT;
        req.lines = 1;
        req.default_values[0] = 0;
        std::strncpy(req.consumer_label, "imu-led", sizeof(req.consumer_label) - 1);

        if (ioctl(chip_fd, GPIO_GET_LINEHANDLE_IOCTL, &req) < 0) {
            close(chip_fd);
            throw std::runtime_error("Failed to get GPIO line handle");
        }
    }

    ~LED() {
        if (req.fd > 0) close(req.fd);
        if (chip_fd > 0) close(chip_fd);
    }

    void On()  { set(1); }
    void Off() { set(0); }

private:
    void set(__u8 v) {
        gpiohandle_data data{};
        data.values[0] = v;
        if (ioctl(req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data) < 0) {
            std::cerr << "Failed to set GPIO line value\n";
        }
    }

    int chip_fd = -1;
    gpiohandle_request req{};
};
// =====================================================

// ========= Callback that turns LED on =========
class LEDCallback : public PositionDetectorName::UprightCallback {
public:
    explicit LEDCallback(LED& ledRef) : led(ledRef) {}

    void OnUprightDetected() override {
        led.On();
        std::cout << "[LED] Upright detected -> LED ON\n";
    }

private:
    LED& led;
};
// =============================================

int main(int argc, char** argv)
{
    std::signal(SIGINT, on_sigint);

    // Allow overriding defaults:
    // ./app [/dev/rfcomm0] [samplePeriodMs]
    const char* devPath = "/dev/rfcomm0";
    int samplePeriodMs = 50;

    if (argc >= 2) devPath = argv[1];
    if (argc >= 3) samplePeriodMs = std::stoi(argv[2]);

    LED led("/dev/gpiochip0", 22);

    PositionDetectorName::PositionDetector positionDetector;
    LEDCallback ledCb(led);
    positionDetector.RegisterCallback(&ledCb);

    std::cout << "Pi 5: reading IMU samples from " << devPath << "\n";
    std::cout << "Ctrl+C to stop\n";
    std::cout << std::fixed << std::setprecision(3);

    std::atomic_bool runFlag(true);

    // Run RFCOMM reader in its own thread
    std::thread detectorThread([&] {
        const bool ok = positionDetector.RunFromRFCOMM(devPath, runFlag, samplePeriodMs);
        if (!ok) {
            std::cerr << "RunFromRFCOMM() failed (device missing or disconnected?)\n";
        }
    });

    // Main thread: just manage LED OFF when not upright
    while (g_run) {
        if (!positionDetector.IsUpright()) {
            led.Off();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    runFlag = false;
    detectorThread.join();

    led.Off();
    return 0;
}