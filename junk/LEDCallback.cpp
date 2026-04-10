#include <iostream>
#include <iomanip>
#include <csignal>
#include <stdexcept>
#include <cstring>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "positiondetector.h"
#include "IMUPublisher.hpp"

static volatile std::sig_atomic_t g_run = 1;
static void on_sigint(int) { g_run = 0; }

// ========= Simple raw LED GPIO =========
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
        Off();
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
// =======================================

// ========= LED worker thread object =========
class LEDController {
public:
    explicit LEDController(LED& ledRef)
        : led(ledRef),
          running(false),
          desiredState(false),
          stateChanged(false)
    {
    }

    ~LEDController() {
        stop();
    }

    void start() {
        if (running) return;
        running = true;
        workerThread = std::thread(&LEDController::worker, this);
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mtx);
            running = false;
            stateChanged = true;
        }
        cv.notify_one();

        if (workerThread.joinable()) {
            workerThread.join();
        }
    }

    void setStartingPosition(bool active) {
        {
            std::lock_guard<std::mutex> lock(mtx);
            desiredState = active;
            stateChanged = true;
        }
        cv.notify_one();
    }

private:
    void worker() {
        led.Off();

        bool currentOutput = false;

        while (true) {
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock, [&] { return stateChanged; });

            if (!running) {
                break;
            }

            const bool newState = desiredState;
            stateChanged = false;
            lock.unlock();

            if (newState != currentOutput) {
                if (newState) {
                    led.On();
                    std::cout << "[LED THREAD] Starting position detected -> LED ON\n";
                } else {
                    led.Off();
                    std::cout << "[LED THREAD] Starting position lost -> LED OFF\n";
                }
                currentOutput = newState;
            }
        }

        led.Off();
    }

    LED& led;
    std::thread workerThread;
    std::mutex mtx;
    std::condition_variable cv;

    bool running;
    bool desiredState;
    bool stateChanged;
};
// ===========================================

// ========= Callback from detector to LED thread =========
class LEDCallback : public PositionDetectorName::UprightCallback {
public:
    explicit LEDCallback(LEDController& controllerRef)
        : controller(controllerRef) {}

    void OnUprightStateChanged(bool isUpright) override {
        controller.setStartingPosition(isUpright);
    }

private:
    LEDController& controller;
};
// ========================================================

int main()
{
    std::signal(SIGINT, on_sigint);

    try {
        LED led("/dev/gpiochip0", 22);
        LEDController ledController(led);
        ledController.start();

        PositionDetectorName::PositionDetector detector;
        LEDCallback ledCb(ledController);
        detector.RegisterCallback(&ledCb);

        icm20948::IMUPublisher imuPublisher(
            1,                      // I2C bus
            ICM20948_I2C_ADDR,      // IMU address
            "/dev/gpiochip0",       // GPIO chip
            17                      // RDY pin from IMU
        );

        if (!imuPublisher.init()) {
            std::cerr << "IMU init failed\n";
            ledController.stop();
            return 1;
        }

        imuPublisher.registerCallback(
            [&detector](const icm20948::IMUSample& sample) {
                detector.HasSample(sample);
            }
        );

        if (!imuPublisher.start()) {
            std::cerr << "Failed to start IMU publisher\n";
            ledController.stop();
            return 1;
        }

        std::cout << "System running. Ctrl+C to stop.\n";

        while (g_run) {
            pause();
        }

        imuPublisher.stop();
        ledController.stop();
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << "\n";
        return 1;
    }
}