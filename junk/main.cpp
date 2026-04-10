#include <iostream>
#include <csignal>
#include <stdexcept>
#include <cstring>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <string>
#include <sstream>
#include <atomic>

#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "Starting_position/positiondetector.h"

static volatile std::sig_atomic_t g_run = 1;

static void on_sigint(int)
{
    g_run = 0;
}

// =====================================================
// Low-level LED GPIO output object
// =====================================================
class LED
{
public:
    LED(const char* chipDev, int gpioLine)
    {
        chip_fd = open(chipDev, O_RDWR);
        if (chip_fd < 0) {
            throw std::runtime_error("Failed to open GPIO chip device");
        }

        std::memset(&req, 0, sizeof(req));
        req.lineoffsets[0] = static_cast<__u32>(gpioLine);
        req.flags = GPIOHANDLE_REQUEST_OUTPUT;
        req.lines = 1;
        req.default_values[0] = 0;
        std::strncpy(req.consumer_label, "starting-position-led",
                     sizeof(req.consumer_label) - 1);

        if (ioctl(chip_fd, GPIO_GET_LINEHANDLE_IOCTL, &req) < 0) {
            close(chip_fd);
            throw std::runtime_error("Failed to get GPIO line handle");
        }
    }

    ~LED()
    {
        Off();

        if (req.fd > 0) {
            close(req.fd);
        }

        if (chip_fd > 0) {
            close(chip_fd);
        }
    }

    void On()
    {
        set(1);
    }

    void Off()
    {
        set(0);
    }

private:
    void set(__u8 value)
    {
        gpiohandle_data data{};
        data.values[0] = value;

        if (ioctl(req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data) < 0) {
            std::cerr << "Failed to set LED GPIO line value\n";
        }
    }

    int chip_fd = -1;
    gpiohandle_request req{};
};

// =====================================================
// LED controller with its own worker thread
// =====================================================
class LEDController
{
public:
    explicit LEDController(LED& ledRef)
        : led(ledRef),
          running(false),
          desiredState(false),
          stateChanged(false)
    {
    }

    ~LEDController()
    {
        stop();
    }

    void start()
    {
        if (running) {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mtx);
            running = true;
            desiredState = false;
            stateChanged = true;
        }

        workerThread = std::thread(&LEDController::worker, this);
    }

    void stop()
    {
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

    void setStartingPosition(bool active)
    {
        {
            std::lock_guard<std::mutex> lock(mtx);
            desiredState = active;
            stateChanged = true;
        }

        cv.notify_one();
    }

private:
    void worker()
    {
        bool currentOutput = false;
        led.Off();

        while (true) {
            std::unique_lock<std::mutex> lock(mtx);

            cv.wait(lock, [this] {
                return stateChanged;
            });

            if (!running) {
                break;
            }

            const bool newState = desiredState;
            stateChanged = false;
            lock.unlock();

            if (newState != currentOutput) {
                if (newState) {
                    led.On();
                    std::cout << "[LED THREAD] Starting position -> LED ON\n";
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

// =====================================================
// Detector -> LED callback
// =====================================================
class LEDCallback : public PositionDetectorName::UprightCallback
{
public:
    explicit LEDCallback(LEDController& controllerRef)
        : controller(controllerRef)
    {
    }

    void OnUprightStateChanged(bool isUpright) override
    {
        controller.setStartingPosition(isUpright);
    }

private:
    LEDController& controller;
};

// =====================================================
// Bluetooth receiver thread
// Blocks on read() from RFCOMM and sends samples to detector
// =====================================================
static void BluetoothReceiverWorker(const char* devPath,
                                    std::atomic_bool& runFlag,
                                    PositionDetectorName::PositionDetector& detector)
{
    std::cout << "[BT] Opening RFCOMM device: " << devPath << "\n";

    int fd = ::open(devPath, O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        std::perror("open(rfcomm)");
        return;
    }

    std::string buffer;
    buffer.reserve(4096);

    char tmp[512];

    auto clean_line = [](std::string& s) {
        // Keep only from first plausible CSV character onward:
        // either 't' from "ts_ms" or a digit from timestamp data.
        std::size_t start = std::string::npos;
        for (std::size_t i = 0; i < s.size(); ++i) {
            unsigned char c = static_cast<unsigned char>(s[i]);
            if (c == 't' || (c >= '0' && c <= '9')) {
                start = i;
                break;
            }
        }

        if (start == std::string::npos) {
            s.clear();
            return;
        }

        s.erase(0, start);

        // Remove trailing CR/LF/space/tab/non-printables
        while (!s.empty()) {
            unsigned char c = static_cast<unsigned char>(s.back());
            if (c == '\r' || c == '\n' || c == ' ' || c == '\t' || c < 32) {
                s.pop_back();
            } else {
                break;
            }
        }
    };

    while (runFlag.load()) {
        const ssize_t n = ::read(fd, tmp, sizeof(tmp));

        if (n > 0) {
            buffer.append(tmp, static_cast<std::size_t>(n));

            std::size_t pos = 0;
            while (true) {
                const std::size_t nl = buffer.find('\n', pos);
                if (nl == std::string::npos) {
                    break;
                }

                std::string line = buffer.substr(pos, nl - pos);
                pos = nl + 1;

                clean_line(line);

                if (line.empty()) {
                    continue;
                }

                // Skip header lines
                if (line.rfind("ts_ms,", 0) == 0 ||
                    line.rfind("ts_", 0) == 0 ||
                    line.rfind("ts,", 0) == 0) {
                    continue;
                }

                // Real data lines should start with timestamp digit
                if (!(line[0] >= '0' && line[0] <= '9')) {
                    std::cerr << "[BT] Ignoring malformed line: " << line << "\n";
                    continue;
                }

                std::stringstream ss(line);
                std::string token;
                icm20948::IMUSample sample{};

                try {
                    if (!std::getline(ss, token, ',')) continue; // timestamp

                    if (!std::getline(ss, token, ',')) continue;
                    sample.ax = std::stof(token);

                    if (!std::getline(ss, token, ',')) continue;
                    sample.ay = std::stof(token);

                    if (!std::getline(ss, token, ',')) continue;
                    sample.az = std::stof(token);

                    if (!std::getline(ss, token, ',')) continue;
                    sample.gx = std::stof(token);

                    if (!std::getline(ss, token, ',')) continue;
                    sample.gy = std::stof(token);

                    if (!std::getline(ss, token, ',')) continue;
                    sample.gz = std::stof(token);

                    if (!std::getline(ss, token, ',')) continue;
                    sample.mx = std::stof(token);

                    if (!std::getline(ss, token, ',')) continue;
                    sample.my = std::stof(token);

                    if (!std::getline(ss, token, ',')) continue;
                    sample.mz = std::stof(token);
                }
                catch (...) {
                    std::cerr << "[BT] Failed to parse line: " << line << "\n";
                    continue;
                }

                detector.HasSample(sample);
            }

            if (pos > 0) {
                buffer.erase(0, pos);
            }
        }
        else if (n == 0) {
            std::cerr << "[BT] RFCOMM closed\n";
            break;
        }
        else {
            std::perror("read(rfcomm)");
            break;
        }
    }

    ::close(fd);
    std::cout << "[BT] Receiver thread stopped\n";
}
// =====================================================
// Main
// =====================================================
int main(int argc, char** argv)
{
    std::signal(SIGINT, on_sigint);

    const char* devPath = "/dev/rfcomm0";
    if (argc >= 2) {
        devPath = argv[1];
    }

    try {
        // -----------------------------
        // Actuator side
        // -----------------------------
        LED led("/dev/gpiochip0", 22);
        LEDController ledController(led);
        ledController.start();

        // -----------------------------
        // Processing side
        // -----------------------------
        PositionDetectorName::PositionDetector detector;
        LEDCallback ledCb(ledController);
        detector.RegisterCallback(&ledCb);

        // -----------------------------
        // Bluetooth receiver thread
        // -----------------------------
        std::atomic_bool runFlag(true);

        std::thread btThread([&]() {
            BluetoothReceiverWorker(devPath, runFlag, detector);
        });

        std::cout << "System running from Bluetooth sender on " << devPath
                  << ". Ctrl+C to stop.\n";

        // Main thread just waits for Ctrl+C
        while (g_run) {
            pause();
        }

        std::cout << "\nStopping system...\n";

        runFlag = false;

        // Closing the RFCOMM device from another thread is not handled here,
        // so if read() stays blocked you may need to disconnect rfcomm or send
        // one final line from the Pi 3 before shutdown.
        if (btThread.joinable()) {
            btThread.join();
        }

        ledController.stop();
        led.Off();

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << "\n";
        return 1;
    }
}