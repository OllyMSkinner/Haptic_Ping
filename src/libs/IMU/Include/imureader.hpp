#pragma once

#include <atomic>
#include <functional>
#include <thread>

#include "Icm20948driver.hpp"
#include <gpiod.h>
#include <future>

// IMUReader provides a threaded, event-driven wrapper around the low-level
// ICM-20948 driver.

// Design intention:
// - Single Responsibility:
//   This class is responsible for coordinating asynchronous IMU sample
//   acquisition from the data-ready GPIO and forwarding decoded samples to
//   client code.
// - Open/Closed:
//   New behaviours can be attached through the callback interface without
//   changing the reader logic itself.
// - Encapsulation:
//   Thread management, GPIO handles, and event-loop details are hidden behind
//   a small public API.
// - Dependency control:
//   The low-level register and I2C interaction remain inside the IMU driver,
//   while this class focuses on threading and sample dispatch.
//
// Threaded IMU reader driven by the data-ready GPIO.

class IMUReader
{
public:
    // Callback invoked for each decoded IMU sample.
    // Using a callback keeps the class open for extension: logging, filtering,
    // fusion, motion detection, or UI updates can be added without modifying
    // the reader implementation.
    using SampleCallback = std::function<void(float ax, float ay, float az,
                                              float gx, float gy, float gz,
                                              float mx, float my)>;

    // Construct the reader with its hardware configuration and runtime settings.
    // The IMU configuration is injected from outside so the reader can be
    // reused with different buses, addresses, and sensor settings.
    IMUReader(unsigned           i2c_bus,
              unsigned           i2c_address = ICM20948_I2C_ADDR,
              const char*        gpiochip    = "/dev/gpiochip0",
              unsigned           rdy_line    = 27,
              icm20948::settings settings    = icm20948::settings());

    // Destructor ensures the threaded reader can clean up safely through the
    // class interface.
    ~IMUReader();

    // Initialise the underlying IMU device before starting the worker thread.
    bool init();

    // Register client code to receive new IMU samples.
    // This keeps sample processing outside the reader itself, improving
    // flexibility and separation of concerns.
    void setCallback(SampleCallback cb);

    // Start the background reader thread and GPIO-driven event loop.
    bool start();

    // Stop the worker thread and release runtime resources cleanly.
    void stop();

private:
    // Background worker loop waiting for DRDY events and dispatching samples.
    // This keeps blocking/event-loop behaviour internal to the class so client
    // code does not need to manage low-level synchronisation details.
    void worker();

    // Low-level IMU driver instance.
    // Hardware-specific register access is delegated to the driver so this
    // class can focus on threaded sample acquisition and dispatch.
    icm20948::ICM20948_I2C imu_;

    // Client callback for newly acquired samples.
    // Stored privately so callback ownership and invocation remain controlled
    // by the reader.
    SampleCallback         callback_;

    // Background reader thread.
    // Thread lifetime is encapsulated inside the class rather than exposed to
    // higher-level application code.
    std::thread            worker_thread_;

    // Run flag shared with the worker thread.
    // Atomic state allows safe coordination between start/stop and the worker.
    std::atomic_bool       running_;

    // GPIO chip path and DRDY line number.
    // These hardware configuration details are stored with the reader instance
    // so callers only configure them once during construction.
    const char*         gpiochip_path_;
    unsigned            rdy_line_;

    // libgpiod handles for the DRDY line.
    // These low-level resources are kept private to preserve encapsulation.
    gpiod_chip*         chip_;
    gpiod_line_request* request_;

    // File descriptors used by the event loop.
    // Internal event-loop state is hidden so callers interact only through the
    // public start/stop interface.
    int                 gpio_fd_;
    int                 stop_fd_;
};
