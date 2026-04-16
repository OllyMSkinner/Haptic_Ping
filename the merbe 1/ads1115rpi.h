#ifndef __ADS1115RPI_H
#define __ADS1115RPI_H

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <atomic>
#include <thread>
#include <gpiod.hpp>
#include <functional>
#include <memory>

#ifndef NDEBUG
#define DEBUG
#endif

struct ADS1115settings
{
    int i2c_bus = 1;

    static constexpr uint8_t DEFAULT_ADS1115_ADDRESS = 0x48;
    uint8_t address = DEFAULT_ADS1115_ADDRESS;

    enum SamplingRates
    {
        FS8HZ = 0,
        FS16HZ = 1,
        FS32HZ = 2,
        FS64HZ = 3,
        FS128HZ = 4,
        FS250HZ = 5,
        FS475HZ = 6,
        FS860HZ = 7
    };

    inline unsigned getSamplingRate() const
    {
        const unsigned SamplingRateEnum2Value[8] =
            {8, 16, 32, 64, 128, 250, 475, 860};
        return SamplingRateEnum2Value[samplingRate];
    }

    SamplingRates samplingRate = FS128HZ;

    enum PGA
    {
        FSR2_048 = 2,
        FSR1_024 = 3,
        FSR0_512 = 4,
        FSR0_256 = 5
    };

    PGA pgaGain = FSR2_048;

    enum Input
    {
        AIN0 = 0,
        AIN1 = 1,
        AIN2 = 2,
        AIN3 = 3
    };

    Input channel = AIN0;

    int drdy_chip = 0;

    static constexpr int DEFAULT_ALERT_RDY_TO_GPIO = 26;
    int drdy_gpio = DEFAULT_ALERT_RDY_TO_GPIO;
};

ADS1115settings makeDefaultADS1115Settings();

class ADS1115rpi
{
public:
    ~ADS1115rpi()
    {
        stop();
    }

    using ADSCallbackInterface = std::function<void(float)>;

    void registerCallback(ADSCallbackInterface ci)
    {
        adsCallbackInterface = ci;
    }

    void setChannel(ADS1115settings::Input channel);
    void start(ADS1115settings settings = ADS1115settings());
    ADS1115settings getADS1115settings() const
    {
        return ads1115settings;
    }
    void stop();

    // Enable retries on I2C read failure. Safe to call from any thread.
    // Default: disabled. Enable only when highest duty cycle is active.
    void setRetryEnabled(bool enabled) { retry_enabled_.store(enabled); }

private:
    ADS1115settings ads1115settings;

    void dataReady();
    void worker();

    void i2c_selectDevice();
    void i2c_writeWord(uint8_t reg, unsigned data);
    unsigned i2c_readWord(uint8_t reg);
    int i2c_readConversion();

    const uint8_t reg_config = 1;
    const uint8_t reg_lo_thres = 2;
    const uint8_t reg_hi_thres = 3;

    float fullScaleVoltage()
    {
        switch (ads1115settings.pgaGain)
        {
        case ADS1115settings::FSR2_048:
            return 2.048f;
        case ADS1115settings::FSR1_024:
            return 1.024f;
        case ADS1115settings::FSR0_512:
            return 0.512f;
        case ADS1115settings::FSR0_256:
            return 0.256f;
        }
        assert(1 == 0);
        return 0.0f;
    }

    std::shared_ptr<gpiod::chip> chip;
    std::shared_ptr<gpiod::line_request> request;
    std::thread thr;
    int fd_i2c = -1;
    bool running = false;
    ADSCallbackInterface adsCallbackInterface;
    std::atomic<bool> retry_enabled_{false};

    static constexpr int64_t ISR_TIMEOUT_MS = 500;
};

#endif
