#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <thread>

#include <gpiod.hpp>

class ADS1115settings
{
public:
    enum Mux : unsigned
    {
        AIN0_GND = 0b100,
        AIN1_GND = 0b101,
        AIN2_GND = 0b110,
        AIN3_GND = 0b111,
        // (add other mux modes if you have them)
    };

    enum SamplingRate : unsigned
    {
        FS8HZ   = 0b000,
        FS16HZ  = 0b001,
        FS32HZ  = 0b010,
        FS64HZ  = 0b011,
        FS128HZ = 0b100,
        FS250HZ = 0b101,
        FS475HZ = 0b110,
        FS860HZ = 0b111
    };

    enum PgaGain : unsigned
    {
        FSR6_144 = 0b000,
        FSR4_096 = 0b001,
        FSR2_048 = 0b010,
        FSR1_024 = 0b011,
        FSR0_512 = 0b100,
        FSR0_256 = 0b101
    };

    // ---- defaults ----
    int i2c_bus = 1;
    uint8_t address = 0x48;

    Mux mux = AIN0_GND;
    SamplingRate samplingRate = FS250HZ;
    PgaGain pgaGain = FSR2_048;

    // DRDY / ALERT-RDY (optional)
    bool use_drdy = true;     // <<< ADD THIS
    int drdy_chip = 0;
    unsigned drdy_gpio = 17;
};

class ADS1115rpi
{
public:
    using Callback = std::function<void(float)>;

    ADS1115rpi() = default;
    ~ADS1115rpi() { stop(); }

    void start(ADS1115settings settings);
    void stop();

    void setMux(ADS1115settings::Mux mux);

    void registerCallback(Callback cb) { adsCallbackInterface = std::move(cb); }

    float readVoltageOnce();

private:
    void worker();
    void dataReady();

    void i2c_writeWord(uint8_t reg, unsigned data);
    unsigned i2c_readWord(uint8_t reg);
    int16_t i2c_readConversion();

    float fullScaleVoltage() const
    {
        // Convert PGA setting to full scale (approx). ADS1115 is ±FSR.
        switch (ads1115settings.pgaGain)
        {
        case ADS1115settings::FSR6_144: return 6.144f;
        case ADS1115settings::FSR4_096: return 4.096f;
        case ADS1115settings::FSR2_048: return 2.048f;
        case ADS1115settings::FSR1_024: return 1.024f;
        case ADS1115settings::FSR0_512: return 0.512f;
        case ADS1115settings::FSR0_256: return 0.256f;
        default: return 2.048f;
        }
    }

private:
    ADS1115settings ads1115settings{};
    bool use_drdy_ = true; // <<< remember mode used at start()

    int fd_i2c = -1;

    std::shared_ptr<gpiod::chip> chip;
    std::shared_ptr<gpiod::line_request> request;

    bool running = false;
    std::thread thr;

    Callback adsCallbackInterface;

    // registers (match your existing values if different)
    static constexpr uint8_t reg_config    = 0x01;
    static constexpr uint8_t reg_lo_thres  = 0x02;
    static constexpr uint8_t reg_hi_thres  = 0x03;

    static constexpr int ISR_TIMEOUT_MS = 2000;
};
