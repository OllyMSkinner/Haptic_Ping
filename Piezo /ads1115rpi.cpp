#include "ads1115rpi.h"

#include <stdexcept>
#include <chrono>
#include <string>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#ifndef I2C_SLAVE
#include <linux/i2c-dev.h>
#endif

void ADS1115rpi::start(ADS1115settings settings)
{
    ads1115settings = settings;
    use_drdy_ = settings.use_drdy;

    char i2cPath[32];
    snprintf(i2cPath, sizeof(i2cPath), "/dev/i2c-%d", settings.i2c_bus);

    fd_i2c = open(i2cPath, O_RDWR);
    if (fd_i2c < 0)
        throw std::invalid_argument("Could not open I2C.");

    if (ioctl(fd_i2c, I2C_SLAVE, settings.address) < 0)
        throw std::invalid_argument("Could not access I2C address.");

    // Enable RDY via comparator trick (ALERT/RDY)
    // (Safe to leave these even if polling; they won’t hurt)
    i2c_writeWord(reg_lo_thres, 0x0000);
    i2c_writeWord(reg_hi_thres, 0x8000);

    // Build config register
    unsigned r = 0;
    r |= (0b1u << 15); // OS
    r |= (static_cast<unsigned>(settings.mux) << 12);          // MUX
    r |= (static_cast<unsigned>(settings.pgaGain) << 9);       // PGA
    r |= (0u << 8);                                            // MODE: 0 = continuous
    r |= (static_cast<unsigned>(settings.samplingRate) << 5);  // DR

    // Comparator bits:
    // We'll keep your original idea but you can tune later.
    // COMP_QUE = 00 to allow ALERT/RDY behaviour
    r |= 0b00u;              // COMP_QUE = 00
    r &= ~(0b11u);           // ensure COMP_QUE bits clear (bits 1..0)

    // Set comparator mode/polarity/latch:
    // For DRDY pulses, active-low + non-latching is typically easier,
    // but leave as-is if you prefer. Since polling doesn't use it, it doesn't matter.
    // r &= ~(1u << 3); // COMP_POL = 0 (active low)
    // r &= ~(1u << 2); // COMP_LAT = 0 (non-latching)

    i2c_writeWord(reg_config, r);

    // ---- DRDY/GPIO setup only if enabled ----
    if (!settings.use_drdy)
        return;

    const std::string chipPath = "/dev/gpiochip" + std::to_string(settings.drdy_chip);
    const std::string consumername =
        "gpioconsumer_" + std::to_string(settings.drdy_chip) + "_" + std::to_string(settings.drdy_gpio);

    gpiod::line_config line_cfg;
    line_cfg.add_line_settings(
        settings.drdy_gpio,
        gpiod::line_settings()
            .set_direction(gpiod::line::direction::INPUT)
            .set_edge_detection(gpiod::line::edge::RISING));

    chip = std::make_shared<gpiod::chip>(chipPath);

    auto builder = chip->prepare_request();
    builder.set_consumer(consumername);
    builder.set_line_config(line_cfg);
    request = std::make_shared<gpiod::line_request>(builder.do_request());

    running = true;
    thr = std::thread(&ADS1115rpi::worker, this);
}

void ADS1115rpi::setMux(ADS1115settings::Mux mux)
{
    unsigned r = i2c_readWord(reg_config);
    r &= ~(0b111u << 12);
    r |= (static_cast<unsigned>(mux) << 12);
    i2c_writeWord(reg_config, r);
    ads1115settings.mux = mux;
}

void ADS1115rpi::dataReady()
{
    const int16_t raw = i2c_readConversion();
    float v = (static_cast<float>(raw) / 32768.0f) * fullScaleVoltage();

    if (adsCallbackInterface)
        adsCallbackInterface(v);
}

void ADS1115rpi::worker()
{
    while (running)
    {
        bool got = request->wait_edge_events(std::chrono::milliseconds(ISR_TIMEOUT_MS));
        if (got)
        {
            gpiod::edge_event_buffer buffer;
            request->read_edge_events(buffer, 1);
            dataReady();
        }
    }
}

void ADS1115rpi::stop()
{
    // Only stop the worker/gpio if we started it
    if (use_drdy_)
    {
        if (running)
        {
            running = false;
            if (thr.joinable())
                thr.join();
        }

        if (request)
            request->release();
        if (chip)
            chip->close();

        request.reset();
        chip.reset();
    }

    if (fd_i2c >= 0)
        close(fd_i2c);

    fd_i2c = -1;
}

void ADS1115rpi::i2c_writeWord(uint8_t reg, unsigned data)
{
    uint8_t tmp[3];
    tmp[0] = reg;
    tmp[1] = static_cast<uint8_t>((data & 0xff00) >> 8);
    tmp[2] = static_cast<uint8_t>(data & 0x00ff);

    long int r = write(fd_i2c, tmp, 3);
    if (r < 0)
        throw std::invalid_argument("Could not write to i2c.");
}

unsigned ADS1115rpi::i2c_readWord(uint8_t reg)
{
    uint8_t tmp[2];
    uint8_t regbuf[1] = {reg};

    write(fd_i2c, regbuf, 1);
    long int r = read(fd_i2c, tmp, 2);
    if (r < 0)
        throw std::invalid_argument("Could not read from i2c.");

    return (static_cast<unsigned>(tmp[0]) << 8) | static_cast<unsigned>(tmp[1]);
}

int16_t ADS1115rpi::i2c_readConversion()
{
    const uint8_t reg = 0; // conversion register
    uint8_t tmp[2];
    write(fd_i2c, &reg, 1);

    long int r = read(fd_i2c, tmp, 2);
    if (r < 0)
        throw std::invalid_argument("Could not read from i2c.");

    return static_cast<int16_t>((tmp[0] << 8) | tmp[1]);
}

float ADS1115rpi::readVoltageOnce()
{
    const int16_t raw = i2c_readConversion();
    return (static_cast<float>(raw) / 32768.0f) * fullScaleVoltage();
}
