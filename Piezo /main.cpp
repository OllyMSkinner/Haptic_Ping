#include "ads1115rpi.h"
#include <iostream>
#include <thread>
#include <chrono>

int main()
{
    ADS1115rpi ads;

    ADS1115settings s;
    s.i2c_bus = 1;
    s.address = 0x48;
    s.mux = ADS1115settings::AIN0_GND;
    s.samplingRate = ADS1115settings::FS250HZ;
    s.pgaGain = ADS1115settings::FSR2_048;

    // IMPORTANT: polling mode (no ALERT/RDY GPIO needed)
    s.use_drdy = false;

    ads.start(s);

    while (true)
    {
        std::cout << ads.readVoltageOnce() << "\n"; // numbers only for plotting
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
