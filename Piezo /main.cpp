#include <iostream>
#include <stdexcept>

#include "ads1115rpi.h"
#include "event_detector.h"

int main()
{
    try
    {
        constexpr float DIP_THRESHOLD  = 0.5f;
        constexpr float PEAK_THRESHOLD =  4.5f;

        PiezoEventDetector detector(DIP_THRESHOLD, PEAK_THRESHOLD);

        ADS1115rpi ads1115rpi;

        ads1115rpi.registerCallback([&](float v)
        {
            std::cout << v << std::endl;
            detector.processSample(v);
        });


        ADS1115settings s;
        s.i2c_bus = 1;
        s.address = 0x48;
        s.samplingRate = ADS1115settings::FS128HZ;

        std::cout << "Starting ADS1115..." << std::endl;
        ads1115rpi.start(s);

        std::cout << "Running. Press Enter to stop." << std::endl;
        std::cin.get();

        ads1115rpi.stop();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
