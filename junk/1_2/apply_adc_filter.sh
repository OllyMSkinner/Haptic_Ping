#!/usr/bin/env bash
set -euo pipefail

cp main.cpp main.cpp.bak.$(date +%Y%m%d_%H%M%S)

python3 - <<'PY'
from pathlib import Path
p = Path("main.cpp")
s = p.read_text()

old_block = r'''    ADS1115rpi ads1115rpi;
    ads1115rpi.registerCallback([&](float v) {
        static int piezo_count = 0;
        if (++piezo_count % 32 == 0) {
            std::printf("Piezo: %.3f\n", v);
        }
        piezoDetector.processSample(v);
    });

    try {
        ads1115rpi.start();
    } catch (const std::exception& e) {'''

new_block = r'''    ADS1115rpi ads1115rpi;
    ads1115rpi.registerCallback([&](float v) {
        static int piezo_count = 0;
        static bool filter_init = false;
        static float filtered_v = 0.0f;

        if (!filter_init) {
            filtered_v = v;
            filter_init = true;
        } else {
            // simple low-pass filter
            filtered_v = 0.8f * filtered_v + 0.2f * v;
        }

        if (++piezo_count % 32 == 0) {
            std::printf("Piezo raw: %.3f  filtered: %.3f\n", v, filtered_v);
        }

        piezoDetector.processSample(filtered_v);
    });

    ADS1115settings adcSettings;
    adcSettings.i2c_bus = 1;
    adcSettings.address = ADS1115settings::DEFAULT_ADS1115_ADDRESS;
    adcSettings.samplingRate = ADS1115settings::FS128HZ;
    adcSettings.pgaGain = ADS1115settings::FSR2_048;
    adcSettings.channel = ADS1115settings::AIN0;
    adcSettings.drdy_chip = 0;
    adcSettings.drdy_gpio = ADS1115settings::DEFAULT_ALERT_RDY_TO_GPIO;

    try {
        ads1115rpi.start(adcSettings);
    } catch (const std::exception& e) {'''

if old_block not in s:
    raise SystemExit("Did not find expected ADS1115 block in main.cpp")

s = s.replace(old_block, new_block)

p.write_text(s)
PY

rm -rf build
mkdir build
cd build
cmake ..
make -j