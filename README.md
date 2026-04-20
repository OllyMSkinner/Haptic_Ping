<p align="center">
  <img src="images/HAPTIC.png" alt="Haptic Ping Logo" width="250"/>
</p>

# Haptic Ping

Haptic Ping combines table tennis shadow practice with haptic feedback to simulate a training experience that more closely resembles real play. Shadow practice is a widely used technique where table tennis players practice their strokes and form, but in the absence of a coach it can be difficult for players to know whether their position and movements are correct.

Our prototype addresses this limitation by providing real-time haptic feedback, allowing players to practice effectively without the need for a coach or any additional table tennis equipment.

Haptic Ping integrates an inertial measurement unit (IMU) and a piezo sensor mounted on the bat to monitor both orientation and physical interaction. The piezo sensor detects correct grip and handling, which triggers an LED to confirm proper finger placement. Once the correct starting position is established, a second LED illuminates to indicate the system is ready for swing analysis. The IMU then tracks the bat's motion to analyse swing patterns in real time. When a correct swing is detected, the system provides immediate haptic feedback through an eccentric rotating motor (ERM).

A project from ENG 5220 Real-Time Embedded Programming
University of Glasgow, 2026

---

## Hardware Components

| Component | Quantity | Link |
|---|---|---|
| Raspberry Pi 5 | 1× | [RS Online](https://uk.rs-online.com/web/p/raspberry-pi/0219255/) |
| ICM-20948 IMU | 1× | [RS Online](https://uk.rs-online.com/web/p/sensor-development-tools/2836590) |
| ERM Vibration Motor | 1× | [The Pi Hut](https://thepihut.com/products/vibrating-mini-motor-disc) |
| ADS1115 ADC | 1× | [The Pi Hut](https://thepihut.com/products/adafruit-ads1115-16-bit-adc) |
| Piezoelectric Sensor | 1× | [RS Online](https://uk.rs-online.com/web/p/piezo-buzzers/8377840) |
| LEDs (green) | 2× | — |
| Table Tennis Bat | 1× | — |

---

## GPIO Pin Assignments

| Signal | GPIO Pin |
|---|---|
| PWM (ERM motor) | GPIO 18 |
| IMU LED | GPIO 22 |
| Piezo LED | GPIO 25 |
| ADS1115 DRDY interrupt | GPIO 26 |
| IMU data-ready interrupt | GPIO 27 |
| I2C SDA | GPIO 2 |
| I2C SCL | GPIO 3 |

IMU I2C address: `0x69`, ADS1115 I2C address: `0x48`, I2C bus: `1`

---

## Wiring Diagram

![Wiring Diagram](images/Diagram_wLabel2.png)

---

## Hardware Assembly

- The LED outputs and Raspberry Pi were housed at a secondary station positioned in front of the user for clear visibility.
- The circuit board was affixed to the bat using Velcro to provide stability throughout the swing range of motion.
- The IMU was mounted at the distal end of the bat inside a custom 3D-printed enclosure to ensure consistent placement and minimise sensor displacement during dynamic use.
- Extended wiring (~1 metre) was fabricated by splicing female-to-female jumper cables with standard wire via soldering. Heat-shrink tubing was applied over each joint for insulation and mechanical reinforcement.

---

## How It Works

The system uses a gate logic sequence to ensure feedback is only triggered for a valid swing:

1. **Grip detection** — The piezo sensor on the bat is sampled at 128 Hz by the ADS1115 ADC. An EMA filter smooths the signal and a threshold of 0.95V triggers a press event, illuminating the piezo LED.
2. **Position confirmation** — The IMU data-ready interrupt (GPIO 27) wakes the reader thread at each sample. The position detector checks the acceleration vector against a reference orientation using a dot product. After 10 consecutive valid samples the system is considered in the correct starting position and the IMU LED illuminates.
3. **Swing analysis** — Once both grip and position are confirmed, the swing processor gates through the IMU acceleration magnitude. Gravity bias is removed via a rolling calibration window. The net linear acceleration magnitude is classified into duty cycle levels:
   - `>= 25 m/s²` → Highest Duty Cycle (long vibration burst)
   - `>= 15 m/s²` → Medium Duty Cycle (double burst)
   - `>= 10 m/s²` → Low Duty Cycle (short burst)
4. **Haptic feedback** — The ERM motor is driven by software PWM on GPIO 18 ramping up to 80% duty cycle to avoid back-EMF spikes. The feedback pattern plays and the system resets for the next swing.

---

## Realtime Design

The code is entirely event-driven with no polling loops:

- **IMU sampling** — driven by the data-ready GPIO interrupt via `epoll` + `eventfd`. The reader thread blocks on `epoll_wait` and wakes only when new data is available.
- **ADS1115 sampling** — driven by the ALERT/RDY GPIO edge event via `libgpiodcxx`. The worker thread blocks on `wait_edge_events`.
- **PWM generation** — runs in a dedicated thread using `timerfd` for precise on/off timing at 30 Hz. No `usleep` or busy-wait.
- **Haptic patterns** — played in a detached worker thread using `timerfd`-based waits for each ramp step.
- **Shutdown** — handled via `signalfd` blocking on `SIGINT`/`SIGHUP` rather than signal handlers.

## Maximum Latency
# IMU - ICM20948
The sampling frequency is calculated by 
	• Accelerometer: 1.125 kHz/(1+ sample_rate_div)
	• Gyrometer: 1.1kHz/(1+ sample_rate_div)
The maximum frequency for accelerometer is therefore 1.125 kHz and gyrometer is 1.1kHz. The maximum latency found for the accelerometer to work is at _____ (sample_rate_div = ), and ______ (sample_rate_div = ) for the gyrometer. 

# Piezo - ADS1115
The available sample rate are 8Hz, 16Hz, 32Hz, 64Hz, 128Hz, 250Hz, 475Hz, and 860Hz. The maximum latency for the ADS1115 is at _________.

This can be adjusted in the main.cpp. 


---

## Prerequisites

This project runs on Linux (Raspberry Pi OS). Not compatible with Windows.

### Enable I2C
```bash
sudo raspi-config
# Navigate to: Interface Options → I2C → Enable
```

### Install dependencies
```bash
sudo apt update
sudo apt install -y libgpiod-dev libgpiod2 libi2c-dev i2c-tools libyaml-cpp-dev cmake build-essential
```

---

## Libraries

### libgpiod
Used for all GPIO control on the Raspberry Pi via the Linux character device interface. In this project it is used in three places: the IMU reader configures the data-ready line (GPIO 27) as a rising-edge input and uses `epoll` on the GPIO file descriptor to wake the reader thread; the ADS1115 driver configures the ALERT/RDY line (GPIO 26) as a rising-edge input and blocks on `wait_edge_events`; and the LED controllers and PWM driver configure output lines to drive the LEDs and ERM motor.

### Linux I2C
Used to communicate with both the ICM-20948 IMU (address `0x69`) and the ADS1115 ADC (address `0x48`) over I2C bus 1. The IMU driver uses raw `ioctl(I2C_SLAVE)` and register-level read/write to initialise the sensor, configure accelerometer, gyroscope and magnetometer settings, and read decoded samples. The ADS1115 driver uses atomic `I2C_RDWR` transactions with retry logic to read conversion results. A shared `i2c1_mutex` coordinates access between the two threads to prevent interleaved bus transactions.

### yaml-cpp
Used by the ICM-20948 driver settings class to load sensor configuration parameters — accelerometer scale, gyroscope scale, sample rate divider, and digital low-pass filter settings — from a YAML file. This allows sensor tuning without recompiling.

---

## Compilation from Source

```bash
git clone <repo-url>
cd RTES_Table_Tennis

# Build main application
cmake -S src -B build
cmake --build build -j$(nproc)
```

---

## Usage

```bash
sudo ./build/rtes_main
```

The system will:
1. Wait for correct grip detected by piezo sensor → **Piezo LED on**
2. Wait for correct bat starting position detected by IMU → **IMU LED on**
3. Analyse swing in real time
4. Vibrate ERM motor on correct swing detection

Press `Ctrl+C` or `SIGHUP` to stop cleanly.

---

## Running Tests

```bash
cmake -S src -B build -DBUILD_TESTS=ON
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

31 unit tests across 6 test suites covering:

| Test Suite | What it tests |
|---|---|
| `PiezoEventDetector` | EMA filter, press/release thresholds, reset behaviour |
| `PositionDetector` | Stability counter, orientation detection, state transitions |
| `SwingCalibrator` | Bias averaging, recalibration trigger, callback firing |
| `SwingDetector` | Level classification, threshold boundaries, reset |
| `SwingFeedback` | PWM ramp patterns, reset callback, timeout guard |
| `SwingProcessor` | Gate logic, force/position interaction, magnitude output |

---

## Project Structure
```
RTES_Table_Tennis/
├── src/
│   ├── main.cpp
│   ├── CMakeLists.txt
│   └── libs/
│       ├── IMU/        # ICM-20948 driver and threaded reader
│       ├── IMU_math/   # Position detection, swing analysis, calibration
│       ├── LEDs/       # GPIO-backed LED controllers
│       ├── Motor/      # Software PWM and swing feedback patterns
│       └── Piezo/      # ADS1115 ADC driver and event detector
├── test/               # GTest unit tests
└── images/
```
---

## Authors

| Name | Matric |
|---|---|
| Despina Charalambous | 2689332C |
| Najaree Janjerdsak | 2717383J |
| Natalia McCoy | 2661134M |
| Olivia Skinner | 2671612S |
| Wiktoria Smolarek | 2619869S |

---

## Media

- [Instagram](https://www.instagram.com/hapticping)
- [TikTok](https://www.tiktok.com/@hapticping)

---

## License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.

---

## Acknowledgements

- [libgpiod](https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git/)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- [Google Test](https://github.com/google/googletest)
- University of Glasgow ENG 5220 teaching team
