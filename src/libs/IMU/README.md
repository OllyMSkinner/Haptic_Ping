# IMU - ICM-20948

## Pin Connection List

### ICM-20948 Pin Connections

VDD  - Pin 1  (3.3V)
GND  - Pin 6  (GND)
SDA  - Pin 3  (I2C SDA, GPIO 2)
SCL  - Pin 5  (I2C SCL, GPIO 3)
INT  - Pin 13 (GPIO 27, data-ready interrupt)
AD0  - GND    (sets I2C address to 0x69)


## I2C Configuration

I2C Bus     : 1
I2C Address : 0x69 (AD0 tied to GND)


## Sensor Settings

Accelerometer range : ±2G
Gyroscope range     : ±250 DPS
Magnetometer mode   : 100Hz continuous
DLPF                : enabled


## Data-Ready Interrupt

The ICM-20948 asserts a rising edge on the INT pin when a new sample is ready. This is wired to GPIO 27 on the Raspberry Pi.

## Hardware Assembly

[Insert photograph — IMU mounted on bat]

[Insert photograph — wiring connections]

[Insert photograph — 3D printed enclosure]
