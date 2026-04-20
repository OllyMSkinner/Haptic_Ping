## ADS1115 Pin Connection List
| ADC Connections | GPIO Pin | Physical Pin |
|---|---|---|
| VDD | 3.3v | 17 |
| SDA | GPIO 2 | 3 |
| SCL | GPIO 3 | 5 |
| A0 | Positive Piezo | N/A |
| ADDR | GND | 6 |
| GND | GND | 6 |
| ALRT | GPIO 26 | 37 |


## Piezo Pin Connection 
| Piezo | Connections |
|---|---|
| Positive | A0 |
| Negative | GND |
| Both | 330&Omega; Resistor |


## ADS1115 Settings
- I2C bus: 1
- I2C address: 0x48
- Sampling rate: 128Hz
- Gain/full-scale range: +/-2.048V


## Piezo Event Detector Settings
- Press threshold: 0.95V
- Release threshold: 0.70V


## Hardware Assembly
<img width="1536" height="2048" alt="bat with piezo image" src="https://github.com/user-attachments/assets/905d2ba3-e871-4899-8160-cf823f02477e" />
<img width="1536" height="2048" alt="ads1115 front view" src="https://github.com/user-attachments/assets/44e4d61b-a571-47fa-87b1-4d7ba9cb8323" />



## Acknowledgements 
The ADS1115rpi ADC was adapted from the implementation, rpi_ads1115, written by the Bernd Porr (University of Glasgow, 2025), available at: [(https://github.com/berndporr/rpi_ads1115)]
