# ESP32 Environment Sentinel

Initial prototype/experimentation writing ESP32 firmware using ESP-IDF + FreeRTOS. 
The goal is to have a simple, low-power, self-contained environment sensor that can be used to monitor the environment and eventually air quality of a room.

Once I get a reasonable prototype completed, I might design a PCB for it (for the love of the game, hahah).

## Hardware (so far)

- ESP32-DevKitC-32D
- BH1750 I2C light sensor module (ambient light, lux)
- BME280 I2C environmental sensor module (temperature, humidity, pressure)
- INMP441 I2C microphone module (noise, dB)

### On the way

- SGP40 / SGP41 I2C air quality sensor module (VOC, TVOC)
- SPS30 PM2.5 / PM10 sensor module (particles, ug/m3) (order cancelled, hopefully can find a replacement)
- SCD41 I2C CO2 sensor module (CO2, ppm) (the one I received was defective, will see if I can find one but they are elusive)

## Software

- ESP-IDF
- FreeRTOS