


# Ardupilot port to the esp32 series mcu



## Building instructions
0. Build currently tested on linux
1. Install toolchain and esp-idf from espressif https://docs.espressif.com/projects/esp-idf/en/latest/get-started/
Please use lastest stable version of the esp-idf (https://github.com/espressif/esp-idf/tree/release/v3.3)
2. Checkout this branch https://github.com/sh83/ardupilot/tree/esp32
3. Use script Tools/scripts/install-prereqs-* to install ardupilot requirements
4. Configure and run build:
```bash
./waf configure --board=esp32diy
./waf plane
or
./waf copter
```
5. To flash binary use espressif flash tool via `make flash` inside `libraries/AP_HAL_ESP32/targets/plane/` directory. Also other make targets are avaliable (`make monitor` , `make size` and so on) 

## Test hardware
Currently esp32 dev board with connected gy-91 10dof sensor board is supported. Pinout (consult UARTDriver.cpp and SPIDriver.cpp for reference):
### Uart connecion
Internally connected on most devboards, just for reference

| ESP32 | CP2102 |
| --- | --- |
| GPIO3 | UART_TX |
| GPIO1 | UART_RX |


### GY-91 connection
|ESP32|GY-91|
|---|---|
|GND|GND|
|3.3V|3V3|
|IO5|NCS|
|IO23|SDA|
|IO19|SDO/SAO|
|IO18|SCL|
|IO26|CSB|


## Current progress
### Main tasks
- [x] Build system
- [x] Scheduler and semaphores
- [x] SPI driver
- [x] WiFi driver
- [x] Uart driver
- [ ] RCIN driver
- [x] I2C driver
- [ ] PWM driver
- [x] Storage
- [x] OTA update of the fw
- [x] SdCard
- [ ] Buzzer
- [x] Custom boards build
- [x] Perfomance optimization

### Future development
- [ ] Pin remapping via parameters
- [ ] GPIO driver
- [ ] AnalogIn driver
- [ ] DShot driver / 4way pass / esc telemetry / ws2812b led
- [ ] INA219 driver
- [ ] GSD
