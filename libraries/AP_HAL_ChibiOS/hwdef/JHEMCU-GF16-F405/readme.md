# JHEMCU GF16 F405 Flight Controller

The JHEMCU GF16 F405 is a flight controller produced by [JHEMCU](https://www.jhemcu.com/).

## Features 

 - MCU - STM32F405 
 - IMU - MPU6000 or ICM42688 or BMI270 depending of the version of the board
 - Barometer - BMP280 or SPL06
 - OSD - AT7456E
 - 9x PWM Outputs (8 Motor Output, 1 LED)
 - Battery input voltage: 2S-4S
 - Black box: 16MB
 - IIC: Support
 - BEC: 5V/3A
 - UART: UART1 (RC), UART2, UART3, UART4, UART6
 - USB: micro usb
 - Size: 16*16MM M3
 - Receiver: ELRS (CRSF), TBS (CRSF), SBUS, IBUS, DSM2, DSMX
 - Support programmable LED such as WS2812
 - Support buzzer
 - Built-in voltage and current sensors
 

 ## Pinout

 ![JHEMCU GF16 F405](pinout.jpg "JHEMCUGF16F405")

## RC Input

RC input is configured on the R1 (UART1_RX) pin. It supports ELRS (CRSF), TBS (CRSF), SBUS, IBUS, DSM2, DSMX.


## OSD Support

The JHEMCU GF16 F405 supports OSD using OSD_TYPE 1 (MAX7456 driver).

 ## Compass

The board does not have a builtin compass, but you can attach an external compass using I2C on the SDA and SCL pads.

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*.apj firmware files.
