#Hardware

## Load device tree
To load the BBBMINI device tree type `startup.sh load`.

## Build dtbo file
To rebuild the dtbo file type `make` and than `make install`to copy the dtbo file to `/lib/firmware`.

## Pin assigment

### RCInput
BBB | RC Receiver | Remark 
------------ | ------------- | ------------- 
P8.15 RC_IN | RC Out (Spektrum / PPM-sum)

### RCOutput 
BBB | ESC / Servo | Remark
------------ | ------------- | ------------- 
P8.28 | RC_OUT_CH_1 | Use level shifter 3.3 Volt to 5 Volt
P8.27 | RC_OUT_CH_2 | Use level shifter 3.3 Volt to 5 Volt
P8.30 | RC_OUT_CH_3 | Use level shifter 3.3 Volt to 5 Volt
P8.29 | RC_OUT_CH_4 | Use level shifter 3.3 Volt to 5 Volt
P8.40 | RC_OUT_CH_5 | Use level shifter 3.3 Volt to 5 Volt
P8.39 | RC_OUT_CH_6 | Use level shifter 3.3 Volt to 5 Volt
P8.42 | RC_OUT_CH_7 | Use level shifter 3.3 Volt to 5 Volt
P8.41 | RC_OUT_CH_8 | Use level shifter 3.3 Volt to 5 Volt
P8.44 | RC_OUT_CH_9 | Use level shifter 3.3 Volt to 5 Volt
P8.43 | RC_OUT_CH_10 | Use level shifter 3.3 Volt to 5 Volt
P8.46 | RC_OUT_CH_11 | Use level shifter 3.3 Volt to 5 Volt
P8.45 | RC_OUT_CH_12 | Use level shifter 3.3 Volt to 5 Volt

### SPI0
BBB | Sensor | Remark
------------ | ------------- | ------------- 
P9.22 | SPI0_SCLK | not used
P9.21 | SPI0_MISO | not used
P9.18 | SPI0_MOSI | not used
P9.17 | SPI0_CS | not used

### SPI1
BBB | Sensor | Remark
------------ | ------------- | ------------- 
P9.31 | SPI1_SCLK | MPU9250, MS5611
P9.29 | SPI1_MISO | MPU9250, MS5611
P9.30 | SPI1_MOSI | MPU9250, MS5611
P9.28 | SPI1_CS0 | MPU9250 CS
P9.19 | SPI1_CS1 | MS5611 CS

### UART4
BBB | Sensor | Remark
------------ | ------------- | ------------- 
P9.11 UART4_RXD | not used 
P9.13 UART4_TXD | not used

### UART5 GPS

38400,8,n,1

BBB | Sensor | Remark
------------ | ------------- | ------------- 
P8.38 UART5_RXD | GPS_TXD | 
P8.37 UART5_TXD | GPS_RXD |

### Sound (PWM)

BBB | Speaker | Remark
------------ | ------------- | ------------- 
P8.36 | PWM | Use a power amplifier
