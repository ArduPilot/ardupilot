# 3DR Control N1

The Control N1 is a flight controller produced by [3DR](https://store.3dr.com/control-n1/), launched on October 2025.

![3DR Control N1](https://vddwxegfxugwzpfnrrlp.supabase.co/storage/v1/object/public/Website-CDN/autopilot-img/control_n1.png)

## Features

- Processor - STM32H743 32-bit Arm Cortex-M7 core, double-precision FPU
- Sensors 
    - 2x IIM42653 Accel, Gyro 
    - AK09940A Magnetometer 
    - DPS368 Barometer
- Power
    - External Power Supply
    - Logic level IO at 3.3V
- Interfaces
    - 12x PWM / IO 
    - 4x GPIO
    - 7x UARTs (3x w/hardware flow control)
    - 2x CAN
    - 1x SPI
    - 2x I2C
    - uSD card socket/SDMMC broken out to Connectors
    - Serial Wire Debug (SWD)
- Memory
    - 1Mb F-RAM
- Miscellaneous
    - Onboard 3 color LED (WS2812B Type)
    - Mini Buzzer
- Connectors
    - 2x Front DF40 30-pin plug type connector
    - 1x rear DF40 80-pin plug type connector.

### Physical parameters

- Weight: 2.7 g \| _0.095 oz_
- Width: 17.6 mm \| _0.69 in_
- Length: 28 mm \| _1.1 in_
- Height: 3.8 mm \| _0.15 in_

### 2D drawing

![CN1 2D](https://vddwxegfxugwzpfnrrlp.supabase.co/storage/v1/object/public/Website-CDN/autopilot-img/cn1/R0024A.2_bot_drawing.png)

A PDF version of the above with more details is available [here](https://vddwxegfxugwzpfnrrlp.supabase.co/storage/v1/object/public/Website-CDN/autopilot-img/cn1/ControlN1_2D-DWG.pdf)

## Changelog

### R0024A.1 Initial release

## Pinout

3DR Control N1 footprint has 3 Hirose DF40 series connectors with the following characteristics:

| Reference | Pin count | Part number               |
| --------- | --------- | ------------------------- |
| J100      | 30        | DF40HC(3.5)-30DS-0.4V(51) |
| J200      | 30        | DF40HC(3.5)-30DS-0.4V(51) |
| J300      | 80        | DF40HC(3.5)-80DS-0.4V(51) |

The pinout for each connector is as follows:

**J100**

| Group | Name      | Pin | Pin | Name       | Group |
| ----- | --------- | --- | --- | ---------- | ----- |
| -     | RESERVED  | 1   | 2   | USART2_TX  | SG1   |
| USB   | USB_HS_DM | 3   | 4   | USART2_RX  | SG1   |
| USB   | USB_HS_DP | 5   | 6   | USART2_CTS | SG1   |
| POWER | GND       | 7   | 8   | USART2_RTS | SG1   |
| SDMMC | SD_DET    | 9   | 10  | GND        | POWER |
| SDMMC | SDIO_D1   | 11  | 12  | UART4_TX   | SG2   |
| SDMMC | SDIO_D0   | 13  | 14  | UART4_RX   | SG2   |
| SDMMC | SDIO_CK   | 15  | 16  | UART4_CTS  | SG2   |
| POWER | GND       | 17  | 18  | UART4_RTS  | SG2   |
| SDMMC | SDIO_CMD  | 19  | 20  | GND        | POWER |
| SDMMC | SDIO_D3   | 21  | 22  | USART3_TX  | SG4   |
| SDMMC | SDIO_D2   | 23  | 24  | USART3_RX  | SG4   |
| POWER | GND       | 25  | 26  | GND        | POWER |
| MISC  | ALARM     | 27  | 28  | USART1_TX  | SG5   |
| POWER | GND       | 29  | 30  | USART1_RX  | SG5   |

**J200**

| Group | Name      | Pin | Pin | Name      | Group |
| ----- | --------- | --- | --- | --------- | ----- |
| SWD   | SWCLK     | 1   | 2   | USART6_TX | SG6   |
| SWD   | SWDIO     | 3   | 4   | USART6_RX | SG6   |
| POWER | GND       | 5   | 6   | RESERVED  | -     |
| -     | AP_nRST   | 7   | 8   | RESERVED  | -     |
| -     | BOOT0     | 9   | 10  | GND       | POWER |
| POWER | GND       | 11  | 12  | RESERVED  | -     |
| FDCAN | FDCAN1_5V | 13  | 14  | RESERVED  | -     |
| FDCAN | FDCAN1_H  | 15  | 16  | GND       | POWER |
| FDCAN | FDCAN1_L  | 17  | 18  | ADDR_LED  | MISC  |
| POWER | GND       | 19  | 20  | RESERVED  | -     |
| -     | BRD_EN    | 21  | 22  | SPI_SCK   | SPI   |
| POWER | 5V_PWRIN1 | 23  | 24  | SPI_SDI   | SPI   |
| POWER | 5V_PWRIN1 | 25  | 26  | SPI_SDO   | SPI   |
| POWER | GND       | 27  | 28  | SPI_nCS   | SPI   |
| POWER | GND       | 29  | 30  | RESERVED  | -     |

**J300**

| Group | Name      | Pin | Pin | Name      | Group |
| ----- | --------- | --- | --- | --------- | ----- |
| POWER | GND       | 1   | 2   | GND       | POWER |
| FDCAN | FDCAN2_L  | 3   | 4   | GND       | POWER |
| FDCAN | FDCAN2_H  | 5   | 6   | 5V_PWRIN2 | POWER |
| FDCAN | FDCAN2_5V | 7   | 8   | 5V_PWRIN2 | POWER |
| POWER | GND       | 9   | 10  | 3V3_FC    | POWER |
| -     | RESERVED  | 11  | 12  | RESERVED  | -     |
| -     | RESERVED  | 13  | 14  | RESERVED  | -     |
| POWER | GND       | 15  | 16  | RESERVED  | -     |
| -     | RESERVED  | 17  | 18  | RESERVED  | -     |
| -     | RESERVED  | 19  | 20  | GND       | POWER |
| POWER | GND       | 21  | 22  | RESERVED  | -     |
| I2C   | I2C2_SDA  | 23  | 24  | RESERVED  | -     |
| I2C   | I2C2_SCL  | 25  | 26  | GND       | -     |
| POWER | GND       | 27  | 28  | ADC2      | ADC   |
| I2C   | I2C1_SDA  | 29  | 30  | ADC1      | ADC   |
| I2C   | I2C1_SCL  | 31  | 32  | GND       | POWER |
| POWER | GND       | 33  | 34  | RESERVED  | -     |
| SG7   | UART5_RX  | 35  | 36  | RESERVED  | -     |
| SG7   | UART5_TX  | 37  | 38  | RESERVED  | -     |
| POWER | GND       | 39  | 40  | RESERVED  | -     |
| SG3   | UART7_RTS | 41  | 42  | GND       | POWER |
| SG3   | UART7_CTS | 43  | 44  | GP4       | GPIO  |
| SG3   | UART7_RX  | 45  | 46  | GP3       | GPIO  |
| SG3   | UART7_TX  | 47  | 48  | GP2       | GPIO  |
| -     | RESERVED  | 49  | 50  | GP1       | GPIO  |
| -     | RESERVED  | 51  | 52  | GND       | POWER |
| POWER | GND       | 53  | 54  | CH12      | GPIO  |
| -     | RESERVED  | 55  | 56  | CH11      | GPIO  |
| POWER | GND       | 57  | 58  | CH10      | GPIO  |
| -     | RESERVED  | 59  | 60  | CH9       | GPIO  |
| -     | RESERVED  | 61  | 62  | GND       | POWER |
| -     | RESERVED  | 63  | 64  | CH8       | GPIO  |
| POWER | GND       | 65  | 66  | CH7       | GPIO  |
| -     | RESERVED  | 67  | 68  | CH6       | GPIO  |
| POWER | GND       | 69  | 70  | CH5       | GPIO  |
| -     | RESERVED  | 71  | 72  | GND       | POWER |
| -     | RESERVED  | 73  | 74  | CH4       | GPIO  |
| POWER | GND       | 75  | 76  | CH3       | GPIO  |
| -     | RESERVED  | 77  | 78  | CH2       | GPIO  |
| -     | RESERVED  | 79  | 80  | CH1       | GPIO  |

## UART Mapping

| Port | UART   | Protocol | TX DMA | RX DMA | Flow control |
| ---- | ------ | -------- | ------ | ------ | ------------ |
| 0    | USB    | MAVLink2 | ✘      | ✘      | N/A          |
| 1    | USART2 | MAVLink2 | ✔     | ✔     | ✔           |
| 2    | UART4  | MAVLink2 | ✔     | ✔     | ✔           |
| 3    | UART8  | GPS      | ✔     | ✔     | ✘            |
| 4    | USART3 | GPS      | ✔     | ✔     | ✘            |
| 5    | USART6 | None     | ✔     | ✔     | ✘            |
| 6    | UART7  | None     | ✔     | ✔     | ✔           |
| 7    | USART1 | None     | ✔     | ✔     | ✘            |

## Analog Inputs

The Control N1 has 2 user ADC inputs:

- ADC1 Pin18 -> Battery Voltage
- ADC1 Pin10 -> Battery Current

And one internal ADC input:

- ADC1 Pin8 -> Vcc (5V)

## PWM Output

The Control N1 supports 13 PWM outputs, output 13 is defaulted to serial LED and not available otherwise.
All outputs are Bidirectional DShot capable. 

The PWM outputs are distributed in 3 groups:

- PWM 1-4 in group 1
- PWM 5-8 in group 2
- PWM 9-12 in group 4

Channels within the same group must use only one output rate. If any channel is using DShot or BiDirDShot the rest of the group will use the said output type.

## Addressable LED

The RGB LED onboard is a WS2812B addressable type LED, connected to PWM channel 13 (TIM3_CH4).
The LED's **data out** pin is exposed as ADDR_LED, so it can be used for external addressable LEDs.

## Power Supply

This board requires a 5V, 500mA power supply.

## Battery Monitoring

Analog voltage and current or I2C sensors may be used. The following settings need to be set to work with a Power Zero Module (M10077):

| Parameter name    | Value |
| ----------------- | ----- |
| `BATT_MONITOR`    | 4     |
| `BATT_VOLT_PIN`   | 18    |
| `BATT_CURR_PIN`   | 10    |
| `BATT_VOLT_MULT`  | 15.3  |
| `BATT_AMP_PERVLT` | 50.0  |

_Other Power Modules need to be adjusted accordingly_

## Onboard compass

The Control N1 has an onboard compass, connected to I2C bus 1. 
Due to potential interference, it is recommended to use an external compass module. 
Hence, the internal compass is disabled by default.

## Build

Binaries are available at [https://firmware.ardupilot.org](https://firmware.ardupilot.org) in sub-folders labeled as `3DRControlN1` within each vehicle type.

However, if you want to build the firmware yourself, you need to have the necessary tools installed and follow the instructions in the ArduPilot Development documentation.

`./waf configure --board=3DRControlN1`

`./waf copter`

The compiled binary will be located in `build/3DRControlN1/bin/arducopter.apj`

## Uploading Firmware

Any Control N1 has a preloaded Ardupilot bootloader, which allows the user to use a compatible Ground Station software to upload the `.apj` file.
