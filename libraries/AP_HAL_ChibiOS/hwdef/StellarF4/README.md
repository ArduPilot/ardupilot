# StellarF4 Flight Controller

https://stingbee.com.ua/flight_controllers/stellarf4


## Features
    Processor
        STM32F405
    Sensors
        ICM-42688p/BMI270 Acc/Gyro
        DPS310/BMP280 barometer
        AT7456E OSD
        W25Q128 dataflash
    Power
        2S-6S Lipo input voltage with voltage monitoring
        12V, 3A BEC for powering Video Transmitter
        5V, 2A BEC for internal and peripherals
    Interfaces
        10x PWM outputs DShot capable, 4 PWM outputs BDShot capable
        6x UARTs
        1x I2C
        2x ADC
        SPI flash for logging
        USB-C port
    LED
        Red, 3.3V power indicator
        Green, FC status
    Size
        41 x 41mm PCB with 30.5mm M3 mounting

  
## Overview

![StellarF4](StellarF4-top.png)

![StellarF4](StellarF4-bot.png)

## Wiring Diagram


## UART Mapping

The UARTs are marked Rx* and Tx* in the above pinouts. The Rx* pin is the
receive pin for UART*. The Tx* pin is the transmit pin for UART*. The UARTs 1,2,6 are DMA capable.

 - SERIAL0 -> USB
 - SERIAL1 -> USART1 (DJI / VTX, DMA capable)
 - SERIAL2 -> USART2 (Serial RC input, DMA capable)
 - SERIAL3 -> USART3 (User) (NO DMA)
 - SERIAL4 -> UART4  (User) (NO DMA)
 - SERIAL5 -> UART5  (ESC Telemetry) (NO DMA)
 - SERIAL6 -> USART6 (GPS) (NO DMA)


## CAN and I2C

StellarF4 supports I2C bus
multiple I2C peripherals can be connected to one I2C bus in parallel.


## RC Input

The default RC input is configured on the UART2 RX2 input and can be used for all ArduPilot supported unidirectional receiver protocols.
* SBUS/DSM/SRXL connects to the PPM pad or RX2 pin on the HD VTX connector. PPM pin connected to RX2 via inverter.
* CRSF also requires a TX2 connection, in addition to RX2, and automatically provides telemetry.
* FPort requires connection to TX2 and :ref:`SERIAL2_OPTIONS<SERIAL2_OPTIONS>` set to "7". See :ref:`common-FPort-receivers`.


## OSD Support

StellarF4 supports using its internal OSD, and/or DisplayPort on Serial1, by default. See :ref:`common-msp-osd-overview-4.2` for more info.

## PWM Output

StellarF4 supports up to 10 PWM outputs. All outputs support DShot. First 4 outputs support BDShot. Channels 1-8 marked as M1-M8 on the board. Channels 9 and 10 are marked as S1 and S2 on the board. PWM outputs are grouped and every group must use the same output protocol:
* 1, 2, 3, 4 are Group 1;
* 5, 6, 7, 8 are Group 2;
* 9          are Group 3;
* 10         are Group 4;


## Battery Monitoring

The board has 1 built-in voltage dividers and 1x current ADC. support external 3.3V based current sensor.
The voltage input is compatible with 2~6S LiPo batteries.

The default battery parameters are:

* :ref:`BATT_MONITOR<BATT_MONITOR>` = 4
* :ref:`BATT_VOLT_PIN<BATT_VOLT_PIN__AP_BattMonitor_Analog>` = 10
* :ref:`BATT_CURR_PIN<BATT_CURR_PIN__AP_BattMonitor_Analog>` = 11 (CURR pin)
* :ref:`BATT_VOLT_MULT<BATT_VOLT_MULT__AP_BattMonitor_Analog>` = 11.2
* :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT__AP_BattMonitor_Analog>` = 52.7


## Compass

StellarF4 does not have a built-in compass, but you can attach an external compass using I2C on the SDA and SCL pads.


## Camera Switch

GPIO 81 controls which camera input (CAM1 or CAM2) is applied to the internal OSD.
The camera switch is controlled by the RELAY1 parameter. The default is to use CAM1.


## Loading Firmware

Firmware for these boards can be found at https://firmware.ardupilot.org in sub-folders labeled StellarH7V2.

Initial firmware load can be done with DFU by plugging in USB with the
boot button pressed. Then you should load the "ardu*_with_bl.hex" firmware, using your favourite DFU loading tool. eg STM32CubeProgrammer

Subsequently, you can update firmware with Mission Planner.


