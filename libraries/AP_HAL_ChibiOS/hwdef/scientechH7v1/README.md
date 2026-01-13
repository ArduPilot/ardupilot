# ScientechH7v1-String Flight Controller

The ScientechH7v1-String is a flight controller produced by [Scientech Technology](https://scientechworld.com/). 

## Features

- MCU：STM32H743VIH6
- Gyro：ICM42688P x2
- Baro：DPS310
- Blackbox：SD CARD
- PWM output：12CH
- UART：7CH
- Power Supply：3-6SLipo
- BEC Output：5V/2.5A, 12V/3A
- USB Connector: Type-C
- Weight：24.4g
- Size：60mm x 40mm
- Mounting Hole：30.5mm x 30.5mm

## Pinout


![ScientechH7v1 Top](01.jpg "ScientechH7v1 Top")
![ScientechH7v1 Bottom](02.jpg "ScientechH7v1 Bottom")


## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the receive pin for UARTn. The Tn pin is the transmit pin for UARTn.

 - SERIAL0 -> USB (primary mavlink, usually USB)
 - SERIAL1 -> UART7 (GPS)
 - SERIAL2 -> UART1 (ESC telemetry ) 
 - SERIAL3 -> UART2 (SBUS )
 - SERIAL4 -> UART3 (None)
 - SERIAL5 -> UART8 (ELRS/CRSF receiver)
 - SERIAL6 -> UART4 (VTX)
 - SERIAL7 -> UART6 (None)

## RC Input

RC input is configured on SERIAL5 (UART8), which is available on the Rx1, Tx1. PPM receivers are *not* supported as this input does not have a timer resource available. 

*Note* It is recommend to use CRSF/ELRS. 

With recommended option:

- Set SERIAL1_PROTOCOL<SERIAL1_PROTOCOL must be set to "23"
- Set SERIAL1_OPTIONS<SERIAL1_OPTIONS to "0".
  
## OSD Support

The ScientechH7v1 supports OSD using OSD_TYPE 1 (MAX7456 driver).

## PWM Output

The ScientechH7v1 supports up to 12 PWM outputs. 


## Battery Monitoring

The board has a built-in voltage and current sensor. The current sensor can read up to 130 Amps. The voltage sensor can handle from 3S to 6S LiPo batteries.

The correct battery setting parameters are:

 - BATT_MONITOR_DEFAULT 4
 - BATT_VOLT_PIN 10
 - BATT_CURR_PIN 11
 - BATT_VOLT_SCALE 11.0
 - BATT_CURR_SCALE 40.0

## Compass

The ScientechH7v1 does not have a built-in compass, but you can attach an external compass using I2C on the SDA and SCL pads.

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the bootloader button pressed. Then you should load the "with_bl.hex" firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using any ArduPilot ground station software. Updates should be done with the *.apj firmware files.
