# Brahma H7 Flight Controller

The Brahma H7 is a flight controller featuring a powerful H7 processor and dual IMUs.

## Features

 - **MCU:** STM32H743VIH6 microcontroller running at 480MHz
 - **Flash:** 2MB
 - **Sensors:** Dual ICM42688-P IMUs, DPS310 barometer
 - **Power:** 2S-6S LiPo input voltage
 - **BEC:** 5V 3A for peripherals, 10V 3A for Video (User switchable)
 - **OSD:** AT7456E analog OSD
 - **Interfaces:** 
    - 6x UARTs plus USB
    - 12x Motor PWM outputs
    - 1x RC Input
    - 1x I2C port
    - MicroSD card slot
    - CAN port

## Pinout

![Brahma H7 Top](Top.png "Brahma H7 Top")
![Brahma H7 Bottom](Bottom.png "Brahma H7 Bottom")

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.

 - SERIAL0 -> USB
 - SERIAL1 -> USART1 (Telem2)
 - SERIAL2 -> USART2 (GPS1)
 - SERIAL3 -> USART3 (GPS2)
 - SERIAL4 -> UART4 (ESC Telemetry)
 - SERIAL6 -> USART6 (RC Input, DMA-enabled)
 - SERIAL7 -> UART7 (Telem1)
 - SERIAL8 -> UART8 (Spare)

## RC Input

RC input is configured by default via the USART6 RX input (SERIAL6). It supports all serial RC protocols.

* For FPort the receiver must be tied to the USART6 TX pin, and :ref:`SERIAL6_OPTIONS<SERIAL6_OPTIONS>` set to "7" (invert TX/RX, half duplex).
* For full duplex CRSF/ELRS use both TX and RX on USART6, and set :ref:`SERIAL6_PROTOCOL<SERIAL6_PROTOCOL>` to 23.

## FrSky Telemetry

FrSky Telemetry is supported using an unused UART, such as the Tx pin of UART4 (SERIAL4).
You need to set the following parameters to enable support for FrSky S.PORT:

 - :ref:`SERIAL4_PROTOCOL<SERIAL4_PROTOCOL>` 10
 - :ref:`SERIAL4_OPTIONS<SERIAL4_OPTIONS>` 7

## OSD Support

The Brahma H7 supports analog OSD using its onboard MAX7456.
External MSP DisplayPort OSDs (like DJI or Walksnail) are supported on any spare UART.

## PWM Output

The Brahma H7 supports up to 13 PWM outputs.
Outputs 1-10 support bi-directional DShot.
Output 13 is dedicated for LED strip (Serial LED by default).

The PWM is in 4 groups:

 - PWM 1-2 (TIM3)
 - PWM 3-6 (TIM5)
 - PWM 7-10 (TIM4)
 - PWM 11-12 (TIM15)

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot.

## Battery Monitoring

The board has internal voltage and current sensors.

The default battery parameters are:

 - :ref:`BATT_MONITOR<BATT_MONITOR>` 4
 - :ref:`BATT_VOLT_PIN<BATT_VOLT_PIN__AP_BattMonitor_Analog>` 10
 - :ref:`BATT_CURR_PIN<BATT_CURR_PIN__AP_BattMonitor_Analog>` 11
 - :ref:`BATT_VOLT_MULT<BATT_VOLT_MULT__AP_BattMonitor_Analog>` 11.0
 - :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT__AP_BattMonitor_Analog>` 78.4

## Compass

The Brahma H7 does not have a builtin compass, but you can attach an external compass using I2C on the SDA and SCL pads.

## VTX Power

GPIO 81 controls the VTX power. It is high by default (On).

## Loading Firmware

The Brahma H7 does not come with ArduPilot firmware pre-installed. Use the instructions here to load ArduPilot the first time :ref:`common-loading-firmware-onto-chibios-only-boards`.
Firmware for the Brahma H7 can be found `here <https://firmware.ardupilot.org>`_ in sub-folders labeled "BrahmaH7".

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*.apj firmware files.
