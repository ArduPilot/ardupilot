# ModalAI Flight Core v1 Controller

The ModalAI FlightCore is a flight controller produced by [ModalAI](http://www.modalai.com/).

## Features

 - STM32F765 microcontroller
 - ICM42688 and ICM20602 IMUs
 - BMP388 barometer
 - microSD card slot
 - 7 UARTs plus USB
 - 8 PWM outputs

## Pinout

![ModalAI_v1 Board](fc-overlay-top-144-dpi.jpg "ModalAI_v1")

For detailed pinout descriptions see [FlightCore Pinout](https://docs.modalai.com/flight-core-datasheets-connectors/)

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.

 - SERIAL0 -> USB
 - SERIAL1 -> UART7 (Telem1)
 - SERIAL2 -> UART5 (Telem2)
 - SERIAL3 -> USART1 (GPS)
 - SERIAL4 -> UART4 (GPS2)
 - SERIAL5 -> USART2
 - SERIAL6 -> USART6 (spektrum RCIN)
 - SERIAL7 -> USART3
 - SERIAL8 -> USB2

## RC Input

RC input is configured on both the PPM input pin and the "spektrum"
USART6 UART. The PPM pin supports all one-way RC protocols. For
protocols requiring half-duplex serial to transmit telemetry (such as
FPort) you should use the spektrum port, mapped to SERIAL6. Both PPM
and spektrum ports are enabled for RCIN by default.

## PWM Output

The ModalAI_v1 supports up to 8 PWM outputs on the PWM output connector

The PWM is in 2 groups:

 - PWM 1, 2, 3, 4 in group1
 - PWM 5, 6, 7, 8 in group2

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot.

## Power Monitoring

In addition to the normal range of ArduPilot power monitoring options,
the modalAI build supports two I2C power monitors, the INA231 and the
LTC2946. The FlightCore board comes with the INA231 and you should set
BATTERY_MONITOR to 21. For the LTC2946 based power brick you should
set BATTERY_MONITOR to 22.

## Compass

The ModalAI_v1 does not have a built-in compass, but you can attach an
external compass

