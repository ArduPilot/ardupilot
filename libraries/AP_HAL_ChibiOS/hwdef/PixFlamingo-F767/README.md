# PixFlamingo-F767 Flight Controller

The PixFlamingo-F767 is a flight controller produced by Dheeran Labs

## Features
    Processor
        STM32F767 32-bit processor
    Sensors
        ICM42670, MPU6500 Acc/Gyro
        MS5611 / BMP280 barometer
	LIS3MDL Compass
    Power
        5v input voltage with voltage monitoring
    Interfaces
        10x PWM outputs 
        1x RC input
        5x UARTs/serial for GPS and other peripherals
        2x I2C ports for external compass, airspeed, etc.
        USB-C port

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.

 - SERIAL0 -> USB
 - SERIAL1 -> UART3 (TELEM1) with CTS/RTS DMA Enabled
 - SERIAL2 -> UART6 (TELEM2) with CTS/RTS DMA Enabled
 - SERIAL3 -> UART1 (GPS) DMA Enabled
 - SERIAL4 -> EMPTY
 - SERIAL5 -> UART7 (GPS2)
 - SERIAL6 -> USART2 (User) DMA Enabled

## RC Input

Supports I-Bus/S-bus

## PWM Output

The PixFlaminog-F767 supports up to 10 PWM outputs. 

The PWM is in 5 groups:

 - PWM 1-4 in group1
 - PWM 5-8 in group2
 - PWM 9 in group3
 - PWM 10 in group4

