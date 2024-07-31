# Holybro DroneCAN PMU

https://holybro.com/products/dronecan-pm08-power-module-14s-200a

## Features

- Processor��STM32F405RG 168MHz 1024KB Flash  196KB RAM
- Voltage input: 7~60.9V (2S~14S) 
- Continuous current:200A
- Burst current 400A @ 25�� 1 sec��1000A @ 25�� < 1 sec
- Max current sensing: 376A
- Voltage accuracy: ��0.1V 
- Current accuracy: ��5%
- Temperature accuracy:��1��
- Power port output: 5.3V/3A each port
- Protocol: DroneCAN 
- Operating temperature :-25��~105��
- Firmware upgrade: Support
- Calibration:  Support

## Interface Type

- Power & CAN Port: Molex CLIK-Mate 2mm 6Pin
- Battery IN/OUT Options:
     --XT90 Connectors
     --Tinned Wires
     --XT90 & Ring Terminals

## Status LED

- Flashing quickly continuously: MCU is in the bootloader stage, waiting for firmware to be flash
- Flashing quickly for 3 seconds, and then on for 1 second: Waiting for CAN connection
- Flashing slowly (one-second intervals): CAN is successfully connected

## Mechanical Spec

- Size: 45mm��41mm��26mm (Not Include Cable)
- Weight: 185g (Include Cable)

## Loading Firmware

You can upgraded the *.bin firmware files using the dronecan GUI tool. *.apj files can also be upgraded using mossionplanner ground station.