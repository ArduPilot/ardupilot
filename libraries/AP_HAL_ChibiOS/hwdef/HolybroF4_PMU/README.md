# Holybro DroneCAN PMU

https://holybro.com/products/dronecan-pm08-power-module-14s-200a

## Features

- Processor£ºSTM32F405RG 168MHz 1024KB Flash  196KB RAM
- Voltage input: 7~60.9V (2S~14S) 
- Continuous current:200A
- Burst current 400A @ 25¡æ 1 sec£¬1000A @ 25¡æ < 1 sec
- Max current sensing: 376A
- Voltage accuracy: ¡À0.1V 
- Current accuracy: ¡À5%
- Temperature accuracy:¡À1¡æ
- Power port output: 5.3V/3A each port
- Protocol: DroneCAN 
- Operating temperature :-25¡æ~105¡æ
- Firmware upgrade: Support
- Calibration:  Support

## Interface Type

- Power & CAN Port: Molex CLIK-Mate 2mm 6Pin
- Battery IN/OUT Options:
     --XT90 Connectors
     --Tinned Wires
     --XT90 & Ring Terminals

## Status LED

- Flashing quickly continuously: MCU is in the bootloader stage, waiting for firmware to be flashed
- Flashing quickly for 3 seconds, and then on for 1 second: Waiting for CAN connection
- Flashing slowly (one-second intervals): CAN is successfully connected

## Mechanical Spec

- Size: 45mm¡Á41mm¡Á26mm (not include cable)
- Weight: 185g (include cable)

## Loading Firmware

You can upgrade the *.bin firmware files using the DroneCan GUI tool. *.apj files can also be upgraded using mossionplanner ground station.
