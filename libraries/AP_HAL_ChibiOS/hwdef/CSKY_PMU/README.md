# CSKY GaN DroneCAN PMU

## Features

- Processor: STM32F412 100MHz 1024KB Flash 256KB RAM
- Voltage input: 4S-14S (HV)
- Continuous current: 200A
- Burst current: 300A @ 25℃ 1 sec
- Max current sensing: 200A
- Voltage accuracy: ±0.05V
- Current accuracy: ±0.7%
- Temperature accuracy: ±1℃
- Power port 1 output (Molex): 5V/3A
- Power port 2 output (XT30): 12V/10A
- Efficiency: 97% @10A
- Protocol: DroneCAN
- Operating temperature: -25℃~105℃
- CAN Firmware updade: Support
- Calibration: No need

## Interface Type

- Power 1 & CAN Port: Molex CLIK-Mate 2mm 6Pin
- Power 2 Port: XT30PW-F
- Battery IN/OUT Options: Tinned Wires

## Status LED

- Flashing quickly continuously: MCU is in the bootloader stage, waiting for firmware to be flashed
- Flashing quickly for 3 seconds, and then on for 1 second: Waiting for CAN connection
- Flashing slowly (one-second intervals): CAN is successfully connected

## Mechanical Spec

- Size: 57.5mm×57.5mm×17.1mm (not include cable)
- Weight: 140g (include cable)

## Loading Firmware

You can upgrade the *.bin firmware files using the DroneCan GUI tool. *.apj files can also be upgraded using Mission Planner ground station.
