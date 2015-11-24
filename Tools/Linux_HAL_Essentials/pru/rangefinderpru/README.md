# PRU firmware for HC-SR04 rangefinder

HC-SR04 driver that can be used with BeagleBone Black

## Install PRU C-Compiler
1. `sudo apt-get update`
2. `sudo apt-get install ti-pru-cgt-installer`

## Rebuild rangefinder.c
1. `cd ardupilot/Tools/Linux_HAL_Essentials/rangefinderpru/`
2. `make`

