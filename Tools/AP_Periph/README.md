# AP_Periph UAVCAN Peripheral Firmware

This is an ArduPilot based UAVCAN peripheral firmware. This firmware
takes advantage of the wide range of sensor drivers in ArduPilot to
make building a UAVCAN peripheral firmware easy.

The AP_Periph firmware is based on the same ChibiOS hwdef.dat system
that is used to define pinouts for STM32 based flight controllers
supported by ArduPilot. That means you can add support for a new
UAVCAN peripheral based on the STM32 by just writing a simple
hwdef.dat that defines the pinout of your device.

Currently we have four targets building for AP_Periph firmwares:

 - A STM32F103 128k flash part made by mRobotics (target f103-GPS)
 - A STM32F412 512k flash part made by CUAV (target CUAV_GPS)
 - A STM32F105 256k flash part (used in ZubaxGNSSv2)
 - A STM32F303 256k flash part made by mRobotics (target f303-GPS)

More can be added using the hwdef.dat system

# Features

The AP_Periph firmware can be configured to enable a wide range of
UAVCAN sensor types. Support is included for:

 - GPS modules (including RTK GPS)
 - Magnetometers (SPI or I2C)
 - Barometers (SPI or I2C)
 - Airspeed sensors (I2C)
 - Rangefinders (UART or I2C)
 - ADSB (Ping ADSB receiver on UART)
 - Battery Monitor (Analog, I2C/SMBus, UART)
 - LEDs (GPIO, I2C or WS2812 serial)
 - Safety LED and Safety Switch
 - Buzzer (tonealarm or simple GPIO)
 - RC Output (All standard RCOutput protocols)

An AP_Periph UAVCAN firmware supports these UAVCAN features:

 - dynamic or static CAN node allocation
 - firmware upload
 - automatically generated bootloader
 - parameter storage in flash
 - easy bootloader update
 - high resiliance features using watchdog, CRC and board checks
 - firmware and parameter update via MissionPlanner or DroneCAN GUI tool when attached to an autopilot
 - firmware update via USB if USB port is provided
 - parameter update using SLCAN and DroneCAN GUI on standalone peripheral via USB, if provided

# Building

Using f103-GPS as an example, build the main firmware like this:

 - ./waf configure --board f103-GPS
 - ./waf AP_Periph

that will build a file build/f103-GPS/bin/AP_Periph.bin. You can
now load that using the CAN bootloader and either uavcan_gui_tool or
MissionPlanner SLCAN support.

# Flashing

To load directly with a stlink-v2, do this:

 - st-flash write build/f103-GPS/bin/AP_Periph.bin 0x8006400

for the CUAV_GPS which loads at offset 0x10000 use this:

 - st-flash write build/CUAV_GPS/bin/AP_Periph.bin 0x8010000

# Flashing bootloader

To flash the bootloader use this:

 - st-flash write Tools/bootloaders/f103-GPS_bl.bin 0x8000000

# Building bootloader

To build the bootloader use this:

 - Tools/scripts/build_bootloaders.py f103-GPS

the resulting bootloader will be in Tools/bootloaders

# Firmware Builds

Firmware targets are automatically built and distributed on the
ArduPilot firmware server on firmware.ardupilot.org. These firmwares
can be loaded using Mission Planner or the UAVCAN GUI Tool. Parameters
for peripherals can be changed using the Mission Planner SLCAN support
or using UAVCAN GUI Tools.

# User Bootloader Update

The bootloader is automatically stored in ROMFS in the main
firmware. End users can update the bootloader by setting the UAVCAN
parameter "FLASH_BOOTLOADER" to 1. After setting it to 1 the node will
respond with a debug text message which can be seen in the UAVCAN GUI
tool to show the result of the flash.

# SITL Testing

Currently GPS peripheral build is supported under linux environment,
we simulate a UAVCAN GPS Peripheral on SocketCAN.

Setup can be done as follows, this is on top of usual setup required 
to build ardupilot:

```
sudo dpkg --add-architecture i386
sudo apt-get update
sudo apt-get install -y gcc-multilib g++-multilib
sudo apt-get update
sudo apt-get -y install can-utils iproute2 linux-modules-extra-$(uname -r)
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```
Build Commands:
```
./waf configure --board sitl_periph_gps
./waf AP_Periph
```
Autotest Command:
```
Tools/autotest/autotest.py -v Copter build.SITLPeriphGPS test.CAN
```


---
**Note**

To run valgrind on AP_Periph binary you might need to get 32 bit version of libc6-dbg which can be simply get using following command for Ubuntu machines: `sudo apt-get install libc6-dbg:i386`

---


https://github.com/linux-can/can-utils contains a nice set of utility to do CAN related testings on Linux system. I used Ubuntu for this development, for Ubuntu systems you can simply download this tool using `sudo apt-get install can-utils`

Following are the common commands that can be used while testing or developing:
* Create Virtual CAN Interface:
```
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
sudo ip link add dev vcan1 type vcan
sudo ip link set up vcan1
```
* Route one CANSocket to another
```
sudo modprobe can-gw
sudo cangw -A -s vcan0 -d vcan1 -e
sudo cangw -A -s vcan1 -d vcan0 -e
```
* Delete routes
```
sudo cangw -D -s vcan0 -d vcan1 -e
sudo cangw -D -s vcan1 -d vcan0 -e
```
* Route SLCAN to VCAN, this allows connecting CAN devices to SITL run via CAN Adapter like the one running in Ardupilot itself.
```
sudo modprobe slcan
sudo modprobe can-gw
sudo slcan_attach -f -s8 -o /dev/ttyACM0
sudo slcand ttyACM0 slcan0
sudo ifconfig slcan0 up
sudo cangw -A -s vcan0 -d slcan0 -e
sudo cangw -A -s slcan0 -d vcan0 -e
```
* Dump can messages:
```
sudo candump vcan0
```

# Discussion and Feedback

Please join the discussions at these locations:

 - https://discuss.ardupilot.org/t/ap-periph-1-0-0-stable-released/49049
 - https://gitter.im/ArduPilot/CANBUS
