# ArduPilot on Qualcomm Flight

This is a port of ArduPilot to the Qualcomm Flight development board:

  http://shop.intrinsyc.com/products/snapdragon-flight-dev-kit

This board is interesting because it is small but offers a lot of CPU
power and two on-board cameras.

The board has 4 'Krait' ARM cores which run Linux (by default Ubuntu
14.04 Trusty), plus 3 'Hexagon' DSP cores which run the QURT RTOS.

There are two ports of ArduPilot to this board. One is called
'HAL_QURT' and runs primarily on the DSPs, with just a small shim on
the ARM cores. The other is a HAL_Linux subtype called 'QFLIGHT' which
runs mostly on the ARM cores, with just sensor and UARTs on the DSPs.

This is the readme for the QFLIGHT port. See the AP_HAL_QURT directory
for information on the QURT port.

# Building ArduPilot for 'QFLIGHT'

Due to some rather unusual licensing terms from Intrinsyc we cannot
distribute binaries of ArduPilot (or any program built with the
Qualcomm libraries). So you will have to build the firmware yourself.

To build ArduPilot you will need 3 library packages from
Intrinsyc. They are:

 * the HEXAGON_Tools package, tested with version 7.2.11
 * the Hexagon_SDK packet, version 2.0
 * the HexagonFCAddon package, tested with Flight_BSP_1.1_ES3_003.2

These packages should all be unpacked in a $HOME/Qualcomm directory.

To build APM:Copter you then do:

```
 cd ArduCopter
 make qflight -j4
```

you can then upload the firmware to your board by joining to the WiFi
network of the board and doing this

```
 make qflight_send FLIGHT_BOARD=myboard
```

where "myboard" is the hostname or IP address of your board.

This will install two files:

```
 /root/ArduCopter.elf
 /usr/share/data/adsp/libqflight_skel.so
```

To start ArduPilot just run the elf file as root on the flight
board. You can control UART output with command line options. A
typical startup command would be:

```
/root/ArduCopter.elf -A udp:192.168.1.255:14550:bcast -e /dev/tty-3 -B qflight:/dev/tty-2 --dsm /dev/tty-4
```

That will start ArduPilot with telemetry over UDP on port 14550, GPS
on tty-2 on the DSPs, Skektrum satellite RC input on tty-4 and
ESC output on tty-3.

Then you can open your favourite MAVLink compatible GCS and connect
with UDP.

# Logging

Logs will appear in /var/APM/logs as usual for Linux ArduPilot
ports. You can download logs over MAVLink or transfer over WiFi.

# UART connections

The Qualcomm Flight board has 4 DF13 6 pin UART connectors. Be careful
though as they do not have the same pinout as the same connectors on a
Pixhawk.

The pinout of them all is:

 * pin1: power
 * pin2: TX
 * pin3: RX
 * pin5: GND

3 of the 4 ports provide 3.3V power on pin1, while the 4th port
provides 5V power. Note that pin6 is not ground, unlike on a Pixhawk.

The 4 ports are called /dev/tty-1, /dev/tty-2, /dev/tty-3 and
/dev/tty-4. The first port is the one closest to the USB3
connector. The ports proceed counter-clockwise from there. So tty-2 is
the one closest to the power connector.

Only tty-2 provides 5V power. The others provide 3.3V power. You will
need to check whether your GPS can be powered off 3.3V.

# ESC PWM Output

To get signals to ESCs or servos you need to use a UART. The default
setup is to send 4 PWM signals as serial data on /dev/tty-3. This is
designed to work with this firmware for any ArduPilot compatible
board:

  https://github.com/tridge/ardupilot/tree/hal-qurt/libraries/RC_Channel/examples/RC_UART

that firmware will read the UART serial stream and output to the PWM
output of the board you use. For example, you could use a Pixracer or
Pixhawk board. This is an interim solution until Qualcomm/Intrinsyc
release an ESC add-on control board for the Qualcomm Flight.

Note that you can also use RC input from that attached board, allowing
you to use any ArduPilot compatible RC receiver.
