# AtomRC F405-NAVI- Deluxe

![ATOMRCF405NAVI-Deluxe](atomrcf405-navi-deluxe.jpg)


the above image and some content courtesy of `ATOMRC <http://atomrc.com/>`__

.. note::

	Due to flash memory limitations, this board does not include all ArduPilot features.
        See :ref:`Firmware Limitations <common-limited_firmware>` for details.

# Specifications

-  **Processor**

   -  STM32F405RGT6 ARM (168MHz)
   -  AT7456E OSD
   -  ESP32 Bluetooth RF module


-  **Sensors**

   -  ICM-42688 IMU (accel, gyro)
   -  SPL-06 barometer
   -  Voltage & 120A current sensor


-  **Power**

   -  6V ~ 30V DC input power
   -  5.15/6/7.15V seelctable, 5A BEC for servos
   -  9V/12V, 2A BEC for video


-  **Interfaces**

   -  6x UARTS
   -  12x PWM outputs (PWM12 defaults to serial LED)
   -  1x RC input with inverter for SBUS/PPM
   -  I2C port for external compass and airspeed sensor
   -  Type-C USB port
   -  SD Card Slot
   -  6 pin JST-GH for GPS/Compass
   -  6 pin JST-GH for DJI air units
   -  6 pin JST-GH for remote USB/Buzzer included with autopilot


-  **Size and Dimensions**

   - 50mm x 30mm x 12mm
   - 21g

# Where to Buy

[ATOMRC](<https://atomrc.com)

# Pinout


tbd


# Default UART order

- SERIAL0 = console = USB
- SERIAL1 = RF Module = USART1(MAVLink2), not usable by AP GCS
- SERIAL2 = RCinput, DMA capable = USART2 (RX2 connected to SBUS pins via inverter for SBUS receivers)
- SERIAL3 = USER = USART3
- SERIAL4 = GPS2 = UART4
- SERIAL5 = DisplayPort = UART5
- SERIAL6 = USER = USART6 (DMA capable)


Serial protocols shown are defaults, but can be adjusted to personal preferences.

# Dshot capability

All motor/servo outputs are Dshot and PWM capable. Outputs 1/2 and 6/7 are Bi-Directional DSHot capable.

Mixing Dshot and normal PWM operation for outputs is restricted into groups, ie. enabling Dshot for an output in a group requires that ALL outputs in that group be configured and used as Dshot, rather than PWM outputs. The output groups that must be the same (PWM rate or Dshot, when configured as a normal servo/motor output) are: 1/2, 3/4, 5/6/7, 8/9/10, and 11/12(LED).

.. note:: PWM12 is marked as "LED" and defaulted to serial led protocol, so output 11 must also be used for serial LED unless output 12 function is changed.

# RC Input

The SBUS pin, is passed by an inverter to RX2 (UART2 RX). UART2 is defaulted to RCIN protocol and can be used for all ArduPilot supported receiver protocols, except CRSF/ELRS and SRXL2 which require a true UART connection.

- PPM is not supported.

- DSM/SRXL connects to the RX2  pin, but SBUS would still be connected to SBUS.

- FPort requires connection to TX2 and RX2 via a bi-directional inverter. See :ref:`common-FPort-receivers`.

- CRSF/ELRS also requires a TX2 connection, in addition to RX2, and automatically provides telemetry.

- SRXL2 requires a connection to TX2 and automatically provides telemetry.  Set :ref:`SERIAL6_OPTIONS<SERIAL6_OPTIONS>` to "4".


.. note:: the 5v pin above the SBUS pin is powered when USB is connected. All other 5V pins are only powered when battery is present.

# Battery Monitor Configuration

These settings are set as defaults when the firmware is loaded (except :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT>` which needs to be changed from the default value). However, if they are ever lost, you can manually set the parameters:

Enable Battery monitor.

:ref:`BATT_MONITOR<BATT_MONITOR>` =4

Then reboot.

:ref:`BATT_VOLT_PIN<BATT_VOLT_PIN__AP_BattMonitor_Analog>` 10

:ref:`BATT_CURR_PIN<BATT_CURR_PIN__AP_BattMonitor_Analog>` 11

:ref:`BATT_VOLT_MULT<BATT_VOLT_MULT__AP_BattMonitor_Analog>` 11

:ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT__AP_BattMonitor_Analog>` 30 

# Connecting a GPS/Compass module

This board does not include a GPS or compass. But a connector is provided to attach an external module.

# OSD

The internal analog OSD is enabled by default. Simultaneous use of HD VTX with OSD (DisplayPort) is also enabled by default via the 6 pin connector labeled "DJI". Either 9V (default) or 12V VTX power can be selected by solder jumper.

# BLE RF Module
An integrated BLE RF module is attached to UART1 and its power controlled by a pin (81) which is preset to be controlled by RELAY1 (high is power on to module). By default, the module is powered down on boot since ArduPilot GCS currently do not support BLE.

# Loading Firmware

Firmware for this board can be found `here <https://firmware.ardupilot.org>`__  in sub-folders labeled “ATOMRCF405NAVI-Deluxe”.

Initial firmware load can be done with DFU by plugging in USB with the
boot button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using any ArduPilot ground station software. Updates should be done with the \*.apj firmware files.

.



