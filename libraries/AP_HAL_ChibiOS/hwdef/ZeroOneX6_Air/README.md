## ZeroOneX6 Air/Air+ Flight Controller
The ZeroOneX6_Air is a series of flight controllers manufactured by ZeroOne, which is based on the open-source FMU v6C architecture and Pixhawk Autopilot Bus open source specifications.  

![Uploading ZeroOneX6_Air.jpg…](https://github.com/ZeroOne-Aero/ardupilot/blob/pr-ZeroOneX6_Air_Series-250103/libraries/AP_HAL_ChibiOS/hwdef/ZeroOneX6_Air/ZeroOneX6_Air.png?raw=true)


## Features:
- Separate flight control Inertial Measurement Unit design.
- MCU 
   STM32H743IIK6 32-bit processor running at 400MHz
   2MB Flash
   1MB RAM
   microSD card interface
   Safety Switch /LED
   2x CAN
   6x UART, one with RTS/CTS
   2x I2C
   2x ADC inputs
- IO MCU
   STM32F103
- **Air's Sensors**
    - IMU:  
        With Synced IMU, BalancedGyro technology, low noise and more shock-resistant:  
        IMU- ICM45686 (No vibration isolation)  
    - Baro:  
      one barometer ：ICP20100  
    - Magnetometer:  
      Builtin IST8310 magnetometer 
- **Air+ Sensors**
    - IMU:  
       Internal Vibration Isolation for IMU  
       IMU constant temperature heating (1W heating power).  
       With Double Synced IMUs, BalancedGyro technology, low noise and more shock-resistant:  
       IMU1- ICM45686 (With vibration isolation)  
       IMU2- ICM45686 (No vibration isolation)  
    - Baro:  
      Two barometers ：2 x ICP20100
    - Magnetometer:  
      Builtin IST8310 magnetometer

## Pinout
![ZeroOneX6_Air Pinout](https://github.com/ZeroOne-Aero/ardupilot/blob/pr-ZeroOneX6_Air_Series-250103/libraries/AP_HAL_ChibiOS/hwdef/ZeroOneX6_Air/ZeroOneX6_AirSeriesPinout.jpg "ZeroOneX6_Air")

## UART Mapping
The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the receive pin for UARTn. The Tn pin is the transmit pin for UARTn.
| Name    | Function | MCU PINS |   DMA   |
| :-----: | :------: | :------: | :------:|
| SERIAL0 | OTG1     | USB      | 
| SERIAL1 | Telem1   | UART7    |DMA Enabled |
| SERIAL2 | Telem2   | UART5    |DMA Enabled |
| SERIAL3 | GPS1     | UART1   |DMA Enabled |
| SERIAL4 | GPS2     | UART8    |DMA Enabled |
| SERIAL5 | Telem3   | UART2   |DMA Enabled |
| SERIAL6 | User    | UART4    |DMA Enabled |
| SERIAL7 | Debug    | UART3    |DMA Enabled |
| SERIAL8 | OTG-SLCAN| USB      |

## RC Input
The SBUS pin, can be used for all ArduPilot supported receiver protocols, except CRSF/ELRS and SRXL2 which require a true UART connection. However, FPort, when connected in this manner, will only provide RC without telemetry. 
To allow CRSF and embedded telemetry available in Fport, CRSF, and SRXL2 receivers, a full UART, such as SERIAL6 (UART4) would need to be used for receiver connections. Below are setups using Serial6.
- :ref:`SERIAL6_PROTOCOL<SERIAL6_PROTOCOL>` should be set to "23".
- FPort would require :ref:`SERIAL6_OPTIONS<SERIAL6_OPTIONS>` be set to "15".
- CRSF/ELRS would require :ref:`SERIAL6_OPTIONS<SERIAL6_OPTIONS>` be set to "0".
- SRXL2 would require :ref:`SERIAL6_OPTIONS<SERIAL6_OPTIONS>` be set to "4" and connects only the TX pin.
Any UART can be used for RC system connections in ArduPilot also, and is compatible with all protocols except PPM. See :ref:`common-rc-systems` for details.

## PWM Output
The X6_Air flight controller supports up to 15 PWM outputs.  
First 8 outputs (labelled M1 to M8) are controlled by a dedicated STM32F103 IO controller.   
The remaining 7 outputs (labelled A9 to A15) are the "auxiliary" outputs. These are directly attached to the STM32H753 FMU controller.  
All 15 outputs support normal PWM output formats. All outputs support DShot, except A15.

The M1-8 outputs are in 4 groups:
- Outputs 1 and 2 in group1
- Outputs 3 and 4 in group2
- Outputs 5, 6, 7 and 8 in group3
  
The A9-15 outputs are in 4 groups:
- Outputs 9 - 12 in group1
- Outputs 13 and 14 in group2
- Outputs 15 in group3

Channels within the same group need to use the same output rate. If any channel in a group uses DShot then all channels in the group need to use DShot.

## GPIO
All PWM outputs can be used as GPIOs (relays, camera, RPM etc). To use them you need to set the output’s SERVOx_FUNCTION to -1. The numbering of the GPIOs for PIN variables in ArduPilot is:

<table>
  <tr>
    <th colspan="3">IO Pins</th>
    <th colspan="1"> </th>
    <th colspan="3">FMU Pins</th>
  </tr>
  <tr><td> Name </td><td> Value </td><td> Option </td><td>  </td><td> Name </td><td> Value </td><td> Option </td></tr>
  <tr><td> M1 </td><td> 101 </td> <td> MainOut1 </td><td>  </td><td> A9 </td><td> 50 </td><td> AuxOut1 </td></tr>
  <tr><td> M2 </td><td> 102 </td> <td> MainOut2 </td><td>  </td><td> A10 </td><td> 51 </td><td> AuxOut2 </td></tr>
  <tr><td> M3 </td><td> 103 </td> <td> MainOut3 </td><td>  </td><td> A11 </td><td> 52 </td><td> AuxOut3 </td></tr>
  <tr><td> M4 </td><td> 104 </td> <td> MainOut4 </td><td>  </td><td> A12 </td><td> 53 </td><td> AuxOut4 </td></tr>
  <tr><td> M5 </td><td> 105 </td> <td> MainOut5 </td><td>  </td><td> A13 </td><td> 54 </td><td> AuxOut5 </td></tr>
  <tr><td> M6 </td><td> 106 </td> <td> MainOut6 </td><td>  </td><td> A14 </td><td> 55 </td><td> AuxOut6 </td></tr>
  <tr><td> M7 </td><td> 107 </td> <td> MainOut7 </td><td>  </td><td> A15 </td><td> 56 </td><td>  </td></tr>
  <tr><td> M8 </td><td> 108 </td> <td> MainOut8 </td><td>  </td><td>  </td><td>  </td><td>  </td></tr>
</table>

## Battery Monitoring
The X6_Air flight controller has one six-pin power connectors, supporting CAN interface power supply.
The autopilot defaults are setup for CAN Power Module use (normally supplied with autopilot):
- :ref:`BATT_MONITOR<BATT_MONITOR>` = 8
- :ref:`CAN_P1_DRIVER<CAN_P1_DRIVER>` = 1
- :ref:`CAN_P2_DRIVER<CAN_P2_DRIVER>` = 1
- :ref:`CAN_D1_PROTOCOL<CAN_D1_PROTOCOL>` = 1
- :ref:`CAN_D2_PROTOCOL<CAN_D2_PROTOCOL>` = 1

## Compass
The X6_Air flight controller built-in industrial-grade electronic compass chip IST8310. Due to potential
interference, the autopilot is usually used with an external I2C compass as
part of a GPS/Compass combination.

## Analog inputs
The X6_Air flight controller has 2 analog inputs.
- ADC Pin12 -> ADC 6.6V Sense
- ADC Pin13 -> ADC 3.3V Sense

Loading Firmware
================
The board comes pre-installed with an ArduPilot compatible bootloader,
allowing the loading of xxxxxx.apj firmware files with any ArduPilot
compatible ground station.
Firmware for these boards can be found `here <https://firmware.ardupilot.org>`_ in  sub-folders labeled "ZeroOne_Air".

## Where to Buy
https://www.01aero.cn
