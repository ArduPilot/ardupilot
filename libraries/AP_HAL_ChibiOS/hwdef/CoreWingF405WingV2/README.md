# CoreWingF405WingV2 Flight Controller

The CoreWing F405 Wing V2 is a fixed-wing and QuadPlane/VTOL flight controller produced by [CoreWing](https://www.corewing.com/en/).

## Where to Buy

- [CoreWing official website](https://www.corewing.com/en/)
- [GetFPV](https://www.getfpv.com/corewing-f405-wing-v2-flight-controller-stack.html)

## Features

- Processor
  - STM32F405RGT6 ARM, 168MHz
  - AT7456E OSD
  - ESP32-S3 wireless module for BLE/WiFi telemetry
- Sensors
  - ICM-42688P or BMI270 IMU, depending on board variant
  - SPA06-003 barometer
  - Voltage and current sensors
- Power
  - 3S to 6S LiPo input, 10V to 28V
  - 90A continuous / 215A peak current sensing
  - Integrated PDB with separate FC, servo, and VTX/CAM power rails
  - FC BEC: fixed 5.2V, 4A continuous / 5A peak
  - Servo BEC: 5V, 6V, or 7.2V, 8A continuous / 14A peak
  - VTX/CAM BEC: 5V, 9V, or 12V, 2A continuous / 3A peak
- Interfaces
  - 11 servo/motor outputs on pin headers, labeled S1 to S11
  - Additional pad labeled `LED` mapped as PWM12
  - PWM1-PWM10 support PWM/DShot
  - PWM11-PWM12 support normal PWM only in ArduPilot
  - SBUS/PPM input
  - Dedicated serial RC input for CRSF/ELRS/TBS Crossfire
  - 6 UARTs plus USB; UART1 is internally tied to the wireless module
  - I2C port for external compass, digital airspeed, and other I2C peripherals
  - Analog airspeed input
  - microSD card slot
  - USB-C port
  - Onboard BLE/WiFi wireless board
  - Switchable VTX/CAM supply

## Pinout

### Front

![CoreWingF405WingV2 Front](CoreWingF405WingV2_Front.jpg)

### Back

![CoreWingF405WingV2 Back](CoreWingF405WingV2_Back.jpg)

## Wiring Diagram

![CoreWingF405WingV2 Wiring](CoreWingF405WingV2_wiring.jpg)

![CoreWingF405WingV2 Wiring 2](CoreWingF405WingV2_wiring2.jpg)

## UART Mapping

| Port | UART    | Protocol | TX DMA | RX DMA |
|------|---------|----------|:------:|:------:|
| 0    | USB     | MAVLink2 | ✘      | ✘      |
| 1    | USART1  | MAVLink2 | ✔      | ✔      |
| 2    | USART2* | None     | ✘      | ✘      |
| 3    | USART3  | None     | ✔      | ✘      |
| 4    | UART4   | None     | ✔      | ✘      |
| 5    | UART5   | GPS      | ✔      | ✘      |
| 6    | USART6  | RCIN     | ✔      | ✔      |

\* USART2 RX is tied to the inverted SBUS RC input, but can be used as a normal UART if `BRD_ALT_CONFIG = 1` and no signal drives the SBUS pin.

## RC Input

The SBUS input is passed through an inverter to RX2. By default, RX2 is mapped to a timer input instead of the UART and can be used for SBUS, PPM, and receiver protocols that do not require a true UART.

Serial receiver protocols such as CRSF, ELRS, MAVLink RC input, and SRXL2 should be connected to UART6 using TX6 and RX6. `SERIAL6_PROTOCOL = 23` is set by default for serial RC input.

Recommended receiver connections:

- PPM: connect to the SBUS input
- SBUS: connect to the SBUS input
- CRSF / ELRS / TBS Crossfire: connect to TX6 and RX6, set `SERIAL6_OPTIONS = 0`
- DSM / SRXL: connect to RX6
- SRXL2: connect to TX6 and set `SERIAL6_OPTIONS = 4`
- FPort: connect to TX6 and RX6 through a bidirectional inverter, set `SERIAL6_OPTIONS = 0`

For more information, see [Radio Control Systems](https://ardupilot.org/plane/docs/common-rc-systems.html).

## PWM Output

The board provides 12 PWM output channels.

PWM outputs 1 to 11 are available on pin headers, labeled S1 to S11. The additional pad labeled `LED` is mapped as PWM12.

PWM outputs 1 to 10 support PWM and DShot. PWM outputs 1 to 4 additionally support
bi-directional DShot. PWM outputs 11 and 12 support normal PWM only in ArduPilot.

The PWM outputs are in 5 groups:

- PWM 1,2 in group1
- PWM 3,4 in group2
- PWM 5-7 in group3
- PWM 8-10 in group4
- PWM 11,12 in group5

Channels within the same group need to use the same output rate and protocol. If any DShot-capable output in a group uses DShot then all DShot-capable outputs in that group need to use DShot.

### LED / PWM12 Pad

The pad labeled `LED` is mapped as PWM12.

With INAV firmware, this pad can be used for serial LED output. A serial LED
output requires a DMA stream, but no DMA stream is available for TIM1 given this
board's current allocation. Therefore, the `LED` pad can only be used as a normal
PWM12 output in ArduPilot.

PWM11 (S11) and PWM12 share TIM1 and therefore belong to the same output group.

## GPIOs

| GPIO | Function |
|------|----------|
| 50 | PWM1 |
| 51 | PWM2 |
| 52 | PWM3 |
| 53 | PWM4 |
| 54 | PWM5 |
| 55 | PWM6 |
| 56 | PWM7 |
| 57 | PWM8 |
| 58 | PWM9 |
| 59 | PWM10 |
| 60 | PWM11 |
| 61 | PWM12, pad labeled LED, normal PWM only |
| 80 | Buzzer |
| 81 | VTX/CAM power relay |

## Integrated PDB and Power Wiring

The board includes an integrated PDB with separate power rails for the flight controller/peripherals, servos, and VTX/camera equipment.

CoreWing F405 Wing V2 includes three onboard BECs: an FC BEC, a servo BEC, and a VTX/CAM BEC. The VTX/CAM BEC output can be selected as 5V, 9V, or 12V. The servo BEC output can be selected as 5V, 6V, or 7.2V.

![CoreWingF405WingV2 PDB](CoreWingF405WingV2_pdb.jpg)

Do not connect an ESC BEC red wire to the servo rail unless the board power jumper configuration allows an external BEC input.

## Wireless Connection

The onboard CoreWing wireless board supports BLE and WiFi AP/STA modes. The [CoreWing app](https://www.corewing.com/en/app/) can be used for wireless parameter setup, wireless firmware flashing, and wireless board settings. Mission Planner can connect through WiFi using TCP or UDP.

WiFi AP default settings:

- SSID: `CoreWing WING-WiFi`
- Password: `88888888`
- IP address: `192.168.1.1`
- TCP port: `4278`
- UDP port: `14550`

## OSD Support

The CoreWing F405 Wing V2 supports using its internal OSD using OSD_TYPE 1 (MAX7456 driver). External OSD support such as DJI or MSP DisplayPort is supported using UART3 or any other free UART. See [MSP OSD](https://ardupilot.org/copter/docs/common-msp-osd-overview-4.2.html) for more information.

## VTX Control

UART3 TX is located in the video output connector and can be used to control video transmitters that support IRC Tramp or SmartAudio. See [VTX support](https://ardupilot.org/plane/docs/common-vtx.html) for more information.

## VTX Power Control

GPIO 81 controls the VTX/CAM power output through Relay 1 (`RELAY1_PIN` is preset to 81). The output is high-side switched: driving the GPIO high removes power from the VTX/CAM pins and low restores it. Relay 1 defaults to off, which drives the GPIO low, so the VTX/CAM supply is powered on at boot.

To switch the supply from a transmitter, assign an RC channel to Relay 1, for example on Channel 7:

- RC7_OPTION = 28

Turning Relay 1 on removes power from the VTX/CAM pins.

## Battery Monitoring

The board has a built-in voltage and current sensor. The current sensor can read up to 90A continuously and 215A peak. The voltage sensor can handle up to 6S LiPo batteries.

The correct battery setting parameters are set by default and are:

- BATT_MONITOR = 4
- BATT_VOLT_PIN = 10
- BATT_CURR_PIN = 11
- BATT_VOLT_MULT = 11.05
- BATT_AMP_PERVLT = 64

A 35V 470uF electrolytic capacitor is included in the package and should be installed on the power input to reduce ESC switching noise.

## Compass and GPS

The board does not have a built-in GPS or compass. An external GPS/compass module can be connected to the GPS connector. UART5 is GPS1 by default and the I2C bus can be used for an external compass.

The 4V5 pins are also powered when USB is connected. Avoid connecting high-current loads to 4V5 when powered only from USB.

## Analog Inputs

| ADC Pin | Function |
|---------|----------|
| 10 | Battery voltage |
| 11 | Battery current |
| 14 | Analog RSSI input, labeled `RSSI` on the board |
| 15 | Analog airspeed |

## Airspeed

The board supports both analog and digital airspeed sensors.

- Analog airspeed: use the AIR analog input, 0V to 6.6V range
- Digital airspeed: use the I2C airspeed connector

The MS4525DO, ASP5033, MS5525, SDP3X and NMEA digital airspeed sensors are enabled by default, in addition to analog airspeed. Other digital airspeed sensor drivers require a custom firmware build using the [Custom Firmware Build Server](https://custom.ardupilot.org).

## Loading Firmware

Firmware for this board can be found at the [ArduPilot firmware server](https://firmware.ardupilot.org) in the `CoreWingF405WingV2` sub-folder.

Initial firmware load can be done with DFU by plugging in USB with the boot button pressed. Subsequently, firmware can be updated with Mission Planner or the CoreWing app.
