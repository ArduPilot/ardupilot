# Skystars H7HDv2 Flight Controller

The Skystars H7HDv2 is a flight controller produced by [Skystars](http://www.skystars-rc.com/).

## Features

- STM32H743 microcontroller
- ICM42688 IMU x2
- BMP280 barometer
- AT7456E OSD
- 8 UARTs
- 9 PWM outputs

## Pinout

![Skystars H7HDv2 Board](SkystarsH743v2.png "Skystars H7HDv2")
![Skystars H7HDv2 Wiring](SkystarsWiring.jpg "Skystars H7HDv2 Wiring")

## UART Mapping

The UARTs are marked RX and TX in the above pinouts. The RX pin is the
receive pin for UARTn. The TX pin is the transmit pin for UARTn.

- SERIAL0 -> USB
- SERIAL1 -> UART1 (RCin, DMA-enabled)
- SERIAL2 -> UART2 (MAVLink2 DMA-enabled)
- SERIAL3 -> UART3 (ESC Telem)
- SERIAL4 -> UART4 (GPS, DMA-enabled)
- SERIAL5 -> UART5 (SmartAudio)
- SERIAL6 -> UART6 (DisplayPort, DMA-enabled)
- SERIAL7 -> UART7 (USER DMA-enabled)
- SERIAL8 -> UART8 (USER)

## RC Input

RC input is configured on the R1 (UART1_RX) pin in the DJI connector or via the R1/T1 (UART1) pads.
It supports all serial RC protocols. For protocols requiring half-duplex serial to transmit
telemetry (such as FPort) you should setup SERIAL1 with half-duplex and connect to T1 or use
R1 with pin-swap.

## OSD Support

The Skystars H7HDv2 supports OSD using OSD_TYPE 1 (MAX7456 driver) and simultaneously DisplayPort
using UART6 on the HD VTX connector.

## PWM Output

The Skystars H7HDv2 supports up to 9 PWM outputs. The pads for motor output
M1 to M8 on the two motor connectors, plus M9 for LED strip or another
PWM output.

The PWM is in 5 groups:

- PWM 1, 2 in group1
- PWM 3, 4 in group2
- PWM 5-8  in group3
- PWM 9    in group4

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot. Outputs 1-8 support bi-directional dshot.

## Battery Monitoring

The board has a builtin voltage and current sensor. The current
sensor can read up to 130 Amps. The voltage sensor can handle up to 6S
LiPo batteries.

The correct battery setting parameters are:

- [BATT_MONITOR](https://ardupilot.org/copter/docs/parameters.html#batt-monitor-battery-monitoring) = 4
- [BATT_VOLT_PIN](https://ardupilot.org/copter/docs/parameters.html#batt-volt-pin-ap-battmonitor-analog-battery-voltage-sensing-pin) = 10
- [BATT_CURR_PIN](https://ardupilot.org/copter/docs/parameters.html#batt-curr-pin-ap-battmonitor-analog-battery-current-sensing-pin) = 11 (CURR pin)
- [BATT_VOLT_MULT](https://ardupilot.org/copter/docs/parameters.html#batt-volt-mult-ap-battmonitor-analog-voltage-multiplier) = 10.1
- [BATT_AMP_PERVLT](https://ardupilot.org/copter/docs/parameters.html#batt-amp-pervlt-ap-battmonitor-analog-amps-per-volt) = 17.0

## Compass

The Skystars H7HDv2 does not have a builtin compass, but you can attach an external compass using I2C on the SDA and SCL pads.

## VTX Power Control

If the JP jumper is bridged to PIT then GPIO 81 controls the VTX BEC output to pins marked "10V" and is included on the HD VTX connector. Setting this GPIO low removes voltage supply to this pin/pad. RELAY2 is configured by default to control this GPIO and is high by default.

GPIO 82 controls the analogue camera outputs. Setting this GPIO high selects Camera 1, low selects Camera 2. RELAY3 is configured by default to control this GPIO and is high by default.

## Camera Control

GPIO 83 is marked as "OSD". It can be used as a general GPIO pin. By default RELAY4 is configured to control this pin and sets the GPIO high.

## Loading Firmware

Firmware for these boards can be found [here](https://firmware.ardupilot.org) in sub-folders labeled "SkystarsH7HDv2".

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*.apj firmware files.
