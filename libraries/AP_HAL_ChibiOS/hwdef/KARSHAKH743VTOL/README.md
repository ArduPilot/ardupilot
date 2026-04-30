# KARSHAKH743VTOL Flight Controller

The KARSHAKH743VTOL is a hardware produced by [Karshak Drones Pvt. Ltd.](https://www.karshakdrones.com/) and will be sold on soon enough.


## Features

- STM32H743 microcontroller
- Single BMI088 IMU
- SPL06 barometer
- 12V 2.5A BEC; 5V 2.5A BEC
- microSD card slot
- USB-TypeC port
- 7 UART plus USB
- 10 PWM outputs
- 2 I2C ports
- 1 CAN port
- 2 External SPI ports
- 1 SWD Debug port
- Builtin RGB LED
- External/builtin Buzzer
- Voltage and Current monitoring for Vcc

## UART Mapping

- SERIAL0 -> USB
- SERIAL1 -> UART2 (MAVLink2, DMA-enabled)
- SERIAL2 -> UART4 (RCIN, Shared port)
- SERIAL3 -> UART1 (GPS, DMA-enabled)
- SERIAL4 -> UART3 (VTX-HD, DMA-enabled)
- SERIAL5 -> UART7 (ESC Telemetry, DMA-enabled)
- SERIAL6 -> UART5 (MAVLink2, DMA-enabled)
- SERIAL7 -> UART8 (MAVLink2, DMA-enabled)

## RC Input

RC input is configured on the RCIN pin, at one end of the servo rail, marked RCIN in the above diagram. All ArduPilot supported unidirectional RC protocols can be input here including PPM. For bi-directional or half-duplex protocols, such as CRSF/ELRS a full UART(such as UART2, UART5 or UART8) will have to be used. Set the protocol to receive RC data: `SERIALn_PROTOCOL=23` and change SERIAL2 _Protocol to something other than '23'.

## VTX Support

The generic 2.54mm pinHeader connector, situated at top right corner, supports a DJI Air Unit / HD VTX connection. Protocol defaults to DisplayPort. Pin 1 of the connector is 9V so be careful not to connect this to a peripheral requiring 5V/3.3V.

## PWM Output

The KARSHAKH743VTOL supports up to 10 PWM outputs. All channels support bi-directional DShot.

10 PWM outputs are grouped in 3 groups:

- PWM 1, 2, 3, 4 are in Group1 (TIM1);
- PWM 5 and 6 are in Group2 (TIM2);
- PWM 7, 8, 9, 10 are in Group3 (TIM3);

Channels in the same group must use the same output protocol and/or output rate. If any channel in a group uses DShot then all the channels in the group needs to use DShot.

## Battery Monitoring

The board has a internal voltage sensor and connections on the ESC connector for an external current sensor input.
The voltage sensor can handle up to 6S LiPo batteries.

The default battery parameters are:

- BATT_MONITOR 4
- BATT_VOLT_PIN 4
- BATT_CURR_PIN 8
- BATT_VOLT_MULT 10.2
- BATT_AMP_PERVLT 20.4

## Compass

The KARSHAKH743VTOL does not have a built-in compass, but you can attach an external compass using I2C on the SDA and SCL connector.

## Analog Inputs

The KARSHAKH743VTOL has 2(+1) analog inputs.

- ADC1 -> Battery Voltage (internal sensor)
- ADC2 -> Battery Current Sensor (external sensor)
- ADC3 -> RSSI voltage monitoring (disabled)/AUXGPIO

## GPIOs

The 3 outputs labelled AUX1, AUX2 and AUX3 are the "auxiliary" outputs.

The numbering of the GPIOs for PIN variables in ardupilot is:

- AUX1 - PA4 - 81
- AUX2 - PC1 - 82
- AUX3 - PC0 - 83

## Physical

- Mounting: 30.5 x 30.5mm, Φ2mm
- Dimensions: 60 x 37mm
- Weight: 16.5g (with onboard buzzer, external IMU connector and USB TypeC port)


## Loading Firmware

Firmware for these boards can be found at the [ArduPilot firmware server](https://firmware.ardupilot.org) in sub-folders labeled "KARSHAKH743VTOL".

The board comes pre-installed with an ArduPilot compatible bootloader, enabling the loading of `*.apj` firmware files through any ArduPilot compatible ground station.

However, if the board does not include the ArduPilot bootloader by default, you can perform the initial firmware installation by entering the DFU mode—connect the USB cable while holding the bootloader button. Then flash the `****_bl.hex` file using any DFU programming utility.

Once this initial step is complete, all subsequent firmware updates can be applied via ArduPilot ground station software using the standard `*.apj` files.