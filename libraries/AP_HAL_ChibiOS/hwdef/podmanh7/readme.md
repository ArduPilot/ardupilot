# podmanH7 Flight Controller

The podmanH7  flight controller is manufactured and sold by [makermare](http://www.makermare.com).

## Features
	Processor
		STM32H743 microcontroller 2MB 32-bit processor
		AT7456E OSD
	Sensors
		Three IMUs: BMI270, ICM42688 and IIM42652
		BMI270 Acc/Gyro
		Internal vibration isolation for 2 IMUs
		IIM42652 Acc/Gyro
		ICM42688P Acc/Gyro
		SPL06 barometer
	Power
		2S - 6S Lipo input voltage with voltage monitoring(2 power analog ports)
	Interfaces
		16x PWM outputs DShot capable (8 PWM protocols as well as DShot)
		1x RC input (SBUS and PPM signals)
		6x UARTs/serial for GPS and other peripherals, 
		6th UART internally tied to Wireless board)
		USART1, USART2, USART3, UART4, UART7, UART8 and OTG1 have full DMA (RX and TX).
		3 I2C ports for external compass, airspeed, etc.
		I2C_ORDER: I2C1 I2C2 I2C4
		microSDCard for logging, etc.
		Internal RGB LED
		USB-C port
		Safety switch port
		Buzzer port

## UART Mapping

SERIAL_ORDER: OTG1 USART1 USART2 USART3 UART4 UART7 UART8 OTG2

 - SERIAL0 -> USB(OTG1)
 - SERIAL1 -> USART1(Telem1)
 - SERIAL2 -> USART2 (Telem2)
 - SERIAL3 -> USART3 (GPS)
 - SERIAL4 -> UART4,(GPS2)
 - SERIAL5 -> UART7,(SBUS)
 - SERIAL6 -> UART8 (Computer)
 - SERIAL7 -> USB2(OTG2)

## RC Input

The remote control signal should be connected to the “RC IN” pin, at one side of the servo channels.
This signal pin supports two types of remote control signal inputs, SBUS and PPM signals.

## PWM and GPIO Output

The podmanH7 supports up to 16 PWM outputs.support 8 PWM protocols as well as DShot. 
All 16 PWM outputs have GND on the top row, 5V on the middle row and signal on the bottom row.

All 16 PWM channels can be used for GPIO functions (relays, buttons, RPM etc).

The 9-16 PWM(as well as DShot) ports is in 5 groups:
 - PWM 1,2,3,4 in group1
 - PWM 5,6 in group2
 - PWM 7,8 in group3

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in that group would need
to use DShot.

The pin numbers for these PWM channels in ArduPilot are shown below:

| PWM Channels | Pin  | PWM Channels  | Pin  |
| ------------ | ---- | ------------  | ---- |
| PWM1         | 101   | PWM9         | 50   |
| PWM2         | 102   | PWM10        | 51   |
| PWM3         | 103   | PWM11        | 52   |
| PWM4         | 104   | PWM12        | 53   |
| PWM5         | 105   | PWM13        | 54   |
| PWM6         | 106   | PWM14        | 55   |
| PWM7         | 107   | PWM15        | 56   |
| PWM8         | 108   | PWM16        | 57   |

The correct 9-16 PWM and GPIO setting parameters are set by default and are:

 - PE14 TIM1_CH4 TIM1 PWM(1) GPIO(50)
 - PE13 TIM1_CH3 TIM1 PWM(2) GPIO(51)
 - PE11 TIM1_CH2 TIM1 PWM(3) GPIO(52)
 - PE9  TIM1_CH1 TIM1 PWM(4) GPIO(53)
 - PD13 TIM4_CH2 TIM4 PWM(5) GPIO(54)
 - PD14 TIM4_CH3 TIM4 PWM(6) GPIO(55)
 - PB0 TIM3_CH3 TIM3 PWM(7) GPIO(56)
 - PB1 TIM3_CH4 TIM3 PWM(8) GPIO(57)

## Battery Monitoring

The podmanH7 flight controller has two six-pin power connectors, 
supporting analog interface power supply.

The board has a builting voltage and current sensor.
The current sensor can read up to 90A continuosly, 
215 Amps peak. The voltage sensor can handle up to 6S LiPo batteries.

The correct battery setting parameters are set by default and are:

 - BATT_VOLT_PIN 2
 - BATT_CURR_PIN 3
 - BATT_AMP_PERVLT 39.877
 - BATT_VOLT_MULT 12.02
 - BATT_VOLT_SCALE 10.1
 - HAL_BATT_CURR_SCALE 17.0
 - BATT2_VOLT_PIN 13
 - BATT2_CURR_PIN 14
 - BATT2_AMP_PERVLT 39.877
 - BATT2_VOLT_MULT 12.02

## Compass

The podmanH7 flight controller does not have a built-in compass, 
but you can attach an external compass using I2C on the SDA and SCL pads.

## Analog inputs

The podmanH7 flight controller has 5 analog inputs

 - ADC Pin3 -> Battery Current 
 - ADC Pin2  -> Battery Voltage 
 - ADC Pin14   -> ADC Current Sense
 - ADC Pin13  ->  ADC Voltage Sense
 - ADC Pin15  ->  Pressure sens

## Loading Firmware

The podmanH7 flight controller comes pre-installed with an ArduPilot 
compatible bootloader, allowing the loading of *.apj firmware files with any ArduPilot compatible ground station.

Initial firmware load can be done with DFU by plugging in USB with the
boot button pressed. Then you should load the "podmanH7_bl.hex"
firmware, using your favourite DFU loading tool.

Subsequently, you can update firmware with Mission Planner.
