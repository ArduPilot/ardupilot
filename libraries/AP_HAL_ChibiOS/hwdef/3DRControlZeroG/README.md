# 3DR (mRo) Control Zero H7 OEM Flight Controller revision G

The Control Zero H7 OEM revision G is a flight controller produced by [3DR (mRo)](https://store.3dr.com/control-zero-h7-oem-g/).

![3DR Control Zero H7 OEM rev G - Top](https://vddwxegfxugwzpfnrrlp.supabase.co/storage/v1/object/public/Website-CDN/autopilot-img/CZOEM_revG_front.JPG)
![3DR Control Zero H7 OEM rev G - Bottom](https://vddwxegfxugwzpfnrrlp.supabase.co/storage/v1/object/public/Website-CDN/autopilot-img/CZOEM_revG_back.JPG?t=2024-03-08T20%3A18%3A49.140Z)
![3DR Control Zero H7 OEM rev G - Top w/ case](https://vddwxegfxugwzpfnrrlp.supabase.co/storage/v1/object/public/Website-CDN/autopilot-img/CZOEM_revG_case_front.JPG?t=2024-03-08T20%3A18%3A57.128Z)
![3DR Control Zero H7 OEM rev G - Bottomi w/ case](https://vddwxegfxugwzpfnrrlp.supabase.co/storage/v1/object/public/Website-CDN/autopilot-img/CZOEM_revG_case.jpg?t=2024-03-08T20%3A18%3A52.750Z)

## Features

	Processor
		STM32H743IIK6 32-bit Processor
	Sensors
		BMI088 6DOF
		ICM20602 6DOF
		ICM20948 9DOF
		DPS368 Baro
	Power
		External Power Supply
		Logic level at 3.3V
	Interfaces
		Bottom Connectors: 36pin front and 40pin back Samtec FTM-118-02-x and FTM-120-02-x
		8x PWM / IO - DMA capable
		1x RC Input 
		5x UARTs (2x with hardware flow control)
		2x CAN
		1x SPI
		3x I2C
		SWD via TC2030 header
		SDCARD Socket
	Memory
		FRAM (256KB)
	Miscellaneous
		Onboard 3 color LED
		Buzzer
		Safety Button


###  Uncased Weight and Dimensions

 Weight: 3.66g (13.oz)
 Width:  20mm (.79in)
 Length: 34mm (1.34in)

 *Case sold separately*

## Changelog

- M10059C - Initial Release
- M10059G adds external power supply and TCXO.

## Pinout

![Control Zero H7 OEM revision G pinout](https://vddwxegfxugwzpfnrrlp.supabase.co/storage/v1/object/public/Website-CDN/pinouts/czoem_pinout_revG_topview.png)

## UART Mapping

- SERIAL0 -> USB1, for GCS connection

- SERIAL1 -> USART2 (TELEM 1) DMA Enabled

- SERIAL2 -> USART3 (TELEM 2) DMA Enabled

- SERIAL3 -> UART4 (GPS) DMA Enabled

- SERIAL4 -> UART8 (GPS 2) DMA Enabled

- SERIAL5 -> UART7 (DEBUG) DMA Enabled

- SERIAL6 -> USART6 (Additional USART) DMA Enabled

- SERIAL7 -> USB2, MAVLink interface

## RC Input

RC input is configured on the RC_IN pin. These are the supported RC input protocols: 

Spektrum DSM / DSM2 / DSM-X® Satellite compatible input and binding. 
Futaba S.BUS® & S.BUS2® compatible input. 
Graupner SUMD. Yuneec ST24.

## Analog Inputs

The Control Zero H7 OEM revision G has 4 ADC inputs:

- ADC1 Pin11 -> RSSI IN
- ADC1 Pin14 -> Battery Voltage
- ADC1 Pin15 -> Battery Current
- ADC1 Pin18 -> 5V Sensor

## PWM Output

The Control Zero H7 OEM revision G supports up to 8 PWM outputs. All DShot and BiDirDShot capable.

The PWM outputs are distributed in 3 groups:

- PWM 1-4 in group 1
- PWM 5-6 in group 4
- PWM 7-8 in group 8

Channels within the same group must use only one output rate. If any channel is using DShot or BiDirDShot the rest of the group will use the said output type.

## Power Supply

This board requires a 5V, 1 Amps power supply.

## Battery Monitoring

This board has a built-in voltage and current sensors. The following settings need to be present already on the board to work with a Power Zero Module (M10077):

- BATT_MONITOR 4
- BATT_VOLT_PIN 14
- BATT_CURR_PIN 15
- BATT_VOLT_SCALE 15.3
- BATT_CURR_SCALE 50.0

*Other Power Module needs to be adjusted accordingly*

## Build

`./waf configure --board=3DRControlZeroG`

`./waf copter` (check ArduPilot's docs for more info about the available targets)

The compiled binary will be located in `build/3DRControlZeroG/bin/arducopter.apj`.

## Uploading Firmware

Any Control Zero H7 OEM revision G has a preloaded Ardupilot bootloader, which allows the user to use a compatible Ground Station software to upload the `.apj` file.  
