# QioTek AdeptF407 Flight Controller

The Qiotek AdeptF407 an autopilot produced by [QioTek](https://www.qiotek.io).

It is an autopilot used CKS MCU.

## Features
 - MCU: CKS32F407VGT6
 - Accelerometer/Gyro: ICM4 series/ICM2 series or ICM4 series/ICM4 series
 - BEC output: 5V 3A for autopilot and peripheral hardware
 - BEC output: 9V/12V 3A for camera and analog video transmission
 - Barometer: DPS310
 - OSD: AT7456E
 - builtin IST8310 magnetometer(internal I2C)
 - builtin RAMTRON(SPI)
 - 12 dedicated PWM/Capture inputs on FMU
 - 5 UARTS: (USART1, USART2, USART3, UART4, USART7)
 - 2 I2C ports
 - 1 CAN port
 - 2 Analog inputs of voltage / current for battery monitoring
 - 2 analog video input channels
 - 1 analog video output source switcher switching by relay5
 - 4 relays output control
 - 1 Status LED
 - 1 nARMED

## Pinout
![QioTek AdpetF407 Board](../QioTekAdeptF407/adept_f407.jpg "QioTek AdpetF407")

## Connectors

**External USB**

| Pin  | Signal  | Volt  |
| :--: | :-----: | :---: |
|  1   |   VCC   |  +5V  |
|  2   |   D+    |  D_P  |
|  3   |   D-    |  D_N  |
|  4   |   GND   |  GND  |

**Telme1**

| Pin  | Signal | Volt  |
| :--: | :----: | :---: |
|  1   |  VCC   |  +5V  |
|  2   |  TX    | +3.3V |
|  3   |  RX    | +3.3V |
|  4   |  GND   |  GND  |

**Telme2**

| Pin  | Signal | Volt  |
| :--: | :----: | :---: |
|  1   |  VCC   |  +5V  |
|  2   |  TX    | +3.3V |
|  3   |  RX    | +3.3V |
|  4   |  NC    |  - -  |
|  5   |  NC    |  - -  |
|  6   |  GND   |  GND  |

**GPS1**

| Pin  | Signal | Volt  |
| :--: | :----: | :---: |
|  1   |  VCC   |  +5V  |
|  2   |  TX    | +3.3V |
|  3   |  RX    | +3.3V |
|  4   |  scl2  | +3.3V |
|  5   |  sda2  | +3.3V |
|  6   |  GND   |  GND  |

**GPS1**

| Pin  | Signal | Volt  |
| :--: | :----: | :---: |
|  1   |  VCC   |  +5V  |
|  2   |  TX    | +3.3V |
|  3   |  RX    | +3.3V |
|  4   |  scl2  | +3.3V |
|  5   |  sda2  | +3.3V |
|  6   |  GND   |  GND  |

**UART4 and UART5**

| Pin  | Signal | Volt  |
| :--: | :----: | :---: |
|  1   |  VCC   |  +5V  |
|  2   |  TX    | +3.3V |
|  3   |  RX    | +3.3V |
|  4   |  GND   |  GND  |

**CAN1**

| Pin  | Signal | Volt  |
| :--: | :----: | :---: |
|  1   |  VCC   |  +5V  |
|  2   | CAN1_H | CAN_P |
|  3   | CAN1_L | CAN_N |
|  4   |  GND   |  GND  |

**IIC1**

| Pin  | Signal | Volt  |
| :--: | :----: | :---: |
|  1   |  VCC   |  +5V  |
|  2   |  SDA1  | +3.3V |
|  3   |  SCL1  | +3.3V |
|  4   |  GND   |  GND  |

**Safety and buzzer**

| Pin  | Signal | Volt  |
| :--: | :----: | :---: |
|  1   | VCC/3.3| +3.3V |
|  2   |  VCC5  |  +5V  |
|  3   | Safety | +3.3V |
|  4   |  LED   | +3.3V |
|  5   | Buzzer | +3.3V |
|  6   |  GND   |  GND  |

**VT Port**

| Pin  |   Signal   | Volt  |
| :--: |   :----:   | :---: |
|  1   |   AV_OUT   |  +5V  |
|  2   | S.AU ctrl. | +3.3V |
|  3   |   9V/12V   | 9V/12V|
|  4   |    GND     |  GND  |

**PWM11 and PWM12**

| Pin  | Signal | Volt  |
| :--: | :----: | :---: |
|  1   | PWM11  | +3.3V |
|  2   |  GND   |  GND  |
|  3   | PWM12  | +3.3V |
|  4   |  GND   |  GND  |

**POWER1 and Power2**

| Pin  |     Signal      | Volt  |
| :--: | :-------------: | :---: |
|  1   |     VCC_IN      |  +5V  |
|  2   |     VCC_IN      |  +5V  |
|  3   | BAT_CRRENT_ADC  | +6.6V |
|  4   | BAT_VOLTAGE_ADC | +6.3V |
|  5   |       GND       |  GND  |
|  6   |       GND       |  GND  |

**Battery Input**

| Pin  |     Signal      |   Volt  |
| :--: | :-------------: |  :---:  |
|  1   |  Battery_VCC    | MAX 30V |
|  2   |  Battery_GND    |   GND   |

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.
|Name|Pin|Function|
|:-|:-|:-|
|SERIAL0|OTG1|USB|
|SERIAL1|TX6/RX6|UART1 (Telem1)|
|SERIAL2|TX3/RX3|UART2 (Telem2 buadrate 921600)|
|SERIAL3|TX1/RX1|UART3 (GNSS)|
|SERIAL4|TX4/RX4|UART4 (Reserve for GNSS2)|
|SERIAL5|TX2/RX2|UART5 (Debug)|

## RC Input
RC input is configured on the RCIN pin by PA15 TIM2_CH1 TIM2 , at one end of the servo rail, marked RC in the above diagram. This pin supports PPM and S.Bus. protocols.

## OSD Support

QioTek AdpetF407 supports OSD using OSD_TYPE 1 (MAX7456 driver).

## PWM Output

The QioTek AdpetF407 AIO supports up to 12 PWM outputs. All 14 PWM outputs have GND on the top row, 5V on the middle row and signal on the bottom row.

The 12 PWM outputs are in 3 groups:

PWM 1 and 4 in group1
PWM 4 and 8 in group2
PWM 9 and 12 in group3

Channels within the same group need to use the same output rate. If any channel in a group uses DShot or then all channels in the group need to use DShot.

## CAN Port

The CAN port is disabled by default. Enable the CAN port setting the parameters CAN_P1_Driver to 1 (DroneCAN protocol).

## Battery Monitoring

The board has two dedicated power monitor ports on 6 pin connectors. The correct battery setting parameters are dependent on the type of power brick which is connected.

The correct battery setting parameters are:

BATT_VOLT_PIN 2
BATT_CURR_PIN 3
BATT_VOLT_MULT 20.000
BATT_AMP_PERVLT 60.000
BATT2_VOLT_PIN 14
BATT2_CURR_PIN 15
BATT2_VOLT_MULT 20.000
BATT2_AMP_PERVLT 60.000

In addition, the builtin voltage divider circuit can be used by Solder pad to switching to share the battery voltage monitoring by power2 support to 6S.

If you want to use the built-in voltage monitor on power 1, you can manually invert the BATT_ VOLT_ PIN to 14, BATT_ CURR_ PIN to 15, BATT2_ VOLT_ PIN to 2, BATT2_ CURR_ PIN to 3.

**Built-in BEC**
The built-in BEC 5V output has a starting voltage of 2S, and 9V/12V has a starting voltage of 3S/4S respectively.

## Compass

The QioTek AdpetF407 has a builtin QMC5883 compass. Due to potential interference the board is usually used with an external I2C compass as part of a GPS/Compass combination.

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

The AdeptF407 auto pilot pre-installed with an ArduPilot compatible bootloader. 
Updates should be done with the *.apj firmware files.
