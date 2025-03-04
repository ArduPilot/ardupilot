# [NarinFC-H7 VOLOLAND CO., LTD](https://vololand.com/pages/product/computer "NarinFC-H7")

The NarinFC-H7 is a flight controller produced by [VOLOLAND CO., LTD](https://vololand.com "VOLOLAND CO., LTD")

![NarinFC-H7](./images/NarinFC_Header.jpg "NarinFC")

##
## Features
![Basic Parameters](./images/1.Basic_Parameters.png "Basic Parameters")

##
## Outline Dimensions
![Outline Dimensions](./images/2.Outline_Dimensions.png "Outline Dimensions")

##
## Wiring Diagram
![Wire Diagram](./images/3.Wire_Diagram.png "Wire Diagram")

##
## UART Mapping (Port Diagram & Pin outs)
The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the receive pin for UARTn. The Tn pin is the transmit pin for UARTn.
  - SERIAL0 -> USB
  - SERIAL1 -> USART(TELEM1)
  - SERIAL2 -> USART(TELEM2)
  - SERIAL3 -> USART(GPS1) 
  - SERIAL4 -> USART(GPS2)
  - SERIAL5 -> USART(UESR)
  - SERIAL6 -> USART(UESR)
  - SERIAL7 -> UART(DEBUG)

![Port Diagram & Pin outs](./images/4.Port_Diagram_Pin_outs_Diagram-A.png "Port Diagram-A")

#### 1. TELEM1, TELEM2 Port

![TELEM Pinout](./images/4.1.TELEM1,TELEM2_Port_JST_GH_6P_Connector.png "TELEM Pinout")

  - JST GH 6P connector
  - TELEMETRY Port

#### 2. CAN1, CAN2 Port

![CAN Port](./images/4.2.CAN1,CAN2_Port_JST_HG_4P_Connector.png "CAN Port")

  - JST GH 4P connector
  - Communication Protocol: UAVCAN v0 (default), UAVCAN v1 (limited support)
  - Power Supply: Typically provides 5V or 12V output
  - Pin Configuration: Usually includes CAN_H, CAN_L, VCC, and GND

#### 3. I2C, I2C2, I2C3, I2C4 Port

![I2C Port](./images/4.3.I2C1,I2C2,I2C3,I2C4_Port_JST_GH_4P_Connector.png "I2C Port")

  - JST GH 4P connector

#### 4. UART4 Port

![UART Port](./images/4.4.UART4_Port_JST_GH_6P_Connector.png "UART Port")
  
  - JST GH 6P connector

#### 5. GPS & Safety Port

![GPS & Safety Port](./images/4.5.GPS_Safety_Port_JST_GH_10P_Connector.png "GPS & Safety Port")

  - JST GH 10P connector
  - GPS NODMA

![Port Diagram & Pin outs](./images/4.Port_Diagram_Pin_outs_Diagram-B.png "Port Diagram-B")

#### 6. PWM Out

![PWM Out](./images/4.6.PWM_Out_M1-M14.png "PWM Out")

  - 2.54mm pitch Dupont connector
  - M1-M14 PWM Output
  - M13 NODMA, M14 NODMA

#### 7. Power Input

![Power Input](./images/4.7.Power_Input.png "Power Input")

  - 2mm pitch Dupont connector

#### 8. DEBUG Port

![DEBUG Port](./images/4.8.DEBUG_Port_JST_HG_6P_Connector.png "DEBUG Port")
  
  - JST GH 6P connector
  - DEBUG PORT
  - DEBUG NODMA

#### 9. USB Port
  - USB C Type

#### 10. SPI Port

![SPI Port](./images/4.10.SPI_Port_JST_GH_7P_Connector.png "SPI Port")

  - JST GH 7P connector
  - SPI Port

#### 11. SD CARD
  - SD CARD

##
## Loading Firmware
Firmware for these boards can be found at https://firmware.ardupilot.org in sub-folders labeled “NarinFC-H7”.

The board comes pre-installed with an NarinFC-H7 bootloader, allowing the loading of *.apj firmware files with any ArduPilot compatible ground station.

you can update firmware with Mission Planner.

<br>

#
# [VOLOLAND CO., LTD](https://vololand.com "VOLOLAND CO., LTD")

