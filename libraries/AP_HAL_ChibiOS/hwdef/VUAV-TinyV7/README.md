# VUAV-V7Tiny Flight Controller

The VUAV-V7Tiny  flight controller is manufactured and sold by [V-UAV](http://www.v-uav.com/).

## Features

 - STM32H743 microcontroller
 - Two IMUs: ICM45686,BMI088
 - Internal IST8310 magnetometer
 - Internal ICP-20100 barometer
 - Internal RGB LED
 - MicroSD card slot port
 - 1 Analog power port
 - 1 Battery power port
 - 5 UARTs and 1 USB ports
 - 12 PWM output ports
 - 1 I2C and 1CAN ports
 - Safety switch port
 - Buzzer port
 - RC IN port

##  Pinout

![VUAV-V7Tiny-interface.png](VUAV-V7Tiny-interface.png)

![VUAV-V7Tiny-pinout.png](./png/UAV-V7Tiny-pinout.png)

## UART Mapping
 - SERIAL0 -> USB
 - SERIAL1 -> UART2 (Telem1) (DMA enabled)
 - SERIAL2 -> UART5 (Telem2) (DMA enabled)
 - SERIAL3 -> UART1 (GPS1)
 - SERIAL4 -> UART3 (GPS2)
 - SERIAL5 -> UART7 (Telem3)
 - SERIAL6 -> USB2 (virtual port on same connector)

The Telem1 port has RTS/CTS pins, the other UARTs do not have RTS/CTS.

##  Connectors

### TELEM1 port

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin </th>
   <th>Signal </th>
   <th>Volt </th>
   </tr>
   <tr>
   <td>1</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2</td>
   <td>TX (OUT)</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>3</td>
   <td>RX (IN)</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>4</td>
   <td>CTS</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>5</td>
   <td>RTS</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>6</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>

### TELEM2 ADC port

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin </th>
   <th>Signal </th>
   <th>Volt </th>
   </tr>
   <tr>
   <td>1</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2</td>
   <td>TX (OUT)</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>3</td>
   <td>RX (IN)</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>4</td>
   <td>ADC_3V3</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>5</td>
   <td>ADC_6V6</td>
   <td>+6.6V</td>
   </tr>
   <tr>
   <td>6</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>

### TELEM3 port

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin </th>
   <th>Signal </th>
   <th>Volt </th>
   </tr>
   <tr>
   <td>1</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2</td>
   <td>TX (OUT)</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>3</td>
   <td>RX (IN)</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>4</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>

### GPS1/I2C2 port

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin</th>
   <th>Signal</th>
   <th>Volt</th>
   </tr>
   <tr>
   <td>1</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2</td>
   <td>TX (OUT)</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>3</td>
   <td>RX (IN)</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>4</td>
   <td>SCL I2C2</td>
   <td>+3.3V (pullups)</td>
   </tr>
   <tr>
   <td>5</td>
   <td>SDA I2C2</td>
   <td>+3.3V (pullups)</td>
   </tr>
   <tr>
   <td>6</td>
   <td>SafetyButton</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>7</td>
   <td>SafetyLED</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>8</td>
   <td>-</td>
   <td>-</td>
   </tr>
   <tr>
   <td>9</td>
   <td>Buzzer</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>10</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>

### GPS2/I2C2 port

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin</th>
   <th>Signal</th>
   <th>Volt</th>
   </tr>
   <tr>
   <td>1</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2</td>
   <td>TX (OUT)</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>3</td>
   <td>RX (IN)</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>4</td>
   <td>SCL I2C2</td>
   <td>+3.3V (pullups)</td>
   </tr>
   <tr>
   <td>5</td>
   <td>SDA I2C2</td>
   <td>+3.3V (pullups)</td>
   </tr>
   <tr>
   <td>6</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>

### CAN1port

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin</th>
   <th>Signal</th>
   <th>Volt</th>
   </tr>
   <tr>
   <td>1</td>  
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2</td>
   <td>CAN_H</td>
   <td>+24V</td>
   </tr>
   <tr>
   <td>3</td>
   <td>CAN_L</td>
   <td>+24V</td>
   </tr>
   <tr>
   <td>4</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>

### I2C port

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin</th>
   <th>Signal</th>
   <th>Volt</th>
   </tr>
   <tr>
   <td>1</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2</td>
   <td>SCL I2C4</td>
   <td>+3.3 (pullups)</td>
   </tr>
   <tr>
   <td>3</td>
   <td>SDA I2C4</td>
   <td>+3.3 (pullups)</td>
   </tr>
   <tr>
   <td>4</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>

### POWER

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin</th>
   <th>Signal</th>
   <th>Volt</th>
   </tr>
   <tr>
   <td>1</td>
   <td>VCC IN</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2</td>
   <td>VCC IN</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>3</td>
   <td>CURRENT</td>
   <td>up to +3.3V</td>
   </tr>
   <tr>
   <td>4</td>
   <td>VOLTAGE</td>
   <td>up to +3.3V</td>
   </tr>
   <td>5</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   <td>6</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>

### ESC

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin</th>
   <th>Signal</th>
   <th>Volt</th>
   </tr>
   <tr>
   <td>1</td>
   <td>BAT VCC IN </td>
   <td>+6V to +26V</td>
   </tr>
   <tr>
   <td>2</td>
   <td>CURRENT </td>
   <td>up to +3.3V</td>  
   </tr>
   <tr>
   <td>3</td>
   <td>UART7_RX</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>4</td>
   <td>PWM9</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>5</td>
   <td>PWM10</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>6</td>
   <td>PWM11</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>7</td>
   <td>PWM12</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>8</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>

## RC Input

The RC input is configured on the RCIN pin at one end of the servo rail. This pin supports all unidirectional RC protocols. For bidirectional protocols, such as CRSF/ELRS, SERIAL1~5 can be set to protocol "23" and the receiver can be connected to SERIAL1~5.

## PWM Output

The VUAV-V7Tiny  supports up to 12 PWM outputs,support all PWM protocols. Outputs 1-8support  DShot. Outputs 1-8 support bi-directional Dshot. All 12 PWM outputs have GND on the top row, 5V on the middle row and signal on the bottom row.

The 12 PWM outputs are in 4 groups:

 - PWM 1, 2, 3 and 4 in group1
 - PWM 5, 6, 7 and 8 in group2
 - PWM 9, 10 in group3
 - PWM 11, 12in group4

Channels within the same group need to use the same output rate. If any channel in a group uses DShot, then all channels in that group need to use DShot.

## GPIOs

All 12 PWM channels can be used for GPIO functions (relays, buttons, RPM etc).
The pin numbers for these PWM channels in ArduPilot are shown below:

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>PWM Channels</th>
   <th>Pin</th>
   <th>PWM Channels</th>
   <th>Pin</th>
   </tr>
   <tr>
   <td>PWM1</td>
   <td>50</td>
   <td>PWM7</td>
   <td>56</td>
   </tr>
   <tr>
   <td>PWM2</td>
   <td>51</td>
   <td>PWM8</td>
   <td>57</td>
   </tr>
   <tr>
   <td>PWM3</td>
   <td>52</td>
   <td>PWM9</td>
   <td>58</td>
   </tr>
   <tr>
   <td>PWM4</td>
   <td>53</td>
   <td>PWM10</td>
   <td>59</td>
   </tr>
   <tr>
   <td>PWM5</td>
   <td>54</td>
   <td>PWM11</td>
   <td>60</td>
   </tr>
   <tr>
   <td>PWM6</td>
   <td>55</td>
   <td>PWM12</td>
   <td>61</td>
   </tr>
   </tbody>
   </table>

## Analog inputs

The VUAV-V7Tiny  flight controller has 5 Analog inputs

 - ADC Pin18-> Battery Current
 - ADC Pin4 -> Battery Voltage 
 - ADC Pin19 -> ADC 3V3 Sense
 - ADC Pin5 -> ADC 6V6 Sense
 - ADC Pin10  -> Battery2 Voltage
 - ADC Pin8  -> Servo Voltage

## Battery Monitor Configuration

The board has voltage and current inputs sensor on the POWER and ESC connector.
The correct battery setting parameters are:

Enable POWER monitor:
 - BATT_MONITOR   4
 - BATT_VOLT_PIN 4
 - BATT_CUR_PIN 8
 - BATT_VOLT_MULT 20
 - BATT_AMP_PERVLT 24

Enable ESC monitor (if used):
 - BATT2_MONITOR  3
 - BATT2_VOLT_PIN 10
 - BATT2_VOLT_MULT 10.09

## Loading Firmware

The VUAV-V7Tiny flight controller comes pre-installed with an ArduPilot compatible bootloader, allowing the loading of \*.apj firmware files with any ArduPilot compatible ground station.