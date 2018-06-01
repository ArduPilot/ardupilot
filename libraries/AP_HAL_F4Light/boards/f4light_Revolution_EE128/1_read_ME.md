this is for Revolution/RevoMini board with removed 25q16 and soldered 25q128 or 25q256. This is very easy!

I tried to maintain compatibility with the OpenPilot documentation. The main difference - FlexiPort can 
be Serial and external I2C port is on pins 7&8 of Input port. But this can be changed by BRD_FLEXI_I2C parameter


Main Port        - telemetry, Serial1. As a variant it can be used as SBUS input with hardware inverter (Parameter BRD_UART_SBUS)
FlexiPort        - OSD, Serial2
Uart6 (pins 5&6) - GPS


Input Port - PWM input is not supported - this is general trend
pin 1 of Input port is GND
pin 2 of Input port is +5
pin 3 of Input port is 1st PPM/SBUS/DSM/SUMD input or Servo9 (if you use RC via UART)
pin 4 of Input port is 2nd PPM/SBUS/DSM/SUMD input or Servo10, also can work as RX-only UART (for GPS)
pins 5&6 of Input port are Tx and Rx of UART6 (for GPS - Serial3)
pins 7&8 of Input port are SCL and SDA of external I2C (or Tx and Rx for SoftSerial if I2C moved to FlexiPort) - or Servos 7&8


Output Port for MOTORs
Connect to PWM output pins in ArduCopter, CleanFlight or OpenPilot order, and set parameter BRD_MOTOR_LAYOUT accordingly

5&6 PWM Output pins are Rx and Tx of Serial4 - but only for quads (except motor layout 1, see below) or planes

also pins 1&2 of OutputPort can be used as servos, in this case connect motors to pins 3-6 in ArduCopter order


PWM input is not supported - this is general trend



OpLink port

DSM satellite can be connected to Oplink port (hardware Serial5) or to PPM inputs (pins 3&4 of input port)

binding of DSM satellite can be done in 2 ways:
1. with some additional hardware - managed stabilizer 3.3 volts. 
2. directly connected to 3.3v, binding will require short power off

Connection to OpLink port (RevoMini)
Pin 1 is Gnd, 
pin 2 is +5   (DSM sat requires 3.3!)
pin 3 is PD2  (pin 54) Rx 
pin 4 is PA15 (pin 50) Enable for 3.3 stab.
pin 5 is PC10 (pin 51) SCK
pin 6 is PC12 (pin 53) MOSI
pin 7 is PC11 (pin 52) MISO

Also Oplink port can be used as external SPI

Airspeed & RSSI 

Pins Servo5 & Servo6 can be used as Analog Input for Airspeed sensor and/or RSSI, connection:

Servo5 = 48
Servo6 = 47


LEDs

All valuable info indicated by 2 LEDs of RevoMini

Blue led: system state
 Very fast blinking                     - initializing
 Blinking                               - ready to arm
 Double blinking                        - pre-arm check failed
 Solid                                  - armed
 blink slowly (around 2Hz) in flight    - battery failsafe
 blink fast  (around 4Hz) in flight     - radio failsafe
 blinking (small dark pauses)           - autotune complete
 double darkening                       - autotune failed
 

Green led: GPS state
 Dark: no fix
 Blinking: number of blinks shows number of sats minus 6, so if there is 10 sats LED will blink in 4 pulses

special modes:
 policy lights                          - ESC calibration or SaveTrim
 short blinks by both LEDs              - compass calibration


this project got donations from:
* Andrea Belloni
* Sebastian Thoem
* Alexander Malishev
* Alexandr Kuzmitski
* Gediminas Ustinavicius
* Thomas Jorgensen

