
# Aeromind6X

The Aeromind6X autopilot is manufactured by [Arkin Labs Private Limited](https://www.arkinlabs.in/)

The Aeromind6X flight controller is a cutting-edge, indigenous solution engineered to deliver exceptional performance for autonomous drones. Featuring **Triple Redundant IMUs**, the Aeromind6X ensures uninterrupted flight even in the event of hardware failures, significantly enhancing the reliability and safety of critical missions. Its **Integrated High-Speed Ethernet** capabilities provide a robust framework for real-time control with intelligent onboard interfaces, enabling rapid data transfer to support complex flight operations. Designed with an eye toward future-proofing, the Aeromind6X also integrates seamlessly with the **M3 Coprocessor**, allowing for the addition of sensors and peripherals, which guarantees adaptability for a wide range of UAV applications.

<!-- ![Aeromind6X](fc.png) -->

<img 
  src="fc.png" 
  alt="FC Image" 
  style="width: 50%; height: auto; position: relative; margin-left: 10px;" 
/>

Specifications
======

### Main Processors
- FMU Processor: `STM32H753`  
  - 32-bit Arm® Cortex®-M7 @ 480 MHz  
  - 2 MB Flash | 1 MB RAM  
- Co-Processor: `STM32F10x`  
- IO Processor: `STM32F103`  
  - 32-bit Arm® Cortex®-M3 @ 24 MHz  
  - 8 KB SRAM  

### Security
- NXP EdgeLock SE051  
  - IEC62443-4-2 certified  
  - CC EAL6+ certified secure element  
  - 46 KB user memory (up to 104 KB customizable)  
  - AES and 3DES encryption/decryption  

### Onboard Sensors
- Accelerometer / Gyroscope: `3x ICM-4568 (Rev. 8)`  
- Magnetometer: `PNI RM3100`  
- Barometer: `2x ICP20100`  

---

### Electrical Specifications
- Max Input Voltage: 6 V  
- USB Power Input: 4.75 V ~ 5.25 V  

---

### Mechanical Specifications
- Weight: 150 g  
- Dimensions: 94 mm × 46 mm × 38 mm  





Interfaces 
========
### Debug Ports
- FMU-Debug  
- IO-Debug  

### Communication
- Ethernet  
- **Telemetry**: 3x Telemetry Ports  
- **CAN**: 2x UAVCAN  

### Actuator Outputs
- **Main Outputs**: 8 Channels  
- **Aux Outputs**: 8 Channels  

### RC Input
- SBUS / DSM / RSSI  
- PPM Input  

### Power Inputs
- 2x SMBUS (I2C)  
- 2x UAVCAN  
- SBUS In  
- PPM In  

### Other Ports
- 2x USB Ports  - 1 JST GH + 1 Type C
- 1x TF Card Slot (Supports MicroSD for flight logs)  


Pinout
======
![Pinout](Pinout.png)

UART Mapping
============

| SERIAL Port | UART Interface | Label / Function   | Flow Control |
|-------------|----------------|--------------------|---------------|
| SERIAL1     | UART7          | TELEM1             | Yes           |
| SERIAL2     | UART5          | TELEM2             | Yes           |
| SERIAL3     | UART1          | GPS1               | No            |
| SERIAL4     | UART8          | GPS2               | No            |
| SERIAL5     | USART2         | TELEM3             | Yes           |
| SERIAL6     | UART4          | UART4 (General Use)| No            |
| SERIAL7     | UART3          | FMU_DEBUG          | No            |


UART7, USART2 and UART5 have flow control pins.

RC input
========
<!--This is the most difficult section and varies widely.-->
RC input is configured via parameters and is controlled via IO Processor. In addition to this, the Autopilot has separate ports dedicated each for RC protocols such as SBUS, PPM. 

PWM Outputs
===========
The controller supports up to 16 PWM outputs.
First 8 outputs (labelled M1 to M8) are controlled by a dedicated STM32F103 IO controller. The remaining 8 outputs (A 1 to 8) are the "auxiliary" outputs. These are directly attached to the STM32H753 FMU controller .

Battery Monitoring
===========
The board has two dedicated power monitor ports on 6 pin connectors. The correct battery setting parameters are dependent on the type of power brick which is connected.

Compass
=======
The Autopilot INS module has PNI RM3100 as internal compass source.


Analog Ports
===============
Autopilot has two native ADC ports - ADC3V3 and ADC6V6. Refer the pin out diagram.


Firmware
========
Firmware for the Aeromind6X is available from [ArduPilot Firmware Server](https://firmware.ardupilot.org) under the `Aeromind6X` target.

Loading Firmware
================
The board comes pre-installed with an ArduPilot compatible bootloader, allowing the loading of *.apj firmware files with any ArduPilot compatible ground station.