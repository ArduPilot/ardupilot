# Radian Flight Controller

<img src="https://aerium.bitbucket.io/documentation/assets/images/Aerium-RadianH7-83f2b41071cb173dd3da2982a9e824c1.png#logobig" width="35%" alt="Radian Flight Controller">

---

## Overview

The **Radian** by [Aerium](https://www.aerium.co.il) is a compact, high-performance flight controller designed for professional multirotors, fixed-wing aircraft, and VTOL platforms. Built to Pixhawk standards, it supports ArduPilot, PX4, and Betaflight firmware, ensuring flexibility across a wide range of applications.

Engineered for reliability and versatility, the Radian integrates advanced sensors, robust power management, and extensive I/O—all within a compact 46 × 38 mm carrier board. The detachable core module, measuring just 32 × 32 mm, enables seamless integration into embedded systems where space is at a premium.

Explore the [Aerium Radian product page](https://www.aerium.co.il/product-page/radian) for detailed specifications, imagery, and purchase options.

---

## Technical Specifications

- **Processor:** STM32H743 – ARM Cortex-M7 @ 400 MHz with FPU, 2 MB Flash, 1 MB SRAM  
- **IMUs:** Bosch BMI270 and TDK ICM42688P  
- **Barometers:** Bosch BMP260 and ST LPS22HBTR  
- **Compasses:** Bosch BMM150 and ST LIS3MDL  
- **Firmware Support:** ArduPilot, PX4, and Betaflight  
- **Power Input:** Dual onboard redundant power inputs up to 60V with integrated eFuse, voltage sensing, and DC-DC conversion  
- **Interfaces:**
  - 7 × UART (3.3V TTL, only 3 exposed on the FPV carrier board)
  - 2 × CAN (only 1 exposed on the FPV carrier board)
  - 2 × I2C (only 1 exposed on the FPV carrier board)
  - 3 × SPI (internal)
  - 1 × RC input
  - 16 × PWM/servo outputs (only 8 exposed on the FPV carrier board)
  - 1 × microSD card slot (for logging)
  - 2 × ADC inputs (no exposed on FPV carrier board)
  - 1 × onboard FFC connector for stabilized/dampened heated IMU
- **Weight:** Approximately 20 g  
- **Dimensions:**
  - **Carrier board:** 46 × 38 mm (M4 mounting holes for M3 dampers, 30.5 × 30.5 mm spacing)  
  - **Radian core module:** 32 × 32 mm (M2 mounting holes with integrated standoffs)

<img src="https://aerium.bitbucket.io/documentation/assets/images/radian-dimensions-3eba0ff37b66f5fc1343267414ca2eb3.jpg#diagram" width="35%" alt="Radian Dimensions">

---

## Purchase

- Available at the [Aerium online store](https://www.aerium.co.il/product-page/radian)

---

## Connectors and Pinouts

<img src="https://aerium.bitbucket.io/documentation/assets/images/radian-pinout-top-50f03ed9e3b93c7910bc5317b7327ac3.jpg#diagram" width="35%" alt="Radian Top View">

<img src="https://aerium.bitbucket.io/documentation/assets/images/radian-pinout-bottom-bd16e46157ac40b959d25613e66bdec4.jpg#diagram" width="35%" alt="Radian Bottom View">

For detailed pin definitions and usage, refer to the [Radian documentation portal](https://aerium.bitbucket.io/documentation/Product%20Manuals/Radian/).

---

## UART Port Mapping

| ArduPilot SERIALx | Connector | Function                              |
|-------------------|-----------|----------------------------------------|
| SERIAL1           | TELEM1    | Primary telemetry                      |
| SERIAL2           | TELEM2    | Secondary telemetry or companion link |
| SERIAL3           | GPS       | GNSS module                            |
| SERIAL3           | GPS       | GNSS module                            |
| SERIAL3           | GPS       | GNSS module                            |
| SERIAL3           | GPS       | GNSS module                            |

---

## PWM / Servo Outputs

- Provides 8 independent PWM or servo outputs  
- Functions can be assigned using `SERVOx_FUNCTION` parameters in supported firmware  

---

## RC Input

- Supports PPM-Sum, S.Bus, and iBUS protocols via the RC input port  
- Additional receiver types can be connected via configurable serial ports  
- For setup instructions, consult the [ArduPilot RC systems guide](https://ardupilot.org/copter/docs/common-rc-systems.html)

---

## Power & Battery Monitoring

- Dual onboard voltage sensors for independent monitoring of both power inputs  
- Supports external voltage and current monitoring through power connectors  
- Default firmware configurations are pre-mapped to the correct pins  
- Use onboard VSense jumpers to toggle between internal and external sensing for each input  
- Voltage and current scalers can be adjusted for precise measurements  

---

## Compass Configuration

- Includes dual onboard compass modules for redundancy  
- Additional external compasses can be connected via the I2C interface  

---

## Firmware Installation

1. Connect the Radian to your PC via USB-C  
2. Launch **Mission Planner** or **QGroundControl**  
3. Select the appropriate board and firmware  
4. Click **Install** and follow the guided steps  
5. Once flashed, perform a power cycle using your power module to verify sensor initialization  

> For initial bootloader flashing, connect the USB cable while holding the boot button to enter DFU mode.

---

## Support & Community

- **Technical Support:** [Aerium Contact Page](https://www.aerium.co.il/contact)  
- **Community Forum:** [ArduPilot Discourse](https://discuss.ardupilot.org)  
- **Documentation:** [ArduPilot Wiki](https://ardupilot.org)

---
