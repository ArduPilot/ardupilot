# ArduPilot for ESP32

This port supports ESP32, ESP32-S2, and ESP32-S3 boards using the ESP-IDF framework.
Build system support is also available for RISC-V variants (ESP32-C3, C6, H2, P4).

## 1. Environment Setup

The build process uses Waf and CMake, wrapped around the ESP-IDF tools.

### Install Prerequisites (Linux)

**Ubuntu:**

```bash
Tools/environment_install/install-prereqs-ubuntu.sh
```

**Arch Linux:**

```bash
Tools/environment_install/install-prereqs-arch.sh
```

### Install ESP-IDF

We use a submodule for the ESP-IDF to ensure compatibility.

```bash
cd ardupilot
git submodule update --init --recursive

# Install ESP-IDF tools (compilers, python env)
./Tools/scripts/esp32_get_idf.sh
cd modules/esp_idf
./install.sh

# Export environment variables (REQUIRED before every build session)
. ./export.sh
cd ../../..
```

## 2. Building Firmware

The build system mirrors the standard ArduPilot `hwdef` workflow used for STM32 boards.

### Configure and Build

1. Configure for your board:

```bash
./waf configure --board=esp32s3devkit
```

*Available boards are in `libraries/AP_HAL_ESP32/hwdef/`*

2. Build:

```bash
./waf copter
# or
./waf plane
# or
./waf rover
```

### Flashing

Upload the firmware via USB:

```bash
./waf copter --upload
```

* Port Selection: Waf usually auto-detects the port. To specify manually:
  `ESPPORT=/dev/ttyUSB0 ./waf copter --upload`
* Baud Rate: Defaults to 921600. Adjust if needed:
  `ESPBAUD=115200 ./waf copter --upload`

## 3. Board Configuration (hwdef.dat)

**This is the single source of truth for board configuration.**

Unlike older versions of this port, you do **not** need to manually edit `sdkconfig` or C++ headers. All pinouts, sensor configurations, and driver selections are defined in:

`libraries/AP_HAL_ESP32/hwdef/<board_name>/hwdef.dat`

See [MIGRATION.md](MIGRATION.md) for instructions on converting legacy boards to the new system.

### Creating a New Board

1. Copy an existing board directory (e.g., `esp32s3devkit` or `esp32buzz`) to a new folder in `libraries/AP_HAL_ESP32/hwdef/<myboard>`.
2. Edit `hwdef.dat`.
3. Configure and build: `./waf configure --board=<myboard>`.

### hwdef.dat Syntax

The syntax follows the standard ArduPilot `hwdef.dat` format with ESP32-specific extensions.

**Pin Assignments:**

```bash
# Define SPI Bus
SPI1_SCK_PIN 35
SPI1_MISO_PIN 37
SPI1_MOSI_PIN 36

# Define I2C Bus
I2C1_SDA_PIN 16
I2C1_SCL_PIN 15

# Define UARTs (Serial0 is usually Console/USB)
SERIAL0_TX_PIN 43
SERIAL0_RX_PIN 44
```

**Sensor Probing:**

```bash
# Probe for BMP280 barometer on I2C bus 0 at address 0x77
define HAL_BARO_PROBE_LIST PROBE_BARO_I2C(BMP280, 0, 0x77)

# Probe for MPU9250 on SPI bus using CS pin 34
define HAL_INS_PROBE_LIST PROBE_IMU_SPI(Invensense, "mpu9250", ROTATION_NONE)
MPU9250_CS_PIN 34
```

**WiFi Configuration:**

```bash
# Enable WiFi (Defaults to enabled if omitted, but good to be explicit)
define HAL_ESP32_WIFI 1
define WIFI_SSID "MyDrone"
define WIFI_PWD "password123"
```

### Advanced: ESP-IDF Configuration

The `hwdef.dat` file automatically generates `sdkconfig`. If you need to override low-level ESP-IDF settings (like compiler flags or RTOS tweaks), you can add a `sdkconfig.board` file in your board directory, but most common settings are handled by `esp32_hwdef.py`.

## 4. Hardware Reference (Wiring)

### ESP32-S3 DevKit (Default)

See `libraries/AP_HAL_ESP32/hwdef/esp32s3devkit/hwdef.dat` for the exact pinout.

### Legacy "Buzz" Wiring (ESP32 Classic)

*Reference for `esp32buzz` board configuration.*

**GPS (Serial1):**

* TX: GPIO 17
* RX: GPIO 16

**I2C (Compass/Airspeed):**

* SDA: GPIO 13
* SCL: GPIO 12

**RC Input:**

* Pin: GPIO 4 (PPM/SBUS)

**RCOUT (PWM):**

* Motors 1-4: GPIO 25, 27, 33, 32

## 5. Debugging

See `README.s3.debug.howto.md` for detailed JTAG/OpenOCD debugging instructions.

### Analyzing Core Dumps

If the firmware crashes, it may output a base64-encoded core dump to the console.

1. Save the base64 text to a file (e.g., `core.txt`).
2. Run the analysis tool:

```bash
esp-idf/components/espcoredump/espcoredump.py dbg_corefile build/<board>/esp-idf_build/ardupilot.elf -c core.txt -t b64
```
