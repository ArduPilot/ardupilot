# ESP32 Board Migration Guide: Upgrading to Advanced Build Logic

This document outlines how to upgrade an ESP32 board from the "Simple" V1 build system (current master) to the "Advanced" V2 system.

## 1. The V2 Activation Trigger

The build system automatically detects which logic to use based on your `hwdef.dat`.

* V1 Mode (Compatibility): If the `MCU` directive is **missing**, the script emulates current master behavior. It accepts existing `ESP32_...` syntax and relies on static environment templates.
* V2 Mode (Advanced): Adding the `MCU` directive (e.g., `MCU ESP32S3`) activates the advanced data-driven engine.

## 2. Benefits of Upgrading to V2

By adding the `MCU` tag, you unlock several features without needing to change your existing pin configuration:

1. **Dynamic sdkconfig:** You can add `FLASH_SIZE_MB 8` or `PSRAM_SIZE 4MB` directly to `hwdef.dat`, eliminating the need for manual background template edits.
2. **Strict Validation:** Every pin assignment is checked against a chip capability database to catch conflicts and "impossible" assignments at compile time.
3. **RISC-V Support:** Enables builds for ESP32-C3, C6, and P4 variants using the same syntax.
4. **Feature Flags:** Automatically defines C++ capability flags (e.g., `HAL_ESP32_HAS_MCPWM`) based on the detected hardware.

## 3. Migration Steps

### Step 1: Add the MCU Directive

Add the target chip type to the top of your `hwdef.dat`:

```bash
MCU ESP32S3  # Options: ESP32, ESP32S2, ESP32S3, ESP32C3, ESP32C6, ESP32P4
```

### Step 2: Migrate Board-Specific Configuration (Optional)

Previously, board-specific settings like Flash size and PSRAM were often hardcoded in the shared `sdkconfig.defaults` for a whole chip family. These should now be moved to each board's `hwdef.dat`:

```bash
FLASH_SIZE_MB 8
PSRAM_SIZE 4MB
```

The build system will generate a board-specific `sdkconfig.board` in the build directory that contains these values, ensuring correct hardware support without manual template edits.

*(No other tag changes are required, though you can now also use the `RESERVED_PINS` directive for extra safety).*

## 4. Verification

1. Run: `./waf configure --board=<yourboard>`
2. Verify `build/hwdef.h` contains the `#define HAL_ESP32_HWDEF_V2 1` flag.
3. Check that hardware features (like `HAL_ESP32_HAS_MCPWM`) are correctly defined for your chip.
4. Build: `./waf copter`.
CI Reset: 2026-02-22
