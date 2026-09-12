#include "AP_Vehicle.h"

#if AP_VEHICLE_ENABLED

#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>

void AP_Vehicle::load_parameters(AP_Int16 &format_version, const uint16_t expected_format_version)
{
    if (!format_version.load() ||
        format_version != expected_format_version) {

#if CONFIG_HAL_BOARD != HAL_BOARD_ZEPHYR
        // erase all parameters
        hal.console->printf("Firmware change: erasing EEPROM...\n");
        StorageManager::erase();
        AP_Param::erase_all();

        // save the current format version
        format_version.set_and_save(expected_format_version);
        hal.console->printf("done.\n");
#else
        /*
          AP_HAL_Zephyr skips the format-change erase. It was disabled during
          bring-up because early boot on RAM-only storage could not survive
          it. That is no longer the general case: mr_vmu_rt1176 stores
          parameters through the BootROM flash API and the ESP32-S3 / C6 and
          native_sim targets use ZMS.

          TODO(zephyr): re-test per board and delete this branch. Until then a
          Zephyr board keeps its existing parameter table across a firmware
          format change instead of erasing it.
         */
#endif
    }
    format_version.set_default(expected_format_version);

    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
}

#endif  // AP_VEHICLE_ENABLED
