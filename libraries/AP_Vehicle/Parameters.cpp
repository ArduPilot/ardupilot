#include "AP_Vehicle.h"

#if AP_VEHICLE_ENABLED

#include <AP_Common/AP_FWVersion.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>

void AP_Vehicle::load_parameters(AP_Int16 &format_version, const uint16_t expected_format_version)
{
    if (!format_version.load() ||
        format_version != expected_format_version) {

        // erase all parameters
        hal.console->printf("Firmware change: erasing EEPROM...\n");
        StorageManager::erase();
        AP_Param::erase_all();

        // save the current format version
        format_version.set_and_save(expected_format_version);
        hal.console->printf("done.\n");
    }
    format_version.set_default(expected_format_version);

    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
}

// save the currently running firmware version into a parameter.  Into
// the future we may use this number to prevent a user from attempting
// to upgrade from a truly ancient version of the firmware to a more
// modern one where we may have removed parameter upgrade code.
void AP_Vehicle::check_firmware_version()
{
    const auto &fwver = AP::fwversion();
    const uint32_t current_version = (uint32_t(fwver.major) << 16) |
                                     (uint32_t(fwver.minor) << 8) |
                                     (uint32_t(fwver.patch) << 0);
    const uint32_t mask = 0x00FFFFFF;
    if ((uint32_t(_last_firmware_version.get()) & mask) != current_version) {
        _last_firmware_version.set_and_save(int32_t(current_version));
    }
}

#endif  // AP_VEHICLE_ENABLED
