/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// this include file *defines* the structure holding the version information for the ArduPilot binary.  It must only be 
// included in a single place, thus the following protection:

#ifndef FORCE_VERSION_H_INCLUDE
#error AP_FWVersionDefine.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include <AP_Common/AP_FWVersion.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

/*
  allow vendors to set AP_CUSTOM_FIRMWARE_STRING in hwdef.dat
 */
#ifdef AP_CUSTOM_FIRMWARE_STRING
#define ACTIVE_FWSTR AP_CUSTOM_FIRMWARE_STRING
#define ORIGINAL_FWSTR THISFIRMWARE
#else
#define ACTIVE_FWSTR THISFIRMWARE
#define ORIGINAL_FWSTR nullptr
#endif

/**
 * The version number should be used when the structure is updated
 * Major: For breaking changes of the structure
 * Minor: For new fields that does not brake the structure or corrections
 */
const AP_FWVersion AP_FWVersion::fwver{
    // Version header struct
    .header = 0x61706677766572fb, // First 7 MSBs: "apfwver", LSB is the checksum of the previous string: 0xfb
    .header_version = 0x0200U, // Major and minor version
    .pointer_size = static_cast<uint8_t>(sizeof(void*)),
    .reserved = 0,
    .vehicle_type = static_cast<uint8_t>(APM_BUILD_DIRECTORY),
    .board_type = static_cast<uint8_t>(CONFIG_HAL_BOARD),
    .board_subtype = static_cast<uint16_t>(CONFIG_HAL_BOARD_SUBTYPE),
    .major = FW_MAJOR,
    .minor = FW_MINOR,
    .patch = FW_PATCH,
    .fw_type = FW_TYPE,
#ifdef BUILD_DATE_YEAR
    // encode build date in os_sw_version
   .os_sw_version = (BUILD_DATE_YEAR*100*100) + (BUILD_DATE_MONTH*100) + BUILD_DATE_DAY,
#else
   .os_sw_version = 0,
#endif
#ifndef GIT_VERSION
    .fw_string = ACTIVE_FWSTR,
    .fw_hash_str = "",
#else
    .fw_string = ACTIVE_FWSTR " (" GIT_VERSION ")",
    .fw_hash_str = GIT_VERSION,
#endif
#ifndef GIT_VERSION_INT
    .fw_hash = 0,
#else
    .fw_hash = GIT_VERSION_INT,
#endif
    .fw_string_original = ORIGINAL_FWSTR,
    .fw_short_string = ACTIVE_FWSTR,
    .middleware_name = nullptr,
    .middleware_hash_str = nullptr,
#ifdef CHIBIOS_GIT_VERSION
    .os_name = "ChibiOS",
    .os_hash_str = CHIBIOS_GIT_VERSION,
#else
    .os_name = nullptr,
    .os_hash_str = nullptr,
#endif
};
