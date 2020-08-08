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

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

#include <AP_Common/AP_FWVersion.h>

const AP_FWVersion AP_FWVersion::fwver{
    .major = FW_MAJOR,
    .minor = FW_MINOR,
    .patch = FW_PATCH,
    .fw_type = FW_TYPE,
#ifndef GIT_VERSION
    .fw_string = THISFIRMWARE,
    .fw_hash_str = "",
#else
    .fw_string = THISFIRMWARE " (" GIT_VERSION ")",
    .fw_hash_str = GIT_VERSION,
#endif
    .middleware_name = nullptr,
    .middleware_hash_str = nullptr,
#ifdef CHIBIOS_GIT_VERSION
    .os_name = "ChibiOS",
    .os_hash_str = CHIBIOS_GIT_VERSION,
#else
    .os_name = nullptr,
    .os_hash_str = nullptr,
#endif
#ifdef BUILD_DATE_YEAR
    // encode build date in os_sw_version
   .os_sw_version = (BUILD_DATE_YEAR*100*100) + (BUILD_DATE_MONTH*100) + BUILD_DATE_DAY,
#else
   .os_sw_version = 0,
#endif
};
