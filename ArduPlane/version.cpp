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

#include "Plane.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

#include <AP_Common/AP_FWVersion.h>

const AP_FWVersion AP_FWVersion::fwver{
    .major = FW_MAJOR,
    .minor = FW_MINOR,
    .patch = FW_PATCH,
    .fw_type = (enum FIRMWARE_VERSION_TYPE)(FW_TYPE),
#ifndef GIT_VERSION
    .fw_string = THISFIRMWARE,
#else
    .fw_string = THISFIRMWARE " (" GIT_VERSION ")",
    .fw_hash_str = GIT_VERSION,
#endif
#ifdef CHIBIOS_GIT_VERSION
    .middleware_name = nullptr,
    .middleware_hash_str = nullptr,
    .os_name = "ChibiOS",
    .os_hash_str = CHIBIOS_GIT_VERSION,
#endif
};
