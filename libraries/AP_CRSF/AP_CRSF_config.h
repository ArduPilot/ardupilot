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

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_RCProtocol/AP_RCProtocol_config.h>

#ifndef AP_CRSF_PROTOCOL_ENABLED
#define AP_CRSF_PROTOCOL_ENABLED 1
#endif

#ifndef AP_CRSF_OUT_ENABLED
#define AP_CRSF_OUT_ENABLED AP_CRSF_PROTOCOL_ENABLED && AP_RCPROTOCOL_CRSF_ENABLED
#endif

