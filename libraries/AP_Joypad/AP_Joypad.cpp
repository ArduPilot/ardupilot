/*
   Joypad Interface Driver for URUS and Ardupilot.
   Copyright (c) 2017-2018 Hiroshi Takey <htakey@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#if ((CONFIG_HAL_BOARD == HAL_BOARD_URUS) && (CONFIG_SHAL_CORE == SHAL_CORE_APM)) || \
    (CONFIG_HAL_BOARD == HAL_BOARD_SITL) || \
    (CONFIG_HAL_BOARD == HAL_BOARD_PX4) || \
    (CONFIG_HAL_BOARD == HAL_BOARD_LINUX) || \
    (CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS) || \
    (CONFIG_HAL_BOARD == HAL_BOARD_F4LIGHT) || \
    (CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN)

#include "AP_Joypad.h"
#include "AP_Joypad_Backend.h"
#include "AP_Joypad_USB.h"

extern const AP_HAL::HAL& hal;

AP_Joypad::AP_Joypad()
{}

void AP_Joypad::configure(ProcessMode process_mode)
{
    _configure_backends(process_mode);
}

void AP_Joypad::update()
{
    if (!_backends) {
        return;
    }

    for (uint8_t i = 0; i < _backend_count; i++) {
        _backends[i]->update();
    }
}

void AP_Joypad::_configure_backends()
{
    if (_backends_configuring) {
        return;
    }

    _backends_configuring = true;
#if defined(SHAL_CORE_APM16U)
    _add_backend(AP_Joypad_USB::configure(*this));
#endif
    _backends_configuring = false;
}

void AP_Joypad::_configure_backends(ProcessMode process_mode)
{
    _configure_backends();

    for (uint8_t i = 0; i < _backend_count; i++) {
        _backends[i]->process(process_mode);
    }
}

bool AP_Joypad::_add_backend(AP_Joypad_Backend *backend)
{
    if (!backend) {
        return false;
    }

    if (_backend_count == JOYPAD_MAX_BACKENDS) {
#if HAL_CPU_CLASS > HAL_CPU_CLASS_16
        hal.console->printf("JOYPAD: MAX BACKEND REACHED!\n");
#else
#if defined(SHAL_CORE_APM16U)
        while(1);
#else
        hal.console->printf_PS(PSTR("JOYPAD: MAX BACKEND REACHED!\n"));
#endif
#endif
    }

    _backends[_backend_count++] = backend;

    return true;
}

#endif

