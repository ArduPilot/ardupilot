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

#pragma once

#include <AP_HAL/AP_HAL.h>
#if ((CONFIG_HAL_BOARD == HAL_BOARD_URUS) && (CONFIG_SHAL_CORE == SHAL_CORE_APM)) || \
    (CONFIG_HAL_BOARD == HAL_BOARD_SITL) || \
    (CONFIG_HAL_BOARD == HAL_BOARD_PX4) || \
    (CONFIG_HAL_BOARD == HAL_BOARD_LINUX) || \
    (CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS) || \
    (CONFIG_HAL_BOARD == HAL_BOARD_F4LIGHT) || \
    (CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN)

#define JOYPAD_MAX_BACKENDS 1

class AP_Joypad_Backend;

class AP_Joypad {

    friend class AP_Joypad_Backend;

public:

    enum ProcessMode {
        auto_process = 0,
        loop_process
    };

    AP_Joypad();

    /** Wrap only, make some people happy, this will be
      * removed in the future.
      * By default init() call to configure() in auto process
      * @param None.
      * @return None.
      */
    void init() {
        configure();
    }

    /** Configure the backend with their implementations.
      * By default configure() set "auto process" mode on
      * process() backend function.
      * @param  process_mode:
      *         [AutoProcess] - Update process run in the scheduled
      *         callback. This is the default mode.
      *         [LoopProcess] - Update process run not in scheduled
      *         callback. udpate() need to be called in somewhere
      *         to see the action, otherwise nothing happen.
      * @return None.
      */
    void configure(ProcessMode process_mode = ProcessMode::auto_process);

    /** Update the backend process.
      * @param  none.
      * @return None.
      */
    void update();

private:

    AP_Joypad_Backend *_backends[JOYPAD_MAX_BACKENDS];
    uint8_t _backend_count;
    bool _backends_configuring:1;

    // load backend drivers
    bool _add_backend(AP_Joypad_Backend *backend);
    void _configure_backends();
    void _configure_backends(ProcessMode process_mode);
};

#endif
