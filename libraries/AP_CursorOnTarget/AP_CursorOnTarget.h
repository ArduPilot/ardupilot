/*
   Copyright (C) 2023  Kraus Hamdani Aerospace Inc. All rights reserved.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   Author: Tom Pittenger
 */

#pragma once

#include "AP_CursorOnTarget_config.h"

#if AP_CURSORONTARGET_ENABLED

#include <AP_Param/AP_Param.h>

class AP_CursorOnTarget {

public:
    AP_CursorOnTarget();

    /* Do not allow copies */
    AP_CursorOnTarget(const AP_CursorOnTarget &other) = delete;
    AP_CursorOnTarget &operator=(const AP_CursorOnTarget&) = delete;

    static AP_CursorOnTarget *get_singleton() { return _singleton; }

    // update - should be called at least 10Hz
    void update();

    // indicate whether this module is enabled or not
    bool enabled() const { return _params.enabled && _uart != nullptr; }

    static const struct AP_Param::GroupInfo        var_info[];

private:
    static AP_CursorOnTarget* _singleton;
    
    void init();

    bool send_basic_position();

    uint32_t _last_send_basic_position_ms;
    AP_HAL::UARTDriver* _uart;

    struct {
        AP_Int8     enabled;
        AP_Int32    send_basic_position_interval_ms;
    } _params;
};

namespace AP {
    AP_CursorOnTarget *CursorOnTarget();
};

#endif // AP_CURSORONTARGET_ENABLED
