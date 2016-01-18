/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __AP_BOARDCONFIG_H__
#define __AP_BOARDCONFIG_H__

#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Param.h>

class AP_BoardConfig
{
public:
    // constructor
    AP_BoardConfig(void)
    {
		AP_Param::setup_object_defaults(this, var_info);
    };

    void init(void);

    static const struct AP_Param::GroupInfo var_info[];

private:
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    AP_Int8 _pwm_count;
    AP_Int8 _ser1_rtscts;
    AP_Int8 _ser2_rtscts;
    AP_Int8 _safety_enable;
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

#endif
};

#endif // __AP_BOARDCONFIG_H__


