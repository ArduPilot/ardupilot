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
 *
 * AP_OSD partially based on betaflight and inav osd.c implemention.
 * clarity.mcm font is taken from inav configurator.
 * Many thanks to their authors.
 */

/*
  parameter object for one setting in AP_OSD
 */

#include "AP_OSD.h"

const AP_Param::GroupInfo AP_OSD_Setting::var_info[] = {
    // @Param: _EN
    // @DisplayName: Enable
    // @Description: Enable setting
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_EN", 1, AP_OSD_Setting, enabled, 0),

    // @Param: _X
    // @DisplayName: X position
    // @Description: Horizontal position on screen
    // @Range: 0 29
    // @User: Standard
    AP_GROUPINFO("_X", 2, AP_OSD_Setting, xpos, 0),

    // @Param: _Y
    // @DisplayName: Y position
    // @Description: Vertical position on screen
    // @Range: 0 15
    // @User: Standard
    AP_GROUPINFO("_Y", 3, AP_OSD_Setting, ypos, 0),

    AP_GROUPEND
};

// constructor
AP_OSD_Setting::AP_OSD_Setting(bool _enabled, uint8_t x, uint8_t y)
{
    enabled.set(_enabled);
    xpos.set(x);
    ypos.set(y);
}
