/*
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
 */
/*
  serial options support, for serial over DroneCAN
 */

#include "AP_Periph.h"

#if AP_PERIPH_SERIAL_OPTIONS_ENABLED

#include "serial_options.h"
#include <AP_SerialManager/AP_SerialManager_config.h>

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo SerialOptions::var_info[] {

#if HAL_HAVE_SERIAL0
    // @Group: 0_
    // @Path: serial_options_dev.cpp
    AP_SUBGROUPINFO(devs[0], "0_",  1, SerialOptions, SerialOptionsDev),
#endif

#if HAL_HAVE_SERIAL1
    // @Group: 1_
    // @Path: serial_options_dev.cpp
    AP_SUBGROUPINFO(devs[1], "1_",  2, SerialOptions, SerialOptionsDev),
#endif

#if HAL_HAVE_SERIAL2
    // @Group: 2_
    // @Path: serial_options_dev.cpp
    AP_SUBGROUPINFO(devs[2], "2_",  3, SerialOptions, SerialOptionsDev),
#endif

#if HAL_HAVE_SERIAL3
    // @Group: 3_
    // @Path: serial_options_dev.cpp
    AP_SUBGROUPINFO(devs[3], "3_",  4, SerialOptions, SerialOptionsDev),
#endif

#if HAL_HAVE_SERIAL4
    // @Group: 4_
    // @Path: serial_options_dev.cpp
    AP_SUBGROUPINFO(devs[4], "4_",  5, SerialOptions, SerialOptionsDev),
#endif

#if HAL_HAVE_SERIAL5
    // @Group: 5_
    // @Path: serial_options_dev.cpp
    AP_SUBGROUPINFO(devs[5], "5_",  6, SerialOptions, SerialOptionsDev),
#endif

#if HAL_HAVE_SERIAL6
    // @Group: 6_
    // @Path: serial_options_dev.cpp
    AP_SUBGROUPINFO(devs[6], "6_",  7, SerialOptions, SerialOptionsDev),
#endif

#if HAL_HAVE_SERIAL7
    // @Group: 7_
    // @Path: serial_options_dev.cpp
    AP_SUBGROUPINFO(devs[7], "7_",  8, SerialOptions, SerialOptionsDev),
#endif

#if HAL_HAVE_SERIAL8
    // @Group: 8_
    // @Path: serial_options_dev.cpp
    AP_SUBGROUPINFO(devs[8], "8_",  9, SerialOptions, SerialOptionsDev),
#endif

#if HAL_HAVE_SERIAL9
    // @Group: 9_
    // @Path: serial_options_dev.cpp
    AP_SUBGROUPINFO(devs[9], "9_",  10, SerialOptions, SerialOptionsDev),
#endif
        
    AP_GROUPEND
};

SerialOptions::SerialOptions(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void SerialOptions::init(void)
{
    for (uint8_t i=0; i<ARRAY_SIZE(devs); i++) {
        auto *uart = hal.serial(i);
        if (uart != nullptr) {
            auto &d = devs[i];
            uart->set_options(d.options);
            uart->set_flow_control(AP_HAL::UARTDriver::flow_control(d.rtscts.get()));
        }
    }
}

#endif  // AP_PERIPH_SERIAL_OPTIONS_ENABLED
