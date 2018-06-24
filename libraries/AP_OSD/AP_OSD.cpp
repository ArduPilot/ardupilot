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

#include "AP_OSD.h"
#include "AP_OSD_MAX7456.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Util.h>

#include <utility>

const AP_Param::GroupInfo AP_OSD::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: OSD type
    // @Description: OSD type
    // @Values: 0:None,1:MAX7456
    // @User: Standard
    AP_GROUPINFO_FLAGS("_TYPE", 1, AP_OSD, osd_type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _UPDATE_FONT
    // @DisplayName: Update font
    // @Description: Undate font inside osd chip
    // @Values: 0:Do not update,1:Update
    // @User: Standard
    AP_GROUPINFO("_UPDATE_FONT", 2, AP_OSD, update_font, 1),

    // @Group: 1_
    // @Path: AP_OSD_Screen.cpp
    AP_SUBGROUPINFO(screen[0], "1_", 3, AP_OSD, AP_OSD_Screen),

    // @Group: 2_
    // @Path: AP_OSD_Screen.cpp
    AP_SUBGROUPINFO(screen[1], "2_", 4, AP_OSD, AP_OSD_Screen),

    // @Group: 2_
    // @Path: AP_OSD_Screen.cpp
    AP_SUBGROUPINFO(screen[2], "3_", 5, AP_OSD, AP_OSD_Screen),

    // @Group: 2_
    // @Path: AP_OSD_Screen.cpp
    AP_SUBGROUPINFO(screen[3], "4_", 6, AP_OSD, AP_OSD_Screen),
    
    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

AP_OSD::AP_OSD() : backend(nullptr), last_update_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_OSD::init()
{
    switch ((enum osd_types)osd_type.get()) {
    case OSD_NONE:
        break;

    case OSD_MAX7456:
        AP_HAL::OwnPtr<AP_HAL::Device> spi_dev = std::move(hal.spi->get_device("osd"));
        if (!spi_dev) {
            break;
        }
        backend = AP_OSD_MAX7456::probe(*this, std::move(spi_dev));
        if (backend == nullptr) {
            break;
        }
        hal.console->printf("Started MAX7456 OSD\n");
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_OSD::timer, void));
        break;
    }
}

void AP_OSD::timer()
{
    uint32_t now = AP_HAL::millis();

    if (now - last_update_ms > 100) {
        last_update_ms = now;
        update_osd();
    }
}

void AP_OSD::update_osd()
{
    backend->clear();

    screen[current_screen].set_backend(backend);
    screen[current_screen].draw();

    backend->flush();
}

