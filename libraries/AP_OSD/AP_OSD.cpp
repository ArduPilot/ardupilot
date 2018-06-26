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
#include <RC_Channel/RC_Channel.h>

#include <utility>

const AP_Param::GroupInfo AP_OSD::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: OSD type
    // @Description: OSD type
    // @Values: 0:None,1:MAX7456
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("_TYPE", 1, AP_OSD, osd_type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _CHAN
    // @DisplayName: Screen switch transmitter channel
    // @Description: This sets the channel used to switch different OSD screens.
    // @Values: 0:Disable,5:Chan5,6:Chan6,7:Chan7,8:Chan8,9:Chan9,10:Chan10,11:Chan11,12:Chan12,13:Chan13,14:Chan14,15:Chan15,16:Chan16
    // @User: Standard
    AP_GROUPINFO("_CHAN", 2, AP_OSD, rc_channel, 0),

    // @Group: 1_
    // @Path: AP_OSD_Screen.cpp
    AP_SUBGROUPINFO(screen[0], "1_", 3, AP_OSD, AP_OSD_Screen),

    // @Group: 2_
    // @Path: AP_OSD_Screen.cpp
    AP_SUBGROUPINFO(screen[1], "2_", 4, AP_OSD, AP_OSD_Screen),

    // @Group: 3_
    // @Path: AP_OSD_Screen.cpp
    AP_SUBGROUPINFO(screen[2], "3_", 5, AP_OSD, AP_OSD_Screen),

    // @Group: 4_
    // @Path: AP_OSD_Screen.cpp
    AP_SUBGROUPINFO(screen[3], "4_", 6, AP_OSD, AP_OSD_Screen),

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

AP_OSD::AP_OSD()
{
    AP_Param::setup_object_defaults(this, var_info);
    // default first screen enabled
    screen[0].enabled = 1;
}

void AP_OSD::init()
{
    switch ((enum osd_types)osd_type.get()) {
    case OSD_NONE:
    default:
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

    if (now - last_update_ms >= 100) {
        last_update_ms = now;
        update_osd();
    }
}

void AP_OSD::update_osd()
{
    backend->clear();

    update_current_screen();

    screen[current_screen].set_backend(backend);
    screen[current_screen].draw();

    backend->flush();
}

void AP_OSD::update_current_screen()
{
    if (rc_channel == 0) {
        return;
    }

    RC_Channel *channel = RC_Channels::rc_channel(rc_channel-1);
    if (channel == nullptr) {
        return;
    }

    int16_t channel_value = channel->get_radio_in();
    for (uint8_t i=0; i<AP_OSD_NUM_SCREENS; i++) {
        if (screen[i].enabled && screen[i].channel_min <= channel_value && screen[i].channel_max > channel_value) {
            current_screen = i;
            break;
        }
    }
}

