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
#ifdef WITH_SITL_OSD
#include "AP_OSD_SITL.h"
#endif
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

    // @Param: _SW_METHOD
    // @DisplayName: Screen switch method
    // @Description: This sets the method used to switch different OSD screens.
    // @Values: 0: switch to next screen if channel value was changed,
    //          1: select screen based on pwm ranges specified for each screen,
    //          2: switch to next screen after low to high transition and every 1s while channel value is high
    // @User: Standard
    AP_GROUPINFO("_SW_METHOD", 7, AP_OSD, sw_method, AP_OSD::TOGGLE),

    // @Param: _OPTIONS
    // @DisplayName: OSD Options
    // @Description: This sets options that change the display
    // @Bitmask: 0:UseDecimalPack
    // @User: Standard
    AP_GROUPINFO("_OPTIONS", 8, AP_OSD, options, 0),
    
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

    case OSD_MAX7456: {
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

#ifdef WITH_SITL_OSD
    case OSD_SITL: {
        backend = AP_OSD_SITL::probe(*this);
        if (backend == nullptr) {
            break;
        }
        hal.console->printf("Started SITL OSD\n");
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_OSD::timer, void));
        break;
    }
#endif
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

//Thanks to minimosd authors for the multiple osd screen idea
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
    switch (sw_method) {
    //switch to next screen if channel value was changed
    default:
    case TOGGLE:
        if (previous_channel_value == 0) {
            //do not switch to the next screen just after initialization
            previous_channel_value = channel_value;
        }
        if (abs(channel_value-previous_channel_value) > 200) {
            if (switch_debouncer) {
                next_screen();
                previous_channel_value = channel_value;
            } else {
                switch_debouncer = true;
                return;
            }
        }
        break;
    //select screen based on pwm ranges specified
    case PWM_RANGE:
        for (int i=0; i<AP_OSD_NUM_SCREENS; i++) {
            if (screen[i].enabled && screen[i].channel_min <= channel_value && screen[i].channel_max > channel_value) {
                current_screen = i;
                break;
            }
        }
        break;
    //switch to next screen after low to high transition and every 1s while channel value is high
    case AUTO_SWITCH:
        if (channel_value > channel->get_radio_trim()) {
            if (switch_debouncer) {
                uint32_t now = AP_HAL::millis();
                if (now - last_switch_ms > 1000) {
                    next_screen();
                    last_switch_ms = now;
                }
            } else {
                switch_debouncer = true;
                return;
            }
        } else {
            last_switch_ms = 0;
        }
        break;
    }
    switch_debouncer = false;
}

//select next avaliable screen, do nothing if all screens disabled
void AP_OSD::next_screen()
{
    uint8_t t = current_screen;
    do {
        t = (t + 1)%AP_OSD_NUM_SCREENS;
    } while (t != current_screen && !screen[t].enabled);
    current_screen = t;
}

