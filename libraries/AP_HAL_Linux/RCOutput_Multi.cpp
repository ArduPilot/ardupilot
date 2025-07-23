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
  this is a driver for multiple RCOutput methods on one board
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "RCOutput_Multi.h"

#if HAL_LINUX_RCOUTPUT_MULTI_ENABLED

extern const AP_HAL::HAL& hal;

using namespace Linux;


// constructor
RCOutput_Multi::RCOutput_Multi(uint8_t _num_outputs, ...) :
    num_outputs(_num_outputs)
{
    va_list ap;
    outputs = NEW_NOTHROW RCOutputGroup[num_outputs];
    if (outputs == nullptr) {
        AP_HAL::panic("failed to allocated RCOutputGroup array");
    }
    va_start(ap, _num_outputs);
    for (uint8_t i=0; i<num_outputs; i++) {
        RCOutputGroup *group = va_arg(ap, RCOutputGroup *);
        //hal.console->printf("foo: %d\n", group.num_channels);
        outputs[i].output = group->output;
        outputs[i].num_channels = group->num_channels;
        if (outputs[i].output == nullptr) {
            AP_HAL::panic("Bad RCOutput object");
        }
    }
    va_end(ap);
}

void RCOutput_Multi::init()
{
    for (uint8_t i=0; i<num_outputs; i++) {
        outputs[i].output->init();
    }
}

bool RCOutput_Multi::resolve_channel(uint8_t ch, uint8_t & gid, uint8_t &cid)
{
    for ( uint8_t i=0; i<num_outputs; i++) {
        uint8_t group_ch_count = outputs[i].num_channels;
        if (ch < group_ch_count) {
            gid = i;
            cid = ch;
            return true;
        } else {
            ch -= group_ch_count;
        }
    }
    return false;
}


void RCOutput_Multi::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    for ( uint8_t i=0; i<num_outputs; i++) {
        uint8_t group_ch_count = outputs[i].num_channels;
        uint32_t group_chmask = (1<<group_ch_count)-1;
        if (chmask & group_chmask) {
            outputs[i].output->set_freq(chmask & group_chmask, freq_hz);
        }
        chmask >>= group_ch_count;
    }
}

uint16_t RCOutput_Multi::get_freq(uint8_t ch)
{
    for ( uint8_t i=0; i<num_outputs; i++) {
        uint8_t group_ch_count = outputs[i].num_channels;
        if ( ch < group_ch_count ) {
            outputs[i].output->get_freq(ch);
        }
        ch -= group_ch_count;
    }
    return 0;
}

void RCOutput_Multi::enable_ch(uint8_t ch)
{
    for ( uint8_t i=0; i<num_outputs; i++) {
        uint8_t group_ch_count = outputs[i].num_channels;
        if ( ch < group_ch_count ) {
            outputs[i].output->enable_ch(ch);
            break;
        }
        ch -= group_ch_count;
    }
}

void RCOutput_Multi::disable_ch(uint8_t ch)
{
    for ( uint8_t i=0; i<num_outputs; i++) {
        uint8_t group_ch_count = outputs[i].num_channels;
        if ( ch < group_ch_count ) {
            outputs[i].output->disable_ch(ch);
            break;
        }
        ch -= group_ch_count;
    }
}

bool RCOutput_Multi::force_safety_on()
{
    bool ret = true;
    /* Shutdown before sleeping. */
    for ( uint8_t i=0; i<num_outputs; i++) {
        ret &= outputs[i].output->force_safety_on();
    }
    return ret;
}

void RCOutput_Multi::force_safety_off()
{
    for ( uint8_t i=0; i<num_outputs; i++) {
        outputs[i].output->force_safety_off();
    }
}

void RCOutput_Multi::cork()
{
    for ( uint8_t i=0; i<num_outputs; i++) {
        outputs[i].output->cork();
    }
}

void RCOutput_Multi::push()
{
    for ( uint8_t i=0; i<num_outputs; i++) {
        outputs[i].output->push();
    }
}

void RCOutput_Multi::write(uint8_t ch, uint16_t period_us)
{
    for ( uint8_t i=0; i<num_outputs; i++) {
        uint8_t group_ch_count = outputs[i].num_channels;
        if ( ch < group_ch_count ) {
            outputs[i].output->write(ch, period_us);
            return;
        }
        ch -= group_ch_count;
    }
}

uint16_t RCOutput_Multi::read(uint8_t ch)
{
    for ( uint8_t i=0; i<num_outputs; i++) {
        uint8_t group_ch_count = outputs[i].num_channels;
        if ( ch < group_ch_count ) {
            return outputs[i].output->read(ch);
        }
        ch -= group_ch_count;
    }

    return 0;
}

void RCOutput_Multi::read(uint16_t* period_us, uint8_t len)
{
    for ( uint8_t i=0; i<num_outputs; i++) {
        uint8_t group_ch_count = outputs[i].num_channels;
        for ( unsigned c = 0; c < MIN(group_ch_count, len); ++c) {
            period_us[c] = outputs[i].output->read(c);
        }

        if ( len < group_ch_count ) {
            return;
        }

        period_us += group_ch_count;
        len -= group_ch_count;
    }
}

#endif // HAL_LINUX_RCOUTPUT_MULTI_ENABLED

