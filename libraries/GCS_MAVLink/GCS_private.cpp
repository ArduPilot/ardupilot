// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  Common GCS MAVLink functions for all vehicle types

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

#include <GCS_private.h>
#include <AP_AHRS.h>
#include "GCS_MAVLink.h"


extern const AP_HAL::HAL& hal;

uint32_t GCS_private::last_radio_status_remrssi_ms;
uint8_t GCS_private::mavlink_active = 0;


GCS_private::GCS_private() : waypoint_receive_timeout(5000)
{

}

// snoop function for vehicle types that want to see messages for
// other targets
void (*GCS_private::msg_snoop)(const mavlink_message_t* msg) = NULL;

void GCS_private::reset_cli_timeout() {
      _cli_timeout = hal.scheduler->millis();
}

/*
  return true if a channel has flow control
 */
bool GCS_private::have_flow_control() {
    // sanity check chan
    if (chan >= MAVLINK_COMM_NUM_BUFFERS) {
        return false;
    }

    if (mavlink_comm_port[chan] == NULL) {
        return false;
    }

    if (chan == MAVLINK_COMM_0) {
        // assume USB console has flow control
        return hal.gpio->usb_connected() || mavlink_comm_port[chan]->get_flow_control() != AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE;
    } else {
        // all other channels
        return mavlink_comm_port[chan]->get_flow_control() != AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE;
    }
}

uint16_t GCS_private::_count_parameters()
{
    // if we haven't cached the parameter count yet...
    if (0 == _parameter_count) {
        AP_Param  *vp;
        AP_Param::ParamToken token;

        vp = AP_Param::first(&token, NULL);
        do {
            _parameter_count++;
        } while (NULL != (vp = AP_Param::next_scalar(&token, NULL)));
    }
    return _parameter_count;
}

void GCS_private::init(AP_HAL::UARTDriver *port, mavlink_channel_t mav_chan)
{
    // sanity check chan
    if (mav_chan >= MAVLINK_COMM_NUM_BUFFERS) {
        return;
    }

    _port = port;
    chan = mav_chan;

    mavlink_comm_port[chan] = _port;
    _queued_parameter = NULL;
    reset_cli_timeout();
}