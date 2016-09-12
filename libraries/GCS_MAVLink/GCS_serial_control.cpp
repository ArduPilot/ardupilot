// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  MAVLink SERIAL_CONTROL handling
 */

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


#include <AP_HAL/AP_HAL.h>
#include "GCS.h"
#include <DataFlash/DataFlash.h>

extern const AP_HAL::HAL& hal;

/**
   handle a SERIAL_CONTROL message
 */
void GCS_MAVLINK::handle_serial_control(mavlink_message_t *msg, AP_GPS &gps)
{
    mavlink_serial_control_t packet;
    mavlink_msg_serial_control_decode(msg, &packet);

    AP_HAL::UARTDriver *port = NULL;
    AP_HAL::Stream *stream = NULL;

    if (packet.flags & SERIAL_CONTROL_FLAG_REPLY) {
        // how did this packet get to us?
        return;
    }

    bool exclusive = (packet.flags & SERIAL_CONTROL_FLAG_EXCLUSIVE) != 0;

    switch (packet.device) {
    case SERIAL_CONTROL_DEV_TELEM1:
        stream = port = hal.uartC;
        lock_channel(MAVLINK_COMM_1, exclusive);
        break;
    case SERIAL_CONTROL_DEV_TELEM2:
        stream = port = hal.uartD;
        lock_channel(MAVLINK_COMM_2, exclusive);
        break;
    case SERIAL_CONTROL_DEV_GPS1:
        stream = port = hal.uartB;
        gps.lock_port(0, exclusive);
        break;
    case SERIAL_CONTROL_DEV_GPS2:
        stream = port = hal.uartE;
        gps.lock_port(1, exclusive);
        break;
    case SERIAL_CONTROL_DEV_SHELL:
        stream = hal.util->get_shell_stream();
        break;
    default:
        // not supported yet
        return;
    }
    
    if (exclusive && port != NULL) {
        // force flow control off for exclusive access. This protocol
        // is used to talk to bootloaders which may not have flow
        // control support
        port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    }

    // optionally change the baudrate
    if (packet.baudrate != 0 && port != NULL) {
        port->begin(packet.baudrate);
    }

    // write the data
    if (packet.count != 0) {
        if ((packet.flags & SERIAL_CONTROL_FLAG_BLOCKING) == 0) {
            stream->write(packet.data, packet.count);
        } else {
            const uint8_t *data = &packet.data[0];
            uint8_t count = packet.count;
            while (count > 0) {
                while (stream->txspace() <= 0) {
                    hal.scheduler->delay(5);
                }
                uint16_t n = stream->txspace();
                if (n > packet.count) {
                    n = packet.count;
                }
                stream->write(data, n);                
                data += n;
                count -= n;
            }
        }
    }

    if ((packet.flags & SERIAL_CONTROL_FLAG_RESPOND) == 0) {
        // no response expected
        return;
    }

    uint8_t flags = packet.flags;

more_data:
    // sleep for the timeout
    while (packet.timeout != 0 && 
           stream->available() < (int16_t)sizeof(packet.data)) {
        hal.scheduler->delay(1);
        packet.timeout--;
    }

    packet.flags = SERIAL_CONTROL_FLAG_REPLY;

    // work out how many bytes are available
    int16_t available = stream->available();
    if (available < 0) {
        available = 0;
    }
    if (available > (int16_t)sizeof(packet.data)) {
        available = sizeof(packet.data);
    }
    if (available == 0 && (flags & SERIAL_CONTROL_FLAG_BLOCKING) == 0) {
        return;
    }

    if (packet.flags & SERIAL_CONTROL_FLAG_BLOCKING) {
        while (!HAVE_PAYLOAD_SPACE(chan, SERIAL_CONTROL)) {
            hal.scheduler->delay(1);
        }
    } else {
        if (!HAVE_PAYLOAD_SPACE(chan, SERIAL_CONTROL)) {
            // no space for reply
            return;
        }
    }

    // read any reply data
    packet.count = 0;
    memset(packet.data, 0, sizeof(packet.data));
    while (available > 0) {
        packet.data[packet.count++] = (uint8_t)stream->read();
        available--;
    }

    // and send the reply
    _mav_finalize_message_chan_send(chan, 
                                    MAVLINK_MSG_ID_SERIAL_CONTROL,
                                    (const char *)&packet,
                                    MAVLINK_MSG_ID_SERIAL_CONTROL_MIN_LEN,
                                    MAVLINK_MSG_ID_SERIAL_CONTROL_LEN,
                                    MAVLINK_MSG_ID_SERIAL_CONTROL_CRC);
    if ((flags & SERIAL_CONTROL_FLAG_MULTI) && packet.count != 0) {
        if (flags & SERIAL_CONTROL_FLAG_BLOCKING) {
            hal.scheduler->delay(1);
        }
        goto more_data;
    }
}
