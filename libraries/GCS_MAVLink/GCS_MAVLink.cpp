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

/// @file	GCS_MAVLink.cpp

/*
This provides some support code and variables for MAVLink enabled sketches

*/
#include "GCS.h"
#include "GCS_MAVLink.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#ifdef MAVLINK_SEPARATE_HELPERS
// Shut up warnings about missing declarations; TODO: should be fixed on
// mavlink/pymavlink project for when MAVLINK_SEPARATE_HELPERS is defined
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#include "include/mavlink/v2.0/mavlink_helpers.h"
#pragma GCC diagnostic pop
#endif

AP_HAL::UARTDriver	*mavlink_comm_port[MAVLINK_COMM_NUM_BUFFERS];
bool gcs_alternative_active[MAVLINK_COMM_NUM_BUFFERS];

// per-channel lock
static HAL_Semaphore chan_locks[MAVLINK_COMM_NUM_BUFFERS];
static bool chan_discard[MAVLINK_COMM_NUM_BUFFERS];

mavlink_system_t mavlink_system = {7,1};

// routing table
MAVLink_routing GCS_MAVLINK::routing;

GCS_MAVLINK *GCS_MAVLINK::find_by_mavtype_and_compid(uint8_t mav_type, uint8_t compid, uint8_t &sysid) {
    mavlink_channel_t channel;
    if (!routing.find_by_mavtype_and_compid(mav_type, compid, sysid, channel)) {
        return nullptr;
    }
    return gcs().chan(channel);
}

// set a channel as private. Private channels get sent heartbeats, but
// don't get broadcast packets or forwarded packets
void GCS_MAVLINK::set_channel_private(mavlink_channel_t _chan)
{
    const uint8_t mask = (1U<<(unsigned)_chan);
    mavlink_private |= mask;
}

// return a MAVLink parameter type given a AP_Param type
MAV_PARAM_TYPE GCS_MAVLINK::mav_param_type(enum ap_var_type t)
{
    if (t == AP_PARAM_INT8) {
	    return MAV_PARAM_TYPE_INT8;
    }
    if (t == AP_PARAM_INT16) {
	    return MAV_PARAM_TYPE_INT16;
    }
    if (t == AP_PARAM_INT32) {
	    return MAV_PARAM_TYPE_INT32;
    }
    // treat any others as float
    return MAV_PARAM_TYPE_REAL32;
}


/// Check for available transmit space on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
uint16_t comm_get_txspace(mavlink_channel_t chan)
{
    GCS_MAVLINK *link = gcs().chan(chan);
    if (link == nullptr) {
        return 0;
    }
    return link->txspace();
}

/*
  send a buffer out a MAVLink channel
 */
void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len)
{
    if (!valid_channel(chan) || mavlink_comm_port[chan] == nullptr || chan_discard[chan]) {
        return;
    }
#if HAL_HIGH_LATENCY2_ENABLED
    // if it's a disabled high latency channel, don't send
    GCS_MAVLINK *link = gcs().chan(chan);
    if (link->is_high_latency_link && !gcs().get_high_latency_status()) {
        return;
    }
#endif
    if (gcs_alternative_active[chan]) {
        // an alternative protocol is active
        return;
    }
    const size_t written = mavlink_comm_port[chan]->write(buf, len);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (written < len) {
        AP_HAL::panic("Short write on UART: %lu < %u", (unsigned long)written, len);
    }
#else
    (void)written;
#endif
}

/*
  lock a channel for send
  if there is insufficient space to send size bytes then all bytes
  written to the channel by the mavlink library will be discarded
  while the lock is held.
 */
void comm_send_lock(mavlink_channel_t chan_m, uint16_t size)
{
    const uint8_t chan = uint8_t(chan_m);
    chan_locks[chan].take_blocking();
    if (mavlink_comm_port[chan]->txspace() < size) {
        chan_discard[chan] = true;
        gcs_out_of_space_to_send(chan_m);
    }
}

/*
  unlock a channel
 */
void comm_send_unlock(mavlink_channel_t chan_m)
{
    const uint8_t chan = uint8_t(chan_m);
    chan_discard[chan] = false;
    chan_locks[chan].give();
}

/*
  return reference to GCS channel lock, allowing for
  HAVE_PAYLOAD_SPACE() to be run with a locked channel
 */
HAL_Semaphore &comm_chan_lock(mavlink_channel_t chan)
{
    return chan_locks[uint8_t(chan)];
}
