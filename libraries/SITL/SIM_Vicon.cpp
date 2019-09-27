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
  simple vicon simulator class

  XKFR

*/

#include "SIM_Vicon.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

using namespace SITL;

#define USE_VISION_POSITION_ESTIMATE 1


Vicon::Vicon() :
    SerialDevice::SerialDevice()
{
    if (!valid_channel(mavlink_ch)) {
        AP_HAL::panic("Invalid mavlink channel");
    }
}

void Vicon::maybe_send_heartbeat()
{
    const uint32_t now = AP_HAL::millis();

    if (now - last_heartbeat_ms < 100) {
        // we only provide a heartbeat every so often
        return;
    }
    last_heartbeat_ms = now;

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(system_id,
                               component_id,
                               &msg,
                               MAV_TYPE_GCS,
                               MAV_AUTOPILOT_INVALID,
                               0,
                               0,
                               0);
}

void Vicon::update_vicon_position_estimate(const Location &loc,
                                           const Vector3f &position,
                                           const Quaternion &attitude)
{
    const uint64_t now_us = AP_HAL::micros64();

    if (time_offset_us == 0) {
        time_offset_us = (unsigned(random()) % 7000) * 1000000ULL;
        printf("time_offset_us %llu\n", (long long unsigned)time_offset_us);
    }
    
    if (time_send_us && now_us >= time_send_us) {
        uint8_t msgbuf[300];
        uint16_t msgbuf_len = mavlink_msg_to_send_buffer(msgbuf, &obs_msg);

        if (::write(fd_my_end, (void*)&msgbuf, msgbuf_len) != msgbuf_len) {
            ::fprintf(stderr, "Vicon: write failure\n");
        }
        time_send_us = 0;
    }
    if (time_send_us != 0) {
        // waiting for the last msg to go out
        return;
    }

    if (now_us - last_observation_usec < 70000) {
        // create observations at 70ms intervals (matches EK2 max rate)
        return;
    }

    float roll;
    float pitch;
    float yaw;
    attitude.to_euler(roll, pitch, yaw);

#if USE_VISION_POSITION_ESTIMATE
    // use the more recent VISION_POSITION_ESTIMATE message
    mavlink_msg_vision_position_estimate_pack_chan(
        system_id,
        component_id,
        mavlink_ch,
        &obs_msg,
        now_us + time_offset_us,
        position.x,
        position.y,
        position.z,
        roll,
        pitch,
        yaw,
        NULL, 0);
#else
    mavlink_msg_vicon_position_estimate_pack_chan(
        system_id,
        component_id,
        mavlink_ch,
        &obs_msg,
        now_us + time_offset_us,
        position.x,
        position.y,
        position.z,
        roll,
        pitch,
        yaw,
        NULL);
#endif // USE_VISION_POSITION_ESTIMATE

    uint32_t delay_ms = 25 + unsigned(random()) % 300;
    time_send_us = now_us + delay_ms * 1000UL;
}

/*
  update vicon sensor state
 */
void Vicon::update(const Location &loc, const Vector3f &position, const Quaternion &attitude)
{
    if (!init_sitl_pointer()) {
        return;
    }

    maybe_send_heartbeat();
    update_vicon_position_estimate(loc, position, attitude);
}

