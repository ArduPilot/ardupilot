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

Vicon::Vicon()
{
    int tmp[2];
    if (pipe(tmp) == -1) {
        AP_HAL::panic("pipe() failed");
    }
    fd_my_end    = tmp[1];
    fd_their_end = tmp[0];

    // make sure we don't screw the simulation up by blocking:
    fcntl(fd_my_end, F_SETFL, fcntl(fd_my_end, F_GETFL, 0) | O_NONBLOCK);
    fcntl(fd_their_end, F_SETFL, fcntl(fd_their_end, F_GETFL, 0) | O_NONBLOCK);

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
    const uint32_t now = AP_HAL::millis();

    if (now - vicon.last_observation_ms < vicon.observation_interval_ms) {
        return;
    }

    obs_elements new_obs = {
        now,
        position,
        attitude
    };
    vicon.last_observation_ms = now;

    obs_elements observation;
    if (_sitl->vicon_observation_history_length == 0) {
        // no delay
        observation = new_obs;
    } else {
        vicon.observation_history->push(new_obs);

        if (vicon.observation_history->space() != 0) {
            ::fprintf(stderr, "Insufficient delay\n");
            return;
        }
        if (!vicon.observation_history->pop(observation)) {
            abort();
        }
    }

    float roll;
    float pitch;
    float yaw;
    observation.attitude.to_euler(roll, pitch, yaw);

    mavlink_message_t msg;
    const uint16_t n = mavlink_msg_vicon_position_estimate_pack_chan(
        system_id,
        component_id,
        mavlink_ch,
        &msg,
        observation.time_ms*1000,
        observation.position.x,
        observation.position.y,
        observation.position.z,
        roll,
        pitch,
        yaw);

    // ::fprintf(stderr, "Vicon: %u: writing pos=(%03.03f %03.03f %03.03f) att=(%01.03f %01.03f %01.03f)\n", observation.time_ms, observation.position.x, observation.position.y, observation.position.z, roll, pitch, yaw);
    if (::write(fd_my_end, (void*)&msg, n) != n) {
        ::fprintf(stderr, "Vicon: write failure\n");
        // abort();
    }
}

bool Vicon::init_sitl_pointer()
{
    if (_sitl == nullptr) {
        _sitl = (SITL *)AP_Param::find_object("SIM_");
        if (_sitl == nullptr) {
            return false;
        }
    }
    if (_sitl->vicon_observation_history_length > 0 &&
        vicon.observation_history == nullptr) {
        const uint8_t maxlen = 100;
        if (_sitl->vicon_observation_history_length > maxlen) {
            ::fprintf(stderr, "Clamping history length to %u", maxlen);
            _sitl->vicon_observation_history_length = maxlen;
        }
        vicon.observation_history = new ObjectArray<obs_elements>(_sitl->vicon_observation_history_length);
        if (vicon.observation_history == nullptr) {
            AP_HAL::panic("Failed to allocate history");
        }
    }
    return true;
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

