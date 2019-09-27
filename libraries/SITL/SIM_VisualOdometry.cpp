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
  visual odometry simulator class
*/

#include "SIM_VisualOdometry.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

using namespace SITL;

VisualOdometry::VisualOdometry()
{
    int tmp[2];
    if (pipe(tmp) == -1) {
        AP_HAL::panic("pipe() failed");
    }
    fd_my_end    = tmp[1];
    fd_their_end = tmp[0];

    // close file descriptors on exec:
    fcntl(fd_my_end, F_SETFD, FD_CLOEXEC);
    fcntl(fd_their_end, F_SETFD, FD_CLOEXEC);

    // make sure we don't screw the simulation up by blocking:
    fcntl(fd_my_end, F_SETFL, fcntl(fd_my_end, F_GETFL, 0) | O_NONBLOCK);
    fcntl(fd_their_end, F_SETFL, fcntl(fd_their_end, F_GETFL, 0) | O_NONBLOCK);

    if (!valid_channel(mavlink_ch)) {
        AP_HAL::panic("Invalid mavlink channel");
    }
}

void VisualOdometry::maybe_send_heartbeat()
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

void VisualOdometry::update_vision_position_estimate(Vector3f &position, Quaternion &attitude)
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
            ::fprintf(stderr, "VisualOdometry: write failure\n");
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

    // VISION_POSITION_ESTIMATE message
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

    uint32_t delay_ms = 25 + unsigned(random()) % 300;
    time_send_us = now_us + delay_ms * 1000UL;
}

bool VisualOdometry::init_sitl_pointer()
{
    if (_sitl == nullptr) {
        _sitl = AP::sitl();
        if (_sitl == nullptr) {
            return false;
        }
    }
    return true;
}

void VisualOdometry::save_original_state(const Vector3f &position, const Quaternion &attitude)
{
    original_position = position;
    original_attitude = attitude;
}
/*
 * simulate the normal behaviour of visual odometry
 */
void VisualOdometry::simulate_normal_behaviour(Vector3f &simulated_position,
                                        Quaternion &simulated_attitude,
                                        const Vector3f &true_position,
                                        const Quaternion &true_attitude,
                                        float scale_factor)
{
    // apply offsets so that the origin is (0,0,0) and heading is 0
    simulated_position = true_position - original_position;
    simulated_attitude = true_attitude * original_attitude.inverse();

    // position from visual odometry might be off from ground truth by a scale factor
    const Vector3f scale(scale_factor, scale_factor, scale_factor);
    simulated_position *= scale;
}

/*
  update simulated visual odometry state based on true state
 */
void VisualOdometry::update(const Location &true_loc, const Vector3f &true_position, const Quaternion &true_attitude)
{
    if (!init_sitl_pointer()) {
        return;
    }

    // use SIM_VISO_ENABLE param to start and stop sending visual odometry messages
    if (_sitl->viso_enable) {
        Vector3f simulated_position;
        Quaternion simulated_attitude;

        maybe_send_heartbeat();

        simulate_normal_behaviour(simulated_position, simulated_attitude, true_position, true_attitude, _sitl->viso_scale_factor);

        // different scenarios that might happen during operation
        // for now, we only handle position-related cases
        if (_sitl->viso_error_type == SITL::SITL::SITL_VISOFail_Diverge) {
            // when divergence happens, position will gradually move to infinity from last good position
            static Vector3f diverged_position = simulated_position;

            diverged_position *= _sitl->viso_divergence_rate;

            simulated_position = diverged_position;
        } else if (_sitl->viso_error_type == SITL::SITL::SITL_VISOFail_Jump) {
            // when loop closure or relocalization occur, position will suddenly jump erratically in any direction
            // after which the position will resume normal behavior.
            static Vector3f jump_vector(1, 1, 1); // an example jump of 1m in all directions

            simulated_position += jump_vector;
        }

        update_vision_position_estimate(simulated_position, simulated_attitude);
    } else {
        // original position and heading of visual odometry are reset everytime viso is enabled
        save_original_state(true_position, true_attitude);
    }
}