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

  Simulator for Siyi ZT30

 */

#include "SIM_config.h"

#if AP_SIM_SIYI_ZT30_ENABLED

#include "SIM_Siyi_ZT30.h"

using namespace SITL;

void Siyi_ZT30::update(const Aircraft &aircraft)
{
    gimbal.update(aircraft);

    // base class update function:
    Siyi::update(aircraft);
}

void Siyi_ZT30::handle_message(const PackedMessage<FirmwareVersionRequest> &request)
{
    if (!request.response_requested()) {
        // weird
        AP_HAL::panic("response not requested?");
    }
    const PackedMessage<FirmwareVersion> response{
        FirmwareVersion{
            camera_major, camera_minor, camera_patch,
            gimbal_major, gimbal_minor, gimbal_patch,
            zoom_major, zoom_minor, zoom_patch
        },
    };
    write_to_autopilot((char*)&response, sizeof(response));
}

void Siyi_ZT30::handle_message(const PackedMessage<HardwareIDRequest> &request)
{
    if (!request.response_requested()) {
        // weird
        AP_HAL::panic("response not requested?");
    }
    const PackedMessage<HardwareID> response{
        HardwareID{
            hwid_a, hwid_b
        },
    };
    write_to_autopilot((char*)&response, sizeof(response));
}

void Siyi_ZT30::handle_message(const PackedMessage<GimbalAttitudeRequest> &request)
{
    if (!request.response_requested()) {
        // weird
        AP_HAL::panic("response not requested?");
    }

    uint32_t delta_time_us;
    Vector3f delta_angles;
    Vector3f delta_velocities;
    gimbal.get_deltas(delta_angles, delta_velocities, delta_time_us);
    const Vector3<int16_t> rates_decidegrees(
        (delta_angles.x * RAD_TO_DEG * 1e6 / delta_time_us),
        delta_angles.y * RAD_TO_DEG * 1e6 / delta_time_us,
        delta_angles.z * RAD_TO_DEG * 1e6 / delta_time_us
    );

    Vector3f joint_angles;
    gimbal.get_joint_angles(joint_angles);
    const Vector3<int16_t> joint_angles_decidegrees(
        joint_angles.x * 10 * RAD_TO_DEG,
        joint_angles.y * 10 * RAD_TO_DEG,
        joint_angles.z * 10 * RAD_TO_DEG
    );

    const PackedMessage<GimbalAttitude> response{
        GimbalAttitude{
            joint_angles_decidegrees.x, joint_angles_decidegrees.y, joint_angles_decidegrees.z,
            rates_decidegrees.x, rates_decidegrees.y, rates_decidegrees.z
        },
    };
    write_to_autopilot((char*)&response, sizeof(response));
}

void Siyi_ZT30::handle_message(const PackedMessage<ExternalAttitude> &request)
{
    // MAVProxy and the ArduPilot driver both request an ACK but the
    // cameras don't seem to supply it.

    // if (request.ctrl & 0x1) {
    //     AP_HAL::panic("Response requested for ExternalAttitude?");
    // }
}

void Siyi_ZT30::handle_message(const PackedMessage<SetCameraImageType> &request)
{
    // MAVProxy and the ArduPilot driver both request an ACK but the
    // cameras don't seem to supply it.

    // if (request.ctrl & 0x1) {
    //     AP_HAL::panic("Response requested for SetCameraImageType?");
    // }
}

void Siyi_ZT30::handle_message(const PackedMessage<Photo> &request)
{
    // MAVProxy and the ArduPilot driver both request an ACK but the
    // cameras don't seem to supply it.

    // if (request.ctrl & 0x1) {
    //     AP_HAL::panic("Response requested for SetCameraImageType?");
    // }
}

void Siyi_ZT30::handle_message(const PackedMessage<AcquireGimbalConfigInfo> &request)
{
    // MAVProxy and the ArduPilot driver both request an ACK but the
    // cameras don't seem to supply it.

    // if (request.ctrl & 0x1) {
    //     AP_HAL::panic("Response requested for AcquireGimbalConfigInfo?");
    // }
}

void Siyi_ZT30::handle_message(const PackedMessage<GimbalRotation> &request)
{
    const float max_demanded_yaw_rate_rad = 2*M_PI;
    const float max_demanded_pitch_rate_rad = 2*M_PI;

    const Vector3f demanded_rates{
        max_demanded_yaw_rate_rad * request.msg.yaw_rate_pct * 0.01f,
        max_demanded_pitch_rate_rad * request.msg.pitch_rate_pct * 0.01f,
        0
    };
    gimbal.set_demanded_rates(demanded_rates);

    // MAVProxy and the ArduPilot driver both request an ACK but the
    // cameras don't seem to supply it.

    // if (request.ctrl & 0x1) {
    //     AP_HAL::panic("Response requested for GimbalRotation?");
    // }
}

void Siyi_ZT30::handle_message(const PackedMessage<PositionData> &request)
{
    // MAVProxy and the ArduPilot driver both request an ACK but the
    // cameras don't seem to supply it.

    // if (request.ctrl & 0x1) {
    //     AP_HAL::panic("Response requested for GimbalRotation?");
    // }
}

void Siyi_ZT30::handle_message(const PackedMessage<ReadRangefinderRequest> &request)
{
    const PackedMessage<RangefinderDistance> response{
        RangefinderDistance{50},  // 50m simulated distance
    };
    write_to_autopilot((char*)&response, sizeof(response));
}

void Siyi_ZT30::handle_message(const PackedMessage<SetTime> &request)
{
    epoch_us = request.msg.epoch_us;

    if (!request.response_requested()) {
        // weird
        return;
    }

    const PackedMessage<SetTimeResponse> response{
        SetTimeResponse{1},
    };
    write_to_autopilot((char*)&response, sizeof(response));
}

#endif  // AP_SIM_SIYI_ZT30_ENABLED
