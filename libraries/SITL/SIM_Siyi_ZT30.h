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
  Simulator for Siyi ZT30

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:siyi_zt30 --speedup=1

param set MNT1_TYPE 8         # siyi
param set CAM1_TYPE 4         # mount
param set SERIAL5_PROTOCOL 8  # gimbal
reboot

long REQUEST_MESSAGE CAMERA_INFORMATION
status *CAM*

*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_SIYI_ZT30_ENABLED

#include "SIM_Siyi.h"
#include "SIM_Gimbal.h"

namespace SITL {

class Siyi_ZT30 : public Siyi {
public:

    void update(const Aircraft &aircraft) override;

protected:

    void handle_message(const PackedMessage<FirmwareVersionRequest> &request) override;
    void handle_message(const PackedMessage<HardwareIDRequest> &request) override;
    void handle_message(const PackedMessage<GimbalAttitudeRequest> &request) override;
    void handle_message(const PackedMessage<ExternalAttitude> &request) override;
    void handle_message(const PackedMessage<SetCameraImageType> &request) override;
    void handle_message(const PackedMessage<Photo> &request) override;
    void handle_message(const PackedMessage<AcquireGimbalConfigInfo> &request) override;
    void handle_message(const PackedMessage<GimbalRotation> &request) override;
    void handle_message(const PackedMessage<PositionData> &request) override;
    void handle_message(const PackedMessage<SetTime> &request) override;
    void handle_message(const PackedMessage<ReadRangefinderRequest> &request) override;

private:

    // the physical gimbal:
    Gimbal gimbal;

    class PACKED FirmwareVersion : Siyi::BaseMessage {
    public:
        FirmwareVersion(
            uint8_t _camera_major, uint8_t _camera_minor, uint8_t _camera_patch,
            uint8_t _gimbal_major, uint8_t _gimbal_minor, uint8_t _gimbal_patch,
            uint8_t _zoom_major, uint8_t _zoom_minor, uint8_t _zoom_patch
            ) : BaseMessage(Siyi::CommandID::ACQUIRE_FIRMWARE_VERSION),
            camera(_camera_major << 16 | _camera_minor << 8 | _camera_patch),
            gimbal(_gimbal_major << 16 | _gimbal_minor << 8 | _gimbal_patch),
            zoom(_zoom_major << 16 | _zoom_minor << 8 | _zoom_patch)
        { }

        uint32_t camera;
        uint32_t gimbal;
        uint32_t zoom;
    };

    class PACKED HardwareID : Siyi::BaseMessage {
    public:
        HardwareID(uint8_t _a, uint8_t _b) :
            BaseMessage(Siyi::CommandID::HARDWARE_ID),
            major{_a},
            minor{_b}
        { }

        uint8_t major;
        uint8_t minor;
    };

    class PACKED GimbalAttitude : Siyi::BaseMessage {
    public:
        GimbalAttitude(int16_t _yaw_decidegrees,
                       int16_t _pitch_decidegrees,
                       int16_t _roll_decidegrees,
                       int16_t _yaw_rate_decidegrees,
                       int16_t _pitch_rate_decidegrees,
                       int16_t _roll_rate_decidegrees
            ) :
            BaseMessage(Siyi::CommandID::ACQUIRE_GIMBAL_ATTITUDE),
            yaw_decidegrees{_yaw_decidegrees},
            pitch_decidegrees{_pitch_decidegrees},
            roll_decidegrees{_roll_decidegrees},
            yaw_rate_decidegreess{_yaw_rate_decidegrees},
            pitch_rate_decidegreess{_pitch_rate_decidegrees},
            roll_rate_decidegreess{_roll_rate_decidegrees}
        { }

        int16_t yaw_decidegrees;
        int16_t pitch_decidegrees;
        int16_t roll_decidegrees;

        int16_t yaw_rate_decidegreess;
        int16_t pitch_rate_decidegreess;
        int16_t roll_rate_decidegreess;
    };

    class PACKED GimbalConfigInfo : Siyi::BaseMessage {
    public:
        GimbalConfigInfo(uint8_t _hdr_status,
                         uint8_t _record_status,
                         uint8_t _motion_mode,
                         uint8_t _mounting_dir,
                         uint8_t _video_mode
            ) :
            BaseMessage(Siyi::CommandID::ACQUIRE_GIMBAL_CONFIG_INFO),
            hdr_status{_hdr_status},
            record_status{_record_status},
            motion_mode{_motion_mode},
            mounting_dir{_mounting_dir},
            video_mode{_video_mode}
        { }

        uint8_t reserved1;
        uint8_t hdr_status;
        uint8_t reserved3;
        uint8_t record_status;
        uint8_t motion_mode;
        uint8_t mounting_dir;
        uint8_t video_mode;
    };

    class PACKED RangefinderDistance : Siyi::BaseMessage {
    public:
        RangefinderDistance(uint16_t _dist_m) :
            BaseMessage(Siyi::CommandID::READ_RANGEFINDER),
            dist_m{_dist_m}
        { }
        uint16_t dist_m;
    };

    class PACKED SetTimeResponse : Siyi::BaseMessage {
    public:
        SetTimeResponse(uint8_t _result) :
            BaseMessage(Siyi::CommandID::SET_TIME),
            result{_result}
        { }

        uint8_t result;
    };

    static constexpr uint8_t hwid_a { '7' };
    static constexpr uint8_t hwid_b { 'A' };

    static constexpr uint8_t camera_major { 1 };
    static constexpr uint8_t camera_minor { 2 };
    static constexpr uint8_t camera_patch { 3 };

    static constexpr uint8_t gimbal_major { 4 };
    static constexpr uint8_t gimbal_minor { 5 };
    static constexpr uint8_t gimbal_patch { 6 };

    static constexpr uint8_t zoom_major { 7 };
    static constexpr uint8_t zoom_minor { 8 };
    static constexpr uint8_t zoom_patch { 9 };

    uint64_t epoch_us;  // from SET_TIME

};  // end class Siyi_ZT30
};  // end namespace SITL

#endif  // AP_SIM_SIYI_ZT30_ENABLED
