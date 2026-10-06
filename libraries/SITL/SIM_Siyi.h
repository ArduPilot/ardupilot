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
  Base class for Siyi camera simulators
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_SIYI_ENABLED

#include "SIM_Mount.h"

#include <AP_Math/crc.h>
#include <AP_Common/Location.h>
#include <AP_InternalError/AP_InternalError.h>

namespace SITL {

class Siyi : public Mount {
public:

    void update(const class Aircraft &aircraft) override;

protected:

    static constexpr uint8_t HEADER1 { 0x55 };
    static constexpr uint8_t HEADER2 { 0x66 };

    template <typename T>
    class PACKED PackedMessage {
    public:
        PackedMessage(T _msg) :
            msg(_msg)
        {
            update_checksum();
        }
        uint8_t header1 { HEADER1 };
        uint8_t header2 { HEADER2 };
        uint8_t ctrl;
        uint16_t datalen {sizeof(T)-1 };  // -1 to exclude command ID
        uint16_t seq;
        T msg;
        uint16_t checksum;

        uint16_t calculate_checksum(uint16_t len) const WARN_IF_UNUSED {
            return crc16_ccitt((uint8_t*)this, len, 0);
        }
        uint16_t calculate_checksum() const WARN_IF_UNUSED {
            return calculate_checksum(sizeof(*this)-2);
        }

        void update_checksum() {
            checksum = calculate_checksum();
        }

        bool response_requested() const { return (ctrl & 0x1) == 0x1; }
    };

    // message ids
    enum class CommandID : uint8_t {
        ACQUIRE_FIRMWARE_VERSION = 0x01,
        HARDWARE_ID = 0x02,
        // AUTO_FOCUS = 0x04,
        // MANUAL_ZOOM_AND_AUTO_FOCUS = 0x05,
        // MANUAL_FOCUS = 0x06,
        GIMBAL_ROTATION = 0x07,
        // CENTER = 0x08,
        ACQUIRE_GIMBAL_CONFIG_INFO = 0x0A,
        // FUNCTION_FEEDBACK_INFO = 0x0B,
        PHOTO = 0x0C,
        ACQUIRE_GIMBAL_ATTITUDE = 0x0D,
        // ABSOLUTE_ZOOM = 0x0F,
        SET_CAMERA_IMAGE_TYPE = 0x11,
        GET_TEMP_FULL_IMAGE = 0x14,
        READ_RANGEFINDER = 0x15,
        EXTERNAL_ATTITUDE = 0x22,
        SET_TIME = 0x30,
        POSITION_DATA = 0x3e,
    };

    class PACKED BaseMessage {
    public:
        BaseMessage(CommandID _cmdid) :
            cmdid{uint8_t(_cmdid)}
        { }
        uint8_t cmdid;
    };

    class PACKED FirmwareVersionRequest : public BaseMessage { };
    virtual void handle_message(const PackedMessage<FirmwareVersionRequest> &request) = 0;

    class PACKED HardwareIDRequest : public BaseMessage { };
    virtual void handle_message(const PackedMessage<HardwareIDRequest> &request) = 0;

    class PACKED GimbalAttitudeRequest : public BaseMessage { };
    virtual void handle_message(const PackedMessage<GimbalAttitudeRequest> &request) = 0;

    class PACKED ExternalAttitude : public BaseMessage {
    public:
        uint32_t time_boot_ms;
        float roll, pitch, yaw;
        float rollspeed, pitchspeed, yawspeed;
    };
    virtual void handle_message(const PackedMessage<ExternalAttitude> &request) {
        AP_HAL::panic("Unhandled ExternalAttitude message");
    }

    class PACKED SetCameraImageType : public BaseMessage { };
    virtual void handle_message(const PackedMessage<SetCameraImageType> &request) {
        AP_HAL::panic("Unhandled SetCameraImageType message");
    }

    class PACKED Photo : public BaseMessage {
    public:
        uint8_t command;
    };
    virtual void handle_message(const PackedMessage<Photo> &request) {
        AP_HAL::panic("Unhandled Photo message");
    }

    class PACKED AcquireGimbalConfigInfo : public BaseMessage {
    public:
    };
    virtual void handle_message(const PackedMessage<AcquireGimbalConfigInfo> &request) {
        AP_HAL::panic("Unhandled AcquireGimbalConfigInfo message");
    }

    class PACKED GetTempFullImageRequest : public BaseMessage {
    public:
        uint8_t mode;
    };
    virtual void handle_message(const PackedMessage<GetTempFullImageRequest> &request) { }

    class PACKED ReadRangefinderRequest : public BaseMessage { };
    virtual void handle_message(const PackedMessage<ReadRangefinderRequest> &request) { }

    class PACKED GimbalRotation : public BaseMessage {
    public:
        uint8_t yaw_rate_pct;
        uint8_t pitch_rate_pct;
    };
    virtual void handle_message(const PackedMessage<GimbalRotation> &request) {
        AP_HAL::panic("Unhandled GimbalRotation message");
    }

    class PACKED PositionData : public BaseMessage {
    public:
        uint32_t time_boot_ms;
        int32_t lat, lon;
        int32_t alt_msl, alt_ellipsoid;
        int32_t velocity_ned_int32[3];
    };
    virtual void handle_message(const PackedMessage<PositionData> &request) {
        AP_HAL::panic("Unhandled PositionData message");
    }

    class PACKED SetTime : public BaseMessage {
    public:
        uint64_t epoch_us;
    };
    virtual void handle_message(const PackedMessage<SetTime> &request) {
        AP_HAL::panic("Unhandled SetTime message");
    }

    /*
     *  Input Handling
     */
    void update_input();
    void handle_received_message();
    uint32_t expected_message_length(CommandID id) const;

    union u {
        u() {}
        uint8_t buffer[256]; // from-autopilot
        PackedMessage<BaseMessage> packed_empty;
        PackedMessage<FirmwareVersionRequest> packed_firmwareversionrequest;
        PackedMessage<HardwareIDRequest> packed_hardwareidrequest;
        PackedMessage<GimbalAttitudeRequest> packed_gimbalattituderequest;
        PackedMessage<ExternalAttitude> packed_externalattitude;
        PackedMessage<SetCameraImageType> packed_setcameraimgetype;
        PackedMessage<Photo> packed_photo;
        PackedMessage<AcquireGimbalConfigInfo> packed_acquiregimbalconfiginfo;
        PackedMessage<GetTempFullImageRequest> packed_gettempfullimage;
        PackedMessage<ReadRangefinderRequest> packed_readrangefinder;
        PackedMessage<GimbalRotation> packed_gimbalrotation;
        PackedMessage<PositionData> packed_positiondata;
        PackedMessage<SetTime> packed_settime;
    } msg;
    uint8_t buflen;

    void move_preamble_in_buffer(uint8_t search_start_pos=0);

    /*
     *  Output Handling
     */
    void update_output(const Location &location);

    uint16_t expected_seq;

    static constexpr bool strict_parsing = true;

};  // end class Siyi

};  // end namespace SITL

#endif  // AP_SIM_SIYI_ENABLED
