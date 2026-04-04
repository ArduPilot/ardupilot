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
  Simulator for the LightWare S45B proximity sensor

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:sf45b --speedup=1 -l 51.8752066,14.6487840,54.15,0

param set SERIAL5_PROTOCOL 11  # proximity
param set PRX1_TYPE 8  # s45b
reboot

arm throttle
rc 3 1600

# for avoidance:
param set DISARM_DELAY 0
param set AVOID_ENABLE 2 # use proximity sensor
param set AVOID_MARGIN 2.00  # 2m
param set AVOID_BEHAVE 0 # slide
reboot
mode loiter
script /tmp/post-locations.scr
arm throttle
rc 3 1600
rc 3 1500
rc 2 1450

*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_PS_LIGHTWARE_SF45B_ENABLED

#include "SIM_PS_LightWare.h"

#include <AP_Math/crc.h>
#include <AP_InternalError/AP_InternalError.h>

namespace SITL {

class PS_LightWare_SF45B : public PS_LightWare {
public:

    using PS_LightWare::PS_LightWare;

    uint32_t packet_for_location(const Location &location,
                                 uint8_t *data,
                                 uint8_t buflen) override;

    void update(const Location &location) override;

private:

    template <typename T>
    class PACKED PackedMessage {
    public:
        PackedMessage(T _msg, uint16_t _flags) :
            flags(_flags),
            msg(_msg)
        {
            flags |= (sizeof(T)) << 6;
        }
        uint8_t preamble { PREAMBLE };
        uint16_t flags;
        T msg;
        uint16_t checksum;

        uint16_t calculate_checksum(uint16_t len) const WARN_IF_UNUSED {
            uint16_t ret = 0;
            for (uint8_t i=0; i<len; i++) {
                ret = crc_xmodem_update(ret, ((const char*)this)[i]);
            }
            return ret;
        }
        uint16_t calculate_checksum() const WARN_IF_UNUSED {
            return calculate_checksum(3+sizeof(T));
        }

        void update_checksum() {
            checksum = calculate_checksum();
        }
    };

    class PACKED MsgStream {
    public:
        MsgStream(uint32_t _stream) :
            stream(_stream)
        { }
        uint8_t msgid { (uint8_t)MessageID::STREAM };
        uint32_t stream;
    };

    class PACKED DistanceOutput {
    public:
        DistanceOutput(uint32_t _desired_fields) :
            desired_fields(_desired_fields)
        { }
        uint8_t msgid { (uint8_t)MessageID::DISTANCE_OUTPUT };
        uint32_t desired_fields;
    };

    class PACKED UpdateRate {
    public:
        UpdateRate(uint8_t _rate) :
            rate(_rate)
        { }
        uint8_t msgid { (uint8_t)MessageID::UPDATE_RATE };
        uint8_t rate;
    };

    class PACKED DistanceDataCM {
    public:
        DistanceDataCM(uint16_t _distance_cm, uint16_t _angle_cd) :
            distance_cm(_distance_cm),
            angle_cd(_angle_cd)
        { }
        uint8_t msgid { (uint8_t)MessageID::DISTANCE_DATA_CM };
        uint16_t distance_cm;
        uint16_t angle_cd;
    };

    // message ids
    enum class MessageID : uint8_t {
        // PRODUCT_NAME = 0,
        // HARDWARE_VERSION = 1,
        // FIRMWARE_VERSION = 2,
        // SERIAL_NUMBER = 3,
        // TEXT_MESSAGE = 7,
        // USER_DATA = 9,
        // TOKEN = 10,
        // SAVE_PARAMETERS = 12,
        // RESET = 14,
        // STAGE_FIRMWARE = 16,
        // COMMIT_FIRMWARE = 17,
        DISTANCE_OUTPUT = 27,
        STREAM = 30,
        DISTANCE_DATA_CM = 44,
        // DISTANCE_DATA_MM = 45,
        // LASER_FIRING = 50,
        // TEMPERATURE = 57,
        UPDATE_RATE = 66,
        // NOISE = 74,
        // ZERO_OFFSET = 75,
        // LOST_SIGNAL_COUNTER = 76,
        // BAUD_RATE = 79,
        // I2C_ADDRESS = 80,
        // STEPPER_STATUS = 93,
        // SCAN_ON_STARTUP = 94,
        // SCAN_ENABLE = 96,
        // SCAN_POSITION = 97,
        // SCAN_LOW_ANGLE = 98,
        // SCAN_HIGH_ANGLE = 99
    };


    /*
     *  Input Handling
     */
    void update_input();

    // make the message in the buffer a read message and return to sender
    void boomerang_buffer_message(uint8_t len);

    // handle a complete checksummed message
    void handle_message();

    void send(const char *data, uint32_t len);

    union u {
        u() {}
        char buffer[256]; // from-autopilot
        PackedMessage<MsgStream> packed_msgstream;
        PackedMessage<DistanceOutput> packed_distance_output;
        PackedMessage<UpdateRate> packed_update_rate;
    } _msg;
    uint8_t _buflen;

    static uint16_t checksum_bytes(const char *buffer, uint8_t len) {
        uint16_t crc = 0;
        for (uint8_t i=0; i<len; i++) {
            crc = crc_xmodem_update(crc, buffer[i]);
        }
        return crc;
    }

    uint16_t msg_checksum() const {
        // 4 is 1 preamble, 2 flags, 1 msgid
        return checksum_bytes(_msg.buffer, payload_length() + 4);
    }

    uint8_t payload_length() const {
        if (_buflen < 3) {
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
        return (_msg.packed_msgstream.flags >> 6) - 1;
    }

    static const uint8_t PREAMBLE = 0xAA;

    void move_preamble_in_buffer(uint8_t search_start_pos=0);

    enum class InputState {
        WANT_PREAMBLE = 45,
        WANT_FLAGS = 46,
        WANT_PAYLOAD = 47,
        WANT_CRC = 48,
    };
    InputState _inputstate = InputState::WANT_PREAMBLE;
    void set_inputstate(InputState newstate) {
        // ::fprintf(stderr, "Moving from inputstate (%u) to (%u)\n", (uint8_t)_inputstate, (uint8_t)newstate);
        _inputstate = newstate;
    }

    /*
     * SETTINGS
     */
    uint32_t stream;
    uint32_t desired_fields;
    uint8_t update_rate;

    /*
     * OUTPUT HANDLING
     */

    struct {
        bool stream;
        bool desired_fields;
        bool update_rate;
    } send_response;

    enum class State {
        SCANNING = 21,
    };
    State _state = State::SCANNING;
    void set_state(State newstate) {
        // ::fprintf(stderr, "Moving from state (%u) to (%u)\n", (uint8_t)_state, (uint8_t)newstate);
        _state = newstate;
    }

    void update_output(const Location &location);
    void update_output_responses();
    void update_output_scan(const Location &location);

    uint32_t last_scan_output_time_ms;

    float last_degrees_bf;  // previous iteration's lidar angle
    float last_dir = 1;     // previous iterations movement direction.  +1 CW, -1 for CCW

};

};

#endif  // AP_SIM_PS_LIGHTWARE_SF45B_ENABLED
