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
  Simulator for the FETtecOneWire ESCs

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --uartF=sim:fetteconewireesc --speedup=1 --console

param set SERIAL5_PROTOCOL 38
param set SERIAL5_BAUD 500000
param set SERVO_FTW_MASK 15
param set SIM_FTOWESC_ENA 1
reboot

param fetch

#param set SIM_FTOWESC_FM 1  # fail mask

./Tools/autotest/autotest.py --gdb --debug build.ArduCopter fly.ArduCopter.FETtecESC

*/

#pragma once

#include <AP_Param/AP_Param.h>

#include "SITL_Input.h"

#include "SIM_SerialDevice.h"

#include <stdio.h>

#define SIM_FTW_DEBUGGING 0
#if SIM_FTW_DEBUGGING
#define simfet_debug(fmt, args ...)  do {::fprintf(stderr,"SIMFET: %s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define simfet_debug(fmt, args ...)
#endif


namespace SITL {

class FETtecOneWireESC : public SerialDevice {
public:

    FETtecOneWireESC();

    // update state
    void update(const class Aircraft &aircraft);

    static const AP_Param::GroupInfo var_info[];

    bool enabled() const { return _enabled.get(); }

    void update_sitl_input_pwm(struct sitl_input &input);

private:

    AP_Int8  _enabled;  // enable FETtec ESC sim
    AP_Int32  _powered_mask;  // mask of ESCs with power

    struct PACKED ConfigMessageHeader {
        uint8_t header;  // master is always 0x01
        uint8_t target_id;  // 5 bits only
        uint16_t frame_type;
        uint8_t frame_len;
        uint8_t request_type;
    };

    enum class TLMType : uint8_t {
        NORMAL = 0,
        ALTERNATIVE =1,
    };

    template <typename T>
    class PACKED PackedMessage {
    public:
        PackedMessage(uint8_t _target_id, T _msg) :
            target_id{_target_id},
            msg{_msg}
        {
            frame_len = 6 + sizeof(T);
            update_checksum();
        }
        uint8_t header = (uint8_t)ResponseFrameHeaderID::ESC;  // master is always 0x01
        uint8_t target_id;  // 5 bits only
        uint16_t frame_type = 0;
        uint8_t frame_len;
        T msg;
        uint8_t checksum;

        uint8_t calculate_checksum() const WARN_IF_UNUSED {
            return crc8_dvb_update(0, (const uint8_t*)this, frame_len-1);
        }

        void update_checksum() {
            checksum = calculate_checksum();
        }
    };

    class PACKED SetTLMType {
    public:
        SetTLMType(uint8_t _type) :
            type(_type)
            { }
        uint8_t request_type { (uint8_t)ConfigMessageType::SET_TLM_TYPE };
        uint8_t type;
    };

    class PACKED OK {
    public:
        OK() { }
        uint8_t request_type { (uint8_t)ConfigMessageType::OK };
    };

    class PACKED ESC_TYPE {
    public:
        ESC_TYPE(uint8_t _type) :
            type{_type} { }
        uint8_t type;
    };

    class PACKED SW_VER {
    public:
        SW_VER(uint8_t _version, uint8_t _subversion) :
            version{_version},
            subversion{_subversion}
            { }
        uint8_t version;
        uint8_t subversion;
    };

    class PACKED SN {
    public:
        SN(uint8_t *_sn, uint8_t snlen) {
            memset(sn, 0, ARRAY_SIZE(sn));
            memcpy(sn, _sn, MIN(ARRAY_SIZE(sn), snlen));
        }
        uint8_t sn[12];
    };

    class PACKED SetFastComLength {
    public:
        SetFastComLength() {
        }
        uint8_t length;
        uint8_t byte_count;
        uint8_t min_esc_id;
        uint8_t id_count;
    };

            // tlm_from_id = (uint8_t)telem[1];

            // t.temperature_cdeg = int16_t(telem[5+0] * 100);
            // t.voltage = float((telem[5+1]<<8)|telem[5+2]) * 0.01f;
            // t.current = float((telem[5+3]<<8)|telem[5+4]) * 0.01f;
            // centi_erpm = (telem[5+5]<<8)|telem[5+6];
            // t.consumption_mah = float((telem[5+7]<<8)|telem[5+8]);
            // tx_err_count = (telem[5+9]<<8)|telem[5+10];
    class PACKED ESCTelem {
    public:
        ESCTelem(int8_t _temp, uint16_t _voltage, uint16_t _current, int16_t _rpm, uint16_t _consumption_mah, uint16_t _tx_err_count) :
            temp{_temp},
            voltage{_voltage},
            current{_current},
            rpm{_rpm},
            consumption_mah{_consumption_mah},
            tx_err_count{_tx_err_count}
        { }
        int8_t temp;  // centidegrees
        uint16_t voltage;  // centivolts
        uint16_t current;  // centiamps  (signed?)
        int16_t rpm;  // centi-rpm
        uint16_t consumption_mah;  // ???
        uint16_t tx_err_count;
    };


    union MessageUnion {
        MessageUnion() { }
        uint8_t buffer[255];
//        uint16_t checksum_buffer[35];
        struct ConfigMessageHeader config_message_header;

        PackedMessage<OK> packed_ok;
        PackedMessage<SetTLMType> packed_set_tlm_type;
        PackedMessage<SetFastComLength> packed_set_fast_com_length;

        // void update_checksum();
    };
    MessageUnion u;
    uint8_t buflen;

    // remove count bytes from the start of u.buffer
    void consume_bytes(uint8_t count);

    enum class ConfigMessageType {
        OK = 0,
        BL_PAGE_CORRECT = 1,   // BL only
        NOT_OK = 2,
        BL_START_FW = 3,       // BL only
        BL_PAGES_TO_FLASH = 4, // BL only
        REQ_TYPE = 5,
        REQ_SN = 6,
        REQ_SW_VER = 7,
        BEEP = 13,
        SET_FAST_COM_LENGTH = 26,
        SET_TLM_TYPE = 27, //1 for alternative telemetry. ESC sends full telem per ESC: Temp, Volt, Current, ERPM, Consumption, CrcErrCount
        SET_LED_TMP_COLOR = 51,
    };

    class ESC {
    public:

        enum class State {
            POWERED_OFF = 17,
            IN_BOOTLOADER,
            RUNNING_START,
            RUNNING,
            // UNRESPONSIVE,
        };

        void set_state(State _state) {
            simfet_debug("Moving ESC.id=%u from state=%u to state=%u", (unsigned)id, (unsigned)state, (unsigned)_state);
            state = _state;
        }

        State state = State::POWERED_OFF;

        uint8_t sn[12];
        TLMType telem_type;
        uint8_t id;
        uint8_t type = 34;
        static const uint8_t sw_version = 3;
        static const uint8_t sw_subversion = 4;

        // make sure to zero any state when powering the virtual ESC on
        struct {
            uint8_t length;
            uint8_t byte_count;
            uint8_t min_esc_id;
            uint8_t id_count = 255;
        }  fast_com;

        uint16_t pwm;
        bool telem_request;  // true if we've been asked to send telem

        uint8_t ofs;

        float temperature;
    };

    // canonical structure used for fast com, copied from an ESC
    struct {
        uint8_t min_esc_id;
        uint8_t id_count = 255;
    }  fast_com;

    void bootloader_handle_config_message(ESC &esc);
    void running_handle_config_message(ESC &esc);

    void handle_config_message();
    void handle_config_message_set_tlm_type(ESC &esc);

    void handle_fast_esc_data();

    void update_escs();
    void update_send(const class Aircraft &aircraft);
    void update_input();

    void send_esc_telemetry(const class Aircraft &aircraft);

    template <typename T>
    void send_response(const T& r);

    enum class ResponseFrameHeaderID {
        MASTER = 0x01,
        BOOTLOADER = 0x02,
        ESC = 0x03,
    };

    ESC escs[16];
};

}
