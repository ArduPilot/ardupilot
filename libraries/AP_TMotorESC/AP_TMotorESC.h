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

/* Initial protocol implementation by Amilcar Lucas, IAV GmbH */

#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef AP_TMOTOR_ESC_ENABLED
#define AP_TMOTOR_ESC_ENABLED !HAL_MINIMIZE_FEATURES && BOARD_FLASH_SIZE > 1024
#endif


#if AP_TMOTOR_ESC_ENABLED

#define TME_DEBUGGING 0
#if TME_DEBUGGING
#include <stdio.h>
#define tme_debug(fmt, args ...)  do {::fprintf(stderr,"TMotor: %s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define tme_debug(fmt, args ...)
#endif

#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_Param/AP_Param.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>

class AP_TMotorESC : public AP_ESC_Telem_Backend
{

public:
    AP_TMotorESC();

    /// Do not allow copies
    AP_TMotorESC(const AP_TMotorESC &other) = delete;
    AP_TMotorESC &operator=(const AP_TMotorESC&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    static AP_TMotorESC *get_singleton() {
        return _singleton;
    }

    /// periodically called from SRV_Channels::push()
    void update();

    /// called from AP_Arming; should return false if arming should be
    /// disallowed
    bool pre_arm_check(char *failure_msg, const uint8_t failure_msg_len) const;

private:
    static AP_TMotorESC *_singleton;
    AP_HAL::UARTDriver *_uart;

#if HAL_WITH_ESC_TELEM
    AP_Int8 _pole_count_parameter;
#endif

    static constexpr uint8_t FRAME_OVERHEAD = 6;          ///< message frame overhead (header+tail bytes)
    static constexpr uint8_t MAX_RECEIVE_LENGTH = 12;     ///< max receive message payload length in bytes

    /**
        initialize the device driver: configure serial port, wake-up and configure ESCs
    */
    void init();

    /**
        initialize the serial port
    */
    void init_uart();

    // states configured ESCs can be in:
    enum class ESCState : uint8_t {
        UNINITIALISED = 5,  // when we haven't tried to send anything to the ESC

        WANT_SEND_OK_TO_GET_RUNNING_SW_TYPE = 10,
        WAITING_OK_FOR_RUNNING_SW_TYPE = 11,

        WANT_SEND_START_FW = 20,
        WAITING_OK_FOR_START_FW = 21,

#if HAL_WITH_ESC_TELEM
        WANT_SEND_SET_TLM_TYPE = 60,
        WAITING_SET_TLM_TYPE_OK = 61,
#endif

        RUNNING = 100,
    };

    class ESC {
    public:

#if HAL_WITH_ESC_TELEM
        uint32_t last_telem_us;              ///< last time we got telemetry from this ESC
        uint16_t unexpected_telem;
        uint16_t error_count_at_throttle_count_overflow;            ///< overflow counter for error counter from the ESCs.
        bool telem_expected;                 ///< this ESC is fully configured and is now expected to send us telemetry
        bool telem_requested;                ///< this ESC is fully configured and at some point was requested to send us telemetry
#endif

        uint8_t id;         ///< ESC ID
        uint8_t servo_ofs;  ///< offset into ArduPilot servo array
        bool is_awake;
        void set_state(ESCState _state) {
            tme_debug("Moving ESC.id=%u from state=%u to state=%u", (unsigned)id, (unsigned)state, (unsigned)_state);
            state = _state;
        };
        ESCState state = ESCState::UNINITIALISED;
    };

    uint32_t _last_not_running_warning_ms;  ///< last time we warned the user their ESCs are stuffed
    int32_t _running_mask;                  ///< a bitmask of the actively running ESCs
    uint32_t _last_transmit_us;             ///< last time the transmit() function sent data
    ESC *_escs;
    uint8_t _esc_count;                ///< number of allocated ESCs

    bool _init_done;     ///< device driver is initialized; ESCs may still need to be configured

    enum class FrameSource : uint8_t {
        MASTER     = 0x01,  ///< master is always 0x01
        BOOTLOADER = 0x02,
        ESC        = 0x03,
    };

    enum class MsgType : uint8_t
    {
        OK                  = 0,
        BL_PAGE_CORRECT     = 1,  ///< Bootloader only
        NOT_OK              = 2,
        BL_START_FW         = 3,  ///< Bootloader only - exit the boot loader and start the standard firmware
        BL_PAGES_TO_FLASH   = 4,  ///< Bootloader only
        SET_TLM_TYPE        = 27, ///< telemetry operation mode
    };

    /**
    a frame looks like:
    byte 1 = frame header (master is always 0x01)
    byte 2 = target ID (5bit)
    byte 3 & 4 = frame type (always 0x00, 0x00 used for bootloader. here just for compatibility)
    byte 5 = frame length over all bytes
    byte 6 - X = request type, followed by the payload
    byte X+1 = 8bit CRC
    */
    template <typename T>
    class PACKED PackedMessage {
    public:
        PackedMessage(uint8_t _esc_id, T _msg) :
            esc_id(_esc_id),
            msg(_msg)
        {
            update_checksum();
        }
        uint8_t frame_source { (uint8_t)FrameSource::MASTER };
        uint8_t esc_id;
        uint16_t frame_type { 0 };  // bootloader only, always zero
        uint8_t frame_length {sizeof(T) + FRAME_OVERHEAD};  // all bytes including frame_source and checksum
        T msg;
        uint8_t checksum;

        void update_checksum() {
            checksum = crc8_dvb_update(0, (const uint8_t*)this, frame_length-1);
        }
    };

    class PACKED OK {
    public:
        uint8_t msgid { (uint8_t)MsgType::OK };
    };

    class PACKED START_FW {
    public:
        uint8_t msgid { (uint8_t)MsgType::BL_START_FW };
    };

/*
 * Messages, methods and states for dealing with ESC telemetry
 */
#if HAL_WITH_ESC_TELEM
    void handle_message_telem(ESC &esc);

    /// the ESC at this offset into _escs should be the next to send a
    /// telemetry request for:
    uint8_t _esc_ofs_to_request_telem_from;

    class PACKED SET_TLM_TYPE {
    public:
        SET_TLM_TYPE(uint8_t _tlm_type) :
            tlm_type{_tlm_type}
        { }
        uint8_t msgid { (uint8_t)MsgType::SET_TLM_TYPE };
        uint8_t tlm_type;
    };

    class PACKED TLM {
    public:
        TLM(int8_t _temp, uint16_t _voltage, uint16_t _current, int16_t _rpm, uint16_t _consumption_mah, uint16_t _tx_err_count) :
            temp{_temp},
            voltage{_voltage},
            current{_current},
            rpm{_rpm},
            consumption_mah{_consumption_mah},
            tx_err_count{_tx_err_count}
        { }
        int8_t temp;              // centi-degrees
        uint16_t voltage;         // centi-Volt
        uint16_t current;         // centi-Ampere  (signed?)
        int16_t rpm;              // centi-rpm
        uint16_t consumption_mah; // mili-Ampere.hour
        uint16_t tx_err_count;    // CRC error count, as perceived from the ESC receiving side
    };

#endif  // HAL_WITH_ESC_TELEM

    /*
     * Methods and data for transmitting data to the ESCSs:
     */

    /**
        transmits data to ESCs
        @param bytes  bytes to transmit
        @param length number of bytes to transmit
        @return false there's no space in the UART for this message
    */
    bool transmit(const uint8_t* bytes, const uint8_t length);

    template <typename T>
    bool transmit(const PackedMessage<T> &msg) {
        return transmit((const uint8_t*)&msg, sizeof(msg));
    }

    /**
        transmits configuration request data to ESCs
        @param bytes  bytes to transmit
        @param length number of bytes to transmit
        @return false if vehicle armed or there's no space in the UART for this message
    */
    bool transmit_config_request(const uint8_t* bytes, const uint8_t length);

    template <typename T>
    bool transmit_config_request(const PackedMessage<T> &msg) {
        return transmit_config_request((const uint8_t*)&msg, sizeof(msg));
    }

    /*
     * Methods and data for receiving data from the ESCs:
     */

    // FIXME: this should be tighter - and probably calculated.  Note
    // that we can't request telemetry faster than the loop interval,
    // which is 20ms on Plane, so that puts a constraint here.  When
    // using fast-throttle with 12 ESCs on Plane you could expect
    // 240ms between telem updates.  Why you have a Plane with 12 ESCs
    // is a bit of a puzzle.
    static const uint32_t max_telem_interval_us = 100000;

    void handle_message(ESC &esc, const uint8_t length);

    /**
        reads data from the UART, calling handle_message on any message found
    */
    void read_data_from_uart();
    union MessageUnion {
        MessageUnion() { }
        PackedMessage<OK> packed_ok;
#if HAL_WITH_ESC_TELEM
        PackedMessage<TLM> packed_tlm;
#endif
        uint8_t receive_buf[FRAME_OVERHEAD + MAX_RECEIVE_LENGTH];
    } u;

    static_assert(sizeof(u.packed_ok) <= sizeof(u.receive_buf),"packed_ok does not fit in receive_buf. MAX_RECEIVE_LENGTH too small?");
#if HAL_WITH_ESC_TELEM
    static_assert(sizeof(u.packed_tlm) <= sizeof(u.receive_buf),"packed_tlm does not fit in receive_buf. MAX_RECEIVE_LENGTH too small?");
#endif

    uint16_t _unknown_esc_message;
    uint16_t _message_invalid_in_state_count;
    uint16_t _period_too_short;
    uint16_t crc_rec_err_cnt;
    uint8_t _receive_buf_used;

    /// shifts data to start of buffer based on magic header bytes
    void move_frame_source_in_receive_buffer(const uint8_t search_start_pos = 0);

    /// cut n bytes from start of buffer
    void consume_bytes(const uint8_t n);

    /// returns true if the first message in the buffer is OK
    bool buffer_contains_ok(const uint8_t length);
};
#endif // AP_TMOTOR_ESC_ENABLED
