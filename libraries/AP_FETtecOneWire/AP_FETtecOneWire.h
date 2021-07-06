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

/* Initial protocol implementation was provided by FETtec */
/* Strongly modified by Amilcar Lucas, IAV GmbH */

#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef HAL_AP_FETTEC_ONEWIRE_ENABLED
#define HAL_AP_FETTEC_ONEWIRE_ENABLED !HAL_MINIMIZE_FEATURES && !defined(HAL_BUILD_AP_PERIPH) && BOARD_FLASH_SIZE > 1024
#endif

// Support both full-duplex at 500Kbit/s as well as half-duplex at 2Mbit/s (optional feature)
#ifndef HAL_AP_FETTEC_HALF_DUPLEX
#define HAL_AP_FETTEC_HALF_DUPLEX 0
#endif

// Get static info from the ESCs (optional feature)
#ifndef HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
#define HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO 1
#endif

// provide beep support (optional feature)
#ifndef HAL_AP_FETTEC_ESC_BEEP
#define HAL_AP_FETTEC_ESC_BEEP 1
#endif

// provide light support (optional feature)
#ifndef HAL_AP_FETTEC_ESC_LIGHT
#define HAL_AP_FETTEC_ESC_LIGHT 1
#endif

#if HAL_AP_FETTEC_ONEWIRE_ENABLED

#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_Param/AP_Param.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>

class AP_FETtecOneWire : public AP_ESC_Telem_Backend
{

public:
    AP_FETtecOneWire();

    /// Do not allow copies
    AP_FETtecOneWire(const AP_FETtecOneWire &other) = delete;
    AP_FETtecOneWire &operator=(const AP_FETtecOneWire&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    bool pre_arm_check(char *failure_msg, const uint8_t failure_msg_len) const;

    /// periodically called from SRV_Channels::push()
    void update();
    static AP_FETtecOneWire *get_singleton() {
        return _singleton;
    }

#if HAL_AP_FETTEC_ESC_BEEP
    /**
    makes all connected ESCs beep
    @param beep_frequency a 8 bit value from 0-255. higher make a higher beep
    */
    void beep(const uint8_t beep_frequency);
#endif

#if HAL_AP_FETTEC_ESC_LIGHT
    /**
    sets the racewire color for all ESCs
    @param r red brightness
    @param g green brightness
    @param b blue brightness
    */
    void led_color(const uint8_t r, const uint8_t g, const uint8_t b);
#endif

private:
    static AP_FETtecOneWire *_singleton;
    AP_HAL::UARTDriver *_uart;

#if HAL_WITH_ESC_TELEM
    static constexpr uint8_t MOTOR_COUNT_MAX = ESC_TELEM_MAX_ESCS; ///< OneWire supports up-to 15 ESCs, but Ardupilot only supports 12
#else
    static constexpr uint8_t MOTOR_COUNT_MAX = 12;                 ///< OneWire supports up-to 15 ESCs, but Ardupilot only supports 12
#endif
    AP_Int32 _motor_mask_parameter;
    AP_Int32 _reverse_mask_parameter;
#if HAL_WITH_ESC_TELEM
    AP_Int8 _pole_count_parameter;
#endif

    static constexpr uint8_t FRAME_OVERHEAD = 6;
    static constexpr uint8_t MAX_RECEIVE_LENGTH = 12;

    /**
        initialize the serial port, scan the OneWire bus, setup the found ESCs
    */
    void init();

    /**
        check if the current configuration is OK
    */
    void configuration_check();

    /**
        transmits data to ESCs
        @param bytes  bytes to transmit
        @param length number of bytes to transmit
        @return false there's no space in the UART for this message
    */
    bool transmit(const uint8_t* bytes, uint8_t length);

    enum class receive_response : uint8_t
    {
        NO_ANSWER_YET,
        ANSWER_VALID,
        CRC_MISSMATCH,
        REQ_OVERLENGTH
    };

    /**
        reads the FETtec OneWire answer frame of an ESC
        @param bytes 8 bit byte array, where the received answer gets stored in
        @param length bytes available in bytes
        @return receive_response enum
    */
    receive_response receive(uint8_t *bytes, uint8_t length);
    uint8_t receive_buf[FRAME_OVERHEAD + MAX_RECEIVE_LENGTH];
    uint8_t receive_buf_used;
    void move_preamble_in_receive_buffer(uint8_t search_start_pos = 0);
    void consume_bytes(uint8_t n);

    enum class pull_state : uint8_t {
        BUSY,
        COMPLETED,
        FAILED
    };

    /**
        Pulls a complete request between flight controller and ESC
        @param esc_id id of the ESC
        @param command 8bit array containing the command that should be send including the possible payload
        @param response 8bit array where the response will be stored in
        @param req_len transmit request length
        @return pull_state enum
    */
    template <typename T, typename R>
    pull_state pull_command(const T &cmd, R &response);

    /**
        Scans for all ESCs in bus. Configures fast-throttle and telemetry for the ones found.
        Should be periodically called until _scan.state == scan_state_t::DONE
    */
    void scan_escs();

    /**
        configure the fast-throttle command.
        Should be called once after scan_escs() is completted and before config_escs()
    */
    void config_fast_throttle();

#if HAL_WITH_ESC_TELEM
    /**
        increment message packet count for every ESC
    */
    void inc_sent_msg_count();

    /**
        calculates tx (outgoing packets) error-rate by converting the CRC error counts reported by the ESCs into percentage
        @param esc_id id of ESC, that the error is calculated for
        @param esc_error_count the error count given by the esc
        @return the error in percent
    */
    float calc_tx_crc_error_perc(const uint8_t esc_id, uint16_t esc_error_count);

    /**
        if init is complete checks if the requested telemetry is available.
        @param t telemetry datastructure where the read telemetry will be stored in.
        @param centi_erpm 16bit centi-eRPM value returned from the ESC
        @param tx_err_count Ardupilot->ESC communication CRC error counter
        @param tlm_from_id receives the ID from the ESC that has respond with its telemetry
        @return receive_response enum
    */
    receive_response decode_single_esc_telemetry(TelemetryData& t, int16_t& centi_erpm, uint16_t& tx_err_count, uint8_t &tlm_from_id);
#endif

    /**
        if init is complete sends a single fast-throttle frame containing the throttle for all found OneWire ESCs.
        @param motor_values a 16bit array containing the throttle values that should be sent to the motors. 0-2000 where 1001-2000 is positive rotation and 0-999 reversed rotation
        @param tlm_request the ESC to request telemetry from (-1 for no telemetry, 0 for ESC1, 1 for ESC2, 2 for ESC3, ...)
    */
    void escs_set_values(const uint16_t *motor_values, const int8_t tlm_request);

    static constexpr uint8_t SERIAL_NR_BITWIDTH = 12;

    class FETtecOneWireESC
    {
        public:
#if HAL_WITH_ESC_TELEM
        uint16_t error_count;                ///< error counter from the ESCs.
        uint16_t error_count_since_overflow; ///< error counter from the ESCs to pass the overflow.
#endif
        bool active;
#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
        uint8_t firmware_version;
        uint8_t firmware_sub_version;
        uint8_t esc_type;
        uint8_t serial_number[SERIAL_NR_BITWIDTH];
#endif
    } _found_escs[MOTOR_COUNT_MAX]; ///< Zero-indexed array

    uint32_t _last_config_check_ms;
#if HAL_WITH_ESC_TELEM
    float _crc_error_rate_factor; ///< multiply factor. Used to avoid division operations
    uint16_t _sent_msg_count;     ///< number of fast-throttle commands sent by the flight controller
    uint16_t _update_rate_hz;
#endif
    uint16_t _motor_mask;
    uint16_t _reverse_mask;
    uint8_t _nr_escs_in_bitmask; ///< number of ESCs set on the FTW_MASK parameter
    uint8_t _found_escs_count;   ///< number of ESCs auto-scanned in the bus by the scan_escs() function
    uint8_t _configured_escs;    ///< number of ESCs fully configured by the scan_escs() function, might be smaller than _found_escs_count

    int8_t _requested_telemetry_from_esc; ///< the ESC to request telemetry from (-1 for no telemetry, 0 for ESC1, 1 for ESC2, 2 for ESC3, ...)
#if HAL_AP_FETTEC_HALF_DUPLEX
    uint8_t _ignore_own_bytes; ///< bytes to ignore while receiving, because we have transmitted them ourselves
    uint8_t _last_crc;       ///< the CRC from the last sent fast-throttle command
    bool _use_hdplex;        ///< use asynchronous half-duplex serial communication
#endif
    bool _initialised;       ///< device driver and ESCs are fully initialized
    bool _pull_busy;         ///< request-reply transaction is busy

    enum class msg_type : uint8_t
    {
        OK                  = 0,
        BL_PAGE_CORRECT     = 1,  ///< Bootloader only
        NOT_OK              = 2,
        BL_START_FW         = 3,  ///< Bootloader only - exit the boot loader and start the standard firmware
        BL_PAGES_TO_FLASH   = 4,  ///< Bootloader only
#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
        REQ_TYPE            = 5,  ///< ESC type
        REQ_SN              = 6,  ///< serial number
        REQ_SW_VER          = 7,  ///< software version
#endif
#if HAL_AP_FETTEC_ESC_BEEP
        BEEP                = 13, ///< make noise
#endif
        SET_FAST_COM_LENGTH = 26, ///< configure fast-throttle command
        SET_TLM_TYPE        = 27, ///< telemetry operation mode
        SIZEOF_RESPONSE_LENGTH,   ///< size of the _response_length array used in the pull_command() function, you can move this one around
#if HAL_AP_FETTEC_ESC_LIGHT
        SET_LED_TMP_COLOR   = 51, ///< msg_type::SET_LED_TMP_COLOR is ignored here. You must update this if you add new msg_type cases
#endif
    };

    /*
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
        uint8_t preamble { 0x01 };  // (master is always 0x01
        uint8_t esc_id;
        uint16_t frame_type = 0;  // bootloader only, always zero
        uint8_t frame_length = sizeof(T) + 6;  // all bytes inc preamble and checksum
        T msg;
        uint8_t checksum;

        void update_checksum() {
            checksum = crc8_dvb_update(0, (const uint8_t*)this, frame_length-1);
        }
    };

    class PACKED OK {
    public:
        uint8_t msgid { (uint8_t)msg_type::OK };
    };

#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
    class PACKED REQ_TYPE {
    public:
        uint8_t msgid { (uint8_t)msg_type::REQ_TYPE };
    };

    class PACKED REQ_SW_VER {
    public:
        uint8_t msgid { (uint8_t)msg_type::REQ_SW_VER };
    };

    class PACKED REQ_SN {
    public:
        uint8_t msgid { (uint8_t)msg_type::REQ_SN };
    };
#endif

    class PACKED SET_TLM_TYPE {
    public:
        SET_TLM_TYPE(uint8_t _tlm_type) :
            tlm_type{_tlm_type}
        { }
        uint8_t msgid { (uint8_t)msg_type::SET_TLM_TYPE };
        uint8_t tlm_type;
    };

    class PACKED SET_FAST_COM_LENGTH {
    public:
        SET_FAST_COM_LENGTH(uint8_t _byte_count, uint8_t _min_esc_id, uint8_t _esc_count) :
            byte_count{_byte_count},
            min_esc_id{_min_esc_id},
            esc_count{_esc_count}
        { }
        uint8_t msgid { (uint8_t)msg_type::SET_FAST_COM_LENGTH };
        uint8_t byte_count;
        uint8_t min_esc_id;
        uint8_t esc_count;
    };


#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
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

#endif

    class PACKED START_FW {
    public:
        uint8_t msgid { (uint8_t)msg_type::BL_START_FW };
    };

#if HAL_AP_FETTEC_ESC_BEEP
    class PACKED Beep {
    public:
        Beep(uint8_t _beep_frequency) :
            beep_frequency{_beep_frequency}
        { }
        uint8_t msgid { (uint8_t)msg_type::BEEP };
        uint8_t beep_frequency;
        // add two zeros to make sure all ESCs can catch their command as we don't wait for a response here  (don't blame me --pb)
        uint16_t spacer = 0;
    };
#endif

#if HAL_AP_FETTEC_ESC_LIGHT
    class PACKED LEDColour {
    public:
        LEDColour(uint8_t _r, uint8_t _g, uint8_t _b) :
            r{_r},
            g{_g},
            b{_b}
          { }
        uint8_t msgid { (uint8_t)msg_type::SET_LED_TMP_COLOR };
        uint8_t r;
        uint8_t g;
        uint8_t b;
        // add two zeros to make sure all ESCs can catch their command as we don't wait for a response here  (don't blame me --pb)
        uint16_t spacer = 0;
    };
#endif

    /**
        transmits a FETtec OneWire frame to an ESC
        @param msg message to transmit
        @return false if message won't fit in transmit buffer
    */
    template <typename T>
    bool transmit(const PackedMessage<T> &msg) {
        return transmit((const uint8_t*)&msg, sizeof(msg));
    }

    enum class scan_state_t : uint8_t {
        WAIT_FOR_BOOT,            ///< initial state, wait for a ESC(s) cold-start
        IN_BOOTLOADER,            ///< in bootloader?
        START_FW,                 ///< start the firmware
        WAIT_START_FW,            ///< wait for the firmware to start
#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
        ESC_TYPE,                 ///< ask the ESC type
        SW_VER,                   ///< ask the software version
        SN,                       ///< ask the serial number
#endif
        NEXT_ID,                  ///< increment ESC ID and jump to IN_BOOTLOADER
        CONFIG_FAST_THROTTLE,     ///< configure fast-throttle command header
#if HAL_WITH_ESC_TELEM
        CONFIG_TLM,               ///< configure telemetry mode
#endif
        CONFIG_NEXT_ACTIVE_ESC,   ///< increment ESC ID and jump to CONFIG_FAST_THROTTLE
        DONE                      ///< configuration done
    };

    /// presistent scan state data (only used inside scan_escs() function)
    struct scan_state
    {
        uint32_t last_us;          ///< last transaction time in microseconds
        uint8_t id;                ///< Zero-indexed ID of the used ESC
        scan_state_t state;        ///< scan state-machine state
        uint8_t rx_try_cnt;        ///< receive try counter
        uint8_t trans_try_cnt;     ///< transaction (transmit and response) try counter
    } _scan;

    /// fast-throttle command configuration
    struct fast_throttle_config
    {
        uint16_t bits_to_add_left; ///< bits to add after the header
        uint8_t byte_count;        ///< nr bytes in a fast throttle command
        uint8_t min_id;            ///< Zero-indexed ESC ID
        uint8_t max_id;            ///< Zero-indexed ESC ID
    } _fast_throttle;

    struct {
        uint8_t byte_count;
        uint8_t min_esc_id;
        uint8_t esc_count;
    } _fast_throttle_command;

};
#endif // HAL_AP_FETTEC_ONEWIRE_ENABLED
