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
#define HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO 0
#endif

// provide beep support (optional feature)
#ifndef HAL_AP_FETTEC_ESC_BEEP
#define HAL_AP_FETTEC_ESC_BEEP 0
#endif

// provide light support (optional feature)
#ifndef HAL_AP_FETTEC_ESC_LIGHT
#define HAL_AP_FETTEC_ESC_LIGHT 0
#endif

#if HAL_AP_FETTEC_ONEWIRE_ENABLED

#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_Param/AP_Param.h>



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
    static constexpr uint8_t MAX_TRANSMIT_LENGTH = 4;
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
        transmits a FETtec OneWire frame to an ESC
        @param esc_id id of the ESC
        @param bytes  8 bit array of bytes. Where byte 1 contains the command, and all following bytes can be the payload
        @param length length of the bytes array
        @return false if length is bigger than MAX_TRANSMIT_LENGTH, true on write success
    */
    bool transmit(const uint8_t esc_id, const uint8_t *bytes, uint8_t length);

    enum class return_type : uint8_t
    {
        RESPONSE,
        FULL_FRAME
    };

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
        @param length the expected answer length
        @param return_full_frame can be return_type::RESPONSE or return_type::FULL_FRAME
        @return receive_response enum
    */
    receive_response receive(uint8_t *bytes, uint8_t length, return_type return_full_frame);
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
        @param return_full_frame can be return_type::RESPONSE or return_type::FULL_FRAME
        @param req_len transmit request length
        @return pull_state enum
    */
    pull_state pull_command(const uint8_t esc_id, const uint8_t *command, uint8_t *response, return_type return_full_frame, const uint8_t req_len);

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
        uint8_t command[4];        ///< fast-throttle command frame header bytes
        uint8_t byte_count;        ///< nr bytes in a fast throttle command
        uint8_t min_id;            ///< Zero-indexed ESC ID
        uint8_t max_id;            ///< Zero-indexed ESC ID
    } _fast_throttle;

    /// response length lookup table, saves 104 bytes of flash and speeds up the pull_command() function
    uint8_t _response_length[uint8_t(msg_type::SIZEOF_RESPONSE_LENGTH)];

};
#endif // HAL_AP_FETTEC_ONEWIRE_ENABLED
