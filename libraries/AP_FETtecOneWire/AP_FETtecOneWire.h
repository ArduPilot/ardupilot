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

    /* Protocol implementation was provided by FETtec */

#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef HAL_AP_FETTECONEWIRE_ENABLED
#define HAL_AP_FETTECONEWIRE_ENABLED !HAL_MINIMIZE_FEATURES && !defined(HAL_BUILD_AP_PERIPH) && BOARD_FLASH_SIZE > 1024
#endif

#ifndef HAL_AP_FETTECONEWIRE_GET_STATIC_INFO
#define HAL_AP_FETTECONEWIRE_GET_STATIC_INFO 0
#endif

#if HAL_AP_FETTECONEWIRE_ENABLED

#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_Param/AP_Param.h>



class AP_FETtecOneWire
#if HAL_WITH_ESC_TELEM
 : public AP_ESC_Telem_Backend
#endif
{

public:
    AP_FETtecOneWire();

    /// Do not allow copies
    AP_FETtecOneWire(const AP_FETtecOneWire &other) = delete;
    AP_FETtecOneWire &operator=(const AP_FETtecOneWire&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    void update();
    static AP_FETtecOneWire *get_singleton() {
        return _singleton;
    }
private:
    /// initialize FETtecOneWire protocol
    void init();
    static AP_FETtecOneWire *_singleton;
    bool _initialised;
    AP_HAL::UARTDriver *_uart;
    AP_Int32 motor_mask;
    AP_Int8 pole_count;

    uint32_t _last_send_us;
    static constexpr uint32_t DELAY_TIME_US = 700;
#if HAL_WITH_ESC_TELEM
    static constexpr uint8_t MOTOR_COUNT_MAX = ESC_TELEM_MAX_ESCS; /// OneWire supports up-to 25 ESCs, but Ardupilot only supports 12
#else
    static constexpr uint8_t MOTOR_COUNT_MAX = 12;                 /// OneWire supports up-to 25 ESCs, but Ardupilot only supports 12
#endif
    uint8_t _telem_req_type; /// the requested telemetry type (telem_type::XXXXX)

/**
    calculates crc tx error rate for incoming packages. It converts the CRC error counts into percentage
    @param esc_id id of ESC, that the error is calculated for
    @param esc_error_count the error count given by the esc
    @param increment_only if this is set to 1 it only increases the message count and returns 0. If set to 1 it does not increment but gives back the error count.
    @return the error in percent
*/
float calc_tx_crc_error_perc(uint8_t esc_id, uint16_t esc_error_count, uint8_t increment_only);

/**
    generates used 8 bit CRC for arrays
    @param buf 8 bit byte array
    @param buf_len count of bytes that should be used for CRC calculation
    @return 8 bit CRC
*/
    uint8_t get_crc8(uint8_t *buf, uint16_t buf_len) const;

/**
    transmits a FETtec OneWire frame to an ESC
    @param esc_id id of the ESC
    @param bytes  8 bit array of bytes. Where byte 1 contains the command, and all following bytes can be the payload
    @param length length of the bytes array
*/
    void transmit(uint8_t esc_id, uint8_t *bytes, uint8_t length);

/**
    reads the FETtec OneWire answer frame of an ESC
    @param bytes 8 bit byte array, where the received answer gets stored in
    @param length the expected answer length
    @param return_full_frame can be OW_RETURN_RESPONSE or OW_RETURN_FULL_FRAME
    @return 1 if the expected answer frame was there, 0 if dont
*/
    uint8_t receive(uint8_t *bytes, uint8_t length, uint8_t return_full_frame);

/**
    Resets a pending pull request
*/
    void pull_reset();

/**
    Pulls a complete request between flight controller and ESC
    @param esc_id id of the ESC
    @param command 8bit array containing the command that should be send including the possible payload
    @param response 8bit array where the response will be stored in
    @param return_full_frame can be OW_RETURN_RESPONSE or OW_RETURN_FULL_FRAME
    @return 1 if the request is completed, 0 if dont
*/
    uint8_t pull_command(uint8_t esc_id, uint8_t *command, uint8_t *response, uint8_t return_full_frame);

/**
    scans for ESCs in bus. should be called until _scan_active >= MOTOR_COUNT_MAX
    @return the current scanned ID
*/
    uint8_t scan_escs();

/**
    starts all ESCs in bus and prepares them for receiving teh fast throttle command should be called until _setup_active >= MOTOR_COUNT_MAX
    @return the current used ID
*/
    uint8_t init_escs();

/**
    sets the telemetry mode to full mode, where one ESC answers with all telem values including CRC Error count and a CRC
    @return returns the response code
*/
    uint8_t set_full_telemetry(uint8_t active);

/**
    checks if the requested telemetry is available.
    @param telemetry 16bit array where the read telemetry will be stored in.
    @param return the telemetry request number or -1 if unavailable
*/
    int8_t check_for_tlm(uint16_t *telemetry);

/**
    checks if the requested telemetry is available.
    @param telemetry 16bit array where the read telemetry will be stored in.
    @param esc_id OneWire ID to match incoming telemetry package
    @param return the esc id request number or -1 if unavailable
*/
    
    int8_t check_for_full_telemetry(uint16_t* telemetry, uint8_t esc_id);
/**
    does almost all of the job.
    scans for ESCs if not already done.
    initializes the ESCs if not already done.
    sends fast throttle signals if init is complete.
    @param motor_values a 16bit array containing the throttle signals that should be sent to the motors. 0-2000 where 1001-2000 is positive rotation and 999-0 reversed rotation
    @param telemetry 16bit array where the read telemetry will be stored in.
    @param motor_count the count of motors that should get values send
    @param tlm_request the requested telemetry type (telem_type::XXXXX)
    @return the telemetry request if telemetry was available, -1 if dont
*/
    int8_t escs_set_values(uint16_t *motor_values, uint16_t *telemetry, uint8_t motor_count, uint8_t tlm_request);

    static constexpr uint8_t ALL_ID = 0x1F;
    typedef struct FETtecOneWireESC
    {
      uint8_t in_boot_loader;
#if HAL_AP_FETTECONEWIRE_GET_STATIC_INFO
      uint8_t firmware_version;
      uint8_t firmware_sub_version;
      uint8_t esc_type;
      uint8_t serial_number[12];
#endif
    } FETtecOneWireESC_t;

    uint8_t active_esc_ids[MOTOR_COUNT_MAX] = {0};
    FETtecOneWireESC_t _found_escs[MOTOR_COUNT_MAX];
    uint8_t _found_escs_count;
    uint8_t _scan_active;
    uint8_t _setup_active;

    static constexpr uint8_t use_full_telemetry = 1; //Set to 1 to use alternative, set to 0 to use standard TLM
    uint8_t _set_full_telemetry_active = 1; //Helper to set alternative TLM for every ESC
    uint8_t _set_full_telemetry_retry_count = 0; 
    uint8_t _ignore_own_bytes;
    int8_t _min_id = MOTOR_COUNT_MAX;
    int8_t _max_id;
    uint8_t _id_count;
    uint8_t _fast_throttle_byte_count;
    uint8_t _pull_success;
    uint8_t _pull_busy;
    uint8_t _tlm_request;
    uint8_t _last_crc;

    enum return_type
    {
      OW_RETURN_RESPONSE,
      OW_RETURN_FULL_FRAME
    };

    enum msg_type
    {
      OW_OK,
      OW_BL_PAGE_CORRECT,   // BL only
      OW_NOT_OK,
      OW_BL_START_FW,       // BL only
      OW_BL_PAGES_TO_FLASH, // BL only
      OW_REQ_TYPE,
      OW_REQ_SN,
      OW_REQ_SW_VER,
      OW_BEEP = 13,
      OW_SET_FAST_COM_LENGTH = 26,
      OW_SET_TLM_TYPE = 27, //1 for alternative telemetry. ESC sens full telem per ESC: Temp, Volt, Current, ERPM, Consumption, CrcErrCount
      OW_SET_LED_TMP_COLOR = 51,
    };

    enum telem_type
    {
      TEMP,
      VOLT,
      CURRENT,
      ERPM,
      CONSUMPTION,
      DEBUG1,
      DEBUG2,
      DEBUG3
    };

    /// presistent scan state data (only used inside scan_escs() function)
    struct scan_state
    {
      uint16_t delay_loops;
      uint8_t scan_id;
      uint8_t scan_state;
      uint8_t scan_timeout;
    } _ss;

    /// presistent init state data (only used inside init_escs() function)
    struct init_state
    {
      uint8_t delay_loops;
      uint8_t active_id;
      uint8_t state;
      uint8_t timeout;
      uint8_t wake_from_bl;
      uint8_t set_fast_command[4] = {OW_SET_FAST_COM_LENGTH, 0, 0, 0};
    } _is;

    uint8_t _response_length[OW_SET_TLM_TYPE+1]; // OW_SET_LED_TMP_COLOR is ignored here
    uint8_t _request_length[OW_SET_TLM_TYPE+1];  // OW_SET_LED_TMP_COLOR is ignored here

};
#endif // HAL_AP_FETTECONEWIRE_ENABLED

