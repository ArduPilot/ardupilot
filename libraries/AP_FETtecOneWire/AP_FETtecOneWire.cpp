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

#include <AP_Math/AP_Math.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>

#include "AP_FETtecOneWire.h"
#if HAL_AP_FETTEC_ONEWIRE_ENABLED

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_FETtecOneWire::var_info[] {

    // @Param: MASK
    // @DisplayName: Servo channel output bitmask
    // @Description: Servo channel mask specifying FETtec ESC output.  Set bits must be contiguous.
    // @Bitmask: 0:SERVO1,1:SERVO2,2:SERVO3,3:SERVO4,4:SERVO5,5:SERVO6,6:SERVO7,7:SERVO8,8:SERVO9,9:SERVO10,10:SERVO11,11:SERVO12
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO_FLAGS("MASK",  1, AP_FETtecOneWire, _motor_mask_parameter, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: RVMASK
    // @DisplayName: Servo channel reverse rotation bitmask
    // @Description: Servo channel mask to reverse rotation of FETtec ESC outputs.
    // @Bitmask: 0:SERVO1,1:SERVO2,2:SERVO3,3:SERVO4,4:SERVO5,5:SERVO6,6:SERVO7,7:SERVO8,8:SERVO9,9:SERVO10,10:SERVO11,11:SERVO12
    // @User: Standard
    AP_GROUPINFO("RVMASK",  2, AP_FETtecOneWire, _reverse_mask_parameter, 0),

#if HAL_WITH_ESC_TELEM
    // @Param: POLES
    // @DisplayName: Nr. electrical poles
    // @Description: Number of motor electrical poles
    // @Range: 2 50
    // @RebootRequired: False
    // @User: Standard
    AP_GROUPINFO("POLES", 3, AP_FETtecOneWire, _pole_count_parameter, 14),
#endif

    AP_GROUPEND
};

AP_FETtecOneWire *AP_FETtecOneWire::_singleton;

AP_FETtecOneWire::AP_FETtecOneWire()
{
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_FETtecOneWire must be singleton");
    }
#endif
    _singleton = this;
}

/**
  initialize the serial port, scan the bus, setup the found ESCs

*/
void AP_FETtecOneWire::init()
{
    if (_uart == nullptr) {
        const AP_SerialManager& serial_manager = AP::serialmanager();
        _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FETtecOneWire, 0);
        if (_uart == nullptr) {
            return; // no serial port available, so nothing to do here
        }
        _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _uart->set_unbuffered_writes(true);
        _uart->set_blocking_writes(false);
#if HAL_AP_FETTEC_HALF_DUPLEX
        if (_uart->get_options() & _uart->OPTION_HDPLEX) { //Half-Duplex is enabled
            _use_hdplex = true;
            _uart->begin(2000000U);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FTW using Half-Duplex");
        } else {
            _uart->begin(500000U);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FTW using Full-Duplex");
        }
#else
        _uart->begin(500000U);
#endif
    }

    if (_scan.state != scan_state_t::DONE) {
        scan_escs();
        return;
    }

#if HAL_WITH_ESC_TELEM
    _update_rate_hz = AP::scheduler().get_loop_rate_hz();
    _crc_error_rate_factor = 100.0f/(float)_update_rate_hz; //to save the division in loop, precalculate by the motor loops 100%/400Hz
#endif

    // do not read telemetry information until a fast-throttle command is send
    // and if HAL_WITH_ESC_TELEM is disabled this also ensures correct operation
    _requested_telemetry_from_esc = -1;

    // get the user-configured FETtec ESCs bitmask parameter
    // if the user changes this parameter, he will have to reboot
    _motor_mask = uint16_t(_motor_mask_parameter.get());
    uint16_t smask = _motor_mask; // shifted version of the _motor_mask user parameter
    uint16_t mmask = 0;     // will be a copy of _motor_mask with only the contiguous LSBs set

    static_assert(MOTOR_COUNT_MAX <= sizeof(_motor_mask)*8, "_motor_mask is too narrow for MOTOR_COUNT_MAX ESCs");
    static_assert(MOTOR_COUNT_MAX <= sizeof(_reverse_mask)*8, "_reverse_mask is too narrow for MOTOR_COUNT_MAX ESCs");
    static_assert(MOTOR_COUNT_MAX <= sizeof(smask)*8, "smask is too narrow for MOTOR_COUNT_MAX ESCs");
    static_assert(MOTOR_COUNT_MAX <= sizeof(mmask)*8, "mmask is too narrow for MOTOR_COUNT_MAX ESCs");

    _nr_escs_in_bitmask = 0;
    // count the number of contiguous user-configured FETtec ESCs in the bitmask parameter
    for (uint8_t i = 0; i < MOTOR_COUNT_MAX; i++) {
        if ((smask & 0x01) == 0x00) {
            break;
        }
        smask >>= 1;
        _nr_escs_in_bitmask++;

        // build a copy of _motor_mask_parameter with only the contiguous LSBs set
        mmask |= 0x1;
        mmask <<= 1;
    }

    // tell SRV_Channels about ESC capabilities
    SRV_Channels::set_digital_outputs(mmask, 0);

    _initialised = true;
}

/**
  check if the current configuration is OK
*/
void AP_FETtecOneWire::configuration_check()
{
    if (hal.util->get_soft_armed()) {
        return; // checks are only done when vehicle is disarmed, because the GCS_SEND_TEXT() function calls use lots of resources
    }

    // for safety, only update the reversed motors bitmask when motors are disarmed
    _reverse_mask= _reverse_mask_parameter;

    const uint32_t now = AP_HAL::millis();
    if ((now - _last_config_check_ms < 3000) && _last_config_check_ms != 0) {  // only runs once every 3 seconds
        return;
    }
    _last_config_check_ms = now;

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    if (!_uart->is_dma_enabled()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FTW UART needs DMA");
        return;
    }
#endif

    const bool all_escs_found = _found_escs_count >= _nr_escs_in_bitmask;
    const bool all_escs_configured = _found_escs_count == _configured_escs;
    const bool all_escs_contiguous = _fast_throttle.max_id - _fast_throttle.min_id < _found_escs_count;
    bool telem_rx_missing = false;
#if HAL_WITH_ESC_TELEM
    // TLM recovery, if e.g. a power loss occurred but FC is still powered by USB.
    const uint16_t active_esc_mask = AP::esc_telem().get_active_esc_mask();
    const uint8_t num_active_escs = __builtin_popcount(active_esc_mask & _motor_mask);

    telem_rx_missing = (num_active_escs < _nr_escs_in_bitmask) && (_sent_msg_count > 2 * MOTOR_COUNT_MAX);
#endif

    if (__builtin_popcount(_motor_mask_parameter.get()) != _nr_escs_in_bitmask) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FTW: gap in SERVO_FTW_MASK parameter bits");
    }

    if (!all_escs_contiguous){
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FTW: gap in IDs found");
    }

    if (!all_escs_found) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FTW: found only %i of %i ESCs", _found_escs_count, _nr_escs_in_bitmask);
    }

    if (!all_escs_configured) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FTW: configured only %i of %i ESCs", _configured_escs, _found_escs_count);
    }

#if HAL_WITH_ESC_TELEM
    if (telem_rx_missing) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FTW: got TLM from only %i of %i ESCs", num_active_escs, _nr_escs_in_bitmask);
    }
#endif

    if (!all_escs_contiguous || !all_escs_found || !all_escs_configured || telem_rx_missing) {
        // re-init the entire device driver
#if HAL_WITH_ESC_TELEM
        _sent_msg_count = 0;
#endif
        _scan.state = scan_state_t::WAIT_FOR_BOOT;
        _initialised = false;
    }
}

/**
    transmits data to ESCs
    @param bytes  bytes to transmit
    @param length number of bytes to transmit
    @return false there's no space in the UART for this message
*/
bool AP_FETtecOneWire::transmit(const uint8_t* bytes, uint8_t length)
{
    if (length > _uart->txspace()) {
        return false;
    }
    _uart->write(bytes, length);
#if HAL_AP_FETTEC_HALF_DUPLEX
    if (_use_hdplex) {
        _ignore_own_bytes += length;
    }
#endif
    return true;
}

void AP_FETtecOneWire::move_preamble_in_receive_buffer(uint8_t search_start_pos)
{
    uint8_t i;
    for (i=search_start_pos; i<receive_buf_used; i++) {
        if ((uint8_t)receive_buf[i] == 0x02 ||
            (uint8_t)receive_buf[i] == 0x03) {
            break;
        }
    }
    consume_bytes(i);
}

void AP_FETtecOneWire::consume_bytes(uint8_t n)
{
    if (n == 0) {
        return;
    }
    memmove(receive_buf, &receive_buf[n], receive_buf_used-n);
    receive_buf_used = receive_buf_used - n;
}

/**
    reads the FETtec OneWire answer frame of an ESC
    @param bytes 8 bit byte array, where the received answer gets stored in
    @param length bytes available for storage in *bytes
    @return receive_response enum
*/
AP_FETtecOneWire::receive_response AP_FETtecOneWire::receive(uint8_t* bytes, uint8_t length)
{
    /*
    a frame looks like:
    byte 1 = frame header (0x02 = bootloader, 0x03 = ESC firmware)
    byte 2 = sender ID (5bit)
    byte 3 & 4 = frame type (always 0x00, 0x00 used for bootloader. here just for compatibility)
    byte 5 = frame length over all bytes
    byte 6 - X = answer type, followed by the payload
    byte X+1 = 8bit CRC
    */

    if (length > MAX_RECEIVE_LENGTH) {
        return receive_response::REQ_OVERLENGTH;
    }

#if HAL_AP_FETTEC_HALF_DUPLEX
    //ignore own bytes
    if (_use_hdplex) {
        while (_ignore_own_bytes > 0 && _uart->available()) {
            _ignore_own_bytes--;
            _uart->read();
        }
    }
#endif

    // read as much from the uart as we can:
    const uint8_t bytes_to_read = ARRAY_SIZE(receive_buf) - receive_buf_used;
    uint32_t nbytes = _uart->read(&receive_buf[receive_buf_used], bytes_to_read);
    if (nbytes == 0) {
        return receive_response::NO_ANSWER_YET;
    }
    receive_buf_used += nbytes;

    move_preamble_in_receive_buffer();

    // we know what length of message should be present
    if (receive_buf_used < length) {
        return receive_response::NO_ANSWER_YET;
    }

    if (crc8_dvb_update(0, receive_buf, length-1) != receive_buf[length-1]) {
        // bad message; shift away this preamble byte to try to find
        // another message
        move_preamble_in_receive_buffer(1);
        return receive_response::CRC_MISSMATCH;
    }

    // message is good
    memcpy(bytes, receive_buf, length);

    consume_bytes(length);

    return receive_response::ANSWER_VALID;
}

/**
    Pulls a complete request between flight controller and ESC
    @param esc_id  id of the ESC
    @param command 8bit array containing the command that should be send including the possible payload
    @param response 8bit array where the response will be stored in
    @param req_len transmit request length
    @return pull_state enum
*/
template <typename T, typename R>
AP_FETtecOneWire::pull_state AP_FETtecOneWire::pull_command(const T &cmd, R &response)
{
    R temp_response{response}; // remember esc_id and msgid
    if (!_pull_busy) {
        _pull_busy = transmit(cmd);
    } else if (receive((uint8_t*)&response, sizeof(response)) == receive_response::ANSWER_VALID) {
        if (temp_response.esc_id != response.esc_id) {
            // we got a valid packet back - but it wasn't from the correct ESC!
        } else {
            _scan.rx_try_cnt = 0;
            _scan.trans_try_cnt = 0;
            _pull_busy = false;
            return pull_state::COMPLETED;
        }
    }

    // it will try multiple times to read the response of a request
    if (_scan.rx_try_cnt > 1) {
        _scan.rx_try_cnt = 0;

        _pull_busy = false; // re-transmit the request, in the hope of getting a valid response later

        if (_scan.trans_try_cnt > 4) {
            // the request re-transmit failed multiple times
            _scan.trans_try_cnt = 0;
            return pull_state::FAILED;
        } else {
            _scan.trans_try_cnt++;
        }
    } else {
        _scan.rx_try_cnt++;
    }
    return pull_state::BUSY;
}

/**
    Scans for all ESCs in bus. Configures fast-throttle and telemetry for the ones found.
    Should be periodically called until _scan.state == scan_state_t::DONE
*/
void AP_FETtecOneWire::scan_escs()
{
    const uint32_t now = AP_HAL::micros();
    if (now - _scan.last_us < (_scan.state == scan_state_t::WAIT_START_FW ? 5000U : 2000U)) {
        // the scan_escs() call period must be bigger than 2000 US,
        // as the bootloader has some message timing requirements. And we might be in bootloader
        return;
    }
    _scan.last_us = now;

    PackedMessage<OK> ok_response{uint8_t(_scan.id+1), OK{}};

    switch (_scan.state) {

    // initial state, wait for a ESC(s) cold-start
    case scan_state_t::WAIT_FOR_BOOT:
        _found_escs_count = 0;
        _scan.id = 0;
        for (uint8_t i = 0; i < MOTOR_COUNT_MAX; i++) {
            _found_escs[i].active = false;
        }
        if (now > 500000U) {
            _scan.state = scan_state_t::IN_BOOTLOADER;
        }
        break;

    // is bootloader running?
    case scan_state_t::IN_BOOTLOADER: {
        switch (pull_command(PackedMessage<OK>{uint8_t(_scan.id+1), OK{}}, ok_response)) {
        case pull_state::BUSY:
            break;
        case pull_state::COMPLETED:
            if (ok_response.preamble == 0x02) {
                _scan.state = scan_state_t::START_FW; // is in bootloader, must start firmware
            } else {
                if (!_found_escs[_scan.id].active) {
                    _found_escs_count++; // found a new ESC not in bootloader
                }
                _found_escs[_scan.id].active = true;
#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
                _scan.state = scan_state_t::ESC_TYPE;
#else
                _scan.state = scan_state_t::NEXT_ID;
#endif
            }
            break;
        case pull_state::FAILED:
            _scan.state = scan_state_t::NEXT_ID;
            break;
        }
        break;
    }
    // start the firmware
    case scan_state_t::START_FW:
        if (transmit(PackedMessage<START_FW>{uint8_t(_scan.id+1), START_FW{}})) {
            _scan.state = scan_state_t::WAIT_START_FW;
        }
        break;

    // wait for the firmware to start
    case scan_state_t::WAIT_START_FW:
        _uart->discard_input(); // discard the answer to the previous transmit
        _scan.state = scan_state_t::IN_BOOTLOADER;
        break;

#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
    // ask the ESC type
    case scan_state_t::ESC_TYPE: {
        PackedMessage<ESC_TYPE> response{uint8_t(_scan.id+1), ESC_TYPE{0}};
        switch (pull_command(PackedMessage<REQ_TYPE>{uint8_t(_scan.id+1), REQ_TYPE{}}, response)) {
        case pull_state::BUSY:
            break;
        case pull_state::COMPLETED:
            _found_escs[_scan.id].esc_type = response.msg.type;
            _scan.state = scan_state_t::SW_VER;
            break;
        case pull_state::FAILED:
            _scan.state = scan_state_t::NEXT_ID;
            break;
        }
        break;
    }
    // ask the software version
    case scan_state_t::SW_VER: {
        PackedMessage<SW_VER> response{uint8_t(_scan.id+1), SW_VER{0, 0}};
        switch (pull_command(PackedMessage<REQ_SW_VER>{uint8_t(_scan.id+1), REQ_SW_VER{}}, response)) {
        case pull_state::BUSY:
            break;
        case pull_state::COMPLETED:
            _found_escs[_scan.id].firmware_version = response.msg.version;
            _found_escs[_scan.id].firmware_sub_version = response.msg.subversion;
            _scan.state = scan_state_t::SN;
            break;
        case pull_state::FAILED:
            _scan.state = scan_state_t::NEXT_ID;
            break;
        }
        break;
    }

    // ask the serial number
    case scan_state_t::SN: {
        PackedMessage<SN> response{uint8_t(_scan.id+1), SN{(uint8_t*)"A", 1}};
        switch (pull_command(PackedMessage<REQ_SN>{uint8_t(_scan.id+1), REQ_SN{}}, response)) {
        case pull_state::BUSY:
            break;
        case pull_state::COMPLETED:
            memset(_found_escs[_scan.id].serial_number, '\0', ARRAY_SIZE(_found_escs[_scan.id].serial_number));
            memcpy(_found_escs[_scan.id].serial_number,
                   response.msg.sn,
                   MIN(ARRAY_SIZE(_found_escs[_scan.id].serial_number),
                       ARRAY_SIZE(response.msg.sn)));
            _scan.state = scan_state_t::NEXT_ID;
            break;
        case pull_state::FAILED:
            _scan.state = scan_state_t::NEXT_ID;
            break;
        }
        break;
    }
#endif

    // increment ESC ID and jump to IN_BOOTLOADER
    case scan_state_t::NEXT_ID:
        _scan.state = scan_state_t::IN_BOOTLOADER;
        _scan.id++; // re-run this state machine with the next ESC ID
        if (_scan.id == MOTOR_COUNT_MAX) {
            _scan.id = 0;
            if (_found_escs_count) {
                // one or more ESCs found, scan is completed, now configure the ESCs found
                config_fast_throttle();
                _scan.id = _fast_throttle.min_id;
                _configured_escs = 0;
                _scan.state = scan_state_t::CONFIG_FAST_THROTTLE;
            }
        }
        break;

    // configure fast-throttle command header
    case scan_state_t::CONFIG_FAST_THROTTLE: {
        switch (pull_command(PackedMessage<SET_FAST_COM_LENGTH>{uint8_t(_scan.id+1), SET_FAST_COM_LENGTH{_fast_throttle_command.byte_count, _fast_throttle_command.min_esc_id, _fast_throttle_command.esc_count}}, ok_response)) {
        case pull_state::BUSY:
            break;
        case pull_state::COMPLETED:
#if HAL_WITH_ESC_TELEM
            _scan.state = scan_state_t::CONFIG_TLM;
#else
            _configured_escs++;
            _scan.state = scan_state_t::CONFIG_NEXT_ACTIVE_ESC;
#endif
            break;
        case pull_state::FAILED:
            _scan.state = scan_state_t::CONFIG_NEXT_ACTIVE_ESC;
            break;
        }
        break;
    }

#if HAL_WITH_ESC_TELEM
    // configure telemetry mode
    case scan_state_t::CONFIG_TLM: {
        // 1 here means Alternative telemetry mode -> a single ESC
        // sends it's full telem (Temp, Volt, Current, ERPM,
        // Consumption, CrcErrCount) in a single frame
        switch (pull_command(PackedMessage<SET_TLM_TYPE>{uint8_t(_scan.id+1), SET_TLM_TYPE{1}}, ok_response)) {
        case pull_state::BUSY:
            break;
        case pull_state::COMPLETED:
            _configured_escs++;
            _scan.state = scan_state_t::CONFIG_NEXT_ACTIVE_ESC;
            break;
        case pull_state::FAILED:
            _scan.state = scan_state_t::CONFIG_NEXT_ACTIVE_ESC;
            break;
        }
        break;
    }
#endif

    // increment ESC ID and jump to CONFIG_FAST_THROTTLE
    case scan_state_t::CONFIG_NEXT_ACTIVE_ESC:
        do {
            _scan.id++;
        } while (_scan.id < MOTOR_COUNT_MAX && _found_escs[_scan.id].active == false);
        _scan.state = scan_state_t::CONFIG_FAST_THROTTLE;
        if (_scan.id == MOTOR_COUNT_MAX) {
            _scan.id = 0;
            _scan.state = scan_state_t::DONE;  // one or more ESCs found, scan is completed
        }
        break;

    case scan_state_t::DONE:
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        break;
    }
}

/**
    configure the fast-throttle command.
    Should be called once after scan_escs() is completted and before config_escs()
*/
void AP_FETtecOneWire::config_fast_throttle()
{
    _fast_throttle.min_id = MOTOR_COUNT_MAX;
    _fast_throttle.max_id = 0;
    for (uint8_t i = 0; i < MOTOR_COUNT_MAX; i++) {
        if (_found_escs[i].active) {
            if (i < _fast_throttle.min_id) {
                _fast_throttle.min_id = i;
            }
            if (i > _fast_throttle.max_id) {
                _fast_throttle.max_id = i;
            }
        }
    }

    _fast_throttle.byte_count = 1;
    int16_t bit_count = 12 + (_found_escs_count * 11);
    _fast_throttle.bits_to_add_left = bit_count - 16;
    while (bit_count > 0) {
        _fast_throttle.byte_count++;
        bit_count -= 8;
    }
    _fast_throttle_command.byte_count = _fast_throttle.byte_count; // just for older ESC FW versions since 1.0 001 this byte is ignored as the ESC calculates it itself
    _fast_throttle_command.min_esc_id = _fast_throttle.min_id+1;   // one-indexed min ESC id
    _fast_throttle_command.esc_count  = _found_escs_count;         // count of ESCs that will get signals
}

#if HAL_WITH_ESC_TELEM
/**
    increment message packet count for every ESC
*/
void AP_FETtecOneWire::inc_sent_msg_count()
{
    _sent_msg_count++;
    if (_sent_msg_count > 4 * _update_rate_hz) { // resets every four seconds
        _sent_msg_count = 0;
        for (int i=0; i<_found_escs_count; i++) {
            _found_escs[i].error_count_since_overflow = _found_escs[i].error_count; //save the current ESC error state
        }
    }
}

/**
    calculates tx (outgoing packets) error-rate by converting the CRC error counts reported by the ESCs into percentage
    @param esc_id id of ESC, that the error is calculated for
    @param current_error_count the error count given by the esc
    @return the error in percent
*/
float AP_FETtecOneWire::calc_tx_crc_error_perc(const uint8_t esc_id, uint16_t current_error_count)
{
    _found_escs[esc_id].error_count = current_error_count; //Save the error count to the esc
    uint16_t corrected_error_count = (uint16_t)((uint16_t)_found_escs[esc_id].error_count - (uint16_t)_found_escs[esc_id].error_count_since_overflow); //calculates error difference since last overflow.
    return (float)corrected_error_count*_crc_error_rate_factor; //calculates percentage
}

/**
    if init is complete checks if the requested telemetry is available.
    @param t telemetry datastructure where the read telemetry will be stored in.
    @param centi_erpm 16bit centi-eRPM value returned from the ESC
    @param tx_err_count Ardupilot->ESC communication CRC error counter
    @param tlm_from_id receives the ID from the ESC that has respond with its telemetry
    @return receive_response enum
*/
AP_FETtecOneWire::receive_response AP_FETtecOneWire::decode_single_esc_telemetry(TelemetryData& t, int16_t& centi_erpm, uint16_t& tx_err_count, uint8_t &tlm_from_id)
{
    receive_response ret = receive_response::NO_ANSWER_YET;
    static constexpr uint8_t TELEM_LENGTH = 11;
    static_assert(MAX_RECEIVE_LENGTH >= TELEM_LENGTH, "MAX_RECEIVE_LENGTH is too small");

    if (_found_escs_count > 0) {
        uint8_t telem[FRAME_OVERHEAD + TELEM_LENGTH];
        ret = receive((uint8_t *) telem, ARRAY_SIZE(telem));

        if (ret == receive_response::ANSWER_VALID) {
            if (telem[1] <= _fast_throttle.min_id || telem[1] > _fast_throttle.max_id+1) {
                return receive_response::NO_ANSWER_YET; // this data came from an unexpected ESC
            }
            tlm_from_id = (uint8_t)telem[1]-1; // convert external ESC's one-indexed IDs to Ardupilot's internal zero-indexed IDs

            t.temperature_cdeg = int16_t(telem[5+0] * 100);
            t.voltage = float((telem[5+1]<<8)|telem[5+2]) * 0.01f;
            t.current = float((telem[5+3]<<8)|telem[5+4]) * 0.01f;
            centi_erpm = (telem[5+5]<<8)|telem[5+6];
            t.consumption_mah = float((telem[5+7]<<8)|telem[5+8]);
            tx_err_count = (telem[5+9]<<8)|telem[5+10];
        }
    }
    return ret;
}
#endif

/**
    if init is complete sends a single fast-throttle frame containing the throttle for all found OneWire ESCs.
    @param motor_values a 16bit array containing the throttle values that should be sent to the motors. 0-2000 where 1001-2000 is positive rotation and 0-999 reversed rotation
    @param tlm_request the ESC to request telemetry from (-1 for no telemetry, 0 for ESC1, 1 for ESC2, 2 for ESC3, ...)
*/
void AP_FETtecOneWire::escs_set_values(const uint16_t* motor_values, const int8_t tlm_request)
{
    if (_found_escs_count > 0) {
        // 8  bits - OneWire Header
        // 4  bits - telemetry request
        // 11 bits - throttle value per ESC
        // 8  bits - frame CRC
        // 7  dummy for rounding up the division by 8
        uint8_t fast_throttle_command[(8+4+(11*MOTOR_COUNT_MAX)+8+7)/8] { 0 };
        uint8_t act_throttle_command = 0;

        // byte 1:
        // bit 0,1,2,3 = ESC ID, Bit 4 = MSB bit of first ESC (11bit) throttle value, bit 5,6,7 = frame header
        // so AAAABCCC
        // A = ID from the ESC telemetry is requested from. ESC ID == 0 means no request.
        // B = MSB from first throttle value
        // C = frame header
        static_assert(MOTOR_COUNT_MAX<=15, "OneWire supports at most 15 ESCs, because of the 4 bit limitation bellow");
        fast_throttle_command[0] = (tlm_request+1) << 4; // convert from zero indexed to one-indexed. -1 (AP no telemetry) gets correctly converted to 0 (ESC no telemetry)
        fast_throttle_command[0] |= ((motor_values[act_throttle_command] >> 10) & 0x01) << 3;
        fast_throttle_command[0] |= 0x01;

        // byte 2:
        // AAABBBBB
        // A = next 3 bits from (11bit) throttle value
        // B = 5bit target ID
        fast_throttle_command[1] = (((motor_values[act_throttle_command] >> 7) & 0x07)) << 5;
        fast_throttle_command[1] |= 0x1F;      // All IDs

        // following bytes are the rest 7 bit of the first (11bit) throttle value,
        // and all bits from all other values, followed by the CRC byte
        uint8_t bits_left_from_command = 7;
        uint8_t act_byte = 2;
        uint8_t bits_from_byte_left = 8;
        int16_t bits_to_add_left = _fast_throttle.bits_to_add_left; // must be signed
        while (bits_to_add_left > 0) {
            if (bits_from_byte_left >= bits_left_from_command) {
                fast_throttle_command[act_byte] |=
                        (motor_values[act_throttle_command] & ((1 << bits_left_from_command) - 1))
                                << (bits_from_byte_left - bits_left_from_command);
                bits_to_add_left -= bits_left_from_command;
                bits_from_byte_left -= bits_left_from_command;
                act_throttle_command++;
                bits_left_from_command = 11;
                if (bits_to_add_left == 0) {
                    act_byte++;
                    bits_from_byte_left = 8;
                }
            } else {
                fast_throttle_command[act_byte] |=
                        (motor_values[act_throttle_command] >> (bits_left_from_command - bits_from_byte_left))
                                & ((1 << bits_from_byte_left) - 1);
                bits_to_add_left -= bits_from_byte_left;
                bits_left_from_command -= bits_from_byte_left;
                act_byte++;
                bits_from_byte_left = 8;
                if (bits_left_from_command == 0) {
                    act_throttle_command++;
                    bits_left_from_command = 11;
                }
            }
        }

        fast_throttle_command[_fast_throttle.byte_count - 1] =
            crc8_dvb_update(0, fast_throttle_command, _fast_throttle.byte_count - 1);

#if HAL_AP_FETTEC_HALF_DUPLEX
        // last byte of signal can be used to make sure the first TLM byte is correct, in case of spike corruption
        _last_crc = fast_throttle_command[_fast_throttle.byte_count - 1];
#endif

        // No command was yet sent, so no reply is expected and all information
        // on the receive buffer is either garbage or noise. Discard it
        _uart->discard_input();

        // send throttle commands to all configured ESCs in a single packet transfer
        transmit(fast_throttle_command, _fast_throttle.byte_count);
    }
}

bool AP_FETtecOneWire::pre_arm_check(char *failure_msg, const uint8_t failure_msg_len) const
{
    if (!_initialised) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Not initialised");
        return false;
    }
    return true;
}

/// periodically called from SRV_Channels::push()
void AP_FETtecOneWire::update()
{
    if (!_initialised) {
        init();
        return; // the rest of this function can only run after fully initted
    }

    const uint32_t now = AP_HAL::micros();
    if (now - _scan.last_us < 700U) {
        // the update() call period must be bigger than 700 us,
        // as to have time to receive the telemetry data
        return;
    }
    _scan.last_us = now;

    // get ESC set points
    uint16_t motor_pwm[MOTOR_COUNT_MAX] {};
    for (uint8_t i = 0; i < _nr_escs_in_bitmask; i++) {
        const SRV_Channel* c = SRV_Channels::srv_channel(i);
        if (c == nullptr) { // this should never ever happen, but just in case ...
            motor_pwm[i] = 1000;  // stop motor
            continue;
        }
        // check if safety switch has been pushed
        if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
            motor_pwm[i] = 1000;  // stop motor
        } else {
            motor_pwm[i] = constrain_int16(c->get_output_pwm(), 1000, 2000);
        }
        if (_reverse_mask & (1U << i)) {
            motor_pwm[i] = 2000-motor_pwm[i];
        }
    }

#if HAL_WITH_ESC_TELEM
    // receive and decode the telemetry data from one ESC
    // but do not process it any further to reduce timing jitter in the escs_set_values() function call
    TelemetryData t {};
    int16_t centi_erpm = 0;    // initialize to prevent false positive error: ‘centi_erpm’ may be used uninitialized in this function
    uint16_t tx_err_count = 0; // initialize to prevent false positive error: ‘tx_err_count’ may be used uninitialized in this function
    receive_response tlm_ok = receive_response::NO_ANSWER_YET;
    uint8_t tlm_from_id = 0;
    if (_requested_telemetry_from_esc != -1) {
        tlm_ok = decode_single_esc_telemetry(t, centi_erpm, tx_err_count, tlm_from_id);
        if (_nr_escs_in_bitmask) {
            _requested_telemetry_from_esc++;
            if (_requested_telemetry_from_esc > _fast_throttle.max_id) {
                _requested_telemetry_from_esc = _fast_throttle.min_id; // restart from the first ESC
            }
        }
    } else {
        _requested_telemetry_from_esc = _fast_throttle.min_id; // start from the first ESC
    }
#endif

    if (_nr_escs_in_bitmask) {
        // send motor setpoints to ESCs, and request for telemetry data
        escs_set_values(motor_pwm, _requested_telemetry_from_esc);

#if HAL_WITH_ESC_TELEM
        // now that escs_set_values() has been executed we can fully process the telemetry data from the ESC

        inc_sent_msg_count(); // increment message packet count for every ESC

        if (_requested_telemetry_from_esc != -1 && tlm_ok == receive_response::ANSWER_VALID) { //only use telemetry if it is ok.
            if (_pole_count_parameter < 2) { // if user set parameter is invalid use 14 Poles
                _pole_count_parameter = 14;
            }
            const float tx_err_rate = calc_tx_crc_error_perc(tlm_from_id, tx_err_count);
            update_rpm(tlm_from_id-_fast_throttle.min_id, centi_erpm*100*2/_pole_count_parameter.get(), tx_err_rate);

            update_telem_data(tlm_from_id-_fast_throttle.min_id, t, TelemetryType::TEMPERATURE|TelemetryType::VOLTAGE|TelemetryType::CURRENT|TelemetryType::CONSUMPTION);
        }
#endif
    }

    // Now that all real-time tasks above have been done, do some periodic checks.
    configuration_check();
}

#if HAL_AP_FETTEC_ESC_BEEP
/**
    makes all connected ESCs beep
    @param beep_frequency a 8 bit value from 0-255. higher make a higher beep
*/
void AP_FETtecOneWire::beep(const uint8_t beep_frequency)
{
    if (_found_escs_count > 0) {
        for (uint8_t i = _fast_throttle.min_id; i <= _fast_throttle.max_id; i++) {
            transmit(PackedMessage<Beep>{i, Beep{beep_frequency}});
        }
    }
}
#endif

#if HAL_AP_FETTEC_ESC_LIGHT
/**
    sets the racewire color for all ESCs
    @param r red brightness
    @param g green brightness
    @param b blue brightness
*/
void AP_FETtecOneWire::led_color(const uint8_t r, const uint8_t g, const uint8_t b)
{
    if (_found_escs_count > 0) {
        for (uint8_t i = _fast_throttle.min_id; i <= _fast_throttle.max_id; i++) {
            transmit(PackedMessage<LEDColour>{i, LEDColour{r, g, b}});
        }
    }
}
#endif

#endif  // HAL_AP_FETTEC_ONEWIRE_ENABLED
