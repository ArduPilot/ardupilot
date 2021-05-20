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

#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_FETtecOneWire.h"
#if HAL_AP_FETTECONEWIRE_ENABLED

#define FTOW_UART_LOCK_KEY 0x30150102


const AP_Param::GroupInfo AP_FETtecOneWire::var_info[] = {
    // @Param: MASK
    // @DisplayName: Channel Bitmask
    // @Description: Enable of FETtec OneWire ESC protocol to specific channels
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("MASK",  1, AP_FETtecOneWire, motor_mask, 0),

    // @Param: POLES
    // @DisplayName: Nr. electrical poles
    // @Description: Number of motor electrical poles
    // @Range: 2 50
    // @RebootRequired: False
    // @User: Standard
    AP_GROUPINFO("POLES", 2, AP_FETtecOneWire, pole_count, 14),

    AP_GROUPEND
};

AP_FETtecOneWire *AP_FETtecOneWire::_singleton;

AP_FETtecOneWire::AP_FETtecOneWire()
{
    _singleton = this;

    _response_length[OW_OK] = 1;
    _response_length[OW_BL_START_FW] = 0;       // BL only
    _response_length[OW_REQ_TYPE] = 1;
    _response_length[OW_REQ_SN] = 12;
    _response_length[OW_REQ_SW_VER] = 2;
    _response_length[OW_SET_FAST_COM_LENGTH] = 1;
    _response_length[OW_SET_TLM_TYPE] = 1;

    _request_length[OW_OK] = 1;
    _request_length[OW_BL_START_FW] = 1;       // BL only
    _request_length[OW_REQ_TYPE] = 1;
    _request_length[OW_REQ_SN] = 1;
    _request_length[OW_REQ_SW_VER] = 1;
    _request_length[OW_SET_FAST_COM_LENGTH] = 4;
    _request_length[OW_SET_TLM_TYPE] = 2;
}

void AP_FETtecOneWire::init()
{
    AP_SerialManager& serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FETtecOneWire, 0);
    if (_uart) {
        _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _uart->set_unbuffered_writes(true);
        _uart->set_blocking_writes(false);
        _uart->begin(2000000);
    }
}


void AP_FETtecOneWire::update()
{

    if (!_initialised) {
        _initialised = true;
        init();
        _last_send_us = AP_HAL::micros();
        return;
    }

    if (_uart == nullptr) {
        return;
    }

    if (!_uart->is_dma_enabled()){
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Uart for OneWire needs DMA!");
    }
    // tell SRV_Channels about ESC capabilities
    const uint16_t mask = uint16_t(motor_mask.get());
    SRV_Channels::set_digital_mask(mask);

    // get ESC set points, stop as soon as there is a gap
    uint8_t nr_escs = 0;
    uint16_t motor_pwm[MOTOR_COUNT_MAX] = {1000};

    for (uint8_t i = 0; i < MOTOR_COUNT_MAX; i++) {
        SRV_Channel* c = SRV_Channels::srv_channel(i);
        if (c == nullptr) {
            break;
        }
        nr_escs++;
        motor_pwm[i] = constrain_int16(c->get_output_pwm(), 0, 2000);
    }

#if HAL_WITH_ESC_TELEM
    // receive and decode the telemetry data from one ESC
    // but do not process it any further to reduce timing jitter in the escs_set_values() function call
    TelemetryData t {};
    int16_t centi_erpm;
    uint16_t tx_err_count;
    uint8_t tlm_ok = 0; //decode_single_esc_telemetry returns 1 if telemetry is ok, 0 if its waiting and 2 if there is a crc mismatch.
    uint8_t tlm_from_id = 0;
    if (_requested_telemetry_from_esc) {

        tlm_ok = decode_single_esc_telemetry(t, centi_erpm, tx_err_count, tlm_from_id);

    }
    if (_requested_telemetry_from_esc == _found_escs_count) { //if found esc number is reached restart request counter
        _requested_telemetry_from_esc = 1; // restart from ESC 1 
    } else {
        _requested_telemetry_from_esc++;
    }
#endif

    uint32_t txSpace =  _uart->txspace(); //Ugly workaround. 
    if (txSpace==511 && !_uart->tx_pending()){ //Ugly workaround. 
        escs_set_values(motor_pwm, _found_escs_count, _requested_telemetry_from_esc);
    }




#if HAL_WITH_ESC_TELEM
    // now that escs_set_values() has been executed we can fully process the telemetry data from the ESC

    calc_tx_crc_error_perc(0, 0, 1); //Just incrementing package count for every ESC. ID does not matter if increment only is set.

    if (_requested_telemetry_from_esc && tlm_ok==1) { //only use telemetry if it is ok.
        if (pole_count < 2) { // If Parameter is invalid use 14 Poles
            pole_count = 14;
        }
        const float tx_err_rate = calc_tx_crc_error_perc(_requested_telemetry_from_esc-1, tx_err_count, 0);
        update_rpm(tlm_from_id-1, centi_erpm*100*2/pole_count.get(), tx_err_rate);

        update_telem_data(tlm_from_id-1, t, AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE|AP_ESC_Telem_Backend::TelemetryType::VOLTAGE|AP_ESC_Telem_Backend::TelemetryType::CURRENT|AP_ESC_Telem_Backend::TelemetryType::CONSUMPTION);
    }
#endif

}

/**
    generates used 8 bit CRC for arrays
    @param buf 8 bit byte array
    @param buf_len count of bytes that should be used for CRC calculation
    @return 8 bit CRC
*/
uint8_t AP_FETtecOneWire::get_crc8(uint8_t* buf, uint16_t buf_len) const
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < buf_len; i++) {
        crc = crc8_telem(buf[i], crc);
    }
    return (crc);
}

/**
    transmits a FETtec OneWire frame to an ESC
    @param esc_id id of the ESC
    @param bytes 8 bit array of bytes. Where byte 1 contains the command, and all following bytes can be the payload
    @param length length of the bytes array
*/
void AP_FETtecOneWire::transmit(uint8_t esc_id, uint8_t* bytes, uint8_t length)
{
    /*
    a frame looks like:
    byte 1 = frame header (master is always 0x01)
    byte 2 = target ID (5bit)
    byte 3 & 4 = frame type (always 0x00, 0x00 used for bootloader. here just for compatibility)
    byte 5 = frame length over all bytes
    byte 6 - X = request type, followed by the payload
    byte X+1 = 8bit CRC
    */
    uint8_t transmit_arr[256] = {0x01, esc_id, 0x00, 0x00};
    transmit_arr[4] = length + 6;
    for (uint8_t i = 0; i < length; i++) {
        transmit_arr[i + 5] = bytes[i];
    }
    transmit_arr[length + 5] = get_crc8(transmit_arr, length + 5); // crc
    _uart->write_locked(transmit_arr, length + 6,FTOW_UART_LOCK_KEY);
    _ignore_own_bytes += length + 6;
}

/**
    reads the FETtec OneWire answer frame of an ESC
    @param bytes 8 bit byte array, where the received answer gets stored in
    @param length the expected answer length
    @param return_full_frame can be OW_RETURN_RESPONSE or OW_RETURN_FULL_FRAME
    @return 2 on CRC error, 1 if the expected answer frame was there, 0 if dont
*/
uint8_t AP_FETtecOneWire::receive(uint8_t* bytes, uint8_t length, uint8_t return_full_frame)
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

    //ignore own bytes
    while (_ignore_own_bytes > 0 && _uart->available()) {
        _ignore_own_bytes--;
        _uart->read();
    }
    // look for the real answerOW_SET_TLM_TYPE
    if (_uart->available() >= length + 6u) {
        // sync to frame starte byte
        uint8_t test_frame_start = 0;
        do {
            test_frame_start = _uart->read();
        }
        while (test_frame_start != 0x02 && test_frame_start != 0x03 && _uart->available());
        // copy message
        if (_uart->available() >= length + 5u) {
            uint8_t receive_buf[20] = {0};
            receive_buf[0] = test_frame_start;
            for (uint8_t i = 1; i < length + 6; i++) {
                receive_buf[i] = _uart->read();
            }
            // check CRC
            if (get_crc8(receive_buf, length + 5) == receive_buf[length + 5]) {
                if (!return_full_frame) {
                    for (uint8_t i = 0; i < length; i++) {
                        bytes[i] = receive_buf[5 + i];
                    }
                } else {
                    for (uint8_t i = 0; i < length + 6; i++) {
                        bytes[i] = receive_buf[i];
                    }
                }
                return 1; //correct CRC
            } else {
                return 2; // crc missmatch
            } 
        } else {
            return 0; // no answer yet
        } 
    } else {
        return 0; // no answer yet
    }
}

/**
    Resets a pending pull request
*/
void AP_FETtecOneWire::pull_reset()
{
    _pull_success = 0;
    _pull_busy = 0;
}

/**
    Pulls a complete request between flight controller and ESC
    @param esc_id  id of the ESC
    @param command 8bit array containing the command that should be send including the possible payload
    @param response 8bit array where the response will be stored in
    @param return_full_frame can be OW_RETURN_RESPONSE or OW_RETURN_FULL_FRAME
    @return 1 if the request is completed, 0 if dont
*/
uint8_t AP_FETtecOneWire::pull_command(uint8_t esc_id, uint8_t* command, uint8_t* response,
        uint8_t return_full_frame)
{
    if (!_pull_busy) {
        _pull_busy = 1;
        _pull_success = 0;
        transmit(esc_id, command, _request_length[command[0]]);
    } else {
        uint8_t recv_ret = receive(response, _response_length[command[0]], return_full_frame);
        switch (recv_ret) {
            case 1:
                _pull_success = 1;
                _pull_busy = 0;
                break;
            default:
                break;
        }
    }
    return _pull_success;
}

/**
    scans for ESCs in bus. should be called until _scan_active >= MOTOR_COUNT_MAX
    @return the current scanned ID
*/
uint8_t AP_FETtecOneWire::scan_escs()
{
    uint8_t response[18] = {0};
    uint8_t request[1] = {0};
    if (_scan_active == 0) {
        _ss.delay_loops = 500;
        _ss.scan_id = 0;
        _ss.scan_state = 0;
        _ss.scan_timeout = 0;
        return _scan_active + 1;
    }
    if (_ss.delay_loops > 0) {
        _ss.delay_loops--;
        return _scan_active;
    }
    if (_ss.scan_id < _scan_active) {
        _ss.scan_id = _scan_active;
        _ss.scan_state = 0;
        _ss.scan_timeout = 0;
    }
    if (_ss.scan_timeout == 3 || _ss.scan_timeout == 6 || _ss.scan_timeout == 9 || _ss.scan_timeout == 12) {
        pull_reset();
    }
    if (_ss.scan_timeout < 15) {
        switch (_ss.scan_state) {
        case 0:request[0] = OW_OK;
            if (pull_command(_ss.scan_id, request, response, OW_RETURN_FULL_FRAME)) {
                _ss.scan_timeout = 0;
                _active_esc_ids[_ss.scan_id] = 1;
                _found_escs_count++;
                if (response[0] == 0x02) {
                    _found_escs[_ss.scan_id].in_boot_loader = 1;
                } else {
                    _found_escs[_ss.scan_id].in_boot_loader = 0;
                }
                _ss.delay_loops = 1;
                _ss.scan_state++;
            } else {
                _ss.scan_timeout++;
            }
            break;
        case 1:request[0] = OW_REQ_TYPE;
            if (pull_command(_ss.scan_id, request, response, OW_RETURN_RESPONSE)) {
                _ss.scan_timeout = 0;
#if HAL_AP_FETTECONEWIRE_GET_STATIC_INFO
                _found_escs[_ss.scanID].esc_type = response[0];
#endif
                _ss.delay_loops = 1;
                _ss.scan_state++;
            } else {
                _ss.scan_timeout++;
            }
            break;
        case 2:request[0] = OW_REQ_SW_VER;
            if (pull_command(_ss.scan_id, request, response, OW_RETURN_RESPONSE)) {
                _ss.scan_timeout = 0;
#if HAL_AP_FETTECONEWIRE_GET_STATIC_INFO
                _found_escs[_ss.scanID].firmware_version = response[0];
                _found_escs[_ss.scanID].firmware_sub_version = response[1];
#endif
                _ss.delay_loops = 1;
                _ss.scan_state++;
            } else {
                _ss.scan_timeout++;
            }
            break;
        case 3:request[0] = OW_REQ_SN;
            if (pull_command(_ss.scan_id, request, response, OW_RETURN_RESPONSE)) {
                _ss.scan_timeout = 0;
#if HAL_AP_FETTECONEWIRE_GET_STATIC_INFO
                for (uint8_t i = 0; i < 12; i++) {
                    _found_escs[_ss.scanID].serialNumber[i] = response[i];
                }
#endif
                _ss.delay_loops = 1;
                return _ss.scan_id + 1;
            } else {
                _ss.scan_timeout++;
            }
            break;
        }
    } else {
        pull_reset();
        return _ss.scan_id + 1;
    }
    return _ss.scan_id;
}

/**
    sets the telemetry mode to full mode, where one ESC answers with all telem values including CRC Error count and a CRC
    @return returns the response code
*/
uint8_t AP_FETtecOneWire::set_full_telemetry(uint8_t active)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESC active state %i", _active_esc_ids[_set_full_telemetry_active-1]);
    if (_active_esc_ids[_set_full_telemetry_active]==1) { //If ESC is detected at this ID
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Setting full telemetry for ESC: %i", _set_full_telemetry_active);
        uint8_t response[1] = {0};
        uint8_t request[2] = {0};
        request[0] = OW_SET_TLM_TYPE;
        request[1] = active; //Alternative Tlm => 1, normal TLM => 0
        uint8_t pull_response = pull_command(_set_full_telemetry_active, request, response, OW_RETURN_RESPONSE);
        if(pull_response) {
            if(response[0] == OW_OK) {//Ok received or max retrys reached.
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "OW OK. Full Telemetry activ");
                _set_full_telemetry_active++;   //If answer from ESC is OK, increase ID.
                _set_full_telemetry_retry_count=0; //Reset retry count for new ESC ID
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "OW Fail");
                _set_full_telemetry_retry_count++; //No OK received, increase retry count
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Retry Count %i",_set_full_telemetry_retry_count);
            }
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PullCommand Fail %i",pull_response);
            _set_full_telemetry_retry_count++;
            if (_set_full_telemetry_retry_count>128) { //It is important to have the correct telemetry set so start over if there is something wrong.
                _pull_busy=0;
                _set_full_telemetry_active=1;
            }
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Retry Count %i",_set_full_telemetry_retry_count);
        }
    } else { //If there is no ESC detected skip it.
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "No ESC at ID: %i",_set_full_telemetry_active);
        _set_full_telemetry_active++;
    }
    return _set_full_telemetry_active;
}


/**
    starts all ESCs in bus and prepares them for receiving the fast throttle command should be called until _setup_active >= MOTOR_COUNT_MAX
    @return the current used ID
*/
uint8_t AP_FETtecOneWire::init_escs()
{
    uint8_t response[18] = {0};
    uint8_t request[1] = {0};
    if (_setup_active == 0) {
        _is.delay_loops = 0;
        _is.active_id = 1;
        _is.state = 0;
        _is.timeout = 0;
        _is.wake_from_bl = 1;
        return _setup_active + 1;
    }
    while (_active_esc_ids[_setup_active] == 0 && _setup_active < MOTOR_COUNT_MAX) {
        _setup_active++;
    }

    if (_setup_active == MOTOR_COUNT_MAX && _is.wake_from_bl == 0) {
        return _setup_active;
    } else if (_setup_active == MOTOR_COUNT_MAX && _is.wake_from_bl) {
        _is.wake_from_bl = 0;
        _is.active_id = 1;
        _setup_active = 1;
        _is.state = 0;
        _is.timeout = 0;

        _min_id = MOTOR_COUNT_MAX;
        _max_id = 0;
        _id_count = 0;
        for (uint8_t i = 0; i < MOTOR_COUNT_MAX; i++) {
            if (_active_esc_ids[i] != 0) {
                _id_count++;
                if (i < _min_id) {
                    _min_id = i;
                }
                if (i > _max_id) {
                    _max_id = i;
                }
            }
        }

        if (_id_count == 0
                || _max_id - _min_id > _id_count - 1) { // loop forever
            _is.wake_from_bl = 1;
            return _is.active_id;
        }
        _fast_throttle_byte_count = 1;
        int8_t bitCount = 12 + (_id_count * 11);
        while (bitCount > 0) {
            _fast_throttle_byte_count++;
            bitCount -= 8;
        }
        _is.set_fast_command[1] = _fast_throttle_byte_count; // just for older ESC FW versions since 1.0 001 this byte is ignored as the ESC calculates it itself
        _is.set_fast_command[2] = _min_id;                 // min ESC id
        _is.set_fast_command[3] = _id_count;               // count of ESCs that will get signals
    }

    if (_is.delay_loops > 0) {
        _is.delay_loops--;
        return _setup_active;
    }

    if (_is.active_id < _setup_active) {
        _is.active_id = _setup_active;
        _is.state = 0;
        _is.timeout = 0;
    }

    if (_is.timeout == 3 || _is.timeout == 6 || _is.timeout == 9 || _is.timeout == 12) {
        pull_reset();
    }

    if (_is.timeout < 15) {
        if (_is.wake_from_bl) {
            switch (_is.state) {
            case 0:request[0] = OW_BL_START_FW;
                if (_found_escs[_is.active_id].in_boot_loader == 1) {
                    transmit(_is.active_id, request, _request_length[request[0]]);
                    _is.delay_loops = 5;
                } else {
                    return _is.active_id + 1;
                }
                _is.state = 1;
                break;
            case 1:request[0] = OW_OK;
                if (pull_command(_is.active_id, request, response, OW_RETURN_FULL_FRAME)) {
                    _is.timeout = 0;
                    if (response[0] == 0x02) {
                        _found_escs[_is.active_id].in_boot_loader = 1;
                        _is.state = 0;
                    } else {
                        _found_escs[_is.active_id].in_boot_loader = 0;
                        _is.delay_loops = 1;
                        return _is.active_id + 1;
                    }
                } else {
                    _is.timeout++;
                }
                break;
            }
        } else {
            if (pull_command(_is.active_id, _is.set_fast_command, response, OW_RETURN_RESPONSE)) {
                _is.timeout = 0;
                _is.delay_loops = 1;
                return _is.active_id + 1;
            } else {
                _is.timeout++;
            }
        }
    } else {
        pull_reset();
        return _is.active_id + 1;
    }
    return _is.active_id;
}

/**
    calculates crc tx error rate for incoming packages. It converts the CRC error counts into percentage
    @param esc_id id of ESC, that the error is calculated for
    @param current_error_count the error count given by the esc
    @param increment_only if this is set to 1 it only increases the message count and returns 0. If set to 1 it does not increment but gives back the error count.
    @return the error in percent
*/
float AP_FETtecOneWire::calc_tx_crc_error_perc(uint8_t esc_id, uint16_t current_error_count, uint8_t increment_only){

    static uint16_t error_count[MOTOR_COUNT_MAX]; //saves the error counter from the ESCs
    static uint16_t error_count_since_overflow[MOTOR_COUNT_MAX]; //saves the error counter from the ESCs to pass the overflow
    static uint16_t send_msg_count = 0; //counts the messages that are send by fc
    float error_count_percentage = 0.0f;
    #define HZ_MOTOR 400.0
    #define PERCENTAGE_DIVIDER (100.0/HZ_MOTOR) //to save the division in loop precalculate by the motor loops 100%/400Hz

    if (increment_only) {
        send_msg_count++;
        if (send_msg_count > 63000) { // The update loop runs at 400Hz, hence this resets every second
            send_msg_count = 0; //reset the counter
            for (int i=0; i<_found_escs_count; i++) {
                    error_count_since_overflow[i] = error_count[i]; //save the current ESC error state               
            }
        }
    } else { //Calculate the percentage
        error_count[esc_id] = current_error_count; //Save the error count to the esc
        uint16_t corrected_error_count = (uint16_t)((uint16_t)error_count[esc_id] - (uint16_t)error_count_since_overflow[esc_id]); //calculates error difference since last overflow.
        error_count_percentage = (float)corrected_error_count*(float)PERCENTAGE_DIVIDER; //calculates percentage

        //Debug or Info
       // if (send_msg_count==1200-4 || send_msg_count==1200-3 || send_msg_count==1200-2 ||send_msg_count==1200-1) {
        //   GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%i: p: %.2f, m: %u, eD: %u, eEr: %u, Ov: %u", esc_id, error_count_percentage, send_msg_count, corrected_error_count, error_count[esc_id],error_count_since_overflow[esc_id]);
        //}
    }
    return error_count_percentage;
}

#if HAL_WITH_ESC_TELEM
/**
    checks if the requested telemetry is available.
    @param t telemetry datastructure where the read telemetry will be stored in.
    @param centi_erpm 16bit centi-eRPM value returned from the ESC
    @param tx_err_count Ardupilot->ESC communication CRC error counter
    @param tlm_from_id receives the ID from the ESC that has respond with its telemetry
    @return 1 if CRC is correct, 2 on CRC mismatch, 0 on waiting for answer
*/
int8_t AP_FETtecOneWire::decode_single_esc_telemetry(TelemetryData& t, int16_t& centi_erpm, uint16_t& tx_err_count, uint8_t &tlm_from_id)
{
    int8_t ret = 0;
    if (_id_count > 0) {
        uint8_t telem[17] = {0};
        ret = receive((uint8_t *) telem, 11, OW_RETURN_FULL_FRAME); 

        if (ret == 1) { //Answer ok
            tlm_from_id = (uint8_t)telem[1];

            t.temperature_cdeg = int16_t(telem[5+0] * 100);
            t.voltage = float((telem[5+1]<<8)|telem[5+2]) * 0.01f;
            t.current = float((telem[5+3]<<8)|telem[5+4]) * 0.01f;
            centi_erpm = (telem[5+5]<<8)|telem[5+6];
            t.consumption_mah = float((telem[5+7]<<8)|telem[5+8]);
            tx_err_count = (telem[5+9]<<8)|telem[5+10];
        }
        if (ret == 2) { //CRC Error
            increment_CRC_error_counter(_requested_telemetry_from_esc-1);
        }
    } else {
        ret = -1;
    }
    return ret;
}
#endif

/**
    scans for ESCs if not already done.
    initializes the ESCs if not already done.
    sends fast throttle signals if init is complete.
    @param motor_values a 16bit array containing the throttle signals that should be sent to the motors. 0-2000 where 1001-2000 is positive rotation and 999-0 reversed rotation
    @param telemetry 16bit array where the read telemetry will be stored in.
    @param motorCount the count of motors that should get values send
    @param tlm_request the ESC to request telemetry from (0 for no telemetry, 1 for ESC0, 2 for ESC1, 3 for ESC2, ...)
*/
void AP_FETtecOneWire::escs_set_values(const uint16_t* motor_values, const uint8_t motorCount, const uint8_t tlm_request)
{
    // init should not be done too fast. as at last the bootloader has some timing requirements with messages. so loop delays must fit more or less
    if (_scan_active < MOTOR_COUNT_MAX || _setup_active < MOTOR_COUNT_MAX || _set_full_telemetry_active < MOTOR_COUNT_MAX ) {
        const uint32_t now = AP_HAL::micros();
        if (now - _last_send_us < DELAY_TIME_US) {
            return;
        } else {
            _last_send_us = now;
        }

        if (_scan_active < MOTOR_COUNT_MAX) {
            // scan for all ESCs in onewire bus
            _scan_active = scan_escs();

        } else if (_setup_active < MOTOR_COUNT_MAX) {
            if (_found_escs_count == 0) {
                _scan_active = 0;
            } else {
                // check if in bootloader, start ESCs FW if they are and prepare fast throttle command
                _setup_active = init_escs();
            }

        }
        else if (_set_full_telemetry_active < MOTOR_COUNT_MAX+1) { //Set telemetry to alternative mode
               _set_full_telemetry_active = set_full_telemetry(1);
        }
    } else {
        //send fast throttle signals
        if (_id_count > 0) {
            uint8_t fast_throttle_command[36] = {0};
            uint8_t act_throttle_command = 0;

            // byte 1:
            // bit 0,1,2,3 = ESC ID, Bit 4 = first bit of first ESC (11bit)signal, bit 5,6,7 = frame header
            // so AAAABCCC
            // A = ESC ID, telemetry is requested from. ESC ID == 0 means no request. 
            // B = first bit from first throttle signal
            // C = frame header
            fast_throttle_command[0] = (tlm_request << 4);
            fast_throttle_command[0] |= ((motor_values[act_throttle_command] >> 10) & 0x01) << 3;
            fast_throttle_command[0] |= 0x01;
            // byte 2:
            // AAABBBBB
            // A = next 3 bits from (11bit)throttle signal
            // B = 5bit target ID
            fast_throttle_command[1] = (((motor_values[act_throttle_command] >> 7) & 0x07)) << 5;
            fast_throttle_command[1] |= ALL_ID;

            // following bytes are the rest 7 bit of the first (11bit) throttle signal, and all bit from all other signals, followed by the CRC byte
            uint8_t bits_left_from_command = 7;
            uint8_t act_byte = 2;
            uint8_t bits_from_byte_left = 8;
            uint8_t bits_to_add_left = (12 + (((_max_id - _min_id) + 1) * 11)) - 16;
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
            // empty buffer
            while (_uart->available()) {
                _uart->read();
            }

            // send throttle signal
            fast_throttle_command[_fast_throttle_byte_count - 1] = get_crc8(
                    fast_throttle_command, _fast_throttle_byte_count - 1);
            _uart->write_locked(fast_throttle_command, _fast_throttle_byte_count,FTOW_UART_LOCK_KEY);
            // last byte of signal can be used to make sure the first TLM byte is correct, in case of spike corruption
            _ignore_own_bytes = _fast_throttle_byte_count - 1;
            _last_crc = fast_throttle_command[_fast_throttle_byte_count - 1];
            // the ESCs will answer the TLM as 16bit each ESC, so 2byte each ESC. @torsten: is this comment correct?
        }
    }
}
#endif  // HAL_AP_FETTECONEWIRE_ENABLED