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

#include "AP_FETtecOneWire.h"
#if HAL_AP_FETTECONEWIRE_ENABLED

const AP_Param::GroupInfo AP_FETtecOneWire::var_info[] = {
    // @Param: MASK
    // @DisplayName: Channel Bitmask
    // @Description: Enable of FETtec OneWire ESC protocol to specific channels
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("MASK",  1, AP_FETtecOneWire, motor_mask, 0),

    AP_GROUPEND
};

AP_FETtecOneWire *AP_FETtecOneWire::_singleton;

AP_FETtecOneWire::AP_FETtecOneWire()
{
    _singleton = this;

    _ResponseLength[OW_OK] = 1;
    _ResponseLength[OW_BL_START_FW] = 0;       // BL only
    _ResponseLength[OW_REQ_TYPE] = 1;
    _ResponseLength[OW_REQ_SN] = 12;
    _ResponseLength[OW_REQ_SW_VER] = 2;
    _ResponseLength[OW_SET_FAST_COM_LENGTH] = 1;

    _RequestLength[OW_OK] = 1;
    _RequestLength[OW_BL_START_FW] = 1;       // BL only
    _RequestLength[OW_REQ_TYPE] = 1;
    _RequestLength[OW_REQ_SN] = 1;
    _RequestLength[OW_REQ_SW_VER] = 1;
    _RequestLength[OW_SET_FAST_COM_LENGTH] = 4;
}

void AP_FETtecOneWire::init()
{
    AP_SerialManager& serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FETtecOneWire, 0);
    if (_uart) {
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

    const uint16_t mask = uint16_t(motor_mask.get());

    // tell SRV_Channels about ESC capabilities
    SRV_Channels::set_digital_mask(mask);
    for (uint8_t i = 0; i < MOTOR_COUNT_MAX; i++) {
        SRV_Channel* c = SRV_Channels::srv_channel(i);
        if (c == nullptr) {
            continue;
        }
        _motorpwm[i] = c->get_output_pwm();
    }

    uint16_t requestedTelemetry[MOTOR_COUNT_MAX] = {0};
    _telem_avail = ESCsSetValues(_motorpwm, requestedTelemetry, MOTOR_COUNT_MAX, _telem_req_type);

    if (++_telem_req_type == telem_type::DEBUG1) {
        // telem_type::DEBUG1, telem_type::DEBUG2, telem_type::DEBUG3 are ignored
        _telem_req_type = telem_type::TEMP;
    }

    if (_telem_avail != -1) {
        TelemetryData t {};
        uint16_t esc_mask = 1;
        for (uint8_t i = 0; i < MOTOR_COUNT_MAX; i++) {
            if (mask & esc_mask) { // only update telemetry of enabled ESCs
                switch(_telem_avail) {
                case telem_type::TEMP:
                    t.temperature_cdeg = int16_t(100 * requestedTelemetry[i]);
                    update_telem_data(i, t, AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
                    break;

                case telem_type::VOLT:
                    t.voltage = float(requestedTelemetry[i]) * 0.01f;  // centi-Volt to Volt
                    update_telem_data(i, t, AP_ESC_Telem_Backend::TelemetryType::VOLTAGE);
                    break;

                case telem_type::CURRENT:
                    t.current = float(requestedTelemetry[i]) * 0.01f;  // centi-Ampere to Ampere
                    update_telem_data(i, t, AP_ESC_Telem_Backend::TelemetryType::CURRENT);
                    break;

                case telem_type::ERPM:
                    update_rpm(i, requestedTelemetry[i]);
                    break;

                case telem_type::CONSUMPTION:
                    t.consumption_mah = float(requestedTelemetry[i]);
                    update_telem_data(i, t, AP_ESC_Telem_Backend::TelemetryType::CONSUMPTION);
                    break;
                }
            }
            esc_mask <<= 1;
        }
    }
}

/**
    generates used 8 bit CRC for arrays
    @param Buf 8 bit byte array
    @param BufLen count of bytes that should be used for CRC calculation
    @return 8 bit CRC
*/
uint8_t AP_FETtecOneWire::Get_crc8(uint8_t* Buf, uint16_t BufLen) const
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < BufLen; i++) {
        crc = crc8_telem(Buf[i], crc);
    }
    return (crc);
}

/**
    transmits a FETtec OneWire frame to an ESC
    @param ESC_id id of the ESC
    @param Bytes 8 bit array of bytes. Where byte 1 contains the command, and all following bytes can be the payload
    @param Length length of the Bytes array
*/
void AP_FETtecOneWire::Transmit(uint8_t ESC_id, uint8_t* Bytes, uint8_t Length)
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
    uint8_t transmitArr[256] = {0x01, ESC_id, 0x00, 0x00};
    transmitArr[4] = Length + 6;
    for (uint8_t i = 0; i < Length; i++) {
        transmitArr[i + 5] = Bytes[i];
    }
    transmitArr[Length + 5] = Get_crc8(transmitArr, Length + 5); // crc
    _uart->write(transmitArr, Length + 6);
    _IgnoreOwnBytes += Length + 6;
}

/**
    reads the FETtec OneWire answer frame of an ESC
    @param Bytes 8 bit byte array, where the received answer gets stored in
    @param Length the expected answer length
    @param returnFullFrame can be OW_RETURN_RESPONSE or OW_RETURN_FULL_FRAME
    @return 1 if the expected answer frame was there, 0 if dont
*/
uint8_t AP_FETtecOneWire::Receive(uint8_t* Bytes, uint8_t Length, uint8_t returnFullFrame)
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
    while (_IgnoreOwnBytes > 0 && _uart->available()) {
        _IgnoreOwnBytes--;
        _uart->read();
    }
    // look for the real answer
    if (_uart->available() >= Length + 6u) {
        // sync to frame starte byte
        uint8_t testFrameStart = 0;
        do {
            testFrameStart = _uart->read();
        }
        while (testFrameStart != 0x02 && testFrameStart != 0x03 && _uart->available());
        // copy message
        if (_uart->available() >= Length + 5u) {
            uint8_t ReceiveBuf[20] = {0};
            ReceiveBuf[0] = testFrameStart;
            for (uint8_t i = 1; i < Length + 6; i++) {
                ReceiveBuf[i] = _uart->read();
            }
            // check CRC
            if (Get_crc8(ReceiveBuf, Length + 5) == ReceiveBuf[Length + 5]) {
                if (!returnFullFrame) {
                    for (uint8_t i = 0; i < Length; i++) {
                        Bytes[i] = ReceiveBuf[5 + i];
                    }
                } else {
                    for (uint8_t i = 0; i < Length + 6; i++) {
                        Bytes[i] = ReceiveBuf[i];
                    }
                }
                return 1;
            } else {
                return 0;
            } // crc missmatch
        } else {
            return 0;
        } // no answer yet
    } else {
        return 0;
    } // no answer yet
}

/**
    Resets a pending pull request
*/
void AP_FETtecOneWire::PullReset()
{
    _PullSuccess = 0;
    _PullBusy = 0;
}

/**
    Pulls a complete request between flight controller and ESC
    @param ESC_id  id of the ESC
    @param command 8bit array containing the command that should be send including the possible payload
    @param response 8bit array where the response will be stored in
    @param returnFullFrame can be OW_RETURN_RESPONSE or OW_RETURN_FULL_FRAME
    @return 1 if the request is completed, 0 if dont
*/
uint8_t AP_FETtecOneWire::PullCommand(uint8_t ESC_id, uint8_t* command, uint8_t* response,
        uint8_t returnFullFrame)
{
    if (!_PullBusy) {
        _PullBusy = 1;
        _PullSuccess = 0;
        Transmit(ESC_id, command, _RequestLength[command[0]]);
    } else {
        if (Receive(response, _ResponseLength[command[0]], returnFullFrame)) {
            _PullSuccess = 1;
            _PullBusy = 0;
        }
    }
    return _PullSuccess;
}

/**
    scans for ESCs in bus. should be called until _ScanActive >= MOTOR_COUNT_MAX
    @return the current scanned ID
*/
uint8_t AP_FETtecOneWire::ScanESCs()
{
    uint8_t response[18] = {0};
    uint8_t request[1] = {0};
    if (_ScanActive == 0) {
        _ss.delayLoops = 500;
        _ss.scanID = 0;
        _ss.scanState = 0;
        _ss.scanTimeOut = 0;
        return _ScanActive + 1;
    }
    if (_ss.delayLoops > 0) {
        _ss.delayLoops--;
        return _ScanActive;
    }
    if (_ss.scanID < _ScanActive) {
        _ss.scanID = _ScanActive;
        _ss.scanState = 0;
        _ss.scanTimeOut = 0;
    }
    if (_ss.scanTimeOut == 3 || _ss.scanTimeOut == 6 || _ss.scanTimeOut == 9 || _ss.scanTimeOut == 12) {
        PullReset();
    }
    if (_ss.scanTimeOut < 15) {
        switch (_ss.scanState) {
        case 0:request[0] = OW_OK;
            if (PullCommand(_ss.scanID, request, response, OW_RETURN_FULL_FRAME)) {
                _ss.scanTimeOut = 0;
                _activeESC_IDs[_ss.scanID] = 1;
                _FoundESCs++;
                if (response[0] == 0x02) {
                    _foundESCs[_ss.scanID].inBootLoader = 1;
                } else {
                    _foundESCs[_ss.scanID].inBootLoader = 0;
                }
                _ss.delayLoops = 1;
                _ss.scanState++;
            } else {
                _ss.scanTimeOut++;
            }
            break;
        case 1:request[0] = OW_REQ_TYPE;
            if (PullCommand(_ss.scanID, request, response, OW_RETURN_RESPONSE)) {
                _ss.scanTimeOut = 0;
#if HAL_AP_FETTECONEWIRE_GET_STATIC_INFO
                _foundESCs[_ss.scanID].ESCtype = response[0];
#endif
                _ss.delayLoops = 1;
                _ss.scanState++;
            } else {
                _ss.scanTimeOut++;
            }
            break;
        case 2:request[0] = OW_REQ_SW_VER;
            if (PullCommand(_ss.scanID, request, response, OW_RETURN_RESPONSE)) {
                _ss.scanTimeOut = 0;
#if HAL_AP_FETTECONEWIRE_GET_STATIC_INFO
                _foundESCs[_ss.scanID].firmWareVersion = response[0];
                _foundESCs[_ss.scanID].firmWareSubVersion = response[1];
#endif
                _ss.delayLoops = 1;
                _ss.scanState++;
            } else {
                _ss.scanTimeOut++;
            }
            break;
        case 3:request[0] = OW_REQ_SN;
            if (PullCommand(_ss.scanID, request, response, OW_RETURN_RESPONSE)) {
                _ss.scanTimeOut = 0;
#if HAL_AP_FETTECONEWIRE_GET_STATIC_INFO
                for (uint8_t i = 0; i < 12; i++) {
                    _foundESCs[_ss.scanID].serialNumber[i] = response[i];
                }
#endif
                _ss.delayLoops = 1;
                return _ss.scanID + 1;
            } else {
                _ss.scanTimeOut++;
            }
            break;
        }
    } else {
        PullReset();
        return _ss.scanID + 1;
    }
    return _ss.scanID;
}

/**
    starts all ESCs in bus and prepares them for receiving the fast throttle command should be called until _SetupActive >= MOTOR_COUNT_MAX
    @return the current used ID
*/
uint8_t AP_FETtecOneWire::InitESCs()
{
    uint8_t response[18] = {0};
    uint8_t request[1] = {0};
    if (_SetupActive == 0) {
        _is.delayLoops = 0;
        _is.activeID = 1;
        _is.State = 0;
        _is.TimeOut = 0;
        _is.wakeFromBL = 1;
        return _SetupActive + 1;
    }
    while (_activeESC_IDs[_SetupActive] == 0 && _SetupActive < MOTOR_COUNT_MAX) {
        _SetupActive++;
    }

    if (_SetupActive == MOTOR_COUNT_MAX && _is.wakeFromBL == 0) {
        return _SetupActive;
    } else if (_SetupActive == MOTOR_COUNT_MAX && _is.wakeFromBL) {
        _is.wakeFromBL = 0;
        _is.activeID = 1;
        _SetupActive = 1;
        _is.State = 0;
        _is.TimeOut = 0;

        _minID = MOTOR_COUNT_MAX;
        _maxID = 0;
        _IDcount = 0;
        for (uint8_t i = 0; i < MOTOR_COUNT_MAX; i++) {
            if (_activeESC_IDs[i] != 0) {
                _IDcount++;
                if (i < _minID) {
                    _minID = i;
                }
                if (i > _maxID) {
                    _maxID = i;
                }
            }
        }

        if (_IDcount == 0
                || _maxID - _minID > _IDcount - 1) { // loop forever
            _is.wakeFromBL = 1;
            return _is.activeID;
        }
        _FastThrottleByteCount = 1;
        int8_t bitCount = 12 + (_IDcount * 11);
        while (bitCount > 0) {
            _FastThrottleByteCount++;
            bitCount -= 8;
        }
        _is.setFastCommand[1] = _FastThrottleByteCount; // just for older ESC FW versions since 1.0 001 this byte is ignored as the ESC calculates it itself
        _is.setFastCommand[2] = _minID;                 // min ESC id
        _is.setFastCommand[3] = _IDcount;               // count of ESCs that will get signals
    }

    if (_is.delayLoops > 0) {
        _is.delayLoops--;
        return _SetupActive;
    }

    if (_is.activeID < _SetupActive) {
        _is.activeID = _SetupActive;
        _is.State = 0;
        _is.TimeOut = 0;
    }

    if (_is.TimeOut == 3 || _is.TimeOut == 6 || _is.TimeOut == 9 || _is.TimeOut == 12) {
        PullReset();
    }

    if (_is.TimeOut < 15) {
        if (_is.wakeFromBL) {
            switch (_is.State) {
            case 0:request[0] = OW_BL_START_FW;
                if (_foundESCs[_is.activeID].inBootLoader == 1) {
                    Transmit(_is.activeID, request, _RequestLength[request[0]]);
                    _is.delayLoops = 5;
                } else {
                    return _is.activeID + 1;
                }
                _is.State = 1;
                break;
            case 1:request[0] = OW_OK;
                if (PullCommand(_is.activeID, request, response, OW_RETURN_FULL_FRAME)) {
                    _is.TimeOut = 0;
                    if (response[0] == 0x02) {
                        _foundESCs[_is.activeID].inBootLoader = 1;
                        _is.State = 0;
                    } else {
                        _foundESCs[_is.activeID].inBootLoader = 0;
                        _is.delayLoops = 1;
                        return _is.activeID + 1;
                    }
                } else {
                    _is.TimeOut++;
                }
                break;
            }
        } else {
            if (PullCommand(_is.activeID, _is.setFastCommand, response, OW_RETURN_RESPONSE)) {
                _is.TimeOut = 0;
                _is.delayLoops = 1;
                return _is.activeID + 1;
            } else {
                _is.TimeOut++;
            }
        }
    } else {
        PullReset();
        return _is.activeID + 1;
    }
    return _is.activeID;
}

/**
    checks if the requested telemetry is available.
    @param Telemetry 16bit array where the read Telemetry will be stored in.
    @return the telemetry request number or -1 if unavailable
*/
int8_t AP_FETtecOneWire::CheckForTLM(uint16_t* Telemetry)
{
    int8_t return_TLM_request = 0;
    if (_IDcount > 0) {
        // empty buffer
        while (_IgnoreOwnBytes > 0 && _uart->available()) {
            _uart->read();
            _IgnoreOwnBytes--;
        }

        // first two byte are the ESC Telemetry of the first ESC. next two byte of the second....
        if (_uart->available() == (_IDcount * 2) + 1u) {
            // look if first byte in buffer is equal to last byte of throttle command (crc)
            if (_uart->read() == _lastCRC) {
                for (uint8_t i = 0; i < _IDcount; i++) {
                    Telemetry[i] = _uart->read() << 8;
                    Telemetry[i] |= _uart->read();
                }
                return_TLM_request = _TLM_request;
            } else {
                return_TLM_request = -1;
            }
        } else {
            return_TLM_request = -1;
        }
    } else {
        return_TLM_request = -1;
    }
    return return_TLM_request;
}

/**
    does almost all of the job.
    scans for ESCs if not already done.
    initializes the ESCs if not already done.
    sends fast throttle signals if init is complete.
    @param motorValues a 16bit array containing the throttle signals that should be sent to the motors. 0-2000 where 1001-2000 is positive rotation and 999-0 reversed rotation
    @param Telemetry 16bit array where the read telemetry will be stored in.
    @param motorCount the count of motors that should get values send
    @param tlmRequest the requested telemetry type (telem_type::XXXXX)
    @return the telemetry request if telemetry was available, -1 if dont
*/
int8_t AP_FETtecOneWire::ESCsSetValues(uint16_t* motorValues, uint16_t* Telemetry, uint8_t motorCount,
        uint8_t tlmRequest)
{
    int8_t return_TLM_request = -2;

    // init should not be done too fast. as at last the bootloader has some timing requirements with messages. so loop delays must fit more or less
    if (_ScanActive < MOTOR_COUNT_MAX || _SetupActive < MOTOR_COUNT_MAX) {
        const uint32_t now = AP_HAL::micros();
        if (now - _last_send_us < DELAY_TIME_US) {
            return 0;
        } else {
            _last_send_us = now;
        }

        if (_ScanActive < MOTOR_COUNT_MAX) {
            // scan for all ESCs in onewire bus
            _ScanActive = ScanESCs();
        } else if (_SetupActive < MOTOR_COUNT_MAX) {
            if (_FoundESCs == 0) {
                _ScanActive = 0;
            } else {
                // check if in bootloader, start ESCs FW if they are and prepare fast throttle command
                _SetupActive = InitESCs();
            }
        }
    } else {
        //send fast throttle signals
        if (_IDcount > 0) {

            // check for telemetry
            return_TLM_request = CheckForTLM(Telemetry);
            _TLM_request = tlmRequest;

            //prepare fast throttle signals
            uint16_t useSignals[24] = {0};
            uint8_t fast_throttle_command[36] = {0};
            if (motorCount > _IDcount) {
                motorCount = _IDcount;
            }
            for (uint8_t i = 0; i < motorCount; i++) {
                useSignals[i] = constrain_int16(motorValues[i], 0, 2000);
            }

            uint8_t actThrottleCommand = 0;

            // byte 1:
            // bit 0 = TLMrequest, bit 1,2,3 = TLM type, bit 4 = first bit of first ESC (11bit)signal, bit 5,6,7 = frame header
            // so ABBBCDDD
            // A = TLM request yes or no
            // B = TLM request type (temp, volt, current, erpm, consumption, debug1, debug2, debug3)
            // C = first bit from first throttle signal
            // D = frame header
            fast_throttle_command[0] = 128 | (_TLM_request << 4);
            fast_throttle_command[0] |= ((useSignals[actThrottleCommand] >> 10) & 0x01) << 3;
            fast_throttle_command[0] |= 0x01;

            // byte 2:
            // AAABBBBB
            // A = next 3 bits from (11bit)throttle signal
            // B = 5bit target ID
            fast_throttle_command[1] = (((useSignals[actThrottleCommand] >> 7) & 0x07)) << 5;
            fast_throttle_command[1] |= ALL_ID;

            // following bytes are the rest 7 bit of the first (11bit) throttle signal, and all bit from all other signals, followed by the CRC byte
            uint8_t BitsLeftFromCommand = 7;
            uint8_t actByte = 2;
            uint8_t bitsFromByteLeft = 8;
            uint8_t bitsToAddLeft = (12 + (((_maxID - _minID) + 1) * 11)) - 16;
            while (bitsToAddLeft > 0) {
                if (bitsFromByteLeft >= BitsLeftFromCommand) {
                    fast_throttle_command[actByte] |=
                            (useSignals[actThrottleCommand] & ((1 << BitsLeftFromCommand) - 1))
                                    << (bitsFromByteLeft - BitsLeftFromCommand);
                    bitsToAddLeft -= BitsLeftFromCommand;
                    bitsFromByteLeft -= BitsLeftFromCommand;
                    actThrottleCommand++;
                    BitsLeftFromCommand = 11;
                    if (bitsToAddLeft == 0) {
                        actByte++;
                        bitsFromByteLeft = 8;
                    }
                } else {
                    fast_throttle_command[actByte] |=
                            (useSignals[actThrottleCommand] >> (BitsLeftFromCommand - bitsFromByteLeft))
                                    & ((1 << bitsFromByteLeft) - 1);
                    bitsToAddLeft -= bitsFromByteLeft;
                    BitsLeftFromCommand -= bitsFromByteLeft;
                    actByte++;
                    bitsFromByteLeft = 8;
                    if (BitsLeftFromCommand == 0) {
                        actThrottleCommand++;
                        BitsLeftFromCommand = 11;
                    }
                }
            }
            // empty buffer
            while (_uart->available()) {
                _uart->read();
            }

            // send throttle signal
            fast_throttle_command[_FastThrottleByteCount - 1] = Get_crc8(
                    fast_throttle_command, _FastThrottleByteCount - 1);
            _uart->write(fast_throttle_command, _FastThrottleByteCount);
            // last byte of signal can be used to make sure the first TLM byte is correct, in case of spike corruption
            _IgnoreOwnBytes = _FastThrottleByteCount - 1;
            _lastCRC = fast_throttle_command[_FastThrottleByteCount - 1];
            // the ESCs will answer the TLM as 16bit each ESC, so 2byte each ESC.
        }
    }
    return return_TLM_request; // returns the read tlm as it is 1 loop delayed
}
#endif  // HAL_AP_FETTECONEWIRE_ENABLED
