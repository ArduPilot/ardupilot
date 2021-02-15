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
#include <SRV_Channel/SRV_Channel.h>

#ifndef HAL_AP_FETTECONEWIRE_ENABLED
#define HAL_AP_FETTECONEWIRE_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#if HAL_AP_FETTECONEWIRE_ENABLED

#include <AP_Param/AP_Param.h>



class AP_FETtecOneWire {
public:
    AP_FETtecOneWire();

    /// Do not allow copies
    AP_FETtecOneWire(const AP_FETtecOneWire &other) = delete;
    AP_FETtecOneWire &operator=(const AP_FETtecOneWire&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    void update();
    void send_esc_telemetry_mavlink(uint8_t mav_chan) const;
    static AP_FETtecOneWire *get_singleton() {
        return _singleton;
    }
private:
    void init();
    static AP_FETtecOneWire *_singleton;
    bool _initialised;
    AP_HAL::UARTDriver *_uart;
    AP_Int32 motor_mask;

    uint32_t _last_send_us;
    uint32_t _last_log_ms;
    static constexpr uint32_t DELAY_TIME_US = 700;
    static constexpr uint8_t MOTOR_COUNT_MAX = 12; /// OneWire supports up-to 25 ESCs, but Ardupilot only supports 12
    int8_t _telem_avail = -1;
    /// contains 6 fields per ESC:
    ///  - _telemetry[i][0] -> temperature[idx]
    ///  - _telemetry[i][1] -> voltage[idx]
    ///  - _telemetry[i][2] -> current[idx]
    ///  - _telemetry[i][3] -> rpm[idx]
    ///  - _telemetry[i][4] -> totalcurrent[idx]
    ///  - _telemetry[i][5] -> count[idx]
    uint16_t _telemetry[MOTOR_COUNT_MAX][6] = {0};
    uint16_t _motorpwm[MOTOR_COUNT_MAX] = {1000};
    uint8_t _telem_req_type; /// the requested telemetry type (OW_TLM_XXXXX)

/**
    initialize FETtecOneWire protocol
*/
    void Init();

/**
    deinitialize FETtecOneWire protocol
    TODO: remove it? it is not used anywhere
*/
    void DeInit();

/**
    generates used 8 bit CRC
    @param crc byte to be added to CRC
    @param crc_seed CRC where it gets added too
    @return 8 bit CRC
*/
    uint8_t Update_crc8(uint8_t crc, uint8_t crc_seed) const;

/**
    generates used 8 bit CRC for arrays
    @param Buf 8 bit byte array
    @param BufLen count of bytes that should be used for CRC calculation
    @return 8 bit CRC
*/
    uint8_t Get_crc8(uint8_t *Buf, uint16_t BufLen) const;

/**
    transmitts a FETtecOneWire frame to a ESC
    @param ESC_id id of the ESC
    @param Bytes  8 bit array of bytes. Where byte 1 contains the command, and all following bytes can be the payload
    @param Length length of the Bytes array
*/
    void Transmit(uint8_t ESC_id, uint8_t *Bytes, uint8_t Length);

/**
    reads the answer frame of a ESC
    @param Bytes 8 bit byte array, where the received answer gets stored in
    @param Length the expected answer length
    @param returnFullFrame can be OW_RETURN_RESPONSE or OW_RETURN_FULL_FRAME
    @return 1 if the expected answer frame was there, 0 if dont
*/
    uint8_t Receive(uint8_t *Bytes, uint8_t Length, uint8_t returnFullFrame);

/**
    Resets a pending pull request
*/
    void PullReset();

/**
    Pulls a complete request between for ESC
    @param ESC_id id of the ESC
    @param command 8bit array containing the command that should be send including the possible payload
    @param response 8bit array where the response will be stored in
    @param returnFullFrame can be OW_RETURN_RESPONSE or OW_RETURN_FULL_FRAME
    @return 1 if the request is completed, 0 if dont
*/
    uint8_t PullCommand(uint8_t ESC_id, uint8_t *command, uint8_t *response, uint8_t returnFullFrame);

/**
    scans for ESCs in bus. should be called until _ScanActive >= MOTOR_COUNT_MAX
    @return the current scanned ID
*/
    uint8_t ScanESCs();

/**
    starts all ESCs in bus and prepares them for receiving teh fast throttle command should be called until _SetupActive >= MOTOR_COUNT_MAX
    @return the current used ID
*/
    uint8_t InitESCs();

/**
    checks if the requested telemetry is available.
    @param Telemetry 16bit array where the read Telemetry will be stored in.
    @param return the telemetry request number or -1 if unavailable
*/
    int8_t CheckForTLM(uint16_t *Telemetry);

/**
    does almost all of the job.
    scans for ESCs if not already done.
    initializes the ESCs if not already done.
    sends fast throttle signals if init is complete.
    @param motorValues a 16bit array containing the throttle signals that should be sent to the motors. 0-2000 where 1001-2000 is positive rotation and 999-0 reversed rotation
    @param Telemetry 16bit array where the read telemetry will be stored in.
    @param motorCount the count of motors that should get values send
    @param tlmRequest the requested telemetry type (OW_TLM_XXXXX)
    @return the telemetry request if telemetry was available, -1 if dont
*/
    int8_t ESCsSetValues(uint16_t *motorValues, uint16_t *Telemetry, uint8_t motorCount, uint8_t tlmRequest);

    static constexpr uint8_t ALL_ID = 0x1F;
    typedef struct FETtecOneWireESC
    {
      uint8_t inBootLoader;
      uint8_t firmWareVersion;
      uint8_t firmWareSubVersion;
      uint8_t ESCtype;
      uint8_t serialNumber[12];
    } FETtecOneWireESC_t;

    uint8_t _activeESC_IDs[MOTOR_COUNT_MAX] = {0};
    FETtecOneWireESC_t _foundESCs[MOTOR_COUNT_MAX];
    uint8_t _FoundESCs;
    uint8_t _ScanActive;
    uint8_t _SetupActive;
    uint8_t _IgnoreOwnBytes;
    int8_t _minID = MOTOR_COUNT_MAX;
    int8_t _maxID;
    uint8_t _IDcount;
    uint8_t _FastThrottleByteCount;
    uint8_t _PullSuccess;
    uint8_t _PullBusy;
    uint8_t _TLM_request;
    uint8_t _lastCRC;
    uint8_t _firstInitDone;


    enum
    {
      OW_RETURN_RESPONSE,
      OW_RETURN_FULL_FRAME
    };

    enum
    {
      OW_OK,
      OW_BL_PAGE_CORRECT,   // BL only
      OW_NOT_OK,
      OW_BL_START_FW,       // BL only
      OW_BL_PAGES_TO_FLASH, // BL only
      OW_REQ_TYPE,
      OW_REQ_SN,
      OW_REQ_SW_VER
    };

    enum
    {
      OW_RESET_TO_BL = 10,
      OW_THROTTLE = 11,
      OW_REQ_TLM = 12,
      OW_BEEP = 13,

      OW_SET_FAST_COM_LENGTH = 26,

      OW_GET_ROTATION_DIRECTION = 28,
      OW_SET_ROTATION_DIRECTION = 29,

      OW_GET_USE_SIN_START = 30,
      OW_SET_USE_SIN_START = 31,

      OW_GET_3D_MODE = 32,
      OW_SET_3D_MODE = 33,

      OW_GET_ID = 34,
      OW_SET_ID = 35,

/*
    OW_GET_LINEAR_THRUST = 36,
    OW_SET_LINEAR_THRUST = 37,
*/

      OW_GET_EEVER = 38,

      OW_GET_PWM_MIN = 39,
      OW_SET_PWM_MIN = 40,

      OW_GET_PWM_MAX = 41,
      OW_SET_PWM_MAX = 42,

      OW_GET_ESC_BEEP = 43,
      OW_SET_ESC_BEEP = 44,

      OW_GET_CURRENT_CALIB = 45,
      OW_SET_CURRENT_CALIB = 46,

      OW_SET_LED_TMP_COLOR = 51,
      OW_GET_LED_COLOR = 52,
      OW_SET_LED_COLOR = 53

    };

    enum
    {
      OW_TLM_TEMP,
      OW_TLM_VOLT,
      OW_TLM_CURRENT,
      OW_TLM_ERPM,
      OW_TLM_CONSUMPTION,
      OW_TLM_DEBUG1,
      OW_TLM_DEBUG2,
      OW_TLM_DEBUG3
    };

    /// presistent scan state data (only used inside ScanESCs() function)
    struct scan_state
    {
      uint16_t delayLoops = 500;
      uint8_t scanID = 0;
      uint8_t scanState = 0;
      uint8_t scanTimeOut = 0;
    } _ss;

    /// presistent init state data (only used inside InitESCs() function)
    struct init_state
    {
      uint8_t delayLoops = 0;
      uint8_t activeID = 1;
      uint8_t State = 0;
      uint8_t TimeOut = 0;
      uint8_t wakeFromBL = 1;
      uint8_t setFastCommand[4] = {OW_SET_FAST_COM_LENGTH, 0, 0, 0};
    } _is;

    uint8_t _ResponseLength[54];
    uint8_t _RequestLength[54];

};
#endif // HAL_AP_FETTECONEWIRE_ENABLED

