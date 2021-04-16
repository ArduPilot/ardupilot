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

    uint32_t _last_send_us;
    static constexpr uint32_t DELAY_TIME_US = 700;
#if HAL_WITH_ESC_TELEM
    static constexpr uint8_t MOTOR_COUNT_MAX = ESC_TELEM_MAX_ESCS; /// OneWire supports up-to 25 ESCs, but Ardupilot only supports 12
#else
    static constexpr uint8_t MOTOR_COUNT_MAX = 12;                 /// OneWire supports up-to 25 ESCs, but Ardupilot only supports 12
#endif
    int8_t _telem_avail = -1;
    uint16_t _motorpwm[MOTOR_COUNT_MAX] = {1000};
    uint8_t _telem_req_type; /// the requested telemetry type (telem_type::XXXXX)

/**
    generates used 8 bit CRC for arrays
    @param Buf 8 bit byte array
    @param BufLen count of bytes that should be used for CRC calculation
    @return 8 bit CRC
*/
    uint8_t Get_crc8(uint8_t *Buf, uint16_t BufLen) const;

/**
    transmits a FETtec OneWire frame to an ESC
    @param ESC_id id of the ESC
    @param Bytes  8 bit array of bytes. Where byte 1 contains the command, and all following bytes can be the payload
    @param Length length of the Bytes array
*/
    void Transmit(uint8_t ESC_id, uint8_t *Bytes, uint8_t Length);

/**
    reads the FETtec OneWire answer frame of an ESC
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
    Pulls a complete request between flight controller and ESC
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
    @param tlmRequest the requested telemetry type (telem_type::XXXXX)
    @return the telemetry request if telemetry was available, -1 if dont
*/
    int8_t ESCsSetValues(uint16_t *motorValues, uint16_t *Telemetry, uint8_t motorCount, uint8_t tlmRequest);

    static constexpr uint8_t ALL_ID = 0x1F;
    typedef struct FETtecOneWireESC
    {
      uint8_t inBootLoader;
#if HAL_AP_FETTECONEWIRE_GET_STATIC_INFO
      uint8_t firmWareVersion;
      uint8_t firmWareSubVersion;
      uint8_t ESCtype;
      uint8_t serialNumber[12];
#endif
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

    /// presistent scan state data (only used inside ScanESCs() function)
    struct scan_state
    {
      uint16_t delayLoops;
      uint8_t scanID;
      uint8_t scanState;
      uint8_t scanTimeOut;
    } _ss;

    /// presistent init state data (only used inside InitESCs() function)
    struct init_state
    {
      uint8_t delayLoops;
      uint8_t activeID;
      uint8_t State;
      uint8_t TimeOut;
      uint8_t wakeFromBL;
      uint8_t setFastCommand[4] = {OW_SET_FAST_COM_LENGTH, 0, 0, 0};
    } _is;

    uint8_t _ResponseLength[OW_SET_FAST_COM_LENGTH+1]; // OW_SET_LED_TMP_COLOR is ignored here
    uint8_t _RequestLength[OW_SET_FAST_COM_LENGTH+1];  // OW_SET_LED_TMP_COLOR is ignored here

};
#endif // HAL_AP_FETTECONEWIRE_ENABLED

