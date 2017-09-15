#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_uLanding : public AP_RangeFinder_Backend
{

public:
    // constructor
	AP_RangeFinder_uLanding(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state,
                                   AP_SerialManager &serial_manager);

    // static detection function
    static bool detect(RangeFinder &ranger, uint8_t instance, AP_SerialManager &serial_manager);

    // update state
    void update(void);

private:
    // detect uLanding Firmware Version
    bool detect_version(void);

    // get a reading
    bool get_reading(uint16_t &reading_cm);

    AP_HAL::UARTDriver *uart;
    uint8_t  _linebuf[6];
    uint8_t  _linebuf_len;
    uint32_t _last_reading_ms;
    bool     _version_known;
    uint8_t  _header;
    uint8_t  _version;
};
