// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

// default baud rate
#define LDDARONE_DEFAULT_BAUD_RATE 1115200

// default slabe address
#define LEDDARONE_DEFAULT_ADDRESS 0x01

// error codes
#define LEDDARONE_ERR_BAD_CRC -1
#define LEDDARONE_ERR_NO_RESPONSES -2
#define LEDDARONE_ERR_BAD_RESPONSE -3
#define LEDDARONE_ERR_SHORT_RESPONSE -4
#define LEDDARONE_ERR_SERIAL_PORT -5
#define LEDDARONE_ERR_NUMBER_DETECTIONS -6

struct detection {
	uint8_t segment;
	uint16_t distance;
	uint16_t amplitude;

	// default constructor
	detection(): segment(0), distance(0xFFFF), amplitude(0){};
};


class AP_RangeFinder_LeddarOne : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_LeddarOne(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state,
                                   AP_SerialManager &serial_manager);

    // static detection function
    static bool detect(RangeFinder &ranger, uint8_t instance, AP_SerialManager &serial_manager);

    // update state
    void update(void);

private:
    // get a reading
    bool get_reading(uint16_t &reading_cm);

    // CRC16
    bool CRC16(uint8_t *aBuffer, uint8_t aLength, bool aCheck);

    // send a request message to execute ModBus function
    int8_t send_request(void);

    // parse a response message from ModBus
    int8_t parse_response(void);

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_reading_ms = 0;
    //char linebuf[10];
    //uint8_t linebuf_len = 0;

    detection detections[3];

};
