#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

#define LEDDARONE_DETECTIONS_MAX 3

// default slave address
#define LEDDARONE_DEFAULT_ADDRESS 0x01
#define LEDDARONE_DATA_INDEX_OFFSET 11

// LeddarOne status
enum LeddarOne_Status {
    LEDDARONE_OK = 0,
	LEDDARONE_READING_BUFFER = 1,
    LEDDARONE_ERR_BAD_CRC = -1,
    LEDDARONE_ERR_NO_RESPONSES = -2,
    LEDDARONE_ERR_BAD_RESPONSE = -3,
    LEDDARONE_ERR_SHORT_RESPONSE = -4,
    LEDDARONE_ERR_SERIAL_PORT = -5,
    LEDDARONE_ERR_NUMBER_DETECTIONS = -6
};

// LeddarOne Modbus status
enum LeddarOne_ModbusStatus {
	LEDDARONE_MODBUS_PRE_SEND_REQUEST = 0,
	LEDDARONE_MODBUS_SENT_REQUEST,
	LEDDARONE_MODBUS_AVAILABLE
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
    LeddarOne_Status send_request(void);

    // parse a response message from ModBus
    LeddarOne_Status parse_response(uint8_t &number_detections);

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_reading_ms;
    uint32_t last_sending_request_ms;

    uint16_t detections[LEDDARONE_DETECTIONS_MAX];
    uint32_t sum_distance;

    LeddarOne_ModbusStatus modbus_status = LEDDARONE_MODBUS_PRE_SEND_REQUEST;
    uint8_t read_buffer[25];
    uint32_t read_len;
};
