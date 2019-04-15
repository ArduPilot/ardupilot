#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

// defines
#define LEDDARONE_DEFAULT_ADDRESS 0x01
#define LEDDARONE_MODOBUS_FUNCTION_CODE 0x04
#define LEDDARONE_MODOBUS_FUNCTION_REGISTER_ADDRESS 20
#define LEDDARONE_MODOBUS_FUNCTION_READ_NUMBER 10

#define LEDDARONE_SERIAL_PORT_MAX 250
#define LEDDARONE_READ_BUFFER_SIZE 25

#define LEDDARONE_DETECTIONS_MAX 3
#define LEDDARONE_DETECTION_DATA_NUMBER_INDEX 10
#define LEDDARONE_DETECTION_DATA_INDEX_OFFSET 11
#define LEDDARONE_DETECTION_DATA_OFFSET 4

// LeddarOne status
enum LeddarOne_Status {
    LEDDARONE_STATE_OK = 0,
    LEDDARONE_STATE_READING_BUFFER = 1,
    LEDDARONE_STATE_ERR_BAD_CRC = -1,
    LEDDARONE_STATE_ERR_NO_RESPONSES = -2,
    LEDDARONE_STATE_ERR_BAD_RESPONSE = -3,
    LEDDARONE_STATE_ERR_SHORT_RESPONSE = -4,
    LEDDARONE_STATE_ERR_SERIAL_PORT = -5,
    LEDDARONE_STATE_ERR_NUMBER_DETECTIONS = -6
};

// LeddarOne Modbus status
enum LeddarOne_ModbusStatus {
    LEDDARONE_MODBUS_STATE_INIT = 0,
    LEDDARONE_MODBUS_STATE_PRE_SEND_REQUEST,
    LEDDARONE_MODBUS_STATE_SENT_REQUEST,
    LEDDARONE_MODBUS_STATE_AVAILABLE
};

class AP_RangeFinder_LeddarOne : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_LeddarOne(RangeFinder::RangeFinder_State &_state,
                             AP_RangeFinder_Params &_params,
                             AP_SerialManager &serial_manager,
                             uint8_t serial_instance);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager, uint8_t serial_instance);

    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    // get a reading
    bool get_reading(uint16_t &reading_cm);

    // CRC16
    bool CRC16(uint8_t *aBuffer, uint8_t aLength, bool aCheck);

    // parse a response message from ModBus
    LeddarOne_Status parse_response(uint8_t &number_detections);

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_sending_request_ms;
    uint32_t last_available_ms;

    uint16_t detections[LEDDARONE_DETECTIONS_MAX];
    uint32_t sum_distance;

    LeddarOne_ModbusStatus modbus_status = LEDDARONE_MODBUS_STATE_INIT;
    uint8_t read_buffer[LEDDARONE_READ_BUFFER_SIZE];
    uint32_t read_len;

    // Modbus send request buffer
    // read input register (function code 0x04)
    const uint8_t send_request_buffer[8] = {
        LEDDARONE_DEFAULT_ADDRESS,
        LEDDARONE_MODOBUS_FUNCTION_CODE,
        0,
        LEDDARONE_MODOBUS_FUNCTION_REGISTER_ADDRESS,   // 20: Address of first register to read
        0,
        LEDDARONE_MODOBUS_FUNCTION_READ_NUMBER,        // 10: The number of consecutive registers to read
        0x30,   // CRC Lo
        0x09    // CRC Hi
    };
};
