#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AC_Lander
{
private:
    /* data */
    // automatic protocol decision variables
    
    enum class ProtocolState {
        UNKNOWN,    // the protocol used is not yet known
        LEGACY,     // legacy protocol, distances are sent as strings
        BINARY      // binary protocol, distances are sent using two bytes
    } protocol_state;
    uint8_t legacy_valid_count;
    uint8_t binary_valid_count;

    bool no_signal = false;


    char linebuf[10];           // legacy protocol buffer
    uint8_t linebuf_len;        // legacy protocol buffer length
    uint32_t last_init_ms;      // init time used to switch lw20 to serial mode
    uint8_t high_byte;          // binary protocol high byte
    bool high_byte_received;    // true if high byte has been received

    enum class Status {
        NotConnected = 0,
        NoData,
        OutOfRangeLow,
        OutOfRangeHigh,
        Good
    };

    uint32_t last_reading_ms;

    float max_distance_cm =  700;

public:
    // return distance to target in meters
    bool get_reading(float &reading_m);
    void init(uint8_t serial_instance);

    

protected:

    AP_HAL::UARTDriver *uart = nullptr;
    
    // baudrate used during object construction:
    uint32_t initial_baudrate(uint8_t serial_instance) const;

    // the value 0 is special to the UARTDriver - it's "use default"
    uint16_t rx_bufsize() const { return 0; }
    uint16_t tx_bufsize() const { return 0; }
    
    void init_serial(uint8_t serial_instance);
    bool is_lost_signal_distance(int16_t distance_cm, int16_t distance_cm_max);
};