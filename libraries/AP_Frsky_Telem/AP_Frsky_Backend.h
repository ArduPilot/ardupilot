#pragma once

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_Frsky_Backend {
public:

    AP_Frsky_Backend(AP_HAL::UARTDriver *port) :
        _port(port) { }

    virtual ~AP_Frsky_Backend()  {}

    virtual bool init();
    virtual void send() = 0;

    // SPort is at 57600, D overrides this
    virtual uint32_t initial_baud() const { return 57600; }

    // get next telemetry data for external consumers of SPort data
    virtual bool get_telem_data(uint8_t &frame, uint16_t &appid, uint32_t &data) {
        return false;
    }

    virtual void queue_text_message(MAV_SEVERITY severity, const char *text) { }

protected:

    AP_HAL::UARTDriver *_port;  // UART used to send data to FrSky receiver

    virtual bool init_serial_port();

    // methods related to the nuts-and-bolts of sending data
    void send_byte(uint8_t value);
    void send_uint16(uint16_t id, uint16_t data);

    void calc_nav_alt(void);
    void calc_gps_position(void);

    float get_vspeed_ms(void);

    // methods to convert flight controller data to FrSky D or SPort format
    float format_gps(float dec);

    struct
    {
        int32_t vario_vspd;
        char lat_ns, lon_ew;
        uint16_t latdddmm;
        uint16_t latmmmm;
        uint16_t londddmm;
        uint16_t lonmmmm;
        uint16_t alt_gps_meters;
        uint16_t alt_gps_cm;
        uint16_t alt_nav_meters;
        uint16_t alt_nav_cm;
        int16_t speed_in_meter;
        uint16_t speed_in_centimeter;
        uint16_t yaw;
    } _SPort_data;

private:

    void loop(void);

};
