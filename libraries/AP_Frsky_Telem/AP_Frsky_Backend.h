#pragma once

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

/* 
for FrSky D protocol (D-receivers)
*/
// FrSky sensor hub data IDs
#define DATA_ID_GPS_ALT_BP          0x01
#define DATA_ID_TEMP1               0x02
#define DATA_ID_FUEL                0x04
#define DATA_ID_TEMP2               0x05
#define DATA_ID_GPS_ALT_AP          0x09
#define DATA_ID_BARO_ALT_BP         0x10
#define DATA_ID_GPS_SPEED_BP        0x11
#define DATA_ID_GPS_LONG_BP         0x12
#define DATA_ID_GPS_LAT_BP          0x13
#define DATA_ID_GPS_COURS_BP        0x14
#define DATA_ID_GPS_SPEED_AP        0x19
#define DATA_ID_GPS_LONG_AP         0x1A
#define DATA_ID_GPS_LAT_AP          0x1B
#define DATA_ID_BARO_ALT_AP         0x21
#define DATA_ID_GPS_LONG_EW         0x22
#define DATA_ID_GPS_LAT_NS          0x23
#define DATA_ID_CURRENT             0x28
#define DATA_ID_VARIO               0x30
#define DATA_ID_VFAS                0x39

#define START_STOP_D                0x5E
#define BYTESTUFF_D                 0x5D

/* 
for FrSky SPort and SPort Passthrough (OpenTX) protocols (X-receivers)
*/
// FrSky Sensor IDs
#define SENSOR_ID_VARIO             0x00 // Sensor ID  0
#define SENSOR_ID_FAS               0x22 // Sensor ID  2
#define SENSOR_ID_GPS               0x83 // Sensor ID  3
#define SENSOR_ID_SP2UR             0xC6 // Sensor ID  6
#define SENSOR_ID_27                0x1B // Sensor ID 27

// FrSky data IDs
#define GPS_LONG_LATI_FIRST_ID      0x0800
#define DIY_FIRST_ID                0x5000

#define FRAME_HEAD                  0x7E
#define FRAME_DLE                   0x7D
#define FRAME_XOR                   0x20

#define SPORT_DATA_FRAME            0x10

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
