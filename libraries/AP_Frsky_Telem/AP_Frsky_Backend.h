#pragma once

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
//OW
#include "AP_Frsky_SPort_Protocol.h"
//OWEND

#ifndef HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
#define HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL 1
#endif

class AP_Frsky_Backend
{
public:

    AP_Frsky_Backend(AP_HAL::UARTDriver *port) :
        _port(port) { }

    virtual ~AP_Frsky_Backend()  {}

//OW
    AP_Frsky_SPort_Protocol* _protocol;
//OWEND

    virtual bool init();
    virtual void send() = 0;

    typedef union {
        struct PACKED {
            uint8_t sensor;
            uint8_t frame;
            uint16_t appid;
            uint32_t data;
        };
        uint8_t raw[8];
    } sport_packet_t;

    // SPort is at 57600, D overrides this
    virtual uint32_t initial_baud() const
    {
        return 57600;
    }

    // get next telemetry data for external consumers of SPort data
    virtual bool get_telem_data(sport_packet_t* packet_array, uint8_t &packet_count, const uint8_t max_size)
    {
        return false;
    }

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    virtual bool set_telem_data(const uint8_t frame, const uint16_t appid, const uint32_t data)
    {
        return false;
    }
#endif

    virtual void queue_text_message(MAV_SEVERITY severity, const char *text) { }

protected:

    AP_HAL::UARTDriver *_port;  // UART used to send data to FrSky receiver

    virtual bool init_serial_port();

    void calc_nav_alt(void);
    void calc_gps_position(void);

    // methods to convert flight controller data to FrSky D or SPort format
    float format_gps(float dec);

    struct {
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

    /*
      for FrSky D protocol (D-receivers)
    */
    // FrSky sensor hub data IDs
    static const uint8_t DATA_ID_GPS_ALT_BP        = 0x01;
    static const uint8_t DATA_ID_TEMP1             = 0x02;
    static const uint8_t DATA_ID_FUEL              = 0x04;
    static const uint8_t DATA_ID_TEMP2             = 0x05;
    static const uint8_t DATA_ID_GPS_ALT_AP        = 0x09;
    static const uint8_t DATA_ID_BARO_ALT_BP       = 0x10;
    static const uint8_t DATA_ID_GPS_SPEED_BP      = 0x11;
    static const uint8_t DATA_ID_GPS_LONG_BP       = 0x12;
    static const uint8_t DATA_ID_GPS_LAT_BP        = 0x13;
    static const uint8_t DATA_ID_GPS_COURS_BP      = 0x14;
    static const uint8_t DATA_ID_GPS_SPEED_AP      = 0x19;
    static const uint8_t DATA_ID_GPS_LONG_AP       = 0x1A;
    static const uint8_t DATA_ID_GPS_LAT_AP        = 0x1B;
    static const uint8_t DATA_ID_BARO_ALT_AP       = 0x21;
    static const uint8_t DATA_ID_GPS_LONG_EW       = 0x22;
    static const uint8_t DATA_ID_GPS_LAT_NS        = 0x23;
    static const uint8_t DATA_ID_CURRENT           = 0x28;
    static const uint8_t DATA_ID_VFAS              = 0x39;

    static const uint8_t START_STOP_D              = 0x5E;
    static const uint8_t BYTESTUFF_D               = 0x5D;

    /*
      for FrSky X protocol (X-receivers)
    */
    // FrSky 2 bytes DATA IDs;
    static const uint16_t ALT_ID                    = 0x010F;
    static const uint16_t VARIO_ID                  = 0x011F;
    static const uint16_t CURR_ID                   = 0x020F;
    static const uint16_t VFAS_ID                   = 0x021F;
    static const uint16_t TEMP1_ID                  = 0x040F;
    static const uint16_t TEMP2_ID                  = 0x041F;
    static const uint16_t RPM1_ID                   = 0x050E;
    static const uint16_t RPM2_ID                   = 0x050F;
    static const uint16_t FUEL_ID                   = 0x060F;
    static const uint16_t GPS_LONG_LATI_FIRST_ID    = 0x0800;
    static const uint16_t GPS_ALT_ID                = 0x082F;
    static const uint16_t GPS_SPEED_ID              = 0x083F;
    static const uint16_t GPS_COURS_ID              = 0x084F;
    static const uint16_t DIY_FIRST_ID              = 0x5000;

    static const uint8_t FRAME_HEAD                = 0x7E;
    static const uint8_t FRAME_DLE                 = 0x7D;
    static const uint8_t FRAME_XOR                 = 0x20;

    static const uint8_t SPORT_DATA_FRAME          = 0x10;

    // for FrSky SPort and SPort Passthrough (OpenTX) protocols (X-receivers)
    static const uint8_t SENSOR_ID_VARIO           = 0x00; // Sensor ID  0
    static const uint8_t SENSOR_ID_FAS             = 0x22; // Sensor ID  2
    static const uint8_t SENSOR_ID_GPS             = 0x83; // Sensor ID  3
    static const uint8_t SENSOR_ID_RPM             = 0xE4; // Sensor ID  4
    static const uint8_t SENSOR_ID_SP2UR           = 0xC6; // Sensor ID  6

    enum frsky_options_e : uint8_t {
        OPTION_AIRSPEED_AND_GROUNDSPEED = 1U<<0,
    };

private:

    void loop(void);

};
