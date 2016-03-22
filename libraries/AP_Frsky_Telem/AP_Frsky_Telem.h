// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_SerialManager/AP_SerialManager.h>


class AP_Frsky_Telem
{
public:
    //constructor
    AP_Frsky_Telem(AP_AHRS &ahrs, AP_BattMonitor &battery, AP_Baro* baro = NULL);

    // supported protocols
    enum FrSkyProtocol {
        FrSkyUnknown = 0,
        FrSkyDPORT = 2,
        FrSkySPORT = 3
    };

    // init - perform require initialisation including detecting which protocol to use
    void init(const AP_SerialManager& serial_manager);

    // send_frames - sends updates down telemetry link for both DPORT and SPORT protocols
    //  should be called by main program at 50hz to allow poll for serial bytes
    //  coming from the receiver for the SPort protocol
    void send_frames(uint8_t control_mode, bool armed);

private:

    // init_uart_for_sport - initialise uart for use by sport
    void init_uart_for_sport();

    // send_hub_frame - main transmission function when protocol is FrSkyDPORT
    void send_hub_frame();

    // sport_tick - main call to send updates to transmitter when protocol is FrSkySPORT
    //  called by scheduler at a high rate
    void sport_tick();

    // methods related to the nuts-and-bolts of sending data
    void calc_crc(uint8_t byte);
    void send_crc();
    void frsky_send_byte(uint8_t value);
    void frsky_send_hub_startstop();
    void frsky_send_sport_prim();
    void frsky_send_data(uint8_t id, int16_t data);
    void frsky_send_data_smart(uint16_t id, uint32_t data);

    // methods to convert flight controller data to frsky telemetry format
    void calc_baro_alt();
    float frsky_format_gps(float dec);
    void calc_gps_position();
    void calc_battery();
    void calc_gps_sats();

    // methods to send individual pieces of data down telemetry link
    void send_gps_sats(void);
    void send_smart_gps_sats(void);
    void send_mode(void);
    void send_smart_mode(void);
    void send_baro_alt_m(void);
    void send_baro_alt_cm(void);
    void send_smart_baro_alt(void);
    void send_smart_vario(void);
    void send_smart_attitude(void);
    void send_batt_remain(void);
    void send_batt_volts(void);
    void send_current(void);
    void send_smart_batt_volts(void);
    void send_smart_current(void);
    void send_prearm_error(void);
    void send_heading(void);
    void send_smart_gps_lat(void);
    void send_smart_gps_lon(void);
    void send_smart_gps_cog(void);
    void send_smart_gps_alt(void);
    void send_smart_gps_spd(void);
    void send_gps_lat_dd(void);
    void send_gps_lat_mm(void);
    void send_gps_lat_ns(void);
    void send_gps_lon_dd(void);
    void send_gps_lon_mm(void);
    void send_gps_lon_ew(void);
    void send_gps_speed_meter(void);
    void send_gps_speed_cm(void);
    void send_gps_alt_meter(void);
    void send_gps_alt_cm(void);

    AP_AHRS &_ahrs;                         // reference to attitude estimate
    AP_BattMonitor &_battery;               // reference to battery monitor object
    AP_Baro *_baro;                         // pointer to (optional) baro object
    AP_HAL::UARTDriver *_port;              // UART used to send data to receiver
    bool _initialised_uart;                 // true when we have detected the protocol and UART has been initialised
    enum FrSkyProtocol _protocol;           // protocol used - detected using SerialManager's SERIALX_PROTOCOL parameter

    uint16_t _crc;
    uint32_t _last_frame1_ms;
    uint32_t _last_frame2_ms;

    bool _battery_data_ready;
    uint16_t _batt_remaining;
    uint16_t _batt_volts;
    uint16_t _batt_amps;

    bool _sats_data_ready;
    uint16_t gps_sats;

    bool _gps_data_ready;
    bool _pos_gps_ok;
    uint16_t _course_in_degrees;
    char _lat_ns, _lon_ew;
    uint16_t _latdddmm;
    uint16_t _latmmmm;
    uint16_t _londddmm;
    uint16_t _lonmmmm;	
    uint16_t _alt_gps_meters;
    uint16_t _alt_gps_cm;
    int16_t _speed_in_meter;
    uint16_t _speed_in_centimeter;

    float _lon;
    float _lat;
    float _baro_alt;

    bool _baro_data_ready;
    int16_t _baro_alt_meters;
    uint16_t _baro_alt_cm;

    bool _mode_data_ready;
    uint8_t _mode; 
    bool _armed;

    uint8_t _fas_call;
    uint8_t _gps_call;
    uint8_t _vario_call;
    uint8_t _various_call;

    uint8_t _sport_status;

    const AP_GPS& _gps;
};
