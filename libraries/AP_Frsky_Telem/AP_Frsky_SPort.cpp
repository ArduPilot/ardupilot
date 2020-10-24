#include "AP_Frsky_SPort.h"

#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>

/*
 * send telemetry data
 * for FrSky SPort protocol (X-receivers)
 */
void AP_Frsky_SPort::send(void)
{
    int16_t numc;
    numc = _port->available();

    // check if available is negative
    if (numc < 0) {
        return;
    }

    // this is the constant for hub data frame
    if (_port->txspace() < 19) {
        return;
    }

    if (numc == 0) {
        // no serial data to process do bg tasks
        if (_SPort.vario_refresh) {
            calc_nav_alt(); // nav altitude is not recalculated until all of it has been sent
            _SPort.vario_refresh = false;
        }
        if (_SPort.gps_refresh) {
            calc_gps_position(); // gps data is not recalculated until all of it has been sent
            _SPort.gps_refresh = false;
        }
        return;
    }

    for (int16_t i = 0; i < numc; i++) {
        int16_t readbyte = _port->read();
        if (_SPort.sport_status == false) {
            if  (readbyte == FRAME_HEAD) {
                _SPort.sport_status = true;
            }
        } else {
            const AP_BattMonitor &_battery = AP::battery();
            switch (readbyte) {
            case SENSOR_ID_VARIO:   // Sensor ID  0
                switch (_SPort.vario_call) {
                case 0:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_BARO_ALT_BP, _SPort_data.alt_nav_meters); // send altitude integer part
                    break;
                case 1:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_BARO_ALT_AP, _SPort_data.alt_nav_cm); // send altitude decimal part
                    break;
                case 2:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_VARIO, _SPort_data.vario_vspd); // send vspeed m/s
                    _SPort.vario_refresh = true;
                    break;
                }
                if (++_SPort.vario_call > 2) {
                    _SPort.vario_call = 0;
                }
                break;
            case SENSOR_ID_FAS: // Sensor ID  2
                switch (_SPort.fas_call) {
                case 0:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_FUEL, (uint16_t)roundf(_battery.capacity_remaining_pct())); // send battery remaining
                    break;
                case 1:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_VFAS, (uint16_t)roundf(_battery.voltage() * 10.0f)); // send battery voltage
                    break;
                case 2: {
                    float current;
                    if (!_battery.current_amps(current)) {
                        current = 0;
                    }
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_CURRENT, (uint16_t)roundf(current * 10.0f)); // send current consumption
                    break;
                }
                break;
                }
                if (++_SPort.fas_call > 2) {
                    _SPort.fas_call = 0;
                }
                break;
            case SENSOR_ID_GPS: // Sensor ID  3
                switch (_SPort.gps_call) {
                case 0:
                    send_sport_frame(SPORT_DATA_FRAME, GPS_LONG_LATI_FIRST_ID, calc_gps_latlng(&_passthrough.send_latitude)); // gps latitude or longitude
                    break;
                case 1:
                    send_sport_frame(SPORT_DATA_FRAME, GPS_LONG_LATI_FIRST_ID, calc_gps_latlng(&_passthrough.send_latitude)); // gps latitude or longitude
                    break;
                case 2:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_GPS_SPEED_BP, _SPort_data.speed_in_meter); // send gps speed integer part
                    break;
                case 3:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_GPS_SPEED_AP, _SPort_data.speed_in_centimeter); // send gps speed decimal part
                    break;
                case 4:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_GPS_ALT_BP, _SPort_data.alt_gps_meters); // send gps altitude integer part
                    break;
                case 5:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_GPS_ALT_AP, _SPort_data.alt_gps_cm); // send gps altitude decimals
                    break;
                case 6:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_GPS_COURS_BP, _SPort_data.yaw); // send heading in degree based on AHRS and not GPS
                    _SPort.gps_refresh = true;
                    break;
                }
                if (++_SPort.gps_call > 6) {
                    _SPort.gps_call = 0;
                }
                break;
            case SENSOR_ID_SP2UR: // Sensor ID  6
                switch (_SPort.various_call) {
                case 0 :
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_TEMP2, (uint16_t)(AP::gps().num_sats() * 10 + AP::gps().status())); // send GPS status and number of satellites as num_sats*10 + status (to fit into a uint8_t)
                    break;
                case 1:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_TEMP1, gcs().custom_mode()); // send flight mode
                    break;
                }
                if (++_SPort.various_call > 1) {
                    _SPort.various_call = 0;
                }
                break;
            }
            _SPort.sport_status = false;
        }
    }
}

/*
 * prepare gps latitude/longitude data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_SPort::calc_gps_latlng(bool *send_latitude)
{
    uint32_t latlng;
    const Location &loc = AP::gps().location(0); // use the first gps instance (same as in send_mavlink_gps_raw)

    // alternate between latitude and longitude
    if ((*send_latitude) == true) {
        if (loc.lat < 0) {
            latlng = ((labs(loc.lat)/100)*6) | 0x40000000;
        } else {
            latlng = ((labs(loc.lat)/100)*6);
        }
        (*send_latitude) = false;
    } else {
        if (loc.lng < 0) {
            latlng = ((labs(loc.lng)/100)*6) | 0xC0000000;
        } else {
            latlng = ((labs(loc.lng)/100)*6) | 0x80000000;
        }
        (*send_latitude) = true;
    }
    return latlng;
}

/*
 * send an 8 bytes SPort frame of FrSky data - for FrSky SPort protocol (X-receivers)
 */
void  AP_Frsky_SPort::send_sport_frame(uint8_t frame, uint16_t appid, uint32_t data)
{
    uint8_t buf[8];

    buf[0] = frame;
    buf[1] = appid & 0xFF;
    buf[2] = appid >> 8;
    memcpy(&buf[3], &data, 4);

    uint16_t sum = 0;
    for (uint8_t i=0; i<sizeof(buf)-1; i++) {
        sum += buf[i];
        sum += sum >> 8;
        sum &= 0xFF;
    }
    sum = 0xff - ((sum & 0xff) + (sum >> 8));
    buf[7] = (uint8_t)sum;

    // perform byte stuffing per SPort spec
    uint8_t len = 0;
    uint8_t buf2[sizeof(buf)*2+1];

    for (uint8_t i=0; i<sizeof(buf); i++) {
        uint8_t c = buf[i];
        if (c == FRAME_DLE || buf[i] == FRAME_HEAD) {
            buf2[len++] = FRAME_DLE;
            buf2[len++] = c ^ FRAME_XOR;
        } else {
            buf2[len++] = c;
        }
    }
#ifndef HAL_BOARD_SITL
    /*
      check that we haven't been too slow in responding to the new
      UART data. If we respond too late then we will overwrite the next
      polling frame.
      SPort poll-to-pool period is 11.65ms, a frame takes 1.38ms
      this leaves us with up to 10ms to respond but to play it safe we
      allow no more than 7500us
     */
    uint64_t tend = _port->receive_time_constraint_us(1);
    uint64_t now = AP_HAL::micros64();
    uint64_t tdelay = now - tend;
    if (tdelay > 7500) {
        // we've been too slow in responding
        return;
    }
#endif
    _port->write(buf2, len);
}
