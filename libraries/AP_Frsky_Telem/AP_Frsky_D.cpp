#include "AP_Frsky_D.h"

#if AP_FRSKY_D_TELEM_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>

/*
  send 1 byte and do byte stuffing
*/
void AP_Frsky_D::send_byte(uint8_t byte)
{
    if (byte == START_STOP_D) {
        _port->write(0x5D);
        _port->write(0x3E);
    } else if (byte == BYTESTUFF_D) {
        _port->write(0x5D);
        _port->write(0x3D);
    } else {
        _port->write(byte);
    }
}

/*
 * send one uint16 frame of FrSky data - for FrSky D protocol (D-receivers)
 */
void AP_Frsky_D::send_uint16(uint16_t id, uint16_t data)
{
    _port->write(START_STOP_D);    // send a 0x5E start byte
    uint8_t *bytes = (uint8_t*)&id;
    send_byte(bytes[0]);
    bytes = (uint8_t*)&data;
    send_byte(bytes[0]); // LSB
    send_byte(bytes[1]); // MSB
}

/*
 * send frame1 and frame2 telemetry data
 * one frame (frame1) is sent every 200ms with baro alt, nb sats, batt volts and amp, control_mode
 * a second frame (frame2) is sent every second (1000ms) with gps position data, and ahrs.get_yaw_deg() heading (instead of GPS heading)
 * for FrSky D protocol (D-receivers)
 */
void AP_Frsky_D::send(void)
{
    const AP_BattMonitor &_battery = AP::battery();
    uint32_t now = AP_HAL::millis();
    // send frame1 every 200ms
    if (now - _D.last_200ms_frame >= 200) {
        _D.last_200ms_frame = now;
        send_uint16(DATA_ID_TEMP2, (uint16_t)(AP::gps().num_sats() * 10 + AP::gps().status())); // send GPS status and number of satellites as num_sats*10 + status (to fit into a uint8_t)
        send_uint16(DATA_ID_TEMP1, gcs().custom_mode()); // send flight mode
        uint8_t percentage = 0;
        IGNORE_RETURN(_battery.capacity_remaining_pct(percentage));
        send_uint16(DATA_ID_FUEL, (uint16_t)roundf(percentage)); // send battery remaining
        send_uint16(DATA_ID_VFAS, (uint16_t)roundf(_battery.voltage() * 10.0f)); // send battery voltage
        float current;
        if (!_battery.current_amps(current)) {
            current = 0;
        }
        send_uint16(DATA_ID_CURRENT, (uint16_t)roundf(current * 10.0f)); // send current consumption
        calc_nav_alt();
        send_uint16(DATA_ID_BARO_ALT_BP, _SPort_data.alt_nav_meters); // send nav altitude integer part
        send_uint16(DATA_ID_BARO_ALT_AP, _SPort_data.alt_nav_cm); // send nav altitude decimal part
    }
    // send frame2 every second
    if (now - _D.last_1000ms_frame >= 1000) {
        _D.last_1000ms_frame = now;
        AP_AHRS &_ahrs = AP::ahrs();
        send_uint16(DATA_ID_GPS_COURS_BP, (uint16_t)_ahrs.get_yaw_deg()); // send heading in degree based on AHRS and not GPS
        calc_gps_position();
        if (AP::gps().status() >= 3) {
            send_uint16(DATA_ID_GPS_LAT_BP, _SPort_data.latdddmm); // send gps latitude degree and minute integer part
            send_uint16(DATA_ID_GPS_LAT_AP, _SPort_data.latmmmm); // send gps latitude minutes decimal part
            send_uint16(DATA_ID_GPS_LAT_NS, _SPort_data.lat_ns); // send gps North / South information
            send_uint16(DATA_ID_GPS_LONG_BP, _SPort_data.londddmm); // send gps longitude degree and minute integer part
            send_uint16(DATA_ID_GPS_LONG_AP, _SPort_data.lonmmmm); // send gps longitude minutes decimal part
            send_uint16(DATA_ID_GPS_LONG_EW, _SPort_data.lon_ew); // send gps East / West information
            send_uint16(DATA_ID_GPS_SPEED_BP, _SPort_data.speed_in_meter); // send gps speed integer part
            send_uint16(DATA_ID_GPS_SPEED_AP, _SPort_data.speed_in_centimeter); // send gps speed decimal part
            send_uint16(DATA_ID_GPS_ALT_BP, _SPort_data.alt_gps_meters); // send gps altitude integer part
            send_uint16(DATA_ID_GPS_ALT_AP, _SPort_data.alt_gps_cm); // send gps altitude decimal part
        }
    }
}

#endif  // AP_FRSKY_D_TELEM_ENABLED
