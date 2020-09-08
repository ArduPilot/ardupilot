/*
  output MSP protocol from AP_Periph for ArduPilot and INAV
  Thanks to input from Konstantin Sharlaimov
 */

#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_MSP

void AP_Periph_FW::msp_init(AP_HAL::UARTDriver *_uart)
{
    msp.port.uart = _uart;
    msp.port.msp_version = MSP::MSP_V2_NATIVE;
    _uart->begin(115200, 512, 512);
}


/*
  send a MSP packet
 */
void AP_Periph_FW::send_msp_packet(uint16_t cmd, void *p, uint16_t size)
{
    uint8_t out_buf[size+16] {};
    MSP::msp_packet_t pkt = {
        .buf = { .ptr = out_buf, .end = MSP_ARRAYEND(out_buf), },
        .cmd = (int16_t)cmd,
        .flags = 0,
        .result = 0,
    };

    sbuf_write_data(&pkt.buf, p, size);
    sbuf_switch_to_reader(&pkt.buf, &out_buf[0]);
    
    MSP::msp_serial_encode(&msp.port, &pkt, MSP::MSP_V2_NATIVE, true);
}

/*
  update MSP sensors
 */
void AP_Periph_FW::msp_sensor_update(void)
{
#ifdef HAL_PERIPH_ENABLE_GPS
    send_msp_GPS();
#endif
#ifdef HAL_PERIPH_ENABLE_BARO
    send_msp_baro();
#endif
#ifdef HAL_PERIPH_ENABLE_MAG
    send_msp_compass();
#endif
}


#ifdef HAL_PERIPH_ENABLE_GPS
/*
  send MSP GPS packet
 */
void AP_Periph_FW::send_msp_GPS(void)
{
    MSP::msp_gps_data_message_t p;

    if (gps.get_type(0) == AP_GPS::GPS_Type::GPS_TYPE_NONE) {
        return;
    }
    if (msp.last_gps_ms == gps.last_message_time_ms(0)) {
        return;
    }
    msp.last_gps_ms = gps.last_message_time_ms(0);

    const Location &loc = gps.location(0);
    const Vector3f &vel = gps.velocity(0);

    p.instance = 0;
    p.gps_week = gps.time_week(0);
    p.ms_tow = gps.get_itow(0);
    p.fix_type = uint8_t(gps.status(0));
    p.satellites_in_view = gps.num_sats(0);

    float hacc=0, vacc=0, sacc=0;
    gps.horizontal_accuracy(0, hacc);
    gps.vertical_accuracy(0, vacc);
    gps.speed_accuracy(0, sacc);

    p.horizontal_vel_accuracy = sacc*100;
    p.horizontal_pos_accuracy = hacc*100;
    p.vertical_pos_accuracy = vacc*100;
    p.hdop = gps.get_hdop(0);
    p.longitude = loc.lng;
    p.latitude = loc.lat;
    p.msl_altitude = loc.alt;
    p.ned_vel_north = vel.x*100;
    p.ned_vel_east = vel.y*100;
    p.ned_vel_down = vel.z*100;
    p.ground_course = wrap_360_cd(gps.ground_course(0)*100);
    float yaw_deg=0, acc;
    if (gps.gps_yaw_deg(0, yaw_deg, acc)) {
        p.true_yaw = wrap_360_cd(yaw_deg*100);
    } else {
        p.true_yaw = 65535; // unknown
    }
    uint64_t tepoch_us = gps.time_epoch_usec(0);
    time_t utc_sec = tepoch_us / (1000U * 1000U);
    struct tm* tm = gmtime(&utc_sec);

    p.year = tm->tm_year+1900;
    p.month = tm->tm_mon;
    p.day = tm->tm_mday;
    p.hour = tm->tm_hour;
    p.min = tm->tm_min;
    p.sec = tm->tm_sec;

    send_msp_packet(MSP2_SENSOR_GPS, &p, sizeof(p));
}
#endif // HAL_PERIPH_ENABLE_GPS


#ifdef HAL_PERIPH_ENABLE_BARO
/*
  send MSP baro packet
 */
void AP_Periph_FW::send_msp_baro(void)
{
    MSP::msp_baro_data_message_t p;

    if (msp.last_baro_ms == baro.get_last_update(0)) {
        return;
    }
    msp.last_baro_ms = baro.get_last_update(0);

    p.instance = 0;
    p.time_ms = msp.last_baro_ms;
    p.pressure_pa = baro.get_pressure();
    p.temp = baro.get_temperature() * 100;

    send_msp_packet(MSP2_SENSOR_BAROMETER, &p, sizeof(p));
}
#endif // HAL_PERIPH_ENABLE_BARO

#ifdef HAL_PERIPH_ENABLE_MAG
/*
  send MSP compass packet
 */
void AP_Periph_FW::send_msp_compass(void)
{
    MSP::msp_compass_data_message_t p;

    if (msp.last_mag_ms == compass.last_update_ms(0)) {
        return;
    }
    msp.last_mag_ms = compass.last_update_ms(0);

    const Vector3f &field = compass.get_field(0);
    p.instance = 0;
    p.time_ms = msp.last_mag_ms;
    p.magX = field.x;
    p.magY = field.y;
    p.magZ = field.z;

    send_msp_packet(MSP2_SENSOR_COMPASS, &p, sizeof(p));
}
#endif // HAL_PERIPH_ENABLE_MAG

#endif // HAL_PERIPH_ENABLE_MSP
