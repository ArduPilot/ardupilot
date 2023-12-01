#include "SIM_GPS_SBP.h"

#if AP_SIM_GPS_SBP_ENABLED

#include <SITL/SITL.h>

using namespace SITL;

void GPS_SBP::publish(const GPS_Data *d)
{
    struct sbp_heartbeat_t {
        bool sys_error : 1;
        bool io_error : 1;
        bool nap_error : 1;
        uint8_t res : 5;
        uint8_t protocol_minor : 8;
        uint8_t protocol_major : 8;
        uint8_t res2 : 7;
        bool ext_antenna : 1;
    } hb; // 4 bytes

    struct PACKED sbp_gps_time_t {
        uint16_t wn;   //< GPS week number
        uint32_t tow;  //< GPS Time of Week rounded to the nearest ms
        int32_t ns;    //< Nanosecond remainder of rounded tow
        uint8_t flags; //< Status flags (reserved)
    } t;
    struct PACKED sbp_pos_llh_t {
        uint32_t tow;        //< GPS Time of Week
        double lat;          //< Latitude
        double lon;          //< Longitude
        double height;       //< Height
        uint16_t h_accuracy; //< Horizontal position accuracy estimate
        uint16_t v_accuracy; //< Vertical position accuracy estimate
        uint8_t n_sats;      //< Number of satellites used in solution
        uint8_t flags;       //< Status flags
    } pos;
    struct PACKED sbp_vel_ned_t {
        uint32_t tow;        //< GPS Time of Week
        int32_t n;           //< Velocity North coordinate
        int32_t e;           //< Velocity East coordinate
        int32_t d;           //< Velocity Down coordinate
        uint16_t h_accuracy; //< Horizontal velocity accuracy estimate
        uint16_t v_accuracy; //< Vertical velocity accuracy estimate
        uint8_t n_sats;      //< Number of satellites used in solution
        uint8_t flags;       //< Status flags (reserved)
    } velned;
    struct PACKED sbp_dops_t {
        uint32_t tow;  //< GPS Time of Week
        uint16_t gdop; //< Geometric Dilution of Precision
        uint16_t pdop; //< Position Dilution of Precision
        uint16_t tdop; //< Time Dilution of Precision
        uint16_t hdop; //< Horizontal Dilution of Precision
        uint16_t vdop; //< Vertical Dilution of Precision
        uint8_t flags; //< Status flags (reserved)
    } dops;

    static const uint16_t SBP_HEARTBEAT_MSGTYPE = 0xFFFF;
    static const uint16_t SBP_GPS_TIME_MSGTYPE = 0x0100;
    static const uint16_t SBP_DOPS_MSGTYPE = 0x0206;
    static const uint16_t SBP_POS_LLH_MSGTYPE = 0x0201;
    static const uint16_t SBP_VEL_NED_MSGTYPE = 0x0205;

    const auto gps_tow = gps_time();

    t.wn = gps_tow.week;
    t.tow = gps_tow.ms;
    t.ns = 0;
    t.flags = 0;
    sbp_send_message(SBP_GPS_TIME_MSGTYPE, 0x2222, sizeof(t), (uint8_t*)&t);

    if (!d->have_lock) {
        return;
    }

    pos.tow = gps_tow.ms;
    pos.lon = d->longitude;
    pos.lat= d->latitude;
    pos.height = d->altitude;
    pos.h_accuracy = _sitl->gps_accuracy[instance]*1000;
    pos.v_accuracy = _sitl->gps_accuracy[instance]*1000;
    pos.n_sats = d->have_lock ? _sitl->gps_numsats[instance] : 3;

    // Send single point position solution
    pos.flags = 0;
    sbp_send_message(SBP_POS_LLH_MSGTYPE, 0x2222, sizeof(pos), (uint8_t*)&pos);
    // Send "pseudo-absolute" RTK position solution
    pos.flags = 1;
    sbp_send_message(SBP_POS_LLH_MSGTYPE, 0x2222, sizeof(pos), (uint8_t*)&pos);

    velned.tow = gps_tow.ms;
    velned.n = 1e3 * d->speedN;
    velned.e = 1e3 * d->speedE;
    velned.d = 1e3 * d->speedD;
    velned.h_accuracy = 5e3;
    velned.v_accuracy = 5e3;
    velned.n_sats = d->have_lock ? _sitl->gps_numsats[instance] : 3;
    velned.flags = 0;
    sbp_send_message(SBP_VEL_NED_MSGTYPE, 0x2222, sizeof(velned), (uint8_t*)&velned);

    static uint32_t do_every_count = 0;
    if (do_every_count % 5 == 0) {

        dops.tow = gps_tow.ms;
        dops.gdop = 1;
        dops.pdop = 1;
        dops.tdop = 1;
        dops.hdop = 100;
        dops.vdop = 1;
        dops.flags = 1;
        sbp_send_message(SBP_DOPS_MSGTYPE, 0x2222, sizeof(dops),
                          (uint8_t*)&dops);

        hb.protocol_major = 0; //Sends protocol version 0
        sbp_send_message(SBP_HEARTBEAT_MSGTYPE, 0x2222, sizeof(hb),
                          (uint8_t*)&hb);

    }
    do_every_count++;
}

#endif  // AP_SIM_GPS_SBP_ENABLED
