#include "SIM_config.h"

#if AP_SIM_GPS_UBLOX_ENABLED

#include "SIM_GPS_UBLOX.h"

#include <SITL/SITL.h>

using namespace SITL;

/*
  send a UBLOX GPS message
 */
void GPS_UBlox::send_ubx(uint8_t msgid, uint8_t *buf, uint16_t size)
{
    const uint8_t PREAMBLE1 = 0xb5;
    const uint8_t PREAMBLE2 = 0x62;
    const uint8_t CLASS_NAV = 0x1;
    uint8_t hdr[6], chk[2];
    hdr[0] = PREAMBLE1;
    hdr[1] = PREAMBLE2;
    hdr[2] = CLASS_NAV;
    hdr[3] = msgid;
    hdr[4] = size & 0xFF;
    hdr[5] = size >> 8;
    chk[0] = chk[1] = hdr[2];
    chk[1] += (chk[0] += hdr[3]);
    chk[1] += (chk[0] += hdr[4]);
    chk[1] += (chk[0] += hdr[5]);
    for (uint16_t i=0; i<size; i++) {
        chk[1] += (chk[0] += buf[i]);
    }
    write_to_autopilot((char*)hdr, sizeof(hdr));
    write_to_autopilot((char*)buf, size);
    write_to_autopilot((char*)chk, sizeof(chk));
}

void GPS_UBlox::update_relposned(ubx_nav_relposned &relposned, uint32_t tow_ms, float yaw_deg)
{
    Vector3f ant1_pos { NaNf, NaNf, NaNf };

    // find our partner:
    for (uint8_t i=0; i<ARRAY_SIZE(_sitl->gps); i++) {
        if (i == instance) {
            // this shouldn't matter as our heading type should never be base
            continue;
        }
        if (_sitl->gps[i].hdg_enabled != SITL::SIM::GPS_HEADING_BASE) {
            continue;
        }
        ant1_pos = _sitl->gps[i].pos_offset.get();
        break;
    }
    if (ant1_pos.is_nan()) {
        return;
    }

    const Vector3f ant2_pos = _sitl->gps[instance].pos_offset.get();
    Vector3f rel_antenna_pos = ant2_pos - ant1_pos;
    Matrix3f rot;
    // project attitude back using gyros to get antenna orientation at time of GPS sample
    Vector3f gyro(radians(_sitl->state.rollRate),
                  radians(_sitl->state.pitchRate),
                  radians(_sitl->state.yawRate));
    rot.from_euler(radians(_sitl->state.rollDeg), radians(_sitl->state.pitchDeg), radians(yaw_deg));
    const float lag = _sitl->gps[instance].delay_ms * 0.001;
    rot.rotate(gyro * (-lag));
    rel_antenna_pos = rot * rel_antenna_pos;
    relposned.version = 1;
    relposned.iTOW = tow_ms;
    relposned.relPosN = rel_antenna_pos.x * 100;
    relposned.relPosE = rel_antenna_pos.y * 100;
    relposned.relPosD = rel_antenna_pos.z * 100;
    relposned.relPosLength = rel_antenna_pos.length() * 100;
    relposned.relPosHeading = degrees(Vector2f(rel_antenna_pos.x, rel_antenna_pos.y).angle()) * 1.0e5;
    relposned.flags = gnssFixOK | diffSoln | carrSolnFixed | isMoving | relPosValid | relPosHeadingValid;
}

/*
  send a new set of GPS UBLOX packets
 */
void GPS_UBlox::publish(const GPS_Data *d)
{
    struct PACKED ubx_nav_posllh {
        uint32_t    time; // GPS msToW
        int32_t     longitude;
        int32_t     latitude;
        int32_t     altitude_ellipsoid;
        int32_t     altitude_msl;
        uint32_t    horizontal_accuracy;
        uint32_t    vertical_accuracy;
    } pos {};
    struct PACKED ubx_nav_status {
        uint32_t    time;               // GPS msToW
        uint8_t     fix_type;
        uint8_t     fix_status;
        uint8_t     differential_status;
        uint8_t     res;
        uint32_t    time_to_first_fix;
        uint32_t    uptime;             // milliseconds
    } status {};
    struct PACKED ubx_nav_velned {
        uint32_t    time;               // GPS msToW
        int32_t     ned_north;
        int32_t     ned_east;
        int32_t     ned_down;
        uint32_t    speed_3d;
        uint32_t    speed_2d;
        int32_t     heading_2d;
        uint32_t    speed_accuracy;
        uint32_t    heading_accuracy;
    } velned {};
    struct PACKED ubx_nav_solution {
        uint32_t time;
        int32_t time_nsec;
        int16_t week;
        uint8_t fix_type;
        uint8_t fix_status;
        int32_t ecef_x;
        int32_t ecef_y;
        int32_t ecef_z;
        uint32_t position_accuracy_3d;
        int32_t ecef_x_velocity;
        int32_t ecef_y_velocity;
        int32_t ecef_z_velocity;
        uint32_t speed_accuracy;
        uint16_t position_DOP;
        uint8_t res;
        uint8_t satellites;
        uint32_t res2;
    } sol {};
    struct PACKED ubx_nav_dop {
        uint32_t time;                                  // GPS msToW
        uint16_t gDOP;
        uint16_t pDOP;
        uint16_t tDOP;
        uint16_t vDOP;
        uint16_t hDOP;
        uint16_t nDOP;
        uint16_t eDOP;
    } dop {};
    struct PACKED ubx_nav_pvt {
        uint32_t itow;
        uint16_t year;
        uint8_t month, day, hour, min, sec;
        uint8_t valid;
        uint32_t t_acc;
        int32_t nano;
        uint8_t fix_type;
        uint8_t flags;
        uint8_t flags2;
        uint8_t num_sv;
        int32_t lon, lat;
        int32_t height, h_msl;
        uint32_t h_acc, v_acc;
        int32_t velN, velE, velD, gspeed;
        int32_t head_mot;
        uint32_t s_acc;
        uint32_t head_acc;
        uint16_t p_dop;
        uint8_t reserved1[6];
        uint32_t headVeh;
        uint8_t reserved2[4];
    } pvt {};
    const uint8_t SV_COUNT = 10;
    struct PACKED ubx_nav_svinfo {
        uint32_t itow;
        uint8_t numCh;
        uint8_t globalFlags;
        uint8_t reserved1[2];
        // repeated block
        struct PACKED svinfo_sv {
            uint8_t chn;
            uint8_t svid;
            uint8_t flags;
            uint8_t quality;
            uint8_t cno;
            int8_t elev;
            int16_t azim;
            int32_t prRes;
        } sv[SV_COUNT];
    } svinfo {};
    ubx_nav_relposned relposned {};
    const uint8_t MSG_POSLLH = 0x2;
    const uint8_t MSG_STATUS = 0x3;
    const uint8_t MSG_DOP = 0x4;
    const uint8_t MSG_VELNED = 0x12;
    const uint8_t MSG_SOL = 0x6;
    const uint8_t MSG_PVT = 0x7;
    const uint8_t MSG_SVINFO = 0x30;
    const uint8_t MSG_RELPOSNED = 0x3c;

    uint32_t _next_nav_sv_info_time = 0;

    const auto gps_tow = gps_time();

    pos.time = gps_tow.ms;
    pos.longitude = d->longitude * 1.0e7;
    pos.latitude  = d->latitude * 1.0e7;
    pos.altitude_ellipsoid = d->altitude * 1000.0f;
    pos.altitude_msl = d->altitude * 1000.0f;
    pos.horizontal_accuracy = d->horizontal_acc*1000;
    pos.vertical_accuracy = d->vertical_acc*1000;

    status.time = gps_tow.ms;
    status.fix_type = d->have_lock?3:0;
    status.fix_status = d->have_lock?1:0;
    status.differential_status = 0;
    status.res = 0;
    status.time_to_first_fix = 0;
    status.uptime = AP_HAL::millis();

    velned.time = gps_tow.ms;
    velned.ned_north = 100.0f * d->speedN;
    velned.ned_east  = 100.0f * d->speedE;
    velned.ned_down  = 100.0f * d->speedD;
    velned.speed_2d = norm(d->speedN, d->speedE) * 100;
    velned.speed_3d = norm(d->speedN, d->speedE, d->speedD) * 100;
    velned.heading_2d = degrees(atan2f(d->speedE, d->speedN)) * 100000.0f;
    if (velned.heading_2d < 0.0f) {
        velned.heading_2d += 360.0f * 100000.0f;
    }
    velned.speed_accuracy = d->speed_acc * 100;  // m/s -> cm/s
    velned.heading_accuracy = 4;

    memset(&sol, 0, sizeof(sol));
    sol.fix_type = d->have_lock?3:0;
    sol.fix_status = 221;
    sol.satellites = d->have_lock ? d->num_sats : 3;
    sol.time = gps_tow.ms;
    sol.week = gps_tow.week;

    dop.time = gps_tow.ms;
    dop.gDOP = 65535;
    dop.pDOP = 65535;
    dop.tDOP = 65535;
    dop.vDOP = 200;
    dop.hDOP = 121;
    dop.nDOP = 65535;
    dop.eDOP = 65535;

    pvt.itow = gps_tow.ms;
    pvt.year = 0;
    pvt.month = 0;
    pvt.day = 0;
    pvt.hour = 0;
    pvt.min = 0;
    pvt.sec = 0;
    pvt.valid = 0; // invalid utc date
    pvt.t_acc = 0;
    pvt.nano = 0;
    pvt.fix_type = d->have_lock? 0x3 : 0;
    pvt.flags = 0b10000011; // carrsoln=fixed, psm = na, diffsoln and fixok
    pvt.flags2 =0;
    pvt.num_sv = d->have_lock ? d->num_sats : 3;
    pvt.lon = d->longitude * 1.0e7;
    pvt.lat  = d->latitude * 1.0e7;
    pvt.height = d->altitude * 1000.0f;
    pvt.h_msl = d->altitude * 1000.0f;
    pvt.h_acc = d->horizontal_acc * 1000;
    pvt.v_acc = d->vertical_acc * 1000;
    pvt.velN = 1000.0f * d->speedN;
    pvt.velE = 1000.0f * d->speedE;
    pvt.velD = 1000.0f * d->speedD;
    pvt.gspeed = norm(d->speedN, d->speedE) * 1000;
    pvt.head_mot = degrees(atan2f(d->speedE, d->speedN)) * 1.0e5;
    pvt.s_acc = velned.speed_accuracy;
    pvt.head_acc = 38 * 1.0e5;
    pvt.p_dop = 65535;
    memset(pvt.reserved1, '\0', ARRAY_SIZE(pvt.reserved1));
    pvt.headVeh = 0;
    memset(pvt.reserved2, '\0', ARRAY_SIZE(pvt.reserved2));

    switch (_sitl->gps[instance].hdg_enabled) {
    case SITL::SIM::GPS_HEADING_NONE:
    case SITL::SIM::GPS_HEADING_BASE:
        break;
    case SITL::SIM::GPS_HEADING_THS:
    case SITL::SIM::GPS_HEADING_KSXT:
    case SITL::SIM::GPS_HEADING_HDT:
        update_relposned(relposned, gps_tow.ms, d->yaw_deg);
        break;
    }

    send_ubx(MSG_POSLLH, (uint8_t*)&pos, sizeof(pos));
    send_ubx(MSG_STATUS, (uint8_t*)&status, sizeof(status));
    send_ubx(MSG_VELNED, (uint8_t*)&velned, sizeof(velned));
    send_ubx(MSG_SOL,    (uint8_t*)&sol, sizeof(sol));
    send_ubx(MSG_DOP,    (uint8_t*)&dop, sizeof(dop));
    send_ubx(MSG_PVT,    (uint8_t*)&pvt, sizeof(pvt));
    if (_sitl->gps[instance].hdg_enabled > SITL::SIM::GPS_HEADING_NONE) {
        send_ubx(MSG_RELPOSNED,    (uint8_t*)&relposned, sizeof(relposned));
    }

    if (gps_tow.ms > _next_nav_sv_info_time) {
        svinfo.itow = gps_tow.ms;
        svinfo.numCh = 32;
        svinfo.globalFlags = 4; // u-blox 8/M8
        // fill in the SV's with some data even though firmware does not currently use it
        // note that this is not using num_sats as we aren't dynamically creating this to match
        for (uint8_t i = 0; i < SV_COUNT; i++) {
            svinfo.sv[i].chn = i;
            svinfo.sv[i].svid = i;
            svinfo.sv[i].flags = (i < d->num_sats) ? 0x7 : 0x6; // sv used, diff correction data, orbit information
            svinfo.sv[i].quality = 7; // code and carrier lock and time synchronized
            svinfo.sv[i].cno = MAX(20, 30 - i);
            svinfo.sv[i].elev = MAX(30, 90 - i);
            svinfo.sv[i].azim = i;
            // not bothering to fill in prRes
        }
        send_ubx(MSG_SVINFO, (uint8_t*)&svinfo, sizeof(svinfo));
        _next_nav_sv_info_time = gps_tow.ms + 10000; // 10 second delay
    }
}

#endif  // AP_SIM_GPS_UBLOX_ENABLED
