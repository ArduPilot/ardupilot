#include "SIM_config.h"

#if AP_SIM_GPS_UBLOX_ENABLED

#include "SIM_GPS_UBLOX.h"

#include <SITL/SITL.h>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

using namespace SITL;

static const uint8_t CLASS_NAV = 0x01;
static const uint8_t CLASS_RXM = 0x02;
static const uint8_t CLASS_ACK = 0x05;
static const uint8_t CLASS_CFG = 0x06;
static const uint8_t CLASS_MON = 0x0A;
static const uint8_t CLASS_TIM = 0x0D;

static const uint8_t MSG_POSLLH = 0x2;
static const uint8_t MSG_STATUS = 0x3;
static const uint8_t MSG_DOP = 0x4;
static const uint8_t MSG_SOL = 0x6;
static const uint8_t MSG_PVT = 0x7;
static const uint8_t MSG_VELNED = 0x12;
static const uint8_t MSG_TIMEGPS = 0x20;
static const uint8_t MSG_SVINFO = 0x30;
static const uint8_t MSG_RELPOSNED = 0x3c;

/*
  send a UBLOX GPS NAV message
 */
void GPS_UBlox::send_ubx(uint8_t msgid, uint8_t *buf, uint16_t size)
{
    send_ubx(CLASS_NAV, msgid, buf, size);
}

/*
  send a UBLOX GPS message
 */
void GPS_UBlox::send_ubx(uint8_t msgclass, uint8_t msgid, const uint8_t *buf, uint16_t size)
{
    const uint8_t PREAMBLE1 = 0xb5;
    const uint8_t PREAMBLE2 = 0x62;
    uint8_t hdr[6], chk[2];
    hdr[0] = PREAMBLE1;
    hdr[1] = PREAMBLE2;
    hdr[2] = msgclass;
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
    struct PACKED ubx_nav_timegps {
        uint32_t itow;
        int32_t ftow;
        uint16_t week;
        int8_t leapS;
        uint8_t valid; //  leapsvalid | weekvalid | tow valid;
        uint32_t tAcc;
    } timegps {};
    ubx_nav_relposned relposned {};

    // Note: This variable is actually a bug but removing it causes some CI failures.
    // Keep this here for now pending further investigation.
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
    if (d->have_lock) {
        sol.satellites = d->num_sats;
        switch(d->fix_type) {
            // https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf?utm_content=UBX-13003221
            // See page 377, 32.17.17 UBX-NAV-PVT
            case 0: // no gps - don't publish data.
                return;
            case 1: // no lock
                pvt.flags = 0b00000000;
                sol.fix_type = 0;
                break;
            case 2: // 2d
                sol.fix_type = 2;
                pvt.flags = 0b00000001;
                break;
            case 3: // 3d
                sol.fix_type = 3;
                pvt.flags = 0b00000001;
                break;
            case 4: // dgps
                sol.fix_type = 3;
                pvt.flags = 0b00000011;
                break;
            case 5: // rtk float
                sol.fix_type = 3;
                pvt.flags = 0b01000011;
                break;
            case 6: // rtk fixed
                sol.fix_type = 3;
                pvt.flags = 0b10000011;
                break;
        }
    } else {
        sol.fix_type = 0;
        pvt.flags = 0b00000000;
        sol.satellites = 3;
    }
    sol.fix_status = 221;
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
    pvt.fix_type = sol.fix_type;
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

    timegps.itow = gps_tow.ms;
    timegps.ftow = 0;                 // we don't simulate fractional ns
    timegps.week = gps_tow.week;
    timegps.leapS = 0;
    timegps.valid = d->have_lock ? 0x03 : 0x00; // tow valid|week valid
    timegps.tAcc  = 0;

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

    if (config_simulated()) {
        // only send what the receiver has been configured to send
        nav_count++;
        if (should_send(CLASS_NAV, MSG_POSLLH)) {
            send_ubx(MSG_POSLLH, (uint8_t*)&pos, sizeof(pos));
        }
        if (should_send(CLASS_NAV, MSG_STATUS)) {
            send_ubx(MSG_STATUS, (uint8_t*)&status, sizeof(status));
        }
        if (should_send(CLASS_NAV, MSG_VELNED)) {
            send_ubx(MSG_VELNED, (uint8_t*)&velned, sizeof(velned));
        }
        if (should_send(CLASS_NAV, MSG_SOL)) {
            send_ubx(MSG_SOL, (uint8_t*)&sol, sizeof(sol));
        }
        if (should_send(CLASS_NAV, MSG_TIMEGPS)) {
            send_ubx(MSG_TIMEGPS, (uint8_t*)&timegps, sizeof(timegps));
        }
        if (should_send(CLASS_NAV, MSG_DOP)) {
            send_ubx(MSG_DOP, (uint8_t*)&dop, sizeof(dop));
        }
        if (should_send(CLASS_NAV, MSG_PVT)) {
            send_ubx(MSG_PVT, (uint8_t*)&pvt, sizeof(pvt));
        }
        if (_sitl->gps[instance].hdg_enabled > SITL::SIM::GPS_HEADING_NONE &&
            should_send(CLASS_NAV, MSG_RELPOSNED)) {
            send_ubx(MSG_RELPOSNED, (uint8_t*)&relposned, sizeof(relposned));
        }
        if (should_send(CLASS_NAV, MSG_SVINFO)) {
            send_svinfo(gps_tow.ms, d->num_sats);
        }
        return;
    }

    send_ubx(MSG_POSLLH, (uint8_t*)&pos, sizeof(pos));
    send_ubx(MSG_STATUS, (uint8_t*)&status, sizeof(status));
    send_ubx(MSG_VELNED, (uint8_t*)&velned, sizeof(velned));
    const bool is_f9p = (_sitl->gps[instance].options & static_cast<int32_t>(SITL::SIM::GPSOptions::UBX_IS_F9P)) != 0;

    if (is_f9p) {
        const uint32_t now_ms = AP_HAL::millis();
        if ((int32_t)(now_ms - _next_timegps_send_ms) >= 0) {
            _next_timegps_send_ms = now_ms + 1000;
            send_ubx(MSG_TIMEGPS, (uint8_t*)&timegps, sizeof(timegps));
        }
    } else {
        // F9P and later use TIMEGPS to set week number, older u-blox use SOL
        send_ubx(MSG_SOL,    (uint8_t*)&sol, sizeof(sol));
    }
    send_ubx(MSG_DOP,    (uint8_t*)&dop, sizeof(dop));
    send_ubx(MSG_PVT,    (uint8_t*)&pvt, sizeof(pvt));
    if (_sitl->gps[instance].hdg_enabled > SITL::SIM::GPS_HEADING_NONE) {
        send_ubx(MSG_RELPOSNED,    (uint8_t*)&relposned, sizeof(relposned));
    }

    if (gps_tow.ms > _next_nav_sv_info_time) {
        send_svinfo(gps_tow.ms, d->num_sats);
        _next_nav_sv_info_time = gps_tow.ms + 10000; // 10 second delay
    }
}

void GPS_UBlox::send_svinfo(uint32_t tow_ms, uint8_t num_sats)
{
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

    svinfo.itow = tow_ms;
    svinfo.numCh = 32;
    // the low 3 bits give the hardware generation
    svinfo.globalFlags = model() == Model::UBLOX6 ? 2 : 4; // u-blox 6 or u-blox 8/M8
    // fill in the SV's with some data even though firmware does not currently use it
    // note that this is not using num_sats as we aren't dynamically creating this to match
    for (uint8_t i = 0; i < SV_COUNT; i++) {
        svinfo.sv[i].chn = i;
        svinfo.sv[i].svid = i;
        svinfo.sv[i].flags = (i < num_sats) ? 0x7 : 0x6; // sv used, diff correction data, orbit information
        svinfo.sv[i].quality = 7; // code and carrier lock and time synchronized
        svinfo.sv[i].cno = MAX(20, 30 - i);
        svinfo.sv[i].elev = MAX(30, 90 - i);
        svinfo.sv[i].azim = i;
        // not bothering to fill in prRes
    }
    send_ubx(MSG_SVINFO, (uint8_t*)&svinfo, sizeof(svinfo));
}

/*
  configuration simulation
 */

const GPS_UBlox::MsgID GPS_UBlox::tracked_msgs[] {
    { CLASS_NAV, MSG_POSLLH },
    { CLASS_NAV, MSG_STATUS },
    { CLASS_NAV, MSG_DOP },
    { CLASS_NAV, MSG_SOL },
    { CLASS_NAV, MSG_PVT },
    { CLASS_NAV, MSG_VELNED },
    { CLASS_NAV, MSG_TIMEGPS },
    { CLASS_NAV, MSG_SVINFO },
    { CLASS_NAV, MSG_RELPOSNED },
    { CLASS_RXM, 0x10 },  // RAW
    { CLASS_RXM, 0x15 },  // RAWX
    { CLASS_MON, 0x09 },  // HW
    { CLASS_MON, 0x0b },  // HW2
    { CLASS_TIM, 0x03 },  // TM2
};

// the UART1 index into port arrays; the simulated receiver is
// always attached to the autopilot by UART1
static const uint8_t UBX_PORT = 1;

static const uint32_t SAVED_CONFIG_MAGIC = 0x55425853;  // "SXBU"

// CFG-VALGET/VALSET keys, with the M10's default values
const GPS_UBlox::ConfigKeyInfo GPS_UBlox::config_keys[] {
    { 0x1031001F, 1 },  // CFG-SIGNAL-GPS_ENA
    { 0x10310020, 1 },  // CFG-SIGNAL-SBAS_ENA
    { 0x10310021, 1 },  // CFG-SIGNAL-GAL_ENA
    { 0x10310022, 1 },  // CFG-SIGNAL-BDS_ENA
    { 0x1031000D, 1 },  // CFG-SIGNAL-BDS_B1_ENA
    { 0x1031000F, 0 },  // CFG-SIGNAL-BDS_B1C_ENA
    { 0x10310024, 1 },  // CFG-SIGNAL-QZSS_ENA
    { 0x10310012, 1 },  // CFG-SIGNAL-QZSS_L1CA_ENA
    { 0x10310014, 1 },  // CFG-SIGNAL-QZSS_L1S_ENA
    { 0x10310025, 0 },  // CFG-SIGNAL-GLO_ENA
};
static const uint32_t CFG_NAVSPG_DYNMODEL = 0x20110021;

// ZED-X20P keys and defaults, from the u-blox X20 HPG 2.11 interface
// description (UBXDOC-304424225-21617)
const GPS_UBlox::X20KeyInfo GPS_UBlox::x20_keys[] {
    { 0x1031001F, 1 },     // CFG-SIGNAL-GPS_ENA
    { 0x10310001, 1 },     // CFG-SIGNAL-GPS_L1CA_ENA
    { 0x10310003, 1 },     // CFG-SIGNAL-GPS_L2C_ENA
    { 0x10310004, 1 },     // CFG-SIGNAL-GPS_L5_ENA
    { 0x10310020, 1 },     // CFG-SIGNAL-SBAS_ENA
    { 0x10310005, 1 },     // CFG-SIGNAL-SBAS_L1CA_ENA
    { 0x10310021, 1 },     // CFG-SIGNAL-GAL_ENA
    { 0x10310007, 1 },     // CFG-SIGNAL-GAL_E1_ENA
    { 0x10310009, 1 },     // CFG-SIGNAL-GAL_E5A_ENA
    { 0x1031000A, 0 },     // CFG-SIGNAL-GAL_E5B_ENA
    { 0x1031000B, 1 },     // CFG-SIGNAL-GAL_E6_ENA
    { 0x10310022, 1 },     // CFG-SIGNAL-BDS_ENA
    { 0x1031000D, 1 },     // CFG-SIGNAL-BDS_B1_ENA
    { 0x1031000F, 1 },     // CFG-SIGNAL-BDS_B1C_ENA
    { 0x1031000E, 0 },     // CFG-SIGNAL-BDS_B2_ENA
    { 0x10310028, 1 },     // CFG-SIGNAL-BDS_B2A_ENA
    { 0x10310010, 1 },     // CFG-SIGNAL-BDS_B3_ENA
    { 0x10310024, 0 },     // CFG-SIGNAL-QZSS_ENA
    { 0x10310012, 1 },     // CFG-SIGNAL-QZSS_L1CA_ENA
    { 0x10310039, 1 },     // CFG-SIGNAL-QZSS_L1CB_ENA
    { 0x10310014, 1 },     // CFG-SIGNAL-QZSS_L1S_ENA
    { 0x10310015, 1 },     // CFG-SIGNAL-QZSS_L2C_ENA
    { 0x10310017, 1 },     // CFG-SIGNAL-QZSS_L5_ENA
    { 0x10310025, 1 },     // CFG-SIGNAL-GLO_ENA
    { 0x10310018, 1 },     // CFG-SIGNAL-GLO_L1_ENA
    { 0x1031001A, 1 },     // CFG-SIGNAL-GLO_L2_ENA
    { 0x10310026, 0 },     // CFG-SIGNAL-NAVIC_ENA
    { 0x1031001D, 1 },     // CFG-SIGNAL-NAVIC_L5_ENA
    { 0x1031002A, 0 },     // CFG-SIGNAL-LBAND_ENA
    { 0x1031002F, 1 },     // CFG-SIGNAL-LBAND_PMP_ENA
    { 0x2031003A, 2 },     // CFG-SIGNAL-PLAN
    { 0x30210001, 1000 },  // CFG-RATE-MEAS
    { 0x30210002, 1 },     // CFG-RATE-NAV
    { 0x20110021, 0 },     // CFG-NAVSPG-DYNMODEL
    { 0x201100A4, 10 },    // CFG-NAVSPG-INFIL_MINELEV
    { 0x10530005, 1 },     // CFG-UART2-ENABLED
    { 0x20910007, 0 },     // CFG-MSGOUT-UBX_NAV_PVT_UART1
    { 0x20910008, 0 },     // CFG-MSGOUT-UBX_NAV_PVT_UART2
    { 0x20910048, 0 },     // CFG-MSGOUT-UBX_NAV_TIMEGPS_UART1
    { 0x20910049, 0 },     // CFG-MSGOUT-UBX_NAV_TIMEGPS_UART2
    { 0x20910039, 0 },     // CFG-MSGOUT-UBX_NAV_DOP_UART1
    { 0x2091003A, 0 },     // CFG-MSGOUT-UBX_NAV_DOP_UART2
    { 0x2091035A, 0 },     // CFG-MSGOUT-UBX_MON_RF_UART1
    { 0x2091035B, 0 },     // CFG-MSGOUT-UBX_MON_RF_UART2
    { 0x209102A5, 0 },     // CFG-MSGOUT-UBX_RXM_RAWX_UART1
    { 0x209102A6, 0 },     // CFG-MSGOUT-UBX_RXM_RAWX_UART2
};

// the X20's constellation enable keys and the keys of their signals.
// A constellation is in use when it and at least one of its signals
// are enabled (ZED-X20P integration manual, GNSS signal configuration)
static const struct {
    uint8_t gnss_id;
    uint32_t enable_key;
    uint32_t signal_keys[5];
} x20_gnss[] {
    { 0, 0x1031001F, { 0x10310001, 0x10310003, 0x10310004 } },  // GPS
    { 1, 0x10310020, { 0x10310005 } },  // SBAS
    { 2, 0x10310021, { 0x10310007, 0x10310009, 0x1031000A, 0x1031000B } },  // Galileo
    { 3, 0x10310022, { 0x1031000D, 0x1031000F, 0x1031000E, 0x10310028, 0x10310010 } },  // BeiDou
    { 5, 0x10310024, { 0x10310012, 0x10310039, 0x10310014, 0x10310015, 0x10310017 } },  // QZSS
    { 6, 0x10310025, { 0x10310018, 0x1031001A } },  // GLONASS
    { 7, 0x10310026, { 0x1031001D } },  // NavIC
};

GPS_UBlox::Model GPS_UBlox::model() const
{
    const uint8_t value = _sitl->gps[instance].ubx_model.get();
    if (value > uint8_t(Model::X20P_HPG211)) {
        // unknown model; behave as the simple model
        return Model::SIMPLE;
    }
    return Model(value);
}

// receivers with flash keep their saved configuration without a
// backup supply
bool GPS_UBlox::has_flash() const
{
    switch (model()) {
    case Model::M8N_FW201:
    case Model::M8N_SPG301:
    case Model::M9N_SPG404:
    case Model::X20P_HPG211:
        return true;
    case Model::SIMPLE:
    case Model::UBLOX6:
    case Model::M8_ROM301:
    case Model::M10_SPG510:
        break;
    }
    return false;
}

// only the M10 is simulated as supporting CFG-VALGET/VALSET; the
// driver doesn't use them on the older receivers
bool GPS_UBlox::supports_valset() const
{
    return model() == Model::M10_SPG510;
}

// return true if the receiver supports outputting a message; this
// matches what the real receivers NAK
bool GPS_UBlox::supports(uint8_t msg_class, uint8_t msg_id) const
{
    if (msg_class == CLASS_RXM) {
        // none of the simulated receivers output raw measurements
        return false;
    }
    if (msg_class == CLASS_MON && msg_id == 0x04) {
        // MON-VER is poll-only
        return false;
    }
    switch (model()) {
    case Model::UBLOX6:
        return !(msg_class == CLASS_NAV && msg_id == MSG_PVT);
    case Model::M9N_SPG404:
    case Model::M10_SPG510:
    case Model::X20P_HPG211:
        return !(msg_class == CLASS_NAV && (msg_id == MSG_SOL || msg_id == MSG_SVINFO));
    case Model::SIMPLE:
    case Model::M8N_FW201:
    case Model::M8_ROM301:
    case Model::M8N_SPG301:
        break;
    }
    return true;
}

int8_t GPS_UBlox::tracked_index(uint8_t msg_class, uint8_t msg_id) const
{
    for (uint8_t i=0; i<ARRAY_SIZE(tracked_msgs); i++) {
        if (tracked_msgs[i].msg_class == msg_class && tracked_msgs[i].msg_id == msg_id) {
            return i;
        }
    }
    return -1;
}

// factory default configuration: no UBX output on UART1
void GPS_UBlox::factory_config(Config &cfg) const
{
    memset(&cfg, 0, sizeof(cfg));
    cfg.measure_rate_ms = 1000;
    cfg.nav_rate = 1;
    cfg.timeref = 1;  // GPS time

    struct PACKED {
        uint16_t mask;
        uint8_t dynModel;
        uint8_t fixMode;
        int32_t fixedAlt;
        uint32_t fixedAltVar;
        int8_t minElev;
        uint8_t drLimit;
        uint16_t pDop;
        uint16_t tDop;
        uint16_t pAcc;
        uint16_t tAcc;
        uint8_t staticHoldThresh;
        uint8_t dgnssTimeout;
        uint8_t reserved[12];
    } nav5 {};
    static_assert(sizeof(nav5) == sizeof(cfg.nav5), "NAV5 size");
    nav5.dynModel = 0;  // portable
    nav5.fixMode = 3;   // auto 2D/3D
    nav5.fixedAltVar = 10000;
    nav5.minElev = 5;
    nav5.pDop = 250;
    nav5.tDop = 250;
    nav5.pAcc = 100;
    nav5.tAcc = 350;
    nav5.dgnssTimeout = 60;
    memcpy(cfg.nav5, &nav5, sizeof(nav5));

    // SBAS enabled, ranging/corrections/integrity, up to 3 channels
    const uint8_t sbas[8] { 1, 3, 3, 0, 0, 0, 0, 0 };
    memcpy(cfg.sbas, sbas, sizeof(sbas));

    struct PACKED {
        uint8_t msgVer;
        uint8_t numTrkChHw;
        uint8_t numTrkChUse;
        uint8_t numConfigBlocks;
        struct PACKED {
            uint8_t gnssId;
            uint8_t resTrkCh;
            uint8_t maxTrkCh;
            uint8_t reserved1;
            uint32_t flags;
        } block[7];
    } gnss {
        0, 32, 32, 7, {
            { 0, 8, 16, 0, 0x01010001 },  // GPS
            { 1, 1, 3, 0, 0x01010001 },   // SBAS
            { 2, 4, 8, 0, 0x01010000 },   // Galileo
            { 3, 8, 16, 0, 0x01010000 },  // BeiDou
            { 4, 0, 8, 0, 0x01010000 },   // IMES
            { 5, 0, 3, 0, 0x05050001 },   // QZSS
            { 6, 8, 14, 0, 0x01010001 },  // GLONASS
        }
    };
    static_assert(sizeof(gnss) == sizeof(cfg.gnss), "GNSS size");
    memcpy(cfg.gnss, &gnss, sizeof(gnss));
    cfg.gnss_len = sizeof(gnss);

    for (uint8_t i=0; i<ARRAY_SIZE(config_keys); i++) {
        cfg.key_value[i] = config_keys[i].default_value;
    }
    for (uint8_t i=0; i<ARRAY_SIZE(x20_keys); i++) {
        cfg.x20_value[i] = x20_keys[i].default_value;
    }
}

void GPS_UBlox::config_filename(char *buf, uint8_t buflen) const
{
    snprintf(buf, buflen, "sim_ublox_gps%u.bin", unsigned(instance+1));
}

/*
  on power-up the receiver loads its saved configuration into RAM.
  Receivers without flash need a backup supply to keep it
 */
void GPS_UBlox::load_saved_config()
{
    factory_config(saved);

    const bool no_backup = (_sitl->gps[instance].options & uint32_t(SITL::SIM::GPSOptions::UBX_NO_BACKUP)) != 0;
    if (has_flash() || !no_backup) {
        char filename[32];
        config_filename(filename, sizeof(filename));
        const int fd = ::open(filename, O_RDONLY);
        if (fd != -1) {
            SavedConfig file;
            if (::read(fd, &file, sizeof(file)) == sizeof(file) &&
                file.magic == SAVED_CONFIG_MAGIC &&
                file.model == uint8_t(model())) {
                saved = file.config;
            }
            ::close(fd);
        }
    }

    ram = saved;
    loaded_model = model();
    config_loaded = true;
}

void GPS_UBlox::store_saved_config() const
{
    char filename[32];
    config_filename(filename, sizeof(filename));
    const int fd = ::open(filename, O_WRONLY|O_CREAT|O_TRUNC, 0644);
    if (fd == -1) {
        AP_HAL::panic("open(%s): %s", filename, strerror(errno));
    }
    const SavedConfig file {
        SAVED_CONFIG_MAGIC,
        uint8_t(model()),
        saved
    };
    if (::write(fd, &file, sizeof(file)) != sizeof(file)) {
        AP_HAL::panic("write(%s): %s", filename, strerror(errno));
    }
    ::close(fd);
}

// return true if a message is due to be output this navigation solution
bool GPS_UBlox::should_send(uint8_t msg_class, uint8_t msg_id)
{
    if (!config_loaded || loaded_model != model()) {
        load_saved_config();
    }
    const int8_t idx = tracked_index(msg_class, msg_id);
    if (idx < 0 || !supports(msg_class, msg_id)) {
        return false;
    }
    uint8_t rate = ram.rate[idx];
    if (model() == Model::X20P_HPG211) {
        // the X20 output rates are CFG-MSGOUT keys
        uint32_t key = 0;
        if (msg_class == CLASS_NAV && msg_id == MSG_PVT) {
            key = 0x20910007;  // CFG-MSGOUT-UBX_NAV_PVT_UART1
        } else if (msg_class == CLASS_NAV && msg_id == MSG_TIMEGPS) {
            key = 0x20910048;  // CFG-MSGOUT-UBX_NAV_TIMEGPS_UART1
        } else if (msg_class == CLASS_NAV && msg_id == MSG_DOP) {
            key = 0x20910039;  // CFG-MSGOUT-UBX_NAV_DOP_UART1
        }
        rate = key == 0 ? 0 : x20_value(ram, key);
    }
    return rate != 0 && nav_count % rate == 0;
}

void GPS_UBlox::send_ack(uint8_t msg_class, uint8_t msg_id, bool ack)
{
    const uint8_t payload[2] { msg_class, msg_id };
    send_ubx(CLASS_ACK, ack ? 0x01 : 0x00, payload, sizeof(payload));
}

void GPS_UBlox::send_mon_ver()
{
    struct {
        const char *sw;
        const char *hw;
        const char *ext[7];
    } ver {};
    switch (model()) {
    case Model::UBLOX6:
        // no extension strings at all
        ver = { "7.03 (45969)", "00040007", {} };
        break;
    case Model::M8N_FW201:
        // note the protocol version and module are not "KEY=value"
        ver = { "2.01 (75350)", "00080000", {
                "2.01 (75331)",
                "PROTVER 15.00",
                "FIS 0xEF4015 (79189)",
                "MOD NEO-M8N-0",
                "GPS;SBAS;GLO;BDS;QZSS" } };
        break;
    case Model::M8_ROM301:
        ver = { "ROM CORE 3.01 (107888)", "00080000", {
                "FWVER=SPG 3.01",
                "PROTVER=18.00",
                "GPS;GLO;GAL;BDS",
                "SBAS;IMES;QZSS" } };
        break;
    case Model::M8N_SPG301:
        ver = { "EXT CORE 3.01 (107900)", "00080000", {
                "ROM BASE 3.01 (107888)",
                "FWVER=SPG 3.01",
                "PROTVER=18.00",
                "MOD=NEO-M8N-0",
                "FIS=0xEF4015 (100111)",
                "GPS;GLO;GAL;BDS",
                "SBAS;IMES;QZSS" } };
        break;
    case Model::M9N_SPG404:
        ver = { "EXT CORE 4.04 (7f89f7)", "00190000", {
                "ROM BASE 0x118B2060",
                "FWVER=SPG 4.04",
                "PROTVER=32.01",
                "MOD=NEO-M9N",
                "GPS;GLO;GAL;BDS",
                "SBAS;QZSS" } };
        break;
    case Model::M10_SPG510:
        // no MOD= extension
        ver = { "ROM SPG 5.10 (7b202e)", "000A0000", {
                "FWVER=SPG 5.10",
                "PROTVER=34.10",
                "GPS;GLO;GAL;BDS",
                "SBAS;QZSS" } };
        break;
    case Model::X20P_HPG211:
        // the version strings are not taken from a real receiver;
        // the driver uses the protocol version and module name
        ver = { "EXT CORE 2.11", "00200000", {
                "FWVER=HPG 2.11",
                "PROTVER=50.11",
                "MOD=ZED-X20P",
                "GPS;GLO;GAL;BDS",
                "SBAS;QZSS;NAVIC" } };
        break;
    case Model::SIMPLE:
        return;
    }

    uint8_t payload[40 + 30*ARRAY_SIZE(ver.ext)] {};
    strncpy((char*)&payload[0], ver.sw, 30);
    strncpy((char*)&payload[30], ver.hw, 10);
    uint16_t len = 40;
    for (const char *ext : ver.ext) {
        if (ext == nullptr) {
            break;
        }
        strncpy((char*)&payload[len], ext, 30);
        len += 30;
    }
    send_ubx(CLASS_MON, 0x04, payload, len);
}

void GPS_UBlox::send_cfg_prt()
{
    // built bytewise; clang's analyser can't follow byte reads of
    // multi-byte fields in send_ubx()
    uint8_t prt[20] {};
    const uint32_t mode = 0x8c0;        // 8N1
    const uint32_t baudrate = 230400;
    const uint16_t in_proto = 0x07;     // UBX, NMEA, RTCM2
    const uint16_t out_proto = 0x03;    // UBX, NMEA
    prt[0] = UBX_PORT;
    memcpy(&prt[4], &mode, 4);
    memcpy(&prt[8], &baudrate, 4);
    memcpy(&prt[12], &in_proto, 2);
    memcpy(&prt[14], &out_proto, 2);
    send_ubx(CLASS_CFG, 0x00, prt, sizeof(prt));
}

void GPS_UBlox::handle_cfg(uint8_t msg_id, const uint8_t *payload, uint16_t len)
{
    if (model() == Model::X20P_HPG211) {
        // the X20 has none of the older CFG messages
        switch (msg_id) {
        case 0x04:  // RST; the RAM configuration is rebuilt from the saved one
            ram = saved;
            return;
        case 0x09:  // CFG
            break;
        case 0x8a:  // VALSET
            x20_handle_valset(payload, len);
            return;
        case 0x8b:  // VALGET
            x20_handle_valget(payload, len);
            return;
        case 0x8c:  // VALDEL
            x20_handle_valdel(payload, len);
            return;
        default:
            send_ack(CLASS_CFG, msg_id, false);
            return;
        }
    }

    switch (msg_id) {
    case 0x00:  // PRT
        if (len <= 1) {
            send_cfg_prt();
            send_ack(CLASS_CFG, msg_id, true);
        } else {
            send_ack(CLASS_CFG, msg_id, true);
        }
        return;

    case 0x01: {  // MSG
        if (len < 2) {
            break;
        }
        const uint8_t msg_class = payload[0];
        const uint8_t id = payload[1];
        if (!supports(msg_class, id)) {
            break;
        }
        const int8_t idx = tracked_index(msg_class, id);
        if (len == 2) {
            // poll; reply with the rates on all six ports
            uint8_t reply[8] { msg_class, id };
            if (idx >= 0) {
                reply[2+UBX_PORT] = ram.rate[idx];
            }
            send_ubx(CLASS_CFG, msg_id, reply, sizeof(reply));
            send_ack(CLASS_CFG, msg_id, true);
            return;
        }
        if (idx >= 0) {
            if (len == 3) {
                // rate on the current port
                ram.rate[idx] = payload[2];
            } else if (len == 8) {
                ram.rate[idx] = payload[2+UBX_PORT];
            }
        }
        send_ack(CLASS_CFG, msg_id, true);
        return;
    }

    case 0x08:  // RATE
        if (len == 0) {
            uint8_t reply[6];
            memcpy(&reply[0], &ram.measure_rate_ms, 2);
            memcpy(&reply[2], &ram.nav_rate, 2);
            memcpy(&reply[4], &ram.timeref, 2);
            send_ubx(CLASS_CFG, msg_id, reply, sizeof(reply));
            send_ack(CLASS_CFG, msg_id, true);
            return;
        }
        if (len == 6) {
            memcpy(&ram.measure_rate_ms, &payload[0], 2);
            memcpy(&ram.nav_rate, &payload[2], 2);
            memcpy(&ram.timeref, &payload[4], 2);
            send_ack(CLASS_CFG, msg_id, true);
            return;
        }
        break;

    case 0x09: {  // CFG: clear, save and load configuration
        if (len != 12 && len != 13) {
            break;
        }
        uint32_t clear_mask, save_mask, load_mask;
        memcpy(&clear_mask, &payload[0], 4);
        memcpy(&save_mask, &payload[4], 4);
        memcpy(&load_mask, &payload[8], 4);
        if (clear_mask != 0) {
            factory_config(saved);
            store_saved_config();
        }
        if (save_mask != 0) {
            saved = ram;
            saved.x20_set = (1ULL << ARRAY_SIZE(x20_keys)) - 1;
            store_saved_config();
        }
        if (load_mask != 0) {
            ram = saved;
        }
        send_ack(CLASS_CFG, msg_id, true);
        return;
    }

    case 0x16:  // SBAS
        if (len == 0) {
            send_ubx(CLASS_CFG, msg_id, ram.sbas, sizeof(ram.sbas));
            send_ack(CLASS_CFG, msg_id, true);
            return;
        }
        if (len == sizeof(ram.sbas)) {
            memcpy(ram.sbas, payload, len);
            send_ack(CLASS_CFG, msg_id, true);
            return;
        }
        break;

    case 0x24:  // NAV5
        if (len == 0) {
            uint8_t reply[sizeof(ram.nav5)];
            memcpy(reply, ram.nav5, sizeof(reply));
            // the receiver reports all settings as valid
            reply[0] = reply[1] = 0xff;
            send_ubx(CLASS_CFG, msg_id, reply, sizeof(reply));
            send_ack(CLASS_CFG, msg_id, true);
            return;
        }
        if (len == sizeof(ram.nav5)) {
            const uint16_t mask = payload[0] | (payload[1]<<8);
            if (mask & 0x01) {
                ram.nav5[2] = payload[2];   // dynModel
            }
            if (mask & 0x02) {
                ram.nav5[12] = payload[12]; // minElev
            }
            if (mask & 0x04) {
                ram.nav5[3] = payload[3];   // fixMode
            }
            send_ack(CLASS_CFG, msg_id, true);
            return;
        }
        break;

    case 0x3e:  // GNSS
        if (model() == Model::UBLOX6) {
            // CFG-GNSS arrived with u-blox 7
            break;
        }
        if (len == 0) {
            send_ubx(CLASS_CFG, msg_id, ram.gnss, ram.gnss_len);
            send_ack(CLASS_CFG, msg_id, true);
            return;
        }
        if (len >= 4 && len <= sizeof(ram.gnss)) {
            memcpy(ram.gnss, payload, len);
            ram.gnss_len = len;
            send_ack(CLASS_CFG, msg_id, true);
            return;
        }
        break;

    case 0x8a:  // VALSET
        if (supports_valset()) {
            handle_valset(payload, len);
            return;
        }
        break;

    case 0x8b:  // VALGET
        if (supports_valset()) {
            handle_valget(payload, len);
            return;
        }
        break;

    default:
        break;
    }

    send_ack(CLASS_CFG, msg_id, false);
}

// return a pointer to where the value of a single-byte key is held, or nullptr
uint8_t *GPS_UBlox::config_value(Config &cfg, uint32_t key)
{
    if (key == CFG_NAVSPG_DYNMODEL) {
        return &cfg.nav5[2];
    }
    for (uint8_t i=0; i<ARRAY_SIZE(config_keys); i++) {
        if (config_keys[i].key == key) {
            return &cfg.key_value[i];
        }
    }
    return nullptr;
}

/*
  reply with the values of the requested keys from the requested
  layer: 0=RAM, 1=BBR, 2=flash, 7=default.  Any unknown key NAKs the
  request, as the M10 does for a single unknown key
 */
void GPS_UBlox::handle_valget(const uint8_t *payload, uint16_t len)
{
    if (len < 8 || (len - 4) % 4 != 0) {
        send_ack(CLASS_CFG, 0x8b, false);
        return;
    }
    const uint8_t layer = payload[1];
    Config defaults;
    factory_config(defaults);
    Config *cfg;
    switch (layer) {
    case 0:
        cfg = &ram;
        break;
    case 1:
    case 2:
        cfg = &saved;
        break;
    case 7:
        cfg = &defaults;
        break;
    default:
        send_ack(CLASS_CFG, 0x8b, false);
        return;
    }
    uint8_t reply[4 + 5*16];
    reply[0] = 1;  // version
    reply[1] = layer;
    reply[2] = payload[2];
    reply[3] = payload[3];
    uint16_t reply_len = 4;
    for (uint16_t ofs=4; ofs<len; ofs+=4) {
        uint32_t key;
        memcpy(&key, &payload[ofs], 4);
        const uint8_t *value = config_value(*cfg, key);
        if (value == nullptr || size_t(reply_len) + 5 > sizeof(reply)) {
            send_ack(CLASS_CFG, 0x8b, false);
            return;
        }
        memcpy(&reply[reply_len], &key, 4);
        reply[reply_len+4] = *value;
        reply_len += 5;
    }
    send_ubx(CLASS_CFG, 0x8b, reply, reply_len);
    send_ack(CLASS_CFG, 0x8b, true);
}

/*
  set key values in the RAM layer (bit 0) and/or the saved
  configuration (BBR bit 1, flash bit 2)
 */
void GPS_UBlox::handle_valset(const uint8_t *payload, uint16_t len)
{
    if (len < 4 || (len - 4) % 5 != 0) {
        send_ack(CLASS_CFG, 0x8a, false);
        return;
    }
    // check every key before changing anything
    for (uint16_t ofs=4; ofs<len; ofs+=5) {
        uint32_t key;
        memcpy(&key, &payload[ofs], 4);
        if (config_value(ram, key) == nullptr) {
            send_ack(CLASS_CFG, 0x8a, false);
            return;
        }
    }
    const uint8_t layers = payload[1];
    for (uint16_t ofs=4; ofs<len; ofs+=5) {
        uint32_t key;
        memcpy(&key, &payload[ofs], 4);
        if (layers & 0x01) {
            *config_value(ram, key) = payload[ofs+4];
        }
        if (layers & 0x06) {
            *config_value(saved, key) = payload[ofs+4];
        }
    }
    if (layers & 0x06) {
        store_saved_config();
    }
    send_ack(CLASS_CFG, 0x8a, true);
}

int8_t GPS_UBlox::x20_key_index(uint32_t key) const
{
    for (uint8_t i=0; i<ARRAY_SIZE(x20_keys); i++) {
        if (x20_keys[i].key == key) {
            return i;
        }
    }
    return -1;
}

uint32_t GPS_UBlox::x20_value(const Config &cfg, uint32_t key) const
{
    const int8_t idx = x20_key_index(key);
    return idx < 0 ? 0 : cfg.x20_value[idx];
}

// size of a key's value from bits 30..28 of the key
static uint8_t ubx_key_value_size(uint32_t key)
{
    switch ((key >> 28) & 0x07) {
    case 1:
    case 2:
        return 1;
    case 3:
        return 2;
    case 4:
        return 4;
    case 5:
        return 8;
    }
    return 0;
}

/*
  the X20 rejects a signal configuration unless a major constellation
  is in use, QZSS is used only with GPS, and SBAS only with GPS,
  Galileo or BeiDou (ZED-X20P integration manual and X20 HPG 2.11
  interface description, CFG-SIGNAL)
 */
bool GPS_UBlox::x20_signal_config_valid(const Config &cfg) const
{
    uint8_t in_use = 0;
    for (const auto &gnss : x20_gnss) {
        if (x20_value(cfg, gnss.enable_key) == 0) {
            continue;
        }
        for (const uint32_t signal_key : gnss.signal_keys) {
            if (signal_key != 0 && x20_value(cfg, signal_key) != 0) {
                in_use |= 1U << gnss.gnss_id;
                break;
            }
        }
    }
    const uint8_t GPS = 1U<<0, SBAS = 1U<<1, GAL = 1U<<2, BDS = 1U<<3, QZSS = 1U<<5, GLO = 1U<<6;
    if ((in_use & (GPS | GAL | BDS | GLO)) == 0) {
        return false;
    }
    if ((in_use & QZSS) && !(in_use & GPS)) {
        return false;
    }
    if ((in_use & SBAS) && !(in_use & (GPS | GAL | BDS))) {
        return false;
    }
    return true;
}

/*
  reply with the requested keys' values from a layer: 0=RAM, 1=BBR,
  2=flash, 7=default.  Keys may be wildcards for a whole group (item
  0xffff) or all groups (group 0xfff); the saved layers only hold the
  keys that have been set in them.  An unknown key NAKs the request
 */
void GPS_UBlox::x20_handle_valget(const uint8_t *payload, uint16_t len)
{
    if (len < 8 || (len - 4) % 4 != 0 || (len - 4) / 4 > 64) {
        send_ack(CLASS_CFG, 0x8b, false);
        return;
    }
    const uint8_t layer = payload[1];
    const uint16_t position = payload[2] | (payload[3] << 8);
    Config defaults;
    factory_config(defaults);
    const Config *cfg;
    bool saved_layer = false;
    switch (layer) {
    case 0:
        cfg = &ram;
        break;
    case 1:
    case 2:
        cfg = &saved;
        saved_layer = true;
        break;
    case 7:
        cfg = &defaults;
        break;
    default:
        send_ack(CLASS_CFG, 0x8b, false);
        return;
    }

    uint8_t reply[4 + 64*(4+4)];
    reply[0] = 1;  // version
    reply[1] = layer;
    reply[2] = payload[2];
    reply[3] = payload[3];
    uint16_t reply_len = 4;
    uint16_t skipped = 0;
    uint8_t count = 0;
    for (uint16_t ofs=4; ofs<len; ofs+=4) {
        uint32_t key;
        memcpy(&key, &payload[ofs], 4);
        const uint16_t group = (key >> 16) & 0xfff;
        const bool all_groups = group == 0xfff;
        const bool whole_group = (key & 0xffff) == 0xffff;
        if (!all_groups && !whole_group && x20_key_index(key) < 0) {
            send_ack(CLASS_CFG, 0x8b, false);
            return;
        }
        for (uint8_t i=0; i<ARRAY_SIZE(x20_keys); i++) {
            const uint32_t k = x20_keys[i].key;
            if (!all_groups &&
                !(whole_group && ((k >> 16) & 0xfff) == group) &&
                k != key) {
                continue;
            }
            if (saved_layer && (cfg->x20_set & (1ULL << i)) == 0) {
                continue;
            }
            if (skipped < position) {
                skipped++;
                continue;
            }
            if (count == 64) {
                break;
            }
            const uint8_t size = ubx_key_value_size(k);
            memcpy(&reply[reply_len], &k, 4);
            memcpy(&reply[reply_len+4], &cfg->x20_value[i], size);
            reply_len += 4 + size;
            count++;
        }
    }
    send_ubx(CLASS_CFG, 0x8b, reply, reply_len);
    send_ack(CLASS_CFG, 0x8b, true);
}

/*
  set keys in RAM (layer bit 0) and/or the saved configuration (BBR
  bit 1, flash bit 2).  As on the receiver, nothing is applied if any
  key is unknown, and the signal configuration is only checked when
  RAM is being set
 */
void GPS_UBlox::x20_handle_valset(const uint8_t *payload, uint16_t len)
{
    const uint8_t layers = len >= 4 ? payload[1] & 0x07 : 0;
    // transactions are not simulated
    const bool transaction = len >= 4 && payload[0] == 1 && payload[2] != 0;
    if (layers == 0 || transaction) {
        send_ack(CLASS_CFG, 0x8a, false);
        return;
    }
    Config new_ram = ram;
    Config new_saved = saved;
    uint16_t ofs = 4;
    while (ofs < len) {
        uint32_t key;
        if (ofs + 4 > len) {
            send_ack(CLASS_CFG, 0x8a, false);
            return;
        }
        memcpy(&key, &payload[ofs], 4);
        const int8_t idx = x20_key_index(key);
        const uint8_t size = ubx_key_value_size(key);
        if (idx < 0 || ofs + 4 + size > len) {
            send_ack(CLASS_CFG, 0x8a, false);
            return;
        }
        uint32_t value = 0;
        memcpy(&value, &payload[ofs+4], size);
        if (layers & 0x01) {
            new_ram.x20_value[idx] = value;
        }
        if (layers & 0x06) {
            new_saved.x20_value[idx] = value;
            new_saved.x20_set |= 1ULL << idx;
        }
        ofs += 4 + size;
    }
    if ((layers & 0x01) && !x20_signal_config_valid(new_ram)) {
        send_ack(CLASS_CFG, 0x8a, false);
        return;
    }
    ram = new_ram;
    if (layers & 0x06) {
        saved = new_saved;
        store_saved_config();
    }
    send_ack(CLASS_CFG, 0x8a, true);
}

/*
  delete keys from the saved configuration; keys may be wildcards as
  for VALGET
 */
void GPS_UBlox::x20_handle_valdel(const uint8_t *payload, uint16_t len)
{
    if (len < 8 || (len - 4) % 4 != 0 || (payload[1] & 0x06) == 0) {
        send_ack(CLASS_CFG, 0x8c, false);
        return;
    }
    Config defaults;
    factory_config(defaults);
    for (uint16_t ofs=4; ofs<len; ofs+=4) {
        uint32_t key;
        memcpy(&key, &payload[ofs], 4);
        const uint16_t group = (key >> 16) & 0xfff;
        for (uint8_t i=0; i<ARRAY_SIZE(x20_keys); i++) {
            const uint32_t k = x20_keys[i].key;
            if (group == 0xfff ||
                ((key & 0xffff) == 0xffff && ((k >> 16) & 0xfff) == group) ||
                k == key) {
                saved.x20_set &= ~(1ULL << i);
                saved.x20_value[i] = defaults.x20_value[i];
            }
        }
    }
    store_saved_config();
    send_ack(CLASS_CFG, 0x8c, true);
}

// MON-COMMS; the X20 reports the port it is talking on in txErrors
void GPS_UBlox::send_mon_comms()
{
    uint8_t comms[8 + 40] {};
    comms[0] = 0;  // version
    comms[1] = 1;  // one port block
    comms[2] = (UBX_PORT + 1) << 2;  // outputPort: 2 = UART1
    send_ubx(CLASS_MON, 0x36, comms, sizeof(comms));
}

void GPS_UBlox::handle_ubx(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t len)
{
    switch (msg_class) {
    case CLASS_CFG:
        handle_cfg(msg_id, payload, len);
        break;
    case CLASS_MON:
        if (msg_id == 0x04 && len == 0) {
            send_mon_ver();
        }
        if (msg_id == 0x36 && len == 0 && model() == Model::X20P_HPG211) {
            send_mon_comms();
        }
        break;
    case CLASS_NAV:
        if (msg_id == MSG_SVINFO && len == 0 && supports(msg_class, msg_id)) {
            send_svinfo(gps_time().ms, _sitl->gps[instance].numsats);
        }
        break;
    default:
        break;
    }
}

/*
  parse UBX messages from the autopilot; anything else (e.g. the
  $PUBX baudrate sentence) is ignored
 */
void GPS_UBlox::update_read()
{
    if (!config_simulated()) {
        GPS_Backend::update_read();
        return;
    }
    if (!config_loaded || loaded_model != model()) {
        load_saved_config();
    }

    uint8_t buf[256];
    const ssize_t n = read_from_autopilot((char*)buf, sizeof(buf));
    for (ssize_t i=0; i<n; i++) {
        const uint8_t c = buf[i];
        switch (parser.step) {
        case 0:
            if (c == 0xb5) {
                parser.step++;
            }
            break;
        case 1:
            parser.step = (c == 0x62) ? 2 : 0;
            break;
        case 2:
            parser.msg_class = c;
            parser.ck_a = parser.ck_b = c;
            parser.step++;
            break;
        case 3:
            parser.msg_id = c;
            parser.ck_b += (parser.ck_a += c);
            parser.step++;
            break;
        case 4:
            parser.len = c;
            parser.ck_b += (parser.ck_a += c);
            parser.step++;
            break;
        case 5:
            parser.len |= uint16_t(c) << 8;
            parser.ck_b += (parser.ck_a += c);
            parser.count = 0;
            if (parser.len > sizeof(parser.payload)) {
                parser.step = 0;
                break;
            }
            parser.step = parser.len == 0 ? 7 : 6;
            break;
        case 6:
            parser.payload[parser.count++] = c;
            parser.ck_b += (parser.ck_a += c);
            if (parser.count == parser.len) {
                parser.step++;
            }
            break;
        case 7:
            parser.step = (c == parser.ck_a) ? 8 : 0;
            break;
        case 8:
            parser.step = 0;
            if (c == parser.ck_b) {
                handle_ubx(parser.msg_class, parser.msg_id, parser.payload, parser.len);
            }
            break;
        }
    }
}

#endif  // AP_SIM_GPS_UBLOX_ENABLED
