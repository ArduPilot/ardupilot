#include "SIM_config.h"

#if AP_SIM_GPS_UBLOX_ENABLED

#include "SIM_GPS_UBLOX.h"

#include <SITL/SITL.h>

using namespace SITL;

/*
  send a UBLOX GPS message
 */
void GPS_UBlox::send_ubx(uint8_t msg_class, uint8_t msgid, uint8_t *buf, uint16_t size)
{
    const uint8_t PREAMBLE1 = 0xb5;
    const uint8_t PREAMBLE2 = 0x62;
    uint8_t hdr[6], chk[2];
    hdr[0] = PREAMBLE1;
    hdr[1] = PREAMBLE2;
    hdr[2] = msg_class;
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
  parse incoming UBX bytes from the autopilot and handle CFG messages
 */
void GPS_UBlox::update_read()
{
    char raw[64];
    ssize_t nread = read_from_autopilot(raw, sizeof(raw));
    for (ssize_t i = 0; i < nread; i++) {
        const uint8_t b = (uint8_t)raw[i];
        switch (_in_step) {
        case InStep::PREAMBLE1:
            if (b == 0xB5) { _in_step = InStep::PREAMBLE2; }
            break;
        case InStep::PREAMBLE2:
            _in_step = (b == 0x62) ? InStep::CLASS : InStep::PREAMBLE1;
            break;
        case InStep::CLASS:
            _in_class = b;
            _in_ck_a = b; _in_ck_b = b;
            _in_step = InStep::MSG_ID;
            break;
        case InStep::MSG_ID:
            _in_msg_id = b;
            _in_ck_b += (_in_ck_a += b);
            _in_step = InStep::LEN_LO;
            break;
        case InStep::LEN_LO:
            _in_len = b;
            _in_ck_b += (_in_ck_a += b);
            _in_step = InStep::LEN_HI;
            break;
        case InStep::LEN_HI:
            _in_len |= (uint16_t)b << 8;
            _in_ck_b += (_in_ck_a += b);
            _in_pos = 0;
            _in_step = (_in_len > 0) ? InStep::PAYLOAD : InStep::CK_A;
            break;
        case InStep::PAYLOAD:
            _in_ck_b += (_in_ck_a += b);
            if (_in_pos < sizeof(_in_buf)) {
                _in_buf[_in_pos] = b;
            }
            if (++_in_pos >= _in_len) {
                _in_step = InStep::CK_A;
            }
            break;
        case InStep::CK_A:
            _in_step = (b == _in_ck_a) ? InStep::CK_B : InStep::PREAMBLE1;
            break;
        case InStep::CK_B:
            _in_step = InStep::PREAMBLE1;
            if (b == _in_ck_b) {
                handle_incoming_ubx();
            }
            break;
        }
    }
}

void GPS_UBlox::handle_incoming_ubx()
{
    const uint8_t CLASS_CFG    = 0x06;
    const uint8_t CLASS_RXM    = 0x02;
    const uint8_t CLASS_ACK    = 0x05;
    const uint8_t MSG_CFG_PRT  = 0x00;
    const uint8_t MSG_CFG_MSG  = 0x01;
    const uint8_t MSG_RXM_RAWX = 0x15;
    const uint8_t MSG_ACK_ACK  = 0x01;

    if (_in_class != CLASS_CFG) {
        return;
    }

    if (_in_msg_id == MSG_CFG_PRT && _in_len == 0) {
        // Driver polls current port, respond with portID=1 (UART1)
        uint8_t prt[1] = {1};
        send_ubx(CLASS_CFG, MSG_CFG_PRT, prt, sizeof(prt));
        return;
    }

    if (_in_msg_id == MSG_CFG_MSG) {
        if (_in_len == 2) {
            // Driver polls rate for a message class/id pair
            const uint8_t req_class = _in_buf[0];
            const uint8_t req_id    = _in_buf[1];
            if (req_class == CLASS_RXM && req_id == MSG_RXM_RAWX) {
                // respond with current rate so driver can compare
                uint8_t resp[3] = {req_class, req_id, _rawx_rate};
                send_ubx(CLASS_CFG, MSG_CFG_MSG, resp, sizeof(resp));
            }
        } else if (_in_len == 3) {
            // Driver sets rate for a message class/id pair
            const uint8_t req_class = _in_buf[0];
            const uint8_t req_id    = _in_buf[1];
            const uint8_t rate      = _in_buf[2];
            if (req_class == CLASS_RXM && req_id == MSG_RXM_RAWX) {
                _rawx_rate = rate;
                // ACK the config message
                uint8_t ack[2] = {CLASS_CFG, MSG_CFG_MSG};
                send_ubx(CLASS_ACK, MSG_ACK_ACK, ack, sizeof(ack));
            }
        }
    }
}

/*
  send an RXM-RAWX frame with num_meas SV records.
  All SV data is deterministic dummy data; only the header epoch fields are real.
  This lets the parser/logger path be exercised in SITL without real observations.
 */
void GPS_UBlox::send_rxm_rawx(uint8_t num_meas)
{
    const uint8_t CLASS_RXM  = 0x02;
    const uint8_t MSG_RXM_RAWX = 0x15;

    const uint16_t hdr_size = 16;   // ubx_rxm_rawx fixed header
    const uint16_t sv_size  = 32;   // ubx_rxm_rawx_sv record
    const uint16_t payload_len = hdr_size + (uint16_t)num_meas * sv_size;

    const auto gps_tow = gps_time();

    // UBX preamble + class/id/length (6 bytes)
    uint8_t preamble[6];
    preamble[0] = 0xB5;
    preamble[1] = 0x62;
    preamble[2] = CLASS_RXM;
    preamble[3] = MSG_RXM_RAWX;
    preamble[4] = payload_len & 0xFF;
    preamble[5] = payload_len >> 8;
    write_to_autopilot((char*)preamble, sizeof(preamble));

    // Fletcher-8 checksum accumulated over class..last payload byte
    uint8_t ck_a = 0, ck_b = 0;
    auto feed = [&](const uint8_t *data, uint16_t len) {
        for (uint16_t i = 0; i < len; i++) {
            ck_b += (ck_a += data[i]);
        }
    };
    feed(preamble + 2, 4); // class, id, len_lo, len_hi

    // RAWX header (16 bytes) matches ubx_rxm_rawx fixed fields
    struct PACKED rawx_header_t {
        double   rcvTow;
        uint16_t week;
        int8_t   leapS;
        uint8_t  numMeas;
        uint8_t  recStat;
        uint8_t  reserved1[3];
    } rawx_hdr {};
    rawx_hdr.rcvTow  = gps_tow.ms * 1.0e-3;
    rawx_hdr.week    = gps_tow.week;
    rawx_hdr.leapS   = 18;
    rawx_hdr.numMeas = num_meas;
    rawx_hdr.recStat = 0;
    feed((uint8_t*)&rawx_hdr, sizeof(rawx_hdr));
    write_to_autopilot((char*)&rawx_hdr, sizeof(rawx_hdr));

    // SV records (32 bytes each) matches ubx_rxm_rawx_sv
    struct PACKED rawx_sv_t {
        double   prMes;
        double   cpMes;
        float    doMes;
        uint8_t  gnssId;
        uint8_t  svId;
        uint8_t  sigId;
        uint8_t  freqId;
        uint16_t locktime;
        uint8_t  cno;
        uint8_t  prStdev;
        uint8_t  cpStdev;
        uint8_t  doStdev;
        uint8_t  trkStat;
        uint8_t  reserved3;
    };
    for (uint8_t i = 0; i < num_meas; i++) {
        rawx_sv_t sv {};
        sv.gnssId = i % 4;
        sv.svId   = i + 1;
        // Encode GPS instance into cno (40 + instance) so the autotest can
        // verify that GRXS records from different GPS instances are not mixed.
        sv.cno    = 40 + instance;
        feed((uint8_t*)&sv, sizeof(sv));
        write_to_autopilot((char*)&sv, sizeof(sv));
    }

    uint8_t chk[2] = {ck_a, ck_b};
    write_to_autopilot((char*)chk, sizeof(chk));
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
    const uint8_t MSG_TIMEGPS = 0x20;
    const uint8_t MSG_SVINFO = 0x30;
    const uint8_t MSG_RELPOSNED = 0x3c;

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

    const uint8_t CLASS_NAV = 0x1;
    send_ubx(CLASS_NAV, MSG_POSLLH, (uint8_t*)&pos, sizeof(pos));
    send_ubx(CLASS_NAV, MSG_STATUS, (uint8_t*)&status, sizeof(status));
    send_ubx(CLASS_NAV, MSG_VELNED, (uint8_t*)&velned, sizeof(velned));
    const bool is_f9p = (_sitl->gps[instance].options & static_cast<int32_t>(SITL::SIM::GPSOptions::UBX_IS_F9P)) != 0;

    if (is_f9p) {
        const uint32_t now_ms = AP_HAL::millis();
        if ((int32_t)(now_ms - _next_timegps_send_ms) >= 0) {
            _next_timegps_send_ms = now_ms + 1000;
            send_ubx(CLASS_NAV, MSG_TIMEGPS, (uint8_t*)&timegps, sizeof(timegps));
        }
    } else {
        // F9P and later use TIMEGPS to set week number, older u-blox use SOL
        send_ubx(CLASS_NAV, MSG_SOL,    (uint8_t*)&sol, sizeof(sol));
    }
    send_ubx(CLASS_NAV, MSG_DOP,    (uint8_t*)&dop, sizeof(dop));
    send_ubx(CLASS_NAV, MSG_PVT,    (uint8_t*)&pvt, sizeof(pvt));
    if (_sitl->gps[instance].hdg_enabled > SITL::SIM::GPS_HEADING_NONE) {
        send_ubx(CLASS_NAV, MSG_RELPOSNED,    (uint8_t*)&relposned, sizeof(relposned));
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
        send_ubx(CLASS_NAV, MSG_SVINFO, (uint8_t*)&svinfo, sizeof(svinfo));
        _next_nav_sv_info_time = gps_tow.ms + 10000; // 10 second delay
    }

    if (d->have_lock && _rawx_rate > 0) {
        send_rxm_rawx(SITL_RAWX_NUM_MEAS);
    }
}

#endif  // AP_SIM_GPS_UBLOX_ENABLED
