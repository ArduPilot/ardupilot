/*
  SITL handling

  This simulates a GPS on a serial port

  Andrew Tridgell November 2011
 */

#include "SIM_GPS.h"

#if HAL_SIM_GPS_ENABLED

#include <time.h>

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/AP_HAL.h>
#include <SITL/SITL.h>
#include <AP_Common/NMEA.h>

// simulated CAN GPS devices get fed from our SITL estimates:
#if HAL_SIM_GPS_EXTERNAL_FIFO_ENABLED
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <AP_HAL_SITL/AP_HAL_SITL.h>
extern const HAL_SITL& hal_sitl;
#endif

// the number of GPS leap seconds - copied from AP_GPS.h
#define GPS_LEAPSECONDS_MILLIS 18000ULL

extern const AP_HAL::HAL& hal;

using namespace SITL;

GPS::GPS(uint8_t _instance) :
    SerialDevice(),
    instance{_instance}
{

#if HAL_SIM_GPS_EXTERNAL_FIFO_ENABLED
    const uint8_t num_gps = 2;
    // pipe number is SITL instance number (e.g. -I argument to
    // sim_vehicle.py) times the max number of GPSs + the gps instance
    // number:
    const uint8_t num = num_gps * hal_sitl.get_instance() + instance;
    if (asprintf(&_gps_fifo, "/tmp/gps_fifo%u", (unsigned)num) == -1) {  // FIXME - needs to work with simulated periph-gps
        AP_BoardConfig::allocation_error("gps_fifo filepath");
    }
    if (mkfifo(_gps_fifo, 0666) < 0) {
        if (errno != EEXIST) {
            printf("MKFIFO failed with %m\n");
        }
    }
#endif
}

uint32_t GPS::device_baud() const
{
    switch ((Type)_sitl->gps_type[instance].get()) {
        case Type::NOVA:
            return 19200;
        case Type::NONE:
        case Type::UBLOX:
        case Type::NMEA:
        case Type::SBP:
        case Type::SBP2:
#if AP_SIM_GPS_FILE_ENABLED
        case Type::FILE:
#endif
            return 0;  // 0 meaning unset
    }
    return 0;  // 0 meaning unset
}

/*
  write some bytes from the simulated GPS
 */
ssize_t GPS::write_to_autopilot(const char *p, size_t size) const
{
    // the second GPS instance fails in a different way to the first;
    // the first will start sending back 3 satellites, the second just
    // stops responding when disabled.  This is not necessarily a good
    // thing.
    if (instance == 1 && _sitl->gps_disable[instance]) {
        return -1;
    }

#if HAL_SIM_GPS_EXTERNAL_FIFO_ENABLED
    // also write to external fifo
    int fd = open(_gps_fifo, O_WRONLY | O_NONBLOCK);
    if (fd >= 0) {
        UNUSED_RESULT(write(fd, p, size));
        close(fd);
    }
#endif

    const float byteloss = _sitl->gps_byteloss[instance];

    // shortcut if we're not doing byteloss:
    if (!is_positive(byteloss)) {
        return SerialDevice::write_to_autopilot(p, size);
    }

    size_t ret = 0;
    while (size--) {
            float r = ((((unsigned)random()) % 1000000)) / 1.0e4;
            if (r < byteloss) {
                // lose the byte
                p++;
                continue;
            }

        const ssize_t pret = SerialDevice::write_to_autopilot(p, 1);
        if (pret == 0) {
            // no space?
            return ret;
        }
        if (pret != 1) {
            // error has occured?
            return pret;
        }
        ret++;
        p++;
    }

    return ret;
}

/*
  get timeval using simulation time
 */
static void simulation_timeval(struct timeval *tv)
{
    uint64_t now = AP_HAL::micros64();
    static uint64_t first_usec;
    static struct timeval first_tv;
    if (first_usec == 0) {
        first_usec = now;
        first_tv.tv_sec = AP::sitl()->start_time_UTC;
    }
    *tv = first_tv;
    tv->tv_sec += now / 1000000ULL;
    uint64_t new_usec = tv->tv_usec + (now % 1000000ULL);
    tv->tv_sec += new_usec / 1000000ULL;
    tv->tv_usec = new_usec % 1000000ULL;
}

/*
  send a UBLOX GPS message
 */
void GPS::send_ubx(uint8_t msgid, uint8_t *buf, uint16_t size)
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

/*
  return GPS time of week in milliseconds
 */
static void gps_time(uint16_t *time_week, uint32_t *time_week_ms)
{
    struct timeval tv;
    simulation_timeval(&tv);
    const uint32_t epoch = 86400*(10*365 + (1980-1969)/4 + 1 + 6 - 2) - (GPS_LEAPSECONDS_MILLIS / 1000ULL);
    uint32_t epoch_seconds = tv.tv_sec - epoch;
    *time_week = epoch_seconds / AP_SEC_PER_WEEK;
    uint32_t t_ms = tv.tv_usec / 1000;
    // round time to nearest 200ms
    *time_week_ms = (epoch_seconds % AP_SEC_PER_WEEK) * AP_MSEC_PER_SEC + ((t_ms/200) * 200);
}

/*
  send a new set of GPS UBLOX packets
 */
void GPS::update_ubx(const struct gps_data *d)
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
    enum RELPOSNED {
        gnssFixOK          = 1U << 0,
        diffSoln           = 1U << 1,
        relPosValid        = 1U << 2,
        carrSolnFloat      = 1U << 3,

        carrSolnFixed      = 1U << 4,
        isMoving           = 1U << 5,
        refPosMiss         = 1U << 6,
        refObsMiss         = 1U << 7,

        relPosHeadingValid = 1U << 8,
        relPosNormalized   = 1U << 9
    };
    struct PACKED ubx_nav_relposned {
        uint8_t version;
        uint8_t reserved1;
        uint16_t refStationId;
        uint32_t iTOW;
        int32_t relPosN;
        int32_t relPosE;
        int32_t relPosD;
        int32_t relPosLength;
        int32_t relPosHeading;
        uint8_t reserved2[4];
        int8_t relPosHPN;
        int8_t relPosHPE;
        int8_t relPosHPD;
        int8_t relPosHPLength;
        uint32_t accN;
        uint32_t accE;
        uint32_t accD;
        uint32_t accLength;
        uint32_t accHeading;
        uint8_t reserved3[4];
        uint32_t flags;
    } relposned {};

    const uint8_t MSG_POSLLH = 0x2;
    const uint8_t MSG_STATUS = 0x3;
    const uint8_t MSG_DOP = 0x4;
    const uint8_t MSG_VELNED = 0x12;
    const uint8_t MSG_SOL = 0x6;
    const uint8_t MSG_PVT = 0x7;
    const uint8_t MSG_SVINFO = 0x30;
    const uint8_t MSG_RELPOSNED = 0x3c;

    uint32_t _next_nav_sv_info_time = 0;

    uint16_t time_week;
    uint32_t time_week_ms;

    gps_time(&time_week, &time_week_ms);

    pos.time = time_week_ms;
    pos.longitude = d->longitude * 1.0e7;
    pos.latitude  = d->latitude * 1.0e7;
    pos.altitude_ellipsoid = d->altitude * 1000.0f;
    pos.altitude_msl = d->altitude * 1000.0f;
    pos.horizontal_accuracy = _sitl->gps_accuracy[instance]*1000;
    pos.vertical_accuracy = _sitl->gps_accuracy[instance]*1000;

    status.time = time_week_ms;
    status.fix_type = d->have_lock?3:0;
    status.fix_status = d->have_lock?1:0;
    status.differential_status = 0;
    status.res = 0;
    status.time_to_first_fix = 0;
    status.uptime = AP_HAL::millis();

    velned.time = time_week_ms;
    velned.ned_north = 100.0f * d->speedN;
    velned.ned_east  = 100.0f * d->speedE;
    velned.ned_down  = 100.0f * d->speedD;
    velned.speed_2d = norm(d->speedN, d->speedE) * 100;
    velned.speed_3d = norm(d->speedN, d->speedE, d->speedD) * 100;
    velned.heading_2d = ToDeg(atan2f(d->speedE, d->speedN)) * 100000.0f;
    if (velned.heading_2d < 0.0f) {
        velned.heading_2d += 360.0f * 100000.0f;
    }
    velned.speed_accuracy = 40;
    velned.heading_accuracy = 4;

    memset(&sol, 0, sizeof(sol));
    sol.fix_type = d->have_lock?3:0;
    sol.fix_status = 221;
    sol.satellites = d->have_lock?_sitl->gps_numsats[instance]:3;
    sol.time = time_week_ms;
    sol.week = time_week;

    dop.time = time_week_ms;
    dop.gDOP = 65535;
    dop.pDOP = 65535;
    dop.tDOP = 65535;
    dop.vDOP = 200;
    dop.hDOP = 121;
    dop.nDOP = 65535;
    dop.eDOP = 65535;
    
    pvt.itow = time_week_ms;
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
    pvt.num_sv = d->have_lock?_sitl->gps_numsats[instance]:3;
    pvt.lon = d->longitude * 1.0e7;
    pvt.lat  = d->latitude * 1.0e7;
    pvt.height = d->altitude * 1000.0f;
    pvt.h_msl = d->altitude * 1000.0f;
    pvt.h_acc = _sitl->gps_accuracy[instance] * 1000;
    pvt.v_acc = _sitl->gps_accuracy[instance] * 1000;
    pvt.velN = 1000.0f * d->speedN;
    pvt.velE = 1000.0f * d->speedE;
    pvt.velD = 1000.0f * d->speedD;
    pvt.gspeed = norm(d->speedN, d->speedE) * 1000; 
    pvt.head_mot = ToDeg(atan2f(d->speedE, d->speedN)) * 1.0e5; 
    pvt.s_acc = 40; 
    pvt.head_acc = 38 * 1.0e5; 
    pvt.p_dop = 65535; 
    memset(pvt.reserved1, '\0', ARRAY_SIZE(pvt.reserved1));
    pvt.headVeh = 0;
    memset(pvt.reserved2, '\0', ARRAY_SIZE(pvt.reserved2));

    if (_sitl->gps_hdg_enabled[instance] > SITL::SIM::GPS_HEADING_NONE) {
        const Vector3f ant1_pos = _sitl->gps_pos_offset[instance^1].get();
        const Vector3f ant2_pos = _sitl->gps_pos_offset[instance].get();
        Vector3f rel_antenna_pos = ant2_pos - ant1_pos;
        Matrix3f rot;
        // project attitude back using gyros to get antenna orientation at time of GPS sample
        Vector3f gyro(radians(_sitl->state.rollRate),
                      radians(_sitl->state.pitchRate),
                      radians(_sitl->state.yawRate));
        rot.from_euler(radians(_sitl->state.rollDeg), radians(_sitl->state.pitchDeg), radians(d->yaw_deg));
        const float lag = _sitl->gps_delay_ms[instance] * 0.001;
        rot.rotate(gyro * (-lag));
        rel_antenna_pos = rot * rel_antenna_pos;
        relposned.version = 1;
        relposned.iTOW = time_week_ms;
        relposned.relPosN = rel_antenna_pos.x * 100;
        relposned.relPosE = rel_antenna_pos.y * 100;
        relposned.relPosD = rel_antenna_pos.z * 100;
        relposned.relPosLength = rel_antenna_pos.length() * 100;
        relposned.relPosHeading = degrees(Vector2f(rel_antenna_pos.x, rel_antenna_pos.y).angle()) * 1.0e5;
        relposned.flags = gnssFixOK | diffSoln | carrSolnFixed | isMoving | relPosValid | relPosHeadingValid;
    }

    send_ubx(MSG_POSLLH, (uint8_t*)&pos, sizeof(pos));
    send_ubx(MSG_STATUS, (uint8_t*)&status, sizeof(status));
    send_ubx(MSG_VELNED, (uint8_t*)&velned, sizeof(velned));
    send_ubx(MSG_SOL,    (uint8_t*)&sol, sizeof(sol));
    send_ubx(MSG_DOP,    (uint8_t*)&dop, sizeof(dop));
    send_ubx(MSG_PVT,    (uint8_t*)&pvt, sizeof(pvt));
    if (_sitl->gps_hdg_enabled[instance] > SITL::SIM::GPS_HEADING_NONE) {
        send_ubx(MSG_RELPOSNED,    (uint8_t*)&relposned, sizeof(relposned));
    }

    if (time_week_ms > _next_nav_sv_info_time) {
        svinfo.itow = time_week_ms;
        svinfo.numCh = 32;
        svinfo.globalFlags = 4; // u-blox 8/M8
        // fill in the SV's with some data even though firmware does not currently use it
        // note that this is not using num_sats as we aren't dynamically creating this to match
        for (uint8_t i = 0; i < SV_COUNT; i++) {
            svinfo.sv[i].chn = i;
            svinfo.sv[i].svid = i;
            svinfo.sv[i].flags = (i < _sitl->gps_numsats[instance]) ? 0x7 : 0x6; // sv used, diff correction data, orbit information
            svinfo.sv[i].quality = 7; // code and carrier lock and time synchronized
            svinfo.sv[i].cno = MAX(20, 30 - i);
            svinfo.sv[i].elev = MAX(30, 90 - i);
            svinfo.sv[i].azim = i;
            // not bothering to fill in prRes
        }
        send_ubx(MSG_SVINFO, (uint8_t*)&svinfo, sizeof(svinfo));
        _next_nav_sv_info_time = time_week_ms + 10000; // 10 second delay
    }
}

/*
  formatted print of NMEA message, with checksum appended
 */
void GPS::nmea_printf(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    char *s = nmea_vaprintf(fmt, ap);
    va_end(ap);
    if (s != nullptr) {
        write_to_autopilot((const char*)s, strlen(s));
        free(s);
    }
}


/*
  send a new GPS NMEA packet
 */
void GPS::update_nmea(const struct gps_data *d)
{
    struct timeval tv;
    struct tm *tm;
    char tstring[20];
    char dstring[20];
    char lat_string[20];
    char lng_string[20];

    simulation_timeval(&tv);

    tm = gmtime(&tv.tv_sec);

    // format time string
    snprintf(tstring, sizeof(tstring), "%02u%02u%06.3f", tm->tm_hour, tm->tm_min, tm->tm_sec + tv.tv_usec*1.0e-6);

    // format date string
    snprintf(dstring, sizeof(dstring), "%02u%02u%02u", tm->tm_mday, tm->tm_mon+1, tm->tm_year % 100);

    // format latitude
    double deg = fabs(d->latitude);
    snprintf(lat_string, sizeof(lat_string), "%02u%08.5f,%c",
             (unsigned)deg,
             (deg - int(deg))*60,
             d->latitude<0?'S':'N');

    // format longitude
    deg = fabs(d->longitude);
    snprintf(lng_string, sizeof(lng_string), "%03u%08.5f,%c",
             (unsigned)deg,
             (deg - int(deg))*60,
             d->longitude<0?'W':'E');

    nmea_printf("$GPGGA,%s,%s,%s,%01d,%02d,%04.1f,%07.2f,M,0.0,M,,",
                     tstring,
                     lat_string,
                     lng_string,
                     d->have_lock?1:0,
                     d->have_lock?_sitl->gps_numsats[instance]:3,
                     2.0,
                     d->altitude);
    const float speed_mps = norm(d->speedN, d->speedE);
    const float speed_knots = speed_mps * M_PER_SEC_TO_KNOTS;

    float heading = ToDeg(atan2f(d->speedE, d->speedN));
    if (heading < 0) {
        heading += 360.0f;
    }

    //$GPVTG,133.18,T,120.79,M,0.11,N,0.20,K,A*24
    nmea_printf("$GPVTG,%.2f,T,%.2f,M,%.2f,N,%.2f,K,A",
                     tstring,
                     heading,
                     heading,
                     speed_knots,
                     speed_knots * KNOTS_TO_METERS_PER_SECOND * 3.6);

    nmea_printf("$GPRMC,%s,%c,%s,%s,%.2f,%.2f,%s,,",
                     tstring,
                     d->have_lock?'A':'V',
                     lat_string,
                     lng_string,
                     speed_knots,
                     heading,
                     dstring);

    if (_sitl->gps_hdg_enabled[instance] == SITL::SIM::GPS_HEADING_HDT) {
        nmea_printf("$GPHDT,%.2f,T", d->yaw_deg);
    }
    else if (_sitl->gps_hdg_enabled[instance] == SITL::SIM::GPS_HEADING_THS) {
        nmea_printf("$GPTHS,%.2f,%c,T", d->yaw_deg, d->have_lock ? 'A' : 'V');
    } else if (_sitl->gps_hdg_enabled[instance] == SITL::SIM::GPS_HEADING_KSXT) {
        // Unicore support
        // $KSXT,20211016083433.00,116.31296102,39.95817066,49.4911,223.57,-11.32,330.19,0.024,,1,3,28,27,,,,-0.012,0.021,0.020,,*2D
        nmea_printf("$KSXT,%04u%02u%02u%02u%02u%02u.%02u,%.8f,%.8f,%.4f,%.2f,%.2f,%.2f,%.2f,%.3f,%u,%u,%u,%u,,,,%.3f,%.3f,%.3f,,",
                    tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec, unsigned(tv.tv_usec*1.e-4),
                    d->longitude, d->latitude,
                    d->altitude,
                    wrap_360(d->yaw_deg),
                    d->pitch_deg,
                    heading,
                    speed_mps,
                    d->roll_deg,
                    d->have_lock?1:0, // 2=rtkfloat 3=rtkfixed,
                    3, // fixed rtk yaw solution,
                    d->have_lock?_sitl->gps_numsats[instance]:3,
                    d->have_lock?_sitl->gps_numsats[instance]:3,
                    d->speedE * 3.6,
                    d->speedN * 3.6,
                    -d->speedD * 3.6);
    }
}

void GPS::sbp_send_message(uint16_t msg_type, uint16_t sender_id, uint8_t len, uint8_t *payload)
{
    if (len != 0 && payload == 0) {
        return; //SBP_NULL_ERROR;
    }

    uint8_t preamble = 0x55;
    write_to_autopilot((char*)&preamble, 1);
    write_to_autopilot((char*)&msg_type, 2);
    write_to_autopilot((char*)&sender_id, 2);
    write_to_autopilot((char*)&len, 1);
    if (len > 0) {
        write_to_autopilot((char*)payload, len);
    }

    uint16_t crc;
    crc = crc16_ccitt((uint8_t*)&(msg_type), 2, 0);
    crc = crc16_ccitt((uint8_t*)&(sender_id), 2, crc);
    crc = crc16_ccitt(&(len), 1, crc);
    crc = crc16_ccitt(payload, len, crc);
    write_to_autopilot((char*)&crc, 2);
}

void GPS::update_sbp(const struct gps_data *d)
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
    uint16_t time_week;
    uint32_t time_week_ms;

    gps_time(&time_week, &time_week_ms);

    t.wn = time_week;
    t.tow = time_week_ms;
    t.ns = 0;
    t.flags = 0;
    sbp_send_message(SBP_GPS_TIME_MSGTYPE, 0x2222, sizeof(t), (uint8_t*)&t);

    if (!d->have_lock) {
        return;
    }

    pos.tow = time_week_ms;
    pos.lon = d->longitude;
    pos.lat= d->latitude;
    pos.height = d->altitude;
    pos.h_accuracy = _sitl->gps_accuracy[instance]*1000;
    pos.v_accuracy = _sitl->gps_accuracy[instance]*1000;
    pos.n_sats = _sitl->gps_numsats[instance];

    // Send single point position solution
    pos.flags = 0;
    sbp_send_message(SBP_POS_LLH_MSGTYPE, 0x2222, sizeof(pos), (uint8_t*)&pos);
    // Send "pseudo-absolute" RTK position solution
    pos.flags = 1;
    sbp_send_message(SBP_POS_LLH_MSGTYPE, 0x2222, sizeof(pos), (uint8_t*)&pos);

    velned.tow = time_week_ms;
    velned.n = 1e3 * d->speedN;
    velned.e = 1e3 * d->speedE;
    velned.d = 1e3 * d->speedD;
    velned.h_accuracy = 5e3;
    velned.v_accuracy = 5e3;
    velned.n_sats = _sitl->gps_numsats[instance];
    velned.flags = 0;
    sbp_send_message(SBP_VEL_NED_MSGTYPE, 0x2222, sizeof(velned), (uint8_t*)&velned);

    static uint32_t do_every_count = 0;
    if (do_every_count % 5 == 0) {

        dops.tow = time_week_ms;
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


void GPS::update_sbp2(const struct gps_data *d)
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
    static const uint16_t SBP_GPS_TIME_MSGTYPE = 0x0102;
    static const uint16_t SBP_DOPS_MSGTYPE = 0x0208;
    static const uint16_t SBP_POS_LLH_MSGTYPE = 0x020A;
    static const uint16_t SBP_VEL_NED_MSGTYPE = 0x020E;

    uint16_t time_week;
    uint32_t time_week_ms;

    gps_time(&time_week, &time_week_ms);

    t.wn = time_week;
    t.tow = time_week_ms;
    t.ns = 0;
    t.flags = 1;
    sbp_send_message(SBP_GPS_TIME_MSGTYPE, 0x2222, sizeof(t), (uint8_t*)&t);

    if (!d->have_lock) {
        return;
    }

    pos.tow = time_week_ms;
    pos.lon = d->longitude;
    pos.lat= d->latitude;
    pos.height = d->altitude;
    pos.h_accuracy = _sitl->gps_accuracy[instance]*1000;
    pos.v_accuracy = _sitl->gps_accuracy[instance]*1000;
    pos.n_sats = _sitl->gps_numsats[instance];

    // Send single point position solution
    pos.flags = 1;
    sbp_send_message(SBP_POS_LLH_MSGTYPE, 0x2222, sizeof(pos), (uint8_t*)&pos);
    // Send "pseudo-absolute" RTK position solution
    pos.flags = 4;
    sbp_send_message(SBP_POS_LLH_MSGTYPE, 0x2222, sizeof(pos), (uint8_t*)&pos);

    velned.tow = time_week_ms;
    velned.n = 1e3 * d->speedN;
    velned.e = 1e3 * d->speedE;
    velned.d = 1e3 * d->speedD;
    velned.h_accuracy = 5e3;
    velned.v_accuracy = 5e3;
    velned.n_sats = _sitl->gps_numsats[instance];
    velned.flags = 1;
    sbp_send_message(SBP_VEL_NED_MSGTYPE, 0x2222, sizeof(velned), (uint8_t*)&velned);

    static uint32_t do_every_count = 0;
    if (do_every_count % 5 == 0) {

        dops.tow = time_week_ms;
        dops.gdop = 1;
        dops.pdop = 1;
        dops.tdop = 1;
        dops.hdop = 100;
        dops.vdop = 1;
        dops.flags = 1;
        sbp_send_message(SBP_DOPS_MSGTYPE, 0x2222, sizeof(dops),
                          (uint8_t*)&dops);

        hb.protocol_major = 2; //Sends protocol version 2.0
        sbp_send_message(SBP_HEARTBEAT_MSGTYPE, 0x2222, sizeof(hb),
                          (uint8_t*)&hb);
    }
    do_every_count++;
}

void GPS::update_nova(const struct gps_data *d)
{
    static struct PACKED nova_header
    {
        // 0
        uint8_t preamble[3];
        // 3
        uint8_t headerlength;
        // 4
        uint16_t messageid;
        // 6
        uint8_t messagetype;
        //7
        uint8_t portaddr;
        //8
        uint16_t messagelength;
        //10
        uint16_t sequence;
        //12
        uint8_t idletime;
        //13
        uint8_t timestatus;
        //14
        uint16_t week;
        //16
        uint32_t tow;
        //20
        uint32_t recvstatus;
        // 24
        uint16_t resv;
        //26
        uint16_t recvswver;
    } header;

    struct PACKED psrdop
    {
        float gdop;
        float pdop;
        float hdop;
        float htdop;
        float tdop;
        float cutoff;
        uint32_t svcount;
        // extra data for individual prns
    } psrdop {};

    struct PACKED bestpos
    {
        uint32_t solstat;
        uint32_t postype;
        double lat;
        double lng;
        double hgt;
        float undulation;
        uint32_t datumid;
        float latsdev;
        float lngsdev;
        float hgtsdev;
        // 4 bytes
        uint8_t stnid[4];
        float diffage;
        float sol_age;
        uint8_t svstracked;
        uint8_t svsused;
        uint8_t svsl1;
        uint8_t svsmultfreq;
        uint8_t resv;
        uint8_t extsolstat;
        uint8_t galbeisigmask;
        uint8_t gpsglosigmask;
    } bestpos {};

    struct PACKED bestvel
    {
        uint32_t solstat;
        uint32_t veltype;
        float latency;
        float age;
        double horspd;
        double trkgnd;
        // + up
        double vertspd;
        float resv;
    } bestvel {};
    
    uint16_t time_week;
    uint32_t time_week_ms;
    
    gps_time(&time_week, &time_week_ms);
    
    header.preamble[0] = 0xaa;
    header.preamble[1] = 0x44;
    header.preamble[2] = 0x12;
    header.headerlength = sizeof(header);
    header.week = time_week;
    header.tow = time_week_ms;
    
    header.messageid = 174;
    header.messagelength = sizeof(psrdop);
    header.sequence += 1;
    
    psrdop.hdop = 1.20;
    psrdop.htdop = 1.20;
    nova_send_message((uint8_t*)&header,sizeof(header),(uint8_t*)&psrdop, sizeof(psrdop));


    header.messageid = 99;
    header.messagelength = sizeof(bestvel);
    header.sequence += 1;
    
    bestvel.horspd = norm(d->speedN, d->speedE);  
    bestvel.trkgnd = ToDeg(atan2f(d->speedE, d->speedN));
    bestvel.vertspd = -d->speedD;

    nova_send_message((uint8_t*)&header,sizeof(header),(uint8_t*)&bestvel, sizeof(bestvel));


    header.messageid = 42;
    header.messagelength = sizeof(bestpos);
    header.sequence += 1;
    
    bestpos.lat = d->latitude;
    bestpos.lng = d->longitude;
    bestpos.hgt = d->altitude;
    bestpos.svsused = _sitl->gps_numsats[instance];
    bestpos.latsdev=0.2;
    bestpos.lngsdev=0.2;
    bestpos.hgtsdev=0.2;
    bestpos.solstat=0;
    bestpos.postype=32;

    nova_send_message((uint8_t*)&header,sizeof(header),(uint8_t*)&bestpos, sizeof(bestpos));
}

void GPS::nova_send_message(uint8_t *header, uint8_t headerlength, uint8_t *payload, uint8_t payloadlen)
{
    write_to_autopilot((char*)header, headerlength);
write_to_autopilot((char*)payload, payloadlen);

    uint32_t crc = CalculateBlockCRC32(headerlength, header, (uint32_t)0);
    crc = CalculateBlockCRC32(payloadlen, payload, crc);

    write_to_autopilot((char*)&crc, 4);
}

#define CRC32_POLYNOMIAL 0xEDB88320L
uint32_t GPS::CRC32Value(uint32_t icrc)
{
    int i;
    uint32_t crc = icrc;
    for ( i = 8 ; i > 0; i-- )
    {
        if ( crc & 1 )
            crc = ( crc >> 1 ) ^ CRC32_POLYNOMIAL;
        else
            crc >>= 1;
    }
    return crc;
}

uint32_t GPS::CalculateBlockCRC32(uint32_t length, uint8_t *buffer, uint32_t crc)
{
    while ( length-- != 0 )
    {
        crc = ((crc >> 8) & 0x00FFFFFFL) ^ (CRC32Value(((uint32_t) crc ^ *buffer++) & 0xff));
    }
    return( crc );
}

/*
  temporary method to use file as GPS data
 */
#if AP_SIM_GPS_FILE_ENABLED
void GPS::update_file()
{
    static int fd = -1;
    static int fd2 = -1;
    int temp_fd;
    if (instance == 0) {
        if (fd == -1) {
            fd = open("/tmp/gps.dat", O_RDONLY|O_CLOEXEC);
        }
        temp_fd = fd;
    } else {
        if (fd2 == -1) {
            fd2 = open("/tmp/gps2.dat", O_RDONLY|O_CLOEXEC);
        }
        temp_fd = fd2;
    }

    if (temp_fd == -1) {
        return;
    }
    char buf[200];
    ssize_t ret = ::read(temp_fd, buf, sizeof(buf));
    if (ret > 0) {
        ::printf("wrote gps %u bytes\n", (unsigned)ret);
        write_to_autopilot((const char *)buf, ret);
    }
    if (ret == 0) {
        ::printf("gps rewind\n");
        lseek(temp_fd, 0, SEEK_SET);
    }
}
#endif  // AP_SIM_GPS_FILE_ENABLED

/*
  possibly send a new GPS packet
 */
void GPS::update()
{
    if (!init_sitl_pointer()) {
        return;
    }

    double latitude =_sitl->state.latitude;
    double longitude = _sitl->state.longitude;
    float altitude = _sitl->state.altitude;
    const double speedN = _sitl->state.speedN;
    const double speedE = _sitl->state.speedE;
    const double speedD = _sitl->state.speedD;
    const uint32_t now_ms = AP_HAL::millis();

    if (now_ms < 20000) {
        // apply the init offsets for the first 20s. This allows for
        // having the origin a long way from the takeoff location,
        // which makes testing long flights easier
        latitude += _sitl->gps_init_lat_ofs;
        longitude += _sitl->gps_init_lon_ofs;
        altitude += _sitl->gps_init_alt_ofs;
    }

    //Capture current position as basestation location for
    if (!_gps_has_basestation_position &&
        now_ms >= _sitl->gps_lock_time[0]*1000UL) {
        _gps_basestation_data.latitude = latitude;
        _gps_basestation_data.longitude = longitude;
        _gps_basestation_data.altitude = altitude;
        _gps_basestation_data.speedN = speedN;
        _gps_basestation_data.speedE = speedE;
        _gps_basestation_data.speedD = speedD;
        _gps_has_basestation_position = true;
    }

    const uint8_t idx = instance;  // alias to avoid code churn

        struct gps_data d;

        // simulate delayed lock times
        bool have_lock = (!_sitl->gps_disable[idx] && now_ms >= _sitl->gps_lock_time[idx]*1000UL);

        // run at configured GPS rate (default 5Hz)
        if ((now_ms - last_update) < (uint32_t)(1000/_sitl->gps_hertz[idx])) {
            return;
        }

        // swallow any config bytes
        char c;
        read_from_autopilot(&c, 1);

        last_update = now_ms;

        d.latitude = latitude;
        d.longitude = longitude;
        d.yaw_deg = _sitl->state.yawDeg;
        d.roll_deg = _sitl->state.rollDeg;
        d.pitch_deg = _sitl->state.pitchDeg;

        // add an altitude error controlled by a slow sine wave
        d.altitude = altitude + _sitl->gps_noise[idx] * sinf(now_ms * 0.0005f) + _sitl->gps_alt_offset[idx];

        // Add offet to c.g. velocity to get velocity at antenna and add simulated error
        Vector3f velErrorNED = _sitl->gps_vel_err[idx];
        d.speedN = speedN + (velErrorNED.x * rand_float());
        d.speedE = speedE + (velErrorNED.y * rand_float()); 
        d.speedD = speedD + (velErrorNED.z * rand_float());
        d.have_lock = have_lock;

        if (_sitl->gps_drift_alt[idx] > 0) {
            // slow altitude drift
            d.altitude += _sitl->gps_drift_alt[idx]*sinf(now_ms*0.001f*0.02f);
        }

        // correct the latitude, longitude, height and NED velocity for the offset between
        // the vehicle c.g. and GPs antenna
        Vector3f posRelOffsetBF = _sitl->gps_pos_offset[idx];
        if (!posRelOffsetBF.is_zero()) {
            // get a rotation matrix following DCM conventions (body to earth)
            Matrix3f rotmat;
            _sitl->state.quaternion.rotation_matrix(rotmat);

            // rotate the antenna offset into the earth frame
            Vector3f posRelOffsetEF = rotmat * posRelOffsetBF;

            // Add the offset to the latitude, longitude and height using a spherical earth approximation
            double const earth_rad_inv = 1.569612305760477e-7; // use Authalic/Volumetric radius
            double lng_scale_factor = earth_rad_inv / cos(radians(d.latitude));
            d.latitude += degrees(posRelOffsetEF.x * earth_rad_inv);
            d.longitude += degrees(posRelOffsetEF.y * lng_scale_factor);
            d.altitude -= posRelOffsetEF.z;

            // calculate a velocity offset due to the antenna position offset and body rotation rate
            // note: % operator is overloaded for cross product
            Vector3f gyro(radians(_sitl->state.rollRate),
                          radians(_sitl->state.pitchRate),
                          radians(_sitl->state.yawRate));
            Vector3f velRelOffsetBF = gyro % posRelOffsetBF;

            // rotate the velocity offset into earth frame and add to the c.g. velocity
            Vector3f velRelOffsetEF = rotmat * velRelOffsetBF;
            d.speedN += velRelOffsetEF.x;
            d.speedE += velRelOffsetEF.y;
            d.speedD += velRelOffsetEF.z;
        }

        // get delayed data
        d.timestamp_ms = now_ms;
        d = interpolate_data(d, _sitl->gps_delay_ms[instance]);

        // Applying GPS glitch
        // Using first gps glitch
        Vector3f glitch_offsets = _sitl->gps_glitch[idx];
        d.latitude += glitch_offsets.x;
        d.longitude += glitch_offsets.y;
        d.altitude += glitch_offsets.z;


    // do GPS-type-dependent updates:
    switch ((Type)_sitl->gps_type[instance].get()) {
        case Type::NONE:
            // no GPS attached
            break;

        case Type::UBLOX:
            update_ubx(&d);
            break;

        case Type::NMEA:
            update_nmea(&d);
            break;

        case Type::SBP:
            update_sbp(&d);
            break;

        case Type::SBP2:
            update_sbp2(&d);
            break;

        case Type::NOVA:
            update_nova(&d);
            break;

#if AP_SIM_GPS_FILE_ENABLED
        case Type::FILE:
            update_file();
            break;
#endif
    }
}

/*
  get delayed data by interpolation
*/
GPS::gps_data GPS::interpolate_data(const gps_data &d, uint32_t delay_ms)
{
    const uint8_t N = ARRAY_SIZE(_gps_history);
    const uint32_t now_ms = d.timestamp_ms;

    // add in into history array, shifting old elements
    memmove(&_gps_history[1], &_gps_history[0], sizeof(_gps_history[0])*(ARRAY_SIZE(_gps_history)-1));
    _gps_history[0] = d;

    for (uint8_t i=0; i<N-1; i++) {
        uint32_t dt1 = now_ms - _gps_history[i].timestamp_ms;
        uint32_t dt2 = now_ms - _gps_history[i+1].timestamp_ms;
        if (delay_ms >= dt1 && delay_ms <= dt2) {
            // we will interpolate this pair of samples. Start with
            // the older sample
            const gps_data &s1 = _gps_history[i+1];
            const gps_data &s2 = _gps_history[i];
            gps_data d2 = s1;
            const float p = (dt2 - delay_ms) / MAX(1,float(dt2 - dt1));
            d2.latitude += p * (s2.latitude - s1.latitude);
            d2.longitude += p * (s2.longitude - s1.longitude);
            d2.altitude += p * (s2.altitude - s1.altitude);
            d2.speedN += p * (s2.speedN - s1.speedN);
            d2.speedE += p * (s2.speedE - s1.speedE);
            d2.speedD += p * (s2.speedD - s1.speedD);
            d2.yaw_deg += p * (s2.yaw_deg - s1.yaw_deg);
            return d2;
        }
    }
    // delay is too long, use last sample
    return _gps_history[N-1];
}

#endif  // HAL_SIM_GPS_ENABLED
