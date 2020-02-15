/*
  SITL handling

  This simulates a GPS on a serial port

  Andrew Tridgell November 2011
 */

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"

#include <AP_Math/AP_Math.h>
#include <SITL/SITL.h>
#include "Scheduler.h"
#include "UARTDriver.h"
#include <AP_GPS/AP_GPS.h>
#include <AP_GPS/AP_GPS_UBLOX.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>

#pragma GCC diagnostic ignored "-Wunused-result"

using namespace HALSITL;
extern const AP_HAL::HAL& hal;

// state of GPS emulation
static struct gps_state {
    /* pipe emulating UBLOX GPS serial stream */
    int gps_fd, client_fd;
    uint32_t last_update; // milliseconds

    uint8_t next_index;
    uint8_t delay;
} gps_state[2];

/*
  hook for reading from the GPS pipe
 */
ssize_t SITL_State::gps_read(int fd, void *buf, size_t count)
{
#ifdef FIONREAD
    // use FIONREAD to get exact value if possible
    int num_ready;
    while (ioctl(fd, FIONREAD, &num_ready) == 0 && num_ready > 3000) {
        // the pipe is filling up - drain it
        uint8_t tmp[128];
        if (read(fd, tmp, sizeof(tmp)) != sizeof(tmp)) {
            break;
        }
    }
#endif
    return read(fd, buf, count);
}

/*
  setup GPS input pipe
 */
int SITL_State::gps_pipe(uint8_t idx)
{
    int fd[2];
    if (gps_state[idx].client_fd != 0) {
        return gps_state[idx].client_fd;
    }
    pipe(fd);
    gps_state[idx].gps_fd    = fd[1];
    gps_state[idx].client_fd = fd[0];
    gps_state[idx].last_update = AP_HAL::millis();
    fcntl(fd[0], F_SETFD, FD_CLOEXEC);
    fcntl(fd[1], F_SETFD, FD_CLOEXEC);
    HALSITL::UARTDriver::_set_nonblocking(gps_state[idx].gps_fd);
    HALSITL::UARTDriver::_set_nonblocking(fd[0]);
    return gps_state[idx].client_fd;
}

/*
  write some bytes from the simulated GPS
 */
void SITL_State::_gps_write(const uint8_t *p, uint16_t size, uint8_t instance)
{
    if (instance == 1 && !_sitl->gps2_enable) {
        return;
    }
    while (size--) {
        if (_sitl->gps_byteloss > 0.0f) {
            float r = ((((unsigned)random()) % 1000000)) / 1.0e4;
            if (r < _sitl->gps_byteloss) {
                // lose the byte
                p++;
                continue;
            }
        }
        if (gps_state[instance].gps_fd != 0) {
            write(gps_state[instance].gps_fd, p, 1);
        }
        p++;
    }
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
        gettimeofday(&first_tv, nullptr);
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
void SITL_State::_gps_send_ubx(uint8_t msgid, uint8_t *buf, uint16_t size, uint8_t instance)
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
    for (uint8_t i=0; i<size; i++) {
        chk[1] += (chk[0] += buf[i]);
    }
    _gps_write(hdr, sizeof(hdr), instance);
    _gps_write(buf, size, instance);
    _gps_write(chk, sizeof(chk), instance);
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
void SITL_State::_update_gps_ubx(const struct gps_data *d, uint8_t instance)
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
    const uint8_t MSG_POSLLH = 0x2;
    const uint8_t MSG_STATUS = 0x3;
    const uint8_t MSG_DOP = 0x4;
    const uint8_t MSG_VELNED = 0x12;
    const uint8_t MSG_SOL = 0x6;
    const uint8_t MSG_PVT = 0x7;
    const uint8_t MSG_SVINFO = 0x30;

    static uint32_t _next_nav_sv_info_time = 0;

    uint16_t time_week;
    uint32_t time_week_ms;

    gps_time(&time_week, &time_week_ms);

    pos.time = time_week_ms;
    pos.longitude = d->longitude * 1.0e7;
    pos.latitude  = d->latitude * 1.0e7;
    pos.altitude_ellipsoid = d->altitude * 1000.0f;
    pos.altitude_msl = d->altitude * 1000.0f;
    pos.horizontal_accuracy = 1500;
    pos.vertical_accuracy = 2000;

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
    sol.satellites = d->have_lock?_sitl->gps_numsats:3;
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
    pvt.num_sv = d->have_lock?_sitl->gps_numsats:3; 
    pvt.lon = d->longitude * 1.0e7;
    pvt.lat  = d->latitude * 1.0e7;
    pvt.height = d->altitude * 1000.0f;
    pvt.h_msl = d->altitude * 1000.0f;
    pvt.h_acc = 200;
    pvt.v_acc = 200; 
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

    _gps_send_ubx(MSG_POSLLH, (uint8_t*)&pos, sizeof(pos), instance);
    _gps_send_ubx(MSG_STATUS, (uint8_t*)&status, sizeof(status), instance);
    _gps_send_ubx(MSG_VELNED, (uint8_t*)&velned, sizeof(velned), instance);
    _gps_send_ubx(MSG_SOL,    (uint8_t*)&sol, sizeof(sol), instance);
    _gps_send_ubx(MSG_DOP,    (uint8_t*)&dop, sizeof(dop), instance);
    _gps_send_ubx(MSG_PVT,    (uint8_t*)&pvt, sizeof(pvt), instance);

    if (time_week_ms > _next_nav_sv_info_time) {
        svinfo.itow = time_week_ms;
        svinfo.numCh = 32;
        svinfo.globalFlags = 4; // u-blox 8/M8
        // fill in the SV's with some data even though firmware does not currently use it
        // note that this is not using num_sats as we aren't dynamically creating this to match
        for (uint8_t i = 0; i < SV_COUNT; i++) {
            svinfo.sv[i].chn = i;
            svinfo.sv[i].svid = i;
            svinfo.sv[i].flags = (i < _sitl->gps_numsats) ? 0x7 : 0x6; // sv used, diff correction data, orbit information
            svinfo.sv[i].quality = 7; // code and carrier lock and time synchronized
            svinfo.sv[i].cno = MAX(20, 30 - i);
            svinfo.sv[i].elev = MAX(30, 90 - i);
            svinfo.sv[i].azim = i;
            // not bothering to fill in prRes
        }
        _gps_send_ubx(MSG_SVINFO, (uint8_t*)&svinfo, sizeof(svinfo), instance);
        _next_nav_sv_info_time = time_week_ms + 10000; // 10 second delay
    }
}

/*
  MTK type simple checksum
 */
static void mtk_checksum(const uint8_t *data, uint8_t n, uint8_t *ck_a, uint8_t *ck_b)
{
    *ck_a = *ck_b = 0;
    while (n--) {
        *ck_a += *data++;
        *ck_b += *ck_a;
    }
}


/*
  send a new GPS MTK packet
 */
void SITL_State::_update_gps_mtk(const struct gps_data *d, uint8_t instance)
{
    struct PACKED mtk_msg {
        uint8_t preamble1;
        uint8_t preamble2;
        uint8_t msg_class;
        uint8_t msg_id;
        int32_t latitude;
        int32_t longitude;
        int32_t altitude;
        int32_t ground_speed;
        int32_t ground_course;
        uint8_t satellites;
        uint8_t fix_type;
        uint32_t utc_time;
        uint8_t ck_a;
        uint8_t ck_b;
    } p;

    p.preamble1     = 0xb5;
    p.preamble2     = 0x62;
    p.msg_class     = 1;
    p.msg_id        = 5;
    p.latitude      = htonl(d->latitude  * 1.0e6);
    p.longitude     = htonl(d->longitude * 1.0e6);
    p.altitude      = htonl(d->altitude * 100);
    p.ground_speed  = htonl(norm(d->speedN, d->speedE) * 100);
    p.ground_course = htonl(ToDeg(atan2f(d->speedE, d->speedN)) * 1000000.0f);
    if (p.ground_course < 0.0f) {
        p.ground_course += 360.0f * 1000000.0f;
    }
    p.satellites    = d->have_lock?_sitl->gps_numsats:3;
    p.fix_type      = d->have_lock?3:1;

    // the spec is not very clear, but the time field seems to be
    // milliseconds since the start of the day in UTC time,
    // done in powers of 100.
    // The date is powers of 100 as well, but in days since 1/1/2000
    struct tm tm;
    struct timeval tv;

    simulation_timeval(&tv);
    tm = *gmtime(&tv.tv_sec);
    uint32_t hsec = (tv.tv_usec / (10000*20)) * 20; // always multiple of 20

    p.utc_time = htonl(hsec + tm.tm_sec*100 + tm.tm_min*100*100 + tm.tm_hour*100*100*100);

    mtk_checksum(&p.msg_class, sizeof(p)-4, &p.ck_a, &p.ck_b);

    _gps_write((uint8_t*)&p, sizeof(p), instance);
}

/*
  send a new GPS MTK 1.6 packet
 */
void SITL_State::_update_gps_mtk16(const struct gps_data *d, uint8_t instance)
{
    struct PACKED mtk_msg {
        uint8_t preamble1;
        uint8_t preamble2;
        uint8_t size;
        int32_t latitude;
        int32_t longitude;
        int32_t altitude;
        int32_t ground_speed;
        int32_t ground_course;
        uint8_t satellites;
        uint8_t fix_type;
        uint32_t utc_date;
        uint32_t utc_time;
        uint16_t hdop;
        uint8_t ck_a;
        uint8_t ck_b;
    } p;

    p.preamble1     = 0xd0;
    p.preamble2     = 0xdd;
    p.size          = sizeof(p) - 5;
    p.latitude      = d->latitude  * 1.0e6;
    p.longitude     = d->longitude * 1.0e6;
    p.altitude      = d->altitude * 100;
    p.ground_speed  = norm(d->speedN, d->speedE) * 100;
    p.ground_course = ToDeg(atan2f(d->speedE, d->speedN)) * 100.0f;
    if (p.ground_course < 0.0f) {
        p.ground_course += 360.0f * 100.0f;
    }
    p.satellites    = d->have_lock?_sitl->gps_numsats:3;
    p.fix_type      = d->have_lock?3:1;

    // the spec is not very clear, but the time field seems to be
    // milliseconds since the start of the day in UTC time,
    // done in powers of 100.
    // The date is powers of 100 as well, but in days since 1/1/2000
    struct tm tm;
    struct timeval tv;

    simulation_timeval(&tv);
    tm = *gmtime(&tv.tv_sec);
    uint32_t millisec = (tv.tv_usec / (1000*200)) * 200; // always multiple of 200

    p.utc_date = (tm.tm_year-100) + ((tm.tm_mon+1)*100) + (tm.tm_mday*100*100);
    p.utc_time = millisec + tm.tm_sec*1000 + tm.tm_min*1000*100 + tm.tm_hour*1000*100*100;

    p.hdop          = 115;

    mtk_checksum(&p.size, sizeof(p)-4, &p.ck_a, &p.ck_b);

    _gps_write((uint8_t*)&p, sizeof(p), instance);
}

/*
  send a new GPS MTK 1.9 packet
 */
void SITL_State::_update_gps_mtk19(const struct gps_data *d, uint8_t instance)
{
    struct PACKED mtk_msg {
        uint8_t preamble1;
        uint8_t preamble2;
        uint8_t size;
        int32_t latitude;
        int32_t longitude;
        int32_t altitude;
        int32_t ground_speed;
        int32_t ground_course;
        uint8_t satellites;
        uint8_t fix_type;
        uint32_t utc_date;
        uint32_t utc_time;
        uint16_t hdop;
        uint8_t ck_a;
        uint8_t ck_b;
    } p;

    p.preamble1     = 0xd1;
    p.preamble2     = 0xdd;
    p.size          = sizeof(p) - 5;
    p.latitude      = d->latitude  * 1.0e7;
    p.longitude     = d->longitude * 1.0e7;
    p.altitude      = d->altitude * 100;
    p.ground_speed  = norm(d->speedN, d->speedE) * 100;
    p.ground_course = ToDeg(atan2f(d->speedE, d->speedN)) * 100.0f;
    if (p.ground_course < 0.0f) {
        p.ground_course += 360.0f * 100.0f;
    }
    p.satellites    = d->have_lock?_sitl->gps_numsats:3;
    p.fix_type      = d->have_lock?3:1;

    // the spec is not very clear, but the time field seems to be
    // milliseconds since the start of the day in UTC time,
    // done in powers of 100.
    // The date is powers of 100 as well, but in days since 1/1/2000
    struct tm tm;
    struct timeval tv;

    simulation_timeval(&tv);
    tm = *gmtime(&tv.tv_sec);
    uint32_t millisec = (tv.tv_usec / (1000*200)) * 200; // always multiple of 200

    p.utc_date = (tm.tm_year-100) + ((tm.tm_mon+1)*100) + (tm.tm_mday*100*100);
    p.utc_time = millisec + tm.tm_sec*1000 + tm.tm_min*1000*100 + tm.tm_hour*1000*100*100;

    p.hdop          = 115;

    mtk_checksum(&p.size, sizeof(p)-4, &p.ck_a, &p.ck_b);

    _gps_write((uint8_t*)&p, sizeof(p), instance);
}

/*
  NMEA checksum
 */
uint16_t SITL_State::_gps_nmea_checksum(const char *s)
{
    uint16_t cs = 0;
    const uint8_t *b = (const uint8_t *)s;
    for (uint16_t i=1; s[i]; i++) {
        cs ^= b[i];
    }
    return cs;
}

/*
  formatted print of NMEA message, with checksum appended
 */
void SITL_State::_gps_nmea_printf(uint8_t instance, const char *fmt, ...)
{
    char *s = nullptr;
    uint16_t csum;
    char trailer[6];

    va_list ap;

    va_start(ap, fmt);
    vasprintf(&s, fmt, ap);
    va_end(ap);
    csum = _gps_nmea_checksum(s);
    snprintf(trailer, sizeof(trailer), "*%02X\r\n", (unsigned)csum);
    _gps_write((const uint8_t*)s, strlen(s), instance);
    _gps_write((const uint8_t*)trailer, 5, instance);
    free(s);
}


/*
  send a new GPS NMEA packet
 */
void SITL_State::_update_gps_nmea(const struct gps_data *d, uint8_t instance)
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

    _gps_nmea_printf(instance, "$GPGGA,%s,%s,%s,%01d,%02d,%04.1f,%07.2f,M,0.0,M,,",
                     tstring,
                     lat_string,
                     lng_string,
                     d->have_lock?1:0,
                     d->have_lock?_sitl->gps_numsats:3,
                     2.0,
                     d->altitude);
    float speed_knots = norm(d->speedN, d->speedE) * M_PER_SEC_TO_KNOTS;

    float heading = ToDeg(atan2f(d->speedE, d->speedN));
    if (heading < 0) {
        heading += 360.0f;
    }

    //$GPVTG,133.18,T,120.79,M,0.11,N,0.20,K,A*24    
    _gps_nmea_printf(instance, "$GPVTG,%.2f,T,%.2f,M,%.2f,N,%.2f,K,A",
                     tstring,
                     heading,
                     heading,
                     speed_knots,
                     speed_knots * KNOTS_TO_METERS_PER_SECOND * 3.6);
    
    _gps_nmea_printf(instance, "$GPRMC,%s,%c,%s,%s,%.2f,%.2f,%s,,",
                     tstring,
                     d->have_lock?'A':'V',
                     lat_string,
                     lng_string,
                     speed_knots,
                     heading,
                     dstring);

    if (_sitl->gps_hdg_enabled) {
        _gps_nmea_printf(instance, "$GPHDT,%.2f,T", d->yaw);
    }
}

void SITL_State::_sbp_send_message(uint16_t msg_type, uint16_t sender_id, uint8_t len, uint8_t *payload, uint8_t instance)
{
    if (len != 0 && payload == 0) {
        return; //SBP_NULL_ERROR;
    }

    uint8_t preamble = 0x55;
    _gps_write(&preamble, 1, instance);
    _gps_write((uint8_t*)&msg_type, 2, instance);
    _gps_write((uint8_t*)&sender_id, 2, instance);
    _gps_write(&len, 1, instance);
    if (len > 0) {
        _gps_write((uint8_t*)payload, len, instance);
    }

    uint16_t crc;
    crc = crc16_ccitt((uint8_t*)&(msg_type), 2, 0);
    crc = crc16_ccitt((uint8_t*)&(sender_id), 2, crc);
    crc = crc16_ccitt(&(len), 1, crc);
    crc = crc16_ccitt(payload, len, crc);
    _gps_write((uint8_t*)&crc, 2, instance);
}

void SITL_State::_update_gps_sbp(const struct gps_data *d, uint8_t instance)
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
    _sbp_send_message(SBP_GPS_TIME_MSGTYPE, 0x2222, sizeof(t), (uint8_t*)&t, instance);

    if (!d->have_lock) {
        return;
    }

    pos.tow = time_week_ms;
    pos.lon = d->longitude;
    pos.lat= d->latitude;
    pos.height = d->altitude;
    pos.h_accuracy = 5e3;
    pos.v_accuracy = 10e3;
    pos.n_sats = _sitl->gps_numsats;

    // Send single point position solution
    pos.flags = 0;
    _sbp_send_message(SBP_POS_LLH_MSGTYPE, 0x2222, sizeof(pos), (uint8_t*)&pos, instance);
    // Send "pseudo-absolute" RTK position solution
    pos.flags = 1;
    _sbp_send_message(SBP_POS_LLH_MSGTYPE, 0x2222, sizeof(pos), (uint8_t*)&pos, instance);

    velned.tow = time_week_ms;
    velned.n = 1e3 * d->speedN;
    velned.e = 1e3 * d->speedE;
    velned.d = 1e3 * d->speedD;
    velned.h_accuracy = 5e3;
    velned.v_accuracy = 5e3;
    velned.n_sats = _sitl->gps_numsats;
    velned.flags = 0;
    _sbp_send_message(SBP_VEL_NED_MSGTYPE, 0x2222, sizeof(velned), (uint8_t*)&velned, instance);

    static uint32_t do_every_count = 0;
    if (do_every_count % 5 == 0) {

        dops.tow = time_week_ms;
        dops.gdop = 1;
        dops.pdop = 1;
        dops.tdop = 1;
        dops.hdop = 100;
        dops.vdop = 1;
        dops.flags = 1;
        _sbp_send_message(SBP_DOPS_MSGTYPE, 0x2222, sizeof(dops),
                          (uint8_t*)&dops, instance);

        hb.protocol_major = 0; //Sends protocol version 0
        _sbp_send_message(SBP_HEARTBEAT_MSGTYPE, 0x2222, sizeof(hb),
                          (uint8_t*)&hb, instance);

    }
    do_every_count++;
}


void SITL_State::_update_gps_sbp2(const struct gps_data *d, uint8_t instance)
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
    _sbp_send_message(SBP_GPS_TIME_MSGTYPE, 0x2222, sizeof(t), (uint8_t*)&t, instance);

    if (!d->have_lock) {
        return;
    }

    pos.tow = time_week_ms;
    pos.lon = d->longitude;
    pos.lat= d->latitude;
    pos.height = d->altitude;
    pos.h_accuracy = 5e3;
    pos.v_accuracy = 10e3;
    pos.n_sats = _sitl->gps_numsats;

    // Send single point position solution
    pos.flags = 1;
    _sbp_send_message(SBP_POS_LLH_MSGTYPE, 0x2222, sizeof(pos), (uint8_t*)&pos, instance);
    // Send "pseudo-absolute" RTK position solution
    pos.flags = 4;
    _sbp_send_message(SBP_POS_LLH_MSGTYPE, 0x2222, sizeof(pos), (uint8_t*)&pos, instance);

    velned.tow = time_week_ms;
    velned.n = 1e3 * d->speedN;
    velned.e = 1e3 * d->speedE;
    velned.d = 1e3 * d->speedD;
    velned.h_accuracy = 5e3;
    velned.v_accuracy = 5e3;
    velned.n_sats = _sitl->gps_numsats;
    velned.flags = 1;
    _sbp_send_message(SBP_VEL_NED_MSGTYPE, 0x2222, sizeof(velned), (uint8_t*)&velned, instance);

    static uint32_t do_every_count = 0;
    if (do_every_count % 5 == 0) {

        dops.tow = time_week_ms;
        dops.gdop = 1;
        dops.pdop = 1;
        dops.tdop = 1;
        dops.hdop = 100;
        dops.vdop = 1;
        dops.flags = 1;
        _sbp_send_message(SBP_DOPS_MSGTYPE, 0x2222, sizeof(dops),
                          (uint8_t*)&dops, instance);

        hb.protocol_major = 2; //Sends protocol version 2.0
        _sbp_send_message(SBP_HEARTBEAT_MSGTYPE, 0x2222, sizeof(hb),
                          (uint8_t*)&hb, instance);
    }
    do_every_count++;
}

void SITL_State::_update_gps_nova(const struct gps_data *d, uint8_t instance)
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
    _nova_send_message((uint8_t*)&header,sizeof(header),(uint8_t*)&psrdop, sizeof(psrdop), instance);
    
    
    header.messageid = 99;
    header.messagelength = sizeof(bestvel);
    header.sequence += 1;
    
    bestvel.horspd = norm(d->speedN, d->speedE);  
    bestvel.trkgnd = ToDeg(atan2f(d->speedE, d->speedN));
    bestvel.vertspd = -d->speedD;
    
    _nova_send_message((uint8_t*)&header,sizeof(header),(uint8_t*)&bestvel, sizeof(bestvel), instance);
    
    
    header.messageid = 42;
    header.messagelength = sizeof(bestpos);
    header.sequence += 1;
    
    bestpos.lat = d->latitude;
    bestpos.lng = d->longitude;
    bestpos.hgt = d->altitude;
    bestpos.svsused = _sitl->gps_numsats;
    bestpos.latsdev=0.2;
    bestpos.lngsdev=0.2;
    bestpos.hgtsdev=0.2;
    bestpos.solstat=0;
    bestpos.postype=32;
    
    _nova_send_message((uint8_t*)&header,sizeof(header),(uint8_t*)&bestpos, sizeof(bestpos), instance);
}

void SITL_State::_nova_send_message(uint8_t *header, uint8_t headerlength, uint8_t *payload, uint8_t payloadlen, uint8_t instance)
{
    _gps_write(header, headerlength, instance);
    _gps_write(payload, payloadlen, instance);

    uint32_t crc = CalculateBlockCRC32(headerlength, header, (uint32_t)0);
    crc = CalculateBlockCRC32(payloadlen, payload, crc);
    
    _gps_write((uint8_t*)&crc, 4, instance);
}

#define CRC32_POLYNOMIAL 0xEDB88320L
uint32_t SITL_State::CRC32Value(uint32_t icrc)
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

uint32_t SITL_State::CalculateBlockCRC32(uint32_t length, uint8_t *buffer, uint32_t crc)
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
void SITL_State::_update_gps_file(uint8_t instance)
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
        _gps_write((const uint8_t *)buf, ret, instance);
    }
    if (ret == 0) {
        ::printf("gps rewind\n");
        lseek(temp_fd, 0, SEEK_SET);
    }
}

/*
  possibly send a new GPS packet
 */
void SITL_State::_update_gps(double latitude, double longitude, float altitude,
                             double speedN, double speedE, double speedD,
                             double yaw, bool have_lock)
{
    char c;

    // simulate delayed lock times
    if (AP_HAL::millis() < _sitl->gps_lock_time*1000UL) {
        have_lock = false;
    }

    altitude += _sitl->gps_alt_offset;
    
    //Capture current position as basestation location for
    if (!_gps_has_basestation_position) {
        if (have_lock) {
            _gps_basestation_data.latitude = latitude;
            _gps_basestation_data.longitude = longitude;
            _gps_basestation_data.altitude = altitude;
            _gps_basestation_data.speedN = speedN;
            _gps_basestation_data.speedE = speedE;
            _gps_basestation_data.speedD = speedD;
            _gps_basestation_data.have_lock = have_lock;
            _gps_has_basestation_position = true;
        }
    }

    for (uint8_t idx=0; idx<2; idx++) {
        struct gps_data d;

        if (idx == 1 && !_sitl->gps2_enable) {
            continue;
        }

        // run at configured GPS rate (default 5Hz)
        if ((AP_HAL::millis() - gps_state[idx].last_update) < (uint32_t)(1000/_sitl->gps_hertz)) {
            continue;
        }

        // swallow any config bytes
        if (gps_state[idx].gps_fd != 0) {
            read(gps_state[idx].gps_fd, &c, 1);
        }

        gps_state[idx].last_update = AP_HAL::millis();

        d.latitude = latitude;
        d.longitude = longitude;
        d.yaw = yaw;
    
        // add an altitude error controlled by a slow sine wave
        d.altitude = altitude + _sitl->gps_noise * sinf(AP_HAL::millis() * 0.0005f);

        // Add offet to c.g. velocity to get velocity at antenna
        d.speedN = speedN;
        d.speedE = speedE;
        d.speedD = speedD;
        d.have_lock = have_lock;

        if (_sitl->gps_drift_alt > 0) {
            // slow altitude drift
            d.altitude += _sitl->gps_drift_alt*sinf(AP_HAL::millis()*0.001f*0.02f);
        }

        // correct the latitude, longitude, hiehgt and NED velocity for the offset between
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

        // add in some GPS lag
        uint8_t &next_index = gps_state[idx].next_index;
        uint8_t &delay = gps_state[idx].delay;
        _gps_data[idx][next_index++] = d;
        if (next_index >= delay+1) {
            next_index = 0;
        }

        d = _gps_data[idx][next_index];

        if (_sitl->gps_delay != delay) {
            // cope with updates to the delay control
            delay = _sitl->gps_delay;
            for (uint8_t i=0; i<delay; i++) {
                _gps_data[idx][i] = d;
            }
        }

        if (gps_state[idx].gps_fd == 0) {
            continue;
        }

        // Applying GPS glitch
        // Using first gps glitch
        Vector3f glitch_offsets = _sitl->gps_glitch[idx];
        d.latitude += glitch_offsets.x;
        d.longitude += glitch_offsets.y;
        d.altitude += glitch_offsets.z;

        if (gps_state[idx].gps_fd != 0) {
            _update_gps_instance((SITL::SITL::GPSType)_sitl->gps_type[idx].get(), &d, idx);
        }
    }
}

void SITL_State::_update_gps_instance(SITL::SITL::GPSType gps_type, const struct gps_data *data, uint8_t instance) {
    switch (gps_type) {
        case SITL::SITL::GPS_TYPE_NONE:
            // no GPS attached
            break;

        case SITL::SITL::GPS_TYPE_UBLOX:
            _update_gps_ubx(data, instance);
            break;

        case SITL::SITL::GPS_TYPE_MTK:
            _update_gps_mtk(data, instance);
            break;

        case SITL::SITL::GPS_TYPE_MTK16:
            _update_gps_mtk16(data, instance);
            break;

        case SITL::SITL::GPS_TYPE_MTK19:
            _update_gps_mtk19(data, instance);
            break;

        case SITL::SITL::GPS_TYPE_NMEA:
            _update_gps_nmea(data, instance);
            break;

        case SITL::SITL::GPS_TYPE_SBP:
            _update_gps_sbp(data, instance);
            break;

        case SITL::SITL::GPS_TYPE_SBP2:
            _update_gps_sbp2(data, instance);
            break;

        case SITL::SITL::GPS_TYPE_NOVA:
            _update_gps_nova(data, instance);
            break;

        case SITL::SITL::GPS_TYPE_FILE:
            _update_gps_file(instance);
            break;
    }
}

#endif
