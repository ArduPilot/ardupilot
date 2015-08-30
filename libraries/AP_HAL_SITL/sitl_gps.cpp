// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

static uint8_t next_gps_index;
static uint8_t gps_delay;

// state of GPS emulation
static struct gps_state {
    /* pipe emulating UBLOX GPS serial stream */
    int gps_fd, client_fd;
    uint32_t last_update; // milliseconds
} gps_state, gps2_state;

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
int SITL_State::gps_pipe(void)
{
    int fd[2];
    if (gps_state.client_fd != 0) {
        return gps_state.client_fd;
    }
    pipe(fd);
    gps_state.gps_fd    = fd[1];
    gps_state.client_fd = fd[0];
    gps_state.last_update = _scheduler->millis();
    HALSITL::SITLUARTDriver::_set_nonblocking(gps_state.gps_fd);
    HALSITL::SITLUARTDriver::_set_nonblocking(fd[0]);
    return gps_state.client_fd;
}

/*
  setup GPS2 input pipe
 */
int SITL_State::gps2_pipe(void)
{
    int fd[2];
    if (gps2_state.client_fd != 0) {
        return gps2_state.client_fd;
    }
    pipe(fd);
    gps2_state.gps_fd    = fd[1];
    gps2_state.client_fd = fd[0];
    gps2_state.last_update = _scheduler->millis();
    HALSITL::SITLUARTDriver::_set_nonblocking(gps2_state.gps_fd);
    HALSITL::SITLUARTDriver::_set_nonblocking(fd[0]);
    return gps2_state.client_fd;
}

/*
  write some bytes from the simulated GPS
 */
void SITL_State::_gps_write(const uint8_t *p, uint16_t size)
{
    while (size--) {
        if (_sitl->gps_byteloss > 0.0f) {
            float r = ((((unsigned)random()) % 1000000)) / 1.0e4;
            if (r < _sitl->gps_byteloss) {
                // lose the byte
                p++;
                continue;
            }
        }
        write(gps_state.gps_fd, p, 1);
        if (_sitl->gps2_enable) {
            write(gps2_state.gps_fd, p, 1);
        }
        p++;
    }
}

/*
  send a UBLOX GPS message
 */
void SITL_State::_gps_send_ubx(uint8_t msgid, uint8_t *buf, uint16_t size)
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
    _gps_write(hdr, sizeof(hdr));
    _gps_write(buf, size);
    _gps_write(chk, sizeof(chk));
}

/*
  return GPS time of week in milliseconds
 */
static void gps_time(uint16_t *time_week, uint32_t *time_week_ms)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    const uint32_t epoch = 86400*(10*365 + (1980-1969)/4 + 1 + 6 - 2) - 15;
    uint32_t epoch_seconds = tv.tv_sec - epoch;
    *time_week = epoch_seconds / (86400*7UL);
    *time_week_ms = (epoch_seconds % (86400*7UL))*1000 + tv.tv_usec/1000;
}

/*
  send a new set of GPS UBLOX packets
 */
void SITL_State::_update_gps_ubx(const struct gps_data *d)
{
    struct PACKED ubx_nav_posllh {
        uint32_t	time; // GPS msToW
        int32_t		longitude;
        int32_t		latitude;
        int32_t		altitude_ellipsoid;
        int32_t		altitude_msl;
        uint32_t	horizontal_accuracy;
        uint32_t	vertical_accuracy;
    } pos;
    struct PACKED ubx_nav_status {
        uint32_t	time;				// GPS msToW
        uint8_t		fix_type;
        uint8_t		fix_status;
        uint8_t		differential_status;
        uint8_t		res;
        uint32_t	time_to_first_fix;
        uint32_t	uptime;				// milliseconds
    } status;
    struct PACKED ubx_nav_velned {
        uint32_t	time;				// GPS msToW
        int32_t		ned_north;
        int32_t		ned_east;
        int32_t		ned_down;
        uint32_t	speed_3d;
        uint32_t	speed_2d;
        int32_t		heading_2d;
        uint32_t	speed_accuracy;
        uint32_t	heading_accuracy;
    } velned;
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
    } sol;
    struct PACKED ubx_nav_dop {
        uint32_t time;                                  // GPS msToW
        uint16_t gDOP;
        uint16_t pDOP;
        uint16_t tDOP;
        uint16_t vDOP;
        uint16_t hDOP;
        uint16_t nDOP;
        uint16_t eDOP;
    } dop;
    const uint8_t MSG_POSLLH = 0x2;
    const uint8_t MSG_STATUS = 0x3;
    const uint8_t MSG_DOP = 0x4;
    const uint8_t MSG_VELNED = 0x12;
    const uint8_t MSG_SOL = 0x6;
    uint16_t time_week;
    uint32_t time_week_ms;

    gps_time(&time_week, &time_week_ms);

    pos.time = time_week_ms;
    pos.longitude = d->longitude * 1.0e7;
    pos.latitude  = d->latitude * 1.0e7;
    pos.altitude_ellipsoid = d->altitude*1000.0f;
    pos.altitude_msl = d->altitude*1000.0f;
    pos.horizontal_accuracy = 1500;
    pos.vertical_accuracy = 2000;

    status.time = time_week_ms;
    status.fix_type = d->have_lock?3:0;
    status.fix_status = d->have_lock?1:0;
    status.differential_status = 0;
    status.res = 0;
    status.time_to_first_fix = 0;
    status.uptime = hal.scheduler->millis();

    velned.time = time_week_ms;
    velned.ned_north = 100.0f * d->speedN;
    velned.ned_east  = 100.0f * d->speedE;
    velned.ned_down  = 100.0f * d->speedD;
    velned.speed_2d = pythagorous2(d->speedN, d->speedE) * 100;
    velned.speed_3d = pythagorous3(d->speedN, d->speedE, d->speedD) * 100;
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

    _gps_send_ubx(MSG_POSLLH, (uint8_t*)&pos, sizeof(pos));
    _gps_send_ubx(MSG_STATUS, (uint8_t*)&status, sizeof(status));
    _gps_send_ubx(MSG_VELNED, (uint8_t*)&velned, sizeof(velned));
    _gps_send_ubx(MSG_SOL,    (uint8_t*)&sol, sizeof(sol));
    _gps_send_ubx(MSG_DOP,    (uint8_t*)&dop, sizeof(dop));
}

static void swap_uint32(uint32_t *v, uint8_t n)
{
    while (n--) {
        *v = htonl(*v);
        v++;
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
void SITL_State::_update_gps_mtk(const struct gps_data *d)
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
    p.latitude      = d->latitude  * 1.0e6;
    p.longitude     = d->longitude * 1.0e6;
    p.altitude      = d->altitude * 100;
    p.ground_speed  = pythagorous2(d->speedN, d->speedE) * 100;
    p.ground_course = ToDeg(atan2f(d->speedE, d->speedN)) * 1000000.0f;
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

    gettimeofday(&tv, NULL);
    tm = *gmtime(&tv.tv_sec);
    uint32_t hsec = (tv.tv_usec / (10000*20)) * 20; // always multiple of 20

    p.utc_time = hsec + tm.tm_sec*100 + tm.tm_min*100*100 + tm.tm_hour*100*100*100;

    swap_uint32((uint32_t *)&p.latitude, 5);
    swap_uint32((uint32_t *)&p.utc_time, 1);
    mtk_checksum(&p.msg_class, sizeof(p)-4, &p.ck_a, &p.ck_b);

    _gps_write((uint8_t*)&p, sizeof(p));
}

/*
  send a new GPS MTK 1.6 packet
 */
void SITL_State::_update_gps_mtk16(const struct gps_data *d)
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
    p.ground_speed  = pythagorous2(d->speedN, d->speedE) * 100;
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

    gettimeofday(&tv, NULL);
    tm = *gmtime(&tv.tv_sec);
    uint32_t millisec = (tv.tv_usec / (1000*200)) * 200; // always multiple of 200

    p.utc_date = (tm.tm_year-100) + ((tm.tm_mon+1)*100) + (tm.tm_mday*100*100);
    p.utc_time = millisec + tm.tm_sec*1000 + tm.tm_min*1000*100 + tm.tm_hour*1000*100*100;

    p.hdop          = 115;

    mtk_checksum(&p.size, sizeof(p)-4, &p.ck_a, &p.ck_b);

    _gps_write((uint8_t*)&p, sizeof(p));
}

/*
  send a new GPS MTK 1.9 packet
 */
void SITL_State::_update_gps_mtk19(const struct gps_data *d)
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
    p.ground_speed  = pythagorous2(d->speedN, d->speedE) * 100;
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

    gettimeofday(&tv, NULL);
    tm = *gmtime(&tv.tv_sec);
    uint32_t millisec = (tv.tv_usec / (1000*200)) * 200; // always multiple of 200

    p.utc_date = (tm.tm_year-100) + ((tm.tm_mon+1)*100) + (tm.tm_mday*100*100);
    p.utc_time = millisec + tm.tm_sec*1000 + tm.tm_min*1000*100 + tm.tm_hour*1000*100*100;

    p.hdop          = 115;

    mtk_checksum(&p.size, sizeof(p)-4, &p.ck_a, &p.ck_b);

    _gps_write((uint8_t*)&p, sizeof(p));
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
  formated print of NMEA message, with checksum appended
 */
void SITL_State::_gps_nmea_printf(const char *fmt, ...)
{
    char *s = NULL;
    uint16_t csum;
    char trailer[6];

    va_list ap;

    va_start(ap, fmt);
    vasprintf(&s, fmt, ap);
    va_end(ap);
    csum = _gps_nmea_checksum(s);
    snprintf(trailer, sizeof(trailer), "*%02X\r\n", (unsigned)csum);
    _gps_write((const uint8_t*)s, strlen(s));
    _gps_write((const uint8_t*)trailer, 5);
    free(s);
}


/*
  send a new GPS NMEA packet
 */
void SITL_State::_update_gps_nmea(const struct gps_data *d)
{
    struct timeval tv;
    struct tm *tm;
    char tstring[20];
    char dstring[20];
    char lat_string[20];
    char lng_string[20];

    gettimeofday(&tv, NULL);

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

    _gps_nmea_printf("$GPGGA,%s,%s,%s,%01d,%02d,%04.1f,%07.2f,M,0.0,M,,",
                     tstring,
                     lat_string,
                     lng_string,
                     d->have_lock?1:0,
                     d->have_lock?_sitl->gps_numsats:3,
                     2.0,
                     d->altitude);
    float speed_knots = pythagorous2(d->speedN, d->speedE)*1.94384449f;
    float heading = ToDeg(atan2f(d->speedE, d->speedN));
    if (heading < 0) {
        heading += 360.0f;
    }
    _gps_nmea_printf("$GPRMC,%s,%c,%s,%s,%.2f,%.2f,%s,,",
                     tstring,
                     d->have_lock?'A':'V',
                     lat_string,
                     lng_string,
                     speed_knots,
                     heading,
                     dstring);
}

void SITL_State::_sbp_send_message(uint16_t msg_type, uint16_t sender_id, uint8_t len, uint8_t *payload)
{
    if (len != 0 && payload == 0) {
        return; //SBP_NULL_ERROR;
    }

    uint8_t preamble = 0x55;
    _gps_write(&preamble, 1);
    _gps_write((uint8_t*)&msg_type, 2);
    _gps_write((uint8_t*)&sender_id, 2);
    _gps_write(&len, 1);
    if (len > 0) {
        _gps_write((uint8_t*)payload, len);
    }

    uint16_t crc;
    crc = crc16_ccitt((uint8_t*)&(msg_type), 2, 0);
    crc = crc16_ccitt((uint8_t*)&(sender_id), 2, crc);
    crc = crc16_ccitt(&(len), 1, crc);
    crc = crc16_ccitt(payload, len, crc);
    _gps_write((uint8_t*)&crc, 2);
}


void SITL_State::_update_gps_sbp(const struct gps_data *d)
{
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
    _sbp_send_message(SBP_GPS_TIME_MSGTYPE, 0x2222, sizeof(t), (uint8_t*)&t);

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
    _sbp_send_message(SBP_POS_LLH_MSGTYPE, 0x2222, sizeof(pos), (uint8_t*)&pos);
    // Send "pseudo-absolute" RTK position solution
    pos.flags = 1;
    _sbp_send_message(SBP_POS_LLH_MSGTYPE, 0x2222, sizeof(pos), (uint8_t*)&pos);

    velned.tow = time_week_ms;
    velned.n = 1e3 * d->speedN;
    velned.e = 1e3 * d->speedE;
    velned.d = 1e3 * d->speedD;
    velned.h_accuracy = 5e3;
    velned.v_accuracy = 5e3;
    velned.n_sats = _sitl->gps_numsats;
    velned.flags = 0;
    _sbp_send_message(SBP_VEL_NED_MSGTYPE, 0x2222, sizeof(velned), (uint8_t*)&velned);

    static uint32_t do_every_count = 0;
    if (do_every_count % 5 == 0) {

        dops.tow = time_week_ms;
        dops.gdop = 1;
        dops.pdop = 1;
        dops.tdop = 1;
        dops.hdop = 100;
        dops.vdop = 1;
        _sbp_send_message(SBP_DOPS_MSGTYPE, 0x2222, sizeof(dops),
                          (uint8_t*)&dops);

        uint32_t system_flags = 0;
        _sbp_send_message(SBP_HEARTBEAT_MSGTYPE, 0x2222,
                          sizeof(system_flags),
                          (uint8_t*)&system_flags);

    }
    do_every_count++;
}


/*
  temporary method to use file as GPS data
 */
void SITL_State::_update_gps_file(const struct gps_data *d)
{
    static int fd = -1;
    if (fd == -1) {
        fd = open("/tmp/gps.dat", O_RDONLY);
    }
    if (fd == -1) {
        return;
    }
    char buf[200];
    ssize_t ret = ::read(fd, buf, sizeof(buf));
    if (ret > 0) {
        ::printf("wrote gps %u bytes\n", (unsigned)ret);
        _gps_write((const uint8_t *)buf, ret);
    }
    if (ret == 0) {
        ::printf("gps rewind\n");
        lseek(fd, 0, SEEK_SET);
    }
}

/*
  possibly send a new GPS packet
 */
void SITL_State::_update_gps(double latitude, double longitude, float altitude,
                             double speedN, double speedE, double speedD, bool have_lock)
{
    struct gps_data d;
    char c;
    Vector3f glitch_offsets = _sitl->gps_glitch;

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

    // run at configured GPS rate (default 5Hz)
    if ((hal.scheduler->millis() - gps_state.last_update) < (uint32_t)(1000/_sitl->gps_hertz)) {
        return;
    }

    // swallow any config bytes
    if (gps_state.gps_fd != 0) {
        read(gps_state.gps_fd, &c, 1);
    }
    if (gps2_state.gps_fd != 0) {
        read(gps2_state.gps_fd, &c, 1);
    }

    gps_state.last_update = hal.scheduler->millis();
    gps2_state.last_update = hal.scheduler->millis();

    d.latitude = latitude + glitch_offsets.x;
    d.longitude = longitude + glitch_offsets.y;
    d.altitude = altitude + glitch_offsets.z;

    if (_sitl->gps_drift_alt > 0) {
        // slow altitude drift
        d.altitude += _sitl->gps_drift_alt*sinf(hal.scheduler->millis()*0.001f*0.02f);
    }

    d.speedN = speedN;
    d.speedE = speedE;
    d.speedD = speedD;
    d.have_lock = have_lock;

    // add in some GPS lag
    _gps_data[next_gps_index++] = d;
    if (next_gps_index >= gps_delay+1) {
        next_gps_index = 0;
    }

    d = _gps_data[next_gps_index];

    if (_sitl->gps_delay != gps_delay) {
        // cope with updates to the delay control
        gps_delay = _sitl->gps_delay;
        for (uint8_t i=0; i<gps_delay; i++) {
            _gps_data[i] = d;
        }
    }

    if (gps_state.gps_fd == 0 && gps2_state.gps_fd == 0) {
        return;
    }

    switch ((SITL::GPSType)_sitl->gps_type.get()) {
    case SITL::GPS_TYPE_NONE:
        // no GPS attached
        break;

    case SITL::GPS_TYPE_UBLOX:
        _update_gps_ubx(&d);
        break;

    case SITL::GPS_TYPE_MTK:
        _update_gps_mtk(&d);
        break;

    case SITL::GPS_TYPE_MTK16:
        _update_gps_mtk16(&d);
        break;

    case SITL::GPS_TYPE_MTK19:
        _update_gps_mtk19(&d);
        break;

    case SITL::GPS_TYPE_NMEA:
        _update_gps_nmea(&d);
        break;

    case SITL::GPS_TYPE_SBP:
        _update_gps_sbp(&d);
        break;

    case SITL::GPS_TYPE_FILE:
        _update_gps_file(&d);
        break;

    }
}

#endif
