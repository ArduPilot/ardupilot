/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
   Spektrum Telemetry library, based on AP_Frsky_Telem.cpp
   See https://www.spektrumrc.com/ProdInfo/Files/SPM_Telemetry_Developers_Specs.pdf
*/

#include "AP_Spektrum_Telem.h"
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Common/AP_FWVersion.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Common/Location.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <math.h>

#if HAL_SPEKTRUM_TELEM_ENABLED

#define MICROSEC_PER_MINUTE 60000000
#define MAX_TEXTGEN_LEN     13

//#define SPKT_DEBUG
#ifdef SPKT_DEBUG
# define debug(fmt, args...)	hal.console->printf("SPKT:" fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif

extern const AP_HAL::HAL& hal;

AP_Spektrum_Telem *AP_Spektrum_Telem::singleton;

AP_Spektrum_Telem::AP_Spektrum_Telem() : AP_RCTelemetry(0)
{
    singleton = this;
}

AP_Spektrum_Telem::~AP_Spektrum_Telem(void)
{
    singleton = nullptr;
}

bool AP_Spektrum_Telem::init(void)
{
    // sanity check that we are using a UART for RC input
    if (!AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_RCIN, 0)) {
        return false;
    }
    return AP_RCTelemetry::init();
}

/*
  setup ready for passthrough telem
 */
void AP_Spektrum_Telem::setup_wfq_scheduler(void)
{
    // initialize packet weights for the WFQ scheduler
    // priority[i] = 1/_scheduler.packet_weight[i]
    // rate[i] = LinkRate * ( priority[i] / (sum(priority[1-n])) )

    // Spektrum telemetry rate is 46Hz, so these rates must fit
    add_scheduler_entry(50, 100);   // qos        10Hz
    add_scheduler_entry(50, 100);   // rpm        10Hz
    add_scheduler_entry(50, 100);   // text,      10Hz
    add_scheduler_entry(50, 120);   // Attitude and compass 8Hz
    add_scheduler_entry(550, 280);  // GPS        3Hz
    add_scheduler_entry(550, 280);  // ESC        3Hz
    add_scheduler_entry(400, 250);  // altitude   4Hz
    add_scheduler_entry(400, 250);  // airspeed   4Hz
    add_scheduler_entry(700, 500);  // GPS status 2Hz
    add_scheduler_entry(1300, 500); // batt volt  2Hz
    add_scheduler_entry(1300, 500); // batt curr  2Hz
    add_scheduler_entry(1300, 500); // batt mah   2Hz
    add_scheduler_entry(1300, 500); // temp       2Hz
}

void AP_Spektrum_Telem::adjust_packet_weight(bool queue_empty)
{
    if (!queue_empty) {
        _scheduler.packet_weight[TEXT] = 50;         // messages
    } else {
        _scheduler.packet_weight[TEXT] = 5000;        // messages
    }
}

// WFQ scheduler
bool AP_Spektrum_Telem::is_packet_ready(uint8_t idx, bool queue_empty)
{
    bool packet_ready = false;
    switch (idx) {
    case TEXT:
        packet_ready = !queue_empty;
        break;
    default:
        packet_ready = true;
        break;
    }

    return packet_ready;
}

// WFQ scheduler
void AP_Spektrum_Telem::process_packet(uint8_t idx)
{
    // send packet
    switch (idx) {
        case QOS: // QOS
            calc_qos();
            break;
        case RPM: // RPM
            calc_rpm();
            break;
        case TEXT: // status text
            if (repeat_msg_chunk() || get_next_msg_chunk()) {
                send_msg_chunk(_msg_chunk);
            }
            break;
        case ATTITUDE: // Attitude and compass
            calc_attandmag();
            break;
        case GPS_LOC: // GPS location
            calc_gps_location();
            break;
        case ESC: // ESC
            calc_esc();
            break;
        case ALTITUDE: // altitude
            calc_altitude();
            break;
        case AIRSPEED: // airspeed
            calc_airspeed();
            break;
        case GPS_STATUS: // GPS status
            calc_gps_status();
            break;
        case VOLTAGE: // Battery volts
            calc_batt_volts(0);
            break;
        case AMPS: // Battery current
            calc_batt_amps(0);
            break;
        case MAH: // Battery current & mah
            calc_batt_mah();
            break;
        case TEMP: // temperature
            calc_temperature(0);
            break;
        default:
            break;
    }
}

// whether to repeat the last texgen output
bool AP_Spektrum_Telem::repeat_msg_chunk(void)
{
    if (_msg_chunk.repeats == 0) {
        return false;
    }

    // repeat each message chunk 3 times to ensure transmission
    // on slow links reduce the number of duplicate chunks
    uint8_t extra_chunks = 2;

    if (_scheduler.avg_packet_rate < 20) {
        extra_chunks = 0;
    } else if (_scheduler.avg_packet_rate < 30) {
        extra_chunks = 1;
    }

    if (_msg_chunk.repeats++ > extra_chunks) {
        _msg_chunk.repeats = 0;
        return false;
    }
    return true;
}


// grabs one "chunk" (13 bytes) of the queued message to be transmitted
bool AP_Spektrum_Telem::get_next_msg_chunk(void)
{
    _msg_chunk.repeats++;

    if (!_statustext.available) {
        WITH_SEMAPHORE(_statustext.sem);
        if (!_statustext.queue.pop(_statustext.next)) {
            return false;
        }

        _statustext.available = true;

        // We're going to display a new message so first clear the screen
        _msg_chunk.linenumber = 0xFF;
        _msg_chunk.char_index = 0;
        return true;
    }

    uint8_t character = 0;
    memset(_msg_chunk.chunk, 0, MAX_TEXTGEN_LEN);

    const uint8_t message_len = sizeof(_statustext.next.text);
    // the message fits in an entire line of text
    if (message_len < MAX_TEXTGEN_LEN) {
        memcpy(_msg_chunk.chunk, _statustext.next.text, message_len);
        _msg_chunk.linenumber = 0;
        _statustext.available = false;
        return true;
    }

    // a following part of multi-line text
    if (_msg_chunk.linenumber == 0xFF) {
        _msg_chunk.linenumber = 0;
    } else if (_msg_chunk.char_index > 0) {
        _msg_chunk.linenumber++;
    }

    // skip leading whitespace
    while (_statustext.next.text[_msg_chunk.char_index] == ' ' && _msg_chunk.char_index < message_len) {
        _msg_chunk.char_index++;
    }

    uint8_t space_idx = 0;
    const uint8_t begin_idx = _msg_chunk.char_index;
    // can't fit it all on one line so wrap at an appropriate place
    for (int i = 0; i < MAX_TEXTGEN_LEN && _msg_chunk.char_index < message_len; i++) {
        character = _statustext.next.text[_msg_chunk.char_index++];
        // split at the first ':'
        if (character == ':') {
            _msg_chunk.chunk[i] = 0;
            break;
        }
        // record the last space if we need to go back there
        if (character == ' ') {
            space_idx = _msg_chunk.char_index;
        }

        _msg_chunk.chunk[i] = character;

        if (!character) {
            break;
        }
    }
    // still not done, can we break at a word boundary?
    if (character != 0 && _msg_chunk.char_index < message_len && space_idx > 0) {
        _msg_chunk.char_index = space_idx;
        _msg_chunk.chunk[space_idx - begin_idx - 1] = 0;
    }

    // we've reached the end of the message (string terminated by '\0' or last character of the string has been processed)
    if (character == 0 || _msg_chunk.char_index == message_len) {
        _msg_chunk.char_index = 0; // reset index to get ready to process the next message
        _statustext.available = false;
    }

    return true;
}

// prepare qos data - mandatory frame that must be sent periodically
void AP_Spektrum_Telem::calc_qos()
{
    _telem.qos.identifier = TELE_DEVICE_QOS;
    _telem.qos.sID = 0;
    _telem.qos.A = 0xFFFF;
    _telem.qos.B = 0xFFFF;
    _telem.qos.L = 0xFFFF;
    _telem.qos.R = 0xFFFF;
    _telem.qos.F = 0xFFFF;
    _telem.qos.H = 0xFFFF;
    _telem.qos.rxVoltage = 0xFFFF;
    _telem_pending = true;
}

// prepare rpm data - B/E mandatory frame that must be sent periodically
void AP_Spektrum_Telem::calc_rpm()
{
#if AP_BATTERY_ENABLED
    const AP_BattMonitor &_battery = AP::battery();
#endif

    _telem.rpm.identifier = TELE_DEVICE_RPM;
    _telem.rpm.sID = 0;
    // battery voltage in centivolts, can have up to a 12S battery (4.25Vx12S = 51.0V)
#if AP_BATTERY_ENABLED
    _telem.rpm.volts = htobe16(((uint16_t)roundf(_battery.voltage(0) * 100.0f)));
#endif
    _telem.rpm.temperature = htobe16(int16_t(roundf(32.0f + AP::baro().get_temperature(0) * 9.0f / 5.0f)));
#if AP_RPM_ENABLED
    const AP_RPM *rpm = AP::rpm();
    float rpm_value;
    if (!rpm || !rpm->get_rpm(0, rpm_value) || rpm_value < 999.0f) {
        rpm_value = 999.0f;
    }
    _telem.rpm.microseconds = htobe16(uint16_t(roundf(MICROSEC_PER_MINUTE / rpm_value)));
    _telem.rpm.dBm_A = 0x7F;
    _telem.rpm.dBm_B = 0x7F;
#endif
    _telem_pending = true;
}

void AP_Spektrum_Telem::send_msg_chunk(const MessageChunk& chunk)
{
    memcpy(_telem.textgen.text, chunk.chunk, 13);
    _telem.textgen.identifier = TELE_DEVICE_TEXTGEN;
    _telem.textgen.lineNumber = chunk.linenumber;
    _telem.textgen.sID = 0;
    _telem_pending = true;
}

// prepare battery data - B/E but not supported by Spektrum
void AP_Spektrum_Telem::calc_batt_volts(uint8_t instance)
{
#if AP_BATTERY_ENABLED
    const AP_BattMonitor &_battery = AP::battery();

    // battery voltage in centivolts, can have up to a 12S battery (4.25Vx12S = 51.0V)
    _telem.hv.volts = htobe16(uint16_t(roundf(_battery.voltage(instance) * 100.0f)));
#endif
    _telem.hv.identifier = TELE_DEVICE_VOLTAGE;
    _telem.hv.sID = 0;
    _telem_pending = true;
}

// prepare battery data - B/E but not supported by Spektrum
void AP_Spektrum_Telem::calc_batt_amps(uint8_t instance)
{
#if AP_BATTERY_ENABLED
    const AP_BattMonitor &_battery = AP::battery();

    float current;
    if (!_battery.current_amps(current, instance)) {
        current = 0;
    }

    // Range: +/- 150A     Resolution: 300A / 2048 = 0.196791 A/count
    _telem.amps.current = htobe16(int16_t(roundf(current * 2048.0f / 300.0f)));
#endif
    _telem.amps.identifier = TELE_DEVICE_AMPS;
    _telem.amps.sID = 0;
    _telem_pending = true;
}

// prepare battery data - L/E
void AP_Spektrum_Telem::calc_batt_mah()
{
#if AP_BATTERY_ENABLED
    const AP_BattMonitor &_battery = AP::battery();
#endif

    _telem.fpMAH.identifier = TELE_DEVICE_FP_MAH;
    _telem.fpMAH.sID = 0;

#if AP_BATTERY_ENABLED
    float current;
    if (!_battery.current_amps(current, 0)) {
        current = 0;
    }
    _telem.fpMAH.current_A = int16_t(roundf(current * 10.0f));     // Instantaneous current, 0.1A (0-3276.6A)

    float used_mah;
    if (!_battery.consumed_mah(used_mah, 0)) {
        used_mah = 0;
    }
    _telem.fpMAH.chargeUsed_A = int16_t(roundf(used_mah));         // Integrated mAh used, 1mAh (0-32.766Ah)

    float temp;
    if (_battery.get_temperature(temp, 0)) {
         _telem.fpMAH.temp_A = uint16_t(roundf(temp * 10.0f));     // Temperature, 0.1C (0-150C, 0x7FFF indicates not populated)
    } else {
        _telem.fpMAH.temp_A = 0x7FFF;
    }

    if (!_battery.current_amps(current, 1)) {
        current = 0;
    }
    _telem.fpMAH.current_B = int16_t(roundf(current * 10.0f));     // Instantaneous current, 0.1A (0-3276.6A)

    if (!_battery.consumed_mah(used_mah, 1)) {
        used_mah = 0;
    }
    _telem.fpMAH.chargeUsed_B = int16_t(roundf(used_mah));         // Integrated mAh used, 1mAh (0-32.766Ah)

    if (_battery.get_temperature(temp, 1)) {
         _telem.fpMAH.temp_B = uint16_t(roundf(temp * 10.0f));     // Temperature, 0.1C (0-150C, 0x7FFF indicates not populated)
    } else {
        _telem.fpMAH.temp_B = 0x7FFF;
    }
#else
        _telem.fpMAH.temp_A = 0x7FFF;
        _telem.fpMAH.temp_B = 0x7FFF;
#endif

    _telem_pending = true;
}

// prepare temperature data - B/E but not supported by Spektrum
void AP_Spektrum_Telem::calc_temperature(uint8_t instance)
{
    _telem.temp.temperature = htobe16(int16_t(roundf(32.0f + AP::baro().get_temperature(instance) * 9.0f / 5.0f)));
    _telem.temp.identifier = TELE_DEVICE_TEMPERATURE;
    _telem.temp.sID = 0;
    _telem_pending = true;
}

// prepare altitude data - B/E
void AP_Spektrum_Telem::calc_altitude()
{
    _telem.alt.identifier = TELE_DEVICE_ALTITUDE;
    _telem.alt.sID = 0;

    AP_AHRS &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());

    float alt = 0;
    ahrs.get_relative_position_D_home(alt);
    alt = roundf(-alt * 10.0f);
    _telem.alt.altitude = htobe16(uint16_t(alt));            // .1m increments
    _max_alt = MAX(alt, _max_alt);
    _telem.alt.maxAltitude = htobe16(uint16_t(_max_alt));    // .1m increments
    _telem_pending = true;
}

// prepare airspeed data - B/E
void AP_Spektrum_Telem::calc_airspeed()
{
    _telem.speed.identifier = TELE_DEVICE_AIRSPEED;
    _telem.speed.sID = 0;

    AP_AHRS &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());

    float speed = 0.0f;
#if AP_AIRSPEED_ENABLED
    const AP_Airspeed *airspeed = AP::airspeed();
    if (airspeed && airspeed->healthy()) {
        speed = roundf(airspeed->get_airspeed() * 3.6);
    } else {
        speed = roundf(AP::ahrs().groundspeed() * 3.6);
    }
#else
    speed = roundf(AP::ahrs().groundspeed() * 3.6);
#endif

    _telem.speed.airspeed = htobe16(uint16_t(speed));           // 1 km/h increments
    _max_speed = MAX(speed, _max_speed);
    _telem.speed.maxAirspeed = htobe16(uint16_t(_max_speed));   // 1 km/h increments
    _telem_pending = true;
}

// prepare attitude and compass data - L/E
void AP_Spektrum_Telem::calc_attandmag(void)
{
    _telem.attMag.identifier = TELE_DEVICE_ATTMAG;
    _telem.attMag.sID = 0;

    AP_AHRS &_ahrs = AP::ahrs();
    WITH_SEMAPHORE(_ahrs.get_semaphore());

    // Attitude, 3 axes.  Roll is a rotation about the X Axis of the vehicle using the RHR.
    // Units are 0.1 deg - Pitch is a rotation about the Y Axis of the vehicle using the RHR.
    // Yaw is a rotation about the Z Axis of the vehicle using the RHR.
    _telem.attMag.attRoll = _ahrs.roll_sensor / 10;
    _telem.attMag.attPitch = _ahrs.pitch_sensor / 10;
    _telem.attMag.attYaw = _ahrs.yaw_sensor / 10;
    _telem.attMag.heading = (_ahrs.yaw_sensor / 10) % 3600;  // Heading, 0.1deg

    const Vector3f& field = AP::compass().get_field();

    _telem.attMag.magX = int16_t(roundf(field.x * 10.0f));             // Units are 0.1mG
    _telem.attMag.magY = int16_t(roundf(field.y * 10.0f));
    _telem.attMag.magZ = int16_t(roundf(field.z * 10.0f));
    _telem_pending = true;
}

// prepare gps location - L/E
void AP_Spektrum_Telem::calc_gps_location()
{
    const Location &loc = AP::gps().location(0); // use the first gps instance (same as in send_mavlink_gps_raw)
    const uint32_t u1e8 = 100000000, u1e7 = 10000000, u1e6 = 1000000, u1e5 = 100000, u1e4 = 10000;

    _telem.gpsloc.identifier = TELE_DEVICE_GPS_LOC;                 // Source device = 0x16
    _telem.gpsloc.sID = 0;                                          // Secondary ID
    uint32_t alt = (abs(loc.alt) / 10) % u1e6;
    _telem.gpsloc.altitudeLow = ((alt % u1e4 / 1000) << 12) | ((alt % 1000 / 100) << 8)
        | ((alt % 100 / 10) << 4) | (alt % 100); // BCD, meters, format 3.1 (Low order of altitude)

    const float lat = fabsf(loc.lat / 1.0e7f);                       // BCD, format 4.4, Degrees * 100 + minutes, less than 100 degrees
    const float lng = fabsf(loc.lng / 1.0e7f);                       // BCD, format 4.4 , Degrees * 100 + minutes, flag indicates > 99 degrees

    const uint32_t ulat = roundf((int32_t(lat) * 100.0f + (lat - int32_t(lat)) * 60.0f) * 10000.0f);
    const uint32_t ulng = roundf((int32_t(lng) * 100.0f + (lng - int32_t(lng)) * 60.0f) * 10000.0f);

    _telem.gpsloc.latitude = ((ulat % u1e8 / u1e7) << 28) | ((ulat % u1e7 / u1e6) << 24) | ((ulat % u1e6 / u1e5) << 20) | ((ulat % u1e5 / u1e4) << 16)
        | ((ulat % u1e4 / 1000) << 12) | ((ulat % 1000 / 100) << 8) | ((ulat % 100 / 10) << 4) | (ulat % 10);
    _telem.gpsloc.longitude = ((ulng % u1e8 / u1e7) << 28) | ((ulng % u1e7 / u1e6) << 24) | ((ulng % u1e6 / u1e5) << 20) | ((ulng % u1e5 / u1e4) << 16)
        | ((ulng % u1e4 / 1000) << 12) | ((ulng % 1000 / 100) << 8) | ((ulng % 100 / 10) << 4) | (ulng % 10);

    uint16_t course = uint16_t(roundf(AP::gps().ground_course() * 10.0f));
    _telem.gpsloc.course = ((course % u1e5 / u1e4) << 12) | ((course % u1e4 / 1000) << 8)  | ((course % 1000 / 100) << 4) | (course % 100 / 10);  // BCD, 3.1
    uint16_t hdop = AP::gps().get_hdop(0);
    _telem.gpsloc.HDOP = ((hdop % 1000 / 100) << 4) | (hdop % 100 / 10); // BCD, format 1.1
    _telem.gpsloc.GPSflags = 0;

    if (AP::gps().status(0) >= AP_GPS::GPS_OK_FIX_3D) {
        _telem.gpsloc.GPSflags |= GPS_INFO_FLAGS_3D_FIX;
    }
    if (loc.alt < 0) {
        _telem.gpsloc.GPSflags |= GPS_INFO_FLAGS_NEGATIVE_ALT;
    }
    if ((loc.lng / 1e7) > 99) {
        _telem.gpsloc.GPSflags |= GPS_INFO_FLAGS_LONGITUDE_GREATER_99;
    }
    if (loc.lat >= 0) {
        _telem.gpsloc.GPSflags |= GPS_INFO_FLAGS_IS_NORTH;
    }
    if (loc.lng >= 0) {
        _telem.gpsloc.GPSflags |= GPS_INFO_FLAGS_IS_EAST;
    }
    if (AP::gps().status(0) > AP_GPS::NO_FIX) {
        _telem.gpsloc.GPSflags |= GPS_INFO_FLAGS_GPS_FIX_VALID;
    }
    if (AP::gps().status(0) >= AP_GPS::NO_FIX) {
        _telem.gpsloc.GPSflags |= GPS_INFO_FLAGS_GPS_DATA_RECEIVED;
    }
    _telem_pending = true;
}

// prepare gps status - L/E
void AP_Spektrum_Telem::calc_gps_status()
{
    const Location &loc = AP::gps().location(0);

    _telem.gpsstat.identifier = TELE_DEVICE_GPS_STATS;              // Source device = 0x17
    _telem.gpsstat.sID = 0;                                         // Secondary ID

    uint16_t knots = roundf(AP::gps().ground_speed() * 1.94384f * 10.0f);
    _telem.gpsstat.speed = ((knots % 10000 / 1000) << 12) | ((knots % 1000 / 100) << 8) | ((knots % 100 / 10) << 4) | (knots % 10); // BCD, knots, format 3.1
    uint16_t ms;
    uint8_t h, m, s;
#if AP_RTC_ENABLED
    AP::rtc().get_system_clock_utc(h, m, s, ms);                    // BCD, format HH:MM:SS.S, format 6.1
    // FIXME: the above call can fail!
#else
    h = 0;
    m = 0;
    s = 0;
    ms = 0;
#endif
    _telem.gpsstat.UTC = ((((h / 10) << 4) | (h % 10)) << 20) | ((((m / 10) << 4) | (m % 10)) << 12) | ((((s / 10) << 4) | (s % 10)) << 4) | (ms / 100) ;
    uint8_t nsats =  AP::gps().num_sats();
    _telem.gpsstat.numSats = ((nsats / 10) << 4) | (nsats % 10);    // BCD, 0-99
    uint32_t alt = (abs(loc.alt) / 100000);
    _telem.gpsstat.altitudeHigh = ((alt / 10) << 4) | (alt % 10); // BCD, meters, format 2.0 (High order of altitude)
    _telem_pending = true;
}

// prepare ESC information - B/E
void AP_Spektrum_Telem::calc_esc()
{
#if HAL_WITH_ESC_TELEM
    const volatile AP_ESC_Telem_Backend::TelemetryData& td = AP::esc_telem().get_telem_data(0); // ideally should rotate between ESCs

	_telem.esc.identifier = TELE_DEVICE_ESC;	    // Source device = 0x20
	_telem.esc.sID = 0;									// Secondary ID
	_telem.esc.RPM = htobe16(uint16_t(roundf(AP::esc_telem().get_average_motor_frequency_hz() * 60)));	// Electrical RPM, 10RPM (0-655340 RPM)  0xFFFF --> "No data"
	_telem.esc.voltsInput = htobe16(td.voltage * 100);	    // Volts, 0.01v (0-655.34V)       0xFFFF --> "No data"
	_telem.esc.tempFET = htobe16(td.temperature_cdeg * 10);	// Temperature, 0.1C (0-6553.4C)  0xFFFF --> "No data"
	_telem.esc.currentMotor = htobe16(td.current * 100);		// Current, 10mA (0-655.34A)      0xFFFF --> "No data"
	_telem.esc.tempBEC = 0xFFFF;						// Temperature, 0.1C (0-6553.4C)  0xFFFF --> "No data"
	_telem.esc.currentBEC = 0xFF;						// BEC Current, 100mA (0-25.4A)   0xFF ----> "No data"
	_telem.esc.voltsBEC = 0xFF;							// BEC Volts, 0.05V (0-12.70V)    0xFF ----> "No data"
	_telem.esc.throttle = 0xFF;							// 0.5% (0-100%)                  0xFF ----> "No data"
	_telem.esc.powerOut = 0xFF;							// Power Output, 0.5% (0-127%)    0xFF ----> "No data"
    _telem_pending = true;
#endif
}

/*
  fetch Spektrum data for an external transport, such as SRXL2
 */
bool AP_Spektrum_Telem::_get_telem_data(uint8_t* data)
{
    memset(&_telem, 0, 16);
    run_wfq_scheduler();
    if (!_telem_pending) {
        return false;
    }
    memcpy(data, &_telem, 16);
    _telem_pending = false;
    return true;
}

/*
  fetch data for an external transport, such as SRXL2
 */
bool AP_Spektrum_Telem::get_telem_data(uint8_t* data)
{
    if (!singleton && !hal.util->get_soft_armed()) {
        // if telem data is requested when we are disarmed and don't
        // yet have a AP_Spektrum_Telem object then try to allocate one
        new AP_Spektrum_Telem();
        // initialize the passthrough scheduler
        if (singleton) {
            singleton->init();
        }
    }
    if (!singleton) {
        return false;
    }
    return singleton->_get_telem_data(data);
}

namespace AP {
    AP_Spektrum_Telem *spektrum_telem() {
        return AP_Spektrum_Telem::get_singleton();
    }
};

#endif
