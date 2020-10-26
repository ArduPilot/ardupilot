#include "AP_Frsky_SPort_Passthrough.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <GCS_MAVLink/GCS.h>

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
#include "AP_Frsky_MAVlite.h"
#include "AP_Frsky_Parameters.h"
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL

/*
for FrSky SPort Passthrough
*/
// data bits preparation
// for parameter data
#define PARAM_ID_OFFSET             24
#define PARAM_VALUE_LIMIT           0xFFFFFF
// for gps status data
#define GPS_SATS_LIMIT              0xF
#define GPS_STATUS_LIMIT            0x3
#define GPS_STATUS_OFFSET           4
#define GPS_HDOP_OFFSET             6
#define GPS_ADVSTATUS_OFFSET        14
#define GPS_ALTMSL_OFFSET           22
// for battery data
#define BATT_VOLTAGE_LIMIT          0x1FF
#define BATT_CURRENT_OFFSET         9
#define BATT_TOTALMAH_LIMIT         0x7FFF
#define BATT_TOTALMAH_OFFSET        17
// for autopilot status data
#define AP_CONTROL_MODE_LIMIT       0x1F
#define AP_SIMPLE_OFFSET            5
#define AP_SSIMPLE_OFFSET           6
#define AP_FLYING_OFFSET            7
#define AP_ARMED_OFFSET             8
#define AP_BATT_FS_OFFSET           9
#define AP_EKF_FS_OFFSET            10
#define AP_IMU_TEMP_MIN             19.0f
#define AP_IMU_TEMP_MAX             82.0f
#define AP_IMU_TEMP_OFFSET          26
// for home position related data
#define HOME_ALT_OFFSET             12
#define HOME_BEARING_LIMIT          0x7F
#define HOME_BEARING_OFFSET         25
// for velocity and yaw data
#define VELANDYAW_XYVEL_OFFSET      9
#define VELANDYAW_YAW_LIMIT         0x7FF
#define VELANDYAW_YAW_OFFSET        17
// for attitude (roll, pitch) and range data
#define ATTIANDRNG_ROLL_LIMIT       0x7FF
#define ATTIANDRNG_PITCH_LIMIT      0x3FF
#define ATTIANDRNG_PITCH_OFFSET     11
#define ATTIANDRNG_RNGFND_OFFSET    21

extern const AP_HAL::HAL& hal;

bool AP_Frsky_SPort_Passthrough::init()
{
    if (!AP_RCTelemetry::init()) {
        return false;
    }
    return AP_Frsky_SPort::init();
}

bool AP_Frsky_SPort_Passthrough::init_serial_port()
{
    if (_use_external_data) {
        return true;
    }
    return AP_Frsky_SPort::init_serial_port();
}

void  AP_Frsky_SPort_Passthrough::send_sport_frame(uint8_t frame, uint16_t appid, uint32_t data)
{
    if (_use_external_data) {
        external_data.frame = frame;
        external_data.appid = appid;
        external_data.data = data;
        external_data.pending = true;
        return;
    }

    return AP_Frsky_SPort::send_sport_frame(frame, appid, data);
}

/*
  setup ready for passthrough telem
 */
void AP_Frsky_SPort_Passthrough::setup_wfq_scheduler(void)
{
    // initialize packet weights for the WFQ scheduler
    // priority[i] = 1/_scheduler.packet_weight[i]
    // rate[i] = LinkRate * ( priority[i] / (sum(priority[1-n])) )
    set_scheduler_entry(TEXT, 35, 28);          // 0x5000 status text (dynamic)
    set_scheduler_entry(ATTITUDE, 50, 38);      // 0x5006 Attitude and range (dynamic)
    set_scheduler_entry(GPS_LAT, 550, 280);     // 0x800 GPS lat
    set_scheduler_entry(GPS_LON, 550, 280);     // 0x800 GPS lon
    set_scheduler_entry(VEL_YAW, 400, 250);     // 0x5005 Vel and Yaw
    set_scheduler_entry(AP_STATUS, 700, 500);   // 0x5001 AP status
    set_scheduler_entry(GPS_STATUS, 700, 500);  // 0x5002 GPS status
    set_scheduler_entry(HOME, 400, 500);        // 0x5004 Home
    set_scheduler_entry(BATT_2, 1300, 500);     // 0x5008 Battery 2 status
    set_scheduler_entry(BATT_1, 1300, 500);     // 0x5003 Battery 1 status
    set_scheduler_entry(PARAM, 1700, 1000);     // 0x5007 parameters
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    set_scheduler_entry(MAV, 35, 25);           // mavlite
    // initialize sport sensor IDs
    set_sensor_id(_frsky_parameters->_uplink_id, _SPort_bidir.uplink_sensor_id);
    set_sensor_id(_frsky_parameters->_dnlink1_id, _SPort_bidir.downlink1_sensor_id);
    set_sensor_id(_frsky_parameters->_dnlink2_id, _SPort_bidir.downlink2_sensor_id);
    // initialize sport
    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_Frsky_SPort_Passthrough::process_rx_queue, void));
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
}

/*
  dynamically change scheduler priorities based on queue sizes
*/
void AP_Frsky_SPort_Passthrough::adjust_packet_weight(bool queue_empty)
{
    /*
        When queues are empty set a low priority (high weight), when queues
        are not empty set a higher priority (low weight) based on the following
        relative priority order: mavlite > status text > attitude.
     */
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    if (!_SPort_bidir.tx_packet_queue.is_empty()) {
        _scheduler.packet_weight[MAV] = 30;             // mavlite
        if (!queue_empty) {
            _scheduler.packet_weight[TEXT] = 45;        // messages
            _scheduler.packet_weight[ATTITUDE] = 80;    // attitude
        } else {
            _scheduler.packet_weight[TEXT] = 5000;      // messages
            _scheduler.packet_weight[ATTITUDE] = 80;    // attitude
        }
    } else {
        _scheduler.packet_weight[MAV] = 5000;           // mavlite
        if (!queue_empty) {
            _scheduler.packet_weight[TEXT] = 45;        // messages
            _scheduler.packet_weight[ATTITUDE] = 80;    // attitude
        } else {
            _scheduler.packet_weight[TEXT] = 5000;      // messages
            _scheduler.packet_weight[ATTITUDE] = 45;    // attitude
        }
    }
#else   //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    if (!queue_empty) {
        _scheduler.packet_weight[TEXT] = 45;     // messages
        _scheduler.packet_weight[ATTITUDE] = 80;     // attitude
    } else {
        _scheduler.packet_weight[TEXT] = 5000;   // messages
        _scheduler.packet_weight[ATTITUDE] = 45;     // attitude
    }
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
}

// WFQ scheduler
bool AP_Frsky_SPort_Passthrough::is_packet_ready(uint8_t idx, bool queue_empty)
{
    bool packet_ready = false;
    switch (idx) {
    case TEXT:
        packet_ready = !queue_empty;
        break;
    case GPS_LAT:
    case GPS_LON:
        // force gps coords to use sensor 0x1B, always send when used with external data
        packet_ready = _use_external_data || (_passthrough.new_byte == SENSOR_ID_27);
        break;
    case AP_STATUS:
        packet_ready = gcs().vehicle_initialised();
        break;
    case BATT_2:
        packet_ready = AP::battery().num_instances() > 1;
        break;
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
        case MAV:
            packet_ready = !_SPort_bidir.tx_packet_queue.is_empty();
            break;
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    default:
        packet_ready = true;
        break;
    }

    return packet_ready;
}

/*
 * WFQ scheduler
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_SPort_Passthrough::process_packet(uint8_t idx)
{
    // send packet
    switch (idx) {
    case TEXT: // 0x5000 status text
        if (get_next_msg_chunk()) {
            send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID, _msg_chunk.chunk);
        }
        break;
    case ATTITUDE: // 0x5006 Attitude and range
        send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+6, calc_attiandrng());
        break;
    case GPS_LAT: // 0x800 GPS lat
        // sample both lat and lon at the same time
        send_sport_frame(SPORT_DATA_FRAME, GPS_LONG_LATI_FIRST_ID, calc_gps_latlng(_passthrough.send_latitude)); // gps latitude or longitude
        _passthrough.gps_lng_sample = calc_gps_latlng(_passthrough.send_latitude);
        // force the scheduler to select GPS lon as packet that's been waiting the most
            // this guarantees that lat and lon are sent as consecutive packets
        _scheduler.packet_timer[GPS_LON] = _scheduler.packet_timer[GPS_LAT] - 10000;
        break;
    case GPS_LON: // 0x800 GPS lon
        send_sport_frame(SPORT_DATA_FRAME, GPS_LONG_LATI_FIRST_ID, _passthrough.gps_lng_sample); // gps longitude
        break;
    case VEL_YAW: // 0x5005 Vel and Yaw
        send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+5, calc_velandyaw());
        break;
    case AP_STATUS: // 0x5001 AP status
        send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+1, calc_ap_status());
        break;
    case GPS_STATUS: // 0x5002 GPS Status
        send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+2, calc_gps_status());
        break;
    case HOME: // 0x5004 Home
        send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+4, calc_home());
        break;
    case BATT_2: // 0x5008 Battery 2 status
        send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+8, calc_batt(1));
        break;
    case BATT_1: // 0x5003 Battery 1 status
        send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+3, calc_batt(0));
        break;
    case PARAM: // 0x5007 parameters
        send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+7, calc_param());
        break;
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    case MAV: // mavlite
        process_tx_queue();
        break;
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    }
}

/*
 * send telemetry data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_SPort_Passthrough::send(void)
{
    const uint16_t numc = MIN(_port->available(), 1024U);

    // this is the constant for hub data frame
    if (_port->txspace() < 19) {
        return;
    }
    // keep only the last two bytes of the data found in the serial buffer, as we shouldn't respond to old poll requests
    uint8_t prev_byte = 0;
    for (uint16_t i = 0; i < numc; i++) {
        prev_byte = _passthrough.new_byte;
        _passthrough.new_byte = _port->read();
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
        AP_Frsky_SPort::sport_packet_t sp;

        if (_sport_handler.process_byte(sp, _passthrough.new_byte)) {
            queue_rx_packet(sp);
        }
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    }
    // check if we should respond to this polling byte
    if (prev_byte == FRAME_HEAD && is_passthrough_byte(_passthrough.new_byte)) {
        run_wfq_scheduler();
    }
}

/*
 * grabs one "chunk" (4 bytes) of the queued message to be transmitted
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
bool AP_Frsky_SPort_Passthrough::get_next_msg_chunk(void)
{
    if (!_statustext.available) {
        WITH_SEMAPHORE(_statustext.sem);
        if (!_statustext.queue.pop(_statustext.next)) {
            return false;
        }
        _statustext.available = true;
    }

    if (_msg_chunk.repeats == 0) { // if it's the first time get_next_msg_chunk is called for a given chunk
        uint8_t character = 0;
        _msg_chunk.chunk = 0; // clear the 4 bytes of the chunk buffer

        for (uint8_t i = 0; i < 4 && _msg_chunk.char_index < sizeof(_statustext.next.text); i++) {
            character = _statustext.next.text[_msg_chunk.char_index++];

            if (!character) {
                break;
            }

            _msg_chunk.chunk |= character << (3-i) * 8;
        }

        if (!character || (_msg_chunk.char_index == sizeof(_statustext.next.text))) { // we've reached the end of the message (string terminated by '\0' or last character of the string has been processed)
            _msg_chunk.char_index = 0; // reset index to get ready to process the next message
            // add severity which is sent as the MSB of the last three bytes of the last chunk (bits 24, 16, and 8) since a character is on 7 bits
            _msg_chunk.chunk |= (_statustext.next.severity & 0x4)<<21;
            _msg_chunk.chunk |= (_statustext.next.severity & 0x2)<<14;
            _msg_chunk.chunk |= (_statustext.next.severity & 0x1)<<7;
        }
    }

    // repeat each message chunk 3 times to ensure transmission
    // on slow links reduce the number of duplicate chunks
    uint8_t extra_chunks = 2;

    if (_scheduler.avg_packet_rate < 20) {
        // with 3 or more extra frsky sensors on the bus
        // send messages only once
        extra_chunks = 0;
    } else if (_scheduler.avg_packet_rate < 30) {
        // with 1 or 2 extra frsky sensors on the bus
        // send messages twice
        extra_chunks = 1;
    }

    if (_msg_chunk.repeats++ > extra_chunks ) {
        _msg_chunk.repeats = 0;
        if (_msg_chunk.char_index == 0) {
            // we're ready for the next message
            _statustext.available = false;
        }
    }
    return true;
}

/*
 * prepare parameter data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_SPort_Passthrough::calc_param(void)
{
    const AP_BattMonitor &_battery = AP::battery();

    uint32_t param = 0;
    uint8_t last_param = AP::battery().num_instances() > 1 ? BATT_CAPACITY_2 : BATT_CAPACITY_1;

    // cycle through paramIDs
    if (_paramID >= last_param) {
        _paramID = 0;
    }

    _paramID++;
    switch (_paramID) {
    case FRAME_TYPE:
        param = gcs().frame_type(); // see MAV_TYPE in Mavlink definition file common.h
        break;
    case BATT_FS_VOLTAGE:           // was used to send the battery failsafe voltage, lend slot to next param
    case BATT_FS_CAPACITY:          // was used to send the battery failsafe capacity in mAh, lend slot to next param
    case BATT_CAPACITY_1:
        _paramID = 4;
        param = (uint32_t)roundf(_battery.pack_capacity_mah(0)); // battery pack capacity in mAh
        break;
    case BATT_CAPACITY_2:
        param = (uint32_t)roundf(_battery.pack_capacity_mah(1)); // battery pack capacity in mAh
        break;
    }
    //Reserve first 8 bits for param ID, use other 24 bits to store parameter value
    param = (_paramID << PARAM_ID_OFFSET) | (param & PARAM_VALUE_LIMIT);

    return param;
}

/*
 * prepare gps status data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_SPort_Passthrough::calc_gps_status(void)
{
    const AP_GPS &gps = AP::gps();

    // number of GPS satellites visible (limit to 15 (0xF) since the value is stored on 4 bits)
    uint32_t gps_status = (gps.num_sats() < GPS_SATS_LIMIT) ? gps.num_sats() : GPS_SATS_LIMIT;
    // GPS receiver status (limit to 0-3 (0x3) since the value is stored on 2 bits: NO_GPS = 0, NO_FIX = 1, GPS_OK_FIX_2D = 2, GPS_OK_FIX_3D or GPS_OK_FIX_3D_DGPS or GPS_OK_FIX_3D_RTK_FLOAT or GPS_OK_FIX_3D_RTK_FIXED = 3)
    gps_status |= ((gps.status() < GPS_STATUS_LIMIT) ? gps.status() : GPS_STATUS_LIMIT)<<GPS_STATUS_OFFSET;
    // GPS horizontal dilution of precision in dm
    gps_status |= prep_number(roundf(gps.get_hdop() * 0.1f),2,1)<<GPS_HDOP_OFFSET;
    // GPS receiver advanced status (0: no advanced fix, 1: GPS_OK_FIX_3D_DGPS, 2: GPS_OK_FIX_3D_RTK_FLOAT, 3: GPS_OK_FIX_3D_RTK_FIXED)
    gps_status |= ((gps.status() > GPS_STATUS_LIMIT) ? gps.status()-GPS_STATUS_LIMIT : 0)<<GPS_ADVSTATUS_OFFSET;
    // Altitude MSL in dm
    const Location &loc = gps.location();
    gps_status |= prep_number(roundf(loc.alt * 0.1f),2,2)<<GPS_ALTMSL_OFFSET;
    return gps_status;
}

/*
 * prepare battery data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_SPort_Passthrough::calc_batt(uint8_t instance)
{
    const AP_BattMonitor &_battery = AP::battery();

    float current, consumed_mah;
    if (!_battery.current_amps(current, instance)) {
        current = 0;
    }
    if (!_battery.consumed_mah(consumed_mah, instance)) {
        consumed_mah = 0;
    }

    // battery voltage in decivolts, can have up to a 12S battery (4.25Vx12S = 51.0V)
    uint32_t batt = (((uint16_t)roundf(_battery.voltage(instance) * 10.0f)) & BATT_VOLTAGE_LIMIT);
    // battery current draw in deciamps
    batt |= prep_number(roundf(current * 10.0f), 2, 1)<<BATT_CURRENT_OFFSET;
    // battery current drawn since power on in mAh (limit to 32767 (0x7FFF) since value is stored on 15 bits)
    batt |= ((consumed_mah < BATT_TOTALMAH_LIMIT) ? ((uint16_t)roundf(consumed_mah) & BATT_TOTALMAH_LIMIT) : BATT_TOTALMAH_LIMIT)<<BATT_TOTALMAH_OFFSET;
    return batt;
}

/*
 * true if we need to respond to the last polling byte
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
bool AP_Frsky_SPort_Passthrough::is_passthrough_byte(const uint8_t byte)
{
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    if( byte == _SPort_bidir.downlink1_sensor_id || byte == _SPort_bidir.downlink2_sensor_id ) {
        return true;
    }
#endif
    return byte == SENSOR_ID_27;
}

/*
 * prepare various autopilot status data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_SPort_Passthrough::calc_ap_status(void)
{
    // IMU temperature: offset -19, 0 means temp =< 19°, 63 means temp => 82°
    uint8_t imu_temp = (uint8_t) roundf(constrain_float(AP::ins().get_temperature(0), AP_IMU_TEMP_MIN, AP_IMU_TEMP_MAX) - AP_IMU_TEMP_MIN);

    // control/flight mode number (limit to 31 (0x1F) since the value is stored on 5 bits)
    uint32_t ap_status = (uint8_t)((gcs().custom_mode()+1) & AP_CONTROL_MODE_LIMIT);
    // simple/super simple modes flags
    ap_status |= (uint8_t)(gcs().simple_input_active())<<AP_SIMPLE_OFFSET;
    ap_status |= (uint8_t)(gcs().supersimple_input_active())<<AP_SSIMPLE_OFFSET;
    // is_flying flag
    ap_status |= (uint8_t)(AP_Notify::flags.flying) << AP_FLYING_OFFSET;
    // armed flag
    ap_status |= (uint8_t)(AP_Notify::flags.armed)<<AP_ARMED_OFFSET;
    // battery failsafe flag
    ap_status |= (uint8_t)(AP_Notify::flags.failsafe_battery)<<AP_BATT_FS_OFFSET;
    // bad ekf flag
    ap_status |= (uint8_t)(AP_Notify::flags.ekf_bad)<<AP_EKF_FS_OFFSET;
    // IMU temperature
    ap_status |= imu_temp << AP_IMU_TEMP_OFFSET;
    return ap_status;
}

/*
 * prepare home position related data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_SPort_Passthrough::calc_home(void)
{
    uint32_t home = 0;
    Location loc;
    Location home_loc;
    bool got_position = false;
    float _relative_home_altitude = 0;

    {
        AP_AHRS &_ahrs = AP::ahrs();
        WITH_SEMAPHORE(_ahrs.get_semaphore());
        got_position = _ahrs.get_position(loc);
        home_loc = _ahrs.get_home();
    }

    if (got_position) {
        // check home_loc is valid
        if (home_loc.lat != 0 || home_loc.lng != 0) {
            // distance between vehicle and home_loc in meters
            home = prep_number(roundf(home_loc.get_distance(loc)), 3, 2);
            // angle from front of vehicle to the direction of home_loc in 3 degree increments (just in case, limit to 127 (0x7F) since the value is stored on 7 bits)
            home |= (((uint8_t)roundf(loc.get_bearing_to(home_loc) * 0.00333f)) & HOME_BEARING_LIMIT)<<HOME_BEARING_OFFSET;
        }
        // altitude between vehicle and home_loc
        _relative_home_altitude = loc.alt;
        if (!loc.relative_alt) {
            // loc.alt has home altitude added, remove it
            _relative_home_altitude -= home_loc.alt;
        }
    }
    // altitude above home in decimeters
    home |= prep_number(roundf(_relative_home_altitude * 0.1f), 3, 2)<<HOME_ALT_OFFSET;
    return home;
}

/*
 * prepare velocity and yaw data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_SPort_Passthrough::calc_velandyaw(void)
{
    float vspd = get_vspeed_ms();
    // vertical velocity in dm/s
    uint32_t velandyaw = prep_number(roundf(vspd * 10), 2, 1);
    AP_AHRS &_ahrs = AP::ahrs();
    WITH_SEMAPHORE(_ahrs.get_semaphore());
    // horizontal velocity in dm/s (use airspeed if available and enabled - even if not used - otherwise use groundspeed)
    const AP_Airspeed *aspeed = _ahrs.get_airspeed();
    if (aspeed && aspeed->enabled()) {
        velandyaw |= prep_number(roundf(aspeed->get_airspeed() * 10), 2, 1)<<VELANDYAW_XYVEL_OFFSET;
    } else { // otherwise send groundspeed estimate from ahrs
        velandyaw |= prep_number(roundf(_ahrs.groundspeed() * 10), 2, 1)<<VELANDYAW_XYVEL_OFFSET;
    }
    // yaw from [0;36000] centidegrees to .2 degree increments [0;1800] (just in case, limit to 2047 (0x7FF) since the value is stored on 11 bits)
    velandyaw |= ((uint16_t)roundf(_ahrs.yaw_sensor * 0.05f) & VELANDYAW_YAW_LIMIT)<<VELANDYAW_YAW_OFFSET;
    return velandyaw;
}

/*
 * prepare attitude (roll, pitch) and range data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_SPort_Passthrough::calc_attiandrng(void)
{
    const RangeFinder *_rng = RangeFinder::get_singleton();

    AP_AHRS &_ahrs = AP::ahrs();
    // roll from [-18000;18000] centidegrees to unsigned .2 degree increments [0;1800] (just in case, limit to 2047 (0x7FF) since the value is stored on 11 bits)
    uint32_t attiandrng = ((uint16_t)roundf((_ahrs.roll_sensor + 18000) * 0.05f) & ATTIANDRNG_ROLL_LIMIT);
    // pitch from [-9000;9000] centidegrees to unsigned .2 degree increments [0;900] (just in case, limit to 1023 (0x3FF) since the value is stored on 10 bits)
    attiandrng |= ((uint16_t)roundf((_ahrs.pitch_sensor + 9000) * 0.05f) & ATTIANDRNG_PITCH_LIMIT)<<ATTIANDRNG_PITCH_OFFSET;
    // rangefinder measurement in cm
    attiandrng |= prep_number(_rng ? _rng->distance_cm_orient(ROTATION_PITCH_270) : 0, 3, 1)<<ATTIANDRNG_RNGFND_OFFSET;
    return attiandrng;
}

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
/*
  allow external transports (e.g. FPort), to supply telemetry data
 */
bool AP_Frsky_SPort_Passthrough::set_telem_data(const uint8_t frame, const uint16_t appid, const uint32_t data)
{
    // queue only Uplink packets
    if (frame == SPORT_UPLINK_FRAME || frame == SPORT_UPLINK_FRAME_RW) {
        const AP_Frsky_SPort::sport_packet_t sp {
            0x00,   // this is ignored by process_sport_rx_queue() so no need for a real sensor ID
            frame,
            appid,
            data
        };

        _SPort_bidir.rx_packet_queue.push_force(sp);
        return true;
    }
    return false;
}

/*
  fetch Sport data for an external transport, such as FPort
 */
bool AP_Frsky_SPort_Passthrough::get_telem_data(uint8_t &frame, uint16_t &appid, uint32_t &data)
{
    run_wfq_scheduler();
    if (!external_data.pending) {
        return false;
    }
    frame = external_data.frame;
    appid = external_data.appid;
    data = external_data.data;
    external_data.pending = false;
    return true;
}

/*
 * prepare value for transmission through FrSky link
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint16_t AP_Frsky_SPort_Passthrough::prep_number(int32_t number, uint8_t digits, uint8_t power)
{
    uint16_t res = 0;
    uint32_t abs_number = abs(number);

    if ((digits == 2) && (power == 1)) { // number encoded on 8 bits: 7 bits for digits + 1 for 10^power
        if (abs_number < 100) {
            res = abs_number<<1;
        } else if (abs_number < 1270) {
            res = ((uint8_t)roundf(abs_number * 0.1f)<<1)|0x1;
        } else { // transmit max possible value (0x7F x 10^1 = 1270)
            res = 0xFF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<8;
        }
    } else if ((digits == 2) && (power == 2)) { // number encoded on 9 bits: 7 bits for digits + 2 for 10^power
        if (abs_number < 100) {
            res = abs_number<<2;
        } else if (abs_number < 1000) {
            res = ((uint8_t)roundf(abs_number * 0.1f)<<2)|0x1;
        } else if (abs_number < 10000) {
            res = ((uint8_t)roundf(abs_number * 0.01f)<<2)|0x2;
        } else if (abs_number < 127000) {
            res = ((uint8_t)roundf(abs_number * 0.001f)<<2)|0x3;
        } else { // transmit max possible value (0x7F x 10^3 = 127000)
            res = 0x1FF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<9;
        }
    } else if ((digits == 3) && (power == 1)) { // number encoded on 11 bits: 10 bits for digits + 1 for 10^power
        if (abs_number < 1000) {
            res = abs_number<<1;
        } else if (abs_number < 10240) {
            res = ((uint16_t)roundf(abs_number * 0.1f)<<1)|0x1;
        } else { // transmit max possible value (0x3FF x 10^1 = 10240)
            res = 0x7FF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<11;
        }
    } else if ((digits == 3) && (power == 2)) { // number encoded on 12 bits: 10 bits for digits + 2 for 10^power
        if (abs_number < 1000) {
            res = abs_number<<2;
        } else if (abs_number < 10000) {
            res = ((uint16_t)roundf(abs_number * 0.1f)<<2)|0x1;
        } else if (abs_number < 100000) {
            res = ((uint16_t)roundf(abs_number * 0.01f)<<2)|0x2;
        } else if (abs_number < 1024000) {
            res = ((uint16_t)roundf(abs_number * 0.001f)<<2)|0x3;
        } else { // transmit max possible value (0x3FF x 10^3 = 127000)
            res = 0xFFF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<12;
        }
    }
    return res;
}




/*
 * Queue uplink packets in the sport rx queue
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_SPort_Passthrough::queue_rx_packet(const AP_Frsky_SPort::sport_packet_t packet)
{
    // queue only Uplink packets
    if (packet.sensor == _SPort_bidir.uplink_sensor_id && packet.frame == SPORT_UPLINK_FRAME) {
        _SPort_bidir.rx_packet_queue.push_force(packet);
    }
}

/*
 * Extract up to 1 mavlite message from the sport rx packet queue
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_SPort_Passthrough::process_rx_queue()
{
    AP_Frsky_SPort::sport_packet_t packet;
    uint8_t loop_count = 0; // prevent looping forever
    while (_SPort_bidir.rx_packet_queue.pop(packet) && loop_count++ < MAVLITE_MSG_SPORT_PACKETS_COUNT(MAVLITE_MAX_PAYLOAD_LEN)) {
        AP_Frsky_MAVlite_Message rxmsg;

        if (sport_to_mavlite.process(rxmsg, packet)) {
            mavlite.process_message(rxmsg);
            break; // process only 1 mavlite message each call
        }
    }
}

/*
 * Process the sport tx queue
 * pop and send 1 sport packet
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_SPort_Passthrough::process_tx_queue()
{
    AP_Frsky_SPort::sport_packet_t packet;

    if (!_SPort_bidir.tx_packet_queue.peek(packet)) {
        return;
    }

    // when using fport repeat each packet to account for
    // fport packet loss (around 15%)
    if (!_use_external_data || _SPort_bidir.tx_packet_duplicates++ == SPORT_TX_PACKET_DUPLICATES) {
        _SPort_bidir.tx_packet_queue.pop();
        _SPort_bidir.tx_packet_duplicates = 0;
    }

    send_sport_frame(SPORT_DOWNLINK_FRAME, packet.appid, packet.data);
}

/*
 * Utility method to apply constraints in changing sensor id values
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_SPort_Passthrough::set_sensor_id(AP_Int8 param_idx, uint8_t &sensor)
{
    int8_t idx = param_idx.get();

    if (idx == -1) {
        // disable this sensor
        sensor = 0xFF;
        return;
    }
    sensor = calc_sensor_id(idx);
}

/*
 * Send a mavlite message
 * Message is chunked in sport packets pushed in the tx queue
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
bool AP_Frsky_SPort_Passthrough::send_message(const AP_Frsky_MAVlite_Message &txmsg)
{
    return mavlite_to_sport.process(_SPort_bidir.tx_packet_queue, txmsg);
}
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
