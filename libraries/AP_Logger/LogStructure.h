#pragma once

#include <AP_Common/AP_Common.h>

// if you add any new types, units or multipliers, please update README.md

/*
Format characters in the format string for binary log messages
  a   : int16_t[32]
  b   : int8_t
  B   : uint8_t
  h   : int16_t
  H   : uint16_t
  i   : int32_t
  I   : uint32_t
  f   : float
  d   : double
  n   : char[4]
  N   : char[16]
  Z   : char[64]
  c   : int16_t * 100
  C   : uint16_t * 100
  e   : int32_t * 100
  E   : uint32_t * 100
  L   : int32_t latitude/longitude
  M   : uint8_t flight mode
  q   : int64_t
  Q   : uint64_t
 */

struct UnitStructure {
    const char ID;
    const char *unit;
};

struct MultiplierStructure {
    const char ID;
    const double multiplier;
};

// all units here should be base units
// This does mean battery capacity is here as "amp*second"
// Please keep the names consistent with Tools/autotest/param_metadata/param.py:33
const struct UnitStructure log_Units[] = {
    { '-', "" },              // no units e.g. Pi, or a string
    { '?', "UNKNOWN" },       // Units which haven't been worked out yet....
    { 'A', "A" },             // Ampere
    { 'a', "Ah" },            // Ampere hours
    { 'd', "deg" },           // of the angular variety, -180 to 180
    { 'b', "B" },             // bytes
    { 'k', "deg/s" },         // degrees per second. Degrees are NOT SI, but is some situations more user-friendly than radians
    { 'D', "deglatitude" },   // degrees of latitude
    { 'e', "deg/s/s" },       // degrees per second per second. Degrees are NOT SI, but is some situations more user-friendly than radians
    { 'E', "rad/s" },         // radians per second
    { 'G', "Gauss" },         // Gauss is not an SI unit, but 1 tesla = 10000 gauss so a simple replacement is not possible here
    { 'h', "degheading" },    // 0.? to 359.?
    { 'i', "A.s" },           // Ampere second
    { 'J', "W.s" },           // Joule (Watt second)
    // { 'l', "l" },          // litres
    { 'L', "rad/s/s" },       // radians per second per second
    { 'm', "m" },             // metres
    { 'n', "m/s" },           // metres per second
    // { 'N', "N" },          // Newton
    { 'o', "m/s/s" },         // metres per second per second
    { 'O', "degC" },          // degrees Celsius. Not SI, but Kelvin is too cumbersome for most users
    { '%', "%" },             // percent
    { 'S', "satellites" },    // number of satellites
    { 's', "s" },             // seconds
    { 'q', "rpm" },           // rounds per minute. Not SI, but sometimes more intuitive than Hertz
    { 'r', "rad" },           // radians
    { 'U', "deglongitude" },  // degrees of longitude
    { 'u', "ppm" },           // pulses per minute
    { 'v', "V" },             // Volt
    { 'P', "Pa" },            // Pascal
    { 'w', "Ohm" },           // Ohm
    { 'W', "Watt" },        // Watt
    { 'Y', "us" },            // pulse width modulation in microseconds
    { 'z', "Hz" },            // Hertz
    { '#', "instance" }       // (e.g.)Sensor instance number
};

// this multiplier information applies to the raw value present in the
// log.  Any adjustment implied by the format field (e.g. the "centi"
// in "centidegrees" is *IGNORED* for the purposes of scaling.
// Essentially "format" simply tells you the C-type, and format-type h
// (int16_t) is equivalent to format-type c (int16_t*100)
// tl;dr a GCS shouldn't/mustn't infer any scaling from the unit name

const struct MultiplierStructure log_Multipliers[] = {
    { '-', 0 },       // no multiplier e.g. a string
    { '?', 1 },       // multipliers which haven't been worked out yet....
// <leave a gap here, just in case....>
    { '2', 1e2 },
    { '1', 1e1 },
    { '0', 1e0 },
    { 'A', 1e-1 },
    { 'B', 1e-2 },
    { 'C', 1e-3 },
    { 'D', 1e-4 },
    { 'E', 1e-5 },
    { 'F', 1e-6 },
    { 'G', 1e-7 },
// <leave a gap here, just in case....>
    { '!', 3.6 }, // (ampere*second => milliampere*hour) and (km/h => m/s)
    { '/', 3600 }, // (ampere*second => ampere*hour)
};

/*
  unfortunately these need to be macros because of a limitation of
  named member structure initialisation in g++
 */
#define LOG_PACKET_HEADER	       uint8_t head1, head2, msgid;
#define LOG_PACKET_HEADER_INIT(id) head1 : HEAD_BYTE1, head2 : HEAD_BYTE2, msgid : id
#define LOG_PACKET_HEADER_LEN 3 // bytes required for LOG_PACKET_HEADER

// once the logging code is all converted we will remove these from
// this header
#define HEAD_BYTE1  0xA3    // Decimal 163
#define HEAD_BYTE2  0x95    // Decimal 149

#include <AP_DAL/LogStructure.h>
#include <AP_NavEKF2/LogStructure.h>
#include <AP_NavEKF3/LogStructure.h>
#include <AP_GPS/LogStructure.h>
#include <AP_NavEKF/LogStructure.h>
#include <AP_BattMonitor/LogStructure.h>
#include <AP_InertialSensor/LogStructure.h>
#include <AP_AHRS/LogStructure.h>
#include <AP_Camera/LogStructure.h>
#include <AP_Baro/LogStructure.h>
#include <AP_VisualOdom/LogStructure.h>
#include <AC_PrecLand/LogStructure.h>
#include <AC_Avoidance/LogStructure.h>
#include <AP_ESC_Telem/LogStructure.h>

// structure used to define logging format
struct LogStructure {
    uint8_t msg_type;
    uint8_t msg_len;
    const char *name;
    const char *format;
    const char *labels;
    const char *units;
    const char *multipliers;
};

// maximum lengths of fields in LogStructure, including trailing nulls
static const uint8_t LS_NAME_SIZE = 5;
static const uint8_t LS_FORMAT_SIZE = 17;
static const uint8_t LS_LABELS_SIZE = 65;
static const uint8_t LS_UNITS_SIZE = 17;
static const uint8_t LS_MULTIPLIERS_SIZE = 17;

/*
  log structures common to all vehicle types
 */
struct PACKED log_Format {
    LOG_PACKET_HEADER;
    uint8_t type;
    uint8_t length;
    char name[4];
    char format[16];
    char labels[64];
};

struct PACKED log_Unit {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    char type;
    char unit[64]; // you know, this might be overkill...
};

struct PACKED log_Format_Multiplier {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    char type;
    double multiplier;
};

struct PACKED log_Format_Units {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t format_type;
    char units[16];
    char multipliers[16];
};

struct PACKED log_Parameter {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    char name[16];
    float value;
};

struct PACKED log_DSF {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t dropped;
    uint16_t blocks;
    uint32_t bytes;
    uint32_t buf_space_min;
    uint32_t buf_space_max;
    uint32_t buf_space_avg;
};

struct PACKED log_Event {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
};

struct PACKED log_Error {
  LOG_PACKET_HEADER;
  uint64_t time_us;
  uint8_t sub_system;
  uint8_t error_code;
};


struct PACKED log_Message {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    char msg[64];
};

struct PACKED log_RCIN {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t chan1;
    uint16_t chan2;
    uint16_t chan3;
    uint16_t chan4;
    uint16_t chan5;
    uint16_t chan6;
    uint16_t chan7;
    uint16_t chan8;
    uint16_t chan9;
    uint16_t chan10;
    uint16_t chan11;
    uint16_t chan12;
    uint16_t chan13;
    uint16_t chan14;
};

struct PACKED log_RCIN2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t chan15;
    uint16_t chan16;
};

struct PACKED log_RCOUT {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t chan1;
    uint16_t chan2;
    uint16_t chan3;
    uint16_t chan4;
    uint16_t chan5;
    uint16_t chan6;
    uint16_t chan7;
    uint16_t chan8;
    uint16_t chan9;
    uint16_t chan10;
    uint16_t chan11;
    uint16_t chan12;
    uint16_t chan13;
    uint16_t chan14;
};

struct PACKED log_MAV {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t chan;
    uint16_t packet_tx_count;
    uint16_t packet_rx_success_count;
    uint16_t packet_rx_drop_count;
    uint8_t flags;
    uint16_t stream_slowdown_ms;
    uint16_t times_full;
};

struct PACKED log_RSSI {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float RXRSSI;
};

struct PACKED log_Optflow {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t surface_quality;
    float flow_x;
    float flow_y;
    float body_x;
    float body_y;
};

struct PACKED log_POWR {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float Vcc;
    float Vservo;
    uint16_t flags;
    uint16_t accumulated_flags;
    uint8_t safety_and_arm;
};

struct PACKED log_Cmd {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t command_total;
    uint16_t sequence;
    uint16_t command;
    float param1;
    float param2;
    float param3;
    float param4;
    int32_t latitude;
    int32_t longitude;
    float altitude;
    uint8_t frame;
};

struct PACKED log_MAVLink_Command {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t frame;
    uint16_t command;
    uint8_t current;
    uint8_t autocontinue;
    float param1;
    float param2;
    float param3;
    float param4;
    int32_t x;
    int32_t y;
    float z;
    uint8_t result;
    bool was_command_long;
};

struct PACKED log_Radio {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t rssi;
    uint8_t remrssi;
    uint8_t txbuf;
    uint8_t noise;
    uint8_t remnoise;
    uint16_t rxerrors;
    uint16_t fixed;
};

struct PACKED log_PID {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float   target;
    float   actual;
    float   error;
    float   P;
    float   I;
    float   D;
    float   FF;
    float   Dmod;
    float   slew_rate;
    uint8_t limit;
};

struct PACKED log_WheelEncoder {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float distance_0;
    uint8_t quality_0;
    float distance_1;
    uint8_t quality_1;
};

struct PACKED log_ADSB {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t ICAO_address;
    int32_t lat;
    int32_t lng;
    int32_t alt;
    uint16_t heading;
    uint16_t hor_velocity;
    int16_t ver_velocity;
    uint16_t squawk;
};

struct PACKED log_MAG {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    int16_t  mag_x;
    int16_t  mag_y;
    int16_t  mag_z;
    int16_t  offset_x;
    int16_t  offset_y;
    int16_t  offset_z;
    int16_t  motor_offset_x;
    int16_t  motor_offset_y;
    int16_t  motor_offset_z;
    uint8_t  health;
    uint32_t SUS;
};

struct PACKED log_Mode {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t mode;
    uint8_t mode_num;
    uint8_t mode_reason;
};

/*
  rangefinder - support for 4 sensors
 */
struct PACKED log_RFND {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    uint16_t dist;
    uint8_t status;
    uint8_t orient;
};

/*
  terrain log structure
 */
struct PACKED log_TERRAIN {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t status;
    int32_t lat;
    int32_t lng;
    uint16_t spacing;
    float terrain_height;
    float current_height;
    uint16_t pending;
    uint16_t loaded;
};

struct PACKED log_CSRV {
    LOG_PACKET_HEADER;
    uint64_t time_us;     
    uint8_t id;
    float position;
    float force;
    float speed;
    uint8_t power_pct;
};

struct PACKED log_ARSP {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    float   airspeed;
    float   diffpressure;
    int16_t temperature;
    float   rawpressure;
    float   offset;
    bool    use;
    bool    healthy;
    float   health_prob;
    uint8_t primary;
};

struct PACKED log_MAV_Stats {
    LOG_PACKET_HEADER;
    uint64_t timestamp;
    uint32_t seqno;
    uint32_t dropped;
    uint32_t retries;
    uint32_t resends;
    uint8_t state_free_avg;
    uint8_t state_free_min;
    uint8_t state_free_max;
    uint8_t state_pending_avg;
    uint8_t state_pending_min;
    uint8_t state_pending_max;
    uint8_t state_sent_avg;
    uint8_t state_sent_min;
    uint8_t state_sent_max;
    // uint8_t state_retry_avg;
    // uint8_t state_retry_min;
    // uint8_t state_retry_max;
};

struct PACKED log_RPM {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float rpm1;
    float rpm2;
};

struct PACKED log_SbpLLH {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t tow;
    int32_t  lat;
    int32_t  lon;
    int32_t  alt;
    uint8_t  n_sats;
    uint8_t  flags;
};

struct PACKED log_SbpHealth {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t crc_error_counter;
    uint32_t last_injected_data_ms;
    uint32_t last_iar_num_hypotheses;
};

struct PACKED log_SbpRAWH {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t msg_type;
    uint16_t sender_id;
    uint8_t index;
    uint8_t pages;
    uint8_t msg_len;
    uint8_t res;
    uint8_t data[48];
};

struct PACKED log_SbpRAWM {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t msg_type;
    uint16_t sender_id;
    uint8_t index;
    uint8_t pages;
    uint8_t msg_len;
    uint8_t res;
    uint8_t data[104];
};

struct PACKED log_SbpEvent {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t wn;
    uint32_t tow;
    int32_t ns_residual;
    uint8_t level;
    uint8_t quality;
};

struct PACKED log_Rally {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t total;
    uint8_t sequence;
    int32_t latitude;
    int32_t longitude;
    int16_t altitude;
};

struct PACKED log_Beacon {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t health;
    uint8_t count;
    float dist0;
    float dist1;
    float dist2;
    float dist3;
    float posx;
    float posy;
    float posz;
};

// proximity sensor logging
struct PACKED log_Proximity {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    uint8_t health;
    float dist0;
    float dist45;
    float dist90;
    float dist135;
    float dist180;
    float dist225;
    float dist270;
    float dist315;
    float distup;
    float closest_angle;
    float closest_dist;
};
struct PACKED log_Proximity_raw {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    float raw_dist0;
    float raw_dist45;
    float raw_dist90;
    float raw_dist135;
    float raw_dist180;
    float raw_dist225;
    float raw_dist270;
    float raw_dist315;
};

struct PACKED log_Performance {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t num_long_running;
    uint16_t num_loops;
    uint32_t max_time;
    uint32_t mem_avail;
    uint16_t load;
    uint16_t internal_error_last_line;
    uint32_t internal_errors;
    uint32_t internal_error_count;
    uint32_t spi_count;
    uint32_t i2c_count;
    uint32_t i2c_isr_count;
    uint32_t extra_loop_us;
};

struct PACKED log_SRTL {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t active;
    uint16_t num_points;
    uint16_t max_points;
    uint8_t action;
    float N;
    float E;
    float D;
};

struct PACKED log_DSTL {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t stage;
    float target_heading;
    int32_t target_lat;
    int32_t target_lng;
    int32_t target_alt;
    int16_t crosstrack_error;
    int16_t travel_distance;
    float l1_i;
    int32_t loiter_sum_cd;
    float desired;
    float P;
    float I;
    float D;
};

struct PACKED log_Arm_Disarm {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  arm_state;
    uint32_t arm_checks;
    uint8_t forced;
    uint8_t method;
};

struct PACKED log_Winch {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t healthy;
    uint8_t thread_end;
    uint8_t moving;
    uint8_t clutch;
    uint8_t mode;
    float desired_length;
    float length;
    float desired_rate;
    uint16_t tension;
    float voltage;
    int8_t temp;
};

struct PACKED log_PSC {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float pos_target_x;
    float pos_target_Y;
    float position_x;
    float position_y;
    float vel_target_x;
    float vel_target_y;
    float velocity_x;
    float velocity_y;
    float accel_target_x;
    float accel_target_y;
    float accel_x;
    float accel_y;
};

// position controller z-axis logging
struct PACKED log_PSCZ {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float pos_target_z;
    float pos_z;
    float vel_desired_z;
    float vel_target_z;
    float vel_z;
    float accel_desired_z;
    float accel_target_z;
    float accel_z;
    float throttle_out;
};

// FMT messages define all message formats other than FMT
// UNIT messages define units which can be referenced by FMTU messages
// FMTU messages associate types (e.g. centimeters/second/second) to FMT message fields

#define PID_LABELS "TimeUS,Tar,Act,Err,P,I,D,FF,Dmod,SRate,Limit"
#define PID_FMT    "QfffffffffB"
#define PID_UNITS  "s----------"
#define PID_MULTS  "F----------"

// @LoggerMessage: ADSB
// @Description: Automatic Dependant Serveillance - Broadcast detected vehicle information
// @Field: TimeUS: Time since system startup
// @Field: ICAO_address: Transponder address
// @Field: Lat: Vehicle latitude
// @Field: Lng: Vehicle longitude
// @Field: Alt: Vehicle altitude
// @Field: Heading: Vehicle heading
// @Field: Hor_vel: Vehicle horizontal velocity
// @Field: Ver_vel: Vehicle vertical velocity
// @Field: Squark: Transponder squawk code

// @LoggerMessage: ARM
// @Description: Arming status changes
// @Field: TimeUS: Time since system startup
// @Field: ArmState: true if vehicle is now armed
// @Field: ArmChecks: arming bitmask at time of arming
// @Field: Forced: true if arm/disarm was forced
// @Field: Method: method used for arming

// @LoggerMessage: ARSP
// @Description: Airspeed sensor data
// @Field: TimeUS: Time since system startup
// @Field: I: Airspeed sensor instance number
// @Field: Airspeed: Current airspeed
// @Field: DiffPress: Pressure difference between static and dynamic port
// @Field: Temp: Temperature used for calculation
// @Field: RawPress: Raw pressure less offset
// @Field: Offset: Offset from parameter
// @Field: U: True if sensor is being used
// @Field: H: True if sensor is healthy
// @Field: Hfp: Probability sensor has failed
// @Field: Pri: True if sensor is the primary sensor

// @LoggerMessage: BCN
// @Description: Beacon information
// @Field: TimeUS: Time since system startup
// @Field: Health: True if beacon sensor is healthy
// @Field: Cnt: Number of beacons being used
// @Field: D0: Distance to first beacon
// @Field: D1: Distance to second beacon
// @Field: D2: Distance to third beacon
// @Field: D3: Distance to fourth beacon
// @Field: PosX: Calculated beacon position, x-axis
// @Field: PosY: Calculated beacon position, y-axis
// @Field: PosZ: Calculated beacon position, z-axis

// @LoggerMessage: CMD
// @Description: Executed mission command information
// @Field: TimeUS: Time since system startup
// @Field: CTot: Total number of mission commands
// @Field: CNum: This command's offset in mission
// @Field: CId: Command type
// @Field: Prm1: Parameter 1
// @Field: Prm2: Parameter 2
// @Field: Prm3: Parameter 3
// @Field: Prm4: Parameter 4
// @Field: Lat: Command latitude
// @Field: Lng: Command longitude
// @Field: Alt: Command altitude
// @Field: Frame: Frame used for position

// @LoggerMessage: CSRV
// @Description: Servo feedback data
// @Field: TimeUS: Time since system startup
// @Field: Id: Servo number this data relates to
// @Field: Pos: Current servo position
// @Field: Force: Force being applied
// @Field: Speed: Current servo movement speed
// @Field: Pow: Amount of rated power being applied

// @LoggerMessage: DMS
// @Description: DataFlash-Over-MAVLink statistics
// @Field: TimeUS: Time since system startup
// @Field: N: Current block number
// @Field: Dp: Number of times we rejected a write to the backend
// @Field: RT: Number of blocks sent from the retry queue
// @Field: RS: Number of resends of unacknowledged data made
// @Field: Fa: Average number of blocks on the free list
// @Field: Fmn: Minimum number of blocks on the free list
// @Field: Fmx: Maximum number of blocks on the free list
// @Field: Pa: Average number of blocks on the pending list
// @Field: Pmn: Minimum number of blocks on the pending list
// @Field: Pmx: Maximum number of blocks on the pending list
// @Field: Sa: Average number of blocks on the sent list
// @Field: Smn: Minimum number of blocks on the sent list
// @Field: Smx: Maximum number of blocks on the sent list

// @LoggerMessage: DSF
// @Description: Onboard logging statistics
// @Field: TimeUS: Time since system startup
// @Field: Dp: Number of times we rejected a write to the backend
// @Field: Blk: Current block number
// @Field: Bytes: Current write offset
// @Field: FMn: Minimum free space in write buffer in last time period
// @Field: FMx: Maximum free space in write buffer in last time period
// @Field: FAv: Average free space in write buffer in last time period

// @LoggerMessage: DSTL
// @Description: Deepstall Landing data
// @Field: TimeUS: Time since system startup
// @Field: Stg: Deepstall landing stage
// @Field: THdg: Target heading
// @Field: Lat: Landing point latitude
// @Field: Lng: Landing point longitude
// @Field: Alt: Landing point altitude
// @Field: XT: Crosstrack error
// @Field: Travel: Expected travel distance vehicle will travel from this point
// @Field: L1I: L1 controller crosstrack integrator value
// @Field: Loiter: wind estimate loiter angle flown
// @Field: Des: Deepstall steering PID desired value
// @Field: P: Deepstall steering PID Proportional response component
// @Field: I: Deepstall steering PID Integral response component
// @Field: D: Deepstall steering PID Derivative response component

// @LoggerMessage: ERR
// @Description: Specifically coded error messages
// @Field: TimeUS: Time since system startup
// @Field: Subsys: Subsystem in which the error occurred
// @Field: ECode: Subsystem-specific error code

// @LoggerMessage: EV
// @Description: Specifically coded event messages
// @Field: TimeUS: Time since system startup
// @Field: Id: Event identifier

// @LoggerMessage: FMT
// @Description: Message defining the format of messages in this file
// @URL: https://ardupilot.org/dev/docs/code-overview-adding-a-new-log-message.html
// @Field: Type: unique-to-this-log identifier for message being defined
// @Field: Length: the number of bytes taken up by this message (including all headers)
// @Field: Name: name of the message being defined
// @Field: Format: character string defining the C-storage-type of the fields in this message
// @Field: Columns: the labels of the message being defined

// @LoggerMessage: FMTU
// @Description: Message defining units and multipliers used for fields of other messages
// @Field: TimeUS: Time since system startup
// @Field: FmtType: numeric reference to associated FMT message
// @Field: UnitIds: each character refers to a UNIT message.  The unit at an offset corresponds to the field at the same offset in FMT.Format
// @Field: MultIds: each character refers to a MULT message.  The multiplier at an offset corresponds to the field at the same offset in FMT.Format

// @LoggerMessage: LGR
// @Description: Landing gear information
// @Field: TimeUS: Time since system startup
// @Field: LandingGear: Current landing gear state
// @Field: WeightOnWheels: True if there is weight on wheels

// @LoggerMessage: MAG
// @Description: Information received from compasses
// @Field: TimeUS: Time since system startup
// @Field: I: magnetometer sensor instance number
// @Field: MagX: magnetic field strength in body frame
// @Field: MagY: magnetic field strength in body frame
// @Field: MagZ: magnetic field strength in body frame
// @Field: OfsX: magnetic field offset in body frame
// @Field: OfsY: magnetic field offset in body frame
// @Field: OfsZ: magnetic field offset in body frame
// @Field: MOX: motor interference magnetic field offset in body frame
// @Field: MOY: motor interference magnetic field offset in body frame
// @Field: MOZ: motor interference magnetic field offset in body frame
// @Field: Health: true if the compass is considered healthy
// @Field: S: time measurement was taken

// @LoggerMessage: MAV
// @Description: GCS MAVLink link statistics
// @Field: TimeUS: Time since system startup
// @Field: chan: mavlink channel number
// @Field: txp: transmitted packet count
// @Field: rxp: received packet count
// @Field: rxdp: perceived number of packets we never received
// @Field: flags: compact representation of some stage of the channel
// @Field: ss: stream slowdown is the number of ms being added to each message to fit within bandwidth
// @Field: tf: times buffer was full when a message was going to be sent

// @LoggerMessage: MAVC
// @Description: MAVLink command we have just executed
// @Field: TimeUS: Time since system startup
// @Field: TS: target system for command
// @Field: TC: target component for command
// @Field: Fr: command frame
// @Field: Cmd: mavlink command enum value
// @Field: Cur: current flag from mavlink packet
// @Field: AC: autocontinue flag from mavlink packet
// @Field: P1: first parameter from mavlink packet
// @Field: P2: second parameter from mavlink packet
// @Field: P3: third parameter from mavlink packet
// @Field: P4: fourth parameter from mavlink packet
// @Field: X: X coordinate from mavlink packet
// @Field: Y: Y coordinate from mavlink packet
// @Field: Z: Z coordinate from mavlink packet
// @Field: Res: command result being returned from autopilot
// @Field: WL: true if this command arrived via a COMMAND_LONG rather than COMMAND_INT

// @LoggerMessage: MODE
// @Description: vehicle control mode information
// @Field: TimeUS: Time since system startup
// @Field: Mode: vehicle-specific mode number
// @Field: ModeNum: alias for Mode
// @Field: Rsn: reason for entering this mode; enumeration value

// @LoggerMessage: MON
// @Description: Main loop stuck data
// @Field: TimeUS: Time since system startup
// @Field: LDelay: Time main loop has been stuck for
// @Field: Task: Current scheduler task number
// @Field: IErr: Internal error mask; which internal errors have been detected
// @Field: IErrCnt: Internal error count; how many internal errors have been detected
// @Field: IErrLn: Line on which internal error ocurred
// @Field: MavMsg: Id of the last mavlink message processed
// @Field: MavCmd: Id of the last mavlink command processed
// @Field: SemLine: Line number of semaphore most recently taken
// @Field: SPICnt: Number of SPI transactions processed
// @Field: I2CCnt: Number of i2c transactions processed

// @LoggerMessage: MSG
// @Description: Textual messages
// @Field: TimeUS: Time since system startup
// @Field: Message: message text

// @LoggerMessage: MULT
// @Description: Message mapping from single character to numeric multiplier
// @Field: TimeUS: Time since system startup
// @Field: Id: character referenced by FMTU
// @Field: Mult: numeric multiplier

// @LoggerMessage: OF
// @Description: Optical flow sensor data
// @Field: TimeUS: Time since system startup
// @Field: Qual: Estimated sensor data quality
// @Field: flowX: Sensor flow rate, X-axis
// @Field: flowY: Sensor flow rate,Y-axis
// @Field: bodyX: derived velocity, X-axis
// @Field: bodyY: derived velocity, Y-axis

// @LoggerMessage: PARM
// @Description: parameter value
// @Field: TimeUS: Time since system startup
// @Field: Name: parameter name
// @Field: Value: parameter value

// @LoggerMessage: PIDR,PIDP,PIDY,PIDA,PIDS
// @Description: Proportional/Integral/Derivative gain values for Roll/Pitch/Yaw/Altitude/Steering
// @Field: TimeUS: Time since system startup
// @Field: Tar: desired value
// @Field: Act: achieved value
// @Field: Err: error between target and achieved
// @Field: P: proportional part of PID
// @Field: I: integral part of PID
// @Field: D: derivative part of PID
// @Field: FF: controller feed-forward portion of response
// @Field: Dmod: scaler applied to D gain to reduce limit cycling
// @Field: SRate: slew rate used in slew limiter
// @Field: Limit: 1 if I term is limited due to output saturation

// @LoggerMessage: PM
// @Description: autopilot system performance and general data dumping ground
// @Field: TimeUS: Time since system startup
// @Field: NLon: Number of long loops detected
// @Field: NLoop: Number of measurement loops for this message
// @Field: MaxT: Maximum loop time
// @Field: Mem: Free memory available
// @Field: Load: System processor load
// @Field: IntE: Internal error mask; which internal errors have been detected
// @Field: ErrL: Internal error line number; last line number on which a internal error was detected
// @Field: ErrC: Internal error count; how many internal errors have been detected
// @Field: SPIC: Number of SPI transactions processed
// @Field: I2CC: Number of i2c transactions processed
// @Field: I2CI: Number of i2c interrupts serviced
// @Field: Ex: number of microseconds being added to each loop to address scheduler overruns

// @LoggerMessage: POWR
// @Description: System power information
// @Field: TimeUS: Time since system startup
// @Field: Vcc: Flight board voltage
// @Field: VServo: Servo rail voltage
// @Field: Flags: System power flags
// @Field: AccFlags: Accumulated System power flags; all flags which have ever been set
// @Field: Safety: Hardware Safety Switch status

// @LoggerMessage: PRX
// @Description: Proximity Filtered sensor data
// @Field: TimeUS: Time since system startup
// @Field: Layer: Pitch(instance) at which the obstacle is at. 0th layer {-75,-45} degrees. 1st layer {-45,-15} degrees. 2nd layer {-15, 15} degrees. 3rd layer {15, 45} degrees. 4th layer {45,75} degrees. Minimum distance in each layer will be logged.
// @Field: He: True if proximity sensor is healthy
// @Field: D0: Nearest object in sector surrounding 0-degrees
// @Field: D45: Nearest object in sector surrounding 45-degrees
// @Field: D90: Nearest object in sector surrounding 90-degrees
// @Field: D135: Nearest object in sector surrounding 135-degrees
// @Field: D180: Nearest object in sector surrounding 180-degrees
// @Field: D225: Nearest object in sector surrounding 225-degrees
// @Field: D270: Nearest object in sector surrounding 270-degrees
// @Field: D315: Nearest object in sector surrounding 315-degrees
// @Field: DUp: Nearest object in upwards direction
// @Field: CAn: Angle to closest object
// @Field: CDis: Distance to closest object

// @LoggerMessage: PRXR
// @Description: Proximity Raw sensor data
// @Field: TimeUS: Time since system startup
// @Field: Layer: Pitch(instance) at which the obstacle is at. 0th layer {-75,-45} degrees. 1st layer {-45,-15} degrees. 2nd layer {-15, 15} degrees. 3rd layer {15, 45} degrees. 4th layer {45,75} degrees. Minimum distance in each layer will be logged.
// @Field: D0: Nearest object in sector surrounding 0-degrees
// @Field: D45: Nearest object in sector surrounding 45-degrees
// @Field: D90: Nearest object in sector surrounding 90-degrees
// @Field: D135: Nearest object in sector surrounding 135-degrees
// @Field: D180: Nearest object in sector surrounding 180-degrees
// @Field: D225: Nearest object in sector surrounding 225-degrees
// @Field: D270: Nearest object in sector surrounding 270-degrees
// @Field: D315: Nearest object in sector surrounding 315-degrees

// @LoggerMessage: RAD
// @Description: Telemetry radio statistics
// @Field: TimeUS: Time since system startup
// @Field: RSSI: RSSI
// @Field: RemRSSI: RSSI reported from remote radio
// @Field: TxBuf: number of bytes in radio ready to be sent
// @Field: Noise: local noise floor
// @Field: RemNoise: local noise floor reported from remote radio
// @Field: RxErrors: damaged packet count
// @Field: Fixed: fixed damaged packet count

// @LoggerMessage: RALY
// @Description: Rally point information
// @Field: TimeUS: Time since system startup
// @Field: Tot: total number of rally points onboard
// @Field: Seq: this rally point's sequence number
// @Field: Lat: latitude of rally point
// @Field: Lng: longitude of rally point
// @Field: Alt: altitude of rally point

// @LoggerMessage: RCIN
// @Description: RC input channels to vehicle
// @Field: TimeUS: Time since system startup
// @Field: C1: channel 1 input
// @Field: C2: channel 2 input
// @Field: C3: channel 3 input
// @Field: C4: channel 4 input
// @Field: C5: channel 5 input
// @Field: C6: channel 6 input
// @Field: C7: channel 7 input
// @Field: C8: channel 8 input
// @Field: C9: channel 9 input
// @Field: C10: channel 10 input
// @Field: C11: channel 11 input
// @Field: C12: channel 12 input
// @Field: C13: channel 13 input
// @Field: C14: channel 14 input

// @LoggerMessage: RCOU
// @Description: Servo channel output values
// @Field: TimeUS: Time since system startup
// @Field: C1: channel 1 output
// @Field: C2: channel 2 output
// @Field: C3: channel 3 output
// @Field: C4: channel 4 output
// @Field: C5: channel 5 output
// @Field: C6: channel 6 output
// @Field: C7: channel 7 output
// @Field: C8: channel 8 output
// @Field: C9: channel 9 output
// @Field: C10: channel 10 output
// @Field: C11: channel 11 output
// @Field: C12: channel 12 output
// @Field: C13: channel 13 output
// @Field: C14: channel 14 output

// @LoggerMessage: RFND
// @Description: Rangefinder sensor information
// @Field: TimeUS: Time since system startup
// @Field: Instance: rangefinder instance number this data is from
// @Field: Dist: Reported distance from sensor
// @Field: Stat: Sensor state
// @Field: Orient: Sensor orientation

// @LoggerMessage: RPM
// @Description: Data from RPM sensors
// @Field: TimeUS: Time since system startup
// @Field: rpm1: First sensor's data
// @Field: rpm2: Second sensor's data

// @LoggerMessage: RSSI
// @Description: Received Signal Strength Indicator for RC receiver
// @Field: TimeUS: Time since system startup
// @Field: RXRSSI: RSSI

// @LoggerMessage: SIM
// @Description: SITL simulator state
// @Field: TimeUS: Time since system startup
// @Field: Roll: Simulated roll
// @Field: Pitch: Simulated pitch
// @Field: Yaw: Simulated yaw
// @Field: Alt: Simulated altitude
// @Field: Lat: Simulated latitude
// @Field: Lng: Simulated longitude
// @Field: Q1: Attitude quaternion component 1
// @Field: Q2: Attitude quaternion component 2
// @Field: Q3: Attitude quaternion component 3
// @Field: Q4: Attitude quaternion component 4

// @LoggerMessage: SRTL
// @Description: SmartRTL statistics
// @Field: TimeUS: Time since system startup
// @Field: Active: true if SmartRTL could be used right now
// @Field: NumPts: number of points currently in use
// @Field: MaxPts: maximum number of points that could be used
// @Field: Action: most recent internal action taken by SRTL library
// @Field: N: point associated with most recent action (North component)
// @Field: E: point associated with most recent action (East component)
// @Field: D: point associated with most recent action (Down component)

// @LoggerMessage: TERR
// @Description: Terrain database infomration
// @Field: TimeUS: Time since system startup
// @Field: Status: Terrain database status
// @Field: Lat: Current vehicle latitude
// @Field: Lng: Current vehicle longitude
// @Field: Spacing: terrain Tile spacing
// @Field: TerrH: current Terrain height
// @Field: CHeight: Vehicle height above terrain
// @Field: Pending: Number of tile requests outstanding
// @Field: Loaded: Number of tiles in memory

// @LoggerMessage: TSYN
// @Description: Time synchronisation response information
// @Field: TimeUS: Time since system startup
// @Field: SysID: system ID this data is for
// @Field: RTT: round trip time for this system

// @LoggerMessage: UNIT
// @Description: Message mapping from single character to SI unit
// @Field: TimeUS: Time since system startup
// @Field: Id: character referenced by FMTU
// @Field: Label: Unit - SI where available

// @LoggerMessage: WENC
// @Description: Wheel encoder measurements
// @Field: TimeUS: Time since system startup
// @Field: Dist0: First wheel distance travelled
// @Field: Qual0: Quality measurement of Dist0
// @Field: Dist1: Second wheel distance travelled
// @Field: Qual1: Quality measurement of Dist1

// @LoggerMessage: WINC
// @Description: Winch
// @Field: TimeUS: Time since system startup
// @Field: Heal: Healthy
// @Field: ThEnd: Reached end of thread
// @Field: Mov: Motor is moving
// @Field: Clut: Clutch is engaged (motor can move freely)
// @Field: Mode: 0 is Relaxed, 1 is Position Control, 2 is Rate Control
// @Field: DLen: Desired Length
// @Field: Len: Estimated Length
// @Field: DRate: Desired Rate
// @Field: Tens: Tension on line
// @Field: Vcc: Voltage to Motor
// @Field: Temp: Motor temperature

// @LoggerMessage: PSC
// @Description: Position Control data
// @Field: TimeUS: Time since system startup
// @Field: TPX: Target position relative to origin, X-axis
// @Field: TPY: Target position relative to origin, Y-axis
// @Field: PX: Position relative to origin, X-axis
// @Field: PY: Position relative to origin, Y-axis
// @Field: TVX: Target velocity, X-axis
// @Field: TVY: Target velocity, Y-axis
// @Field: VX: Velocity, X-axis
// @Field: VY: Velocity, Y-axis
// @Field: TAX: Target acceleration, X-axis
// @Field: TAY: Target acceleration, Y-axis
// @Field: AX: Acceleration, X-axis
// @Field: AY: Acceleration, Y-axis

// @LoggerMessage: PSCZ
// @Description: Position Control Z-axis
// @Field: TimeUS: Time since system startup
// @Field: TPZ: Target position above EKF origin
// @Field: PZ: Position above EKF origin
// @Field: DVZ: Desired velocity Z-axis
// @Field: TVZ: Target velocity Z-axis
// @Field: VZ: Velocity Z-axis
// @Field: DAZ: Desired acceleration Z-axis
// @Field: TAZ: Target acceleration Z-axis
// @Field: AZ: Acceleration Z-axis
// @Field: ThO: Throttle output

// messages for all boards
#define LOG_BASE_STRUCTURES \
    { LOG_FORMAT_MSG, sizeof(log_Format), \
      "FMT", "BBnNZ",      "Type,Length,Name,Format,Columns", "-b---", "-----" },    \
    { LOG_UNIT_MSG, sizeof(log_Unit), \
      "UNIT", "QbZ",      "TimeUS,Id,Label", "s--","F--" },    \
    { LOG_FORMAT_UNITS_MSG, sizeof(log_Format_Units), \
      "FMTU", "QBNN",      "TimeUS,FmtType,UnitIds,MultIds","s---", "F---" },   \
    { LOG_MULT_MSG, sizeof(log_Format_Multiplier), \
      "MULT", "Qbd",      "TimeUS,Id,Mult", "s--","F--" },   \
    { LOG_PARAMETER_MSG, sizeof(log_Parameter), \
     "PARM", "QNf",        "TimeUS,Name,Value", "s--", "F--"  },       \
LOG_STRUCTURE_FROM_GPS \
    { LOG_MESSAGE_MSG, sizeof(log_Message), \
      "MSG",  "QZ",     "TimeUS,Message", "s-", "F-"}, \
    { LOG_RCIN_MSG, sizeof(log_RCIN), \
      "RCIN",  "QHHHHHHHHHHHHHH",     "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14", "sYYYYYYYYYYYYYY", "F--------------" }, \
    { LOG_RCIN2_MSG, sizeof(log_RCIN2), \
      "RCI2",  "QHH",     "TimeUS,C15,C16", "sYY", "F--" }, \
    { LOG_RCOUT_MSG, sizeof(log_RCOUT), \
      "RCOU",  "QHHHHHHHHHHHHHH",     "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14", "sYYYYYYYYYYYYYY", "F--------------"  }, \
    { LOG_RSSI_MSG, sizeof(log_RSSI), \
      "RSSI",  "Qf",     "TimeUS,RXRSSI", "s-", "F-"  }, \
LOG_STRUCTURE_FROM_BARO \
LOG_STRUCTURE_FROM_PRECLAND \
    { LOG_POWR_MSG, sizeof(log_POWR), \
      "POWR","QffHHB","TimeUS,Vcc,VServo,Flags,AccFlags,Safety", "svv---", "F00---" },  \
    { LOG_CMD_MSG, sizeof(log_Cmd), \
      "CMD", "QHHHffffLLfB","TimeUS,CTot,CNum,CId,Prm1,Prm2,Prm3,Prm4,Lat,Lng,Alt,Frame", "s-------DUm-", "F-------GG0-" }, \
    { LOG_MAVLINK_COMMAND_MSG, sizeof(log_MAVLink_Command), \
      "MAVC", "QBBBHBBffffiifBB","TimeUS,TS,TC,Fr,Cmd,Cur,AC,P1,P2,P3,P4,X,Y,Z,Res,WL", "s---------------", "F---------------" }, \
    { LOG_RADIO_MSG, sizeof(log_Radio), \
      "RAD", "QBBBBBHH", "TimeUS,RSSI,RemRSSI,TxBuf,Noise,RemNoise,RxErrors,Fixed", "s-------", "F-------" }, \
LOG_STRUCTURE_FROM_CAMERA \
    { LOG_ARSP_MSG, sizeof(log_ARSP), "ARSP",  "QBffcffBBfB", "TimeUS,I,Airspeed,DiffPress,Temp,RawPress,Offset,U,H,Hfp,Pri", "s#nPOPP----", "F-00B00----" }, \
    LOG_STRUCTURE_FROM_BATTMONITOR \
    { LOG_MAG_MSG, sizeof(log_MAG), \
      "MAG", "QBhhhhhhhhhBI",    "TimeUS,I,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOX,MOY,MOZ,Health,S", "s#GGGGGGGGG-s", "F-CCCCCCCCC-F" }, \
    { LOG_MODE_MSG, sizeof(log_Mode), \
      "MODE", "QMBB",         "TimeUS,Mode,ModeNum,Rsn", "s---", "F---" }, \
    { LOG_RFND_MSG, sizeof(log_RFND), \
      "RFND", "QBCBB", "TimeUS,Instance,Dist,Stat,Orient", "s#m--", "F-B--" }, \
    { LOG_MAV_STATS, sizeof(log_MAV_Stats), \
      "DMS", "QIIIIBBBBBBBBB",         "TimeUS,N,Dp,RT,RS,Fa,Fmn,Fmx,Pa,Pmn,Pmx,Sa,Smn,Smx", "s-------------", "F-------------" }, \
    { LOG_BEACON_MSG, sizeof(log_Beacon), \
      "BCN", "QBBfffffff",  "TimeUS,Health,Cnt,D0,D1,D2,D3,PosX,PosY,PosZ", "s--mmmmmmm", "F--0000000" }, \
    { LOG_PROXIMITY_MSG, sizeof(log_Proximity), \
      "PRX", "QBBfffffffffff", "TimeUS,Layer,He,D0,D45,D90,D135,D180,D225,D270,D315,DUp,CAn,CDis", "s#-mmmmmmmmmhm", "F--00000000000" }, \
    { LOG_RAW_PROXIMITY_MSG, sizeof(log_Proximity_raw), \
      "PRXR", "QBffffffff", "TimeUS,Layer,D0,D45,D90,D135,D180,D225,D270,D315", "s#mmmmmmmm", "F-00000000" }, \
    { LOG_PERFORMANCE_MSG, sizeof(log_Performance),                     \
      "PM",  "QHHIIHHIIIIII", "TimeUS,NLon,NLoop,MaxT,Mem,Load,ErrL,IntE,ErrC,SPIC,I2CC,I2CI,Ex", "s---b%------s", "F---0A------F" }, \
    { LOG_SRTL_MSG, sizeof(log_SRTL), \
      "SRTL", "QBHHBfff", "TimeUS,Active,NumPts,MaxPts,Action,N,E,D", "s----mmm", "F----000" }, \
LOG_STRUCTURE_FROM_AVOIDANCE \
    { LOG_SIMSTATE_MSG, sizeof(log_AHRS), \
      "SIM","QccCfLLffff","TimeUS,Roll,Pitch,Yaw,Alt,Lat,Lng,Q1,Q2,Q3,Q4", "sddhmDU????", "FBBB0GG????" }, \
    { LOG_TERRAIN_MSG, sizeof(log_TERRAIN), \
      "TERR","QBLLHffHH","TimeUS,Status,Lat,Lng,Spacing,TerrH,CHeight,Pending,Loaded", "s-DU-mm--", "F-GG-00--" }, \
LOG_STRUCTURE_FROM_ESC_TELEM \
    { LOG_CSRV_MSG, sizeof(log_CSRV), \
      "CSRV","QBfffB","TimeUS,Id,Pos,Force,Speed,Pow", "s#---%", "F-0000" }, \
    { LOG_PIDR_MSG, sizeof(log_PID), \
      "PIDR", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS }, \
    { LOG_PIDP_MSG, sizeof(log_PID), \
      "PIDP", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS }, \
    { LOG_PIDY_MSG, sizeof(log_PID), \
      "PIDY", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS }, \
    { LOG_PIDA_MSG, sizeof(log_PID), \
      "PIDA", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS }, \
    { LOG_PIDS_MSG, sizeof(log_PID), \
      "PIDS", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS }, \
    { LOG_PIDN_MSG, sizeof(log_PID), \
      "PIDN", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS }, \
    { LOG_PIDE_MSG, sizeof(log_PID), \
      "PIDE", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS }, \
    { LOG_DSTL_MSG, sizeof(log_DSTL), \
      "DSTL", "QBfLLeccfeffff", "TimeUS,Stg,THdg,Lat,Lng,Alt,XT,Travel,L1I,Loiter,Des,P,I,D", "s??DUm--------", "F??000--------" }, \
LOG_STRUCTURE_FROM_INERTIALSENSOR \
LOG_STRUCTURE_FROM_DAL \
LOG_STRUCTURE_FROM_NAVEKF2 \
LOG_STRUCTURE_FROM_NAVEKF3 \
LOG_STRUCTURE_FROM_NAVEKF \
LOG_STRUCTURE_FROM_AHRS \
    { LOG_DF_FILE_STATS, sizeof(log_DSF), \
      "DSF", "QIHIIII", "TimeUS,Dp,Blk,Bytes,FMn,FMx,FAv", "s--b---", "F--0---" }, \
    { LOG_RPM_MSG, sizeof(log_RPM), \
      "RPM",  "Qff", "TimeUS,rpm1,rpm2", "sqq", "F00" }, \
    { LOG_RALLY_MSG, sizeof(log_Rally), \
      "RALY", "QBBLLh", "TimeUS,Tot,Seq,Lat,Lng,Alt", "s--DUm", "F--GGB" },  \
    { LOG_MAV_MSG, sizeof(log_MAV),   \
      "MAV", "QBHHHBHH",   "TimeUS,chan,txp,rxp,rxdp,flags,ss,tf", "s#----s-", "F-000-C-" },   \
LOG_STRUCTURE_FROM_VISUALODOM \
    { LOG_OPTFLOW_MSG, sizeof(log_Optflow), \
      "OF",   "QBffff",   "TimeUS,Qual,flowX,flowY,bodyX,bodyY", "s-EEnn", "F-0000" }, \
    { LOG_WHEELENCODER_MSG, sizeof(log_WheelEncoder), \
      "WENC",  "Qfbfb", "TimeUS,Dist0,Qual0,Dist1,Qual1", "sm-m-", "F0-0-" }, \
    { LOG_ADSB_MSG, sizeof(log_ADSB), \
      "ADSB",  "QIiiiHHhH", "TimeUS,ICAO_address,Lat,Lng,Alt,Heading,Hor_vel,Ver_vel,Squark", "s-DUmhnn-", "F-GGCBCC-" }, \
    { LOG_EVENT_MSG, sizeof(log_Event), \
      "EV",   "QB",           "TimeUS,Id", "s-", "F-" }, \
    { LOG_ARM_DISARM_MSG, sizeof(log_Arm_Disarm), \
      "ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method", "s----", "F----" }, \
    { LOG_ERROR_MSG, sizeof(log_Error), \
      "ERR",   "QBB",         "TimeUS,Subsys,ECode", "s--", "F--" }, \
    { LOG_WINCH_MSG, sizeof(log_Winch), \
      "WINC", "QBBBBBfffHfb", "TimeUS,Heal,ThEnd,Mov,Clut,Mode,DLen,Len,DRate,Tens,Vcc,Temp", "s-----mmn?vO", "F-----000000" }, \
    { LOG_PSC_MSG, sizeof(log_PSC), \
      "PSC", "Qffffffffffff", "TimeUS,TPX,TPY,PX,PY,TVX,TVY,VX,VY,TAX,TAY,AX,AY", "smmmmnnnnoooo", "F000000000000" }, \
    { LOG_PSCZ_MSG, sizeof(log_PSCZ), \
      "PSCZ", "Qfffffffff", "TimeUS,TPZ,PZ,DVZ,TVZ,VZ,DAZ,TAZ,AZ,ThO", "smmnnnooo%", "F000000002" }

// @LoggerMessage: SBPH
// @Description: Swift Health Data
// @Field: TimeUS: Time since system startup
// @Field: CrcError: Number of packet CRC errors on serial connection
// @Field: LastInject: Timestamp of last raw data injection to GPS
// @Field: IARhyp: Current number of integer ambiguity hypotheses

// @LoggerMessage: SBRH
// @Description: Swift Raw Message Data
// @Field: TimeUS: Time since system startup
// @Field: msg_flag: Swift message type
// @Field: 1: Sender ID
// @Field: 2: index; always 1
// @Field: 3: pages; number of pages received
// @Field: 4: msg length; number of bytes received
// @Field: 5: unused; always zero
// @Field: 6: data received from device

#define LOG_SBP_STRUCTURES \
    { LOG_MSG_SBPHEALTH, sizeof(log_SbpHealth), \
      "SBPH", "QIII", "TimeUS,CrcError,LastInject,IARhyp", "s---", "F---" }, \
    { LOG_MSG_SBPRAWH, sizeof(log_SbpRAWH), \
      "SBRH", "QQQQQQQQ", "TimeUS,msg_flag,1,2,3,4,5,6", "s--b----", "F--0----" }, \
    { LOG_MSG_SBPRAWM, sizeof(log_SbpRAWM), \
      "SBRM", "QQQQQQQQQQQQQQQ", "TimeUS,msg_flag,1,2,3,4,5,6,7,8,9,10,11,12,13", "s??????????????", "F??????????????" }, \
    { LOG_MSG_SBPEVENT, sizeof(log_SbpEvent), \
      "SBRE", "QHIiBB", "TimeUS,GWk,GMS,ns_residual,level,quality", "s?????", "F?????" }

#define LOG_COMMON_STRUCTURES LOG_BASE_STRUCTURES, LOG_SBP_STRUCTURES

// message types 0 to 63 reserved for vehicle specific use

// message types for common messages
enum LogMessages : uint8_t {
    LOG_PARAMETER_MSG = 64,
    LOG_IDS_FROM_NAVEKF2,
    LOG_IDS_FROM_NAVEKF3,
    LOG_MESSAGE_MSG,
    LOG_RCIN_MSG,
    LOG_RCIN2_MSG,
    LOG_RCOUT_MSG,
    LOG_RSSI_MSG,
    LOG_IDS_FROM_BARO,
    LOG_POWR_MSG,
    LOG_IDS_FROM_AHRS,
    LOG_SIMSTATE_MSG,
    LOG_CMD_MSG,
    LOG_MAVLINK_COMMAND_MSG,
    LOG_RADIO_MSG,
    LOG_ATRP_MSG,
    LOG_IDS_FROM_CAMERA,
    LOG_TERRAIN_MSG,
    LOG_CSRV_MSG,
    LOG_ARSP_MSG,
    LOG_IDS_FROM_ESC_TELEM,
    LOG_IDS_FROM_BATTMONITOR,
    LOG_MAG_MSG,

    LOG_IDS_FROM_GPS,

    // LOG_MODE_MSG is used as a check for duplicates. Do not add between this and LOG_FORMAT_MSG
    LOG_MODE_MSG,

    LOG_FORMAT_MSG = 128, // this must remain #128

    LOG_IDS_FROM_DAL,
    LOG_IDS_FROM_INERTIALSENSOR,

    LOG_PIDR_MSG,
    LOG_PIDP_MSG,
    LOG_PIDY_MSG,
    LOG_PIDA_MSG,
    LOG_PIDS_MSG,
    LOG_PIDN_MSG,
    LOG_PIDE_MSG,
    LOG_DSTL_MSG,
    LOG_RPM_MSG,
    LOG_RFND_MSG,
    LOG_MAV_STATS,
    LOG_FORMAT_UNITS_MSG,
    LOG_UNIT_MSG,
    LOG_MULT_MSG,

    LOG_MSG_SBPHEALTH,
    LOG_MSG_SBPLLH,
    LOG_MSG_SBPBASELINE,
    LOG_MSG_SBPTRACKING1,
    LOG_MSG_SBPTRACKING2,
    LOG_MSG_SBPRAWH,
    LOG_MSG_SBPRAWM,
    LOG_MSG_SBPEVENT,

    LOG_RALLY_MSG,
    LOG_IDS_FROM_VISUALODOM,
    LOG_BEACON_MSG,
    LOG_PROXIMITY_MSG,
    LOG_DF_FILE_STATS,
    LOG_SRTL_MSG,
    LOG_PERFORMANCE_MSG,
    LOG_OPTFLOW_MSG,
    LOG_EVENT_MSG,
    LOG_WHEELENCODER_MSG,
    LOG_MAV_MSG,
    LOG_ERROR_MSG,
    LOG_ADSB_MSG,
    LOG_ARM_DISARM_MSG,
    LOG_IDS_FROM_AVOIDANCE,
    LOG_WINCH_MSG,
    LOG_PSC_MSG,
    LOG_PSCZ_MSG,
    LOG_RAW_PROXIMITY_MSG,
    LOG_IDS_FROM_PRECLAND,

    _LOG_LAST_MSG_
};

static_assert(_LOG_LAST_MSG_ <= 255, "Too many message formats");
static_assert(LOG_MODE_MSG < 128, "Duplicate message format IDs");

enum LogOriginType {
    ekf_origin = 0,
    ahrs_home = 1
};
