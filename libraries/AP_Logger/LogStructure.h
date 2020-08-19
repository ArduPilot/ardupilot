#pragma once

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
//    { 'W', "Watt" },        // Watt
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

struct PACKED log_GPS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  status;
    uint32_t gps_week_ms;
    uint16_t gps_week;
    uint8_t  num_sats;
    uint16_t hdop;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  altitude;
    float    ground_speed;
    float    ground_course;
    float    vel_z;
    float    yaw;
    uint8_t  used;
};

struct PACKED log_GPA {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t vdop;
    uint16_t hacc;
    uint16_t vacc;
    uint16_t sacc;
    float    yaw_accuracy;
    uint8_t  have_vv;
    uint32_t sample_ms;
    uint16_t delta_ms;
};

struct PACKED log_Message {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    char msg[64];
};

struct PACKED log_IMU {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    uint32_t gyro_error, accel_error;
    float temperature;
    uint8_t gyro_health, accel_health;
    uint16_t gyro_rate, accel_rate;
};

struct PACKED log_IMUDT {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float delta_time, delta_vel_dt, delta_ang_dt;
    float delta_ang_x, delta_ang_y, delta_ang_z;
    float delta_vel_x, delta_vel_y, delta_vel_z;
};

struct PACKED log_ISBH {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t seqno;
    uint8_t sensor_type; // e.g. GYRO or ACCEL
    uint8_t instance;
    uint16_t multiplier;
    uint16_t sample_count;
    uint64_t sample_us;
    float sample_rate_hz;
};
static_assert(sizeof(log_ISBH) < 256, "log_ISBH is over-size");

struct PACKED log_ISBD {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t isb_seqno;
    uint16_t seqno; // seqno within isb_seqno
    int16_t x[32];
    int16_t y[32];
    int16_t z[32];
};
static_assert(sizeof(log_ISBD) < 256, "log_ISBD is over-size");

struct PACKED log_Vibe {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float vibe_x, vibe_y, vibe_z;
    uint32_t clipping_0, clipping_1, clipping_2;
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

struct PACKED log_BARO {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float   altitude;
    float   pressure;
    int16_t temperature;
    float   climbrate;
    uint32_t sample_time_ms;
    float   drift_offset;
    float   ground_temp;
    uint8_t healthy;
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

struct PACKED log_AHRS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t roll;
    int16_t pitch;
    uint16_t yaw;
    float alt;
    int32_t lat;
    int32_t lng;
    float q1, q2, q3, q4;
};

struct PACKED log_POS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t lat;
    int32_t lng;
    float alt;
    float rel_home_alt;
    float rel_origin_alt;
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

struct PACKED log_EKF1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    int16_t roll;
    int16_t pitch;
    uint16_t yaw;
    float velN;
    float velE;
    float velD;
    float posD_dot;
    float posN;
    float posE;
    float posD;
    int16_t gyrX;
    int16_t gyrY;
    int16_t gyrZ;
    int32_t originHgt;
};

struct PACKED log_EKF2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    int8_t Ratio;
    int8_t AZ1bias;
    int8_t AZ2bias;
    int16_t windN;
    int16_t windE;
    int16_t magN;
    int16_t magE;
    int16_t magD;
    int16_t magX;
    int16_t magY;
    int16_t magZ;
};

struct PACKED log_NKF2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    int8_t AZbias;
    int16_t scaleX;
    int16_t scaleY;
    int16_t scaleZ;
    int16_t windN;
    int16_t windE;
    int16_t magN;
    int16_t magE;
    int16_t magD;
    int16_t magX;
    int16_t magY;
    int16_t magZ;
    uint8_t index;
};

struct PACKED log_XKF2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    int16_t accBiasX;
    int16_t accBiasY;
    int16_t accBiasZ;
    int16_t windN;
    int16_t windE;
    int16_t magN;
    int16_t magE;
    int16_t magD;
    int16_t magX;
    int16_t magY;
    int16_t magZ;
    uint8_t index;
};

struct PACKED log_EKF3 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    int16_t innovVN;
    int16_t innovVE;
    int16_t innovVD;
    int16_t innovPN;
    int16_t innovPE;
    int16_t innovPD;
    int16_t innovMX;
    int16_t innovMY;
    int16_t innovMZ;
    int16_t innovVT;
};

struct PACKED log_NKF3 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    int16_t innovVN;
    int16_t innovVE;
    int16_t innovVD;
    int16_t innovPN;
    int16_t innovPE;
    int16_t innovPD;
    int16_t innovMX;
    int16_t innovMY;
    int16_t innovMZ;
    int16_t innovYaw;
    int16_t innovVT;
};

struct PACKED log_EKF4 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    int16_t sqrtvarV;
    int16_t sqrtvarP;
    int16_t sqrtvarH;
    int16_t sqrtvarMX;
    int16_t sqrtvarMY;
    int16_t sqrtvarMZ;
    int16_t sqrtvarVT;
    int8_t  offsetNorth;
    int8_t  offsetEast;
    uint16_t faults;
    uint8_t timeouts;
    uint16_t solution;
    uint16_t gps;
};

struct PACKED log_NKF4 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    int16_t sqrtvarV;
    int16_t sqrtvarP;
    int16_t sqrtvarH;
    int16_t sqrtvarM;
    int16_t sqrtvarVT;
    float   tiltErr;
    int8_t  offsetNorth;
    int8_t  offsetEast;
    uint16_t faults;
    uint8_t timeouts;
    uint32_t solution;
    uint16_t gps;
    int8_t primary;
};

struct PACKED log_EKF5 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t normInnov;
    int16_t FIX;
    int16_t FIY;
    int16_t AFI;
    int16_t HAGL;
    int16_t offset;
    int16_t RI;
    uint16_t meaRng;
    uint16_t errHAGL;
};

struct PACKED log_NKF5 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t normInnov;
    int16_t FIX;
    int16_t FIY;
    int16_t AFI;
    int16_t HAGL;
    int16_t offset;
    int16_t RI;
    uint16_t meaRng;
    uint16_t errHAGL;
    float angErr;
    float velErr;
    float posErr;
};

struct PACKED log_Quaternion {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    float q1;
    float q2;
    float q3;
    float q4;
};

struct PACKED log_RngBcnDebug {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t ID;             // beacon identifier
    int16_t rng;            // beacon range (cm)
    int16_t innov;          // beacon range innovation (cm)
    uint16_t sqrtInnovVar;  // sqrt of beacon range innovation variance (cm)
    uint16_t testRatio;     // beacon range innovation consistency test ratio *100
    int16_t beaconPosN;     // beacon north position (cm)
    int16_t beaconPosE;     // beacon east position (cm)
    int16_t beaconPosD;     // beacon down position (cm)
    int16_t offsetHigh;     // high estimate of vertical position offset of beacons rel to EKF origin (cm)
    int16_t offsetLow;      // low estimate of vertical position offset of beacons rel to EKF origin (cm)
    int16_t posN;           // North position of receiver rel to EKF origin (cm)
    int16_t posE;           // East position of receiver rel to EKF origin (cm)
    int16_t posD;           // Down position of receiver rel to EKF origin (cm)
};

// visual odometry sensor data
struct PACKED log_VisualOdom {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float time_delta;
    float angle_delta_x;
    float angle_delta_y;
    float angle_delta_z;
    float position_delta_x;
    float position_delta_y;
    float position_delta_z;
    float confidence;
};

// visual position data
struct PACKED log_VisualPosition {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t remote_time_us;
    uint32_t time_ms;
    float pos_x;
    float pos_y;
    float pos_z;
    float roll;     // degrees
    float pitch;    // degrees
    float yaw;      // degrees
    float pos_err;  // meters
    float ang_err;  // radians
    uint8_t reset_counter;
};

struct PACKED log_VisualVelocity {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t remote_time_us;
    uint32_t time_ms;
    float vel_x;
    float vel_y;
    float vel_z;
    float vel_err;
    uint8_t reset_counter;
};

struct PACKED log_ekfBodyOdomDebug {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float velInnovX;
    float velInnovY;
    float velInnovZ;
    float velInnovVarX;
    float velInnovVarY;
    float velInnovVarZ;
};

struct PACKED log_ekfStateVar {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float v00;
    float v01;
    float v02;
    float v03;
    float v04;
    float v05;
    float v06;
    float v07;
    float v08;
    float v09;
    float v10;
    float v11;
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

struct PACKED log_Camera {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t gps_time;
    uint16_t gps_week;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  altitude;
    int32_t  altitude_rel;
    int32_t  altitude_gps;
    int16_t  roll;
    int16_t  pitch;
    uint16_t yaw;
};

struct PACKED log_Attitude {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t  control_roll;
    int16_t  roll;
    int16_t  control_pitch;
    int16_t  pitch;
    uint16_t control_yaw;
    uint16_t yaw;
    uint16_t error_rp;
    uint16_t error_yaw;
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
};

struct PACKED log_Current {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    float    voltage;
    float    voltage_resting;
    float    current_amps;
    float    current_total;
    float    consumed_wh;
    int16_t  temperature; // degrees C * 100
    float    resistance;
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

struct PACKED log_Current_Cells {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    float    voltage;
    uint16_t cell_voltages[12];
};

struct PACKED log_Compass {
    LOG_PACKET_HEADER;
    uint64_t time_us;
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

/*
  UBlox logging
 */
struct PACKED log_Ubx1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    uint16_t noisePerMS;
    uint8_t  jamInd;
    uint8_t  aPower;
    uint16_t agcCnt;
    uint32_t config;
};

struct PACKED log_Ubx2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    int8_t   ofsI;
    uint8_t  magI;
    int8_t   ofsQ;
    uint8_t  magQ;
};

struct PACKED log_GPS_RAW {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t iTOW;
    int16_t week;
    uint8_t numSV;
    uint8_t sv;
    double cpMes;
    double prMes;
    float doMes;
    int8_t mesQI;
    int8_t cno;
    uint8_t lli;
};

struct PACKED log_GPS_RAWH {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    double rcvTow;
    uint16_t week;
    int8_t leapS;
    uint8_t numMeas;
    uint8_t recStat;
};

struct PACKED log_GPS_RAWS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    double prMes;
    double cpMes;
    float doMes;
    uint8_t gnssId;
    uint8_t svId;
    uint8_t freqId;
    uint16_t locktime;
    uint8_t cno;
    uint8_t prStdev;
    uint8_t cpStdev;
    uint8_t doStdev;
    uint8_t trkStat;
};

struct PACKED log_Esc {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    int32_t rpm;
    uint16_t voltage;
    uint16_t current;
    int16_t esc_temp;
    uint16_t current_tot;
    int16_t motor_temp;
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

struct PACKED log_CESC {
    LOG_PACKET_HEADER;
    uint64_t time_us;     
    uint8_t id;
    uint32_t error_count;
    float voltage;
    float current;
    float temperature;
    int32_t rpm;
    uint8_t power_pct;
};

struct PACKED log_AIRSPEED {
    LOG_PACKET_HEADER;
    uint64_t time_us;
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

struct PACKED log_ACCEL {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t sample_us;
    float AccX, AccY, AccZ;
};

struct PACKED log_GYRO {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t sample_us;
    float GyrX, GyrY, GyrZ;
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

struct PACKED log_ORGN {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t origin_type;
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
};

struct PACKED log_RPM {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float rpm1;
    float rpm2;
};

struct PACKED log_Rate {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float   control_roll;
    float   roll;
    float   roll_out;
    float   control_pitch;
    float   pitch;
    float   pitch_out;
    float   control_yaw;
    float   yaw;
    float   yaw_out;
    float   control_accel;
    float   accel;
    float   accel_out;
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

struct PACKED log_AOA_SSA {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float AOA;
    float SSA;
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

struct PACKED log_Performance {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t num_long_running;
    uint16_t num_loops;
    uint32_t max_time;
    uint32_t mem_avail;
    uint16_t load;
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

struct PACKED log_OABendyRuler {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t active;
    uint16_t target_yaw;
    uint16_t yaw;
    bool resist_chg;
    float margin;
    int32_t final_lat;
    int32_t final_lng;
    int32_t oa_lat;
    int32_t oa_lng;
};

struct PACKED log_OADijkstra {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t state;
    uint8_t error_id;
    uint8_t curr_point;
    uint8_t tot_points;
    int32_t final_lat;
    int32_t final_lng;
    int32_t oa_lat;
    int32_t oa_lng;
};

struct PACKED log_SimpleAvoid {
  LOG_PACKET_HEADER;
  uint64_t time_us;
  uint8_t state;
  float desired_vel_x;
  float desired_vel_y;
  float modified_vel_x;
  float modified_vel_y;
  uint8_t backing_up;
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

// FMT messages define all message formats other than FMT
// UNIT messages define units which can be referenced by FMTU messages
// FMTU messages associate types (e.g. centimeters/second/second) to FMT message fields

#define ACC_LABELS "TimeUS,SampleUS,AccX,AccY,AccZ"
#define ACC_FMT   "QQfff"
#define ACC_UNITS "ssnnn"
#define ACC_MULTS "FF000"

// see "struct sensor" in AP_Baro.h and "Write_Baro":
#define BARO_LABELS "TimeUS,Alt,Press,Temp,CRt,SMS,Offset,GndTemp,Health"
#define BARO_FMT   "QffcfIffB"
#define BARO_UNITS "smPOnsmO-"
#define BARO_MULTS "F00B0C?0-"

#define GPA_LABELS "TimeUS,VDop,HAcc,VAcc,SAcc,YAcc,VV,SMS,Delta"
#define GPA_FMT   "QCCCCfBIH"
#define GPA_UNITS "smmmnd-ss"
#define GPA_MULTS "FBBBB0-CC"

// see "struct GPS_State" and "Write_GPS":
#define GPS_LABELS "TimeUS,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ,Yaw,U"
#define GPS_FMT   "QBIHBcLLeffffB"
#define GPS_UNITS "s---SmDUmnhnh-"
#define GPS_MULTS "F---0BGGB000--"

#define GYR_LABELS "TimeUS,SampleUS,GyrX,GyrY,GyrZ"
#define GYR_FMT    "QQfff"
#define GYR_UNITS  "ssEEE"
#define GYR_MULTS  "FF000"

#define IMT_LABELS "TimeUS,DelT,DelvT,DelaT,DelAX,DelAY,DelAZ,DelVX,DelVY,DelVZ"
#define IMT_FMT    "Qfffffffff"
#define IMT_UNITS  "ssssrrrnnn"
#define IMT_MULTS  "FF00000000"

#define ISBH_LABELS "TimeUS,N,type,instance,mul,smp_cnt,SampleUS,smp_rate"
#define ISBH_FMT    "QHBBHHQf"
#define ISBH_UNITS  "s-----sz"
#define ISBH_MULTS  "F-----F-"

#define ISBD_LABELS "TimeUS,N,seqno,x,y,z"
#define ISBD_FMT    "QHHaaa"
#define ISBD_UNITS  "s--ooo"
#define ISBD_MULTS  "F--???"

#define IMU_LABELS "TimeUS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,EG,EA,T,GH,AH,GHz,AHz"
#define IMU_FMT   "QffffffIIfBBHH"
#define IMU_UNITS "sEEEooo--O--zz"
#define IMU_MULTS "F000000-----00"

#define MAG_LABELS "TimeUS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ,Health,S"
#define MAG_FMT   "QhhhhhhhhhBI"
#define MAG_UNITS "sGGGGGGGGG-s"
#define MAG_MULTS "FCCCCCCCCC-F"

#define PID_LABELS "TimeUS,Tar,Act,Err,P,I,D,FF,Dmod"
#define PID_FMT    "Qffffffff"
#define PID_UNITS  "s--------"
#define PID_MULTS  "F--------"

#define QUAT_LABELS "TimeUS,C,Q1,Q2,Q3,Q4"
#define QUAT_FMT    "QBffff"
#define QUAT_UNITS  "s#????"
#define QUAT_MULTS  "F-????"

#define ARSP_LABELS "TimeUS,Airspeed,DiffPress,Temp,RawPress,Offset,U,Health,Hfp,Pri"
#define ARSP_FMT "QffcffBBfB"
#define ARSP_UNITS "snPOPP----"
#define ARSP_MULTS "F00B00----"

// @LoggerMessage: ACC1,ACC2,ACC3
// @Description: IMU accelerometer data
// @Field: TimeUS: Time since system startup
// @Field: SampleUS: time since system startup this sample was taken
// @Field: AccX: acceleration along X axis
// @Field: AccY: acceleration along Y axis
// @Field: AccZ: acceleration along Z axis

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

// @LoggerMessage: AHR2
// @Description: Backup AHRS data
// @Field: TimeUS: Time since system startup
// @Field: Roll: Estimated roll
// @Field: Pitch: Estimated pitch
// @Field: Yaw: Estimated yaw
// @Field: Alt: Estimated altitude
// @Field: Lat: Estimated latitude
// @Field: Lng: Estimated longitude
// @Field: Q1: Estimated attitude quaternion component 1
// @Field: Q2: Estimated attitude quaternion component 2
// @Field: Q3: Estimated attitude quaternion component 3
// @Field: Q4: Estimated attitude quaternion component 4

// @LoggerMessage: ARM
// @Description: Arming status changes
// @Field: TimeUS: Time since system startup
// @Field: ArmState: true if vehicle is now armed
// @Field: ArmChecks: arming bitmask at time of arming
// @Field: Forced: true if arm/disarm was forced
// @Field: Method: method used for arming

// @LoggerMessage: ARSP,ASP2
// @Description: Airspeed sensor data
// @Field: TimeUS: Time since system startup
// @Field: Airspeed: Current airspeed
// @Field: DiffPress: Pressure difference between static and dynamic port
// @Field: Temp: Temperature used for calculation
// @Field: RawPress: Raw pressure less offset
// @Field: Offset: Offset from parameter
// @Field: U: True if sensor is being used
// @Field: Health: True if sensor is healthy
// @Field: Hfp: Probability sensor has failed
// @Field: Pri: True if sensor is the primary sensor

// @LoggerMessage: ATT
// @Description: Canonical vehicle attitude
// @Field: TimeUS: Time since system startup
// @Field: DesRoll: vehicle desired roll
// @Field: Roll: achieved vehicle roll
// @Field: DesPitch: vehicle desired pitch
// @Field: Pitch: achieved vehicle pitch
// @Field: DesYaw: vehicle desired yaw
// @Field: Yaw: achieved vehicle yaw
// @Field: ErrRP: lowest estimated gyro drift error
// @Field: ErrYaw: difference between measured yaw and DCM yaw estimate

// @LoggerMessage: BARO,BAR2,BAR3
// @Description: Gathered Barometer data
// @Field: TimeUS: Time since system startup
// @Field: Alt: calculated altitude
// @Field: Press: measured atmospheric pressure
// @Field: Temp: measured atmospheric temperature
// @Field: CRt: derived climb rate from primary barometer
// @Field: SMS: time last sample was taken
// @Field: Offset: raw adjustment of barometer altitude, zeroed on calibration, possibly set by GCS
// @Field: GndTemp: temperature on ground, specified by parameter or measured while on ground
// @Field: Health: true if barometer is considered healthy

// @LoggerMessage: BAT
// @Description: Gathered battery data
// @Field: TimeUS: Time since system startup
// @Field: Instance: battery instance number
// @Field: Volt: measured voltage
// @Field: VoltR: estimated resting voltage
// @Field: Curr: measured current
// @Field: CurrTot: current * time
// @Field: EnrgTot: energy this battery has produced
// @Field: Temp: measured temperature
// @Field: Res: estimated temperature resistance

// @LoggerMessage: BCL
// @Description: Battery cell voltage information
// @Field: TimeUS: Time since system startup
// @Field: Instance: battery instance number
// @Field: Volt: battery voltage
// @Field: V1: first cell voltage
// @Field: V2: second cell voltage
// @Field: V3: third cell voltage
// @Field: V4: fourth cell voltage
// @Field: V5: fifth cell voltage
// @Field: V6: sixth cell voltage
// @Field: V7: seventh cell voltage
// @Field: V8: eighth cell voltage
// @Field: V9: ninth cell voltage
// @Field: V10: tenth cell voltage
// @Field: V11: eleventh cell voltage
// @Field: V12: twelfth cell voltage

// @LoggerMessage: BCN
// @Description: Beacon informtaion
// @Field: TimeUS: Time since system startup
// @Field: Health: True if beacon sensor is healthy
// @Field: Cnt: Number of beacons being used
// @Field: D0: Distance to first beacon
// @Field: D1: Distance to second beacon
// @Field: D2: Distance to third beacon
// @Field: D3: Distance to fouth beacon
// @Field: PosX: Calculated beacon position, x-axis
// @Field: PosY: Calculated beacon position, y-axis
// @Field: PosZ: Calculated beacon position, z-axis

// @LoggerMessage: CAM,TRIG
// @Description: Camera shutter information
// @Field: TimeUS: Time since system startup
// @Field: GPSTime: milliseconds since start of GPS week
// @Field: GPSWeek: weeks since 5 Jan 1980
// @Field: Lat: current latitude
// @Field: Lng: current longitude
// @Field: Alt: current altitude
// @Field: RelAlt: current altitude relative to home
// @Field: GPSAlt: altitude as reported by GPS
// @Field: Roll: current vehicle roll
// @Field: Pitch: current vehicle pitch
// @Field: Yaw: current vehicle yaw

// @LoggerMessage: CESC
// @Description: CAN ESC data
// @Field: TimeUS: Time since system startup
// @Field: Id: ESC identifier
// @Field: ECnt: Error count
// @Field: Voltage: Battery voltage measurement
// @Field: Curr: Battery current measurement
// @Field: Temp: Temperature
// @Field: RPM: Measured RPM
// @Field: Pow: Rated power output

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

// @LoggerMessage: ESC
// @Description: Feedback received from ESCs
// @Field: TimeUS: microseconds since system startup
// @Field: Instance: ESC instance number
// @Field: RPM: reported motor rotation rate
// @Field: Volt: Perceived input voltage for the ESC
// @Field: Curr: Perceived current through the ESC
// @Field: Temp: ESC temperature
// @Field: CTot: current consumed total
// @Field: MotTemp: measured motor temperature

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

// @LoggerMessage: GPA,GPA2
// @Description: GPS accuracy information
// @Field: TimeUS: Time since system startup
// @Field: VDop: vertical degree of procession
// @Field: HAcc: horizontal position accuracy
// @Field: VAcc: vertical position accuracy
// @Field: SAcc: speed accuracy
// @Field: YAcc: yaw accuracy
// @Field: VV: true if vertical velocity is available
// @Field: SMS: time since system startup this sample was taken
// @Field: Delta: system time delta between the last two reported positions

//note: GPAB is a copy of GPA and GPA2!

// @LoggerMessage: GPAB
// @Description: Blended GPS accuracy information
// @Field: TimeUS: Time since system startup
// @Field: VDop: vertical degree of procession
// @Field: HAcc: horizontal position accuracy
// @Field: VAcc: vertical position accuracy
// @Field: SAcc: speed accuracy
// @Field: YAcc: yaw accuracy
// @Field: VV: true if vertical velocity is available
// @Field: SMS: time since system startup this sample was taken
// @Field: Delta: system time delta between the last two reported positions

// @LoggerMessage: GPS,GPS2
// @Description: Information received from GNSS systems attached to the autopilot
// @Field: TimeUS: Time since system startup
// @Field: Status: GPS Fix type; 2D fix, 3D fix etc.
// @Field: GMS: milliseconds since start of GPS Week
// @Field: GWk: weeks since 5 Jan 1980
// @Field: NSats: number of satellites visible
// @Field: HDop: horizontal precision
// @Field: Lat: latitude
// @Field: Lng: longitude
// @Field: Alt: altitude
// @Field: Spd: ground speed
// @Field: GCrs: ground course
// @Field: VZ: vertical speed
// @Field: Yaw: vehicle yaw
// @Field: U: boolean value indicating whether this GPS is in use

// Note: GPSB is a copy of GPS!

// @LoggerMessage: GPSB
// @Description: Information blended from GNSS systems attached to the autopilot
// @Field: TimeUS: Time since system startup
// @Field: Status: GPS Fix type; 2D fix, 3D fix etc.
// @Field: GMS: milliseconds since start of GPS Week
// @Field: GWk: weeks since 5 Jan 1980
// @Field: NSats: number of satellites visible
// @Field: HDop: horizontal precision
// @Field: Lat: latitude
// @Field: Lng: longitude
// @Field: Alt: altitude
// @Field: Spd: ground speed
// @Field: GCrs: ground course
// @Field: VZ: vertical speed
// @Field: Yaw: vehicle yaw
// @Field: U: boolean value indicating whether this GPS is in use

// @LoggerMessage: GRAW
// @Description: Raw uBlox data
// @Field: TimeUS: Time since system startup
// @Field: WkMS: receiver TimeOfWeek measurement
// @Field: Week: GPS week
// @Field: numSV: number of space vehicles seen
// @Field: sv: space vehicle number of first vehicle
// @Field: cpMes: carrier phase measurement
// @Field: prMes: pseudorange measurement
// @Field: doMes: Doppler measurement
// @Field: mesQI: measurement quality index
// @Field: cno: carrier-to-noise density ratio
// @Field: lli: loss of lock indicator

// @LoggerMessage: GRXH
// @Description: Raw uBlox data - header
// @Field: TimeUS: Time since system startup
// @Field: rcvTime: receiver TimeOfWeek measurement
// @Field: week: GPS week
// @Field: leapS: GPS leap seconds
// @Field: numMeas: number of space-vehicle measurements to follow
// @Field: recStat: receiver tracking status bitfield

// @LoggerMessage: GRXS
// @Description: Raw uBlox data - space-vehicle data
// @Field: TimeUS: Time since system startup
// @Field: prMes: Pseudorange measurement
// @Field: cpMes: Carrier phase measurement
// @Field: doMes: Doppler measurement
// @Field: gnss: GNSS identifier
// @Field: sv: Satellite identifier
// @Field: freq: GLONASS frequency slot
// @Field: lock: carrier phase locktime counter
// @Field: cno: carrier-to-noise density ratio
// @Field: prD: estimated pseudorange measurement standard deviation
// @Field: cpD: estimated carrier phase measurement standard deviation
// @Field: doD: estimated Doppler measurement standard deviation
// @Field: trk: tracking status bitfield

// @LoggerMessage: GYR1,GYR2,GYR3
// @Description: IMU gyroscope data
// @Field: TimeUS: Time since system startup
// @Field: SampleUS: time since system startup this sample was taken
// @Field: GyrX: measured rotation rate about X axis
// @Field: GyrY: measured rotation rate about Y axis
// @Field: GyrZ: measured rotation rate about Z axis

// @LoggerMessage: IMT,IMT2,IMT3
// @Description: Inertial Measurement Unit timing data
// @Field: TimeUS: Time since system startup
// @Field: DelT: Delta time
// @Field: DelvT: Delta velocity accumulation time
// @Field: DelaT: Delta angle accumulation time
// @Field: DelAX: Accumulated delta angle X
// @Field: DelAY: Accumulated delta angle Y
// @Field: DelAZ: Accumulated delta angle Z
// @Field: DelVX: Accumulated delta velocity X
// @Field: DelVY: Accumulated delta velocity Y
// @Field: DelVZ: Accumulated delta velocity Z

// @LoggerMessage: IMU,IMU2,IMU3
// @Description: Inertial Measurement Unit data
// @Field: TimeUS: Time since system startup
// @Field: GyrX: measured rotation rate about X axis
// @Field: GyrY: measured rotation rate about Y axis
// @Field: GyrZ: measured rotation rate about Z axis
// @Field: AccX: acceleration along X axis
// @Field: AccY: acceleration along Y axis
// @Field: AccZ: acceleration along Z axis
// @Field: EG: gyroscope error count
// @Field: EA: accelerometer error count
// @Field: T: IMU temperature
// @Field: GH: gyroscope health
// @Field: AH: accelerometer health
// @Field: GHz: gyroscope measurement rate
// @Field: AHz: accelerometer measurement rate

// @LoggerMessage: LGR
// @Description: Landing gear information
// @Field: TimeUS: Time since system startup
// @Field: LandingGear: Current landing gear state
// @Field: WeightOnWheels: True if there is weight on wheels

// @LoggerMessage: MAG,MAG2,MAG3
// @Description: Information received from compasses
// @Field: TimeUS: Time since system startup
// @Field: MagX: magnetic field strength in body frame
// @Field: MagY: magnetic field strength in body frame
// @Field: MagZ: magnetic field strength in body frame
// @Field: OfsX: magnetic field offset in body frame
// @Field: OfsY: magnetic field offset in body frame
// @Field: OfsZ: magnetic field offset in body frame
// @Field: MOfsX: motor interference magnetic field offset in body frame
// @Field: MOfsY: motor interference magnetic field offset in body frame
// @Field: MOfsZ: motor interference magnetic field offset in body frame
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

// @LoggerMessage: NKF0
// @Description: EKF2 beacon sensor diagnostics
// @Field: TimeUS: Time since system startup
// @Field: ID: Beacon sensor ID
// @Field: rng: Beacon range
// @Field: innov: Beacon range innovation
// @Field: SIV: sqrt of beacon range innovation variance
// @Field: TR: Beacon range innovation consistency test ratio
// @Field: BPN: Beacon north position
// @Field: BPE: Beacon east position
// @Field: BPD: Beacon down position
// @Field: OFH: High estimate of vertical position offset of beacons rel to EKF origin
// @Field: OFL: Low estimate of vertical position offset of beacons rel to EKF origin
// @Field: OFN: always zero
// @Field: OFE: always zero
// @Field: OFD: always zero

// @LoggerMessage: NKF1
// @Description: EKF2 estimator outputs
// @Field: TimeUS: Time since system startup
// @Field: C: EKF2 core this data is for
// @Field: Roll: Estimated roll
// @Field: Pitch: Estimated pitch
// @Field: Yaw: Estimated yaw
// @Field: VN: Estimated velocity (North component)
// @Field: VE: Estimated velocity (East component)
// @Field: VD: Estimated velocity (Down component)
// @Field: dPD: Filtered derivative of vertical position (down)
// @Field: PN: Estimated distance from origin (North component)
// @Field: PE: Estimated distance from origin (East component)
// @Field: PD: Estimated distance from origin (Down component)
// @Field: GX: Estimated gyro bias, X axis
// @Field: GY: Estimated gyro bias, Y axis
// @Field: GZ: Estimated gyro bias, Z axis
// @Field: OH: Height of origin above WGS-84

// @LoggerMessage: NKF2
// @Description: EKF2 estimator secondary outputs
// @Field: TimeUS: Time since system startup
// @Field: C: EKF2 core this data is for
// @Field: AZbias: Estimated accelerometer Z bias
// @Field: GSX: Gyro Scale Factor (X-axis)
// @Field: GSY: Gyro Scale Factor (Y-axis)
// @Field: GSZ: Gyro Scale Factor (Z-axis)
// @Field: VWN: Estimated wind velocity (North component)
// @Field: VWE: Estimated wind velocity (East component)
// @Field: MN: Magnetic field strength (North component)
// @Field: ME: Magnetic field strength (East component)
// @Field: MD: Magnetic field strength (Down component)
// @Field: MX: Magnetic field strength (body X-axis)
// @Field: MY: Magnetic field strength (body Y-axis)
// @Field: MZ: Magnetic field strength (body Z-axis)
// @Field: MI: Magnetometer used for data

// @LoggerMessage: NKF3
// @Description: EKF2 innovations
// @Field: TimeUS: Time since system startup
// @Field: C: EKF2 core this data is for
// @Field: IVN: Innovation in velocity (North component)
// @Field: IVE: Innovation in velocity (East component)
// @Field: IVD: Innovation in velocity (Down component)
// @Field: IPN: Innovation in position (North component)
// @Field: IPE: Innovation in position (East component)
// @Field: IPD: Innovation in position (Down component)
// @Field: IMX: Innovation in magnetic field strength (X-axis component)
// @Field: IMY: Innovation in magnetic field strength (Y-axis component)
// @Field: IMZ: Innovation in magnetic field strength (Z-axis component)
// @Field: IYAW: Innovation in vehicle yaw
// @Field: IVT: Innovation in true-airspeed

// @LoggerMessage: NKF4
// @Description: EKF2 variances
// @Field: TimeUS: Time since system startup
// @Field: C: EKF2 core this data is for
// @Field: SV: Square root of the velocity variance
// @Field: SP: Square root of the position variance
// @Field: SH: Square root of the height variance
// @Field: SM: Magnetic field variance
// @Field: SVT: Square root of the total airspeed variance
// @Field: errRP: Filtered error in roll/pitch estimate
// @Field: OFN: Most recent position recent magnitude (North component)
// @Field: OFE: Most recent position recent magnitude (East component)
// @Field: FS: Filter fault status
// @Field: TS: Filter timeout status
// @Field: SS: Filter solution status
// @Field: GPS: Filter GPS status
// @Field: PI: Primary core index

// @LoggerMessage: NKF5
// @Description: EKF2 Sensor innovations (primary core) and general dumping ground
// @Field: TimeUS: Time since system startup
// @Field: NI: Normalised flow variance
// @Field: FIX: Optical flow LOS rate vector innovations from the main nav filter (X-axis)
// @Field: FIY: Optical flow LOS rate vector innovations from the main nav filter (Y-axis)
// @Field: AFI: Optical flow LOS rate innovation from terrain offset estimator
// @Field: HAGL: Height above ground level
// @Field: offset: Estimated vertical position of the terrain relative to the nav filter zero datum
// @Field: RI: Range finder innovations
// @Field: rng: Measured range
// @Field: Herr: Filter ground offset state error
// @Field: eAng: Magnitude of angular error
// @Field: eVel: Magnitude of velocity error
// @Field: ePos: Magnitude of position error

// @LoggerMessage: NKQ
// @Description: EKF2 quaternion defining the rotation from NED to XYZ (autopilot) axes
// @Field: TimeUS: Time since system startup
// @Field: C: EKF2 core this data is for
// @Field: Q1: Quaternion a term
// @Field: Q2: Quaternion b term
// @Field: Q3: Quaternion c term
// @Field: Q4: Quaternion d term

// @LoggerMessage: OABR
// @Description: Object avoidance (Bendy Ruler) diagnostics
// @Field: TimeUS: Time since system startup
// @Field: Active: True if Bendy Ruler avoidance is being used
// @Field: DesYaw: Best yaw chosen to avoid obstacle
// @Field: Yaw: Current vehicle yaw
// @Field: ResChg: True if BendyRuler resisted changing bearing and continued in last calculated bearing
// @Field: Mar: Margin from path to obstacle on best yaw chosen
// @Field: DLat: Destination latitude
// @Field: DLng: Destination longitude
// @Field: OALat: Intermediate location chosen for avoidance
// @Field: OALng: Intermediate location chosen for avoidance

// @LoggerMessage: OADJ
// @Description: Object avoidance (Dijkstra) diagnostics
// @Field: TimeUS: Time since system startup
// @Field: State: Dijkstra avoidance library state
// @Field: Err: Dijkstra library error condition
// @Field: CurrPoint: Destination point in calculated path to destination
// @Field: TotPoints: Number of points in path to destination
// @Field: DLat: Destination latitude
// @Field: DLng: Destination longitude
// @Field: OALat: Object Avoidance chosen destination point latitude
// @Field: OALng: Object Avoidance chosen destination point longitude

// @LoggerMessage: OF
// @Description: Optical flow sensor data
// @Field: TimeUS: Time since system startup
// @Field: Qual: Estimated sensor data quality
// @Field: flowX: Sensor flow rate, X-axis
// @Field: flowY: Sensor flow rate,Y-axis
// @Field: bodyX: derived velocity, X-axis
// @Field: bodyY: derived velocity, Y-axis

// @LoggerMessage: ORGN
// @Description: Vehicle navigation origin or other notable position
// @Field: TimeUS: Time since system startup
// @Field: Type: Position type
// @Field: Lat: Position latitude
// @Field: Lng: Position longitude
// @Field: Alt: Position altitude

// @LoggerMessage: PARM
// @Description: parameter value
// @Field: TimeUS: Time since system startup
// @Field: Name: parameter name
// @Field: Value: parameter vlaue

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

// @LoggerMessage: PM
// @Description: autopilot system performance and general data dumping ground
// @Field: TimeUS: Time since system startup
// @Field: NLon: Number of long loops detected
// @Field: NLoop: Number of measurement loops for this message
// @Field: MaxT: Maximum loop time
// @Field: Mem: Free memory available
// @Field: Load: System processor load
// @Field: IntE: Internal error mask; which internal errors have been detected
// @Field: IntEC: Internal error count; how many internal errors have been detected
// @Field: SPIC: Number of SPI transactions processed
// @Field: I2CC: Number of i2c transactions processed
// @Field: I2CI: Number of i2c interrupts serviced
// @Field: ExUS: number of microseconds being added to each loop to address scheduler overruns

// @LoggerMessage: POS
// @Description: Canonical vehicle position
// @Field: TimeUS: Time since system startup
// @Field: Lat: Canonical vehicle latitude
// @Field: Lng: Canonical vehicle longitude
// @Field: Alt: Canonical vehicle altitude
// @Field: RelHomeAlt: Canonical vehicle altitude relative to home
// @Field: RelOriginAlt: Canonical vehicle altitude relative to navigation origin

// @LoggerMessage: POWR
// @Description: System power information
// @Field: TimeUS: Time since system startup
// @Field: Vcc: Flight board voltage
// @Field: VServo: Servo rail voltage
// @Field: Flags: System power flags
// @Field: AccFlags: Accumulated System power flags; all flags which have ever been set
// @Field: Safety: Hardware Safety Switch status

// @LoggerMessage: PRX
// @Description: Proximity sensor data
// @Field: TimeUS: Time since system startup
// @Field: Health: True if proximity sensor is healthy
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

// @LoggerMessage: RATE
// @Description: Desired and achieved vehicle attitude rates
// @Field: TimeUS: Time since system startup
// @Field: RDes: vehicle desired roll rate
// @Field: R: achieved vehicle roll rate
// @Field: ROut: normalized output for Roll
// @Field: PDes: vehicle desired pitch rate
// @Field: P: vehicle pitch rate
// @Field: POut: normalized output for Pitch
// @Field: YDes: vehicle desired yaw rate
// @Field: Y: achieved vehicle yaw rate
// @Field: YOut: normalized output for Yaw
// @Field: YDes: vehicle desired yaw rate
// @Field: Y: achieved vehicle yaw rate
// @Field: ADes: desired vehicle vertical acceleration
// @Field: A: achieved vehicle vertical acceleration
// @Field: AOut: percentage of vertical thrust output current being used

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

// @LoggerMessage: SA
// @Description: Simple Avoidance messages
// @Field: TimeUS: Time since system startup
// @Field: State: True if Simple Avoidance is active
// @Field: DVelX: Desired velocity, X-Axis (Velocity before Avoidance)
// @Field: DVelY: Desired velocity, Y-Axis (Velocity before Avoidance)
// @Field: MVelX: Modified velocity, X-Axis (Velocity after Avoidance)
// @Field: MVelY: Modified velocity, Y-Axis (Velocity after Avoidance)
// @Field: Back: True if vehicle is backing away 

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

// @LoggerMessage: UBX1
// @Description: uBlox-specific GPS information (part 1)
// @Field: TimeUS: Time since system startup
// @Field: Instance: GPS instance number
// @Field: noisePerMS: noise level as measured by GPS
// @Field: jamInd: jamming indicator; higher is more likely jammed
// @Field: aPower: antenna power indicator; 2 is don't know
// @Field: agcCnt: automatic gain control monitor
// @Field: config: bitmask for messages which haven't been seen

// @LoggerMessage: UBX2
// @Description: uBlox-specific GPS information (part 2)
// @Field: TimeUS: Time since system startup
// @Field: Instance: GPS instance number
// @Field: ofsI: imbalance of I part of complex signal
// @Field: magI: magnitude of I part of complex signal
// @Field: ofsQ: imbalance of Q part of complex signal
// @Field: magQ: magnitude of Q part of complex signal

// @LoggerMessage: UNIT
// @Description: Message mapping from single character to SI unit
// @Field: TimeUS: Time since system startup
// @Field: Id: character referenced by FMTU
// @Field: Label: Unit - SI where available

// @LoggerMessage: VIBE
// @Description: Processed (acceleration) vibration information
// @Field: TimeUS: Time since system startup
// @Field: VibeX: Primary accelerometer filtered vibration, x-axis
// @Field: VibeY: Primary accelerometer filtered vibration, y-axis
// @Field: VibeZ: Primary accelerometer filtered vibration, z-axis
// @Field: Clip0: Number of clipping events on 1st accelerometer
// @Field: Clip1: Number of clipping events on 2nd accelerometer
// @Field: Clip2: Number of clipping events on 3rd accelerometer

// @LoggerMessage: VISO
// @Description: Visual Odometry
// @Field: TimeUS: System time
// @Field: dt: Time period this data covers
// @Field: AngDX: Angular change for body-frame roll axis
// @Field: AngDY: Angular change for body-frame pitch axis
// @Field: AngDZ: Angular change for body-frame z axis
// @Field: PosDX: Position change for body-frame X axis (Forward-Back)
// @Field: PosDY: Position change for body-frame Y axis (Right-Left)
// @Field: PosDZ: Position change for body-frame Z axis (Down-Up)
// @Field: conf: Confidence

// @LoggerMessage: VISP
// @Description: Vision Position
// @Field: TimeUS: System time
// @Field: RTimeUS: Remote system time
// @Field: CTimeMS: Corrected system time
// @Field: PX: Position X-axis (North-South)
// @Field: PY: Position Y-axis (East-West)
// @Field: PZ: Position Z-axis (Down-Up)
// @Field: Roll: Roll lean angle
// @Field: Pitch: Pitch lean angle
// @Field: Yaw: Yaw angle
// @Field: PErr: Position estimate error
// @Field: AErr: Attitude estimate error
// @Field: RstCnt: Position reset counter

// @LoggerMessage: VISV
// @Description: Vision Velocity
// @Field: TimeUS: System time
// @Field: RTimeUS: Remote system time
// @Field: CTimeMS: Corrected system time
// @Field: VX: Velocity X-axis (North-South)
// @Field: VY: Velocity Y-axis (East-West)
// @Field: VZ: Velocity Z-axis (Down-Up)
// @Field: VErr: Velocity estimate error
// @Field: RstCnt: Velocity reset counter

// @LoggerMessage: WENC
// @Description: Wheel encoder measurements
// @Field: TimeUS: Time since system startup
// @Field: Dist0: First wheel distance travelled
// @Field: Qual0: Quality measurement of Dist0
// @Field: Dist1: Second wheel distance travelled
// @Field: Qual1: Quality measurement of Dist1

// @LoggerMessage: XKF0
// @Description: EKF3 beacon sensor diagnostics
// @Field: TimeUS: Time since system startup
// @Field: ID: Beacon sensor ID
// @Field: rng: Beacon range
// @Field: innov: Beacon range innovation
// @Field: SIV: sqrt of beacon range innovation variance
// @Field: TR: Beacon range innovation consistency test ratio
// @Field: BPN: Beacon north position
// @Field: BPE: Beacon east position
// @Field: BPD: Beacon down position
// @Field: OFH: High estimate of vertical position offset of beacons rel to EKF origin
// @Field: OFL: Low estimate of vertical position offset of beacons rel to EKF origin
// @Field: OFN: North position of receiver rel to EKF origin
// @Field: OFE: East position of receiver rel to EKF origin
// @Field: OFD: Down position of receiver rel to EKF origin

// @LoggerMessage: XKF1
// @Description: EKF3 estimator outputs
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: Roll: Estimated roll
// @Field: Pitch: Estimated pitch
// @Field: Yaw: Estimated yaw
// @Field: VN: Estimated velocity (North component)
// @Field: VE: Estimated velocity (East component)
// @Field: VD: Estimated velocity (Down component)
// @Field: dPD: Filtered derivative of vertical position (down)
// @Field: PN: Estimated distance from origin (North component)
// @Field: PE: Estimated distance from origin (East component)
// @Field: PD: Estimated distance from origin (Down component)
// @Field: GX: Estimated gyro bias, X axis
// @Field: GY: Estimated gyro bias, Y axis
// @Field: GZ: Estimated gyro bias, Z axis
// @Field: OH: Height of origin above WGS-84

// @LoggerMessage: XKF2
// @Description: EKF3 estimator secondary outputs
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: AX: Estimated accelerometer X bias
// @Field: AY: Estimated accelerometer Y bias
// @Field: AZ: Estimated accelerometer Z bias
// @Field: VWN: Estimated wind velocity (North component)
// @Field: VWE: Estimated wind velocity (East component)
// @Field: MN: Magnetic field strength (North component)
// @Field: ME: Magnetic field strength (East component)
// @Field: MD: Magnetic field strength (Down component)
// @Field: MX: Magnetic field strength (body X-axis)
// @Field: MY: Magnetic field strength (body Y-axis)
// @Field: MZ: Magnetic field strength (body Z-axis)
// @Field: MI: Magnetometer used for data

// @LoggerMessage: XKF3
// @Description: EKF3 innovations
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: IVN: Innovation in velocity (North component)
// @Field: IVE: Innovation in velocity (East component)
// @Field: IVD: Innovation in velocity (Down component)
// @Field: IPN: Innovation in position (North component)
// @Field: IPE: Innovation in position (East component)
// @Field: IPD: Innovation in position (Down component)
// @Field: IMX: Innovation in magnetic field strength (X-axis component)
// @Field: IMY: Innovation in magnetic field strength (Y-axis component)
// @Field: IMZ: Innovation in magnetic field strength (Z-axis component)
// @Field: IYAW: Innovation in vehicle yaw
// @Field: IVT: Innovation in true-airspeed

// @LoggerMessage: XKF4
// @Description: EKF3 variances
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: SV: Square root of the velocity variance
// @Field: SP: Square root of the position variance
// @Field: SH: Square root of the height variance
// @Field: SM: Magnetic field variance
// @Field: SVT: Square root of the total airspeed variance
// @Field: errRP: Filtered error in roll/pitch estimate
// @Field: OFN: Most recent position recent magnitude (North component)
// @Field: OFE: Most recent position recent magnitude (East component)
// @Field: FS: Filter fault status
// @Field: TS: Filter timeout status
// @Field: SS: Filter solution status
// @Field: GPS: Filter GPS status
// @Field: PI: Primary core index

// @LoggerMessage: XKF5
// @Description: EKF3 Sensor innovations (primary core) and general dumping ground
// @Field: TimeUS: Time since system startup
// @Field: NI: Normalised flow variance
// @Field: FIX: Optical flow LOS rate vector innovations from the main nav filter (X-axis)
// @Field: FIY: Optical flow LOS rate vector innovations from the main nav filter (Y-axis)
// @Field: AFI: Optical flow LOS rate innovation from terrain offset estimator
// @Field: HAGL: Height above ground level
// @Field: offset: Estimated vertical position of the terrain relative to the nav filter zero datum
// @Field: RI: Range finder innovations
// @Field: rng: Measured range
// @Field: Herr: Filter ground offset state error
// @Field: eAng: Magnitude of angular error
// @Field: eVel: Magnitude of velocity error
// @Field: ePos: Magnitude of position error

// @LoggerMessage: XKFD
// @Description: EKF3 Body Frame Odometry errors
// @Field: TimeUS: Time since system startup
// @Field: IX: Innovation in velocity (X-axis)
// @Field: IY: Innovation in velocity (Y-axis)
// @Field: IZ: Innovation in velocity (Z-axis)
// @Field: IVX: Variance in velocity (X-axis)
// @Field: IVY: Variance in velocity (Y-axis)
// @Field: IVZ: Variance in velocity (Z-axis)

// @LoggerMessage: XKQ
// @Description: EKF3 quaternion defining the rotation from NED to XYZ (autopilot) axes
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: Q1: Quaternion a term
// @Field: Q2: Quaternion b term
// @Field: Q3: Quaternion c term
// @Field: Q4: Quaternion d term

// @LoggerMessage: XKV1
// @Description: EKF3 State variances (primary core)
// @Field: TimeUS: Time since system startup
// @Field: V00: Variance for state 0
// @Field: V01: Variance for state 1
// @Field: V02: Variance for state 2
// @Field: V03: Variance for state 3
// @Field: V04: Variance for state 4
// @Field: V05: Variance for state 5
// @Field: V06: Variance for state 6
// @Field: V07: Variance for state 7
// @Field: V08: Variance for state 8
// @Field: V09: Variance for state 9
// @Field: V10: Variance for state 10
// @Field: V11: Variance for state 11

// @LoggerMessage: XKV2
// @Description: more EKF3 State Variances (primary core)
// @Field: TimeUS: Time since system startup
// @Field: V12: Variance for state 12
// @Field: V13: Variance for state 13
// @Field: V14: Variance for state 14
// @Field: V15: Variance for state 15
// @Field: V16: Variance for state 16
// @Field: V17: Variance for state 17
// @Field: V18: Variance for state 18
// @Field: V19: Variance for state 19
// @Field: V20: Variance for state 20
// @Field: V21: Variance for state 21
// @Field: V22: Variance for state 22
// @Field: V23: Variance for state 23

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
    { LOG_GPS_MSG, sizeof(log_GPS), \
      "GPS",  GPS_FMT, GPS_LABELS, GPS_UNITS, GPS_MULTS }, \
    { LOG_GPS2_MSG, sizeof(log_GPS), \
      "GPS2", GPS_FMT, GPS_LABELS, GPS_UNITS, GPS_MULTS }, \
    { LOG_GPSB_MSG, sizeof(log_GPS), \
      "GPSB", GPS_FMT, GPS_LABELS, GPS_UNITS, GPS_MULTS }, \
    { LOG_GPA_MSG,  sizeof(log_GPA), \
      "GPA",  GPA_FMT, GPA_LABELS, GPA_UNITS, GPA_MULTS }, \
    { LOG_GPA2_MSG, sizeof(log_GPA), \
      "GPA2", GPA_FMT, GPA_LABELS, GPA_UNITS, GPA_MULTS }, \
    { LOG_GPAB_MSG, sizeof(log_GPA), \
      "GPAB", GPA_FMT, GPA_LABELS, GPA_UNITS, GPA_MULTS }, \
    { LOG_IMU_MSG, sizeof(log_IMU), \
      "IMU",  IMU_FMT,     IMU_LABELS, IMU_UNITS, IMU_MULTS }, \
    { LOG_MESSAGE_MSG, sizeof(log_Message), \
      "MSG",  "QZ",     "TimeUS,Message", "s-", "F-"}, \
    { LOG_RCIN_MSG, sizeof(log_RCIN), \
      "RCIN",  "QHHHHHHHHHHHHHH",     "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14", "sYYYYYYYYYYYYYY", "F--------------" }, \
    { LOG_RCOUT_MSG, sizeof(log_RCOUT), \
      "RCOU",  "QHHHHHHHHHHHHHH",     "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14", "sYYYYYYYYYYYYYY", "F--------------"  }, \
    { LOG_RSSI_MSG, sizeof(log_RSSI), \
      "RSSI",  "Qf",     "TimeUS,RXRSSI", "s-", "F-"  }, \
    { LOG_BARO_MSG, sizeof(log_BARO), \
      "BARO",  BARO_FMT, BARO_LABELS, BARO_UNITS, BARO_MULTS }, \
    { LOG_POWR_MSG, sizeof(log_POWR), \
      "POWR","QffHHB","TimeUS,Vcc,VServo,Flags,AccFlags,Safety", "svv---", "F00---" },  \
    { LOG_CMD_MSG, sizeof(log_Cmd), \
      "CMD", "QHHHffffLLfB","TimeUS,CTot,CNum,CId,Prm1,Prm2,Prm3,Prm4,Lat,Lng,Alt,Frame", "s-------DUm-", "F-------GG0-" }, \
    { LOG_MAVLINK_COMMAND_MSG, sizeof(log_MAVLink_Command), \
      "MAVC", "QBBBHBBffffiifBB","TimeUS,TS,TC,Fr,Cmd,Cur,AC,P1,P2,P3,P4,X,Y,Z,Res,WL", "s---------------", "F---------------" }, \
    { LOG_RADIO_MSG, sizeof(log_Radio), \
      "RAD", "QBBBBBHH", "TimeUS,RSSI,RemRSSI,TxBuf,Noise,RemNoise,RxErrors,Fixed", "s-------", "F-------" }, \
    { LOG_CAMERA_MSG, sizeof(log_Camera), \
      "CAM", "QIHLLeeeccC","TimeUS,GPSTime,GPSWeek,Lat,Lng,Alt,RelAlt,GPSAlt,Roll,Pitch,Yaw", "s--DUmmmddd", "F--GGBBBBBB" }, \
    { LOG_TRIGGER_MSG, sizeof(log_Camera), \
      "TRIG", "QIHLLeeeccC","TimeUS,GPSTime,GPSWeek,Lat,Lng,Alt,RelAlt,GPSAlt,Roll,Pitch,Yaw", "s--DUmmmddd", "F--GGBBBBBB" }, \
    { LOG_ARSP_MSG, sizeof(log_AIRSPEED), "ARSP",  ARSP_FMT, ARSP_LABELS, ARSP_UNITS, ARSP_MULTS }, \
    { LOG_ASP2_MSG, sizeof(log_AIRSPEED), "ASP2",  ARSP_FMT, ARSP_LABELS, ARSP_UNITS, ARSP_MULTS }, \
    { LOG_CURRENT_MSG, sizeof(log_Current),                     \
      "BAT", "QBfffffcf", "TimeUS,Instance,Volt,VoltR,Curr,CurrTot,EnrgTot,Temp,Res", "s#vvAiJOw", "F-000!/?0" },  \
    { LOG_CURRENT_CELLS_MSG, sizeof(log_Current_Cells), \
      "BCL", "QBfHHHHHHHHHHHH", "TimeUS,Instance,Volt,V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12", "s#vvvvvvvvvvvvv", "F-0CCCCCCCCCCCC" }, \
	{ LOG_ATTITUDE_MSG, sizeof(log_Attitude),\
      "ATT", "QccccCCCC", "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,ErrRP,ErrYaw", "sddddhhdh", "FBBBBBBBB" }, \
    { LOG_COMPASS_MSG, sizeof(log_Compass), \
      "MAG", MAG_FMT,    MAG_LABELS, MAG_UNITS, MAG_MULTS }, \
    { LOG_MODE_MSG, sizeof(log_Mode), \
      "MODE", "QMBB",         "TimeUS,Mode,ModeNum,Rsn", "s---", "F---" }, \
    { LOG_RFND_MSG, sizeof(log_RFND), \
      "RFND", "QBCBB", "TimeUS,Instance,Dist,Stat,Orient", "s#m--", "F-B--" }, \
    { LOG_MAV_STATS, sizeof(log_MAV_Stats), \
      "DMS", "QIIIIBBBBBBBBB",         "TimeUS,N,Dp,RT,RS,Fa,Fmn,Fmx,Pa,Pmn,Pmx,Sa,Smn,Smx", "s-------------", "F-------------" }, \
    { LOG_BEACON_MSG, sizeof(log_Beacon), \
      "BCN", "QBBfffffff",  "TimeUS,Health,Cnt,D0,D1,D2,D3,PosX,PosY,PosZ", "s--mmmmmmm", "F--BBBBBBB" }, \
    { LOG_PROXIMITY_MSG, sizeof(log_Proximity), \
      "PRX", "QBfffffffffff", "TimeUS,Health,D0,D45,D90,D135,D180,D225,D270,D315,DUp,CAn,CDis", "s-mmmmmmmmmhm", "F-00000000000" }, \
    { LOG_PERFORMANCE_MSG, sizeof(log_Performance),                     \
      "PM",  "QHHIIHIIIIII", "TimeUS,NLon,NLoop,MaxT,Mem,Load,IntE,IntEC,SPIC,I2CC,I2CI,ExUS", "s---b%-----s", "F---0A-----F" }, \
    { LOG_SRTL_MSG, sizeof(log_SRTL), \
      "SRTL", "QBHHBfff", "TimeUS,Active,NumPts,MaxPts,Action,N,E,D", "s----mmm", "F----000" }, \
    { LOG_OA_BENDYRULER_MSG, sizeof(log_OABendyRuler), \
      "OABR","QBHHBfLLLL","TimeUS,Active,DesYaw,Yaw,ResChg,Mar,DLat,DLng,OALat,OALng", "sbdd-mDUDU", "F-----GGGG" }, \
    { LOG_OA_DIJKSTRA_MSG, sizeof(log_OADijkstra), \
      "OADJ","QBBBBLLLL","TimeUS,State,Err,CurrPoint,TotPoints,DLat,DLng,OALat,OALng", "sbbbbDUDU", "F----GGGG" }, \
    { LOG_SIMPLE_AVOID_MSG, sizeof(log_SimpleAvoid), \
      "SA",  "QBffffB","TimeUS,State,DVelX,DVelY,MVelX,MVelY,Back", "sbnnnnb", "F------"}, \
    { LOG_IMU2_MSG, sizeof(log_IMU), \
      "IMU2",  IMU_FMT,     IMU_LABELS, IMU_UNITS, IMU_MULTS }, \
    { LOG_IMU3_MSG, sizeof(log_IMU), \
      "IMU3",  IMU_FMT,     IMU_LABELS, IMU_UNITS, IMU_MULTS }, \
    { LOG_AHR2_MSG, sizeof(log_AHRS), \
      "AHR2","QccCfLLffff","TimeUS,Roll,Pitch,Yaw,Alt,Lat,Lng,Q1,Q2,Q3,Q4","sddhmDU????", "FBBB0GG????" }, \
    { LOG_POS_MSG, sizeof(log_POS), \
      "POS","QLLfff","TimeUS,Lat,Lng,Alt,RelHomeAlt,RelOriginAlt", "sDUmmm", "FGG000" }, \
    { LOG_SIMSTATE_MSG, sizeof(log_AHRS), \
      "SIM","QccCfLLffff","TimeUS,Roll,Pitch,Yaw,Alt,Lat,Lng,Q1,Q2,Q3,Q4", "sddhmDU????", "FBBB0GG????" }, \
    { LOG_NKF1_MSG, sizeof(log_EKF1), \
      "NKF1","QBccCfffffffccce","TimeUS,C,Roll,Pitch,Yaw,VN,VE,VD,dPD,PN,PE,PD,GX,GY,GZ,OH", "s#ddhnnnnmmmkkkm", "F-BBB0000000BBBB" }, \
    { LOG_NKF2_MSG, sizeof(log_NKF2), \
      "NKF2","QBbccccchhhhhhB","TimeUS,C,AZbias,GSX,GSY,GSZ,VWN,VWE,MN,ME,MD,MX,MY,MZ,MI", "s#----nnGGGGGG-", "F-----BBCCCCCC-" }, \
    { LOG_NKF3_MSG, sizeof(log_NKF3), \
      "NKF3","QBcccccchhhcc","TimeUS,C,IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IYAW,IVT", "s#nnnmmmGGG??", "F-BBBBBBCCCBB" }, \
    { LOG_NKF4_MSG, sizeof(log_NKF4), \
      "NKF4","QBcccccfbbHBIHb","TimeUS,C,SV,SP,SH,SM,SVT,errRP,OFN,OFE,FS,TS,SS,GPS,PI", "s#------??-----", "F-------??-----" }, \
    { LOG_NKF5_MSG, sizeof(log_NKF5), \
      "NKF5","QBhhhcccCCfff","TimeUS,NI,FIX,FIY,AFI,HAGL,offset,RI,rng,Herr,eAng,eVel,ePos", "s----m???mrnm", "F----BBBBB000" }, \
    { LOG_NKF10_MSG, sizeof(log_RngBcnDebug), \
      "NKF0","QBccCCcccccccc","TimeUS,ID,rng,innov,SIV,TR,BPN,BPE,BPD,OFH,OFL,OFN,OFE,OFD", "s-m---mmmmmmmm", "F-B---BBBBBBBB" }, \
    { LOG_NKQ_MSG, sizeof(log_Quaternion), "NKQ", QUAT_FMT, QUAT_LABELS, QUAT_UNITS, QUAT_MULTS }, \
    { LOG_XKF1_MSG, sizeof(log_EKF1), \
      "XKF1","QBccCfffffffccce","TimeUS,C,Roll,Pitch,Yaw,VN,VE,VD,dPD,PN,PE,PD,GX,GY,GZ,OH", "s#ddhnnnnmmmkkkm", "F-BBB0000000BBBB" }, \
    { LOG_XKF2_MSG, sizeof(log_XKF2), \
      "XKF2","QBccccchhhhhhB","TimeUS,C,AX,AY,AZ,VWN,VWE,MN,ME,MD,MX,MY,MZ,MI", "s#---nnGGGGGG-", "F----BBCCCCCC-" }, \
    { LOG_XKF3_MSG, sizeof(log_NKF3), \
      "XKF3","QBcccccchhhcc","TimeUS,C,IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IYAW,IVT", "s#nnnmmmGGG??", "F-BBBBBBCCCBB" }, \
    { LOG_XKF4_MSG, sizeof(log_NKF4), \
      "XKF4","QBcccccfbbHBIHb","TimeUS,C,SV,SP,SH,SM,SVT,errRP,OFN,OFE,FS,TS,SS,GPS,PI", "s#------??-----", "F-------??-----" }, \
    { LOG_XKF5_MSG, sizeof(log_NKF5), \
      "XKF5","QBhhhcccCCfff","TimeUS,NI,FIX,FIY,AFI,HAGL,offset,RI,rng,Herr,eAng,eVel,ePos", "s----m???mrnm", "F----BBBBB000" }, \
    { LOG_XKF10_MSG, sizeof(log_RngBcnDebug), \
      "XKF0","QBccCCcccccccc","TimeUS,ID,rng,innov,SIV,TR,BPN,BPE,BPD,OFH,OFL,OFN,OFE,OFD", "s-m---mmmmmmmm", "F-B---BBBBBBBB" }, \
    { LOG_XKQ_MSG, sizeof(log_Quaternion), "XKQ", QUAT_FMT, QUAT_LABELS, QUAT_UNITS, QUAT_MULTS }, \
    { LOG_XKFD_MSG, sizeof(log_ekfBodyOdomDebug), \
      "XKFD","Qffffff","TimeUS,IX,IY,IZ,IVX,IVY,IVZ", "s------", "F------" }, \
    { LOG_XKV1_MSG, sizeof(log_ekfStateVar), \
      "XKV1","Qffffffffffff","TimeUS,V00,V01,V02,V03,V04,V05,V06,V07,V08,V09,V10,V11", "s------------", "F------------" }, \
    { LOG_XKV2_MSG, sizeof(log_ekfStateVar), \
      "XKV2","Qffffffffffff","TimeUS,V12,V13,V14,V15,V16,V17,V18,V19,V20,V21,V22,V23", "s------------", "F------------" }, \
    { LOG_TERRAIN_MSG, sizeof(log_TERRAIN), \
      "TERR","QBLLHffHH","TimeUS,Status,Lat,Lng,Spacing,TerrH,CHeight,Pending,Loaded", "s-DU-mm--", "F-GG-00--" }, \
    { LOG_GPS_UBX1_MSG, sizeof(log_Ubx1), \
      "UBX1", "QBHBBHI",  "TimeUS,Instance,noisePerMS,jamInd,aPower,agcCnt,config", "s#-----", "F------"  }, \
    { LOG_GPS_UBX2_MSG, sizeof(log_Ubx2), \
      "UBX2", "QBbBbB", "TimeUS,Instance,ofsI,magI,ofsQ,magQ", "s#----", "F-----" }, \
    { LOG_GPS_RAW_MSG, sizeof(log_GPS_RAW), \
      "GRAW", "QIHBBddfBbB", "TimeUS,WkMS,Week,numSV,sv,cpMes,prMes,doMes,mesQI,cno,lli", "s--S-------", "F--0-------" }, \
    { LOG_GPS_RAWH_MSG, sizeof(log_GPS_RAWH), \
      "GRXH", "QdHbBB", "TimeUS,rcvTime,week,leapS,numMeas,recStat", "s-----", "F-----" }, \
    { LOG_GPS_RAWS_MSG, sizeof(log_GPS_RAWS), \
      "GRXS", "QddfBBBHBBBBB", "TimeUS,prMes,cpMes,doMes,gnss,sv,freq,lock,cno,prD,cpD,doD,trk", "s------------", "F------------" }, \
    { LOG_ESC_MSG, sizeof(log_Esc), \
      "ESC",  "QBeCCcHc", "TimeUS,Instance,RPM,Volt,Curr,Temp,CTot,MotTemp", "s#qvAO-O", "F-BBBB-B" }, \
    { LOG_CSRV_MSG, sizeof(log_CSRV), \
      "CSRV","QBfffB","TimeUS,Id,Pos,Force,Speed,Pow", "s#---%", "F-0000" }, \
    { LOG_CESC_MSG, sizeof(log_CESC), \
      "CESC","QBIfffiB","TimeUS,Id,ECnt,Voltage,Curr,Temp,RPM,Pow", "s#-vAOq%", "F-000000" }, \
    { LOG_COMPASS2_MSG, sizeof(log_Compass), \
      "MAG2",MAG_FMT,    MAG_LABELS, MAG_UNITS, MAG_MULTS }, \
    { LOG_COMPASS3_MSG, sizeof(log_Compass), \
      "MAG3",MAG_FMT,    MAG_LABELS, MAG_UNITS, MAG_MULTS }, \
    { LOG_ACC1_MSG, sizeof(log_ACCEL), \
      "ACC1", ACC_FMT,        ACC_LABELS, ACC_UNITS, ACC_MULTS }, \
    { LOG_ACC2_MSG, sizeof(log_ACCEL), \
      "ACC2", ACC_FMT,        ACC_LABELS, ACC_UNITS, ACC_MULTS }, \
    { LOG_ACC3_MSG, sizeof(log_ACCEL), \
      "ACC3", ACC_FMT,        ACC_LABELS, ACC_UNITS, ACC_MULTS }, \
    { LOG_GYR1_MSG, sizeof(log_GYRO), \
      "GYR1", GYR_FMT,        GYR_LABELS, GYR_UNITS, GYR_MULTS }, \
    { LOG_GYR2_MSG, sizeof(log_GYRO), \
      "GYR2", GYR_FMT,        GYR_LABELS, GYR_UNITS, GYR_MULTS }, \
    { LOG_GYR3_MSG, sizeof(log_GYRO), \
      "GYR3", GYR_FMT,        GYR_LABELS, GYR_UNITS, GYR_MULTS }, \
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
    { LOG_DSTL_MSG, sizeof(log_DSTL), \
      "DSTL", "QBfLLeccfeffff", "TimeUS,Stg,THdg,Lat,Lng,Alt,XT,Travel,L1I,Loiter,Des,P,I,D", "s??DUm--------", "F??000--------" }, \
    { LOG_BAR2_MSG, sizeof(log_BARO), \
      "BAR2",  BARO_FMT, BARO_LABELS, BARO_UNITS, BARO_MULTS }, \
    { LOG_BAR3_MSG, sizeof(log_BARO), \
      "BAR3",  BARO_FMT, BARO_LABELS, BARO_UNITS, BARO_MULTS }, \
    { LOG_VIBE_MSG, sizeof(log_Vibe), \
      "VIBE", "QfffIII",     "TimeUS,VibeX,VibeY,VibeZ,Clip0,Clip1,Clip2", "s------", "F------" }, \
    { LOG_IMUDT_MSG, sizeof(log_IMUDT), \
      "IMT",IMT_FMT,IMT_LABELS, IMT_UNITS, IMT_MULTS }, \
    { LOG_IMUDT2_MSG, sizeof(log_IMUDT), \
      "IMT2",IMT_FMT,IMT_LABELS, IMT_UNITS, IMT_MULTS }, \
    { LOG_IMUDT3_MSG, sizeof(log_IMUDT), \
      "IMT3",IMT_FMT,IMT_LABELS, IMT_UNITS, IMT_MULTS }, \
    { LOG_ISBH_MSG, sizeof(log_ISBH), \
      "ISBH",ISBH_FMT,ISBH_LABELS,ISBH_UNITS,ISBH_MULTS },  \
    { LOG_ISBD_MSG, sizeof(log_ISBD), \
      "ISBD",ISBD_FMT,ISBD_LABELS, ISBD_UNITS, ISBD_MULTS }, \
    { LOG_ORGN_MSG, sizeof(log_ORGN), \
      "ORGN","QBLLe","TimeUS,Type,Lat,Lng,Alt", "s-DUm", "F-GGB" },   \
    { LOG_DF_FILE_STATS, sizeof(log_DSF), \
      "DSF", "QIHIIII", "TimeUS,Dp,Blk,Bytes,FMn,FMx,FAv", "s--b---", "F--0---" }, \
    { LOG_RPM_MSG, sizeof(log_RPM), \
      "RPM",  "Qff", "TimeUS,rpm1,rpm2", "sqq", "F00" }, \
    { LOG_RATE_MSG, sizeof(log_Rate), \
      "RATE", "Qffffffffffff",  "TimeUS,RDes,R,ROut,PDes,P,POut,YDes,Y,YOut,ADes,A,AOut", "skk-kk-kk-oo-", "F?????????BB-" }, \
    { LOG_RALLY_MSG, sizeof(log_Rally), \
      "RALY", "QBBLLh", "TimeUS,Tot,Seq,Lat,Lng,Alt", "s--DUm", "F--GGB" },  \
    { LOG_MAV_MSG, sizeof(log_MAV),   \
      "MAV", "QBHHHBHH",   "TimeUS,chan,txp,rxp,rxdp,flags,ss,tf", "s#----s-", "F-000-C-" },   \
    { LOG_VISUALODOM_MSG, sizeof(log_VisualOdom), \
      "VISO", "Qffffffff", "TimeUS,dt,AngDX,AngDY,AngDZ,PosDX,PosDY,PosDZ,conf", "ssrrrmmm-", "FF000000-" }, \
    { LOG_VISUALPOS_MSG, sizeof(log_VisualPosition), \
      "VISP", "QQIffffffffB", "TimeUS,RTimeUS,CTimeMS,PX,PY,PZ,Roll,Pitch,Yaw,PErr,AErr,RstCnt", "sssmmmddhmd-", "FFC00000000-" }, \
    { LOG_VISUALVEL_MSG, sizeof(log_VisualVelocity), \
      "VISV", "QQIffffB", "TimeUS,RTimeUS,CTimeMS,VX,VY,VZ,VErr,RstCnt", "sssnnnn-", "FFC0000-" }, \
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
      "WINC", "QBBBBBfffHfb", "TimeUS,Heal,ThEnd,Mov,Clut,Mode,DLen,Len,DRate,Tens,Vcc,Temp", "s-----mmn?vO", "F-----000000" }

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
    LOG_NKF1_MSG = 64,
    LOG_NKF2_MSG,
    LOG_NKF3_MSG,
    LOG_NKF4_MSG,
    LOG_NKF5_MSG,
    LOG_NKF10_MSG,
    LOG_NKQ_MSG,
    LOG_XKF1_MSG,
    LOG_XKF2_MSG,
    LOG_XKF3_MSG,
    LOG_XKF4_MSG,
    LOG_XKF5_MSG,
    LOG_XKF10_MSG,
    LOG_XKQ_MSG,
    LOG_XKFD_MSG,
    LOG_XKV1_MSG,
    LOG_XKV2_MSG,
    LOG_PARAMETER_MSG,
    LOG_GPS_MSG,
    LOG_GPS2_MSG,
    LOG_GPSB_MSG,
    LOG_IMU_MSG,
    LOG_MESSAGE_MSG,
    LOG_RCIN_MSG,
    LOG_RCOUT_MSG,
    LOG_RSSI_MSG,
    LOG_IMU2_MSG,
    LOG_BARO_MSG,
    LOG_POWR_MSG,
    LOG_AHR2_MSG,
    LOG_SIMSTATE_MSG,
    LOG_CMD_MSG,
    LOG_MAVLINK_COMMAND_MSG,
    LOG_RADIO_MSG,
    LOG_ATRP_MSG,
    LOG_CAMERA_MSG,
    LOG_IMU3_MSG,
    LOG_TERRAIN_MSG,
    LOG_GPS_UBX1_MSG,
    LOG_GPS_UBX2_MSG,
    LOG_ESC_MSG,
    LOG_CSRV_MSG,
    LOG_CESC_MSG,
    LOG_BAR2_MSG,
    LOG_ARSP_MSG,
    LOG_ATTITUDE_MSG,
    LOG_CURRENT_MSG,
    LOG_CURRENT_CELLS_MSG,
    LOG_COMPASS_MSG,
    LOG_COMPASS2_MSG,
    LOG_COMPASS3_MSG,
    LOG_MODE_MSG,
    LOG_GPS_RAW_MSG,

    // LOG_GPS_RAWH_MSG is used as a check for duplicates. Do not add between this and LOG_FORMAT_MSG
    LOG_GPS_RAWH_MSG,

    LOG_FORMAT_MSG = 128, // this must remain #128

    LOG_GPS_RAWS_MSG,
    LOG_ACC1_MSG,
    LOG_ACC2_MSG,
    LOG_ACC3_MSG,
    LOG_GYR1_MSG,
    LOG_GYR2_MSG,
    LOG_GYR3_MSG,
    LOG_POS_MSG,
    LOG_PIDR_MSG,
    LOG_PIDP_MSG,
    LOG_PIDY_MSG,
    LOG_PIDA_MSG,
    LOG_PIDS_MSG,
    LOG_DSTL_MSG,
    LOG_VIBE_MSG,
    LOG_IMUDT_MSG,
    LOG_IMUDT2_MSG,
    LOG_IMUDT3_MSG,
    LOG_ORGN_MSG,
    LOG_RPM_MSG,
    LOG_GPA_MSG,
    LOG_GPA2_MSG,
    LOG_GPAB_MSG,
    LOG_RFND_MSG,
    LOG_BAR3_MSG,
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
    LOG_TRIGGER_MSG,

    LOG_RATE_MSG,
    LOG_RALLY_MSG,
    LOG_VISUALODOM_MSG,
    LOG_VISUALPOS_MSG,
    LOG_AOA_SSA_MSG,
    LOG_BEACON_MSG,
    LOG_PROXIMITY_MSG,
    LOG_DF_FILE_STATS,
    LOG_SRTL_MSG,
    LOG_ISBH_MSG,
    LOG_ISBD_MSG,
    LOG_ASP2_MSG,
    LOG_PERFORMANCE_MSG,
    LOG_OPTFLOW_MSG,
    LOG_EVENT_MSG,
    LOG_WHEELENCODER_MSG,
    LOG_MAV_MSG,
    LOG_ERROR_MSG,
    LOG_ADSB_MSG,
    LOG_ARM_DISARM_MSG,
    LOG_OA_BENDYRULER_MSG,
    LOG_OA_DIJKSTRA_MSG,
    LOG_VISUALVEL_MSG,
    LOG_SIMPLE_AVOID_MSG,
    LOG_WINCH_MSG,

    _LOG_LAST_MSG_
};

static_assert(_LOG_LAST_MSG_ <= 255, "Too many message formats");
static_assert(LOG_GPS_RAWH_MSG < 128, "Duplicate message format IDs");

enum LogOriginType {
    ekf_origin = 0,
    ahrs_home = 1
};
