#pragma once

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
    const char name[5];
    const char format[16];
    const char labels[64];
};

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

struct PACKED log_Parameter {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    char name[16];
    float value;
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
    uint8_t  used;
};

struct PACKED log_GPA {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t vdop;
    uint16_t hacc;
    uint16_t vacc;
    uint16_t sacc;
    uint8_t  have_vv;
    uint32_t sample_ms;
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
};

struct PACKED log_IMUDT {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float delta_time, delta_vel_dt, delta_ang_dt;
    float delta_ang_x, delta_ang_y, delta_ang_z;
    float delta_vel_x, delta_vel_y, delta_vel_z;
};

struct PACKED log_Vibe {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float vibe_x, vibe_y, vibe_z;
    uint32_t clipping_0, clipping_1, clipping_2;
};

struct PACKED log_Gimbal1 {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    float delta_time;
    float delta_angles_x;
    float delta_angles_y;
    float delta_angles_z;
    float delta_velocity_x;
    float delta_velocity_y;
    float delta_velocity_z;
    float joint_angles_x;
    float joint_angles_y;
    float joint_angles_z;
};

struct PACKED log_Gimbal2 {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t  est_sta;
    float est_x;
    float est_y;
    float est_z;
    float rate_x;
    float rate_y;
    float rate_z;
    float target_x;
    float target_y;
    float target_z;
};

struct PACKED log_Gimbal3 {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t rl_torque_cmd;
    int16_t el_torque_cmd;
    int16_t az_torque_cmd;
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
};

struct PACKED log_POS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t lat;
    int32_t lng;
    float alt;
    float rel_alt;
};

struct PACKED log_POWR {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float Vcc;
    float Vservo;
    uint16_t flags;
};

struct PACKED log_EKF1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
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
};

struct PACKED log_EKF2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
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

struct PACKED log_EKF3 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
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
    uint16_t solution;
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
    float latitude;
    float longitude;
    float altitude;
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
    float   desired;
    float   P;
    float   I;
    float   D;
    float   FF;
    float   AFF;
};

struct PACKED log_Current {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    battery_voltage;
    float    current_amps;
    float    current_total;
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
    uint16_t dist1;
    uint16_t dist2;
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

struct PACKED log_GPS_SBF_EVENT {  
	LOG_PACKET_HEADER; 
	uint64_t time_us;
	uint32_t TOW;
	uint16_t WNc;
	uint8_t Mode;
	uint8_t Error;
	double Latitude;
	double Longitude;
	double Height;
	float Undulation;
	float Vn;
	float Ve;
	float Vu;
	float COG;
};

struct PACKED log_Esc {
    LOG_PACKET_HEADER;
    uint64_t time_us;     
    int16_t rpm;
    int16_t voltage;
    int16_t current;
    int16_t temperature;
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

struct PACKED log_DF_MAV_Stats {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    uint32_t seqno;
    uint32_t dropped;
    uint32_t retries;
    uint32_t resends;
    uint8_t internal_errors; // uint8_t - wishful thinking?
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

// #if SBP_HW_LOGGING

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

struct PACKED log_SbpRAW1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t msg_type;
    uint16_t sender_id;
    uint8_t msg_len;
    uint8_t data1[64];
};

struct PACKED log_SbpRAW2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t msg_type;
    uint8_t data2[192];
};

// #endif // SBP_HW_LOGGING

/*
Format characters in the format string for binary log messages
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

// messages for all boards
#define LOG_BASE_STRUCTURES \
    { LOG_FORMAT_MSG, sizeof(log_Format), \
      "FMT", "BBnNZ",      "Type,Length,Name,Format,Columns" },    \
    { LOG_PARAMETER_MSG, sizeof(log_Parameter), \
      "PARM", "QNf",        "TimeUS,Name,Value" },    \
    { LOG_GPS_MSG, sizeof(log_GPS), \
      "GPS",  "QBIHBcLLefffB", "TimeUS,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ,U" }, \
    { LOG_GPS2_MSG, sizeof(log_GPS), \
      "GPS2", "QBIHBcLLefefB", "TimeUS,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ,U" }, \
    { LOG_GPA_MSG,  sizeof(log_GPA), \
      "GPA",  "QCCCCBI", "TimeUS,VDop,HAcc,VAcc,SAcc,VV,SMS" }, \
    { LOG_GPA2_MSG, sizeof(log_GPA), \
      "GPA2", "QCCCCBI", "TimeUS,VDop,HAcc,VAcc,SAcc,VV,SMS" }, \
    { LOG_IMU_MSG, sizeof(log_IMU), \
      "IMU",  "QffffffIIfBB",     "TimeUS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,ErrG,ErrA,Temp,GyHlt,AcHlt" }, \
    { LOG_MESSAGE_MSG, sizeof(log_Message), \
      "MSG",  "QZ",     "TimeUS,Message"}, \
    { LOG_RCIN_MSG, sizeof(log_RCIN), \
      "RCIN",  "Qhhhhhhhhhhhhhh",     "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14" }, \
    { LOG_RCOUT_MSG, sizeof(log_RCOUT), \
      "RCOU",  "Qhhhhhhhhhhhhhh",     "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14" }, \
    { LOG_RSSI_MSG, sizeof(log_RSSI), \
      "RSSI",  "Qf",     "TimeUS,RXRSSI" }, \
    { LOG_BARO_MSG, sizeof(log_BARO), \
      "BARO",  "QffcfIf", "TimeUS,Alt,Press,Temp,CRt,SMS,Offset" }, \
    { LOG_POWR_MSG, sizeof(log_POWR), \
      "POWR","QffH","TimeUS,Vcc,VServo,Flags" },  \
    { LOG_CMD_MSG, sizeof(log_Cmd), \
      "CMD", "QHHHfffffff","TimeUS,CTot,CNum,CId,Prm1,Prm2,Prm3,Prm4,Lat,Lng,Alt" }, \
    { LOG_RADIO_MSG, sizeof(log_Radio), \
      "RAD", "QBBBBBHH", "TimeUS,RSSI,RemRSSI,TxBuf,Noise,RemNoise,RxErrors,Fixed" }, \
    { LOG_CAMERA_MSG, sizeof(log_Camera), \
      "CAM", "QIHLLeeeccC","TimeUS,GPSTime,GPSWeek,Lat,Lng,Alt,RelAlt,GPSAlt,Roll,Pitch,Yaw" }, \
    { LOG_TRIGGER_MSG, sizeof(log_Camera), \
      "TRIG", "QIHLLeeeccC","TimeUS,GPSTime,GPSWeek,Lat,Lng,Alt,RelAlt,GPSAlt,Roll,Pitch,Yaw" }, \
    { LOG_ARSP_MSG, sizeof(log_AIRSPEED), \
      "ARSP",  "QffcffB",  "TimeUS,Airspeed,DiffPress,Temp,RawPress,Offset,U" }, \
    { LOG_CURRENT_MSG, sizeof(log_Current), \
      "CURR", "Qfff","TimeUS,Volt,Curr,CurrTot" },\
    { LOG_CURRENT2_MSG, sizeof(log_Current), \
      "CUR2", "Qfff","TimeUS,Volt,Curr,CurrTot" }, \
	{ LOG_ATTITUDE_MSG, sizeof(log_Attitude),\
      "ATT", "QccccCCCC", "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,ErrRP,ErrYaw" }, \
    { LOG_COMPASS_MSG, sizeof(log_Compass), \
      "MAG", "QhhhhhhhhhBI",    "TimeUS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ,Health,S" }, \
    { LOG_MODE_MSG, sizeof(log_Mode), \
      "MODE", "QMBB",         "TimeUS,Mode,ModeNum,Rsn" }, \
    { LOG_RFND_MSG, sizeof(log_RFND), \
      "RFND", "QCC",         "TimeUS,Dist1,Dist2" }, \
    { LOG_DF_MAV_STATS, sizeof(log_DF_MAV_Stats), \
      "DMS", "IIIIIBBBBBBBBBB",         "TimeMS,N,Dp,RT,RS,Er,Fa,Fmn,Fmx,Pa,Pmn,Pmx,Sa,Smn,Smx" }

// messages for more advanced boards
#define LOG_EXTRA_STRUCTURES \
    { LOG_IMU2_MSG, sizeof(log_IMU), \
      "IMU2",  "QffffffIIfBB",     "TimeUS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,ErrG,ErrA,Temp,GyHlt,AcHlt" }, \
    { LOG_IMU3_MSG, sizeof(log_IMU), \
      "IMU3",  "QffffffIIfBB",     "TimeUS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,ErrG,ErrA,Temp,GyHlt,AcHlt" }, \
    { LOG_AHR2_MSG, sizeof(log_AHRS), \
      "AHR2","QccCfLL","TimeUS,Roll,Pitch,Yaw,Alt,Lat,Lng" }, \
    { LOG_POS_MSG, sizeof(log_POS), \
      "POS","QLLff","TimeUS,Lat,Lng,Alt,RelAlt" }, \
    { LOG_SIMSTATE_MSG, sizeof(log_AHRS), \
      "SIM","QccCfLL","TimeUS,Roll,Pitch,Yaw,Alt,Lat,Lng" }, \
    { LOG_EKF1_MSG, sizeof(log_EKF1), \
      "EKF1","QccCfffffffccc","TimeUS,Roll,Pitch,Yaw,VN,VE,VD,dPD,PN,PE,PD,GX,GY,GZ" }, \
    { LOG_EKF2_MSG, sizeof(log_EKF2), \
      "EKF2","Qbbbcchhhhhh","TimeUS,Ratio,AZ1bias,AZ2bias,VWN,VWE,MN,ME,MD,MX,MY,MZ" }, \
    { LOG_EKF3_MSG, sizeof(log_EKF3), \
      "EKF3","Qcccccchhhc","TimeUS,IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IVT" }, \
    { LOG_EKF4_MSG, sizeof(log_EKF4), \
      "EKF4","QcccccccbbHBHH","TimeUS,SV,SP,SH,SMX,SMY,SMZ,SVT,OFN,OFE,FS,TS,SS,GPS" }, \
    { LOG_EKF5_MSG, sizeof(log_EKF5), \
      "EKF5","QBhhhcccCC","TimeUS,normInnov,FIX,FIY,AFI,HAGL,offset,RI,meaRng,errHAGL" }, \
    { LOG_NKF1_MSG, sizeof(log_EKF1), \
      "NKF1","QccCfffffffccc","TimeUS,Roll,Pitch,Yaw,VN,VE,VD,dPD,PN,PE,PD,GX,GY,GZ" }, \
    { LOG_NKF2_MSG, sizeof(log_NKF2), \
      "NKF2","QbccccchhhhhhB","TimeUS,AZbias,GSX,GSY,GSZ,VWN,VWE,MN,ME,MD,MX,MY,MZ,MI" }, \
    { LOG_NKF3_MSG, sizeof(log_NKF3), \
      "NKF3","Qcccccchhhcc","TimeUS,IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IYAW,IVT" }, \
    { LOG_NKF4_MSG, sizeof(log_NKF4), \
      "NKF4","QcccccfbbHBHHb","TimeUS,SV,SP,SH,SM,SVT,errRP,OFN,OFE,FS,TS,SS,GPS,PI" }, \
    { LOG_NKF5_MSG, sizeof(log_EKF5), \
      "NKF5","QBhhhcccCC","TimeUS,normInnov,FIX,FIY,AFI,HAGL,offset,RI,meaRng,errHAGL" }, \
    { LOG_NKF6_MSG, sizeof(log_EKF1), \
      "NKF6","QccCfffffffccc","TimeUS,Roll,Pitch,Yaw,VN,VE,VD,dPD,PN,PE,PD,GX,GY,GZ" }, \
    { LOG_NKF7_MSG, sizeof(log_NKF2), \
      "NKF7","QbccccchhhhhhB","TimeUS,AZbias,GSX,GSY,GSZ,VWN,VWE,MN,ME,MD,MX,MY,MZ,MI" }, \
    { LOG_NKF8_MSG, sizeof(log_NKF3), \
      "NKF8","Qcccccchhhcc","TimeUS,IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IYAW,IVT" }, \
    { LOG_NKF9_MSG, sizeof(log_NKF4), \
      "NKF9","QcccccfbbHBHHb","TimeUS,SV,SP,SH,SM,SVT,errRP,OFN,OFE,FS,TS,SS,GPS,PI" }, \
    { LOG_TERRAIN_MSG, sizeof(log_TERRAIN), \
      "TERR","QBLLHffHH","TimeUS,Status,Lat,Lng,Spacing,TerrH,CHeight,Pending,Loaded" }, \
    { LOG_GPS_UBX1_MSG, sizeof(log_Ubx1), \
      "UBX1", "QBHBBH",  "TimeUS,Instance,noisePerMS,jamInd,aPower,agcCnt" }, \
    { LOG_GPS_UBX2_MSG, sizeof(log_Ubx2), \
      "UBX2", "QBbBbB", "TimeUS,Instance,ofsI,magI,ofsQ,magQ" }, \
    { LOG_GPS2_UBX1_MSG, sizeof(log_Ubx1), \
      "UBY1", "QBHBBH",  "TimeUS,Instance,noisePerMS,jamInd,aPower,agcCnt" }, \
    { LOG_GPS2_UBX2_MSG, sizeof(log_Ubx2), \
      "UBY2", "QBbBbB", "TimeUS,Instance,ofsI,magI,ofsQ,magQ" }, \
    { LOG_GPS_RAW_MSG, sizeof(log_GPS_RAW), \
      "GRAW", "QIHBBddfBbB", "TimeUS,WkMS,Week,numSV,sv,cpMes,prMes,doMes,mesQI,cno,lli" }, \
    { LOG_GPS_RAWH_MSG, sizeof(log_GPS_RAWH), \
      "GRXH", "QdHbBB", "TimeUS,rcvTime,week,leapS,numMeas,recStat" }, \
    { LOG_GPS_RAWS_MSG, sizeof(log_GPS_RAWS), \
      "GRXS", "QddfBBBHBBBBB", "TimeUS,prMes,cpMes,doMes,gnss,sv,freq,lock,cno,prD,cpD,doD,trk" }, \
    { LOG_GPS_SBF_EVENT_MSG, sizeof(log_GPS_SBF_EVENT), \
      "SBFE", "QIHBBdddfffff", "TimeUS,TOW,WN,Mode,Err,Lat,Long,Height,Undul,Vn,Ve,Vu,COG" }, \
    { LOG_ESC1_MSG, sizeof(log_Esc), \
      "ESC1",  "Qcccc", "TimeUS,RPM,Volt,Curr,Temp" }, \
    { LOG_ESC2_MSG, sizeof(log_Esc), \
      "ESC2",  "Qcccc", "TimeUS,RPM,Volt,Curr,Temp" }, \
    { LOG_ESC3_MSG, sizeof(log_Esc), \
      "ESC3",  "Qcccc", "TimeUS,RPM,Volt,Curr,Temp" }, \
    { LOG_ESC4_MSG, sizeof(log_Esc), \
      "ESC4",  "Qcccc", "TimeUS,RPM,Volt,Curr,Temp" }, \
    { LOG_ESC5_MSG, sizeof(log_Esc), \
      "ESC5",  "Qcccc", "TimeUS,RPM,Volt,Curr,Temp" }, \
    { LOG_ESC6_MSG, sizeof(log_Esc), \
      "ESC6",  "Qcccc", "TimeUS,RPM,Volt,Curr,Temp" }, \
    { LOG_ESC7_MSG, sizeof(log_Esc), \
      "ESC7",  "Qcccc", "TimeUS,RPM,Volt,Curr,Temp" }, \
    { LOG_ESC8_MSG, sizeof(log_Esc), \
      "ESC8",  "Qcccc", "TimeUS,RPM,Volt,Curr,Temp" }, \
    { LOG_COMPASS2_MSG, sizeof(log_Compass), \
      "MAG2","QhhhhhhhhhBI",    "TimeUS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ,Health,S" }, \
    { LOG_COMPASS3_MSG, sizeof(log_Compass), \
      "MAG3","QhhhhhhhhhBI",    "TimeUS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ,Health,S" }, \
    { LOG_ACC1_MSG, sizeof(log_ACCEL), \
      "ACC1", "QQfff",        "TimeUS,SampleUS,AccX,AccY,AccZ" }, \
    { LOG_ACC2_MSG, sizeof(log_ACCEL), \
      "ACC2", "QQfff",        "TimeUS,SampleUS,AccX,AccY,AccZ" }, \
    { LOG_ACC3_MSG, sizeof(log_ACCEL), \
      "ACC3", "QQfff",        "TimeUS,SampleUS,AccX,AccY,AccZ" }, \
    { LOG_GYR1_MSG, sizeof(log_GYRO), \
      "GYR1", "QQfff",        "TimeUS,SampleUS,GyrX,GyrY,GyrZ" }, \
    { LOG_GYR2_MSG, sizeof(log_GYRO), \
      "GYR2", "QQfff",        "TimeUS,SampleUS,GyrX,GyrY,GyrZ" }, \
    { LOG_GYR3_MSG, sizeof(log_GYRO), \
      "GYR3", "QQfff",        "TimeUS,SampleUS,GyrX,GyrY,GyrZ" }, \
    { LOG_PIDR_MSG, sizeof(log_PID), \
      "PIDR", "Qffffff",  "TimeUS,Des,P,I,D,FF,AFF" }, \
    { LOG_PIDP_MSG, sizeof(log_PID), \
      "PIDP", "Qffffff",  "TimeUS,Des,P,I,D,FF,AFF" }, \
    { LOG_PIDY_MSG, sizeof(log_PID), \
      "PIDY", "Qffffff",  "TimeUS,Des,P,I,D,FF,AFF" }, \
    { LOG_PIDA_MSG, sizeof(log_PID), \
      "PIDA", "Qffffff",  "TimeUS,Des,P,I,D,FF,AFF" }, \
    { LOG_PIDS_MSG, sizeof(log_PID), \
      "PIDS", "Qffffff",  "TimeUS,Des,P,I,D,FF,AFF" }, \
    { LOG_BAR2_MSG, sizeof(log_BARO), \
      "BAR2",  "QffcfIf", "TimeUS,Alt,Press,Temp,CRt,SMS,Offset" }, \
    { LOG_BAR3_MSG, sizeof(log_BARO), \
      "BAR3",  "QffcfI", "TimeUS,Alt,Press,Temp,CRt,SMS" }, \
    { LOG_VIBE_MSG, sizeof(log_Vibe), \
      "VIBE", "QfffIII",     "TimeUS,VibeX,VibeY,VibeZ,Clip0,Clip1,Clip2" }, \
    { LOG_IMUDT_MSG, sizeof(log_IMUDT), \
      "IMT","Qfffffffff","TimeUS,DelT,DelvT,DelaT,DelAX,DelAY,DelAZ,DelVX,DelVY,DelVZ" }, \
    { LOG_IMUDT2_MSG, sizeof(log_IMUDT), \
      "IMT2","Qfffffffff","TimeUS,DelT,DelvT,DelaT,DelAX,DelAY,DelAZ,DelVX,DelVY,DelVZ" }, \
    { LOG_IMUDT3_MSG, sizeof(log_IMUDT), \
      "IMT3","Qfffffffff","TimeUS,DelT,DelvT,DelaT,DelAX,DelAY,DelAZ,DelVX,DelVY,DelVZ" }, \
    { LOG_ORGN_MSG, sizeof(log_ORGN), \
      "ORGN","QBLLe","TimeUS,Type,Lat,Lng,Alt" }, \
    { LOG_RPM_MSG, sizeof(log_RPM), \
      "RPM",  "Qff", "TimeUS,rpm1,rpm2" }, \
    { LOG_GIMBAL1_MSG, sizeof(log_Gimbal1), \
      "GMB1", "Iffffffffff", "TimeMS,dt,dax,day,daz,dvx,dvy,dvz,jx,jy,jz" }, \
    { LOG_GIMBAL2_MSG, sizeof(log_Gimbal2), \
      "GMB2", "IBfffffffff", "TimeMS,es,ex,ey,ez,rx,ry,rz,tx,ty,tz" }, \
    { LOG_GIMBAL3_MSG, sizeof(log_Gimbal3), \
      "GMB3", "Ihhh", "TimeMS,rl_torque_cmd,el_torque_cmd,az_torque_cmd" }, \
    { LOG_RATE_MSG, sizeof(log_Rate), \
      "RATE", "Qffffffffffff",  "TimeUS,RDes,R,ROut,PDes,P,POut,YDes,Y,YOut,ADes,A,AOut" }

// #if SBP_HW_LOGGING
#define LOG_SBP_STRUCTURES \
    { LOG_MSG_SBPHEALTH, sizeof(log_SbpHealth), \
      "SBPH", "QIII",   "TimeUS,CrcError,LastInject,IARhyp" }, \
    { LOG_MSG_SBPRAW1, sizeof(log_SbpRAW1), \
      "SBR1", "QHHBZ",      "TimeUS,msg_type,sender_id,msg_len,d1" }, \
    { LOG_MSG_SBPRAW2, sizeof(log_SbpRAW2), \
      "SBR2", "QHZZZ",      "TimeUS,msg_type,d2,d3,d4" }
// #endif

#define LOG_COMMON_STRUCTURES LOG_BASE_STRUCTURES, LOG_EXTRA_STRUCTURES, LOG_SBP_STRUCTURES

// message types 0 to 128 reversed for vehicle specific use

// message types for common messages
enum LogMessages {
    LOG_FORMAT_MSG = 128,
    LOG_PARAMETER_MSG,
    LOG_GPS_MSG,
    LOG_GPS2_MSG,
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
    LOG_EKF1_MSG,
    LOG_EKF2_MSG,
    LOG_EKF3_MSG,
    LOG_EKF4_MSG,
    LOG_CMD_MSG,
    LOG_RADIO_MSG,
    LOG_ATRP_MSG,
    LOG_CAMERA_MSG,
    LOG_IMU3_MSG,
    LOG_TERRAIN_MSG,
    LOG_GPS_UBX1_MSG,
    LOG_GPS_UBX2_MSG,
    LOG_GPS2_UBX1_MSG,
    LOG_GPS2_UBX2_MSG,
    LOG_ESC1_MSG,
    LOG_ESC2_MSG,
    LOG_ESC3_MSG,
    LOG_ESC4_MSG,
    LOG_ESC5_MSG,
    LOG_ESC6_MSG,
    LOG_ESC7_MSG,
    LOG_ESC8_MSG,
    LOG_EKF5_MSG,
    LOG_BAR2_MSG,
    LOG_ARSP_MSG,
    LOG_ATTITUDE_MSG,
    LOG_CURRENT_MSG,
    LOG_CURRENT2_MSG,
    LOG_COMPASS_MSG,
    LOG_COMPASS2_MSG,
    LOG_COMPASS3_MSG,
    LOG_MODE_MSG,
    LOG_GPS_RAW_MSG,
    LOG_GPS_RAWH_MSG,
    LOG_GPS_RAWS_MSG,
	LOG_GPS_SBF_EVENT_MSG,
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
    LOG_VIBE_MSG,
    LOG_IMUDT_MSG,
    LOG_IMUDT2_MSG,
    LOG_IMUDT3_MSG,
    LOG_ORGN_MSG,
    LOG_RPM_MSG,
    LOG_GPA_MSG,
    LOG_GPA2_MSG,
    LOG_RFND_MSG,
    LOG_BAR3_MSG,
    LOG_NKF1_MSG,
    LOG_NKF2_MSG,
    LOG_NKF3_MSG,
    LOG_NKF4_MSG,
    LOG_NKF5_MSG,
    LOG_NKF6_MSG,
    LOG_NKF7_MSG,
    LOG_NKF8_MSG,
    LOG_NKF9_MSG,
    LOG_DF_MAV_STATS,

    LOG_MSG_SBPHEALTH,
    LOG_MSG_SBPLLH,
    LOG_MSG_SBPBASELINE,
    LOG_MSG_SBPTRACKING1,
    LOG_MSG_SBPTRACKING2,
    LOG_MSG_SBPRAW1,
    LOG_MSG_SBPRAW2,
    LOG_MSG_SBPRAWx,
    LOG_TRIGGER_MSG,

    LOG_GIMBAL1_MSG,
    LOG_GIMBAL2_MSG,
    LOG_GIMBAL3_MSG,
    LOG_RATE_MSG,
};

enum LogOriginType {
    ekf_origin = 0,
    ahrs_home = 1
};
