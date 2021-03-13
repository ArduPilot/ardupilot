#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_NAVEKF3 \
    LOG_XKF0_MSG, \
    LOG_XKF1_MSG, \
    LOG_XKF2_MSG, \
    LOG_XKF3_MSG, \
    LOG_XKF4_MSG, \
    LOG_XKF5_MSG, \
    LOG_XKFD_MSG, \
    LOG_XKFM_MSG, \
    LOG_XKFS_MSG, \
    LOG_XKQ_MSG,  \
    LOG_XKT_MSG,  \
    LOG_XKTV_MSG, \
    LOG_XKV1_MSG, \
    LOG_XKV2_MSG, \
    LOG_XKY0_MSG, \
    LOG_XKY1_MSG

// @LoggerMessage: XKF0
// @Description: EKF3 beacon sensor diagnostics
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
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
struct PACKED log_XKF0 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
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
struct PACKED log_XKF1 {
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
// @Field: IDX: Innovation in vehicle drag acceleration (X-axis component)
// @Field: IDY: Innovation in vehicle drag acceleration (Y-axis component)
// @Field: IS: Innovation in vehicle sideslip
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
    float innovDragX;
    float innovDragY;
    float innovSideslip;
};


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
// @Field: RErr: Accumulated relative error of this core with respect to active primary core
// @Field: ErSc: A consolidated error score where higher numbers are less healthy
struct PACKED log_XKF3 {
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
    float rerr;
    float errorScore;
};


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
// @Field: OFN: Most recent position reset (North component)
// @Field: OFE: Most recent position reset (East component)
// @Field: FS: Filter fault status
// @Field: TS: Filter timeout status bitmask (0:position measurement, 1:velocity measurement, 2:height measurement, 3:magnetometer measurement, 4:airspeed measurement)
// @Field: SS: Filter solution status
// @Field: GPS: Filter GPS status
// @Field: PI: Primary core index
struct PACKED log_XKF4 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    int16_t sqrtvarV;
    int16_t sqrtvarP;
    int16_t sqrtvarH;
    int16_t sqrtvarM;
    int16_t sqrtvarVT;
    float   tiltErr;
    float  offsetNorth;
    float  offsetEast;
    uint16_t faults;
    uint8_t timeouts;
    uint32_t solution;
    uint16_t gps;
    int8_t primary;
};


// @LoggerMessage: XKF5
// @Description: EKF3 Sensor innovations (primary core) and general dumping ground
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
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
struct PACKED log_XKF5 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
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


// @LoggerMessage: XKFD
// @Description: EKF3 Body Frame Odometry errors
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: IX: Innovation in velocity (X-axis)
// @Field: IY: Innovation in velocity (Y-axis)
// @Field: IZ: Innovation in velocity (Z-axis)
// @Field: IVX: Variance in velocity (X-axis)
// @Field: IVY: Variance in velocity (Y-axis)
// @Field: IVZ: Variance in velocity (Z-axis)
struct PACKED log_XKFD {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    float velInnovX;
    float velInnovY;
    float velInnovZ;
    float velInnovVarX;
    float velInnovVarY;
    float velInnovVarZ;
};

// @LoggerMessage: XKT
// @Description: EKF3 timing information
// @Field: TimeUS: Time since system startup
// @Field: C: EKF core this message instance applies to
// @Field: Cnt: count of samples used to create this message
// @Field: IMUMin: smallest IMU sample interval
// @Field: IMUMax: largest IMU sample interval
// @Field: EKFMin: low-passed achieved average time step rate for the EKF (minimum)
// @Field: EKFMax: low-passed achieved average time step rate for the EKF (maximum)
// @Field: AngMin: accumulated measurement time interval for the delta angle (minimum)
// @Field: AngMax: accumulated measurement time interval for the delta angle (maximum)
// @Field: VMin: accumulated measurement time interval for the delta velocity (minimum)
// @Field: VMax: accumulated measurement time interval for the delta velocity (maximum)
struct PACKED log_XKT {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    uint32_t timing_count;
    float dtIMUavg_min;
    float dtIMUavg_max;
    float dtEKFavg_min;
    float dtEKFavg_max;
    float delAngDT_min;
    float delAngDT_max;
    float delVelDT_min;
    float delVelDT_max;
};


// @LoggerMessage: XKFM
// @Description: EKF3 diagnostic data for on-ground-and-not-moving check
// @Field: TimeUS: Time since system startup
// @Field: C: EKF core this message instance applies to
// @Field: OGNM: True of on ground and not moving
// @Field: GLR: Gyroscope length ratio
// @Field: ALR: Accelerometer length ratio
// @Field: GDR: Gyroscope rate of change ratio
// @Field: ADR: Accelerometer rate of change ratio
struct PACKED log_XKFM {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    uint8_t ongroundnotmoving;
    float gyro_length_ratio;
    float accel_length_ratio;
    float gyro_diff_ratio;
    float accel_diff_ratio;
};


// @LoggerMessage: XKQ
// @Description: EKF3 quaternion defining the rotation from NED to XYZ (autopilot) axes
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: Q1: Quaternion a term
// @Field: Q2: Quaternion b term
// @Field: Q3: Quaternion c term
// @Field: Q4: Quaternion d term
struct PACKED log_XKQ {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    float q1;
    float q2;
    float q3;
    float q4;
};


// @LoggerMessage: XKFS
// @Description: EKF3 sensor selection
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: MI: compass selection index
// @Field: BI: barometer selection index
// @Field: GI: GPS selection index
// @Field: AI: airspeed selection index
struct PACKED log_XKFS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    uint8_t mag_index;
    uint8_t baro_index;
    uint8_t gps_index;
    uint8_t airspeed_index;
};

// @LoggerMessage: XKTV
// @Description: EKF3 Yaw Estimator States
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: TVS: Tilt Error Variance from symbolic equations (rad^2)
// @Field: TVD: Tilt Error Variance from difference method (rad^2)
struct PACKED log_XKTV {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    float tvs;
    float tvd;
};

// @LoggerMessage: XKV1
// @Description: EKF3 State variances (primary core)
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
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
// @Field: C: EKF3 core this data is for
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
struct PACKED log_XKV {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
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

#define LOG_STRUCTURE_FROM_NAVEKF3        \
    { LOG_XKF0_MSG, sizeof(log_XKF0), \
      "XKF0","QBBccCCcccccccc","TimeUS,C,ID,rng,innov,SIV,TR,BPN,BPE,BPD,OFH,OFL,OFN,OFE,OFD", "s#-m---mmmmmmmm", "F--B---BBBBBBBB" }, \
    { LOG_XKF1_MSG, sizeof(log_XKF1), \
      "XKF1","QBccCfffffffccce","TimeUS,C,Roll,Pitch,Yaw,VN,VE,VD,dPD,PN,PE,PD,GX,GY,GZ,OH", "s#ddhnnnnmmmkkkm", "F-BBB0000000BBBB" }, \
    { LOG_XKF2_MSG, sizeof(log_XKF2), \
      "XKF2","QBccccchhhhhhfff","TimeUS,C,AX,AY,AZ,VWN,VWE,MN,ME,MD,MX,MY,MZ,IDX,IDY,IS", "s#---nnGGGGGGoor", "F----BBCCCCCC000" }, \
    { LOG_XKF3_MSG, sizeof(log_XKF3), \
      "XKF3","QBcccccchhhccff","TimeUS,C,IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IYAW,IVT,RErr,ErSc", "s#nnnmmmGGG??--", "F-BBBBBBCCCBB00" }, \
    { LOG_XKF4_MSG, sizeof(log_XKF4), \
      "XKF4","QBcccccfffHBIHb","TimeUS,C,SV,SP,SH,SM,SVT,errRP,OFN,OFE,FS,TS,SS,GPS,PI", "s#------mm-----", "F-------??-----" }, \
    { LOG_XKF5_MSG, sizeof(log_XKF5), \
      "XKF5","QBBhhhcccCCfff","TimeUS,C,NI,FIX,FIY,AFI,HAGL,offset,RI,rng,Herr,eAng,eVel,ePos", "s#----m???mrnm", "F-----BBBBB000" }, \
    { LOG_XKFD_MSG, sizeof(log_XKFD), \
      "XKFD","QBffffff","TimeUS,C,IX,IY,IZ,IVX,IVY,IVZ", "s#------", "F-------" }, \
    { LOG_XKFM_MSG, sizeof(log_XKFM),   \
      "XKFM", "QBBffff", "TimeUS,C,OGNM,GLR,ALR,GDR,ADR", "s#-----", "F------"}, \
    { LOG_XKFS_MSG, sizeof(log_XKFS), \
      "XKFS","QBBBBB","TimeUS,C,MI,BI,GI,AI", "s#----", "F-----" }, \
    { LOG_XKQ_MSG, sizeof(log_XKQ), "XKQ", "QBffff", "TimeUS,C,Q1,Q2,Q3,Q4", "s#????", "F-????" }, \
    { LOG_XKT_MSG, sizeof(log_XKT),   \
      "XKT", "QBIffffffff", "TimeUS,C,Cnt,IMUMin,IMUMax,EKFMin,EKFMax,AngMin,AngMax,VMin,VMax", "s#sssssssss", "F-000000000"}, \
    { LOG_XKTV_MSG, sizeof(log_XKTV),                         \
      "XKTV", "QBff", "TimeUS,C,TVS,TVD", "s#rr", "F-00"}, \
    { LOG_XKV1_MSG, sizeof(log_XKV), \
      "XKV1","QBffffffffffff","TimeUS,C,V00,V01,V02,V03,V04,V05,V06,V07,V08,V09,V10,V11", "s#------------", "F-------------" }, \
    { LOG_XKV2_MSG, sizeof(log_XKV), \
      "XKV2","QBffffffffffff","TimeUS,C,V12,V13,V14,V15,V16,V17,V18,V19,V20,V21,V22,V23", "s#------------", "F-------------" },
