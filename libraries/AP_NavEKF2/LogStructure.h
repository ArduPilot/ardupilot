#pragma once

#include <AP_Logger/LogStructure.h>
#include <AP_AHRS/AP_AHRS_config.h>

#define LOG_IDS_FROM_NAVEKF2 \
    LOG_NKF0_MSG,  \
    LOG_NKF1_MSG,  \
    LOG_NKF2_MSG,  \
    LOG_NKF3_MSG,  \
    LOG_NKF4_MSG,  \
    LOG_NKF5_MSG,  \
    LOG_NKQ_MSG,   \
    LOG_NKT_MSG,   \
    LOG_NKY0_MSG,  \
    LOG_NKY1_MSG


// @LoggerMessage: NKF0
// @Description: EKF2 beacon sensor diagnostics
// @Field: TimeUS: Time since system startup
// @Field: C: EKF2 core this data is for
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
struct PACKED log_NKF0 {
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
struct PACKED log_NKF1 {
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


// @LoggerMessage: NKF2
// @Description: EKF2 estimator secondary outputs
// @Field: TimeUS: Time since system startup
// @Field: C: EKF2 core this data is for
// @Field: AZbias: Estimated accelerometer Z bias
// @Field: GSX: Gyro Scale Factor (X-axis)
// @Field: GSY: Gyro Scale Factor (Y-axis)
// @Field: GSZ: Gyro Scale Factor (Z-axis)
// @Field: VWN: Estimated wind velocity (moving-to-North component)
// @Field: VWE: Estimated wind velocity (moving-to-East component)
// @Field: MN: Magnetic field strength (North component)
// @Field: ME: Magnetic field strength (East component)
// @Field: MD: Magnetic field strength (Down component)
// @Field: MX: Magnetic field strength (body X-axis)
// @Field: MY: Magnetic field strength (body Y-axis)
// @Field: MZ: Magnetic field strength (body Z-axis)
// @Field: MI: Magnetometer used for data
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
// @Field: RErr: Accumulated relative error of this core with respect to active primary core
// @Field: ErSc: A consolidated error score where higher numbers are less healthy
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
    float rerr;
    float errorScore;
};


// @LoggerMessage: NKF4
// @Description: EKF2 variances  SV, SP, SH and SM are probably best described as 'Squared Innovation Test Ratios' where values <1 tells us the measurement was accepted and >1 tells us it was rejected. They represent the square of the (innovation / maximum allowed innovation) where the innovation is the difference between predicted and measured value and the maximum allowed innovation is determined from the uncertainty of the measurement, uncertainty of the prediction and scaled using the number of standard deviations set by the innovation gate parameter for that measurement, eg EK2_MAG_I_GATE, EK2_HGT_I_GATE, etc
// @Field: TimeUS: Time since system startup
// @Field: C: EKF2 core this data is for
// @Field: SV: Square root of the velocity variance
// @Field: SP: Square root of the position variance
// @Field: SH: Square root of the height variance
// @Field: SM: Magnetic field variance
// @Field: SVT: tilt error convergence metric
// @Field: errRP: Filtered error in roll/pitch estimate
// @Field: OFN: Most recent position reset (North component)
// @Field: OFE: Most recent position reset (East component)
// @Field: FS: Filter fault status
// @Field: TS: Filter timeout status bitmask (0:position measurement, 1:velocity measurement, 2:height measurement, 3:magnetometer measurement, 4:airspeed measurement)
// @Field: SS: Filter solution status
// @FieldBitmaskEnum: SS: NavFilterStatusBit
// @Field: GPS: Filter GPS status
// @Field: PI: Primary core index
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
    float  offsetNorth;
    float  offsetEast;
    uint16_t faults;
    uint8_t timeouts;
    uint32_t solution;
    uint16_t gps;
    int8_t primary;
};


// @LoggerMessage: NKF5
// @Description: EKF2 Sensor innovations (primary core) and general dumping ground
// @Field: TimeUS: Time since system startup
// @Field: C: EKF2 core this data is for
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
struct PACKED log_NKF5 {
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


// @LoggerMessage: NKQ
// @Description: EKF2 quaternion defining the rotation from NED to XYZ (autopilot) axes
// @Field: TimeUS: Time since system startup
// @Field: C: EKF2 core this data is for
// @Field: Q1: Quaternion a term
// @Field: Q2: Quaternion b term
// @Field: Q3: Quaternion c term
// @Field: Q4: Quaternion d term
struct PACKED log_NKQ {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    float q1;
    float q2;
    float q3;
    float q4;
};

// @LoggerMessage: NKT
// @Description: EKF2 timing information
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
struct PACKED log_NKT {
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

#if HAL_NAVEKF2_AVAILABLE
#define LOG_STRUCTURE_FROM_NAVEKF2        \
    { LOG_NKF0_MSG, sizeof(log_NKF0), \
      "NKF0","QBBccCCcccccccc","TimeUS,C,ID,rng,innov,SIV,TR,BPN,BPE,BPD,OFH,OFL,OFN,OFE,OFD", "s#-m---mmmmmmmm", "F--B---BBBBBBBB" , true }, \
    { LOG_NKF1_MSG, sizeof(log_NKF1), \
      "NKF1","QBccCfffffffccce","TimeUS,C,Roll,Pitch,Yaw,VN,VE,VD,dPD,PN,PE,PD,GX,GY,GZ,OH", "s#ddhnnnnmmmkkkm", "F-BBB0000000BBBB" , true }, \
    { LOG_NKF2_MSG, sizeof(log_NKF2), \
      "NKF2","QBbccccchhhhhhB","TimeUS,C,AZbias,GSX,GSY,GSZ,VWN,VWE,MN,ME,MD,MX,MY,MZ,MI", "s#----nnGGGGGG-", "F-----BBCCCCCC-" , true }, \
    { LOG_NKF3_MSG, sizeof(log_NKF3), \
      "NKF3","QBcccccchhhccff","TimeUS,C,IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IYAW,IVT,RErr,ErSc", "s#nnnmmmGGGd?--", "F-BBBBBBCCCBB00" , true }, \
    { LOG_NKF4_MSG, sizeof(log_NKF4), \
      "NKF4","QBcccccfffHBIHb","TimeUS,C,SV,SP,SH,SM,SVT,errRP,OFN,OFE,FS,TS,SS,GPS,PI", "s#------mm-----", "F-------??-----" , true }, \
    { LOG_NKF5_MSG, sizeof(log_NKF5), \
      "NKF5","QBBhhhcccCCfff","TimeUS,C,NI,FIX,FIY,AFI,HAGL,offset,RI,rng,Herr,eAng,eVel,ePos", "s#----m???mrnm", "F-----BBBBB000" , true }, \
    { LOG_NKQ_MSG, sizeof(log_NKQ), "NKQ", "QBffff", "TimeUS,C,Q1,Q2,Q3,Q4", "s#----", "F-0000" , true }, \
    { LOG_NKT_MSG, sizeof(log_NKT),   \
      "NKT", "QBIffffffff", "TimeUS,C,Cnt,IMUMin,IMUMax,EKFMin,EKFMax,AngMin,AngMax,VMin,VMax", "s#sssssssss", "F-000000000", true },
#else
#define LOG_STRUCTURE_FROM_NAVEKF2
#endif
