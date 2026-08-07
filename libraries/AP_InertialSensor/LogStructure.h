#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_INERTIALSENSOR \
    LOG_ACC_MSG, \
    LOG_GYR_MSG, \
    LOG_IMU_MSG, \
    LOG_ISBH_MSG, \
    LOG_ISBD_MSG, \
    LOG_VIBE_MSG

// @LoggerMessage: ACC
// @Description: IMU accelerometer data
// @Field: TimeUS: Time since system startup
// @Field: I: accelerometer sensor instance number
// @Field: SampleUS: time since system startup this sample was taken
// @Field: AccX: acceleration along X axis
// @Field: AccY: acceleration along Y axis
// @Field: AccZ: acceleration along Z axis
struct PACKED log_ACC {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    uint64_t sample_us;
    float AccX, AccY, AccZ;
};

// @LoggerMessage: GYR
// @Description: IMU gyroscope data
// @Field: TimeUS: Time since system startup
// @Field: I: gyroscope sensor instance number
// @Field: SampleUS: time since system startup this sample was taken
// @Field: GyrX: measured rotation rate about X axis
// @Field: GyrY: measured rotation rate about Y axis
// @Field: GyrZ: measured rotation rate about Z axis
struct PACKED log_GYR {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    uint64_t sample_us;
    float GyrX, GyrY, GyrZ;
};

// @LoggerMessage: IMU
// @Description: Inertial Measurement Unit data
// @Field: TimeUS: Time since system startup
// @Field: I: IMU sensor instance number
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
struct PACKED log_IMU {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    uint32_t gyro_error, accel_error;
    float temperature;
    uint8_t gyro_health, accel_health;
    uint16_t gyro_rate, accel_rate;
};

// @LoggerMessage: ISBH
// @Description: InertialSensor Batch Logging Header
// @Field: TimeUS: Time since system startup
// @Field: N: batch sequence number
// @Field: type: indicates if this is accel or gyro data
// @Field: instance: IMU sensor instance
// @Field: mul: multiplier to be applied to samples in this batch
// @Field: smp_cnt: samples in this batch
// @Field: SampleUS: timestamp of first sample
// @Field: smp_rate: rate at which samples have been collected
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

// @LoggerMessage: ISBD
// @Description: InertialSensor Batch Logging Data
// @Field: TimeUS: Time since system startup
// @Field: N: batch sequence number
// @Field: seqno: sample sequence number
// @Field: x: x-axis sample value
// @Field: y: y-axis sample value
// @Field: z: z-axis sample value
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

// @LoggerMessage: VIBE
// @Description: Processed (acceleration) vibration information
// @Field: TimeUS: Time since system startup
// @Field: IMU: Vibration instance number
// @Field: VibeX: Primary accelerometer filtered vibration, x-axis
// @Field: VibeY: Primary accelerometer filtered vibration, y-axis
// @Field: VibeZ: Primary accelerometer filtered vibration, z-axis
// @Field: Clip: Number of clipping events on 1st accelerometer
struct PACKED log_Vibe {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t imu;
    float vibe_x, vibe_y, vibe_z;
    uint32_t clipping;
};

#define LOG_STRUCTURE_FROM_INERTIALSENSOR        \
    { LOG_ACC_MSG, sizeof(log_ACC), \
      "ACC", "QBQfff",        "TimeUS,I,SampleUS,AccX,AccY,AccZ", "s#sooo", "F-F000" , true }, \
    { LOG_GYR_MSG, sizeof(log_GYR), \
      "GYR", "QBQfff",        "TimeUS,I,SampleUS,GyrX,GyrY,GyrZ", "s#sEEE", "F-F000" , true }, \
    { LOG_IMU_MSG, sizeof(log_IMU), \
      "IMU",  "QBffffffIIfBBHH", "TimeUS,I,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,EG,EA,T,GH,AH,GHz,AHz", "s#EEEooo--O--zz", "F-000000-----00" , true }, \
    { LOG_VIBE_MSG, sizeof(log_Vibe), \
      "VIBE", "QBfffI", "TimeUS,IMU,VibeX,VibeY,VibeZ,Clip", "s#ooo-", "F-000-" , true }, \
    { LOG_ISBH_MSG, sizeof(log_ISBH), \
      "ISBH", "QHBBHHQf", "TimeUS,N,type,instance,mul,smp_cnt,SampleUS,smp_rate", "s-----sz", "F-----F-" },  \
    { LOG_ISBD_MSG, sizeof(log_ISBD), \
      "ISBD", "QHHaaa", "TimeUS,N,seqno,x,y,z", "s--ooo", "F--???" },
