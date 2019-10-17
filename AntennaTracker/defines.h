#pragma once

// Controller modes
// ----------------

enum ControlMode {
    MANUAL=0,
    STOP=1,
    SCAN=2,
    SERVO_TEST=3,
    AUTO=10,
    INITIALISING=16
};

enum ServoType {
    SERVO_TYPE_POSITION=0,
    SERVO_TYPE_ONOFF=1,
    SERVO_TYPE_CR=2
};

enum AltSource {
	ALT_SOURCE_BARO=0,
	ALT_SOURCE_GPS=1,
	ALT_SOURCE_GPS_VEH_ONLY=2
};

enum class PWMDisarmed {
    ZERO = 0,
    TRIM,
};

//  Filter
#define SERVO_OUT_FILT_HZ               0.1f
#define G_Dt                            0.02f

//  Logging parameters
#define MASK_LOG_ATTITUDE               (1<<0)
#define MASK_LOG_GPS                    (1<<1)
#define MASK_LOG_RCIN                   (1<<2)
#define MASK_LOG_IMU                    (1<<3)
#define MASK_LOG_RCOUT                  (1<<4)
#define MASK_LOG_COMPASS                (1<<5)
#define MASK_LOG_CURRENT                (1<<6)
#define MASK_LOG_ANY                    0xFFFF

//  Logging messages
#define LOG_V_BAR_MSG                   0x04
#define LOG_V_POS_MSG                   0x05
