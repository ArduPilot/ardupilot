#include "Blimp.h"

#if HAL_LOGGING_ENABLED

// Code to Write and Read packets from AP_Logger log memory
// Code to interact with the user to dump or erase logs

struct PACKED log_FINI {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float Right;
    float Front;
    float Down;
    float Yaw;
};

struct PACKED log_FINO {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float Fin1_Amp;
    float Fin1_Off;
    float Fin2_Amp;
    float Fin2_Off;
    float Fin3_Amp;
    float Fin3_Off;
    float Fin4_Amp;
    float Fin4_Off;
};

//Write a fin input packet
void Blimp::Write_FINI(float right, float front, float down, float yaw)
{
    const struct log_FINI pkt {
        LOG_PACKET_HEADER_INIT(LOG_FINI_MSG),
        time_us       : AP_HAL::micros64(),
        Right         : right,
        Front         : front,
        Down          : down,
        Yaw           : yaw
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

//Write a fin output packet
void Blimp::Write_FINO(float *amp, float *off)
{
    const struct log_FINO pkt {
        LOG_PACKET_HEADER_INIT(LOG_FINO_MSG),
        time_us       : AP_HAL::micros64(),
        Fin1_Amp      : amp[0],
        Fin1_Off      : off[0],
        Fin2_Amp      : amp[1],
        Fin2_Off      : off[1],
        Fin3_Amp      : amp[2],
        Fin3_Off      : off[2],
        Fin4_Amp      : amp[3],
        Fin4_Off      : off[3],
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    throttle_in;
    float    angle_boost;
    float    throttle_out;
    float    throttle_hover;
    float    desired_alt;
    float    inav_alt;
    int32_t  baro_alt;
    float    desired_rangefinder_alt;
    float    rangefinder_alt;
    float    terr_alt;
    int16_t  target_climb_rate;
    int16_t  climb_rate;
};

// Write PID packets
void Blimp::Log_Write_PIDs()
{
    logger.Write_PID(LOG_PIVN_MSG, pid_vel_xy.get_pid_info_x());
    logger.Write_PID(LOG_PIVE_MSG, pid_vel_xy.get_pid_info_y());
    logger.Write_PID(LOG_PIVD_MSG, pid_vel_z.get_pid_info());
    logger.Write_PID(LOG_PIVY_MSG, pid_vel_yaw.get_pid_info());
    logger.Write_PID(LOG_PIDN_MSG, pid_pos_xy.get_pid_info_x());
    logger.Write_PID(LOG_PIDE_MSG, pid_pos_xy.get_pid_info_y());
    logger.Write_PID(LOG_PIDD_MSG, pid_pos_z.get_pid_info());
    logger.Write_PID(LOG_PIDY_MSG, pid_pos_yaw.get_pid_info());
}

// Write an attitude packet
void Blimp::Log_Write_Attitude()
{
    //Attitude targets are all zero since Blimp doesn't have attitude control,
    //but the rest of the log message is useful.
    ahrs.Write_Attitude(Vector3f{0,0,0});
}

// Write an EKF and POS packet
void Blimp::Log_Write_EKF_POS()
{
    AP::ahrs().Log_Write();
}

struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    int16_t data_value;
};

// Write an int16_t data packet
UNUSED_FUNCTION
void Blimp::Log_Write_Data(LogDataID id, int16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT16_MSG),
time_us     : AP_HAL::micros64(),
id          : (uint8_t)id,
data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt16t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    uint16_t data_value;
};

// Write an uint16_t data packet
UNUSED_FUNCTION
void Blimp::Log_Write_Data(LogDataID id, uint16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT16_MSG),
time_us     : AP_HAL::micros64(),
id          : (uint8_t)id,
data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int32t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    int32_t data_value;
};

// Write an int32_t data packet
void Blimp::Log_Write_Data(LogDataID id, int32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT32_MSG),
time_us  : AP_HAL::micros64(),
id          : (uint8_t)id,
data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt32t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    uint32_t data_value;
};

// Write a uint32_t data packet
void Blimp::Log_Write_Data(LogDataID id, uint32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT32_MSG),
time_us     : AP_HAL::micros64(),
id          : (uint8_t)id,
data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Float {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    float data_value;
};

// Write a float data packet
UNUSED_FUNCTION
void Blimp::Log_Write_Data(LogDataID id, float value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Float pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_FLOAT_MSG),
time_us     : AP_HAL::micros64(),
id          : (uint8_t)id,
data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_ParameterTuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  parameter;     // parameter we are tuning, e.g. 39 is CH6_CIRCLE_RATE
    float    tuning_value;  // normalized value used inside tuning() function
    float    tuning_min;    // tuning minimum value
    float    tuning_max;    // tuning maximum value
};

void Blimp::Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, float tune_min, float tune_max)
{
    struct log_ParameterTuning pkt_tune = {
        LOG_PACKET_HEADER_INIT(LOG_PARAMTUNE_MSG),
time_us        : AP_HAL::micros64(),
parameter      : param,
tuning_value   : tuning_val,
tuning_min     : tune_min,
tuning_max     : tune_max
    };

    logger.WriteBlock(&pkt_tune, sizeof(pkt_tune));
}

// type and unit information can be found in
// libraries/AP_Logger/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information
const struct LogStructure Blimp::log_structure[] = {
    LOG_COMMON_STRUCTURES,

    // @LoggerMessage: FINI
    // @Description: Fin input
    // @Field: TimeUS: Time since system startup
    // @Field: R: Right
    // @Field: F: Front
    // @Field: D: Down
    // @Field: Y: Yaw

    {
        LOG_FINI_MSG, sizeof(log_FINI),
        "FINI",  "Qffff",     "TimeUS,R,F,D,Y", "s----", "F----"
    },

    // @LoggerMessage: FINO
    // @Description: Fin output
    // @Field: TimeUS: Time since system startup
    // @Field: F1A: Fin 1 Amplitude
    // @Field: F1O: Fin 1 Offset
    // @Field: F2A: Fin 2 Amplitude
    // @Field: F2O: Fin 2 Offset
    // @Field: F3A: Fin 3 Amplitude
    // @Field: F3O: Fin 3 Offset
    // @Field: F4A: Fin 4 Amplitude
    // @Field: F4O: Fin 4 Offset

    {
        LOG_FINO_MSG, sizeof(log_FINO),
        "FINO",  "Qffffffff",     "TimeUS,F1A,F1O,F2A,F2O,F3A,F3O,F4A,F4O", "s--------", "F--------"
    },

    // @LoggerMessage: PIDD,PIVN,PIVE,PIVD,PIVY
    // @Description: Proportional/Integral/Derivative gain values
    // @Field: TimeUS: Time since system startup
    // @Field: Tar: desired value
    // @Field: Act: achieved value
    // @Field: Err: error between target and achieved
    // @Field: P: proportional part of PID
    // @Field: I: integral part of PID
    // @Field: D: derivative part of PID
    // @Field: FF: controller feed-forward portion of response
    // @Field: DFF: controller derivative feed-forward portion of response
    // @Field: Dmod: scaler applied to D gain to reduce limit cycling
    // @Field: SRate: slew rate
    // @Field: Flags: bitmask of PID state flags
    // @FieldBitmaskEnum: Flags: log_PID_Flags
    {
        LOG_PIDD_MSG, sizeof(log_PID),
        "PIDD", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS
    },
    {
        LOG_PIVN_MSG, sizeof(log_PID),
        "PIVN", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS
    },
    {
        LOG_PIVE_MSG, sizeof(log_PID),
        "PIVE", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS
    },
    {
        LOG_PIVD_MSG, sizeof(log_PID),
        "PIVD", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS
    },
    {
        LOG_PIVY_MSG, sizeof(log_PID),
        "PIVY", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS
    },

    // @LoggerMessage: PTUN
    // @Description: Parameter Tuning information
    // @URL: https://ardupilot.org/blimp/docs/tuning.html#in-flight-tuning
    // @Field: TimeUS: Time since system startup
    // @Field: Param: Parameter being tuned
    // @Field: TunVal: Normalized value used inside tuning() function
    // @Field: TunMin: Tuning minimum limit
    // @Field: TunMax: Tuning maximum limit

    {
        LOG_PARAMTUNE_MSG, sizeof(log_ParameterTuning),
        "PTUN", "QBfff",         "TimeUS,Param,TunVal,TunMin,TunMax", "s----", "F----"
    },

    // @LoggerMessage: CTUN
    // @Description: Control Tuning information
    // @Field: TimeUS: Time since system startup
    // @Field: ThI: throttle input
    // @Field: ABst: angle boost
    // @Field: ThO: throttle output
    // @Field: ThH: calculated hover throttle
    // @Field: DAlt: desired altitude
    // @Field: Alt: achieved altitude
    // @Field: BAlt: barometric altitude
    // @Field: DSAlt: desired rangefinder altitude
    // @Field: SAlt: achieved rangefinder altitude
    // @Field: TAlt: terrain altitude
    // @Field: DCRt: desired climb rate
    // @Field: CRt: climb rate

    // @LoggerMessage: D16
    // @Description: Generic 16-bit-signed-integer storage
    // @Field: TimeUS: Time since system startup
    // @Field: Id: Data type identifier
    // @Field: Value: Value

    // @LoggerMessage: DU16
    // @Description: Generic 16-bit-unsigned-integer storage
    // @Field: TimeUS: Time since system startup
    // @Field: Id: Data type identifier
    // @Field: Value: Value

    // @LoggerMessage: D32
    // @Description: Generic 32-bit-signed-integer storage
    // @Field: TimeUS: Time since system startup
    // @Field: Id: Data type identifier
    // @Field: Value: Value

    // @LoggerMessage: DFLT
    // @Description: Generic float storage
    // @Field: TimeUS: Time since system startup
    // @Field: Id: Data type identifier
    // @Field: Value: Value

    // @LoggerMessage: DU32
    // @Description: Generic 32-bit-unsigned-integer storage
    // @Field: TimeUS: Time since system startup
    // @Field: Id: Data type identifier
    // @Field: Value: Value

    {
        LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
        "CTUN", "Qffffffefffhh", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DSAlt,SAlt,TAlt,DCRt,CRt", "s----mmmmmmnn", "F----00B000BB"
    },

    {
        LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),
        "D16",   "QBh",         "TimeUS,Id,Value", "s--", "F--"
    },
    {
        LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),
        "DU16",  "QBH",         "TimeUS,Id,Value", "s--", "F--"
    },
    {
        LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),
        "D32",   "QBi",         "TimeUS,Id,Value", "s--", "F--"
    },
    {
        LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),
        "DU32",  "QBI",         "TimeUS,Id,Value", "s--", "F--"
    },
    {
        LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),
        "DFLT",  "QBf",         "TimeUS,Id,Value", "s--", "F--"
    },
};

void Blimp::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by AP_Logger
    logger.Write_MessageF("Frame: %s", get_frame_string());
    logger.Write_Mode((uint8_t)control_mode, control_mode_reason);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}

void Blimp::log_init(void)
{
    logger.Init(log_structure, ARRAY_SIZE(log_structure));
}

#endif // HAL_LOGGING_ENABLED
