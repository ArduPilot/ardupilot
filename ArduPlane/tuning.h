/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  Plane transmitter tuning
 */
class Tuning
{
public:
    friend class Plane;

    Tuning(void);
    
    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    void check_input(void);
    
private:
    AP_Int8 channel;
    AP_Int8 parm;
    AP_Float minimum;
    AP_Float maximum;

    uint8_t last_input_pct = 255;
    uint32_t last_check_ms;

    struct PACKED log_ParameterTuning {
        LOG_PACKET_HEADER;
        uint64_t time_us;
        uint8_t  parameter;     // parameter we are tuning, e.g. 39 is CH6_CIRCLE_RATE
        float    tuning_value;  // normalized value used inside tuning() function
        float    tuning_low;    // tuning low end value
        float    tuning_high;   // tuning high end value
    };

    void Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, float tune_low, float tune_high);
    
    enum tuning_func {
        TUNING_NONE =                          0,

        // quadplane tuning
        TUNING_Q_RATE_ROLL_PITCH_KPI =         1,
        TUNING_Q_RATE_ROLL_PITCH_KP =          2,
        TUNING_Q_RATE_ROLL_PITCH_KI =          3,
        TUNING_Q_RATE_ROLL_PITCH_KD =          4,

        TUNING_Q_RATE_ROLL_KPI =               5,
        TUNING_Q_RATE_ROLL_KP =                6,
        TUNING_Q_RATE_ROLL_KI =                7,
        TUNING_Q_RATE_ROLL_KD =                8,

        TUNING_Q_RATE_PITCH_KPI =              9,
        TUNING_Q_RATE_PITCH_KP =              10,
        TUNING_Q_RATE_PITCH_KI =              11,
        TUNING_Q_RATE_PITCH_KD =              12,

        TUNING_Q_RATE_YAW_KPI =               13,
        TUNING_Q_RATE_YAW_KP =                14,
        TUNING_Q_RATE_YAW_KI =                15,
        TUNING_Q_RATE_YAW_KD =                16,

        TUNING_Q_ANG_ROLL_KP =                17,
        TUNING_Q_ANG_PITCH_KP =               18,
        TUNING_Q_ANG_YAW_KP =                 19,

        TUNING_Q_PXY_P =                      20,
        TUNING_Q_PZ_P  =                      21,

        TUNING_Q_VXY_P =                      22,
        TUNING_Q_VXY_I =                      23,
        TUNING_Q_VZ_P  =                      24,

        TUNING_Q_AZ_P =                       25,
        TUNING_Q_AZ_I =                       26,
        TUNING_Q_AZ_D  =                      27,

        // fixed wing tuning
        TUNING_RLL_P =                        28,
        TUNING_RLL_I =                        29,
        TUNING_RLL_D =                        30,
        TUNING_RLL_FF =                       31,

        TUNING_PIT_P =                        32,
        TUNING_PIT_I =                        33,
        TUNING_PIT_D =                        34,
        TUNING_PIT_FF =                       35,
    };
};
