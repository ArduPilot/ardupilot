/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Tuning/AP_Tuning.h>

/*
  Plane transmitter tuning
 */
class AP_Tuning_Plane : public AP_Tuning
{
private:
    // table of tuning sets
    static const tuning_set tuning_sets[];

    // table of tuning parameter names for reporting
    static const tuning_name tuning_names[];
    
public:
    // constructor
    AP_Tuning_Plane(void) : AP_Tuning(tuning_sets, tuning_names) {}
    
private:
    enum tuning_func {
        TUNING_NONE =                          0,

        // quadplane tuning
        TUNING_Q_RATE_ROLL_KPI =               1,
        TUNING_Q_RATE_ROLL_KP =                2,
        TUNING_Q_RATE_ROLL_KI =                3,
        TUNING_Q_RATE_ROLL_KD =                4,

        TUNING_Q_RATE_PITCH_KPI =              5,
        TUNING_Q_RATE_PITCH_KP =               6,
        TUNING_Q_RATE_PITCH_KI =               7,
        TUNING_Q_RATE_PITCH_KD =               8,

        TUNING_Q_RATE_YAW_KPI =                9,
        TUNING_Q_RATE_YAW_KP =                10,
        TUNING_Q_RATE_YAW_KI =                11,
        TUNING_Q_RATE_YAW_KD =                12,

        TUNING_Q_ANG_ROLL_KP =                13,
        TUNING_Q_ANG_PITCH_KP =               14,
        TUNING_Q_ANG_YAW_KP =                 15,

        TUNING_Q_PXY_P =                      16,
        TUNING_Q_PZ_P  =                      17,

        TUNING_Q_VXY_P =                      18,
        TUNING_Q_VXY_I =                      19,
        TUNING_Q_VZ_P  =                      20,

        TUNING_Q_AZ_P =                       21,
        TUNING_Q_AZ_I =                       22,
        TUNING_Q_AZ_D  =                      23,

        // fixed wing tuning
        TUNING_RLL_P =                        24,
        TUNING_RLL_I =                        25,
        TUNING_RLL_D =                        26,
        TUNING_RLL_FF =                       27,

        TUNING_PIT_P =                        28,
        TUNING_PIT_I =                        29,
        TUNING_PIT_D =                        30,
        TUNING_PIT_FF =                       31,
    };
    
    enum tuning_sets {
        TUNING_SET_NONE =                      0,

        TUNING_SET_Q_RATE_ROLL_PITCH =         1,
        TUNING_SET_Q_RATE_ROLL =               2,
        TUNING_SET_Q_RATE_PITCH =              3,
    };

    AP_Float *get_param_pointer(uint8_t parm);
    void save_value(uint8_t parm);
    void reload_value(uint8_t parm);
    void set_value(uint8_t parm, float value);

    // tuning set arrays
    static const uint8_t tuning_set_q_rate_roll_pitch[];
    static const uint8_t tuning_set_q_rate_roll[];
    static const uint8_t tuning_set_q_rate_pitch[];
};
