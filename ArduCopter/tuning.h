/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Tuning/AP_Tuning.h>

/*
  Copter transmitter tuning
 */
class AP_Tuning_Copter : public AP_Tuning
{
private:
    // table of tuning sets
    static const tuning_set tuning_sets[];

    // table of tuning parameter names for reporting
    static const tuning_name tuning_names[];
    
public:
    // constructor
    AP_Tuning_Copter(void) : AP_Tuning(tuning_sets, tuning_names) {}

    static const struct AP_Param::GroupInfo  var_info[];
    
private:
    /*
      these are the individual tunable parameters. These are collected
      as part of sets of parameters that are tunable in one flight
     */
    enum tuning_func {
        TUNING_NONE =                        0,

        TUNING_RATE_ROLL_PI =                1,
        TUNING_RATE_ROLL_P =                 2,
        TUNING_RATE_ROLL_I =                 3,
        TUNING_RATE_ROLL_D =                 4,

        TUNING_RATE_PITCH_PI =               5,
        TUNING_RATE_PITCH_P =                6,
        TUNING_RATE_PITCH_I =                7,
        TUNING_RATE_PITCH_D =                8,

        TUNING_RATE_YAW_PI =                 9,
        TUNING_RATE_YAW_P =                 10,
        TUNING_RATE_YAW_I =                 11,
        TUNING_RATE_YAW_D =                 12,

        TUNING_ANG_ROLL_P =                 13,
        TUNING_ANG_PITCH_P =                14,
        TUNING_ANG_YAW_P =                  15,

        TUNING_PXY_P =                      16,
        TUNING_PZ_P  =                      17,

        TUNING_VXY_P =                      18,
        TUNING_VXY_I =                      19,
        TUNING_VZ_P  =                      20,

        TUNING_AZ_P =                       21,
        TUNING_AZ_I =                       22,
        TUNING_AZ_D  =                      23,
    };

    /*
      sets of parameters that can be selected for tuning. These are
      selected by chooing a value of TUNE_PARMSET higher than 100. So
      TUNE_PARMSET=101 chooses the set of parameters for
      RATE_ROLL_PITCH
     */
    enum tuning_sets {
        TUNING_SET_RATE_ROLL_PITCH =         1,
        TUNING_SET_RATE_ROLL =               2,
        TUNING_SET_RATE_PITCH =              3,
        TUNING_SET_RATE_YAW =                4,
        TUNING_SET_ANG_ROLL_PITCH =          5,
        TUNING_SET_VXY =                     6,
        TUNING_SET_AZ =                      7,
    };

    AP_Float *get_param_pointer(uint8_t parm) override;
    void save_value(uint8_t parm) override;
    void set_value(uint8_t parm, float value) override;
    void reload_value(uint8_t parm) override;
    float controller_error(uint8_t parm) override;

    // tuning set arrays
    static const uint8_t tuning_set_rate_roll_pitch[];
    static const uint8_t tuning_set_rate_roll[];
    static const uint8_t tuning_set_rate_pitch[];
    static const uint8_t tuning_set_rate_yaw[];
    static const uint8_t tuning_set_ang_roll_pitch[];
    static const uint8_t tuning_set_vxy[];
    static const uint8_t tuning_set_az[];
};


