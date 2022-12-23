/*
  per-motor compass compensation
 */

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Math/AP_Math.h>

#ifndef AP_COMPASS_PMOT_ENABLED
#define AP_COMPASS_PMOT_ENABLED (BOARD_FLASH_SIZE>1024)
#endif

#ifndef AP_COMPASS_PMOT_USE_THRUST
#define AP_COMPASS_PMOT_USE_THRUST 1
#endif

#define AP_COMPASS_PMOT_MAX_NUM_MOTORS 4

class Compass;

// per-motor compass compensation class. Currently tied to quadcopters
// only, and single magnetometer
class Compass_PerMotor {
public:
    static const struct AP_Param::GroupInfo var_info[];

    Compass_PerMotor(Compass &_compass);

    bool enabled(void) const {
        return enable.get() != 0;
    }
    
    void set_voltage(float _voltage) {
#if !AP_COMPASS_PMOT_USE_THRUST
        // simple low-pass on voltage
        voltage = 0.9f * voltage + 0.1f * _voltage;
#endif
    }

    void calibration_start(void);
    void calibration_update(void);
    void calibration_end(void);
    Vector3f compensate(float current);
    void set_compensation(uint8_t motor, const Vector3f& offset) {
        compensation[motor].set_and_save(offset);
    }
    
private:
    Compass &compass;
    AP_Int8 enable;
    AP_Vector3f compensation[AP_COMPASS_PMOT_MAX_NUM_MOTORS];

    // base field on test start
    Vector3f base_field;
        
    // sum of calibration field samples
    Vector3f field_sum[AP_COMPASS_PMOT_MAX_NUM_MOTORS];

    // sum of output (voltage*scaledpwm) in calibration
    float output_sum[AP_COMPASS_PMOT_MAX_NUM_MOTORS];
        
    // count of calibration accumulation
    uint16_t count[AP_COMPASS_PMOT_MAX_NUM_MOTORS];

    // time a motor started in milliseconds
    uint32_t start_ms[AP_COMPASS_PMOT_MAX_NUM_MOTORS];
        
    // is calibration running?
    bool running;

    // get scaled motor ouput
    float scaled_output(uint8_t motor);
#if !AP_COMPASS_PMOT_USE_THRUST
    AP_Float expo;

    // battery voltage
    float voltage;

    // map of motors
    bool have_motor_map;
    uint8_t motor_map[AP_COMPASS_PMOT_MAX_NUM_MOTORS];
#endif
};

