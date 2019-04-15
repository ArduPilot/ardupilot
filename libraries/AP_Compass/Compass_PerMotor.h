/*
  per-motor compass compensation
 */

#include <AP_Math/AP_Math.h>


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
        // simple low-pass on voltage
        voltage = 0.9f * voltage + 0.1f * _voltage;
    }

    void calibration_start(void);
    void calibration_update(void);
    void calibration_end(void);
    void compensate(Vector3f &offset);
    
private:
    Compass &compass;
    AP_Int8 enable;
    AP_Float expo;
    AP_Vector3f compensation[4];

    // base field on test start
    Vector3f base_field;
        
    // sum of calibration field samples
    Vector3f field_sum[4];

    // sum of output (voltage*scaledpwm) in calibration
    float output_sum[4];
        
    // count of calibration accumulation
    uint16_t count[4];

    // time a motor started in milliseconds
    uint32_t start_ms[4];
        
    // battery voltage
    float voltage;

    // is calibration running?
    bool running;

    // get scaled motor ouput
    float scaled_output(uint8_t motor);

    // map of motors
    bool have_motor_map;
    uint8_t motor_map[4];
};

