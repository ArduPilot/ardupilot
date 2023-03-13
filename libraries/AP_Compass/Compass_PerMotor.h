/*
  per-motor compass compensation
 */

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Math/AP_Math.h>

#define AP_COMPASS_PMOT_MAX_NUM_MOTORS 4

class Compass;

// per-motor compass compensation class. Currently tied to quadcopters
// only, and single magnetometer
class Compass_PerMotor {
public:
    static const struct AP_Param::GroupInfo var_info[];

    Compass_PerMotor();

    bool enabled(void) const {
        return enable.get() != 0;
    }
    
    Vector3f compensate(float current);
    void set_compensation(uint8_t motor, const Vector3f& offset) {
        compensation[motor].set_and_save(offset);
    }

    void copy_from(const Compass_PerMotor per_motor);
    
private:
    AP_Int8 enable;
    AP_Vector3f compensation[AP_COMPASS_PMOT_MAX_NUM_MOTORS];

    // get scaled motor ouput
    float scaled_output(uint8_t motor);
};

