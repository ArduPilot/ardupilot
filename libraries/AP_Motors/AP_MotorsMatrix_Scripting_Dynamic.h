#pragma once
#if AP_SCRIPTING_ENABLED

#include "AP_MotorsMatrix.h"

class AP_MotorsMatrix_Scripting_Dynamic : public AP_MotorsMatrix {
public:

    // Constructor
    AP_MotorsMatrix_Scripting_Dynamic(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(speed_hz)
    {
        if (_singleton != nullptr) {
            AP_HAL::panic("AP_MotorsMatrix_Scripting_Dynamic must be singleton");
        }
        _singleton = this;
    };

    // get singleton instance
    static AP_MotorsMatrix_Scripting_Dynamic *get_singleton() {
        return _singleton;
    }

    struct factor_table {
        float roll[AP_MOTORS_MAX_NUM_MOTORS];
        float pitch[AP_MOTORS_MAX_NUM_MOTORS];
        float yaw[AP_MOTORS_MAX_NUM_MOTORS];
        float throttle[AP_MOTORS_MAX_NUM_MOTORS];
    };

    // base class method must not be used
    void init(motor_frame_class frame_class, motor_frame_type frame_type) override {};

    // Init to be called from scripting
    bool init(uint8_t expected_num_motors) override;

    // add a motor and give its testing order
    bool add_motor(uint8_t motor_num, uint8_t testing_order);

    // add a interpolation point table
    void load_factors(const factor_table &table);

    // output - sends commands to the motors
    void output_to_motors() override;

protected:

    // Do not apply thrust compensation, this is used by Quadplane tiltrotors
    // assume the compensation is done in the mixer and should not be done by quadplane
    void thrust_compensation(void) override {};

    const char* _get_frame_string() const override { return "Dynamic Matrix"; }

private:

    // True when received a factors table, will only init having received a table
    bool had_table;

    // For loading of new factors, cannot load while in use
    HAL_Semaphore _sem;

    static AP_MotorsMatrix_Scripting_Dynamic *_singleton;
};

#endif // AP_SCRIPTING_ENABLED
