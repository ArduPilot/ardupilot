#include <AP_HAL/AP_HAL.h>

#include <SITL/SIM_Motor.h>
#include <SITL/SIM_Frame.h>
#include <SITL/SITL_Input.h>

#include <stdio.h>

/* run with:
    ./waf configure --board linux
    ./waf build --targets examples/EvaluateMotorModel
    ./build/linux/examples/EvaluateMotorModel Tools/autotest/models/Callisto.json 0 50
*/

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// helper to give access to protected functions
class Frame_helper : public SITL::Frame {
public:
    using SITL::Frame::Frame;

    void public_load_frame_params(const char *model_json) {
        load_frame_params(model_json);
    }

    float public_get_air_density(float alt_amsl) {
        return get_air_density(alt_amsl);
    }

    float get_pwm_min() { return model.pwmMin; }
    float get_pwm_max() { return model.pwmMax; }
    float get_spin_min() { return model.spin_min; }
    float get_spin_max() { return model.spin_max; }
    float get_prop_expo() { return model.propExpo; }
    float get_slew_max() { return model.slew_max; }
    float get_max_voltage() { return model.maxVoltage; }
    float get_mass() { return model.mass; }
    float get_ref_current() { return model.refCurrent; }
    float get_ref_voltage() { return model.refVoltage; }
    float get_ref_alt() { return model.refAlt; }
    float get_hover_thr_out() { return model.hoverThrOut; }
    float get_disk_area() { return model.disc_area; }
    float get_mdrag_coef() { return model.mdrag_coef; }
    float get_num_motors() { return model.num_motors; }

};

Frame_helper frame {nullptr, 0, nullptr};
SITL::Motor motor{0,0,1,1};

void setup(void)
{
    uint8_t argc;
    char * const *argv;

    hal.util->commandline_arguments(argc, argv);

    if (argc <= 2) {
        ::printf("pass .json model definition path and velocity\n");
        exit(0);
    }

    // parse JSON vehicle definition
    frame.public_load_frame_params(argv[1]);

    // duplicate of the calculations in SIM_Frame.cpp
    float ref_air_density = frame.public_get_air_density(frame.get_ref_alt());
    float hover_thrust = frame.get_mass() * GRAVITY_MSS;
    float hover_power = frame.get_ref_current() * frame.get_ref_voltage();
    float hover_velocity_out = 2 * hover_power / hover_thrust;
    float effective_disc_area = hover_thrust / (0.5 * ref_air_density * sq(hover_velocity_out));
    float velocity_max = hover_velocity_out / sqrtf(frame.get_hover_thr_out());
    float power_factor = hover_power / hover_thrust;
    float effective_prop_area = effective_disc_area / frame.get_num_motors();
    float true_prop_area = frame.get_disk_area() / frame.get_num_motors();

    motor.setup_params(frame.get_pwm_min(), frame.get_pwm_max(), frame.get_spin_min(), frame.get_spin_max(), frame.get_prop_expo(), frame.get_slew_max(),
                       0, power_factor, frame.get_max_voltage(), effective_prop_area, velocity_max,
                       Vector3f {}, Vector3f {}, 1, true_prop_area, frame.get_mdrag_coef());

    // parse inflow velocity and voltage
    const float velocity = strtof(argv[2], NULL);
    Vector3f velocity3 {0,0,-velocity};

    const float voltage = strtof(argv[3], NULL);

    ::printf("Motors at %0.2fv with %0.2f m/s inflow\n", voltage, velocity);

    uint64_t time = 0;
    hal.scheduler->stop_clock(time);
    const double time_step = 0.05;

    struct sitl_input input;


    ::printf("time, PWM, thrust, torque, current\n");
    const Vector3f gyro {};
    for (uint16_t PWM = frame.get_pwm_min(); PWM <= frame.get_pwm_max(); PWM++) {
        input.servos[0] = PWM;

        Vector3f torque;
        Vector3f thrust;
        motor.calculate_forces(input, 0, torque, thrust, velocity3, gyro, ref_air_density, voltage, true);

        ::printf("%0.2f, %u, %0.2f, %0.2f, %0.2f\n", time * 1.0e-6, PWM, -thrust.z, torque.z, motor.get_current());

        time += time_step*1e6;
        hal.scheduler->stop_clock(time);
    }
}

void loop(void)
{
    exit(0);
}

AP_HAL_MAIN();
