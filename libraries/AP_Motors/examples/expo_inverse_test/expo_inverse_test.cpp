/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>

/* run with:
    ./waf configure --board linux
    ./waf build --targets examples/expo_inverse_test
    ./build/linux/examples/expo_inverse_test
*/

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class AP_MotorsMulticopter_test : public AP_MotorsMulticopter {
public:

    AP_MotorsMulticopter_test(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(speed_hz)
    {
    };

    // have to have these functions as they are pure virtual
    void init(motor_frame_class frame_class, motor_frame_type frame_type) override {};
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override {};
    void _output_test_seq(uint8_t motor_seq, int16_t pwm) override {};
    const char* _get_frame_string() const override { return "TEST"; }
    void output_armed_stabilizing() override {};
    void output_to_motors() override {};

    // helper function to allow setting of expo
    void set_expo(float v) { _thrust_curve_expo.set(v); }

};

AP_MotorsMulticopter_test motors;

/*
 *  rotation tests
 */
void setup(void)
{
    hal.console->begin(115200);
    hal.console->printf("\n\nMotors expo inverse test\n\n");

    const float expo_step = 0.01;
    const float throttle_step = 0.01;

    double max_diff = 0.0;
    float max_diff_throttle = 0;
    float max_diff_expo = 0;

    float expo = -1.0;
    motors.set_dt(1);
    while (expo < 1.0+expo_step*0.5) {
        hal.console->printf("expo: %0.4f\n",expo);
        motors.set_expo(expo);

        float throttle = 0.0;
        while (throttle < 1.0+throttle_step*0.5) {

            const float throttle_out = motors.actuator_to_thrust(motors.thrust_to_actuator(throttle));
            const double diff = fabsf(throttle_out - throttle);
            if (diff > max_diff) {
                max_diff_throttle = throttle;
                max_diff_expo = expo;
                max_diff = diff;
            }

            hal.console->printf("\tthrottle: %0.4f, error %0.8f\n", throttle, diff);

            throttle += throttle_step;
        }
        hal.console->printf("\n");
        expo += expo_step;
    }

    hal.console->printf("\nMotors expo inverse done, max error of %0.8f at expo %0.4f, throttle %0.4f\n\n", max_diff, max_diff_expo, max_diff_throttle);
}

void loop(void) {}

AP_HAL_MAIN();
