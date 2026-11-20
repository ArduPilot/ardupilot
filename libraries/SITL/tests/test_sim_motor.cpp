#include <AP_gtest.h>

#include <SITL/SIM_Motor.h>
#include <SITL/SITL_Input.h>
#include <AP_Motors/AP_Motors.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace SITL;

/*
  These tests exercise Motor::calculate_forces directly. Reversible
  (3D) thrust cannot be flown in SITL yet because no vehicle code
  emits a 1500-centred motor output, so the model is verified here
  instead.

  Assertions are on the thrust that comes out for a given PWM in,
  rather than on how it is computed, so they stay valid if the
  internals change.

  Thrust is in body-frame NED, so a motor pointing up produces
  NEGATIVE Z.
*/

static const uint16_t PWM_MIN = 1000;
static const uint16_t PWM_MAX = 2000;
static const float VOLTAGE = 12.6;

/*
  run a single calculate_forces call on a freshly configured motor and
  return the resulting thrust
*/
static Vector3f thrust_for_pwm(uint16_t pwm, int32_t rev_mask)
{
    Motor motor(AP_MOTORS_MOT_1, 45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);

    motor.setup_params(PWM_MIN, PWM_MAX,
                       0.15,               // spin min
                       0.95,               // spin max
                       0.65,               // expo
                       0,                  // slew max, zero disables the slew limiter
                       0.35,               // diagonal size
                       1.0,                // power factor
                       VOLTAGE,            // voltage max
                       0.05,               // effective prop area
                       30.0,               // velocity max
                       Vector3f {},        // position, derived from angle
                       Vector3f {0, 0, -1},// thrust vector, straight up
                       1.0,                // yaw factor
                       0.05,               // true prop area
                       0.0,                // momentum drag coefficient
                       rev_mask,
                       0);                 // motor offset

    sitl_input input {};
    input.servos[0] = pwm;

    Vector3f torque, thrust;
    motor.calculate_forces(input, 0, torque, thrust,
                           Vector3f {},    // still air
                           Vector3f {},    // no rotation
                           1.225,          // sea level air density
                           VOLTAGE,
                           false);         // no momentum drag

    return thrust;
}

/*
  with no bit set the motor must behave conventionally: no thrust at
  or below minimum PWM, thrust upwards above it, and never reversed
*/
TEST(SIM_Motor, conventional_when_mask_clear)
{
    const float accuracy = 0.001;

    EXPECT_NEAR(thrust_for_pwm(PWM_MIN, 0).z, 0, accuracy);

    const float mid = thrust_for_pwm(1500, 0).z;
    const float full = thrust_for_pwm(PWM_MAX, 0).z;

    // negative Z is up
    EXPECT_LT(mid, 0);
    EXPECT_LT(full, mid);

    // a conventional motor never produces downward thrust
    for (uint16_t pwm = PWM_MIN; pwm <= PWM_MAX; pwm += 100) {
        EXPECT_LE(thrust_for_pwm(pwm, 0).z, accuracy);
    }
}

/*
  with the bit set, mid PWM is the zero-thrust point
*/
TEST(SIM_Motor, reversible_neutral_is_mid_pwm)
{
    EXPECT_NEAR(thrust_for_pwm(1500, 1).z, 0, 0.001);
}

/*
  above neutral the thrust is up, below neutral it is down
*/
TEST(SIM_Motor, reversible_direction_follows_pwm)
{
    // negative Z is up
    EXPECT_LT(thrust_for_pwm(PWM_MAX, 1).z, 0);
    EXPECT_GT(thrust_for_pwm(PWM_MIN, 1).z, 0);
}

/*
  a reversible motor is symmetric about neutral: full reverse must
  produce the same magnitude of thrust as full forward
*/
TEST(SIM_Motor, reversible_is_symmetric)
{
    const float forward = thrust_for_pwm(PWM_MAX, 1).z;
    const float reverse = thrust_for_pwm(PWM_MIN, 1).z;

    EXPECT_NEAR(fabsf(forward), fabsf(reverse), 0.001);
}

/*
  a zero PWM means the channel is not being driven rather than a pulse
  width, and must produce no thrust in either direction
*/
TEST(SIM_Motor, undriven_channel_produces_no_thrust)
{
    EXPECT_NEAR(thrust_for_pwm(0, 1).z, 0, 0.001);
    EXPECT_NEAR(thrust_for_pwm(0, 0).z, 0, 0.001);
}

/*
  only the motors named in the mask are reversible; a motor whose bit
  is clear keeps conventional behaviour even when other bits are set
*/
TEST(SIM_Motor, mask_selects_individual_motors)
{
    // motor under test drives servo 0, so bit 0 selects it
    EXPECT_GT(thrust_for_pwm(PWM_MIN, 1).z, 0);

    // bits 1..3 set, bit 0 clear, so this motor stays conventional
    EXPECT_NEAR(thrust_for_pwm(PWM_MIN, 14).z, 0, 0.001);
}

AP_GTEST_MAIN()
