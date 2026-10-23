/*
  Coverage for the moment of inertia term in SITL::Plane, see issue #26972.

  getTorque() returns aerodynamic moments in N.m, and calculate_forces()
  divides them by coefficient.moment_of_inertia to get angular acceleration.

  Two things need protecting:

  - the unit default must reproduce the previous behaviour exactly, since the
    aerodynamic coefficients and the default gains are tuned around it;
  - the division must actually happen, otherwise a later change could drop it
    and nothing would notice.
 */

#include <AP_gtest.h>

#include <SITL/SIM_Plane.h>
#include <SITL/SITL.h>
#include <AP_Baro/AP_Baro.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace SITL;

// SITL::Plane construction reaches AP::sitl() and, through the battery
// temperature setup, AP::baro().  Both singletons must exist.  They are
// static because SITL::SIM is far too large for the stack.
static SITL::SIM sitl_singleton;
static AP_Baro baro_singleton;

static const float test_airspeed = 20.0;
static const uint16_t aileron_pwm = 1700;
static const uint16_t centred_pwm = 1500;
static const uint16_t throttle_off_pwm = 1000;

// deflection the servo model produces for aileron_pwm over the default
// 1000-2000 range, i.e. what calculate_forces() feeds to the physics
static const float test_aileron = (aileron_pwm - 1500) / 500.0;

// memcmp rather than ==, because gtest's float comparison trips
// -Werror=float-equal and these checks are deliberately bit-exact
#define EXPECT_BIT_IDENTICAL(a, b) \
    EXPECT_EQ(0, memcmp(&(a), &(b), sizeof(Vector3f)))

/*
  exposes SITL::Plane's protected physics entry points and flight state, so
  that models can be driven from an identical starting point.

  final because SITL::Aircraft has no virtual destructor, the simulator never
  deleting a model through a base pointer, and deleting a probe would
  otherwise trip -Wdelete-non-virtual-dtor
 */
class PlaneProbe final : public SITL::Plane {
public:
    using SITL::Plane::Plane;

    // level flight, well clear of the ground so the on_ground() friction
    // term in calculate_forces() cannot apply.
    // set_start_location() is what normally establishes these, and a unit
    // test does not call it, so hagl() has to be pinned down here
    void set_flight_state(void) {
        dcm.identity();
        gyro.zero();
        velocity_ef = Vector3f(test_airspeed, 0, 0);
        velocity_air_ef = velocity_ef;
        velocity_air_bf = Vector3f(test_airspeed, 0, 0);
        airspeed = test_airspeed;
        airspeed_pitot = test_airspeed;
        ground_level = 0;
        local_ground_level = 0;
        home.alt = 0;
        position = Vector3d(0, 0, -100);
    }

    // the full model path, exactly as the simulator runs it
    void run(const struct sitl_input &input, Vector3f &rot_accel_out, Vector3f &accel_body_out) {
        set_flight_state();
        calculate_forces(input, rot_accel_out);
        accel_body_out = accel_body;
    }

    // the aerodynamic force and moment alone, for the same state and the
    // same deflection that run() ends up using
    void aero(float aileron, Vector3f &force_out, Vector3f &torque_out) {
        set_flight_state();
        angle_of_attack = atan2f(velocity_air_bf.z, velocity_air_bf.x);
        beta = atan2f(velocity_air_bf.y, velocity_air_bf.x);
        force_out = getForce(aileron, 0, 0);
        torque_out = getTorque(aileron, 0, 0, 0, force_out);
    }

    void set_moment_of_inertia(const Vector3f &moi) { coefficient.moment_of_inertia = moi; }
    float get_mass_kg(void) const { return mass; }
    bool is_on_ground(void) const { return on_ground(); }
};

// no servo dynamics, so every model sees the commanded deflection at once
// rather than an independently settling filter state
static void disable_servo_model(void)
{
    sitl_singleton.servo.servo_speed.set(0);
    sitl_singleton.servo.servo_delay.set(0);
    sitl_singleton.servo.servo_filter.set(0);
}

static void roll_demand_input(struct sitl_input &input)
{
    for (uint8_t i = 0; i < ARRAY_SIZE(input.servos); i++) {
        input.servos[i] = centred_pwm;
    }
    input.servos[0] = aileron_pwm;      // roll demand
    input.servos[2] = throttle_off_pwm; // no thrust: keeps the linear channel clean
}

/*
  With the unit default, rot_accel must still be the raw aerodynamic moment,
  bit for bit, and must still not depend on mass.

  "plane" (2kg, SIM_Plane.cpp) and "plane-heavy" (8kg) differ only in mass,
  so they isolate it. accel_body is the control: calculate_forces() divides it
  by mass explicitly, so it must show the 4x ratio, which proves the
  measurement is sensitive to mass at all.
 */
TEST(SIM_Plane, unit_inertia_preserves_legacy_behaviour)
{
    disable_servo_model();

    struct sitl_input input {};
    roll_demand_input(input);

    // allocated the way the simulator allocates its model, so that the
    // calloc-backed operator new applies here too
    PlaneProbe *light = NEW_NOTHROW PlaneProbe("plane");
    PlaneProbe *heavy = NEW_NOTHROW PlaneProbe("plane-heavy");
    ASSERT_NE(nullptr, light);
    ASSERT_NE(nullptr, heavy);

    Vector3f light_rot_accel, light_accel_body;
    Vector3f heavy_rot_accel, heavy_accel_body;
    light->run(input, light_rot_accel, light_accel_body);
    heavy->run(input, heavy_rot_accel, heavy_accel_body);

    Vector3f light_force, light_torque, heavy_force, heavy_torque;
    light->aero(test_aileron, light_force, light_torque);
    heavy->aero(test_aileron, heavy_force, heavy_torque);

    const float mass_ratio = light->get_mass_kg() / heavy->get_mass_kg();

    ::printf("mass                      : %.3f kg vs %.3f kg (ratio %.6f)\n",
             light->get_mass_kg(), heavy->get_mass_kg(), mass_ratio);
    ::printf("aero force  N     light   : %.9f %.9f %.9f\n", light_force.x, light_force.y, light_force.z);
    ::printf("aero force  N     heavy   : %.9f %.9f %.9f\n", heavy_force.x, heavy_force.y, heavy_force.z);
    ::printf("aero moment Nm    light   : %.9f %.9f %.9f\n", light_torque.x, light_torque.y, light_torque.z);
    ::printf("aero moment Nm    heavy   : %.9f %.9f %.9f\n", heavy_torque.x, heavy_torque.y, heavy_torque.z);
    ::printf("accel_body  m/s/s light   : %.9f %.9f %.9f\n", light_accel_body.x, light_accel_body.y, light_accel_body.z);
    ::printf("accel_body  m/s/s heavy   : %.9f %.9f %.9f\n", heavy_accel_body.x, heavy_accel_body.y, heavy_accel_body.z);
    ::printf("rot_accel   rad/s/s light : %.9f %.9f %.9f\n", light_rot_accel.x, light_rot_accel.y, light_rot_accel.z);
    ::printf("rot_accel   rad/s/s heavy : %.9f %.9f %.9f\n", heavy_rot_accel.x, heavy_rot_accel.y, heavy_rot_accel.z);
    ::printf("accel_body heavy/light    : %.9f\n", heavy_accel_body.z / light_accel_body.z);
    ::printf("rot_accel  heavy/light    : %.9f\n", heavy_rot_accel.x / light_rot_accel.x);

    // airborne, so the ground friction term cannot corrupt the linear channel
    EXPECT_FALSE(light->is_on_ground());
    EXPECT_FALSE(heavy->is_on_ground());

    // the aileron actually produces a rolling moment
    EXPECT_GT(fabsf(light_torque.x), 0.01);

    // the aerodynamic moment is a property of airflow and geometry, so it
    // does not depend on mass. Same for the aerodynamic force.
    EXPECT_BIT_IDENTICAL(light_torque, heavy_torque);
    EXPECT_BIT_IDENTICAL(light_force, heavy_force);

    // control: the linear channel is divided by mass, so it scales
    EXPECT_NEAR(heavy_accel_body.z / light_accel_body.z, mass_ratio, 1.0e-6);

    // dividing by the unit default leaves the moment untouched
    EXPECT_BIT_IDENTICAL(light_rot_accel, light_torque);
    EXPECT_BIT_IDENTICAL(heavy_rot_accel, heavy_torque);
    EXPECT_BIT_IDENTICAL(light_rot_accel, heavy_rot_accel);

    delete light;
    delete heavy;
}

/*
  With a non-unit inertia, each axis of rot_accel must be the moment on that
  axis divided by that axis' inertia. Distinct powers of two are used so the
  division is exact in binary and a swapped axis cannot pass.
 */
TEST(SIM_Plane, explicit_inertia_scales_angular_acceleration)
{
    disable_servo_model();

    struct sitl_input input {};
    roll_demand_input(input);

    const Vector3f moi{2, 4, 8};

    // allocated the way the simulator allocates its model, so that the
    // calloc-backed operator new applies here too
    PlaneProbe *reference = NEW_NOTHROW PlaneProbe("plane");
    PlaneProbe *scaled = NEW_NOTHROW PlaneProbe("plane");
    ASSERT_NE(nullptr, reference);
    ASSERT_NE(nullptr, scaled);
    scaled->set_moment_of_inertia(moi);

    Vector3f unused_force, torque;
    reference->aero(test_aileron, unused_force, torque);

    Vector3f scaled_rot_accel, scaled_accel_body;
    scaled->run(input, scaled_rot_accel, scaled_accel_body);

    const Vector3f expected{torque.x / moi.x, torque.y / moi.y, torque.z / moi.z};

    ::printf("moment of inertia         : %.3f %.3f %.3f\n", moi.x, moi.y, moi.z);
    ::printf("aero moment Nm            : %.9f %.9f %.9f\n", torque.x, torque.y, torque.z);
    ::printf("rot_accel expected        : %.9f %.9f %.9f\n", expected.x, expected.y, expected.z);
    ::printf("rot_accel measured        : %.9f %.9f %.9f\n",
             scaled_rot_accel.x, scaled_rot_accel.y, scaled_rot_accel.z);

    // the moment must be non-trivial on the axes being checked, otherwise
    // dividing it would prove nothing
    EXPECT_GT(fabsf(torque.x), 0.01);
    EXPECT_GT(fabsf(torque.y), 0.01);

    EXPECT_BIT_IDENTICAL(scaled_rot_accel, expected);

    delete reference;
    delete scaled;
}

AP_GTEST_MAIN()
