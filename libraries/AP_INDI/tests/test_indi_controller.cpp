#include <AP_gtest.h>

#include <AP_INDI/AP_INDI_config.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// the library is only built for boards that enable it (SITL by default);
// elsewhere this file compiles to an empty test binary
#if AP_INDI_ENABLED

#include <AP_INDI/AP_INDI.h>
#include <GCS_MAVLink/GCS_Dummy.h>

// the controller reports sign flips and fallbacks over MAVLink
GCS_Dummy _gcs;

// a rigid body with diagonal effectiveness, first-order actuators and
// a pure delay: omega_dot = G u_f, run at the control loop rate. This is
// the plant model SPEC.md section 7 assumes, so the controller should
// invert it exactly once G is identified.
class Plant
{
public:
    Plant(float dt, const Vector3f &g, float tau, uint8_t delay) :
        _dt(dt), _g(g), _tau(tau), _delay(delay)
    {
        for (uint8_t i = 0; i < 32; i++) {
            _ubuf[i].zero();
        }
    }
    // advance one step with command u, return the gyro reading
    Vector3f step(const Vector3f &u)
    {
        _ubuf[_head] = u;
        const Vector3f &u_del = _ubuf[(_head + 32 - _delay) % 32];
        _head = (_head + 1) % 32;
        if (_tau <= _dt) {
            _uf = u_del;
        } else {
            _uf += (u_del - _uf) * (_dt / _tau);
        }
        Vector3f wd(_g.x * _uf.x, _g.y * _uf.y, _g.z * _uf.z);
        _omega += wd * _dt;
        return _omega;
    }
    // what the actuators are doing now, as a motor-speed sensor would report it
    const Vector3f &actuator() const { return _uf; }
    Vector3f _omega;
private:
    float _dt;
    Vector3f _g;
    float _tau;
    uint8_t _delay;
    Vector3f _ubuf[32];
    uint8_t _head = 0;
    Vector3f _uf;
};

static constexpr float DT = 1.0f / 400.0f;  // Copter loop rate

static AP_INDI::Params default_params()
{
    AP_INDI::Params p;
    p.g_prior[0] = 400;
    p.g_prior[1] = 400;
    p.g_prior[2] = 40;
    p.prior_rel_sigma = 0.7f;
    p.rate_gain = 20;
    p.overdrive_max = 2.5f;
    p.act_tau_s = 0.010f;
    p.act_delay_s = 0.010f;
    p.sensor_filt_hz = 0;  // the plant's gyro is unfiltered
    p.du_min = 0.05f;
    p.window_s = 0.010f;
    p.tikhonov = 1.0e-3f;
    p.pulses_enabled = false;  // excitation comes from the rate targets below
    return p;
}

static AP_INDI::Inputs make_inputs(const Vector3f &gyro, const Vector3f &target)
{
    AP_INDI::Inputs in;
    in.gyro = gyro;
    in.rate_target = target;
    in.u_applied.zero();
    for (uint8_t i = 0; i < 3; i++) {
        in.saturated[i] = false;
    }
    in.gyro_healthy = true;
    in.flying = true;
    in.launched = true;
    in.ident_allowed = true;
    in.pulses_allowed = false;
    return in;
}

// Alternating rate steps on every axis excite the identification. The
// plant is 1.5x more effective than the prior; after a second the
// identified G must match the plant and the rate must track its
// target. This is the whole loop (differentiator, lag model, RLS,
// inversion) working together, which the identifier's own tests
// cannot show. Yaw is asked for a smaller step because its authority
// limit (half the prior, 20 rad/s^2) cannot reverse 2 rad/s in 100 ms.
TEST(INDIController, IdentifiesAndTracks)
{
    AP_INDI indi;
    indi.init(DT, default_params());
    indi.reset(Vector3f());
    const Vector3f g_true(600, 600, 60);
    Plant plant(DT, g_true, 0.010f, 4);

    Vector3f gyro;
    float max_err_late = 0;
    const uint32_t steps = 800;  // 2 s
    for (uint32_t k = 0; k < steps; k++) {
        // square wave with a 100 ms half period, staggered per axis:
        // 1 rad/s on roll and pitch, 0.3 rad/s on yaw
        const float amplitude[3] = { 1.0f, 1.0f, 0.3f };
        Vector3f target;
        for (uint8_t i = 0; i < 3; i++) {
            target[i] = (((k + 13 * i) / 40) % 2 == 0) ? amplitude[i] : -amplitude[i];
        }
        indi.update(make_inputs(gyro, target));
        gyro = plant.step(indi.get_output());
        if (k > steps / 2) {
            // past the half-way point the loop should be tracking: in the
            // last quarter of each step (after 75 ms) the rate error is
            // a small fraction of the step
            for (uint8_t i = 0; i < 3; i++) {
                if (((k + 13 * i) % 40) >= 30) {
                    max_err_late = MAX(max_err_late, fabsf(gyro[i] - target[i]) / amplitude[i]);
                }
            }
        }
    }
    for (uint8_t i = 0; i < 3; i++) {
        EXPECT_GE(uint8_t(indi.ident().get_state(i)), uint8_t(AP_INDI_Ident::AxisState::USABLE)) << "axis " << int(i);
        // within 20% of the plant: the plant integrates one step after
        // the command, a sampling delay the 10 ms model does not carry
        EXPECT_NEAR(indi.ident().get_effectiveness(i, i), g_true[i], 0.20f * g_true[i]) << "axis " << int(i);
    }
    EXPECT_EQ(indi.get_fallback_mask(), 0);
    // within 30% of the step after 75 ms of a 50 ms time constant loop
    // with 20 ms of actuator and sensor delay
    EXPECT_LT(max_err_late, 0.3f);
}

// fly the square-wave rate targets of IdentifiesAndTracks against a plant
// whose actuators lag by plant_tau while the controller models 10 ms, and
// return the worst late-step tracking error as a fraction of the step.
// With offer_measured, the plant's actuator state times measured_scale is
// offered as Inputs::u_measured every step
static float fly_square_wave(AP_INDI &indi, float plant_tau, bool offer_measured, float measured_scale)
{
    indi.init(DT, default_params());
    indi.reset(Vector3f());
    Plant plant(DT, Vector3f(600, 600, 60), plant_tau, 4);
    Vector3f gyro;
    float max_err_late = 0;
    const uint32_t steps = 800;  // 2 s
    const float amplitude[3] = { 1.0f, 1.0f, 0.3f };
    for (uint32_t k = 0; k < steps; k++) {
        Vector3f target;
        for (uint8_t i = 0; i < 3; i++) {
            target[i] = (((k + 13 * i) / 40) % 2 == 0) ? amplitude[i] : -amplitude[i];
        }
        AP_INDI::Inputs in = make_inputs(gyro, target);
        if (offer_measured) {
            in.u_measured = plant.actuator() * measured_scale;
            in.u_measured_valid = true;
        }
        indi.update(in);
        gyro = plant.step(indi.get_output());
        if (k > steps / 2) {
            for (uint8_t i = 0; i < 3; i++) {
                if (((k + 13 * i) % 40) >= 30) {
                    max_err_late = MAX(max_err_late, fabsf(gyro[i] - target[i]) / amplitude[i]);
                }
            }
        }
    }
    return max_err_late;
}

// The actuators are twice as fast as the controller's model (5 ms against
// 10 ms), as on a vehicle whose ACT_TAU was set conservatively. With the
// actuator state measured, as motor-speed feedback gives it on a
// multirotor, the law builds on what the actuators actually did instead of
// on the model: the measured source is used throughout, every axis is
// still identified, nothing falls back, and the rate tracks better than
// with the model alone (0.33 against 0.50 of the step when written).
// Better tracking with the measurement is the good outcome. It is not
// better in every case: an actuator two to three times slower than its
// model tracked worse measured than modelled, because a model that runs
// ahead of the actuator adds damping; see SPEC.md, actuator model.
TEST(INDIController, UsesMeasuredActuatorState)
{
    AP_INDI indi;
    const float err_measured = fly_square_wave(indi, 0.005f, true, 1.0f);
    EXPECT_EQ(indi.get_actuator_source(), AP_INDI::ActuatorSource::MEASURED);
    EXPECT_EQ(indi.get_fallback_mask(), 0);
    for (uint8_t i = 0; i < 3; i++) {
        EXPECT_GE(uint8_t(indi.ident().get_state(i)), uint8_t(AP_INDI_Ident::AxisState::USABLE)) << "axis " << int(i);
    }
    AP_INDI indi_model;
    const float err_model = fly_square_wave(indi_model, 0.005f, false, 1.0f);
    EXPECT_EQ(indi_model.get_actuator_source(), AP_INDI::ActuatorSource::MODEL);
    EXPECT_LT(err_measured, err_model);
}

// Measurements that are not a lagged copy of the command must not be
// flown on: one with every axis reversed (a motor order or sign error in
// the RPM-to-command conversion) and one stuck at zero (telemetry that
// reports but does not update). Each disagrees with the model by more
// than its own amplitude, so it is rejected for the flight and the model
// carries the loop: the axis does not fall back and the rate still tracks
// within the IdentifiesAndTracks bound. Rejection is the good outcome.
TEST(INDIController, RejectsInconsistentMeasurement)
{
    // reversed
    AP_INDI indi_rev;
    const float err_rev = fly_square_wave(indi_rev, 0.010f, true, -1.0f);
    EXPECT_EQ(indi_rev.get_actuator_source(), AP_INDI::ActuatorSource::REJECTED);
    EXPECT_EQ(indi_rev.get_fallback_mask(), 0);
    EXPECT_LT(err_rev, 0.3f);
    // stuck at zero
    AP_INDI indi_stuck;
    const float err_stuck = fly_square_wave(indi_stuck, 0.010f, true, 0.0f);
    EXPECT_EQ(indi_stuck.get_actuator_source(), AP_INDI::ActuatorSource::REJECTED);
    EXPECT_EQ(indi_stuck.get_fallback_mask(), 0);
    EXPECT_LT(err_stuck, 0.3f);
}

// A reversed roll axis: the plant responds the opposite way to the
// prior. The controller must detect it, mirror the prior and then
// fly the axis correctly rather than fall back, and must not touch
// the other axes.
TEST(INDIController, RecoversReversedAxis)
{
    AP_INDI indi;
    indi.init(DT, default_params());
    indi.reset(Vector3f());
    const Vector3f g_true(-400, 400, 40);
    Plant plant(DT, g_true, 0.010f, 4);

    Vector3f gyro;
    bool flipped = false;
    for (uint32_t k = 0; k < 800; k++) {
        Vector3f target;
        for (uint8_t i = 0; i < 3; i++) {
            target[i] = (((k + 13 * i) / 40) % 2 == 0) ? 1.0f : -1.0f;
        }
        indi.update(make_inputs(gyro, target));
        gyro = plant.step(indi.get_output());
        if (indi.ident().get_prior(0, 0) < 0) {
            flipped = true;
        }
        // the roll rate must never run away before the flip is caught:
        // a wrong-sign loop doubles its error every few steps, so a
        // detection inside 50 ms keeps it below a few rad/s
        ASSERT_LT(fabsf(gyro.x), 6.0f) << "roll diverged at step " << k;
    }
    EXPECT_TRUE(flipped);
    EXPECT_EQ(indi.get_fallback_mask(), 0);
    // within 20% of the plant, see IdentifiesAndTracks
    EXPECT_NEAR(indi.ident().get_effectiveness(0, 0), -400.0f, 80.0f);
    EXPECT_GE(uint8_t(indi.ident().get_state(0)), uint8_t(AP_INDI_Ident::AxisState::USABLE));
}

// An unhealthy gyro freezes the loop and, held long enough, falls
// every axis back: nothing the controller measures can be trusted.
TEST(INDIController, GyroUnhealthyFallsBack)
{
    AP_INDI indi;
    indi.init(DT, default_params());
    indi.reset(Vector3f());
    Plant plant(DT, Vector3f(400, 400, 40), 0.010f, 4);
    Vector3f gyro;
    const uint32_t t0 = AP_HAL::millis();
    // keep stepping with the gyro flagged bad until half a second of
    // wall clock has passed; the fallback timer runs on millis()
    while (AP_HAL::millis() - t0 < 700) {
        AP_INDI::Inputs in = make_inputs(gyro, Vector3f());
        in.gyro_healthy = false;
        indi.update(in);
        gyro = plant.step(indi.get_output());
        hal.scheduler->delay(10);
    }
    EXPECT_EQ(indi.get_fallback_mask(), 0x7);
}

#endif // AP_INDI_ENABLED

AP_GTEST_MAIN()
