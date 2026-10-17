#include <AP_gtest.h>

#include <AP_INDI/AP_INDI_config.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// the library is only built for boards that enable it (SITL by default);
// elsewhere this file compiles to an empty test binary
#if AP_INDI_ENABLED

#include <AP_INDI/AP_INDI_Ident.h>
#include <AP_Math/AP_Math.h>

static constexpr uint8_t N = 4;  // a quadrotor: four motors
static constexpr uint8_t MAXA = AP_INDI_Ident::MAX_ACTUATORS;

// prior roll effectiveness used throughout, rad/s^2 per unit command;
// the multirotor under 1 kg class figure from SPEC.md section 5
static constexpr float G_PRIOR = 150.0f;
// stroke amplitude used by the tests, normalised command; well above the
// du_min acceptance gate of 0.05 so every sample is a candidate
static constexpr float STROKE = 0.2f;

// deterministic noise source so the tests are repeatable. A linear
// congruential generator is used rather than rand() so the sequence is
// identical on every platform and compiler.
class LCG
{
public:
    // uniform in [0, 1)
    float uniform()
    {
        // multiplier 1664525 and increment 1013904223 are the Numerical
        // Recipes constants for a full-period 32-bit LCG (Knuth, TAOCP
        // vol 2, section 3.3.4); the arithmetic wraps modulo 2^32
        _s = _s * 1664525u + 1013904223u;
        // keep the top 24 bits, which are the best-distributed ones, and
        // scale by 1/2^24 so the result is exactly representable in a
        // float (24-bit mantissa) and never reaches 1.0
        return (_s >> 8) * (1.0f / 16777216.0f);
    }
    // uniform in [-1, 1)
    float symmetric()
    {
        return 2.0f * uniform() - 1.0f;
    }
private:
    uint32_t _s = 12345;  // arbitrary fixed seed, any non-zero value works
};

// quad-like effectiveness prior: roll from motors alternating sign,
// pitch from front pair against rear pair, yaw at a tenth of the roll
// effectiveness (reaction torque is much weaker than thrust torque)
static void make_prior(float prior[AP_INDI_Ident::NUM_AXES][MAXA], float roll_gain)
{
    for (uint8_t i = 0; i < AP_INDI_Ident::NUM_AXES; i++) {
        for (uint8_t j = 0; j < MAXA; j++) {
            prior[i][j] = 0;
        }
    }
    const float s[N] = { 1, -1, 1, -1 };
    for (uint8_t j = 0; j < N; j++) {
        prior[0][j] = roll_gain * s[j];
        prior[1][j] = roll_gain * (j < 2 ? 1 : -1);
        prior[2][j] = 0.1f * roll_gain * (j % 2 == 0 ? 1 : -1);
    }
}

// drive one axis with random strokes from a true effectiveness row and
// return how many samples were accepted
static uint32_t drive(AP_INDI_Ident &id, uint8_t axis, const float g_true[N], float amp, float noise,
                      uint32_t samples, LCG &rng)
{
    uint32_t accepted = 0;
    for (uint32_t k = 0; k < samples; k++) {
        float du[MAXA] = {};
        float dw = 0;
        for (uint8_t j = 0; j < N; j++) {
            du[j] = amp * rng.symmetric();
            dw += g_true[j] * du[j];
        }
        dw += noise * rng.symmetric();
        if (id.update(axis, du, dw, false)) {
            accepted++;
        }
    }
    return accepted;
}

// The identifier's main job: start from a class prior that is wrong by a
// typical margin (here the airframe is 1.33x more effective than the
// prior says) and converge to the true effectiveness, within the time
// budget of SPEC.md section 4 and to the accuracy the gain schedule
// needs. A low relative sigma is good here: it is what lets the launch
// over-drive relax to nominal bandwidth. Axes that receive no samples
// must be left untouched, since on a real vehicle the axes are excited at
// different times.
TEST(INDIIdent, ConvergesFromPriorError)
{
    float prior[AP_INDI_Ident::NUM_AXES][MAXA];
    make_prior(prior, G_PRIOR);
    // 0.7 is the multirotor prior relative sigma from SPEC.md section 5
    const float rel_sigma[3] = { 0.7f, 0.7f, 0.7f };
    AP_INDI_Ident::Config cfg;
    AP_INDI_Ident id;
    id.init(N, prior, rel_sigma, cfg);

    // true effectiveness is 1.33x the prior on roll
    const float g_true[N] = { 200, -200, 200, -200 };
    LCG rng;
    ASSERT_EQ(id.get_state(0), AP_INDI_Ident::AxisState::PRIOR);

    // noise of 5% of a full stroke response
    const float noise = 0.05f * 200 * STROKE;
    // 30 samples is 30 ms at the 1 kHz loop rate: well inside the 100 ms
    // "usable" target of SPEC.md section 4
    drive(id, 0, g_true, STROKE, noise, 30, rng);
    EXPECT_GE(static_cast<uint8_t>(id.get_state(0)), static_cast<uint8_t>(AP_INDI_Ident::AxisState::USABLE));

    // a further 200 ms must reach the converged state (300 ms target)
    drive(id, 0, g_true, STROKE, noise, 200, rng);
    EXPECT_EQ(id.get_state(0), AP_INDI_Ident::AxisState::CONVERGED);
    EXPECT_LT(id.get_rel_sigma(0), cfg.rel_sigma_converged);
    for (uint8_t j = 0; j < N; j++) {
        // within 5% of the true value, the same as the noise level
        EXPECT_NEAR(id.get_effectiveness(0, j), g_true[j], 0.05f * 200);
    }
    // the untouched axes keep their prior
    EXPECT_EQ(id.get_state(1), AP_INDI_Ident::AxisState::PRIOR);
    EXPECT_FLOAT_EQ(id.get_effectiveness(1, 0), prior[1][0]);
}

// Cross-coupling terms (an actuator the prior says has no effect on an
// axis) must be learnable, because real airframes are asymmetric and the
// INDI inversion is only as good as the full G row. A zero prior is the
// conservative starting point, so this checks that zero is a starting
// point and not a lock: the ratio-to-g_ref parameterisation for
// zero-prior actuators has to pull them to the true value.
TEST(INDIIdent, LearnsCouplingFromZeroPrior)
{
    float prior[AP_INDI_Ident::NUM_AXES][MAXA];
    make_prior(prior, G_PRIOR);
    // declare no yaw effect from motors 2 and 3
    prior[2][2] = 0;
    prior[2][3] = 0;
    const float rel_sigma[3] = { 0.7f, 0.7f, 0.7f };
    AP_INDI_Ident::Config cfg;
    AP_INDI_Ident id;
    id.init(N, prior, rel_sigma, cfg);

    // in truth all four motors contribute equally to yaw, at the prior's
    // yaw magnitude of a tenth of G_PRIOR
    const float g_true[N] = { 15, -15, 15, -15 };
    LCG rng;
    // noise under 1% of a stroke response, 300 samples: coupling terms
    // start from a zero prior so they converge more slowly than the
    // primary, see SPEC.md section 4 on cross-coupling
    drive(id, 2, g_true, STROKE, 0.02f, 300, rng);
    EXPECT_EQ(id.get_state(2), AP_INDI_Ident::AxisState::CONVERGED);
    // the learned coupling must be within 10% of the truth
    EXPECT_NEAR(id.get_effectiveness(2, 2), 15.0f, 1.5f);
    EXPECT_NEAR(id.get_effectiveness(2, 3), -15.0f, 1.5f);
}

// A cross-coupling term learned from a zero prior is bounded. The
// regressors move together in a coordinated manoeuvre and least squares
// then splits the response among them arbitrarily: in SITL a plane's
// pitch row credited the rudder with ten times the elevator's effect and
// a quad's yaw row credited roll with eight times the yaw's, and the
// sign test, reading the response through those couplings, failed both
// axes. The bound keeps such a term at coupling_max of the axis's
// reference even when the data say otherwise (here a coupling four times
// the reference, driven with independent strokes so the fit is exact),
// while the primary still converges and the axis is not failed.
TEST(INDIIdent, BoundsCouplingLearnedFromZeroPrior)
{
    float prior[AP_INDI_Ident::NUM_AXES][MAXA];
    make_prior(prior, G_PRIOR);
    // declare no yaw effect from motors 2 and 3
    prior[2][2] = 0;
    prior[2][3] = 0;
    const float rel_sigma[3] = { 0.7f, 0.7f, 0.7f };
    AP_INDI_Ident::Config cfg;
    // the controller's bound: half of the reference
    cfg.coupling_max = 0.5f;
    AP_INDI_Ident id;
    id.init(N, prior, rel_sigma, cfg);

    // motor 2 moves yaw four times as much as the reference motor's 15:
    // not an airframe, a fit gone wrong
    const float g_true[N] = { 15, -15, 60, 0 };
    LCG rng;
    drive(id, 2, g_true, STROKE, 0.02f, 300, rng);
    EXPECT_NE(id.get_state(2), AP_INDI_Ident::AxisState::FAILED);
    // the coupling is capped at half the reference, 7.5
    EXPECT_LE(fabsf(id.get_effectiveness(2, 2)), 0.5f * 15.0f * 1.001f);
    // the primary keeps the prior's sign and the fit's band
    EXPECT_GT(id.get_effectiveness(2, 0), 0.0f);
    EXPECT_LE(id.get_effectiveness(2, 0), cfg.band_hi * 15.0f * 1.001f);
}

// A reversed actuator (wrong servo direction, mirrored motor order) is the
// single most dangerous setup error, since a controller that trusts the
// prior sign drives the vehicle harder the wrong way. The sign test must
// fail the axis quickly, before the vehicle can diverge, and a failed axis
// must stop learning so a wrong estimate cannot creep back toward
// "acceptable". Fast failure is good here. After the caller mirrors the
// prior (the response in SPEC.md section 8) the axis must converge
// normally, which shows the restart really does reset the state.
TEST(INDIIdent, DetectsReversedActuator)
{
    float prior[AP_INDI_Ident::NUM_AXES][MAXA];
    make_prior(prior, G_PRIOR);
    const float rel_sigma[3] = { 0.7f, 0.7f, 0.7f };
    AP_INDI_Ident::Config cfg;
    AP_INDI_Ident id;
    id.init(N, prior, rel_sigma, cfg);

    // every motor acts opposite to the prior: a reversed roll axis
    const float g_true[N] = { -G_PRIOR, G_PRIOR, -G_PRIOR, G_PRIOR };
    LCG rng;
    // noise of about 2% of a stroke response; feed one sample at a time
    // so the number of samples needed to detect the reversal is known.
    // 100 is only a guard against an infinite loop.
    const float noise = 0.5f;
    uint32_t k = 0;
    while (k < 100 && id.get_state(0) != AP_INDI_Ident::AxisState::FAILED) {
        drive(id, 0, g_true, STROKE, noise, 1, rng);
        k++;
    }
    EXPECT_EQ(id.get_state(0), AP_INDI_Ident::AxisState::FAILED);
    // the reversal must be caught within 20 samples (20 ms at 1 kHz),
    // under the 30 ms sign-confirmation target of SPEC.md section 4
    EXPECT_LT(k, 20u);

    // a failed axis stops accepting even a clean, large sample
    float du[MAXA] = { STROKE, -STROKE, STROKE, -STROKE };
    EXPECT_FALSE(id.update(0, du, 100.0f, false));

    // restart with the sign mirrored and it converges
    id.restart_axis(0, true);
    EXPECT_EQ(id.get_state(0), AP_INDI_Ident::AxisState::PRIOR);
    drive(id, 0, g_true, STROKE, noise, 200, rng);
    EXPECT_EQ(id.get_state(0), AP_INDI_Ident::AxisState::CONVERGED);
    // within 5% of the true (negative) value
    EXPECT_NEAR(id.get_effectiveness(0, 0), -G_PRIOR, 0.05f * G_PRIOR);
}

// The acceptance gates of SPEC.md section 4. Small strokes carry too
// little signal against gyro noise and would pull the estimate toward
// noise; a saturated actuator has not produced the commanded increment,
// so its sample would bias G low. Both must be rejected without touching
// the state, while a clean full stroke is accepted. Rejecting more here is
// good: a skipped sample costs a millisecond, a bad one costs accuracy.
// Also checks that an out-of-range axis index is refused rather than
// indexing past the array.
TEST(INDIIdent, RejectsSmallAndSaturatedStrokes)
{
    float prior[AP_INDI_Ident::NUM_AXES][MAXA];
    make_prior(prior, G_PRIOR);
    const float rel_sigma[3] = { 0.7f, 0.7f, 0.7f };
    AP_INDI_Ident::Config cfg;
    AP_INDI_Ident id;
    id.init(N, prior, rel_sigma, cfg);

    // a stroke of 0.01 per motor has norm 0.02, below the du_min gate of
    // 0.05; the matching response is prior . du = 4 * 150 * 0.01
    float du_small[MAXA] = { 0.01f, -0.01f, 0.01f, -0.01f };
    EXPECT_FALSE(id.update(0, du_small, 6.0f, false));
    // a full stroke with the response the prior predicts, 4 * 150 * 0.2
    float du_ok[MAXA] = { STROKE, -STROKE, STROKE, -STROKE };
    const float dw_ok = 4 * G_PRIOR * STROKE;
    // rejected while an actuator is saturated
    EXPECT_FALSE(id.update(0, du_ok, dw_ok, true));
    EXPECT_EQ(id.get_accepted_count(0), 0u);
    EXPECT_TRUE(id.update(0, du_ok, dw_ok, false));
    EXPECT_EQ(id.get_accepted_count(0), 1u);
    // axis index out of range (only 3 axes) is rejected
    EXPECT_FALSE(id.update(5, du_ok, dw_ok, false));
}

// An airframe far outside its class band (10x the prior) cannot be
// identified safely: the inversion would command strokes sized for the
// wrong vehicle. The estimate must be clamped at the band edge rather than
// run away, and the unclamped shadow passing the edge by a clear margin
// must mark the axis failed so the caller drops to the fixed-gain
// fallback (SPEC.md section 8). A clamped, failed axis is the good
// outcome here; a "converged" estimate at 10x would be the dangerous one.
TEST(INDIIdent, ClampsOutOfBandAndFails)
{
    float prior[AP_INDI_Ident::NUM_AXES][MAXA];
    make_prior(prior, G_PRIOR);
    const float rel_sigma[3] = { 0.7f, 0.7f, 0.7f };
    AP_INDI_Ident::Config cfg;
    AP_INDI_Ident id;
    id.init(N, prior, rel_sigma, cfg);

    // 10x the prior is outside the 0.25x..4x band
    const float g_true[N] = { 10 * G_PRIOR, -10 * G_PRIOR, 10 * G_PRIOR, -10 * G_PRIOR };
    LCG rng;
    // 50 samples is plenty: the band is left within the first few
    drive(id, 0, g_true, STROKE, 1.0f, 50, rng);
    EXPECT_GE(id.get_clamp_count(0), cfg.max_clamps);
    EXPECT_EQ(id.get_state(0), AP_INDI_Ident::AxisState::FAILED);
    // the reported estimate stays at the band edge (1.0 of float slack)
    EXPECT_LE(id.get_effectiveness(0, id.get_primary(0)), cfg.band_hi * G_PRIOR + 1.0f);
}

// Single-precision RLS with a forgetting factor close to one is known to
// lose covariance positive-definiteness or wind up over long runs. This
// drives the identifier for 20 s of cruise with noise at 10% of the stroke
// response and checks three things: the state stays converged (no
// divergence), the relative sigma respects its floor (no collapse to a
// falsely certain estimate, which would make the gain schedule
// over-confident), and the estimate itself stays accurate. Bounded sigma
// is the good outcome, in both directions.
TEST(INDIIdent, CovarianceStaysBoundedUnderLongNoisyRun)
{
    float prior[AP_INDI_Ident::NUM_AXES][MAXA];
    make_prior(prior, G_PRIOR);
    const float rel_sigma[3] = { 0.7f, 0.7f, 0.7f };
    AP_INDI_Ident::Config cfg;
    AP_INDI_Ident id;
    id.init(N, prior, rel_sigma, cfg);

    // the prior is exactly right
    const float g_true[N] = { G_PRIOR, -G_PRIOR, G_PRIOR, -G_PRIOR };
    LCG rng;
    // noise of 10% of a full stroke response, 200 samples to converge
    const float noise = 3.0f;
    drive(id, 0, g_true, STROKE, noise, 200, rng);
    id.set_cruise(true);
    // a long cruise segment at lambda close to one must not blow up or
    // collapse the covariance in single precision: 20000 samples is 20 s
    // at 1 kHz, ten times the 2000-sample cruise memory, with half-size
    // strokes so the noise is relatively larger
    drive(id, 0, g_true, 0.5f * STROKE, noise, 20000, rng);
    EXPECT_EQ(id.get_state(0), AP_INDI_Ident::AxisState::CONVERGED);
    // the covariance floor must hold (1% slack for float rounding)
    EXPECT_GE(id.get_rel_sigma(0), cfg.rel_sigma_floor * 0.99f);
    // within 5% of the true value
    EXPECT_NEAR(id.get_effectiveness(0, 0), G_PRIOR, 0.05f * G_PRIOR);
}

// The covariance is kept in UD factorised form (SPEC.md, Numerics, f32).
// That is the textbook RLS covariance recursion rearranged, not a
// different estimator: in exact arithmetic both give the same P. This
// replays a run in which the prior is exactly right and the data are
// noise-free, so the estimate never moves and the noise estimate stays on
// its floor, and compares the primary's relative sigma with the textbook
// recursion run in double precision on the same accepted samples.
// Agreement to 1% (the f32 against f64 criterion of the validation plan's
// stage 5) shows the factorised update, its forgetting and its use of the
// noise variance are right; a wrong gain or a misplaced forgetting factor
// is off by far more. The covariance bounds are set out of reach so the
// comparison sees the recursion alone.
TEST(INDIIdent, CovarianceMatchesDoublePrecisionReference)
{
    float prior[AP_INDI_Ident::NUM_AXES][MAXA];
    make_prior(prior, G_PRIOR);
    const float rel_sigma[3] = { 0.7f, 0.7f, 0.7f };
    AP_INDI_Ident::Config cfg;
    // start the noise estimate on its floor: with noise-free data the
    // residuals are zero and it stays there, a constant the reference can
    // use
    cfg.noise_rel_init = cfg.noise_rel_floor;
    // a relative sigma floor of 0.1%, below the 0.3% this run settles at,
    // so the floor never acts
    cfg.rel_sigma_floor = 0.001f;
    AP_INDI_Ident id;
    id.init(N, prior, rel_sigma, cfg);

    // the reference recursion, P = (P - P phi phi' P / (lambda r + phi' P phi)) / lambda,
    // from the prior variance on the diagonal
    double P[N][N] = {};
    for (uint8_t j = 0; j < N; j++) {
        P[j][j] = sq(double(rel_sigma[0]));
    }
    const double r = sq(double(cfg.noise_rel_floor));
    const double lambda = cfg.lambda_launch;

    const float g_true[N] = { G_PRIOR, -G_PRIOR, G_PRIOR, -G_PRIOR };
    LCG rng;
    // 1000 samples is five times the 200-sample memory of the launch
    // forgetting factor, so the recursion has reached its steady state
    for (uint32_t k = 0; k < 1000; k++) {
        float du[MAXA] = {};
        float dw = 0;
        for (uint8_t j = 0; j < N; j++) {
            du[j] = STROKE * rng.symmetric();
            dw += g_true[j] * du[j];
        }
        if (!id.update(0, du, dw, false)) {
            continue;
        }
        // the identifier's regressor scaling, du_j prior_j / g_ref
        double phi[N];
        for (uint8_t j = 0; j < N; j++) {
            phi[j] = double(du[j]) * prior[0][j] / G_PRIOR;
        }
        double Pphi[N] = {};
        double denom = lambda * r;
        for (uint8_t j = 0; j < N; j++) {
            for (uint8_t m = 0; m < N; m++) {
                Pphi[j] += P[j][m] * phi[m];
            }
            denom += phi[j] * Pphi[j];
        }
        for (uint8_t j = 0; j < N; j++) {
            for (uint8_t m = 0; m < N; m++) {
                P[j][m] = (P[j][m] - Pphi[j] * Pphi[m] / denom) / lambda;
            }
        }
    }
    // the primary ratio stays at one, so its relative sigma is sqrt(P[0][0])
    const double expected = sqrt(P[0][0]);
    EXPECT_NEAR(id.get_rel_sigma(0), expected, 0.01 * expected);
}

// Why the covariance bounds keep the factorised form. In a coordinated
// manoeuvre the roll and pitch commands move together, so the data
// determine only their combined effect on the axis: the covariance
// shrinks along that one direction with every sample, while across it the
// forgetting factor grows it until the prior variance caps it. Capping
// the diagonal of P element by element, as this identifier did before the
// UD form, leaves the off-diagonal terms where they were, and P stops
// being positive definite (in double precision as much as in single); the
// innovation variance it computes then goes negative, every later sample
// is refused, and the estimate stays wherever it was when that happened.
// On this run that version accepted 10962 samples, about 13 s of them,
// refused every one after, and ended with the combined effect 11% high.
// This drives the identifier as the controller configures it (the three
// axis commands as actuators, a diagonal prior, couplings bounded at half
// the reference) through 20 s of cruise with the roll and pitch strokes
// equal, and checks that it keeps learning and that the combined effect
// is right.
TEST(INDIIdent, KeepsLearningUnderCoordinatedStrokes)
{
    // roll, pitch and yaw command effectiveness priors, rad/s^2 per unit:
    // the roll prior used throughout, the same for pitch and a tenth of it
    // for yaw as in make_prior(), and no coupling expected
    float prior[AP_INDI_Ident::NUM_AXES][MAXA] = {};
    prior[0][0] = G_PRIOR;
    prior[1][1] = G_PRIOR;
    prior[2][2] = 0.1f * G_PRIOR;
    const float rel_sigma[3] = { 0.7f, 0.7f, 0.7f };
    AP_INDI_Ident::Config cfg;
    // the controller's coupling bound, half of the reference
    cfg.coupling_max = 0.5f;
    AP_INDI_Ident id;
    // three actuators: the roll, pitch and yaw commands
    const uint8_t n = 3;
    id.init(n, prior, rel_sigma, cfg);

    // the roll axis is 1.2x as effective as its prior, a typical class
    // prior error, and the pitch command couples into roll at 20 rad/s^2
    // per unit, well inside the bound
    const float g_true[n] = { 1.2f * G_PRIOR, 20, 0 };
    // noise of 1% of the response to a full roll stroke
    const float noise = 0.01f * g_true[0] * STROKE;
    LCG rng;
    // the launch: 300 independent strokes, so the axis has converged
    // before the coordinated cruise starts
    for (uint16_t k = 0; k < 300; k++) {
        float du[MAXA] = {};
        float dw = noise * rng.symmetric();
        for (uint8_t j = 0; j < n; j++) {
            du[j] = STROKE * rng.symmetric();
            dw += g_true[j] * du[j];
        }
        id.update(0, du, dw, false);
    }
    id.set_cruise(true);
    const uint32_t launch_accepted = id.get_accepted_count(0);

    // 20000 samples is 20 s at 1 kHz, ten times the 2000-sample cruise
    // memory
    const uint32_t samples = 20000;
    for (uint32_t k = 0; k < samples; k++) {
        float du[MAXA] = {};
        du[0] = STROKE * rng.symmetric();
        du[1] = du[0];
        const float dw = g_true[0] * du[0] + g_true[1] * du[1] + noise * rng.symmetric();
        id.update(0, du, dw, false);
    }
    // the stroke gate passes 82% of these samples (|du| of the pair above
    // du_min, so a roll stroke above 0.035 of the 0.2 range) and the fit
    // explains every one of them; the version with P gone indefinite
    // accepted 55%
    EXPECT_GE(id.get_accepted_count(0) - launch_accepted, samples * 3 / 4);
    EXPECT_NE(id.get_state(0), AP_INDI_Ident::AxisState::FAILED);
    // the combined effect of a coordinated stroke, which is all the data
    // determine, within 1% of the truth (200 rad/s^2 per unit)
    const float combined = id.get_effectiveness(0, 0) + id.get_effectiveness(0, 1);
    EXPECT_NEAR(combined, g_true[0] + g_true[1], 0.01f * (g_true[0] + g_true[1]));
    // the coupling stays within its bound (0.1% of float slack)
    EXPECT_LE(fabsf(id.get_effectiveness(0, 1)), cfg.coupling_max * G_PRIOR * 1.001f);
    // finite, at or above its floor (1% slack for float rounding) and no
    // larger than the prior's
    const float rs = id.get_rel_sigma(0);
    EXPECT_TRUE(isfinite(rs));
    EXPECT_GE(rs, cfg.rel_sigma_floor * 0.99f);
    EXPECT_LE(rs, rel_sigma[0]);
}

#endif // AP_INDI_ENABLED

AP_GTEST_MAIN()
