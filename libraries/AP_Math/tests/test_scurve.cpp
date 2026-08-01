#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/vector2.h>
#include <AP_Math/vector3.h>
#include <AP_Math/SCurve.h>

// Test inputs and expected outputs for each code path in calculate_path.
//
// With Sm=62.8319, Jm=10: tj ≈ 0.25, Jm*tj ≈ 2.5
// At = MIN(Am, (Vm-V0)/(2*tj), (L - 4*V0*tj)/(4*tj²))
//    ≈ MIN(Am, (Vm-V0)/0.5, (L-V0)/0.25)
//
// Paths exercised:
//   B: V0 >= Vm
//   C: At <= 0 (requires L < V0 approx, sign fix path)
//   D: At<Jm*tj, V0==0, solution=0
//   E: At<Jm*tj, V0==0, solution=2
//   F: At<Jm*tj, V0>0, solution=0
//   G: At<Jm*tj, V0>0, solution=2
//   H: At>=Jm*tj, solution=5
//   I: At>=Jm*tj, solution=7
struct PathTest {
    const char *name;
    float Sm, Jm, V0, Am, Vm, L;
    float exp_Jm, exp_tj, exp_t2, exp_t4, exp_t6;
};

static const PathTest path_tests[] = {

    // ---- Path B: V0 >= Vm ----
    {"B1_exact",  62.8319, 10, 10,   5,  10,  100,    0, 0, 0, 0, 0},
    {"B2_just",   62.8319, 10, 5.1,  5,  5,   100,    0, 0, 0, 0, 0},
    {"B3_mid",    62.8319, 10, 20,   5,  10,  100,    0, 0, 0, 0, 0},
    {"B4_low",    62.8319, 10, 3,    5,  2,   100,    0, 0, 0, 0, 0},

    // ---- Path C: At <= 0 (sign fix) ----
    {"C1_just",   62.8319, 10, 5,    5,  10,  4.9,    0, 0, 0, 0, 0},
    {"C2_deep",   62.8319, 10, 8,    5,  10,  5,      0, 0, 0, 0, 0},
    {"C3_v3",     62.8319, 10, 3,    5,  10,  2.9,    0, 0, 0, 0, 0},
    {"C4_hiV0",   62.8319, 10, 9,    5,  10,  8,      0, 0, 0, 0, 0},

    // ---- Path D: At<Jm*tj, V0==0, solution=0 ----
    {"D1_tiny",   62.8319, 10, 0,    5,  10,  0.01,   3.55656075f, 0.08891395f, 0, 0, 0},
    {"D2_short",  62.8319, 10, 0,    5,  10,  0.1,    6.32455873f, 0.15811385f, 0, 0, 0},
    {"D3_nearE",  62.8319, 10, 0,    5,  10,  0.2,    7.52121019f, 0.18803012f, 0, 0, 0},
    {"D4_lowAm",  62.8319, 10, 0,    1,  10,  0.005,  2.99069929f, 0.07476743f, 0, 0, 0},

    // ---- Path E: At<Jm*tj, V0==0, solution=2 ----
    {"E1_deep",   62.8319, 10, 0,    2,    10,  50,    8.94427586f, 0.22360672f, 0, 4.55278635f, 0},
    {"E2_nearD",  62.8319, 10, 0,    2,    10,  5,     8.94427586f, 0.22360672f, 0, 1.57640016f, 0},
    {"E3_nearI",  62.8319, 10, 0,    2.4,  10,  50,    9.79796314f, 0.24494889f, 0, 3.67676854f, 0},
    {"E4_lowAm",  62.8319, 10, 0,    1,    10,  50,    6.32455778f, 0.15811382f, 0, 9.52690887f, 0},

    // ---- Path F: At<Jm*tj, V0>0, solution=0 ----
    {"F1_nearC",  62.8319, 10, 5,    5,  10,  5.1,    1.60006297f, 0.24999982f, 0, 0, 0},
    {"F2_mid",    62.8319, 10, 1,    5,  10,  1.5,    8.00002861f, 0.24999982f, 0, 0, 0},
    {"F3_v2",     62.8319, 10, 2,    5,  10,  2.3,    4.80003214f, 0.24999982f, 0, 0, 0},
    {"F4_amBind", 62.8319, 10, 0.5,  2,  10,  0.8,    4.80001593f, 0.24999982f, 0, 0, 0},

    // ---- Path G: At<Jm*tj, V0>0, solution=2 ----
    {"G1_deep",   62.8319, 10, 1,    2,  10,  50,     8.00000572f, 0.24999982f, 0, 4.00000000f, 0},
    {"G2_mod",    62.8319, 10, 0.5,  2,  10,  10,     8.00000572f, 0.24999982f, 0, 2.16227818f, 0},
    {"G3_nearF",  62.8319, 10, 2,    2,  10,  10,     8.00000572f, 0.24999982f, 0, 1.50000060f, 0},
    {"G4_loV0",   62.8319, 10, 0.1,  2,  10,  50,     8.00000572f, 0.24999982f, 0, 4.44999981f, 0},

    // ---- Path H: At>=Jm*tj, solution=5 ----
    {"H1_nearI",  62.8319, 10, 0,    5,  3.7, 100,    10.0f, 0.24999982f, 0.24598742f, 0, 0.24598742f},
    {"H2_mid",    62.8319, 10, 0,    5,  3.5, 100,    10.0f, 0.24999982f, 0.22966957f, 0, 0.22966957f},
    {"H3_shortL", 62.8319, 10, 0,    5,  10,  1.5,    10.0f, 0.24999982f, 0.12905994f, 0, 0.12905994f},
    {"H4_v0p",    62.8319, 10, 2,    5,  5.5, 100,    10.0f, 0.24999982f, 0.22966957f, 0, 0.22966957f},

    // ---- Path I: At>=Jm*tj, solution=7 ----
    {"I1_nearH",  62.8319, 10, 0,    5,  3.8, 100,    10.0f, 0.24999982f, 0.25000018f, 0.01000018f, 0.25000018f},
    {"I2_deep",   62.8319, 10, 0,    5,  10,  100,    10.0f, 0.24999982f, 0.25000018f, 1.25000024f, 0.25000018f},
    {"I3_v0p",    62.8319, 10, 2,    5,  10,  100,    10.0f, 0.24999982f, 0.25000018f, 0.85000020f, 0.25000018f},
    {"I4_lowAm",  62.8319, 10, 0,    3,  5,   100,    10.0f, 0.24999982f, 0.05000019f, 1.11666679f, 0.05000019f},
};

// Segment state used to integrate through the profile
struct SegState {
    float A, V, P;
};

// Integrate an increasing-jerk segment (raised cosine from 0 to Jm)
static SegState seg_incr_jerk(SegState s, float tj, float Jm)
{
    if (tj <= 0) return s;
    const float Alpha = Jm * 0.5f;
    const float Beta = M_PI / tj;
    const float AT = Alpha * tj;
    const float VT = Alpha * (sq(tj) * 0.5f - 2.0f / sq(Beta));
    const float PT = Alpha * ((-1.0f / sq(Beta)) * tj + (1.0f / 6.0f) * powf(tj, 3.0f));
    return {s.A + AT,
            s.V + s.A * tj + VT,
            s.P + s.V * tj + 0.5f * s.A * sq(tj) + PT};
}

// Integrate a constant-jerk segment
static SegState seg_const_jerk(SegState s, float t, float J)
{
    if (t <= 0) return s;
    return {s.A + J * t,
            s.V + s.A * t + 0.5f * J * sq(t),
            s.P + s.V * t + 0.5f * s.A * sq(t) + (1.0f / 6.0f) * J * powf(t, 3.0f)};
}

// Integrate a decreasing-jerk segment (raised cosine from Jm to 0)
static SegState seg_decr_jerk(SegState s, float tj, float Jm)
{
    if (tj <= 0) return s;
    const float Alpha = Jm * 0.5f;
    const float Beta = M_PI / tj;
    const float AT = Alpha * tj;
    const float VT = Alpha * (sq(tj) * 0.5f - 2.0f / sq(Beta));
    const float PT = Alpha * ((-1.0f / sq(Beta)) * tj + (1.0f / 6.0f) * powf(tj, 3.0f));
    const float A2T = Jm * tj;
    const float V2T = Jm * sq(tj);
    const float P2T = Alpha * ((-1.0f / sq(Beta)) * 2.0f * tj + (4.0f / 3.0f) * powf(tj, 3.0f));
    return {(s.A - AT) + A2T,
            (s.V - VT) + (s.A - AT) * tj + V2T,
            (s.P - PT) + (s.V - VT) * tj + 0.5f * (s.A - AT) * sq(tj) + P2T};
}

// Integrate a 3-segment jerk block: incr, const, decr
static SegState seg_jerk_block(SegState s, float tj, float Jm, float Tcj, float &peak_A)
{
    s = seg_incr_jerk(s, tj, Jm);
    peak_A = MAX(peak_A, fabsf(s.A));
    s = seg_const_jerk(s, Tcj, Jm);
    peak_A = MAX(peak_A, fabsf(s.A));
    s = seg_decr_jerk(s, tj, Jm);
    peak_A = MAX(peak_A, fabsf(s.A));
    return s;
}

TEST(SCurveCalcPath, coverage_and_outputs)
{
    float Jm_out, tj_out, t2_out, t4_out, t6_out;

    for (const auto &t : path_tests) {
        SCurve::calculate_path(t.Sm, t.Jm, t.V0, t.Am, t.Vm, t.L,
                               Jm_out, tj_out, t2_out, t4_out, t6_out);

        EXPECT_FLOAT_EQ(Jm_out, t.exp_Jm) << "Jm mismatch: " << t.name;
        EXPECT_FLOAT_EQ(tj_out, t.exp_tj) << "tj mismatch: " << t.name;
        EXPECT_FLOAT_EQ(t2_out, t.exp_t2) << "t2 mismatch: " << t.name;
        EXPECT_FLOAT_EQ(t4_out, t.exp_t4) << "t4 mismatch: " << t.name;
        EXPECT_FLOAT_EQ(t6_out, t.exp_t6) << "t6 mismatch: " << t.name;
    }
}

// Verify that calculate_path outputs, when applied through add_segments logic,
// produce a full path that:
// - total distance == 2*L (add_segments calls calculate_path with L*0.5)
// - peak velocity <= Vm
// - peak acceleration <= Am
// - output jerk <= input Jm
// - final velocity == V0 (returns to initial speed)
// - final acceleration == 0
//
// add_segments builds:
//   Accel half:  jerk_block(tj, +Jm, t2) + const(t4, 0) + jerk_block(tj, -Jm, t6)
//   Coast:       const(t_coast, 0) where t_coast fills remaining distance at Vm
//   Decel half:  jerk_block(tj, -Jm, t6) + const(t4, 0) + jerk_block(tj, +Jm, t2)
TEST(SCurveCalcPath, constraints)
{
    const float tol = 1.0e-3f;
    float Jm_out, tj_out, t2_out, t4_out, t6_out;

    for (const auto &t : path_tests) {
        SCurve::calculate_path(t.Sm, t.Jm, t.V0, t.Am, t.Vm, t.L,
                               Jm_out, tj_out, t2_out, t4_out, t6_out);

        // skip zero-output cases (paths B, C)
        if (is_zero(Jm_out) && is_zero(tj_out)) {
            continue;
        }

        // jerk limit: output Jm must not exceed input Jm
        EXPECT_LE(Jm_out, t.Jm + tol) << "Jm exceeded: " << t.name;

        // --- Accel half ---
        float peak_A = 0.0f;
        SegState s = {0.0f, t.V0, 0.0f};

        // accel up: jerk_block(tj, +Jm, t2)
        s = seg_jerk_block(s, tj_out, Jm_out, t2_out, peak_A);
        float peak_V = s.V;

        // coast within accel half: const(t4, 0)
        s = seg_const_jerk(s, t4_out, 0.0f);
        peak_V = MAX(peak_V, s.V);

        // accel down: jerk_block(tj, -Jm, t6)
        s = seg_jerk_block(s, tj_out, -Jm_out, t6_out, peak_A);

        // end of accel half: acceleration should be ~0
        EXPECT_NEAR(s.A, 0.0f, tol) << "accel half final A non-zero: " << t.name;

        const float accel_half_P = s.P;
        const float cruise_V = s.V;

        // --- Coast segment (fill remaining distance at cruise velocity) ---
        const float L_total = 2.0f * t.L;
        const float coast_dist = MAX(0.0f, L_total - 2.0f * accel_half_P);
        float t_coast = 0.0f;
        if (cruise_V > 0.0f) {
            t_coast = coast_dist / cruise_V;
        }
        s = seg_const_jerk(s, t_coast, 0.0f);
        peak_V = MAX(peak_V, s.V);

        // --- Decel half (mirror of accel) ---
        // decel down: jerk_block(tj, -Jm, t6)
        s = seg_jerk_block(s, tj_out, -Jm_out, t6_out, peak_A);

        // coast within decel half: const(t4, 0)
        s = seg_const_jerk(s, t4_out, 0.0f);

        // decel up: jerk_block(tj, +Jm, t2)
        s = seg_jerk_block(s, tj_out, Jm_out, t2_out, peak_A);

        // --- Check constraints ---

        // total distance must match 2*L
        EXPECT_NEAR(s.P, L_total, tol) << "distance mismatch: " << t.name
            << " P=" << s.P << " expected=" << L_total;

        // final velocity must return to V0
        EXPECT_NEAR(s.V, t.V0, tol) << "final velocity mismatch: " << t.name
            << " V=" << s.V << " V0=" << t.V0;

        // final acceleration must be zero
        EXPECT_NEAR(s.A, 0.0f, tol) << "final accel non-zero: " << t.name;

        // peak velocity must not exceed Vm
        EXPECT_LE(peak_V, t.Vm + tol) << "velocity exceeded Vm: " << t.name
            << " peak_V=" << peak_V << " Vm=" << t.Vm;

        // peak acceleration must not exceed Am
        EXPECT_LE(peak_A, t.Am + tol) << "accel exceeded Am: " << t.name
            << " peak_A=" << peak_A << " Am=" << t.Am;
    }
}

// ---------------------------------------------------------------------------
// calculate_track: arc speed limit must come from the arc length, not the chord
// ---------------------------------------------------------------------------

// Runaway guard for every drive loop below. All of them complete in well under this: the
// straight and arc legs in roughly 8k iterations, the longest circle (3.5 turns) in roughly
// 20k, so it only bounds a regression that never reports completion. 50000 iterations is
// 125 s of simulated time at 400 Hz.
static const uint32_t MAX_DRIVE_ITERS = 50000;

// drive a prepared leg to completion, returning the peak horizontal and vertical
// target speed and the final target position
struct LegPeak { float speed_xy; float speed_z; Vector3p final_pos; bool finished; };
static LegPeak drive_leg(SCurve &leg, const Vector3p &origin)
{
    SCurve prev, next;
    prev.init();
    next.init();
    const float dt = 0.0025f;
    LegPeak r {};
    Vector3p pos;
    Vector3f vel, accel;
    bool done = false;
    for (uint32_t i = 0; i < MAX_DRIVE_ITERS && !done; i++) {
        pos = origin;
        vel.zero();
        accel.zero();
        done = leg.advance_target_along_track(prev, next, 2.0f, 2.0f, false, dt, pos, vel, accel);
        r.speed_xy = MAX(r.speed_xy, vel.xy().length());
        r.speed_z = MAX(r.speed_z, fabsf(vel.z));
    }
    r.final_pos = pos;
    r.finished = done;
    return r;
}

// A level straight leg reaches the full horizontal speed, has no vertical motion, and
// finishes at the destination. Covers the set_straight_geometry / generate_path path
// for a non-arc segment.
TEST(SCurveTrack, straight_leg)
{
    const Vector3p origin{0, 0, -50};
    const Vector3p dest{50, 0, -50};    // 50 m north, level
    const float speed_xy = 5.0f;

    SCurve leg;
    leg.calculate_track(origin, dest, 0.0f,   // arc angle 0 -> straight segment
                        speed_xy, 3.0f, 2.0f,
                        2.0f, 2.0f, 2.0f,
                        60.0f, 10.0f);

    const LegPeak p = drive_leg(leg, origin);
    EXPECT_TRUE(p.finished);
    EXPECT_NEAR(p.speed_xy, speed_xy, 0.05f);   // reaches the horizontal speed
    EXPECT_LE(p.speed_z, 0.02f);                // stays level
    EXPECT_NEAR((float)p.final_pos.x, 50.0f, 0.05f);
    EXPECT_NEAR((float)p.final_pos.y, 0.0f, 0.05f);
    EXPECT_NEAR((float)p.final_pos.z, -50.0f, 0.05f);
}

// A climbing arc must take its speed limit from the path it actually flies (the arc
// length), not the shorter chord. This 180-degree arc of radius 20 m spans ~62.8 m
// horizontally but only 40 m of chord; combined with a 10 m altitude change and a
// tight 1 m/s vertical limit, the chord basis would wrongly throttle the leg to
// ~4.1 m/s. Using the arc length lets it reach the full 5 m/s horizontal speed while
// the vertical rate stays within its limit.
TEST(SCurveTrack, climbing_arc_limit_from_arc_length)
{
    const Vector3p origin{20, 0, -50};
    const Vector3p dest{-20, 0, -40};   // 40 m chord, 10 m altitude change
    const float speed_xy = 5.0f, speed_up = 1.0f, speed_down = 1.0f;

    SCurve leg;
    leg.calculate_track(origin, dest, M_PI,
                        speed_xy, speed_up, speed_down,
                        2.0f, 2.0f, 2.0f,   // accel xy, z, corner
                        60.0f, 10.0f);      // snap, jerk

    const LegPeak p = drive_leg(leg, origin);
    EXPECT_TRUE(p.finished);

    // reaches the full horizontal budget (the chord basis would cap it near 4.1 m/s)
    EXPECT_NEAR(p.speed_xy, speed_xy, 0.15f);
    // vertical rate stays within its limit
    EXPECT_LE(p.speed_z, speed_down + 0.02f);
}

// The projected velocity must equal the derivative of the projected position, including for a
// climbing arc. Regression: the arc velocity/acceleration previously used a unit horizontal tangent
// where the horizontal fraction arc.length_ne/seg_length was required, tilting a climbing arc's
// reported velocity too far toward horizontal (and inflating the centripetal term).
TEST(SCurveTrack, arc_velocity_matches_position_derivative)
{
    const Vector3p origin{20, 0, -50};
    const Vector3p dest{0, 20, -20};   // 90-degree arc, radius 20 (~31.4 m arc), climbing 30 m
    SCurve leg;
    leg.calculate_track(origin, dest, M_PI_2,
                        10.0f, 5.0f, 5.0f, 3.0f, 3.0f, 3.0f, 60.0f, 10.0f);

    SCurve prev, next;
    prev.init();
    next.init();
    const float dt = 0.0025f;
    Vector3p pos, pos_prev;
    Vector3f vel, accel;
    bool have_prev = false;
    float max_err = 0.0f;
    for (uint32_t i = 0; i < MAX_DRIVE_ITERS; i++) {
        pos = origin;
        vel.zero();
        accel.zero();
        const bool done = leg.advance_target_along_track(prev, next, 2.0f, 2.0f, false, dt, pos, vel, accel);
        if (have_prev && vel.length() > 3.0f) {
            const Vector3f fd = (pos - pos_prev).tofloat() / dt;   // derivative of the reported position
            max_err = MAX(max_err, (fd - vel).length());
        }
        pos_prev = pos;
        have_prev = true;
        if (done) {
            break;
        }
    }
    // finite difference matches the reported velocity to O(dt); a large gap means the reported
    // velocity is not tangent to the path actually flown
    EXPECT_LT(max_err, 0.05f);
}

// ---------------------------------------------------------------------------
// calculate_circle_track: full-circle / multi-turn arc geometry
// ---------------------------------------------------------------------------

// result of driving one circle leg to completion, sampling the target path
struct CircleResult {
    float max_radius_err;   // largest deviation of the target from radius R
    float peak_speed;       // largest target speed magnitude
    Vector3p final_pos;     // target position at completion
    float first_east;       // east position ~1 s after start (direction check)
    bool finished;          // leg reported completion
};

// build a circle leg and advance it to completion, exactly as AC_WPNav drives a leg
static CircleResult run_circle(const Vector3p &origin, const Vector2f &center,
                               float total_angle_rad, float climb_d_m,
                               float speed_xy, float accel_c)
{
    SCurve prev, leg, next;
    prev.init();
    next.init();
    leg.calculate_circle_track(origin, center, total_angle_rad, climb_d_m,
                               speed_xy, 3.0f, 2.0f,   // speed xy, up, down
                               2.0f, 2.0f, accel_c,    // accel xy, z, corner
                               60.0f, 10.0f);          // snap, jerk

    const float R = (center - origin.xy().tofloat()).length();
    const float dt = 0.0025f;

    CircleResult r {};
    Vector3p pos;
    Vector3f vel, accel;
    bool done = false;
    for (uint32_t i = 0; i < MAX_DRIVE_ITERS && !done; i++) {
        // target_pos must be reset to the leg origin before each advance (see AC_WPNav)
        pos = origin;
        vel.zero();
        accel.zero();
        done = leg.advance_target_along_track(prev, next, 2.0f, accel_c, false, dt, pos, vel, accel);
        r.max_radius_err = MAX(r.max_radius_err, fabsf((pos.xy().tofloat() - center).length() - R));
        r.peak_speed = MAX(r.peak_speed, vel.length());
        if (i == 400) {
            r.first_east = pos.y;
        }
    }
    r.final_pos = pos;
    r.finished = done;
    return r;
}

TEST(SCurveCircle, geometry)
{
    const Vector3p origin{10, 0, -50};   // 10 m north of center, 50 m up
    const Vector2f center{0, 0};
    const float R = 10.0f;
    const float accel_c = 2.0f;
    // horizontal speed is clamped so centripetal accel v^2/R stays within accel_c
    const float vmax_expect = MIN(5.0f, safe_sqrt(accel_c * R));

    struct Case { const char *name; float turns; float climb; };
    const Case cases[] = {
        {"full_cw",    1.0f,   0.0f},
        {"half_cw",    0.5f,   0.0f},
        {"multi_cw",   3.5f,   0.0f},
        {"climb_cw",   2.0f,  -5.0f},
        {"full_ccw",  -1.0f,   0.0f},
        {"multi_ccw", -2.5f,   0.0f},
    };

    for (const auto &c : cases) {
        const float angle = c.turns * M_2PI;
        const CircleResult r = run_circle(origin, center, angle, c.climb, 5.0f, accel_c);

        EXPECT_TRUE(r.finished) << c.name;

        // target never leaves the circle
        EXPECT_LT(r.max_radius_err, 0.05f) << "radius drift: " << c.name
            << " err=" << r.max_radius_err;

        // never exceeds the clamped speed (verifies the centripetal limit)
        EXPECT_LE(r.peak_speed, vmax_expect + 0.05f) << "overspeed: " << c.name
            << " peak=" << r.peak_speed;

        // reaches the commanded net climb
        EXPECT_NEAR((float)r.final_pos.z, (float)origin.z + c.climb, 0.10f) << "climb: " << c.name;

        // NE endpoint matches the swept angle applied to the entry offset
        Vector2f rel = origin.xy().tofloat() - center;
        rel.rotate(angle);
        const Vector2f end_expect = center + rel;
        EXPECT_NEAR((float)r.final_pos.x, end_expect.x, 0.15f) << "end north: " << c.name;
        EXPECT_NEAR((float)r.final_pos.y, end_expect.y, 0.15f) << "end east: " << c.name;

        // direction: positive angle sweeps toward +east, negative toward -east
        if (angle > 0) {
            EXPECT_GT(r.first_east, 0.0f) << "expected CW: " << c.name;
        } else {
            EXPECT_LT(r.first_east, 0.0f) << "expected CCW: " << c.name;
        }
    }
}

// Regression test: AC_WPNav's corner-blending mechanism used to corrupt the position target of
// the leg immediately following a fractional-turn circle. This replicates exactly what
// AC_WPNav does when a plain WP leg follows a circle leg: the finished circle is preserved as
// the new leg's _scurve_prev_leg for corner blending (see AC_WPNav::set_wp_destination_NED_m),
// and advance_target_along_track() unconditionally calls prev_leg.move_to_pos_vel_accel() every
// tick, which subtracts get_origin_to_destination() to cancel a finished leg's contribution
// to zero.
TEST(SCurveCircle, prev_leg_handoff)
{
    const Vector3p O{10, 0, -50};
    const Vector2f center{0, 0};
    const float turns = 0.5f;          // fractional turns -> destination != origin

    SCurve circle;
    circle.calculate_circle_track(O, center, turns * M_2PI, 0.0f,
                                  5.0f, 3.0f, 2.0f,
                                  2.0f, 2.0f, 2.0f,
                                  60.0f, 10.0f);

    // drive the circle leg to completion (mirrors run_circle() above)
    {
        SCurve prev, next;
        prev.init();
        next.init();
        Vector3p pos;
        Vector3f vel, accel;
        bool done = false;
        for (uint32_t i = 0; i < MAX_DRIVE_ITERS && !done; i++) {
            pos = O;
            vel.zero();
            accel.zero();
            done = circle.advance_target_along_track(prev, next, 2.0f, 2.0f, false, 0.0025f, pos, vel, accel);
        }
        ASSERT_TRUE(done);
    }

    // the orbit destination (AC_WPNav derives this from the leg's get_origin_to_destination();
    // computed independently here from the endpoint rotation so the test does not trust
    // seg_delta)
    Vector2f origin_to_end_ne = O.xy().tofloat() - center;
    origin_to_end_ne.rotate(turns * M_2PI);
    const Vector3p D(center.x + origin_to_end_ne.x, center.y + origin_to_end_ne.y, O.z);

    // new leg: origin = D (AC_WPNav sets _origin_ned_m = _destination_ned_m), a straight hop north
    SCurve new_leg;
    new_leg.calculate_track(D, D + Vector3p(20, 0, 0), 0.0f,
                            5.0f, 3.0f, 2.0f,
                            2.0f, 2.0f, 2.0f,
                            60.0f, 10.0f);

    // AC_WPNav preserves the finished circle as _scurve_prev_leg for the new leg
    SCurve scurve_next_leg;
    scurve_next_leg.init();

    Vector3p target_pos = D;
    Vector3f target_vel, target_accel;
    // one tick into a 20 m leg definitely isn't finished; this also exercises the return value
    EXPECT_FALSE(new_leg.advance_target_along_track(circle, scurve_next_leg, 2.0f, 2.0f, false, 0.0025f, target_pos, target_vel, target_accel));

    // the first tick of the new leg must start at (essentially) its own origin D
    EXPECT_NEAR((float)target_pos.x, (float)D.x, 0.05f);
    EXPECT_NEAR((float)target_pos.y, (float)D.y, 0.05f);
    EXPECT_NEAR((float)target_pos.z, (float)D.z, 0.05f);
}

// A circular orbit's speed comes from the commanded turn rate, not from the waypoint speed, so a
// later speed update must be able to slow the orbit but never raise it. Regression: set_speed_max()
// re-derived the leg speed purely from the limits handed to it, which discarded the orbit speed and
// let an unrelated WPNAV_SPEED/WPNAV_SPEED_UP/WPNAV_SPEED_DN change accelerate the circle.
TEST(SCurveCircle, speed_update_cannot_raise_orbit_speed)
{
    const Vector3p origin{10, 0, -50};
    const Vector2f center{0, 0};
    const float orbit_speed = 1.5f;     // well below both the waypoint speed and the centripetal cap
    const float accel_c = 2.0f;         // centripetal cap at radius 10 is sqrt(2 * 10) = 4.47 m/s

    // a speed increase must not be applied to the orbit
    {
        SCurve leg;
        leg.calculate_circle_track(origin, center, M_2PI, 0.0f,
                                   orbit_speed, 3.0f, 2.0f,
                                   2.0f, 2.0f, accel_c,
                                   60.0f, 10.0f);
        leg.set_speed_max(5.0f, 3.0f, 2.0f);
        const LegPeak p = drive_leg(leg, origin);
        EXPECT_TRUE(p.finished);
        EXPECT_NEAR(p.speed_xy, orbit_speed, 0.05f);
    }

    // a speed reduction must still be applied
    {
        SCurve leg;
        leg.calculate_circle_track(origin, center, M_2PI, 0.0f,
                                   orbit_speed, 3.0f, 2.0f,
                                   2.0f, 2.0f, accel_c,
                                   60.0f, 10.0f);
        leg.set_speed_max(1.0f, 3.0f, 2.0f);
        const LegPeak p = drive_leg(leg, origin);
        EXPECT_TRUE(p.finished);
        EXPECT_NEAR(p.speed_xy, 1.0f, 0.05f);
    }

    // contrast: a straight leg carries no intrinsic speed limit, so a speed increase still applies
    {
        SCurve leg;
        leg.calculate_track(origin, origin + Vector3p(50, 0, 0), 0.0f,
                            orbit_speed, 3.0f, 2.0f,
                            2.0f, 2.0f, 2.0f,
                            60.0f, 10.0f);
        leg.set_speed_max(3.0f, 3.0f, 2.0f);
        const LegPeak p = drive_leg(leg, origin);
        EXPECT_TRUE(p.finished);
        EXPECT_NEAR(p.speed_xy, 3.0f, 0.05f);
    }
}

AP_GTEST_MAIN()
int hal = 0; //weirdly the build will fail without this
