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

AP_GTEST_MAIN()
int hal = 0; //weirdly the build will fail without this
