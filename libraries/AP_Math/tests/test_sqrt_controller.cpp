#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>  // FIXME: control.h has include issues...

#include <AP_Math/control.h>

#define EXPECT_VECTOR2F_EQ(v1, v2)              \
    do {                                        \
        EXPECT_FLOAT_EQ(v1[0], v2[0]);          \
        EXPECT_FLOAT_EQ(v1[1], v2[1]);          \
    } while (false);

TEST(control, SqrtController2f)
{
    // a zero-length error gets a zero-length response:
    Vector2f zero;
    EXPECT_VECTOR2F_EQ(zero, sqrt_controller(zero, 17, 18, 0.01));
}

TEST(control, InvSqrtControllerIsActuallyInverse)
{
    // test input - these are the parameters and expected response for
    // a call to sqrt_controller.
    const struct InvSqrtTest {
        float error;
        float p;
        float second_order_limit;
        float dt;
        float expected_sqrt_controller_output;
    } tests[] {
        { 5, 0.1, 1, 0.2, 0.5 },  // simple random number test case
        { -5, 0.1, 1, 0.2, -0.5 },  // negative error should have negavtive response
        { 5, 0.2, 1, 0.2, 1 },  // twice-the-P
        // { 5, 0.1, 1, 0.4, 1 },  // twice-the-dt
        { 0, 0.1, 1, 0.2, 0 },  // zero-error means zero response
        { 0, 0, 0, 0, 0 },  // slightly corner-case-ish....
    };

    // iterate through all tests, calling sqrt_controller, and
    // checking its output.  Take its output and some of the test
    // parameters and ensure that the inverse sqrt controller gives us
    // back the original error.
    for (auto &test : tests) {
        const float sqrt_controller_output = sqrt_controller(
            test.error,
            test.p,
            test.second_order_limit,
            test.dt
        );
        // ensure sqrt controller out put is what we expect
        EXPECT_FLOAT_EQ(sqrt_controller_output, test.expected_sqrt_controller_output);
        // ensure we can invert it back to the initial error
        EXPECT_FLOAT_EQ(test.error, inv_sqrt_controller(
                            sqrt_controller_output,
                            test.p,
                            test.second_order_limit
                            )
        );
    }
}

int hal;

AP_GTEST_MAIN()
