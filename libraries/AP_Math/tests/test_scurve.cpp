#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/vector2.h>
#include <AP_Math/vector3.h>
#include <AP_Math/SCurve.h>

TEST(LinesScurve, test_calculate_path)
{
    // this test doesn't do much...
    float Jm_out, tj_out, t2_out, t4_out, t6_out;
    SCurve::calculate_path(62.8319, 10, 0, 5, 10, 100,
                           Jm_out, tj_out, t2_out, t4_out, t6_out);
    EXPECT_FLOAT_EQ(Jm_out, 10);
    EXPECT_FLOAT_EQ(t2_out, 0.25000018);
    EXPECT_FLOAT_EQ(t4_out, 1.2500002);
    EXPECT_FLOAT_EQ(t6_out, 0.25000018);
}


AP_GTEST_MAIN()
int hal = 0; //weirdly the build will fail without this
