#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/vector2.h>
#include <AP_Math/vector3.h>
#include <AP_Math/SCurve.h>

TEST(LinesScurve, test_calculate_path)
{
    float Jm_out, t2_out, t4_out, t6_out;
    SCurve::calculate_path(0.300000012, 19.4233513, 0, 5.82700586, 188.354691, 2.09772229,
                           Jm_out, t2_out, t4_out, t6_out);
    EXPECT_FLOAT_EQ(Jm_out, 19.423351);
    EXPECT_FLOAT_EQ(t2_out, 0.0);
    EXPECT_FLOAT_EQ(t4_out, 0.0);
    EXPECT_FLOAT_EQ(t6_out, 0.0);
}


AP_GTEST_MAIN()
int hal = 0; //weirdly the build will fail without this
