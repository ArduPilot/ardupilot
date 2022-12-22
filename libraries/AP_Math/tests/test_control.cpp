#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/vector2.h>
#include <AP_Math/vector3.h>
#include <AP_Math/control.h>

TEST(Control, test_control)
{
    postype_t pos_start = 17;
    float vel_start = 20;

    float vel = vel_start;
    postype_t pos = pos_start;
    const float dt = 0.01;
    const float accel = 1.0;

    update_pos_vel_accel(pos, vel, accel, dt, 0, 0, 0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel*dt);
    EXPECT_FLOAT_EQ(pos, pos_start + 0.5*(vel+vel_start)*dt);

    vel = vel_start;
    pos = pos_start;
    update_pos_vel_accel(pos, vel, accel, dt, 1.0, 1.0, 1.0);
    EXPECT_FLOAT_EQ(vel, vel_start);
    EXPECT_FLOAT_EQ(pos, pos_start);

    vel = vel_start;
    pos = pos_start;
    update_pos_vel_accel(pos, vel, accel, dt, -1.0, 1.0, 1.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel*dt);
    EXPECT_FLOAT_EQ(pos, pos_start + 0.5*(vel+vel_start)*dt);
}


AP_GTEST_MAIN()
int hal = 0;
