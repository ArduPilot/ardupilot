#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/vector2.h>
#include <AP_Math/vector3.h>
#include <AP_Math/control.h>

#include <fenv.h>
#include <signal.h>
#include <setjmp.h>

TEST(Control, test_control)
{
    postype_t pos_start = 17;
    float vel_start = 20;
    float accel_start = 1.0;
    const float dt = 0.01;

    // test for update_pos_vel_accel includes update_vel_accel.
    // test unlimited behaviour
    // 1
    float vel = vel_start;
    postype_t pos = pos_start;
    float accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 0.0, 0.0, 0.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // 2
    vel = vel_start;
    pos = pos_start;
    accel = -accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 0.0, 0.0, 0.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // error has no impact when not limited
    // 3
    vel = vel_start;
    pos = pos_start;
    accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 0.0, 1.0, 1.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // 4
    vel = vel_start;
    pos = pos_start;
    accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 0.0, -1.0, -1.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // test unlimited behaviour
    // zero error should result in normal behaviour
    // 5
    vel = vel_start;
    pos = pos_start;
    accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 1.0, 0.0, 0.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // 6
    vel = vel_start;
    pos = pos_start;
    accel = -accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 1.0, 0.0, 0.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // 7
    vel = vel_start;
    pos = pos_start;
    accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, -1.0, 0.0, 0.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // 8
    vel = vel_start;
    pos = pos_start;
    accel = -accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, -1.0, 0.0, 0.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));
    
    // error sign opposite to limit sign should result in normal behaviour
    // 9
    vel = vel_start;
    pos = pos_start;
    accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 1.0, -1.0, -1.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // 10
    vel = vel_start;
    pos = pos_start;
    accel = -accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 1.0, -1.0, -1.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // 11
    vel = vel_start;
    pos = pos_start;
    accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, -1.0, 1.0, 1.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // 12
    vel = vel_start;
    pos = pos_start;
    accel = -accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, -1.0, 1.0, 1.0);
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));
    
    // error sign same as limit sign should result various limited behaviours
    // 13
    vel = vel_start;
    pos = pos_start;
    accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 1.0, 1.0, 1.0);
    // vel is not increased
    EXPECT_FLOAT_EQ(vel, vel_start);
    // pos is not increased
    EXPECT_FLOAT_EQ(pos, pos_start);

    // 14
    vel = vel_start;
    pos = pos_start;
    accel = -accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 1.0, 1.0, 1.0);
    // vel is decreased
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    // pos is not increased
    EXPECT_FLOAT_EQ(pos, pos_start);

    // 15
    vel = vel_start;
    pos = pos_start;
    accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, -1.0, -1.0, -1.0);
    // vel is increased
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    // pos is increased
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // 16
    vel = vel_start;
    pos = pos_start;
    accel = -accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, -1.0, -1.0, -1.0);
    // velocity is limited but limit is not applied because velocity is reducing
    EXPECT_FLOAT_EQ(vel, vel_start + accel * dt);
    // pos is increased
    EXPECT_FLOAT_EQ(pos, pos_start + vel_start * dt + 0.5 * accel * sq(dt));

    // 17
    vel = -vel_start;
    pos = pos_start;
    accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 1.0, 1.0, 1.0);
    // velocity is limited but limit is not applied because velocity is reducing
    EXPECT_FLOAT_EQ(vel, -vel_start + accel * dt);
    // pos is decreased
    EXPECT_FLOAT_EQ(pos, pos_start - vel_start * dt + 0.5 * accel * sq(dt));

    // 18
    vel_start = 0.1 * accel_start * dt;
    vel = vel_start;
    pos = pos_start;
    accel = -accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, -1.0, -1.0, -1.0);
    // velocity is limited but limit is not applied because velocity is reducing
    // final result is zero because velocity would change sign during dt
    EXPECT_FLOAT_EQ(vel, 0.0);
    // pos is not changed because is_negative(vel_start * dt + 0.5 * accel * sq(t))
    EXPECT_FLOAT_EQ(pos, pos_start);

    // 19
    vel = -vel_start;
    pos = pos_start;
    accel = accel_start;
    update_pos_vel_accel(pos, vel, accel, dt, 1.0, 1.0, 1.0);
    // velocity is limited but limit is not applied because velocity is reducing
    // final result is zero because velocity would change sign during dt
    EXPECT_FLOAT_EQ(vel, 0.0);
    // pos is not changed because is_negative(vel_start * dt + 0.5 * accel * sq(t))
    EXPECT_FLOAT_EQ(pos, pos_start);


    // test for update_pos_vel_accel includes update_vel_accel.
    // test unlimited behaviour
    
    // 1
    pos_start = 17;
    vel_start = 20;
    accel_start = 1.0;
    Vector2p posxy = Vector2p(pos_start, 0.0);
    Vector2f velxy = Vector2f(vel_start, 0.0);
    Vector2f accelxy = Vector2f(accel_start, 0.0);
    Vector2f limit = Vector2f(0.0, 0.0);
    Vector2f pos_error = Vector2f(0.0, 0.0);
    Vector2f vel_error = Vector2f(0.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 2
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(-accel_start, 0.0);
    limit = Vector2f(0.0, 0.0);
    pos_error = Vector2f(0.0, 0.0);
    vel_error = Vector2f(0.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // error has no impact when not limited
    // 3
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(accel_start, 0.0);
    limit = Vector2f(0.0, 0.0);
    pos_error = Vector2f(1.0, 0.0);
    vel_error = Vector2f(1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 4
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(accel_start, 0.0);
    limit = Vector2f(0.0, 0.0);
    pos_error = Vector2f(0.0, 0.0);
    vel_error = Vector2f(0.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // test unlimited behaviour
    // zero error should result in normal behaviour
    // 5
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(accel_start, 0.0);
    limit = Vector2f(1.0, 0.0);
    pos_error = Vector2f(0.0, 0.0);
    vel_error = Vector2f(0.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 6
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(-accel_start, 0.0);
    limit = Vector2f(1.0, 0.0);
    pos_error = Vector2f(0.0, 0.0);
    vel_error = Vector2f(0.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 7
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(accel_start, 0.0);
    limit = Vector2f(1.0, 0.0);
    pos_error = Vector2f(0.0, 0.0);
    vel_error = Vector2f(0.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 8
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(-accel_start, 0.0);
    limit = Vector2f(1.0, 0.0);
    pos_error = Vector2f(0.0, 0.0);
    vel_error = Vector2f(0.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    
    // error sign opposite to limit sign should result in normal behaviour
    // 9
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(accel_start, 0.0);
    limit = Vector2f(1.0, 0.0);
    pos_error = Vector2f(-1.0, 0.0);
    vel_error = Vector2f(-1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 10
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(-accel_start, 0.0);
    limit = Vector2f(1.0, 0.0);
    pos_error = Vector2f(-1.0, 0.0);
    vel_error = Vector2f(-1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 11
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(accel_start, 0.0);
    limit = Vector2f(-1.0, 0.0);
    pos_error = Vector2f(1.0, 0.0);
    vel_error = Vector2f(1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 12
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(-accel_start, 0.0);
    limit = Vector2f(-1.0, 0.0);
    pos_error = Vector2f(1.0, 0.0);
    vel_error = Vector2f(1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    
    // error sign same as limit sign should result various limited behaviours
    // 13
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(accel_start, 0.0);
    limit = Vector2f(1.0, 0.0);
    pos_error = Vector2f(1.0, 0.0);
    vel_error = Vector2f(1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    // vel is not increased
    EXPECT_FLOAT_EQ(velxy.x, vel_start);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    // pos is not increased
    EXPECT_FLOAT_EQ(posxy.x, pos_start);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 14
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(-accel_start, 0.0);
    limit = Vector2f(1.0, 0.0);
    pos_error = Vector2f(1.0, 0.0);
    vel_error = Vector2f(1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    // vel is decreased
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    // pos is not increased
    EXPECT_FLOAT_EQ(posxy.x, pos_start);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 15
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(accel_start, 0.0);
    limit = Vector2f(-1.0, 0.0);
    pos_error = Vector2f(-1.0, 0.0);
    vel_error = Vector2f(-1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    // vel is increased
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    // pos is increased
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 16
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(-accel_start, 0.0);
    limit = Vector2f(-1.0, 0.0);
    pos_error = Vector2f(-1.0, 0.0);
    vel_error = Vector2f(-1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    // velocity is limited but limit is not applied because velocity is reducing
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    // pos is increased
    EXPECT_FLOAT_EQ(posxy.x, pos_start + vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 17
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(-vel_start, 0.0);
    accelxy = Vector2f(accel_start, 0.0);
    limit = Vector2f(1.0, 0.0);
    pos_error = Vector2f(1.0, 0.0);
    vel_error = Vector2f(1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    // velocity is limited but limit is not applied because velocity is reducing
    EXPECT_FLOAT_EQ(velxy.x, -vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    // pos is decreased
    EXPECT_FLOAT_EQ(posxy.x, pos_start - vel_start * dt + 0.5 * accelxy.x * sq(dt));
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 18
    vel_start = 0.1 * accel_start * dt;
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(vel_start, 0.0);
    accelxy = Vector2f(-accel_start, 0.0);
    limit = Vector2f(-1.0, 0.0);
    pos_error = Vector2f(-1.0, 0.0);
    vel_error = Vector2f(-1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    // velocity is limited but limit is not applied because velocity is reducing
    // ideally this would be zero but code makes a simplification here
    EXPECT_FLOAT_EQ(velxy.x, vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    // pos is not changed because is_negative(vel_start * dt + 0.5 * accel * sq(t))
    EXPECT_FLOAT_EQ(posxy.x, pos_start);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);

    // 19
    posxy = Vector2p(pos_start, 0.0);
    velxy = Vector2f(-vel_start, 0.0);
    accelxy = Vector2f(accel_start, 0.0);
    limit = Vector2f(1.0, 0.0);
    pos_error = Vector2f(1.0, 0.0);
    vel_error = Vector2f(1.0, 0.0);
    update_pos_vel_accel_xy(posxy, velxy, accelxy, dt, limit, pos_error, vel_error);
    // velocity is limited but limit is not applied because velocity is reducing
    // ideally this would be zero but code makes a simplification here
    EXPECT_FLOAT_EQ(velxy.x, -vel_start + accelxy.x * dt);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
    // pos is not changed because is_negative(vel_start * dt + 0.5 * accel * sq(t))
    EXPECT_FLOAT_EQ(posxy.x, pos_start);
    EXPECT_FLOAT_EQ(velxy.y, 0.0);
}

// catch floating point exceptions
sigjmp_buf avert_your_eyes_children;
static void _tc_sig_fpe(int signum)
{
    siglongjmp(avert_your_eyes_children, 1);
}

TEST(Control, test_limit_accel)
{
    // reproduction of FPE (https://github.com/ArduPilot/ardupilot/issues/28969)
    // FPE will only be raised in SITL HAL, so compiling for linux HAL
    // isn't useful.
    const Vector2f vel{
        0.984285712, 0.176583186
    };
    Vector2f accel{99.9008408, -557.304077};
    const float accel_max = 566.187256;

    struct sigaction old_sa_fpe = {};

    struct sigaction sa_fpe = {};
    sigemptyset(&sa_fpe.sa_mask);
    sa_fpe.sa_handler = _tc_sig_fpe;
    if (sigaction(SIGFPE, &sa_fpe, &old_sa_fpe) == -1) {
        abort();
    }
    const int excepts = FE_UNDERFLOW | FE_OVERFLOW | FE_INVALID;
    fexcept_t old_except_flags;
    if (fegetexceptflag(&old_except_flags, excepts) == -1) {
        abort();
    }

    feenableexcept(excepts);

    bool signal_caught = false;
    if (sigsetjmp(avert_your_eyes_children, 1)) {
        // we come through here if an FPE is triggered (via a goto in
        // our custom signal handler, _tc_sig_fpe)
        signal_caught = true;
    } else {
        // we come through here normally
        EXPECT_TRUE(limit_accel_xy(vel, accel, accel_max));
    }

    EXPECT_FALSE(signal_caught);

    // now restore the original fpe handling
    if (fesetexceptflag(&old_except_flags, excepts) == -1) {
        abort();
    }
    sigaction(SIGFPE, &old_sa_fpe, nullptr);
}

AP_GTEST_MAIN()
int hal = 0;
