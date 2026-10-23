/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// unit tests for the acceleration SplineCurve reports along its path

#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/vector3.h>
#include <AP_Math/SplineCurve.h>

static const uint32_t MAX_ITERS = 200000;

// drive a spline to completion, returning the worst mismatch between the turning part of the
// acceleration it reports and that of the derivative of the velocity it reports
static float worst_accel_mismatch(SplineCurve &leg, float &worst_normal, float &peak_speed)
{
    const float dt = 0.0025f;
    Vector3p pos;
    Vector3f vel, accel, vel_prev;
    bool have_prev = false;
    float worst = 0.0f;
    worst_normal = 0.0f;
    peak_speed = 0.0f;
    for (uint32_t i = 0; i < MAX_ITERS && !leg.reached_destination(); i++) {
        vel_prev = vel;
        leg.advance_target_along_track(dt, pos, vel, accel);
        const float speed = vel.length();
        peak_speed = MAX(peak_speed, speed);
        if (have_prev && speed > 2.0f) {
            const Vector3f fd = (vel - vel_prev) / dt;
            // the components perpendicular to travel
            const Vector3f unit = vel / speed;
            const Vector3f fd_normal = fd - unit * (fd * unit);
            const Vector3f accel_normal = accel - unit * (accel * unit);
            worst = MAX(worst, (fd_normal - accel_normal).length());
            worst_normal = MAX(worst_normal, accel_normal.length());
        }
        have_prev = true;
    }
    EXPECT_TRUE(leg.reached_destination());
    return worst;
}

// The turning acceleration a spline reports must be that of the velocity it reports.
// Anything else means the position controller is being handed a feedforward that does not
// describe the path it is being asked to fly.
TEST(SplineCurve, accel_matches_velocity_derivative)
{
    SplineCurve leg{};
    leg.set_speed_accel(10.0f, 5.0f, 5.0f, 5.0f, 5.0f);
    leg.set_origin_and_destination(Vector3p{0, 0, -50}, Vector3p{60, 40, -70},
                                   Vector3f{15, 0, 0}, Vector3f{0, 15, 0});

    float worst_normal, peak_speed;
    const float worst = worst_accel_mismatch(leg, worst_normal, peak_speed);
    EXPECT_GT(peak_speed, 5.0f);        // it actually flew
    EXPECT_LT(worst, 0.5f);
}

// A vertical spline is the case that matters for inverted flight: the curve must report
// the acceleration that carries it over the top, not zero.
TEST(SplineCurve, vertical_spline_reports_acceleration)
{
    SplineCurve leg{};
    leg.set_speed_accel(20.0f, 20.0f, 20.0f, 20.0f, 20.0f);
    // up and over: leaves climbing, arrives descending
    leg.set_origin_and_destination(Vector3p{0, 0, -60}, Vector3p{40, 0, -60},
                                   Vector3f{0, 0, -25}, Vector3f{0, 0, 25});

    float worst_normal, peak_speed;
    const float worst = worst_accel_mismatch(leg, worst_normal, peak_speed);
    EXPECT_GT(peak_speed, 5.0f);
    EXPECT_LT(worst, 1.0f);
    // it must demand real turning acceleration, which the old code reported as zero
    EXPECT_GT(worst_normal, 1.0f);
}

// For a gentle geometry the reported normal acceleration stays within the lateral budget the
// spline chose its speed against (LATERAL_ACCEL_SCALER is 0.5). The hard bound, which holds
// for any geometry, is the leg's acceleration limit; see rest_endpoint_accel_is_bounded.
TEST(SplineCurve, normal_accel_within_lateral_limit)
{
    const float accel_xy = 6.0f;
    SplineCurve leg{};
    leg.set_speed_accel(12.0f, 6.0f, 6.0f, accel_xy, accel_xy);
    leg.set_origin_and_destination(Vector3p{0, 0, -40}, Vector3p{50, 50, -40},
                                   Vector3f{20, 0, 0}, Vector3f{0, 20, 0});

    float worst_normal, peak_speed;
    worst_accel_mismatch(leg, worst_normal, peak_speed);
    EXPECT_GT(peak_speed, 5.0f);
    EXPECT_LE(worst_normal, 0.5f * accel_xy + 0.5f);
}

// A leg that ends at rest must not report a spurious acceleration as it arrives. At the
// endpoint the spline's parametric velocity collapses to a rounding residual while the
// commanded speed is still being slewed to zero, and dividing by it produced feedforwards
// of 1e10 m/s^2 and more. Integer coordinates happen to cancel exactly, so these are the
// fractional positions a mission actually produces; drive each through arrival and past it.
TEST(SplineCurve, rest_endpoint_accel_is_bounded)
{
    const float accel_max = 5.0f;
    const struct {
        Vector3p origin, dest;
        Vector3f origin_vel;
    } legs[] = {
        {{0.37, 29.13, -23.4}, {-12.71, 1.22, -24.9}, {8.3, 5.1, -2.2}},
        {{-5.62, -33.08, -61.3}, {-4.19, 86.77, -70.1}, {1.4, 3.6, -1.7}},
        {{-12.85, 9.41, -79.6}, {67.03, 19.58, -56.2}, {0.2, -1.9, 2.4}},
        {{36.27, 11.93, -44.7}, {-25.44, 30.16, -82.8}, {1.1, 7.7, -1.3}},
        {{19.66, 21.02, -36.5}, {70.81, 17.39, -85.4}, {-9.2, 7.4, 2.9}},
        {{39.14, 9.78, -41.2}, {91.53, -59.65, -78.3}, {3.8, 9.3, 2.6}},
    };
    for (const auto &l : legs) {
        SplineCurve leg{};
        leg.set_speed_accel(10.0f, 5.0f, 5.0f, accel_max, accel_max);
        leg.set_origin_and_destination(l.origin, l.dest, l.origin_vel, Vector3f{});
        const float dt = 0.0025f;
        Vector3p pos;
        Vector3f vel, accel;
        float worst = 0.0f;
        uint32_t cycles_after = 0;
        for (uint32_t i = 0; i < MAX_ITERS && cycles_after < 200; i++) {
            if (leg.reached_destination()) {
                cycles_after++;
            }
            leg.advance_target_along_track(dt, pos, vel, accel);
            worst = MAX(worst, accel.length());
        }
        EXPECT_EQ(cycles_after, 200U);
        EXPECT_LT(worst, 2.0f * accel_max) << "leg to (" << l.dest.x << "," << l.dest.y << "," << l.dest.z << ")";
    }
}

AP_GTEST_MAIN()
int hal = 0; //weirdly the build will fail without this
