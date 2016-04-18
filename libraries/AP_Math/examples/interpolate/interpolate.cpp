/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// Unit tests for the AP_Math rotations code
//

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// test interpolation from a point to the closest point on segment
static void test_closest_point_on_segment(void)
{
    Vector2f seg_start, seg_end;
    Vector2f point;

    hal.console->printf("\nInterpolate from point to segment (i.e. use closest point on segment)\n");

    // half-way test
    seg_start = Vector2f(1,1);
    seg_end = Vector2f(3,3);
    point = Vector2f(1,3);
    hal.console->printf("Half-way test (ratio should be 0.5):\n");
    hal.console->printf("Start x:%4.2f y:%4.2f, End x:%4.2f y:%4.2f, Point x:%4.2f y:%4.2f\n",
            (double)seg_start.x, (double)seg_start.y,
            (double)seg_end.x, (double)seg_end.y,
            (double)point.x, (double)point.y);
    hal.console->printf("Ratio: %4.2f\n\n", (double)closest_point_on_segment_as_ratio(seg_start, seg_end, point));

    // before start
    seg_start = Vector2f(1,1);
    seg_end = Vector2f(3,3);
    point = Vector2f(-1,0);
    hal.console->printf("Before start (ratio should be 0):\n");
    hal.console->printf("Start x:%4.2f y:%4.2f, End x:%4.2f y:%4.2f, Point x:%4.2f y:%4.2f\n",
                (double)seg_start.x, (double)seg_start.y,
                (double)seg_end.x, (double)seg_end.y,
                (double)point.x, (double)point.y);
    hal.console->printf("Ratio: %4.2f\n\n", (double)closest_point_on_segment_as_ratio(seg_start, seg_end, point));

    // on start
    seg_start = Vector2f(1,1);
    seg_end = Vector2f(3,3);
    point = Vector2f(1,1);
    hal.console->printf("Exactly on start (ratio should be 0):\n");
    hal.console->printf("Start x:%4.2f y:%4.2f, End x:%4.2f y:%4.2f, Point x:%4.2f y:%4.2f\n",
                (double)seg_start.x, (double)seg_start.y,
                (double)seg_end.x, (double)seg_end.y,
                (double)point.x, (double)point.y);
    hal.console->printf("Ratio: %4.2f\n\n", (double)closest_point_on_segment_as_ratio(seg_start, seg_end, point));

    // after end
    seg_start = Vector2f(1,1);
    seg_end = Vector2f(3,3);
    point = Vector2f(4,2);
    hal.console->printf("After end (ratio should be 1.0):\n");
    hal.console->printf("Start x:%4.2f y:%4.2f, End x:%4.2f y:%4.2f, Point x:%4.2f y:%4.2f\n",
                (double)seg_start.x, (double)seg_start.y,
                (double)seg_end.x, (double)seg_end.y,
                (double)point.x, (double)point.y);
    hal.console->printf("Ratio: %4.2f\n\n", (double)closest_point_on_segment_as_ratio(seg_start, seg_end, point));
}

// test barycentric interpolation (i.e. point within a triangle)
static void test_barycentric_interpolate(void)
{
    hal.console->printf("\nInterpolate from a point to a triangle (i.e. use weighting of value at each point of triangle)\n");

    // middle test
    Vector2f triangle_vertex1 = Vector2f(1,1);  // bottom left corner
    Vector2f triangle_vertex2 = Vector2f(2,2.732051);  // top middle corner
    Vector2f triangle_vertex3 = Vector2f(3,1);  // bottom right corner
    Vector2f point = (triangle_vertex1 + triangle_vertex2 + triangle_vertex3) / 3.0f;
    hal.console->printf("Middle test:\n");
    hal.console->printf("V1 x:%4.2f y:%4.2f, V2 x:%4.2f y:%4.2f, V3 x:%4.2f y:%4.2f, Point x:%4.2f y:%4.2f\n",
            (double)triangle_vertex1.x, (double)triangle_vertex1.y,
            (double)triangle_vertex2.x, (double)triangle_vertex2.y,
            (double)triangle_vertex3.x, (double)triangle_vertex3.y,
            (double)point.x, (double)point.y);
    float weight1, weight2, weight3;
    barycentric_interpolate(triangle_vertex1, triangle_vertex2, triangle_vertex3, point, weight1, weight2, weight3);
    hal.console->printf("Weightings V1:%4.2f V2:%4.2f V3:%4.2f\n", (double)weight1, (double)weight2, (double)weight3);
    hal.console->printf("Should be V1:0.33 V2:0.33 V3:0.33\n\n");

    // vector1 test
    point = triangle_vertex1;
    hal.console->printf("On bottom left corner:\n");
    hal.console->printf("V1 x:%4.2f y:%4.2f, V2 x:%4.2f y:%4.2f, V3 x:%4.2f y:%4.2f, Point x:%4.2f y:%4.2f\n",
            (double)triangle_vertex1.x, (double)triangle_vertex1.y,
            (double)triangle_vertex2.x, (double)triangle_vertex2.y,
            (double)triangle_vertex3.x, (double)triangle_vertex3.y,
            (double)point.x, (double)point.y);
    barycentric_interpolate(triangle_vertex1, triangle_vertex2, triangle_vertex3, point, weight1, weight2, weight3);
    hal.console->printf("Weightings V1:%4.2f V2:%4.2f V3:%4.2f\n", (double)weight1, (double)weight2, (double)weight3);
    hal.console->printf("Should be V1:1.0 V2:0.0 V3:0.0\n\n");

    // vector2 test
    point = triangle_vertex2;
    hal.console->printf("On top corner:\n");
    hal.console->printf("V1 x:%4.2f y:%4.2f, V2 x:%4.2f y:%4.2f, V3 x:%4.2f y:%4.2f, Point x:%4.2f y:%4.2f\n",
            (double)triangle_vertex1.x, (double)triangle_vertex1.y,
            (double)triangle_vertex2.x, (double)triangle_vertex2.y,
            (double)triangle_vertex3.x, (double)triangle_vertex3.y,
            (double)point.x, (double)point.y);
    barycentric_interpolate(triangle_vertex1, triangle_vertex2, triangle_vertex3, point, weight1, weight2, weight3);
    hal.console->printf("Weightings V1:%4.2f V2:%4.2f V3:%4.2f\n", (double)weight1, (double)weight2, (double)weight3);
    hal.console->printf("Should be V1:0.0 V2:1.0 V3:0.0\n\n");

    // vector3 test
    point = triangle_vertex3;
    hal.console->printf("On bottom right corner:\n");
    hal.console->printf("V1 x:%4.2f y:%4.2f, V2 x:%4.2f y:%4.2f, V3 x:%4.2f y:%4.2f, Point x:%4.2f y:%4.2f\n",
            (double)triangle_vertex1.x, (double)triangle_vertex1.y,
            (double)triangle_vertex2.x, (double)triangle_vertex2.y,
            (double)triangle_vertex3.x, (double)triangle_vertex3.y,
            (double)point.x, (double)point.y);
    barycentric_interpolate(triangle_vertex1, triangle_vertex2, triangle_vertex3, point, weight1, weight2, weight3);
    hal.console->printf("Weightings V1:%4.2f V2:%4.2f V3:%4.2f\n", (double)weight1, (double)weight2, (double)weight3);
    hal.console->printf("Should be V1:0.0 V2:0.0 V3:1.0\n\n");

    // bottom edge test
    point = (triangle_vertex1 + triangle_vertex3) / 2.0f;
    hal.console->printf("On bottom edge (weight1 and weight3 should be 0.5, weight2 should be 0):\n");
    hal.console->printf("V1 x:%4.2f y:%4.2f, V2 x:%4.2f y:%4.2f, V3 x:%4.2f y:%4.2f, Point x:%4.2f y:%4.2f\n",
            (double)triangle_vertex1.x, (double)triangle_vertex1.y,
            (double)triangle_vertex2.x, (double)triangle_vertex2.y,
            (double)triangle_vertex3.x, (double)triangle_vertex3.y,
            (double)point.x, (double)point.y);
    barycentric_interpolate(triangle_vertex1, triangle_vertex2, triangle_vertex3, point, weight1, weight2, weight3);
    hal.console->printf("Weightings V1:%4.2f V2:%4.2f V3:%4.2f\n", (double)weight1, (double)weight2, (double)weight3);
    hal.console->printf("Should be V1:0.5 V2:0 V3:0.5\n\n");

    // left edge test
    point = (triangle_vertex1 + triangle_vertex2) / 2.0f;
    hal.console->printf("On left edge:\n");
    hal.console->printf("V1 x:%4.2f y:%4.2f, V2 x:%4.2f y:%4.2f, V3 x:%4.2f y:%4.2f, Point x:%4.2f y:%4.2f\n",
            (double)triangle_vertex1.x, (double)triangle_vertex1.y,
            (double)triangle_vertex2.x, (double)triangle_vertex2.y,
            (double)triangle_vertex3.x, (double)triangle_vertex3.y,
            (double)point.x, (double)point.y);
    barycentric_interpolate(triangle_vertex1, triangle_vertex2, triangle_vertex3, point, weight1, weight2, weight3);
    hal.console->printf("Weightings V1:%4.2f V2:%4.2f V3:%4.2f\n", (double)weight1, (double)weight2, (double)weight3);
    hal.console->printf("Should be V1:0.5 V2:0.5 V3:0\n\n");

    // right edge test
    point = (triangle_vertex2 + triangle_vertex3) / 2.0f;
    hal.console->printf("On right edge:\n");
    hal.console->printf("V1 x:%4.2f y:%4.2f, V2 x:%4.2f y:%4.2f, V3 x:%4.2f y:%4.2f, Point x:%4.2f y:%4.2f\n",
            (double)triangle_vertex1.x, (double)triangle_vertex1.y,
            (double)triangle_vertex2.x, (double)triangle_vertex2.y,
            (double)triangle_vertex3.x, (double)triangle_vertex3.y,
            (double)point.x, (double)point.y);
    barycentric_interpolate(triangle_vertex1, triangle_vertex2, triangle_vertex3, point, weight1, weight2, weight3);
    hal.console->printf("Weightings V1:%4.2f V2:%4.2f V3:%4.2f\n", (double)weight1, (double)weight2, (double)weight3);
    hal.console->printf("Should be V1:0 V2:0.5 V3:0.5\n\n");

    // outside of triangle test
    point = Vector2f(1.66666,0);
    hal.console->printf("Outside of triangle:\n");
    hal.console->printf("V1 x:%4.2f y:%4.2f, V2 x:%4.2f y:%4.2f, V3 x:%4.2f y:%4.2f, Point x:%4.2f y:%4.2f\n",
            (double)triangle_vertex1.x, (double)triangle_vertex1.y,
            (double)triangle_vertex2.x, (double)triangle_vertex2.y,
            (double)triangle_vertex3.x, (double)triangle_vertex3.y,
            (double)point.x, (double)point.y);
    barycentric_interpolate(triangle_vertex1, triangle_vertex2, triangle_vertex3, point, weight1, weight2, weight3);
    hal.console->printf("Weightings V1:%4.2f V2:%4.2f V3:%4.2f\n", (double)weight1, (double)weight2, (double)weight3);
    hal.console->printf("Closest Point Ratio: %4.2f\n", (double)closest_point_on_segment_as_ratio(triangle_vertex1, triangle_vertex3, point));
    hal.console->printf("Should be V1:0.33 V2:0 V3:0.66\n\n");
}

/*
 *  run tests
 */
void setup(void)
{
    test_closest_point_on_segment();
    test_barycentric_interpolate();
    hal.console->printf("interpolation tests done\n");
}

void loop(void) {}

AP_HAL_MAIN();
