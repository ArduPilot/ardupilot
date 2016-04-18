#include "AP_Math.h"
#include <float.h>

// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin(float v)
{
    if (isnan(v)) {
        return 0.0f;
    }
    if (v >= 1.0f) {
        return M_PI/2;
    }
    if (v <= -1.0f) {
        return -M_PI/2;
    }
    return asinf(v);
}

// a varient of sqrt() that checks the input ranges and ensures a
// valid value as output. If a negative number is given then 0 is
// returned. The reasoning is that a negative number for sqrt() in our
// code is usually caused by small numerical rounding errors, so the
// real input should have been zero
float safe_sqrt(float v)
{
    float ret = sqrtf(v);
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}

/*
  linear interpolation based on a variable in a range
 */
float linear_interpolate(float low_output, float high_output,
                         float var_value,
                         float var_low, float var_high)
{
    if (var_value <= var_low) {
        return low_output;
    }
    if (var_value >= var_high) {
        return high_output;
    }
    float p = (var_value - var_low) / (var_high - var_low);
    return low_output + p * (high_output - low_output);
}

/*
  returns the closest point on a line segment to a given point
 */
float closest_point_on_segment_as_ratio(const Vector2f &segment_start, const Vector2f &segment_end,
                                        const Vector2f &point)
{
    Vector2f AP = point - segment_start;            // Vector from A to P
    Vector2f AB = segment_end - segment_start;      // Vector from A to B

    float magnitudeAB = AB.x * AB.x + AB.y * AB.y;  // Magnitude of AB vector (it's length squared)

    float ABAPproduct = AP * AB;                    // The dot product of a_to_p and a_to_b
    float distance = ABAPproduct / magnitudeAB;     // The normalized "distance" from a to your closest point
    return constrain_float(distance, 0.0f, 1.0f);   // constrain to end points
}

/*
  returns barycentric weightings of a point within a triangle
 */
void barycentric_interpolate(const Vector2f &triangle_vertex1, const Vector2f &triangle_vertex2, const Vector2f &triangle_vertex3,
                              const Vector2f &point,
                              float &weight1, float &weight2, float &weight3)
{
    Vector2f v0 = triangle_vertex2 - triangle_vertex1;
    Vector2f v1 = triangle_vertex3 - triangle_vertex1;
    Vector2f v2 = point - triangle_vertex1;
    float d00 = v0 * v0;
    float d01 = v0 * v1;
    float d11 = v1 * v1;
    float d20 = v2 * v0;
    float d21 = v2 * v1;
    float denom = d00 * d11 - d01 * d01;
    if (!is_zero(denom)) {
        weight2 = (d11 * d20 - d01 * d21) / denom;
        weight3 = (d00 * d21 - d01 * d20) / denom;
    } else {
        weight2 = 0.0f;
        weight3 = 0.0f;
    }
    weight1 = 1.0f - weight2 - weight3;

    // handle cases where point was outside triangle
    if (weight1 < 0.0f) {
        weight1 = 0.0f;
        weight2 = closest_point_on_segment_as_ratio(triangle_vertex2, triangle_vertex3, point);
        weight3 = 1.0f - weight2;
        return;
    }
    if (weight2 < 0.0f) {
        weight2 = 0.0f;
        weight1 = closest_point_on_segment_as_ratio(triangle_vertex1, triangle_vertex3, point);
        weight3 = 1.0f - weight1;
        return;
    }
    if (weight3 < 0.0f) {
        weight3 = 0.0f;
        weight1 = closest_point_on_segment_as_ratio(triangle_vertex1, triangle_vertex2, point);
        weight2 = 1.0f - weight1;
        return;
    }
}
