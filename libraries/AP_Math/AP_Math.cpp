#include "AP_Math.h"

#include <float.h>

#include <AP_InternalError/AP_InternalError.h>

/*
 * is_equal(): Integer implementation, provided for convenience and
 * compatibility with old code. Expands to the same as comparing the values
 * directly
 */
template <typename Arithmetic1, typename Arithmetic2>
typename std::enable_if<std::is_integral<typename std::common_type<Arithmetic1, Arithmetic2>::type>::value ,bool>::type
is_equal(const Arithmetic1 v_1, const Arithmetic2 v_2)
{
    typedef typename std::common_type<Arithmetic1, Arithmetic2>::type common_type;
    return static_cast<common_type>(v_1) == static_cast<common_type>(v_2);
}

/*
 * is_equal(): double/float implementation - takes into account
 * std::numeric_limits<T>::epsilon() to return if 2 values are equal.
 */
template <typename Arithmetic1, typename Arithmetic2>
typename std::enable_if<std::is_floating_point<typename std::common_type<Arithmetic1, Arithmetic2>::type>::value, bool>::type
is_equal(const Arithmetic1 v_1, const Arithmetic2 v_2)
{
#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
    typedef typename std::common_type<Arithmetic1, Arithmetic2>::type common_type;
    typedef typename std::remove_cv<common_type>::type common_type_nonconst;
    if (std::is_same<double, common_type_nonconst>::value) {
        return fabs(v_1 - v_2) < std::numeric_limits<double>::epsilon();
    }
#endif
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wabsolute-value"
    // clang doesn't realise we catch the double case above and warns
    // about loss of precision here.
    return fabsf(v_1 - v_2) < std::numeric_limits<float>::epsilon();
#pragma clang diagnostic pop
}

template bool is_equal<int>(const int v_1, const int v_2);
template bool is_equal<short>(const short v_1, const short v_2);
template bool is_equal<long>(const long v_1, const long v_2);
template bool is_equal<float>(const float v_1, const float v_2);
template bool is_equal<double>(const double v_1, const double v_2);

template <typename T>
float safe_asin(const T v)
{
    const float f = static_cast<const float>(v);
    if (isnan(f)) {
        return 0.0f;
    }
    if (f >= 1.0f) {
        return static_cast<float>(M_PI_2);
    }
    if (f <= -1.0f) {
        return static_cast<float>(-M_PI_2);
    }
    return asinf(f);
}

template float safe_asin<int>(const int v);
template float safe_asin<short>(const short v);
template float safe_asin<float>(const float v);
template float safe_asin<double>(const double v);

template <typename T>
float safe_sqrt(const T v)
{
    float ret = sqrtf(static_cast<float>(v));
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}

template float safe_sqrt<int>(const int v);
template float safe_sqrt<short>(const short v);
template float safe_sqrt<float>(const float v);
template float safe_sqrt<double>(const double v);

/*
 * linear interpolation based on a variable in a range
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

/* cubic "expo" curve generator
 * alpha range: [0,1] min to max expo
 * input range: [-1,1]
 */
constexpr float expo_curve(float alpha, float x)
{
    return (1.0f - alpha) * x + alpha * x * x * x;
}

/* throttle curve generator
 * thr_mid: output at mid stick
 * alpha: expo coefficient
 * thr_in: [0-1]
 */
float throttle_curve(float thr_mid, float alpha, float thr_in)
{
    float alpha2 = alpha + 1.25 * (1.0f - alpha) * (0.5f - thr_mid) / 0.5f;
    alpha2 = constrain_float(alpha2, 0.0f, 1.0f);
    float thr_out = 0.0f;
    if (thr_in < 0.5f) {
        float t = linear_interpolate(-1.0f, 0.0f, thr_in, 0.0f, 0.5f);
        thr_out = linear_interpolate(0.0f, thr_mid, expo_curve(alpha, t), -1.0f, 0.0f);
    } else {
        float t = linear_interpolate(0.0f, 1.0f, thr_in, 0.5f, 1.0f);
        thr_out = linear_interpolate(thr_mid, 1.0f, expo_curve(alpha2, t), 0.0f, 1.0f);
    }
    return thr_out;
}

template <typename T>
T wrap_180(const T angle)
{
    auto res = wrap_360(angle);
    if (res > T(180)) {
        res -= T(360);
    }
    return res;
}

template <typename T>
T wrap_180_cd(const T angle)
{
    auto res = wrap_360_cd(angle);
    if (res > T(18000)) {
        res -= T(36000);
    }
    return res;
}

template int wrap_180<int>(const int angle);
template short wrap_180<short>(const short angle);
template float wrap_180<float>(const float angle);
#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
template double wrap_180<double>(const double angle);
#endif

template int wrap_180_cd<int>(const int angle);
template long wrap_180_cd<long>(const long angle);
template short wrap_180_cd<short>(const short angle);
template float wrap_180_cd<float>(const float angle);
#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
template double wrap_180_cd<double>(const double angle);
#endif

float wrap_360(const float angle)
{
    float res = fmodf(angle, 360.0f);
    if (res < 0) {
        res += 360.0f;
    }
    return res;
}

#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
double wrap_360(const double angle)
{
    double res = fmod(angle, 360.0);
    if (res < 0) {
        res += 360.0;
    }
    return res;
}
#endif

int wrap_360(const int angle)
{
    int res = angle % 360;
    if (res < 0) {
        res += 360;
    }
    return res;
}

float wrap_360_cd(const float angle)
{
    float res = fmodf(angle, 36000.0f);
    if (res < 0) {
        res += 36000.0f;
    }
    return res;
}

#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
double wrap_360_cd(const double angle)
{
    double res = fmod(angle, 36000.0);
    if (res < 0) {
        res += 36000.0;
    }
    return res;
}
#endif

int wrap_360_cd(const int angle)
{
    int res = angle % 36000;
    if (res < 0) {
        res += 36000;
    }
    return res;
}

long wrap_360_cd(const long angle)
{
    long res = angle % 36000;
    if (res < 0) {
        res += 36000;
    }
    return res;
}
template <typename T>
float wrap_PI(const T radian)
{
    auto res = wrap_2PI(radian);
    if (res > M_PI) {
        res -= M_2PI;
    }
    return res;
}

template float wrap_PI<int>(const int radian);
template float wrap_PI<short>(const short radian);
template float wrap_PI<float>(const float radian);
template float wrap_PI<double>(const double radian);

template <typename T>
float wrap_2PI(const T radian)
{
    float res = fmodf(static_cast<float>(radian), M_2PI);
    if (res < 0) {
        res += M_2PI;
    }
    return res;
}

template float wrap_2PI<int>(const int radian);
template float wrap_2PI<short>(const short radian);
template float wrap_2PI<float>(const float radian);
template float wrap_2PI<double>(const double radian);

template <typename T>
T constrain_value(const T amt, const T low, const T high)
{
    // the check for NaN as a float prevents propagation of floating point
    // errors through any function that uses constrain_value(). The normal
    // float semantics already handle -Inf and +Inf
    if (isnan(amt)) {
        INTERNAL_ERROR(AP_InternalError::error_t::constraining_nan);
        return (low + high) / 2;
    }

    if (amt < low) {
        return low;
    }

    if (amt > high) {
        return high;
    }

    return amt;
}

template int constrain_value<int>(const int amt, const int low, const int high);
template long constrain_value<long>(const long amt, const long low, const long high);
template long long constrain_value<long long>(const long long amt, const long long low, const long long high);
template short constrain_value<short>(const short amt, const short low, const short high);
template float constrain_value<float>(const float amt, const float low, const float high);
template double constrain_value<double>(const double amt, const double low, const double high);


/*
  simple 16 bit random number generator
 */
uint16_t get_random16(void)
{
    static uint32_t m_z = 1234;
    static uint32_t m_w = 76542;
    m_z = 36969 * (m_z & 0xFFFFu) + (m_z >> 16);
    m_w = 18000 * (m_w & 0xFFFFu) + (m_w >> 16);
    return ((m_z << 16) + m_w) & 0xFFFF;
}


#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// generate a random float between -1 and 1, for use in SITL
float rand_float(void)
{
    return ((((unsigned)random()) % 2000000) - 1.0e6) / 1.0e6;
}

Vector3f rand_vec3f(void)
{
    Vector3f v = Vector3f(rand_float(),
                          rand_float(),
                          rand_float());
    if (!is_zero(v.length())) {
        v.normalize();
    }
    return v;
}
#endif

/*
  return true if two rotations are equivalent
  This copes with the fact that we have some duplicates, like ROLL_180_YAW_90 and PITCH_180_YAW_270
 */
bool rotation_equal(enum Rotation r1, enum Rotation r2)
{
    if (r1 == r2) {
        return true;
    }
    Vector3f v(1,2,3);
    Vector3f v1 = v;
    Vector3f v2 = v;
    v1.rotate(r1);
    v2.rotate(r2);
    return (v1 - v2).length() < 0.001;
}

/*
 * return a velocity correction (in m/s in NED) for a sensor's position given it's position offsets
 * this correction should be added to the sensor NED measurement
 * sensor_offset_bf is in meters in body frame (Foward, Right, Down)
 * rot_ef_to_bf is a rotation matrix to rotate from earth-frame (NED) to body frame
 * angular_rate is rad/sec
 */
Vector3f get_vel_correction_for_sensor_offset(const Vector3f &sensor_offset_bf, const Matrix3f &rot_ef_to_bf, const Vector3f &angular_rate)
{
    if (sensor_offset_bf.is_zero()) {
        return Vector3f();
    }

    // correct velocity
    const Vector3f vel_offset_body = angular_rate % sensor_offset_bf;
    return rot_ef_to_bf.mul_transpose(vel_offset_body) * -1.0f;
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// fill an array of float with NaN, used to invalidate memory in SITL
void fill_nanf(float *f, uint16_t count)
{
    while (count--) {
        *f++ = std::numeric_limits<float>::signaling_NaN();
    }
}
#endif
