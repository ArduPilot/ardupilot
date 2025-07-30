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
#if AP_MATH_ALLOW_DOUBLE_FUNCTIONS
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

// sqrt which takes any type and returns 0 if the input is NaN or less than zero
template <typename T>
float safe_sqrt(const T v)
{
    // cast before checking so we sqrtf the same value we check
    const float val = static_cast<float>(v);
    // use IEEE-754 compliant function which returns false if val is NaN
    if (isgreaterequal(val, 0)) {
        return sqrtf(val);
    }
    return 0;
}

template float safe_sqrt<int>(const int v);
template float safe_sqrt<short>(const short v);
template float safe_sqrt<float>(const float v);
template float safe_sqrt<double>(const double v);

/*
  replacement for std::swap() needed for STM32
 */
static void swap_float(float &f1, float &f2)
{
    float tmp = f1;
    f1 = f2;
    f2 = tmp;
}

/*
 * linear interpolation based on a variable in a range
 */
float linear_interpolate(float output_low, float output_high,
                         float input_value,
                         float input_low, float input_high)
{
    if (input_low > input_high) {
        // support either polarity
        swap_float(input_low, input_high);
        swap_float(output_low, output_high);
    }
    if (input_value <= input_low) {
        return output_low;
    }
    if (input_value >= input_high) {
        return output_high;
    }
    float p = (input_value - input_low) / (input_high - input_low);
    return output_low + p * (output_high - output_low);
}

/* cubic "expo" curve generator
 * alpha range: [0,1] min to max expo
 * input range: [-1,1]
 */
float expo_curve(float alpha, float x)
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
#if AP_MATH_ALLOW_DOUBLE_FUNCTIONS
template double wrap_180<double>(const double angle);
#endif

template int wrap_180_cd<int>(const int angle);
template long wrap_180_cd<long>(const long angle);
template short wrap_180_cd<short>(const short angle);
template float wrap_180_cd<float>(const float angle);
#if AP_MATH_ALLOW_DOUBLE_FUNCTIONS
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

#if AP_MATH_ALLOW_DOUBLE_FUNCTIONS
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

#if AP_MATH_ALLOW_DOUBLE_FUNCTIONS
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

ftype wrap_PI(const ftype radian)
{
    ftype res = wrap_2PI(radian);
    if (res > M_PI) {
        res -= M_2PI;
    }
    return res;
}

ftype wrap_2PI(const ftype radian)
{
    ftype res = fmodF(radian, M_2PI);
    if (res < 0) {
        res += M_2PI;
    }
    return res;
}

template <typename T>
T constrain_value_line(const T amt, const T low, const T high, uint32_t line)
{
    // the check for NaN as a float prevents propagation of floating point
    // errors through any function that uses constrain_value(). The normal
    // float semantics already handle -Inf and +Inf
    if (isnan(amt)) {
#if AP_INTERNALERROR_ENABLED
        AP::internalerror().error(AP_InternalError::error_t::constraining_nan, line);
#endif
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

template float constrain_value_line<float>(const float amt, const float low, const float high, uint32_t line);
template double constrain_value_line<double>(const double amt, const double low, const double high, uint32_t line);

template <typename T>
T constrain_value(const T amt, const T low, const T high)
{
    // the check for NaN as a float prevents propagation of floating point
    // errors through any function that uses constrain_value(). The normal
    // float semantics already handle -Inf and +Inf
    if (std::is_floating_point<T>::value) {
        if (isnan(amt)) {
            INTERNAL_ERROR(AP_InternalError::error_t::constraining_nan);
            return (low + high) / 2;
        }
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
template unsigned int constrain_value<unsigned int>(const unsigned int amt, const unsigned int low, const unsigned int high);
template long constrain_value<long>(const long amt, const long low, const long high);
template unsigned long constrain_value<unsigned long>(const unsigned long amt, const unsigned long low, const unsigned long high);
template long long constrain_value<long long>(const long long amt, const long long low, const long long high);
template unsigned long long constrain_value<unsigned long long>(const unsigned long long amt, const unsigned long long low, const unsigned long long high);
template short constrain_value<short>(const short amt, const short low, const short high);
template unsigned short constrain_value<unsigned short>(const unsigned short amt, const unsigned short low, const unsigned short high);
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


// generate a random float between -1 and 1
float rand_float(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    return ((((unsigned)random()) % 2000000) - 1.0e6) / 1.0e6;
#else
    return (get_random16() / 65535.0) * 2 - 1;
#endif
}

// generate a random Vector3f with each value between -1.0 and 1.0
Vector3f rand_vec3f(void)
{
    return Vector3f{
        rand_float(),
        rand_float(),
        rand_float()
    };
}

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
Vector3F get_vel_correction_for_sensor_offset(const Vector3F &sensor_offset_bf, const Matrix3F &rot_ef_to_bf, const Vector3F &angular_rate)
{
    if (sensor_offset_bf.is_zero()) {
        return Vector3F();
    }

    // correct velocity
    const Vector3F vel_offset_body = angular_rate % sensor_offset_bf;
    return rot_ef_to_bf.mul_transpose(vel_offset_body) * -1.0;
}

/*
  calculate a low pass filter alpha value
 */
float calc_lowpass_alpha_dt(float dt, float cutoff_freq)
{
    if (is_negative(dt) || is_negative(cutoff_freq)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return 1.0;
    }
    if (is_zero(cutoff_freq)) {
        return 1.0;
    }
    if (is_zero(dt)) {
        return 0.0;
    }
    float rc = 1.0f / (M_2PI * cutoff_freq);
    return dt / (dt + rc);
}

#ifndef AP_MATH_FILL_NANF_USE_MEMCPY
#define AP_MATH_FILL_NANF_USE_MEMCPY (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// fill an array of float with NaN, used to invalidate memory in SITL
void fill_nanf(float *f, uint16_t count)
{
#if AP_MATH_FILL_NANF_USE_MEMCPY
    static bool created;
    static float many_nanfs[2048];
    if (!created) {
        for (uint16_t i=0; i<ARRAY_SIZE(many_nanfs); i++) {
            created = true;
            many_nanfs[i] = std::numeric_limits<float>::signaling_NaN();
        }
    }
    if (count > ARRAY_SIZE(many_nanfs)) {
        AP_HAL::panic("Too big an area to fill");
    }
    memcpy(f, many_nanfs, count*sizeof(many_nanfs[0]));
#else
    const float n = std::numeric_limits<float>::signaling_NaN();
    while (count--) {
        *f++ = n;
    }
#endif
}

void fill_nanf(double *f, uint16_t count)
{
#if AP_MATH_FILL_NANF_USE_MEMCPY
    static bool created;
    static double many_nanfs[2048];
    if (!created) {
        for (uint16_t i=0; i<ARRAY_SIZE(many_nanfs); i++) {
            created = true;
            many_nanfs[i] = std::numeric_limits<double>::signaling_NaN();
        }
    }
    if (count > ARRAY_SIZE(many_nanfs)) {
        AP_HAL::panic("Too big an area to fill");
    }
    memcpy(f, many_nanfs, count*sizeof(many_nanfs[0]));
#else
    while (count--) {
        *f++ = std::numeric_limits<double>::signaling_NaN();
    }
#endif
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL

// Convert 16-bit fixed-point to float
float fixed2float(const uint16_t input, const uint8_t fractional_bits)
{
    return ((float)input / (float)(1U << fractional_bits));
}

// Convert float to 16-bit fixed-point
uint16_t float2fixed(const float input, const uint8_t fractional_bits)
{
    return (uint16_t)(roundf(input * (1U << fractional_bits)));
}

/*
  calculate turn rate in deg/sec given a bank angle and airspeed for a
  fixed wing aircraft
 */
float fixedwing_turn_rate(float bank_angle_deg, float airspeed)
{
    bank_angle_deg = constrain_float(bank_angle_deg, -80, 80);
    return degrees(GRAVITY_MSS*tanf(radians(bank_angle_deg))/MAX(airspeed,1));
}

// convert degrees farenheight to Kelvin
float degF_to_Kelvin(float temp_f)
{
    return (temp_f + 459.67) * 0.55556;
}

/*
  conversion functions to prevent undefined behaviour
 */
int16_t float_to_int16(const float v)
{
    return int16_t(constrain_float(v, INT16_MIN, INT16_MAX));
}

int32_t float_to_int32(const float v)
{
    return int32_t(constrain_float(v, INT32_MIN, INT32_MAX));
}

uint16_t float_to_uint16(const float v)
{
    return uint16_t(constrain_float(v, 0, UINT16_MAX));
}

uint32_t float_to_uint32(const float v)
{
    return uint32_t(constrain_float(v, 0, UINT32_MAX));
}

uint32_t double_to_uint32(const double v)
{
    return uint32_t(constrain_double(v, 0, UINT32_MAX));
}

int32_t double_to_int32(const double v)
{
    return int32_t(constrain_double(v, INT32_MIN, UINT32_MAX));
}


int32_t float_to_int32_le(const float& value)
{
    int32_t out;
    static_assert(sizeof(value) == sizeof(out), "mismatched sizes");

    // Use memcpy because it's the most portable.
    // It might not be the fastest way on all hardware.
    // At least it's defined behavior in both c and c++.
    memcpy(&out, &value, sizeof(out));
    return out;
}

float int32_to_float_le(const uint32_t& value)
{
    float out;
    static_assert(sizeof(value) == sizeof(out), "mismatched sizes");

    // Use memcpy because it's the most portable.
    // It might not be the fastest way on all hardware.
    // At least it's defined behavior in both c and c++.
    memcpy(&out, &value, sizeof(out));
    return out;
}

double uint64_to_double_le(const uint64_t& value)
{
    double out;
    static_assert(sizeof(value) == sizeof(out), "mismatched sizes");

    // Use memcpy because it's the most portable.
    // It might not be the fastest way on all hardware.
    // At least it's defined behavior in both c and c++.
    memcpy(&out, &value, sizeof(out));
    return out;
}

/*
  get a twos-complement value from the first 'length' bits of a uint32_t
  With thanks to betaflight
 */
int32_t get_twos_complement(uint32_t raw, uint8_t length)
{
    if (raw & ((int)1 << (length - 1))) {
        return ((int32_t)raw) - ((int32_t)1 << length);
    }
    return raw;
}
