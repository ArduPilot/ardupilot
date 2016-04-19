/*
 * This is a 1.5k size hack for PX4, thanks to Lucas for finding this out :)
 */
#include <AP_Math/AP_Math.h>

/*
 * Wrap an angle in centi-degrees
 */
template <class T>
auto wrap_360_cd(const T &angle) -> decltype(wrap_360(angle, 100.f)) {
    return wrap_360(angle, 100.f);
}

template auto  wrap_360_cd<float>   (const float        &angle) -> decltype(wrap_360(angle, 100.f));
template auto  wrap_360_cd<double>  (const double       &angle) -> decltype(wrap_360(angle, 100.f));
template auto  wrap_360_cd<int16_t> (const int16_t      &angle) -> decltype(wrap_360(angle, 100.f));
template auto  wrap_360_cd<int32_t> (const int32_t      &angle) -> decltype(wrap_360(angle, 100.f));

/*
 * Wrap an angle in centi-degrees
 */
template <class T>
auto wrap_180_cd(const T &angle) -> decltype(wrap_180(angle, 100.f)) {
    return wrap_180(angle, 100.f);
} 

template auto  wrap_180_cd<float>   (const float        &angle) -> decltype(wrap_180(angle, 100.f));
template auto  wrap_180_cd<double>  (const double       &angle) -> decltype(wrap_180(angle, 100.f));
template auto  wrap_180_cd<int16_t> (const int16_t      &angle) -> decltype(wrap_180(angle, 100.f));
template auto  wrap_180_cd<int32_t> (const int32_t      &angle) -> decltype(wrap_180(angle, 100.f));