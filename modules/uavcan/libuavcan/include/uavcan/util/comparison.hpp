/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_UTIL_COMPARISON_HPP_INCLUDED
#define UAVCAN_UTIL_COMPARISON_HPP_INCLUDED

#include <uavcan/util/templates.hpp>
#include <uavcan/build_config.hpp>

namespace uavcan
{
/**
 * Exact comparison of two floats that suppresses the compiler warnings.
 */
template <typename T>
UAVCAN_EXPORT
inline bool areFloatsExactlyEqual(const T& left, const T& right)
{
    return (left <= right) && (left >= right);
}

/**
 * This function performs fuzzy comparison of two floating point numbers.
 * Type of T can be either float, double or long double.
 * For details refer to http://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
 * See also: @ref UAVCAN_FLOAT_COMPARISON_EPSILON_MULT.
 */
template <typename T>
UAVCAN_EXPORT
inline bool areFloatsClose(T a, T b, const T& absolute_epsilon, const T& relative_epsilon)
{
    // NAN
    if (isNaN(a) || isNaN(b))
    {
        return false;
    }

    // Infinity
    if (isInfinity(a) || isInfinity(b))
    {
        return areFloatsExactlyEqual(a, b);
    }

    // Close numbers near zero
    const T diff = std::fabs(a - b);
    if (diff <= absolute_epsilon)
    {
        return true;
    }

    // General case
    a = std::fabs(a);
    b = std::fabs(b);
    const T largest = (b > a) ? b : a;
    return (diff <= largest * relative_epsilon);
}

/**
 * This namespace contains implementation details for areClose().
 * Don't try this at home.
 */
namespace are_close_impl_
{

struct Applicable { char foo[1]; };
struct NotApplicable { long long foo[16]; };

template <typename This, typename Rhs>
struct HasIsCloseMethod
{
    template <typename U, typename R, bool (U::*)(const R&) const> struct ConstRef { };
    template <typename U, typename R, bool (U::*)(R) const> struct ByValue { };

    template <typename U, typename R> static Applicable test(ConstRef<U, R, &U::isClose>*);
    template <typename U, typename R> static Applicable test(ByValue<U, R, &U::isClose>*);

    template <typename U, typename R> static NotApplicable test(...);

    enum { Result = sizeof(test<This, Rhs>(UAVCAN_NULLPTR)) };
};

/// First stage: bool L::isClose(R)
template <typename L, typename R>
UAVCAN_EXPORT
inline bool areCloseImplFirst(const L& left, const R& right, IntToType<sizeof(Applicable)>)
{
    return left.isClose(right);
}

/// Second stage: bool R::isClose(L)
template <typename L, typename R>
UAVCAN_EXPORT
inline bool areCloseImplSecond(const L& left, const R& right, IntToType<sizeof(Applicable)>)
{
    return right.isClose(left);
}

/// Second stage: L == R
template <typename L, typename R>
UAVCAN_EXPORT
inline bool areCloseImplSecond(const L& left, const R& right, IntToType<sizeof(NotApplicable)>)
{
    return left == right;
}

/// First stage: select either L == R or bool R::isClose(L)
template <typename L, typename R>
UAVCAN_EXPORT
inline bool areCloseImplFirst(const L& left, const R& right, IntToType<sizeof(NotApplicable)>)
{
    return are_close_impl_::areCloseImplSecond(left, right,
                                               IntToType<are_close_impl_::HasIsCloseMethod<R, L>::Result>());
}

} // namespace are_close_impl_

/**
 * Generic fuzzy comparison function.
 *
 * This function properly handles floating point comparison, including mixed floating point type comparison,
 * e.g. float with long double.
 *
 * Two objects of types A and B will be fuzzy comparable if either method is defined:
 *  - bool A::isClose(const B&) const
 *  - bool A::isClose(B) const
 *  - bool B::isClose(const A&) const
 *  - bool B::isClose(A) const
 *
 * Call areClose(A, B) will be dispatched as follows:
 *
 *  - If A and B are both floating point types (float, double, long double) - possibly different - the call will be
 *    dispatched to @ref areFloatsClose(). If A and B are different types, value of the larger type will be coerced
 *    to the smaller type, e.g. areClose(long double, float) --> areClose(float, float).
 *
 *  - If A defines isClose() that accepts B, the call will be dispatched there.
 *
 *  - If B defines isClose() that accepts A, the call will be dispatched there (A/B swapped).
 *
 *  - Last resort is A == B.
 *
 * Alternatively, a custom behavior can be implemented via template specialization.
 *
 * See also: @ref UAVCAN_FLOAT_COMPARISON_EPSILON_MULT.
 *
 * Examples:
 *  areClose(1.0, 1.0F)                                         --> true
 *  areClose(1.0, 1.0F + std::numeric_limits<float>::epsilon()) --> true
 *  areClose(1.0, 1.1)                                          --> false
 *  areClose("123", std::string("123"))                         --> true (using std::string's operator ==)
 *  areClose(inf, inf)                                          --> true
 *  areClose(inf, -inf)                                         --> false
 *  areClose(nan, nan)                                          --> false (any comparison with nan returns false)
 *  areClose(123, "123")                                        --> compilation error: operator == is not defined
 */
template <typename L, typename R>
UAVCAN_EXPORT
inline bool areClose(const L& left, const R& right)
{
    return are_close_impl_::areCloseImplFirst(left, right,
                                              IntToType<are_close_impl_::HasIsCloseMethod<L, R>::Result>());
}

/*
 * Float comparison specializations
 */
template <>
UAVCAN_EXPORT
inline bool areClose<float, float>(const float& left, const float& right)
{
    return areFloatsClose(left, right, NumericTraits<float>::epsilon(),
                          NumericTraits<float>::epsilon() * FloatComparisonEpsilonMult);
}

template <>
UAVCAN_EXPORT
inline bool areClose<double, double>(const double& left, const double& right)
{
    return areFloatsClose(left, right, NumericTraits<double>::epsilon(),
                          NumericTraits<double>::epsilon() * FloatComparisonEpsilonMult);
}

template <>
UAVCAN_EXPORT
inline bool areClose<long double, long double>(const long double& left, const long double& right)
{
    return areFloatsClose(left, right, NumericTraits<long double>::epsilon(),
                          NumericTraits<long double>::epsilon() * FloatComparisonEpsilonMult);
}

/*
 * Mixed floating type comparison - coercing larger type to smaller type
 */
template <>
UAVCAN_EXPORT
inline bool areClose<float, double>(const float& left, const double& right)
{
    return areClose(left, static_cast<float>(right));
}

template <>
UAVCAN_EXPORT
inline bool areClose<double, float>(const double& left, const float& right)
{
    return areClose(static_cast<float>(left), right);
}

template <>
UAVCAN_EXPORT
inline bool areClose<float, long double>(const float& left, const long double& right)
{
    return areClose(left, static_cast<float>(right));
}

template <>
UAVCAN_EXPORT
inline bool areClose<long double, float>(const long double& left, const float& right)
{
    return areClose(static_cast<float>(left), right);
}

template <>
UAVCAN_EXPORT
inline bool areClose<double, long double>(const double& left, const long double& right)
{
    return areClose(left, static_cast<double>(right));
}

template <>
UAVCAN_EXPORT
inline bool areClose<long double, double>(const long double& left, const double& right)
{
    return areClose(static_cast<double>(left), right);
}

/**
 * Comparison against zero.
 * Helps to compare a floating point number against zero if the exact type is unknown.
 * For non-floating point types performs exact comparison against integer zero.
 */
template <typename T>
UAVCAN_EXPORT
inline bool isCloseToZero(const T& x)
{
    return x == 0;
}

template <>
UAVCAN_EXPORT
inline bool isCloseToZero<float>(const float& x)
{
    return areClose(x, static_cast<float>(0.0F));
}

template <>
UAVCAN_EXPORT
inline bool isCloseToZero<double>(const double& x)
{
    return areClose(x, static_cast<double>(0.0));
}

template <>
UAVCAN_EXPORT
inline bool isCloseToZero<long double>(const long double& x)
{
    return areClose(x, static_cast<long double>(0.0L));
}

}

#endif // UAVCAN_UTIL_COMPARISON_HPP_INCLUDED
