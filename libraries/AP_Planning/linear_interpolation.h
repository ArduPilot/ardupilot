/**
 * @file
 * @brief Linear interpolation functions.
 */

#pragma once

#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>
#include <array>

namespace planning {

/**
 * @brief Linear interpolation between two points of type T.
 * @param x0 The coordinate of the first point.
 * @param t0 The interpolation parameter of the first point.
 * @param x1 The coordinate of the second point.
 * @param t1 The interpolation parameter of the second point.
 * @param t The interpolation parameter for interpolation.
 * @param x The coordinate of the interpolated point.
 * @return Interpolated point.
 */
template <typename T>
T lerp(const T& x0, const float t0, const T& x1, const float t1,
       const float t) {
  if (std::abs(t1 - t0) <= 1.0e-6) {
    return x0;
  }
  const float r = (t - t0) / (t1 - t0);
  const T x = x0 + r * (x1 - x0);
  return x;
}

/**
 * @brief Spherical linear interpolation between two angles.
 *        The two angles are within range [-M_PI, M_PI).
 * @param a0 The value of the first angle.
 * @param t0 The interpolation parameter of the first angle.
 * @param a1 The value of the second angle.
 * @param t1 The interpolation parameter of the second angle.
 * @param t The interpolation parameter for interpolation.
 * @param a The value of the spherically interpolated angle.
 * @return Interpolated angle.
 */
float slerp(const float a0, const float t0, const float a1, const float t1,
             const float t);


template<int N>
inline std::array<float, N> LinSpaced(float start, float end) {
  std::array<float, N> res;
  float step = (end - start) / (N - 1);

  for(int i = 0; i < N; i++) {
    res[i] = start + step * i;
  }

  return res;
}

inline std::vector<float> LinSpaced(float start, float end, int count) {
  std::vector<float> res(count, 0);
  float step = (end - start) / (count - 1);

  for(int i = 0; i < count; i++) {
    res[i] = start + step * i;
  }

  return res;
}

}  // namespace planning

