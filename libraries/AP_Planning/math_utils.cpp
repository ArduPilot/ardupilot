#include "math_utils.h"

#include <utility>

namespace planning {

float Sqr(const float x) { return x * x; }

float CrossProd(const Vec2d& start_point, const Vec2d& end_point_1,
                 const Vec2d& end_point_2) {
  return (end_point_1 - start_point).CrossProd(end_point_2 - start_point);
}

float InnerProd(const Vec2d& start_point, const Vec2d& end_point_1,
                 const Vec2d& end_point_2) {
  return (end_point_1 - start_point).InnerProd(end_point_2 - start_point);
}

float CrossProd(const float x0, const float y0, const float x1,
                 const float y1) {
  return x0 * y1 - x1 * y0;
}

float InnerProd(const float x0, const float y0, const float x1,
                 const float y1) {
  return x0 * x1 + y0 * y1;
}

float WrapAngle(const float angle) {
  const float new_angle = std::fmod(angle, M_PI * 2.0);
  return new_angle < 0 ? new_angle + M_PI * 2.0 : new_angle;
}

float NormalizeAngle(const float angle) {
  float a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

float AngleDiff(const float from, const float to) {
  return NormalizeAngle(to - from);
}

int RandomInt(const int s, const int t, unsigned int rand_seed) {
  if (s >= t) {
    return s;
  }
  return s + rand_r(&rand_seed) % (t - s + 1);
}

float RandomDouble(const float s, const float t, unsigned int rand_seed) {
  return s + (t - s) / 16383.0 * (rand_r(&rand_seed) & 16383);
}

// Gaussian
float Gaussian(const float u, const float std, const float x) {
  return (1.0 / std::sqrt(2 * M_PI * std * std)) *
         std::exp(-(x - u) * (x - u) / (2 * std * std));
}

// Sigmoid
float Sigmoid(const float x) { return 1.0 / (1.0 + std::exp(-x)); }

}  // namespace planning
