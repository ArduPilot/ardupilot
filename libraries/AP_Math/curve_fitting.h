#pragma once

#include <deque>

/*
  common controller helper functions
 */

// The coef is in ascending order,
// i.e., f(x) = coef[0] + coef[1] * x + coef[2] * x^2 ...
float evaluate_polynomial(const std::deque<float> &coef, const float p);

// y = a * x + b
std::deque<float> least_square(const std::deque<float> &points, const float dt, float *ptr_error_square = nullptr);
