#pragma once

/*
  common controller helper functions
 */

// Proportional controller with piecewise sqrt sections to constrain second derivative
float sqrt_controller(float error, float p, float second_ord_lim, float dt);

// limit vector to a given length, returns true if vector was limited
bool limit_vector_length(float &vector_x, float &vector_y, float max_length);
