/*
  polynomial fitting class, originally written by Siddharth Bharat Purohit
  re-written for ArduPilot by Andrew Tridgell

  This fits a polynomial of a given order to a set of data, running as
  an online algorithm with minimal storage
*/

#pragma once

#include <stdint.h>

template <uint8_t order>
class PolyFit {
public:
    void update(float x, float y);
    bool get_polynomial(float res[order]) const;

private:
    float mat[order][order];
    float vec[order];
};

