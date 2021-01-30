/*
  polynomial fitting class, originally written by Siddharth Bharat Purohit
  re-written for ArduPilot by Andrew Tridgell

  This fits a polynomial of a given order to a set of data, running as
  an online algorithm with minimal storage
*/

#pragma once

#include <stdint.h>

/*
  polynomial fit with X axis type xtype and yaxis type vtype (must be a vector)
 */
template <uint8_t order, typename xtype, typename vtype>
class PolyFit
{
public:
    void update(xtype x, vtype y);
    bool get_polynomial(vtype res[order]) const;

private:
    xtype mat[order][order];
    vtype vec[order];
};

