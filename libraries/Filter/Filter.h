#pragma once

/* Umbrella header for the Filter library */

#include "FilterClass.h"
#include "AverageFilter.h"
#include "DerivativeFilter.h"
#include "FilterWithBuffer.h"
#include "LowPassFilter.h"
#include "ModeFilter.h"
#include "Butter.h"

/*
  the filter version is logged in the VER message to assist the online
  analysis tools, so they can display the right filter formulas for
  this version of the code
  This should be incremented on significant filtering changes

  1 - Original filters, version was not logged
  2 - large rework, filters reduce in amplitude as they track under the configured min frequency.
  3 - Quintuple notch added but not functional
  4 - Quintuple notch fixed
 */

#define AP_FILTER_VERSION 4

