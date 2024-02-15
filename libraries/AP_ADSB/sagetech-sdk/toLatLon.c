/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file toLatLon.c
 * @author Jacob.Garrison
 *
 * @date Mar 2, 2021
 *
 */

#include "sgUtil.h"

#define SV_RES_LATLON 180.0 / 8388608.0 // 180 degrees / 2^23

/*
 * Documented in the header file.
 */
double toLatLon(const uint8_t bytes[])
{
    double value = toInt32(bytes);
    value *= SV_RES_LATLON;

    return value;
}
