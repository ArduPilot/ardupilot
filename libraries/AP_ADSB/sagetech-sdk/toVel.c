/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file toVel.c
 * @author Jacob.Garrison
 *
 * @date Mar 2, 2021
 *
 */

#include "sgUtil.h"

#define SV_RES_VEL 0.125

/*
 * Documented in the header file.
 */
double toVel(const uint8_t bytes[])
{
    double value = toInt16(bytes);
    value *= SV_RES_VEL;

    return value;
}
