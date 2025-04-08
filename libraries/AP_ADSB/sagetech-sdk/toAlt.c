/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file toAlt.c
 * @author Jacob.Garrison
 *
 * @date Mar 2, 2021
 *
 */

#include "sgUtil.h"

#define SV_RES_ALT 0.015625

/*
 * Documented in the header file.
 */
double toAlt(const uint8_t bytes[])
{
    double value = toInt32(bytes);
    value *= SV_RES_ALT;

    return value;
}
