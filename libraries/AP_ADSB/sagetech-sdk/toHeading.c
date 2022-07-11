/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file toHeading.c
 * @author Jacob.Garrison
 *
 * @date Mar 2, 2021
 *
 */

#include "sgUtil.h"

#define SV_RES_HEAD 1.40625

/*
 * Documented in the header file.
 */
double toHeading(const uint8_t bytes[])
{
    double value = bytes[0];
    value *= SV_RES_HEAD;

    return value;
}
