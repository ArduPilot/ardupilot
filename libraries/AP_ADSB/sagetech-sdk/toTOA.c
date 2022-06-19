/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file toTOA.c
 * @author Jacob.Garrison
 *
 * @date Mar 2, 2021
 *
 */

#include "sgUtil.h"

/*
 * Documented in the header file.
 */
float toTOA(const uint8_t bytes[])
{
    float toa = toInt16(bytes) & 0xFFFF;
    return toa / 128.0;
}
