/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file toInt16.c
 * @author Jacob.Garrison
 *
 * @date Mar 2, 2021
 *
 */

#include "sgUtil.h"

/*
 * Documented in the header file.
 */
int16_t toInt16(const uint8_t bytes[])
{
    int16_t int16 = ((int16_t)bytes[0] << 8) | bytes[1];

    return int16;
}
