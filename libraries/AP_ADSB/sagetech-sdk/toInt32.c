/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file toInt32.c
 * @author Jacob.Garrison
 *
 * @date Mar 2, 2021
 *
 */

#include "sgUtil.h"

/*
 * Documented in the header file.
 */
int32_t toInt32(const uint8_t bytes[])
{
    int32_t int32 = ((int32_t)bytes[0] << 24) | ((int32_t)bytes[1] << 16) | ((int32_t)bytes[2] << 8);
    int32 = int32 >> 8;

    return int32;
}
