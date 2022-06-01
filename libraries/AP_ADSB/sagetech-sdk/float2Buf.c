/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file float2Buf.c
 * @author Jacob.Garrison
 *
 * @date Mar 2, 2021
 *
 */

#include "sgUtil.h"

/*
 * Documented in the header file.
 */
void float2Buf(uint8_t *bufferPos, float value)
{
    const uint16_t FLOAT_SIZE = 4;

    union
    {
        float val;
        unsigned char bytes[FLOAT_SIZE];
    } conversion;

    conversion.val = value;

    for (int i = 0; i < FLOAT_SIZE; ++i)
    {
        bufferPos[i] = conversion.bytes[i];
    }
}
