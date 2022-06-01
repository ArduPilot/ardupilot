/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file toGS.c
 * @author Jacob.Garrison
 *
 * @date Mar 2, 2021
 *
 */

#include "sgUtil.h"

/*
 * Documented in the header file.
 */
uint8_t toGS(const uint8_t bytes[])
{
    uint8_t code = bytes[0];
    float gs = 0.0f;

    if (code <= 0x01)
    {
        gs = 0.0f;
    }
    else if (code <= 0x08)
    {
        gs = 1.0f;
    }
    else if (code <= 0x0C)
    {
        gs = 1.0f + (code - 0x09) * 0.25f;
    }
    else if (code <= 0x26)
    {
        gs = 2.0f + (code - 0x0D) * 0.5f;
    }
    else if (code <= 0x5D)
    {
        gs = 15.0f + (code - 0x27) * 1.0f;
    }
    else if (code <= 0x6C)
    {
        gs = 70.0f + (code - 0x5E) * 2.0f;
    }
    else if (code <= 0x7B)
    {
        gs = 100.0f + (code - 0x6D) * 5.0f;
    }
    else
    {
        gs = 176.0f;
    }

    // first converting to an 16 bit integer is necessary
    // to keep the floating point conversion from
    // truncating to 0.
    return (uint8_t)((int16_t)gs & 0xFF);
}
