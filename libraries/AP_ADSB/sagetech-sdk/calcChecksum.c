/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file calcChecksum.c
 * @author Jacob.Garrison
 *
 * @date Mar 2, 2021
 *
 */

#include "sgUtil.h"

/*
 * Documented in the header file
 */
uint8_t calcChecksum(uint8_t *buffer, uint8_t len)
{
    uint8_t sum = 0x00;

    // Add all bytes excluding checksum
    for (uint8_t i = 0; i < len - 1; ++i)
    {
        sum += buffer[i];
    }

    return sum;
}
