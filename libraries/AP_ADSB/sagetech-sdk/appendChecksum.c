/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file appendChecksum.c
 * @author Jacob.Garrison
 *
 * @date Mar 2, 2021
 *
 */

#include "sgUtil.h"

/*
 * Documented in the header file
 */
void appendChecksum(uint8_t *buffer, uint8_t len)
{
    buffer[len - 1] = calcChecksum(buffer, len);
}
