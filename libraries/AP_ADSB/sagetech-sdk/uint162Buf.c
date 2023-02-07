/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file uint162Buf.c
 * @author Jacob.Garrison
 *
 * @date Mar 2, 2021
 *
 */

#include "sgUtil.h"

/*
 * Documented in the header file.
 */
void uint162Buf(uint8_t *bufferPos, uint16_t value)
{
    bufferPos[0] = value >> 8;
    bufferPos[1] = value & 0xFF;
}
