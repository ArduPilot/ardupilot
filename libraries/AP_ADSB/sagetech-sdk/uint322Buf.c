/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file uint322Buf.c
 * @author Jacob.Garrison
 *
 * @date Mar 2, 2021
 *
 */

#include "sgUtil.h"

/*
 * Documented in the header file.
 */
void uint322Buf(uint8_t *bufferPos, uint32_t value)
{
    bufferPos[0] = (value & 0xFF000000) >> 24;
    bufferPos[1] = (value & 0x00FF0000) >> 16;
    bufferPos[2] = (value & 0x0000FF00) >> 8;
    bufferPos[3] = (value & 0x000000FF);
}
