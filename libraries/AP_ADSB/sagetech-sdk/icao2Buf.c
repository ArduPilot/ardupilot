/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file icao2Buf.c
 * @author Jacob.Garrison
 *
 * @date Mar 2, 2021
 *
 */

#include "sgUtil.h"

/*
 * Documented in the header file.
 */
void icao2Buf(uint8_t *bufferPos, uint32_t icao)
{
    bufferPos[0] = (icao & 0x00FF0000) >> 16;
    bufferPos[1] = (icao & 0x0000FF00) >> 8;
    bufferPos[2] = (icao & 0x000000FF);
}
