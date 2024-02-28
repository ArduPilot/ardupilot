/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file charArray2Buf.c
 * @author Jacob.Garrison
 *
 * @date Mar 2, 2021
 *
 */

#include "sgUtil.h"
#include <ctype.h>

/*
 * Documented in the header file.
 */
void charArray2Buf(uint8_t *bufferPos, char arr[], uint8_t len)
{
    for (uint8_t i = 0; i < len; ++i)
    {
        bufferPos[i] = toupper(arr[i]);
    }
}
