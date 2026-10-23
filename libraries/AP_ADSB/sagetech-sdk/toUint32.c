/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file toUint32.c
 * @author Jacob.Garrison
 *
 * @date Mar 2, 2021
 *      
 */

#include "sgUtil.h"

/*
 * Documented in the header file.
 */
uint32_t toUint32(const uint8_t bytes[])
{
   uint32_t uint32 = ((uint32_t) bytes[0] << 24) | ((uint32_t) bytes[1] << 16) | ((uint32_t) bytes[2] << 8) | ((uint32_t) bytes[3]);

   return uint32;
}
