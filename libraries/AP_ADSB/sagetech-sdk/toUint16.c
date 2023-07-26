/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file toUint16.c
 * @author Jacob.Garrison
 *
 * @date Mar 9, 2021
 *      
 */

#include "sgUtil.h"

/*
 * Documented in the header file.
 */
uint16_t toUint16(const uint8_t bytes[])
{
   uint16_t uint16 = ((uint16_t) bytes[0] << 8 | (uint16_t) bytes[1]);

   return uint16;
}
