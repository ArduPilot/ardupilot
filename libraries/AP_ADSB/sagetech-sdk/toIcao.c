/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file toIcao.c
 * @author Jacob.Garrison
 *
 * @date Mar 9, 2021
 *
 */

#include "sgUtil.h"

/*
 * Documented in the header file.
 */
uint32_t toIcao(const uint8_t bytes[])
{
    uint32_t icao = (0 << 24) | ((uint32_t)bytes[0] << 16) | ((uint32_t)bytes[1] << 8) | ((uint32_t)bytes[2]);

    return icao;
}
