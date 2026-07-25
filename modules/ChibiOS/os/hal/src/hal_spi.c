/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    hal_spi.c
 * @brief   SPI Driver selector code.
 *
 * @addtogroup SPI
 * @{
 */

#include "hal.h"

#if (HAL_USE_SPI == TRUE) || defined(__DOXYGEN__)

#if defined(HAL_LLD_SELECT_SPI_V2) || defined(__DOXYGEN__)
#include "hal_spi_v2.inc"
#endif
#if !defined(HAL_LLD_SELECT_SPI_V2) || defined(__DOXYGEN__)
#include "hal_spi_v1.inc"
#endif

#endif /* HAL_USE_SPI == TRUE */

/** @} */
