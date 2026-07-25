/*
    ChibiOS - Copyright (C) 2006..2020 Rocco Marco Guglielmi

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
 * @file    ADUCM36x/ADUCM_isr.h
 * @brief   ADUCM36x capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef ADUCM_REGISTRY_H
#define ADUCM_REGISTRY_H

#if !defined(ADUCM36x) || defined(__DOXYGEN__)
#define ADUCM36x
#endif

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/**
 * @name    ADUCM360 capabilities
 * @{
 */
/*===========================================================================*/
/* ADUCM360.                                                                 */
/*===========================================================================*/

#if defined(ADUCM360) || defined(__DOXYGEN__)

/* CLK attributes.*/
#define ADUCM_HAS_CLK_AUTOGATE              FALSE

/* UART attributes.*/
#define ADUCM_HAS_UART0                     TRUE
#define ADUCM_HAS_UART1                     FALSE

#define ADUCM_UART_STEPPING                 0

#else
#error "ADUCM36x device not specified"
#endif

/** @} */

#endif /* ADUCM_REGISTRY_H */

/** @} */
