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
 * @file    SPC56ECxx/ppcparams.h
 * @brief   PowerPC parameters for the SPC56ECxx.
 *
 * @defgroup PPC_SPC56ECxx SPC56ECxx Specific Parameters
 * @ingroup PPC_SPECIFIC
 * @details This file contains the PowerPC specific parameters for the
 *          SPC56ECxx platform.
 * @{
 */

#ifndef PPCPARAMS_H
#define PPCPARAMS_H

/**
 * @brief   Family identification macro.
 */
#define PPC_SPC56ECxx

/**
 * @brief   PPC core model.
 */
#define PPC_VARIANT                 PPC_VARIANT_e200z4

/**
 * @brief   Number of cores.
 */
#define PPC_CORE_NUMBER             1

/**
 * @brief   Number of writable bits in IVPR register.
 */
#define PPC_IVPR_BITS               16

/**
 * @brief   IVORx registers support.
 */
#define PPC_SUPPORTS_IVORS          TRUE

/**
 * @brief   Book E instruction set support.
 */
#define PPC_SUPPORTS_BOOKE          TRUE

/**
 * @brief   VLE instruction set support.
 */
#define PPC_SUPPORTS_VLE            TRUE

/**
 * @brief   Supports VLS Load/Store Multiple Volatile instructions.
 */
#define PPC_SUPPORTS_VLE_MULTI      TRUE

/**
 * @brief   Supports the decrementer timer.
 */
#define PPC_SUPPORTS_DECREMENTER    TRUE

/**
 * @brief   Number of interrupt sources.
 */
#define PPC_NUM_VECTORS             279

#endif /* PPCPARAMS_H */

/** @} */
