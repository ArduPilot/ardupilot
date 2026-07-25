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
 * @file    MAX32625/max32_registry.h
 * @brief   MAX32625 capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef MAX32_REGISTRY_H
#define MAX32_REGISTRY_H

#if !defined(MAX32625) || defined(__DOXYGEN__)
#define MAX32625
#endif

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/**
 * @name    MAX32625 capabilities
 * @{
 */
/*===========================================================================*/
/* MAX32625.                                                                 */
/*===========================================================================*/

#if defined(MAX32625) || defined(__DOXYGEN__)

#else
#error "MAX32 device not specified"
#endif

/** @} */

#endif /* MAX32_REGISTRY_H */

/** @} */
