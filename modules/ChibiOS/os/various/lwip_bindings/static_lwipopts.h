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
 * @file    static_lwipopts.h
 *
 * @addtogroup static_lwipopts
 * @{
 */

#ifndef STATIC_LWIPOPTS_H
#define STATIC_LWIPOPTS_H

#define NO_SYS                          0

#define LWIP_TIMERS                     1
#define LWIP_TIMERS_CUSTOM              0

#define LWIP_TCPIP_CORE_LOCKING         1
#define LWIP_TCPIP_CORE_LOCKING_INPUT   0
#define LWIP_COMPAT_MUTEX_ALLOWED

#define SYS_LIGHTWEIGHT_PROT            1

#define MEM_ALIGNMENT                   4

#endif  /* STATIC_LWIPOPTS_H */

/** @} */
