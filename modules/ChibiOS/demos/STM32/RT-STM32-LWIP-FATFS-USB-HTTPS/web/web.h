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
 * @file web.h
 * @brief HTTP server wrapper thread macros and structures.
 * @addtogroup WEB_THREAD
 * @{
 */

#ifndef WEB_H
#define WEB_H

#if !defined(WEB_THREAD_STACK_SIZE)
#define WEB_THREAD_STACK_SIZE           (16 * 1024)
#endif

#if !defined(WEB_THREAD_PORT)
#define WEB_THREAD_PORT                 443
#endif

#if !defined(WEB_THREAD_PRIORITY)
#define WEB_THREAD_PRIORITY             (LOWPRIO + 2)
#endif

#if !defined(WEB_MAX_PATH_SIZE)
#define WEB_MAX_PATH_SIZE               128
#endif

extern THD_WORKING_AREA(wa_https_server, WEB_THREAD_STACK_SIZE);

#ifdef __cplusplus
extern "C" {
#endif
  THD_FUNCTION(https_server, p);
#ifdef __cplusplus
}
#endif

#endif /* WEB_H */

/** @} */
