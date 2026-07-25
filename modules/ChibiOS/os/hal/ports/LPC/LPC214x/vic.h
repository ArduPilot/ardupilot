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
 * @file    LPC214x/vic.h
 * @brief   LPC214x VIC peripheral support header.
 *
 * @addtogroup LPC214x_VIC
 * @{
 */

#ifndef VIC_H
#define VIC_H

#ifdef __cplusplus
extern "C" {
#endif
  void vic_init(void);
  void SetVICVector(void *handler, int vector, int source);
#ifdef __cplusplus
}
#endif

#endif /* VIC_H */

/** @} */
