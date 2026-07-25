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
 * @file    LPC214x/vic.c
 * @brief   LPC214x VIC peripheral support code.
 *
 * @addtogroup LPC214x_VIC
 * @{
 */

#include "hal.h"

/**
 * @brief   VIC Initialization.
 * @note    Better reset everything in the VIC, it is a HUGE source of trouble.
 *
 * @notapi
 */
void vic_init(void) {
  int i;

  VIC *vic = VICBase;
  vic->VIC_IntSelect = 0;               /* All sources assigned to IRQ. */
  vic->VIC_SoftIntClear = ALLINTMASK;   /* No interrupts enforced */
  vic->VIC_IntEnClear = ALLINTMASK;     /* All sources disabled. */
  for (i = 0; i < 16; i++) {
    vic->VIC_VectCntls[i] = 0;
    vic->VIC_VectAddrs[i] = 0;
    vic->VIC_VectAddr = 0;
  }
}

/**
 * @brief   Initializes a VIC vector.
 * @details Set a vector for an interrupt source and enables it.
 *
 * @param[in] handler   the pointer to the IRQ service routine
 * @param[in] vector    the vector number
 * @param[in] source    the IRQ source to be associated to the vector
 *
 * @api
 */
void SetVICVector(void *handler, int vector, int source) {

  VIC *vicp = VICBase;
  vicp->VIC_VectAddrs[vector] = (IOREG32)handler;
  vicp->VIC_VectCntls[vector] = (IOREG32)(source | 0x20);
}

/** @} */
