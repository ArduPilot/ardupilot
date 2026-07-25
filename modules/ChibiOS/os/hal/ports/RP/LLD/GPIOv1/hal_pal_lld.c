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
 * @file    GPIOv1/hal_pal_lld.c
 * @brief   RP PAL low level driver code.
 *
 * @addtogroup PAL
 * @{
 */

#include "hal.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Event records for the 30 GPIO lines.
 */
palevent_t _pal_events[30];
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (PAL_USE_WAIT || PAL_USE_CALLBACKS) || defined(__DOXYGEN__)
/**
 * @brief   EXTI[0], EXTI[1] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(RP_IO_IRQ_BANK0_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  for(int i=0; i < 4; ++i) {
    int line;
    uint32_t ints = IO_BANK0->PROC[RP_PAL_EVENT_CORE_AFFINITY].INTS[i];

    IO_BANK0->INTR[i] = ints;
    line = i * 8;
    while (ints != 0U) {
      if (ints & 0x0FU) {
        _pal_isr_code(line);
      }
      ints >>= 4;
      ++line;
    };
  };

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   PAL driver initialization.
 *
 * @notapi
 */
void __pal_lld_init(void) {

  hal_lld_peripheral_unreset(RESETS_ALLREG_IO_BANK0);
  hal_lld_peripheral_unreset(RESETS_ALLREG_PADS_BANK0);

  #if PAL_USE_CALLBACKS || PAL_USE_WAIT || defined(__DOXYGEN__)
  unsigned i;

  for (i = 0; i < 30; i++) {
    _pal_init_event(i);
  }

  nvicEnableVector(RP_IO_IRQ_BANK0_NUMBER, RP_IO_IRQ_BANK0_PRIORITY);
  #endif
}

#if (PAL_USE_WAIT || PAL_USE_CALLBACKS) || defined(__DOXYGEN__)

/**
 * @brief   Pad event enable.
 * @note    Programming an unknown or unsupported mode is silently ignored.
 *
 * @param[in] line      line number within the port
 * @param[in] mode      line event mode
 *
 * @notapi
 */
void _pal_lld_enablelineevent(ioline_t line, ioeventmode_t mode) {
  osalDbgAssert(
    !(IO_BANK0->PROC[RP_PAL_EVENT_CORE_AFFINITY].INTE[line/8] & (0x0FU << (4U*(line & 0x07U)))),
    "channel in use");

  uint32_t mode_set = 0;
  uint32_t mode_clr = 0;

  /* Programming the edge and level triggers.*/
  if (mode & PAL_EVENT_MODE_RISING_EDGE)
    mode_set |= 8;
  else
    mode_clr |= 8;

  if (mode & PAL_EVENT_MODE_FALLING_EDGE)
    mode_set |= 4;
  else
    mode_clr |= 4;

  if (mode & RP_PAL_EVENT_MODE_LEVEL_LOW)
    mode_set |= 1;
  else
    mode_clr |= 1;

  if (mode & RP_PAL_EVENT_MODE_LEVEL_HIGH)
    mode_set |= 2;
  else
    mode_clr |= 2;


  IO_BANK0->PROC[RP_PAL_EVENT_CORE_AFFINITY].INTE[line/8] &= 
    ~(mode_clr << (4*(line & 0x07)));
  IO_BANK0->PROC[RP_PAL_EVENT_CORE_AFFINITY].INTE[line/8] |=
    mode_set << (4*(line & 0x07));
};

/**
 * @brief   Pad event disable.
 * @details This function disables previously programmed event callbacks.
 *
 * @param[in] line      line identifier
 *
 * @notapi
 */
void _pal_lld_disablelineevent(ioline_t line) {
  IO_BANK0->PROC[RP_PAL_EVENT_CORE_AFFINITY].INTE[line/8] &=
    ~(0x0FU << (4*(line & 0x07)));
};

/**
 * @brief Force a trigger event on the line.
 * 
 * @param[in]         line identifier
 */
void _pal_lld_forcelineevent(ioline_t line) {
  uint32_t force_mask =
    IO_BANK0->PROC[RP_PAL_EVENT_CORE_AFFINITY].INTE[line/8] &
    (0x0FU << (4*(line & 0x07)));

  IO_BANK0->PROC[RP_PAL_EVENT_CORE_AFFINITY].INTF[line/8] &= ~force_mask;
  IO_BANK0->PROC[RP_PAL_EVENT_CORE_AFFINITY].INTF[line/8] |= force_mask;
};

/**
 * @brief Clear all forced trigger event on the line.
 * 
 * @param[in]         line identifier
 */
void _pal_lld_unforcelineevent(ioline_t line) {
  uint32_t force_mask =
    IO_BANK0->PROC[RP_PAL_EVENT_CORE_AFFINITY].INTE[line/8] &
    (0x0FU << (4*(line & 0x07)));

  IO_BANK0->PROC[RP_PAL_EVENT_CORE_AFFINITY].INTF[line/8] &= ~force_mask;
};

#endif

#endif /* HAL_USE_PAL */

/** @} */
