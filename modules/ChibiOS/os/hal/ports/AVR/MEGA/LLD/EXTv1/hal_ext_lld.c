/*
  ChibiOS - Copyright (C) 2016 Theodore Ateba

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
 * @file  EXTv1/hal_ext_lld.c
 * @brief AVR/MEGA EXT subsystem low level driver source.
 *
 * @addtogroup EXT
 * @{
 */

#include "hal.h"

#if HAL_USE_EXT || defined(__DOXYGEN__)

/*==========================================================================*/
/* Driver local definitions.                                                */
/*==========================================================================*/

/*==========================================================================*/
/* Driver exported variables.                                               */
/*==========================================================================*/

/**
 * @brief EXTD1 driver identifier.
 */
EXTDriver EXTD1;

/*==========================================================================*/
/* Driver local variables and types.                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Driver local functions.                                                  */
/*==========================================================================*/

/**
 * @brief   Set the INTx interrupt trigger front or state.
 *
 * @param[in] channel   the channel to configure
 * @param[in] edge      the front or state to configure
 */
void ext_lld_set_intx_edges(expchannel_t channel, uint8_t edge) {

#if AVR_EXT_USE_INT0 || defined(__DOXYGEN__)
  if (channel == INT0) {
    if (edge == EXT_CH_MODE_RISING_EDGE) {
      EICRA |= (1 << 0);
      EICRA |= (1 << 1);
    } else if (edge == EXT_CH_MODE_FALLING_EDGE) {
      EICRA &= ~(1 << 0);
      EICRA |= (1 << 1);
    } else if (edge == EXT_CH_MODE_BOTH_EDGES) {
      EICRA |= (1 << 0);
      EICRA &= ~(1 << 1);
    } else {
      EICRA &= ~(1 << 0);
      EICRA &= ~(1 << 1);
    }
  }
#endif
#if AVR_EXT_USE_INT1 || defined(__DOXYGEN__)
  if (channel == INT1) {
    if (edge == EXT_CH_MODE_RISING_EDGE) {
      EICRA |= (1 << 2);
      EICRA |= (1 << 3);
    } else if (edge == EXT_CH_MODE_FALLING_EDGE) {
      EICRA &= ~(1 << 2);
      EICRA |= (1 << 3);
    } else if (edge == EXT_CH_MODE_BOTH_EDGES) {
      EICRA |= (1 << 2);
      EICRA &= ~(1 << 3);
    } else {
      EICRA &= ~(1 << 2);
      EICRA &= ~(1 << 3);
    }
  }
#endif
#if AVR_EXT_USE_INT2 || defined(__DOXYGEN__)
  if (channel == INT2) {
    if (edge == EXT_CH_MODE_RISING_EDGE) {
      EICRA |= (1 << 4);
      EICRA |= (1 << 5);
    } else if (edge == EXT_CH_MODE_FALLING_EDGE) {
      EICRA &= ~(1 << 4);
      EICRA |= (1 << 5);
    } else if (edge == EXT_CH_MODE_BOTH_EDGES) {
      EICRA |= (1 << 4);
      EICRA &= ~(1 << 5);
    } else {
      EICRA &= ~(1 << 4);
      EICRA &= ~(1 << 5);
    }
  }
#endif
#if AVR_EXT_USE_INT3 || defined(__DOXYGEN__)
  if (channel == INT3) {
    if (edge == EXT_CH_MODE_RISING_EDGE) {
      EICRA |= (1 << 6);
      EICRA |= (1 << 7);
    } else if (edge == EXT_CH_MODE_FALLING_EDGE) {
      EICRA &= ~(1 << 6);
      EICRA |= (1 << 7);
    } else if (edge == EXT_CH_MODE_BOTH_EDGES) {
      EICRA |= (1 << 6);
      EICRA &= ~(1 << 7);
    } else {
      EICRA &= ~(1 << 6);
      EICRA &= ~(1 << 7);
    }
  }
#endif
#if AVR_EXT_USE_INT4 || defined(__DOXYGEN__)
  if (channel == INT4) {
    if (edge == EXT_CH_MODE_RISING_EDGE) {
      EICRB |= (1 << 0);
      EICRB |= (1 << 1);
    } else if (edge == EXT_CH_MODE_FALLING_EDGE) {
      EICRB &= ~(1 << 0);
      EICRB |= (1 << 1);
    } else if (edge == EXT_CH_MODE_BOTH_EDGES) {
      EICRB |= (1 << 0);
      EICRB &= ~(1 << 1);
    } else {
      EICRB &= ~(1 << 0);
      EICRB &= ~(1 << 1);
    }
  }
#endif
#if AVR_EXT_USE_INT5 || defined(__DOXYGEN__)
  if (channel == INT5) {
    if (edge == EXT_CH_MODE_RISING_EDGE) {
      EICRB |= (1 << 2);
      EICRB |= (1 << 3);
    } else if (edge == EXT_CH_MODE_FALLING_EDGE) {
      EICRB &= ~(1 << 2);
      EICRB |= (1 << 3);
    } else if (edge == EXT_CH_MODE_BOTH_EDGES) {
      EICRB |= (1 << 2);
      EICRB &= ~(1 << 3);
    } else {
      EICRB &= ~(1 << 2);
      EICRB &= ~(1 << 3);
    }
  }
#endif
}

/*==========================================================================*/
/* Driver interrupt handlers.                                               */
/*==========================================================================*/

#if AVR_EXT_USE_INT0 || defined(__DOXYGEN__)
/**
 * @brief EXTI[INT0] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(INT0_vect) {
  OSAL_IRQ_PROLOGUE();
  EXTD1.config->channels[INT0].cb(&EXTD1, INT0);
  OSAL_IRQ_EPILOGUE();
}
#endif
#if AVR_EXT_USE_INT1 || defined(__DOXYGEN__)
/**
 * @brief EXTI[INT1] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(INT1_vect) {
  OSAL_IRQ_PROLOGUE();
  EXTD1.config->channels[INT1].cb(&EXTD1, INT1);
  OSAL_IRQ_EPILOGUE();
}
#endif
#if AVR_EXT_USE_INT2 || defined(__DOXYGEN__)
/**
 * @brief EXTI[INT2] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(INT2_vect) {
  OSAL_IRQ_PROLOGUE();
  EXTD1.config->channels[INT2].cb(&EXTD1, INT2);
  OSAL_IRQ_EPILOGUE();
}
#endif
#if AVR_EXT_USE_INT3 || defined(__DOXYGEN__)
/**
 * @brief EXTI[INT3] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(INT3_vect) {
  OSAL_IRQ_PROLOGUE();
  EXTD1.config->channels[INT3].cb(&EXTD1, INT3);
  OSAL_IRQ_EPILOGUE();
}
#endif
#if AVR_EXT_USE_INT4 || defined(__DOXYGEN__)
/**
 * @brief EXTI[INT4] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(INT4_vect) {
  OSAL_IRQ_PROLOGUE();
  EXTD1.config->channels[INT4].cb(&EXTD1, INT4);
  OSAL_IRQ_EPILOGUE();
}
#endif
#if AVR_EXT_USE_INT5 || defined(__DOXYGEN__)
/**
 * @brief EXTI[INT5] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(INT5_vect) {
  OSAL_IRQ_PROLOGUE();
  EXTD1.config->channels[INT5].cb(&EXTD1, INT5);
  OSAL_IRQ_EPILOGUE();
}
#endif

/*==========================================================================*/
/* Driver functions.                                                        */
/*==========================================================================*/

/**
 * @brief   Enables an EXT channel.
 *
 * @param[in] extp      pointer to the @p EXTDriver object
 * @param[in] channel   channel to be enabled
 *
 * @notapi
 */
void ext_lld_channel_enable(EXTDriver *extp, expchannel_t channel) {

#if AVR_EXT_USE_INT0 || defined(__DOXYGEN__)
  if (channel == INT0) {
    EIMSK |= 1 << INT0;
    ext_lld_set_intx_edges(channel, extp->config->channels[channel].mode);
  }
#endif
#if AVR_EXT_USE_INT1 || defined(__DOXYGEN__)
  if (channel == INT1) {
    EIMSK |= 1 << INT1;
    ext_lld_set_intx_edges(channel, extp->config->channels[channel].mode);
  }
#endif
#if AVR_EXT_USE_INT2 || defined(__DOXYGEN__)
  if (channel == INT2) {
    EIMSK |= (1 << INT2);
    ext_lld_set_intx_edges(channel, extp->config->channels[channel].mode);
  }
#endif
#if AVR_EXT_USE_INT3 || defined(__DOXYGEN__)
  if (channel == INT3) {
    EIMSK |= (1 << INT3);
    ext_lld_set_intx_edges(channel, extp->config->channels[channel].mode);
  }
#endif
#if AVR_EXT_USE_INT4 || defined(__DOXYGEN__)
  if (channel == INT4) {
    EIMSK |= 1 << INT4;
    ext_lld_set_intx_edges(channel, extp->config->channels[channel].mode);
  }
#endif
#if AVR_EXT_USE_INT5 || defined(__DOXYGEN__)
  if (channel == INT5) {
    EIMSK |= 1 << INT5;
    ext_lld_set_intx_edges(channel, extp->config->channels[channel].mode);
  }
#endif
}

/**
 * @brief   Disables an EXT channel.
 *
 * @param[in] extp      pinter to the @p EXTDriver object
 * @param[in] channel   channel to be disabled
 *
 * @notapi
 */
void ext_lld_channel_disable(EXTDriver *extp, expchannel_t channel) {

#if AVR_EXT_USE_INT0 || defined(__DOXYGEN__)
  if (channel == INT0)
    EIMSK &= ~(1 << INT0);
#endif
#if AVR_EXT_USE_INT1 || defined(__DOXYGEN__)
  if (channel == INT1)
    EIMSK &= ~(1 << INT1);
#endif
#if AVR_EXT_USE_INT2 || defined(__DOXYGEN__)
  if (channel == INT2)
    EIMSK &= ~(1 << INT2);
#endif
#if AVR_EXT_USE_INT3 || defined(__DOXYGEN__)
  if (channel == INT3)
    EIMSK &= ~(1 << INT3);
#endif
#if AVR_EXT_USE_INT4 || defined(__DOXYGEN__)
  if (channel == INT4)
    EIMSK &= ~(1 << INT4);
#endif
#if AVR_EXT_USE_INT5 || defined(__DOXYGEN__)
  if (channel == INT5)
    EIMSK &= ~(1 << INT5);
#endif
}

/**
 * @brief   Low level EXT driver initialization.
 *
 * @notapi
 */
void ext_lld_init(void) {

  /* Driver initialization. */
  extObjectInit(&EXTD1);
}

/**
 * @brief   Configures and activates the EXT peripheral.
 *
 * @param[in] extp  pointer to the @p EXTDriver object
 *
 * @notapi
 */
void ext_lld_start(EXTDriver *extp) {

  expchannel_t line;

  if (extp->state == EXT_STOP)
    osalSysUnlock();

  /* Configuration of automatic channels. */
  for (line = 0; line < EXT_MAX_CHANNELS; line++) {
    if (extp->config->channels[line].mode & EXT_CH_MODE_AUTOSTART)
      ext_lld_channel_enable(extp, line);
    else
      ext_lld_channel_disable(extp, line);
  }
}

/**
 * @brief   Deactivates the EXT peripheral.
 *
 * @param[in] extp  pointer to the @p EXTDriver object
 *
 * @notapi
 */
void ext_lld_stop(EXTDriver *extp) {

  if (extp->state == EXT_ACTIVE)
    osalSysLock();
}

#endif /* HAL_USE_EXT */

/** @} */
