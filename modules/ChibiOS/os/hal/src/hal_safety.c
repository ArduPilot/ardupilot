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
 * @file    hal_safety.c
 * @brief   Safety manager code.
 *
 * @addtogroup HAL_SAFETY
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define HAL_US2RTC(freq, usec) (halcnt_t)((((freq) + 999999UL) / 1000000UL) * (usec))

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#if !defined(HAL_LLD_GET_CNT_VALUE)
/* Stub type if the LLD does not implement the timeout functionality.*/
typedef unsigned halcnt_t;
#endif

static inline uint32_t get_frequency(void) {

#if defined(HAL_LLD_GET_CNT_FREQUENCY)
  return HAL_LLD_GET_CNT_FREQUENCY();
#else
  return 1000000U;
#endif
}

static inline halcnt_t get_counter(void) {

#if defined(HAL_LLD_GET_CNT_VALUE)
  return HAL_LLD_GET_CNT_VALUE();
#else
  return (halcnt_t)0;
#endif
}

static inline bool is_counter_within(halcnt_t start, halcnt_t end) {
  halcnt_t cnt = get_counter();

  return (bool)(((halcnt_t)cnt - (halcnt_t)start) <
                ((halcnt_t)end - (halcnt_t)start));
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Common safety fault handler.
 * @brief   This function does not return, any recovery strategy, at this
 *          level, is meant to be destructive.
 *
 * @param[in] message   fault reason string
 *
 * @api
 */
CC_NO_RETURN
void halSftFail(const char *message) {

#if defined(HAL_SAFETY_HANDLER)
    HAL_SAFETY_HANDLER(message);
#else
    osalSysHalt(message);
    while (true) {} /* TODO: Temporary, suppresses a warning.*/
#endif
}

/**
 * @brief   Waits for masked bits to match or a timeout.
 * @note    The timeout feature only works if the HAL LLD implements the
 *          required counter functionality.
 *
 * @param[in] p         address of the register
 * @param[in] mask      mask of bits to be checked
 * @param[in] match     value to be matched
 * @param[in] tmo       timeout in microseconds
 * @param[in] valp      pointer to where to store the last read value
 *                      or @p NULL
 * @return              The final result.
 * @retval false        if the condition has been verified.
 * @retval true         if a timeout occurred.
 *
 * @xclass
 */
bool halRegWaitMatch8X(volatile uint8_t *p, uint8_t mask,
                       uint8_t match, uint32_t tmo, uint8_t *valp) {
  halcnt_t start, end;

  /* Time window for the operation to complete.*/
  start = get_counter();
  end = start + HAL_US2RTC(get_frequency(), tmo);

  /* Testing the condition continuously until it becomes true or the
     timeout expires, it is done at least once.*/
  do {
    /* Getting register value and storing it outside if required.*/
    uint8_t v = *p;
    if (valp != NULL) {
      *valp = v;
    }

    /* Condition check and end of the loop if met.*/
    if ((v & mask) == match) {
      return false;
    }
  } while (is_counter_within(start, end));

  return true;
}

/**
 * @brief   Waits for masked bits to match or a timeout.
 * @note    The timeout feature only works if the HAL LLD implements the
 *          required counter functionality.
 *
 * @param[in] p         address of the register
 * @param[in] mask      mask of bits to be checked
 * @param[in] match     value to be matched
 * @param[in] tmo       timeout in microseconds
 * @param[in] valp      pointer to where to store the last read value
 *                      or @p NULL
 * @return              The final result.
 * @retval false        if the condition has been verified.
 * @retval true         if a timeout occurred.
 *
 * @xclass
 */
bool halRegWaitMatch16X(volatile uint16_t *p, uint16_t mask,
                        uint16_t match, uint32_t tmo, uint16_t *valp) {
  halcnt_t start, end;

  /* Time window for the operation to complete.*/
  start = get_counter();
  end = start + HAL_US2RTC(get_frequency(), tmo);

  /* Testing the condition continuously until it becomes true or the
     timeout expires, it is done at least once.*/
  do {
    /* Getting register value and storing it outside if required.*/
    uint16_t v = *p;
    if (valp != NULL) {
      *valp = v;
    }

    /* Condition check and end of the loop if met.*/
    if ((v & mask) == match) {
      return false;
    }
  } while (is_counter_within(start, end));

  return true;
}

/**
 * @brief   Waits for masked bits to match or a timeout.
 * @note    The timeout feature only works if the HAL LLD implements the
 *          required counter functionality.
 *
 * @param[in] p         address of the register
 * @param[in] mask      mask of bits to be checked
 * @param[in] match     value to be matched
 * @param[in] tmo       timeout in microseconds
 * @param[in] valp      pointer to where to store the last read value
 *                      or @p NULL
 * @return              The final result.
 * @retval false        if the condition has been verified.
 * @retval true         if a timeout occurred.
 *
 * @xclass
 */
bool halRegWaitMatch32X(volatile uint32_t *p, uint32_t mask,
                        uint32_t match, uint32_t tmo, uint32_t *valp) {
  halcnt_t start, end;

  /* Time window for the operation to complete.*/
  start = get_counter();
  end = start + HAL_US2RTC(get_frequency(), tmo);

  /* Testing the condition continuously until it becomes true or the
     timeout expires, it is done at least once.*/
  do {
    /* Getting register value and storing it outside if required.*/
    uint32_t v = *p;
    if (valp != NULL) {
      *valp = v;
    }

    /* Condition check and end of the loop if met.*/
    if ((v & mask) == match) {
      return false;
    }
  } while (is_counter_within(start, end));

  return true;
}

/**
 * @brief   Waits for all specified bits to be set or a timeout.
 * @note    The timeout feature only works if the HAL LLD implements the
 *          required counter functionality.
 *
 * @param[in] p         address of the register
 * @param[in] mask      mask of bits to be checked
 * @param[in] tmo       timeout in microseconds
 * @param[in] valp      pointer to where to store the last read value
 *                      or @p NULL
 * @return              The final result.
 * @retval false        if the condition has been verified.
 * @retval true         if a timeout occurred.
 *
 * @xclass
 */
bool halRegWaitAllSet8X(volatile uint8_t *p, uint8_t mask,
                        uint32_t tmo, uint8_t *valp) {
  halcnt_t start, end;

  /* Time window for the operation to complete.*/
  start = get_counter();
  end = start + HAL_US2RTC(get_frequency(), tmo);

  /* Testing the condition continuously until it becomes true or the
     timeout expires, it is done at least once.*/
  do {
    /* Getting register value and storing it outside if required.*/
    uint8_t v = *p;
    if (valp != NULL) {
      *valp = v;
    }

    /* Condition check and end of the loop if met.*/
    if ((v & mask) == mask) {
      return false;
    }
  } while (is_counter_within(start, end));

  return true;
}

/**
 * @brief   Waits for all specified bits to be set or a timeout.
 * @note    The timeout feature only works if the HAL LLD implements the
 *          required counter functionality.
 *
 * @param[in] p         address of the register
 * @param[in] mask      mask of bits to be checked
 * @param[in] tmo       timeout in microseconds
 * @param[in] valp      pointer to where to store the last read value
 *                      or @p NULL
 * @return              The final result.
 * @retval false        if the condition has been verified.
 * @retval true         if a timeout occurred.
 *
 * @xclass
 */
bool halRegWaitAllSet16X(volatile uint16_t *p, uint16_t mask,
                         uint32_t tmo, uint16_t *valp) {
  halcnt_t start, end;

  /* Time window for the operation to complete.*/
  start = get_counter();
  end = start + HAL_US2RTC(get_frequency(), tmo);

  /* Testing the condition continuously until it becomes true or the
     timeout expires, it is done at least once.*/
  do {
    /* Getting register value and storing it outside if required.*/
    uint16_t v = *p;
    if (valp != NULL) {
      *valp = v;
    }

    /* Condition check and end of the loop if met.*/
    if ((v & mask) == mask) {
      return false;
    }
  } while (is_counter_within(start, end));

  return true;
}

/**
 * @brief   Waits for all specified bits to be set or a timeout.
 * @note    The timeout feature only works if the HAL LLD implements the
 *          required counter functionality.
 *
 * @param[in] p         address of the register
 * @param[in] mask      mask of bits to be checked
 * @param[in] tmo       timeout in microseconds
 * @param[in] valp      pointer to where to store the last read value
 *                      or @p NULL
 * @return              The final result.
 * @retval false        if the condition has been verified.
 * @retval true         if a timeout occurred.
 *
 * @xclass
 */
bool halRegWaitAllSet32X(volatile uint32_t *p, uint32_t mask,
                         uint32_t tmo, uint32_t *valp) {
  halcnt_t start, end;

  /* Time window for the operation to complete.*/
  start = get_counter();
  end = start + HAL_US2RTC(get_frequency(), tmo);

  /* Testing the condition continuously until it becomes true or the
     timeout expires, it is done at least once.*/
  do {
    /* Getting register value and storing it outside if required.*/
    uint32_t v = *p;
    if (valp != NULL) {
      *valp = v;
    }

    /* Condition check and end of the loop if met.*/
    if ((v & mask) == mask) {
      return false;
    }
  } while (is_counter_within(start, end));

  return true;
}

/**
 * @brief   Waits for any of specified bits to be set or a timeout.
 * @note    The timeout feature only works if the HAL LLD implements the
 *          required counter functionality.
 *
 * @param[in] p         address of the register
 * @param[in] mask      mask of bits to be checked
 * @param[in] tmo       timeout in microseconds
 * @param[in] valp      pointer to where to store the last read value
 *                      or @p NULL
 * @return              The final result.
 * @retval false        if the condition has been verified.
 * @retval true         if a timeout occurred.
 *
 * @xclass
 */
bool halRegWaitAnySet8X(volatile uint8_t *p, uint8_t mask,
                        uint32_t tmo, uint8_t *valp) {
  halcnt_t start, end;

  /* Time window for the operation to complete.*/
  start = get_counter();
  end = start + HAL_US2RTC(get_frequency(), tmo);

  /* Testing the condition continuously until it becomes true or the
     timeout expires, it is done at least once.*/
  do {
    /* Getting register value and storing it outside if required.*/
    uint8_t v = *p;
    if (valp != NULL) {
      *valp = v;
    }

    /* Condition check and end of the loop if met.*/
    if ((v & mask) != (uint8_t)0) {
      return false;
    }
  } while (is_counter_within(start, end));

  return true;
}

/**
 * @brief   Waits for any of specified bits to be set or a timeout.
 * @note    The timeout feature only works if the HAL LLD implements the
 *          required counter functionality.
 *
 * @param[in] p         address of the register
 * @param[in] mask      mask of bits to be checked
 * @param[in] tmo       timeout in microseconds
 * @param[in] valp      pointer to where to store the last read value
 *                      or @p NULL
 * @return              The final result.
 * @retval false        if the condition has been verified.
 * @retval true         if a timeout occurred.
 *
 * @xclass
 */
bool halRegWaitAnySet16X(volatile uint16_t *p, uint16_t mask,
                         uint32_t tmo, uint16_t *valp) {
  halcnt_t start, end;

  /* Time window for the operation to complete.*/
  start = get_counter();
  end = start + HAL_US2RTC(get_frequency(), tmo);

  /* Testing the condition continuously until it becomes true or the
     timeout expires, it is done at least once.*/
  do {
    /* Getting register value and storing it outside if required.*/
    uint16_t v = *p;
    if (valp != NULL) {
      *valp = v;
    }

    /* Condition check and end of the loop if met.*/
    if ((v & mask) != (uint16_t)0) {
      return false;
    }
  } while (is_counter_within(start, end));

  return true;
}

/**
 * @brief   Waits for any of specified bits to be set or a timeout.
 * @note    The timeout feature only works if the HAL LLD implements the
 *          required counter functionality.
 *
 * @param[in] p         address of the register
 * @param[in] mask      mask of bits to be checked
 * @param[in] tmo       timeout in microseconds
 * @param[in] valp      pointer to where to store the last read value
 *                      or @p NULL
 * @return              The final result.
 * @retval false        if the condition has been verified.
 * @retval true         if a timeout occurred.
 *
 * @xclass
 */
bool halRegWaitAnySet32X(volatile uint32_t *p, uint32_t mask,
                         uint32_t tmo, uint32_t *valp) {
  halcnt_t start, end;

  /* Time window for the operation to complete.*/
  start = get_counter();
  end = start + HAL_US2RTC(get_frequency(), tmo);

  /* Testing the condition continuously until it becomes true or the
     timeout expires, it is done at least once.*/
  do {
    /* Getting register value and storing it outside if required.*/
    uint32_t v = *p;
    if (valp != NULL) {
      *valp = v;
    }

    /* Condition check and end of the loop if met.*/
    if ((v & mask) != (uint32_t)0) {
      return false;
    }
  } while (is_counter_within(start, end));

  return true;
}

/**
 * @brief   Waits for all specified bits to be cleared or a timeout.
 * @note    The timeout feature only works if the HAL LLD implements the
 *          required counter functionality.
 *
 * @param[in] p         address of the register
 * @param[in] mask      mask of bits to be checked
 * @param[in] tmo       timeout in microseconds
 * @param[in] valp      pointer to where to store the last read value
 *                      or @p NULL
 * @return              The final result.
 * @retval false        if the condition has been verified.
 * @retval true         if a timeout occurred.
 *
 * @xclass
 */
bool halRegWaitAllClear8X(volatile uint8_t *p, uint8_t mask,
                          uint32_t tmo, uint8_t *valp) {
  halcnt_t start, end;

  /* Time window for the operation to complete.*/
  start = get_counter();
  end = start + HAL_US2RTC(get_frequency(), tmo);

  /* Testing the condition continuously until it becomes true or the
     timeout expires, it is done at least once.*/
  do {
    /* Getting register value and storing it outside if required.*/
    uint8_t v = *p;
    if (valp != NULL) {
      *valp = v;
    }

    /* Condition check and end of the loop if met.*/
    if ((v & mask) == (uint8_t)0) {
      return false;
    }
  } while (is_counter_within(start, end));

  return true;
}

/**
 * @brief   Waits for all specified bits to be cleared or a timeout.
 * @note    The timeout feature only works if the HAL LLD implements the
 *          required counter functionality.
 *
 * @param[in] p         address of the register
 * @param[in] mask      mask of bits to be checked
 * @param[in] tmo       timeout in microseconds
 * @param[in] valp      pointer to where to store the last read value
 *                      or @p NULL
 * @return              The final result.
 * @retval false        if the condition has been verified.
 * @retval true         if a timeout occurred.
 *
 * @xclass
 */
bool halRegWaitAllClear16X(volatile uint16_t *p, uint16_t mask,
                           uint32_t tmo, uint16_t *valp) {
  halcnt_t start, end;

  /* Time window for the operation to complete.*/
  start = get_counter();
  end = start + HAL_US2RTC(get_frequency(), tmo);

  /* Testing the condition continuously until it becomes true or the
     timeout expires, it is done at least once.*/
  do {
    /* Getting register value and storing it outside if required.*/
    uint16_t v = *p;
    if (valp != NULL) {
      *valp = v;
    }

    /* Condition check and end of the loop if met.*/
    if ((v & mask) == (uint16_t)0) {
      return false;
    }
  } while (is_counter_within(start, end));

  return true;
}

/**
 * @brief   Waits for all specified bits to be cleared or a timeout.
 * @note    The timeout feature only works if the HAL LLD implements the
 *          required counter functionality.
 *
 * @param[in] p         address of the register
 * @param[in] mask      mask of bits to be checked
 * @param[in] tmo       timeout in microseconds
 * @param[in] valp      pointer to where to store the last read value
 *                      or @p NULL
 * @return              The final result.
 * @retval false        if the condition has been verified.
 * @retval true         if a timeout occurred.
 *
 * @xclass
 */
bool halRegWaitAllClear32X(volatile uint32_t *p, uint32_t mask,
                           uint32_t tmo, uint32_t *valp) {
  halcnt_t start, end;

  /* Time window for the operation to complete.*/
  start = get_counter();
  end = start + HAL_US2RTC(get_frequency(), tmo);

  /* Testing the condition continuously until it becomes true or the
     timeout expires, it is done at least once.*/
  do {
    /* Getting register value and storing it outside if required.*/
    uint32_t v = *p;
    if (valp != NULL) {
      *valp = v;
    }

    /* Condition check and end of the loop if met.*/
    if ((v & mask) == (uint32_t)0) {
      return false;
    }
  } while (is_counter_within(start, end));

  return true;
}

/**
 * @brief   Waits for any of specified bits to be cleared or a timeout.
 * @note    The timeout feature only works if the HAL LLD implements the
 *          required counter functionality.
 *
 * @param[in] p         address of the register
 * @param[in] mask      mask of bits to be checked
 * @param[in] tmo       timeout in microseconds
 * @param[in] valp      pointer to where to store the last read value
 *                      or @p NULL
 * @return              The final result.
 * @retval false        if the condition has been verified.
 * @retval true         if a timeout occurred.
 *
 * @xclass
 */
bool halRegWaitAnyClear8X(volatile uint8_t *p, uint8_t mask,
                          uint32_t tmo, uint8_t *valp) {
  halcnt_t start, end;

  /* Time window for the operation to complete.*/
  start = get_counter();
  end = start + HAL_US2RTC(get_frequency(), tmo);

  /* Testing the condition continuously until it becomes true or the
     timeout expires, it is done at least once.*/
  do {
    /* Getting register value and storing it outside if required.*/
    uint8_t v = *p;
    if (valp != NULL) {
      *valp = v;
    }

    /* Condition check and end of the loop if met.*/
    if ((v & mask) != mask) {
      return false;
    }
  } while (is_counter_within(start, end));

  return true;
}

/**
 * @brief   Waits for any of specified bits to be cleared or a timeout.
 * @note    The timeout feature only works if the HAL LLD implements the
 *          required counter functionality.
 *
 * @param[in] p         address of the register
 * @param[in] mask      mask of bits to be checked
 * @param[in] tmo       timeout in microseconds
 * @param[in] valp      pointer to where to store the last read value
 *                      or @p NULL
 * @return              The final result.
 * @retval false        if the condition has been verified.
 * @retval true         if a timeout occurred.
 *
 * @xclass
 */
bool halRegWaitAnyClear16X(volatile uint16_t *p, uint16_t mask,
                           uint32_t tmo, uint16_t *valp) {
  halcnt_t start, end;

  /* Time window for the operation to complete.*/
  start = get_counter();
  end = start + HAL_US2RTC(get_frequency(), tmo);

  /* Testing the condition continuously until it becomes true or the
     timeout expires, it is done at least once.*/
  do {
    /* Getting register value and storing it outside if required.*/
    uint16_t v = *p;
    if (valp != NULL) {
      *valp = v;
    }

    /* Condition check and end of the loop if met.*/
    if ((v & mask) != mask) {
      return false;
    }
  } while (is_counter_within(start, end));

  return true;
}

/**
 * @brief   Waits for any of specified bits to be cleared or a timeout.
 * @note    The timeout feature only works if the HAL LLD implements the
 *          required counter functionality.
 *
 * @param[in] p         address of the register
 * @param[in] mask      mask of bits to be checked
 * @param[in] tmo       timeout in microseconds
 * @param[in] valp      pointer to where to store the last read value
 *                      or @p NULL
 * @return              The final result.
 * @retval false        if the condition has been verified.
 * @retval true         if a timeout occurred.
 *
 * @xclass
 */
bool halRegWaitAnyClear32X(volatile uint32_t *p, uint32_t mask,
                           uint32_t tmo, uint32_t *valp) {
  halcnt_t start, end;

  /* Time window for the operation to complete.*/
  start = get_counter();
  end = start + HAL_US2RTC(get_frequency(), tmo);

  /* Testing the condition continuously until it becomes true or the
     timeout expires, it is done at least once.*/
  do {
    /* Getting register value and storing it outside if required.*/
    uint32_t v = *p;
    if (valp != NULL) {
      *valp = v;
    }

    /* Condition check and end of the loop if met.*/
    if ((v & mask) != mask) {
      return false;
    }
  } while (is_counter_within(start, end));

  return true;
}

/** @} */
