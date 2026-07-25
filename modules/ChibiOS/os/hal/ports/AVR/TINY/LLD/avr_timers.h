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

#ifndef AVR_TIMERS_H
#define AVR_TIMERS_H

#include "mcuconf.h"

#if ((OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING) && \
     (AVR_GPT_USE_TIM1 || AVR_PWM_USE_TIM1 || AVR_ICU_USE_TIM1))
  #error "Timer 1 cannot be used by drivers when running in tickless mode."
#endif

#if ((AVR_GPT_USE_TIM1 && AVR_PWM_USE_TIM1) || \
     (AVR_GPT_USE_TIM1 && AVR_ICU_USE_TIM1) || \
     (AVR_PWM_USE_TIM1 && AVR_ICU_USE_TIM1))
  #error "Timer 1 cannot simultaneously be used by multiple drivers."
#endif

#if ((AVR_GPT_USE_TIM2 && AVR_PWM_USE_TIM2))
  #error "Timer 2 cannot simultaneously be used by multiple drivers."
#endif

#if ((AVR_GPT_USE_TIM3 && AVR_PWM_USE_TIM3) || \
     (AVR_GPT_USE_TIM3 && AVR_ICU_USE_TIM3) || \
     (AVR_PWM_USE_TIM3 && AVR_ICU_USE_TIM3))
  #error "Timer 3 cannot simultaneously be used by multiple drivers."
#endif

#if ((AVR_GPT_USE_TIM4 && AVR_PWM_USE_TIM4) || \
     (AVR_GPT_USE_TIM4 && AVR_ICU_USE_TIM4) || \
     (AVR_PWM_USE_TIM4 && AVR_ICU_USE_TIM4))
  #error "Timer 4 cannot simultaneously be used by multiple drivers."
#endif

#if ((AVR_GPT_USE_TIM5 && AVR_PWM_USE_TIM5) || \
     (AVR_GPT_USE_TIM5 && AVR_ICU_USE_TIM5) || \
     (AVR_PWM_USE_TIM5 && AVR_ICU_USE_TIM5))
  #error "Timer 5 cannot simultaneously be used by multiple drivers."
#endif

#endif /* AVR_TIMERS_H */
