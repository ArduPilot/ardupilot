/* Copyright 2012 Adam Green     (http://mbed.org/users/AdamGreen/)
   Copyright 2015 Chang,Jia-Rung (https://github.com/JaredCJR)

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
/* Routines used by mri that are specific to the STM32F429xx device. */
#ifndef _STM32F429XX_H_
#define _STM32F429XX_H_

#include <stdint.h>
#include <token.h>
#include "stm32f429xx_usart.h"

/* Flags that can be set in Stm32f429xxState::flags */
#define STM32F429XX_UART_FLAGS_SHARE        1
#define STM32F429XX_UART_FLAGS_MANUAL_BAUD  2

/* Flag to indicate whether context will contain FPU registers or not. */
#define MRI_DEVICE_HAS_FPU 1

typedef struct
{
    const UartConfiguration*  pCurrentUart;
    uint32_t                  flags;
} Stm32f429xxState;

extern Stm32f429xxState __mriStm32f429xxState;

void __mriStm32f429xx_Init(Token* pParameterTokens);

#endif /* _STM32F429XX_H_ */
