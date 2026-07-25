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
/* Routines used to provide STM32F429xx USART functionality to the mri debugger. */
#ifndef _STM32F429XX_USART_H_
#define _STM32F429XX_USART_H_

#include <stdint.h>
#include <stm32f4xx.h>
#include <token.h>

typedef struct 
{
    USART_TypeDef*     pUartRegisters;
    uint32_t    txFunction;
    uint32_t    rxFunction;
} UartConfiguration;


void __mriStm32f429xxUart_Init(Token* pParameterTokens);

#endif /* _STM32F429XX_USART_H_ */
