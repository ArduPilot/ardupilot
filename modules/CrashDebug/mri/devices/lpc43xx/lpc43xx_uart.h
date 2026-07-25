/* Copyright 2015 Adam Green (http://mbed.org/users/AdamGreen/)

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
/* Routines used to provide LPC176x UART functionality to the mri debugger. */
#ifndef _LPC43XX_UART_H_
#define _LPC43XX_UART_H_

#include <stdint.h>
#include <LPC43xx.h>
#include <token.h>

#define SCU_PIN(GROUP, NUM) (((GROUP) << 16) | (NUM))

typedef struct
{
    LPC_USART_T*        pUartRegisters;
    CGU_BASE_CLK_T      baseClock;
    CCU_CLK_T           registerClock;
    CCU_CLK_T           peripheralClock;
    uint32_t            txPin;
    uint32_t            txFunction;
    uint32_t            rxPin;
    uint32_t            rxFunction;
} UartConfiguration;

void __mriLpc43xxUart_Init(Token* pParameterTokens);

#endif /* _LPC43XX_UART_H_ */
