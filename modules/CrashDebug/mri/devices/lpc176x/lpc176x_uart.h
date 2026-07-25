/* Copyright 2012 Adam Green (http://mbed.org/users/AdamGreen/)

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
#ifndef _LPC176X_UART_H_
#define _LPC176X_UART_H_

#include <stdint.h>
#include <LPC17xx.h>
#include <token.h>

typedef struct
{
    volatile uint32_t*  pPeripheralClockSelection;
    volatile uint32_t*  pTxPinSelection;
    volatile uint32_t*  pRxPinSelection;
    LPC_UART_TypeDef*   pUartRegisters;
    uint32_t            powerConfigurationBit;
    uint32_t            peripheralClockSelectionBitmask;
    uint32_t            txPinSelectionMask;
    uint32_t            rxPinSelectionMask;
    uint32_t            pinSelectionValue;
} UartConfiguration;

void __mriLpc176xUart_Init(Token* pParameterTokens);

#endif /* _LPC176X_UART_H_ */
