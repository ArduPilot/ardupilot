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
/* Routines used by mri that are specific to the LPC176x device. */
#ifndef _LPC176X_H_
#define _LPC176X_H_

#include <stdint.h>
#include <token.h>
#include "lpc176x_uart.h"

/* Flags that can be set in Lpc176xState::flags */
#define LPC176X_UART_FLAGS_SHARE        1
#define LPC176X_UART_FLAGS_MANUAL_BAUD  2

/* Flag to indicate whether context will contain FPU registers or not. */
#define MRI_DEVICE_HAS_FPU 0

typedef struct
{
    const UartConfiguration*  pCurrentUart;
    uint32_t                  flags;
} Lpc176xState;

extern Lpc176xState __mriLpc176xState;

void __mriLpc176x_Init(Token* pParameterTokens);

#endif /* _LPC176X_H_ */
