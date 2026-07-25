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
/* Implementation of LPC176x UART0 function to be intercepted and sent to mri instead. */
    .text
    .code 16
    .syntax unified


    .global UART0_IRQHandler
    .weak   UART0_IRQHandler
    .type UART0_IRQHandler, function
    .thumb_func
    /* extern "C" void UART0_IRQHandler(void);
       Override UART0 exceptions and send to __mriExceptionHandler.
    */
UART0_IRQHandler:
    b       __mriExceptionHandler

    .global UART1_IRQHandler
    .weak   UART1_IRQHandler
    .type UART1_IRQHandler, function
    .thumb_func
    /* extern "C" void UART1_IRQHandler(void);
       Override UART1 exceptions and send to __mriExceptionHandler.
    */
UART1_IRQHandler:
    b       __mriExceptionHandler

    .global UART2_IRQHandler
    .weak   UART2_IRQHandler
    .type UART2_IRQHandler, function
    .thumb_func
    /* extern "C" void UART2_IRQHandler(void);
       Override UART2 exceptions and send to __mriExceptionHandler.
    */
UART2_IRQHandler:
    b       __mriExceptionHandler

    .global UART3_IRQHandler
    .weak   UART3_IRQHandler
    .type UART3_IRQHandler, function
    .thumb_func
    /* extern "C" void UART3_IRQHandler(void);
       Override UART3 exceptions and send to __mriExceptionHandler.
    */
UART3_IRQHandler:
    b       __mriExceptionHandler
