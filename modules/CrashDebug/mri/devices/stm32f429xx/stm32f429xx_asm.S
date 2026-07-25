/* Copyright 2015 Adam Green     (http://mbed.org/users/AdamGreen/)
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
/* Implementation of STM32F429xx USART1 function to be intercepted and sent to mri instead. */
    .text
    .code 16
    .syntax unified


    .global USART1_IRQHandler
    .weak   USART1_IRQHandler
    .type USART1_IRQHandler, function
    .thumb_func
    /* extern "C" void USART1_IRQHandler(void);
       Override USART1 exceptions and send to __mriExceptionHandler.
    */
USART1_IRQHandler:
    b       __mriExceptionHandler


    .global USART2_IRQHandler
    .weak   USART2_IRQHandler
    .type USART2_IRQHandler, function
    .thumb_func
    /* extern "C" void USART2_IRQHandler(void);
       Override USART2 exceptions and send to __mriExceptionHandler.
    */
USART2_IRQHandler:
    b       __mriExceptionHandler


    .global USART3_IRQHandler
    .weak   USART3_IRQHandler
    .type USART3_IRQHandler, function
    .thumb_func
    /* extern "C" void USART3_IRQHandler(void);
       Override USART3 exceptions and send to __mriExceptionHandler.
    */
USART3_IRQHandler:
    b       __mriExceptionHandler

