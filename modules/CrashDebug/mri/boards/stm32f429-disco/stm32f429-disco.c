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
/* Routines which expose STM32F429 Discovery specific functionality to the mri debugger. */
#include <string.h>
#include <platforms.h>
#include <try_catch.h>
#include "../../architectures/armv7-m/debug_cm3.h"
#include "../../devices/stm32f429xx/stm32f429xx_init.h"


void Platform_Init(Token* pParameterTokens)
{
    __mriStm32f429xx_Init(pParameterTokens);
}


const uint8_t* __mriPlatform_GetUid(void)
{
    return NULL;
}


uint32_t __mriPlatform_GetUidSize(void)
{
    return 0;
}
