/* Copyright 2013 Adam Green (http://mbed.org/users/AdamGreen/)

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
/* Routines to access memory on target device when running on target itself. */
#include <platforms.h>

uint32_t Platform_MemRead32(const void* pv)
{
    return  *(volatile const uint32_t*)pv;
}

uint16_t Platform_MemRead16(const void* pv)
{
    return  *(volatile const uint16_t*)pv;
}

uint8_t Platform_MemRead8(const void* pv)
{
    return  *(volatile const uint8_t*)pv;
}

void Platform_MemWrite32(void* pv, uint32_t value)
{
    *(volatile uint32_t*)pv = value;
}

void Platform_MemWrite16(void* pv, uint16_t value)
{
    *(volatile uint16_t*)pv = value;
}

void Platform_MemWrite8(void* pv, uint8_t value)
{
    *(volatile uint8_t*)pv = value;
}
