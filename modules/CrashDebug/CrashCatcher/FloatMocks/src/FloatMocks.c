/* Copyright (C) 2015  Adam Green (https://github.com/adamgreen)

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
#include <CrashCatcherPriv.h>
#include <FloatMocks.h>
#include <string.h>


static const uint32_t* g_pAllFloatRegisters;
static const uint32_t* g_pUpperFloatRegisters;


void FloatMocks_Init(void)
{
    g_pAllFloatRegisters = NULL;
    g_pUpperFloatRegisters = NULL;
}


void FloatMocks_Uninit(void)
{
}


void FloatMocks_SetAllFloatingPointRegisters(const uint32_t* pFloatRegisters)
{
    g_pAllFloatRegisters = pFloatRegisters;
}


void FloatMocks_SetUpperFloatingPointRegisters(const uint32_t* pFloatRegisters)
{
    g_pUpperFloatRegisters = pFloatRegisters;
}



/* Mock implementation of CrashCatcher_* floating point routines. */
void CrashCatcher_CopyAllFloatingPointRegisters(uint32_t* pBuffer)
{
    /* There are 32 single-precision and 1 FPCSR register to copy out. */
    memcpy(pBuffer, g_pAllFloatRegisters, (32 + 1) * sizeof(uint32_t));
}

void CrashCatcher_CopyUpperFloatingPointRegisters(uint32_t* pBuffer)
{
    memcpy(pBuffer, g_pUpperFloatRegisters, 16 * sizeof(uint32_t));
}