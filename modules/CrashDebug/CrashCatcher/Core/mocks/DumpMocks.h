/* Copyright (C) 2018  Adam Green (https://github.com/adamgreen)

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
#ifndef _DUMP_MOCKS_H_
#define _DUMP_MOCKS_H_

#include <CrashCatcher.h>
#include <stdint.h>


void DumpMocks_Init(void);
void DumpMocks_Uninit(void);

uint32_t                DumpMocks_GetDumpStartCallCount(void);
const CrashCatcherInfo* DumpMocks_GetDumpStartInfo(void);

void     DumpMocks_EnableDumpStartStackOverflowSimulation(void);
uint32_t DumpMocks_GetDumpEndCallCount(void);
void     DumpMocks_SetDumpEndLoops(uint32_t timesToReturnTryAgain);

void     DumpMocks_SetMemoryRegions(const CrashCatcherMemoryRegion* pRegions);

uint32_t DumpMocks_GetDumpMemoryCallCount(void);
int      DumpMocks_VerifyDumpMemoryItem(uint32_t item,
                                        const void* pvMemory,
                                        CrashCatcherElementSizes elementSize,
                                        size_t elementCount);


#endif /* _DUMP_MOCKS_H_ */
