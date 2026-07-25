/* Copyright (C) 2014  Adam Green (https://github.com/adamgreen)

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
#include <assert.h>
#include <DumpMocks.h>
#include <string.h>


static const CrashCatcherMemoryRegion* g_pRegions;
static const int*                      g_pGetCData;
static char*                           g_pPutCDataStart;
static char*                           g_pPutCDataCurr;
static char*                           g_pPutCDataEnd;

void DumpMocks_Init(size_t putcBufferSize)
{
    g_pRegions = NULL;
    g_pGetCData = NULL;
    g_pPutCDataStart = malloc(putcBufferSize + 1);
    g_pPutCDataCurr = g_pPutCDataStart;
    g_pPutCDataEnd = g_pPutCDataStart + putcBufferSize;
}


void DumpMocks_Uninit(void)
{
    free(g_pPutCDataStart);
    g_pPutCDataStart = NULL;
    g_pPutCDataCurr = NULL;
    g_pPutCDataEnd = NULL;
}


void DumpMocks_SetMemoryRegions(const CrashCatcherMemoryRegion* pRegions)
{
    g_pRegions = pRegions;
}


void DumpMocks_SetGetcData(const int* pData)
{
    g_pGetCData = pData;
}


const char* DumpMocks_GetPutCData(void)
{
    *g_pPutCDataCurr = '\0';
    return g_pPutCDataStart;
}


/* Mock implementation of CrashCatcher_Dump* routines. */
const CrashCatcherMemoryRegion* CrashCatcher_GetMemoryRegions(void)
{
    return g_pRegions;
}


int CrashCatcher_getc(void)
{
    return *g_pGetCData++;
}


void CrashCatcher_putc(int c)
{
    if (g_pPutCDataCurr >= g_pPutCDataEnd)
        return;
    *g_pPutCDataCurr++ = (char)c;
}
