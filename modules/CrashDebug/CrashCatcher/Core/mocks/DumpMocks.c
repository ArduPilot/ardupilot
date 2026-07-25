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
#include <assert.h>
#include <CrashCatcherPriv.h>
#include <DumpMocks.h>
#include <string.h>


typedef struct DumpMemoryItem
{
    void*                    pvMemory;
    CrashCatcherElementSizes elementSize;
    size_t                   elementCount;
} DumpMemoryItem;


static uint32_t                        g_dumpStartCallCount;
static CrashCatcherInfo                g_dumpInfo;
static int                             g_dumpStartSimulateStackOverflow;
static uint32_t                        g_dumpEndCallCount;
static uint32_t                        g_dumpLoopCount;
static uint32_t                        g_dumpMemoryItemCount;
static DumpMemoryItem*                 g_pDumpMemoryItems;
static const CrashCatcherMemoryRegion* g_pRegions;


static void freeMemoryItems(void);


void DumpMocks_Init(void)
{
    g_dumpStartCallCount = 0;
    memset(&g_dumpInfo, 0xFF, sizeof(g_dumpInfo));
    g_dumpStartSimulateStackOverflow = 0;
    g_dumpEndCallCount = 0;
    g_dumpMemoryItemCount = 0;
    g_pDumpMemoryItems = NULL;
    g_pRegions = NULL;
    g_dumpLoopCount = 0;
}


void DumpMocks_Uninit(void)
{
    freeMemoryItems();
}

static void freeMemoryItems(void)
{
    uint32_t i;
    for (i = 0 ; i < g_dumpMemoryItemCount ; i++)
        free(g_pDumpMemoryItems[i].pvMemory);
    free(g_pDumpMemoryItems);
}


uint32_t DumpMocks_GetDumpStartCallCount(void)
{
    return g_dumpStartCallCount;
}

const CrashCatcherInfo* DumpMocks_GetDumpStartInfo(void)
{
    return &g_dumpInfo;
}


void DumpMocks_EnableDumpStartStackOverflowSimulation(void)
{
    g_dumpStartSimulateStackOverflow = 1;
}


uint32_t DumpMocks_GetDumpEndCallCount(void)
{
    return g_dumpEndCallCount;
}


void DumpMocks_SetDumpEndLoops(uint32_t timesToReturnTryAgain)
{
    g_dumpLoopCount = timesToReturnTryAgain;
}


void DumpMocks_SetMemoryRegions(const CrashCatcherMemoryRegion* pRegions)
{
    g_pRegions = pRegions;
}


uint32_t DumpMocks_GetDumpMemoryCallCount(void)
{
    return g_dumpMemoryItemCount;
}

int DumpMocks_VerifyDumpMemoryItem(uint32_t item, const void* pvMemory, CrashCatcherElementSizes elementSize, size_t elementCount)
{
    DumpMemoryItem* pItem = NULL;

    assert( item < g_dumpMemoryItemCount );
    pItem = &g_pDumpMemoryItems[item];

    if (pItem->elementSize != elementSize)
        return FALSE;
    if (pItem->elementCount != elementCount)
        return FALSE;
    if (0 != memcmp(pItem->pvMemory, pvMemory, (size_t)elementSize * elementCount))
        return FALSE;
    return TRUE;
}


/* Mock implementation of CrashCatcher_Dump* routines. */
void CrashCatcher_DumpStart(const CrashCatcherInfo* pInfo)
{
    g_dumpStartCallCount++;
    if (g_dumpStartSimulateStackOverflow)
        g_crashCatcherStack[0] = 0;
    memcpy(&g_dumpInfo, pInfo, sizeof(g_dumpInfo));
}


const CrashCatcherMemoryRegion* CrashCatcher_GetMemoryRegions(void)
{
    return g_pRegions;
}


void CrashCatcher_DumpMemory(const void* pvMemory, CrashCatcherElementSizes elementSize, size_t elementCount)
{
    g_pDumpMemoryItems = realloc(g_pDumpMemoryItems, sizeof(*g_pDumpMemoryItems) * (g_dumpMemoryItemCount + 1));

    g_pDumpMemoryItems[g_dumpMemoryItemCount].pvMemory = malloc((size_t)elementSize * elementCount);
    memcpy(g_pDumpMemoryItems[g_dumpMemoryItemCount].pvMemory, pvMemory, (size_t)elementSize * elementCount);

    g_pDumpMemoryItems[g_dumpMemoryItemCount].elementSize = elementSize;
    g_pDumpMemoryItems[g_dumpMemoryItemCount].elementCount = elementCount;
    g_dumpMemoryItemCount++;
}


CrashCatcherReturnCodes CrashCatcher_DumpEnd(void)
{
    g_dumpEndCallCount++;
    if (g_dumpLoopCount)
    {
        g_dumpLoopCount--;
        return CRASH_CATCHER_TRY_AGAIN;
    }
    else
    {
        return CRASH_CATCHER_EXIT;
    }
}
