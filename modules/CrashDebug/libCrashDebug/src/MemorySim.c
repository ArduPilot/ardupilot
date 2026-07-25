/*  Copyright (C) 2019  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <MemorySim.h>
#include <MallocFailureInject.h>
#include "mriPlatformPriv.h"
/* Bit that can be set in Watchpoint::type field to flag as breakpoint and not watchpoint. */
#define WATCHPOINT_BREAKPOINT (WatchpointType)((1 << 31) | WATCHPOINT_READ)

/* Memory access types are based on watchpoint types so that watchpoint can be used as mask. */
#define AccessType WatchpointType
#define READING    WATCHPOINT_READ
#define WRITING    WATCHPOINT_WRITE
/* Even though loading data into FLASH is a write operation, we don't want bus exceptions generated. */
#define LOADING    WATCHPOINT_READ

/* Should this memory access be checked for break/watchpoints? */
#define ENABLE_WATCHPOINT_CHECK     1
#define DISABLE_WATCHPOINT_CHECK    0

static const char g_xmlHeader[] = "<?xml version=\"1.0\"?>"
                                "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                "<memory-map>";
static const char g_xmlTrailer[] = "</memory-map>";


typedef struct SizedBuffer
{
    char*  pBuffer;
    size_t size;
} SizedBuffer;

typedef enum MatchResult
{
    FOUND,          /* Found requested value. */
    FOUND_HIGHER,   /* Found a value higher than requested but not exact value requested. */
    NOT_FOUND       /* Walked all entries and found no match or anything higher. */

} MatchResult;

/* Forward Declarations */
typedef struct MemorySim MemorySim;
typedef struct MemoryRegion MemoryRegion;
typedef struct Watchpoint Watchpoint;

static void freeRegion(MemoryRegion* pRegion);
void* throwingZeroedMalloc(size_t size);
static void addRegionToTail(MemorySim* pThis, MemoryRegion* pRegion);
static MemoryRegion* findMatchingRegion(MemorySim* pThis, uint32_t* pAddress, uint32_t size);
static void allocateReadCountArrayForReadOnlyRegion(MemoryRegion* pRegion);
static void load32(IMemory* pMemory, uint32_t address, uint32_t value);
static void load8(IMemory* pMemory, uint32_t address, uint8_t value);
static void freeLastRegion(MemorySim* pThis);
static size_t countRegions(MemorySim* pThis);
static void allocateMemoryMapXML(MemorySim* pThis, size_t allocSize);
static void appendMemoryMapXmlHeader(MemorySim* pThis, SizedBuffer* pBuffer);
static void appendMemoryMapRegions(MemorySim* pThis, SizedBuffer* pBuffer);
static void appendMemoryMapXmlTrailer(MemorySim* pThis, SizedBuffer* pBuffer);
static void setWatchpoint(IMemory* pMemory, uint32_t address, uint32_t size, WatchpointType type);
static MatchResult findMatchingOrHigherWatchpoint(MemoryRegion* pRegion, Watchpoint* pKey, uint32_t* pIndex);
static int compareWatchpoints(const void* pvKey, const void* pvCurr);
static int watchpointsMatch(const Watchpoint* p1, const Watchpoint* p2);
static void growWatchpointArrayIfNeeded(MemoryRegion* pRegion, uint32_t requiredSize);
static void clearWatchpoint(IMemory* pMemory, uint32_t address, uint32_t size, WatchpointType type);
static void* getDataPointer(MemorySim* pThis, uint32_t address, uint32_t size, AccessType type, int checkWatchpoints);
static void checkForBreakWatchPoint(MemorySim* pThis,
                                    MemoryRegion* pRegion,
                                    uint32_t address, uint32_t size, AccessType type);
static int accessInRange(Watchpoint* pWatchpoint, uint32_t startAddress, uint32_t endAddress);

static uint32_t read32(IMemory* pMemory, uint32_t address);
static uint16_t read16(IMemory* pMemory, uint32_t address);
static uint8_t read8(IMemory* pMemory, uint32_t address);
static void write32(IMemory* pMemory, uint32_t address, uint32_t value);
static void write16(IMemory* pMemory, uint32_t address, uint16_t value);
static void write8(IMemory* pMemory, uint32_t address, uint8_t value);

static IMemoryVTable g_vTable = {read32, read16, read8, write32, write16, write8};

struct Watchpoint
{
    WatchpointType type;
    uint32_t       startAddress;
    uint32_t       endAddress;

};

struct MemoryRegion
{
    struct MemoryRegion* pNext;
    struct MemoryRegion* pRedirect;
    uint8_t*             pData;
    Watchpoint*          pWatchpoints;
    uint32_t*            pReadCounts;
    uint32_t             baseAddress;
    uint32_t             redirectAddress;
    uint32_t             size;
    uint32_t             watchpointCount;
    uint32_t             watchpointAlloc;
    uint32_t             readCounts;
    int                  isReadOnly;
    int                  isAlias;
};

struct MemorySim
{
    IMemoryVTable* pVTable;
    MemoryRegion*  pHeadRegion;
    MemoryRegion*  pTailRegion;
    char*          pMemoryMapXML;
    int            watchpointEncountered;
};

static MemorySim g_object;


IMemory* MemorySim_Init(void)
{
    memset(&g_object, 0, sizeof(g_object));
    g_object.pVTable = &g_vTable;
    return (IMemory*)&g_object;
}


void MemorySim_Uninit(IMemory* pMemory)
{
    MemorySim*    pThis = (MemorySim*)pMemory;
    MemoryRegion* pCurr;

    if (!pThis)
        return;

    pCurr = pThis->pHeadRegion;
    while (pCurr)
    {
        MemoryRegion* pNext = pCurr->pNext;
        freeRegion(pCurr);
        pCurr = pNext;
    }
    pThis->pHeadRegion = pThis->pTailRegion = NULL;

    free(pThis->pMemoryMapXML);
    pThis->pMemoryMapXML = NULL;
}

static void freeRegion(MemoryRegion* pRegion)
{
    if (!pRegion)
        return;

    free(pRegion->pReadCounts);
    free(pRegion->pWatchpoints);
    free(pRegion->pData);
    free(pRegion);
}


void MemorySim_CreateRegion(IMemory* pMemory, uint32_t baseAddress, uint32_t size)
{
    MemorySim*             pThis = (MemorySim*)pMemory;
    MemoryRegion* volatile pRegion = NULL;

    __try
    {
        pRegion = throwingZeroedMalloc(sizeof(*pRegion));
        pRegion->baseAddress = baseAddress;
        pRegion->size = size;
        pRegion->pData = throwingZeroedMalloc(size);
        addRegionToTail(pThis, pRegion);
    }
    __catch
    {
        freeRegion(pRegion);
        __rethrow;
    }
}

void* throwingZeroedMalloc(size_t size)
{
    void* pvAlloc = malloc(size);
    if (!pvAlloc)
        __throw(outOfMemoryException);
    memset(pvAlloc, 0, size);
    return pvAlloc;
}

static void addRegionToTail(MemorySim* pThis, MemoryRegion* pRegion)
{
    if (!pThis->pTailRegion)
        pThis->pHeadRegion = pRegion;
    else
        pThis->pTailRegion->pNext = pRegion;
    pThis->pTailRegion = pRegion;
}


void MemorySim_CreateAlias(IMemory* pMemory, uint32_t aliasAddress, uint32_t redirectAddress, uint32_t size)
{
    MemorySim*             pThis = (MemorySim*)pMemory;
    MemoryRegion* volatile pRegion = NULL;
    MemoryRegion*          pRedirect = NULL;

    __try
    {
        pRegion = throwingZeroedMalloc(sizeof(*pRegion));
        pRegion->baseAddress = aliasAddress;
        pRegion->redirectAddress = redirectAddress;
        pRedirect = findMatchingRegion(pThis, &redirectAddress, 1);
        uint32_t maxSize = pRedirect->size - (pRegion->redirectAddress - pRedirect->baseAddress);
        if (size > maxSize)
            pRegion->size = maxSize;
        else
            pRegion->size = size;
        pRegion->pRedirect = pRedirect;
        pRegion->pData = throwingZeroedMalloc(size);
        pRegion->isAlias = 1;
        pRegion->isReadOnly = pRedirect->isReadOnly;
        addRegionToTail(pThis, pRegion);
    }
    __catch
    {
        freeRegion(pRegion);
        __rethrow;
    }
}


void MemorySim_MakeRegionReadOnly(IMemory* pMemory, uint32_t baseAddress)
{
    MemorySim* pThis = (MemorySim*)pMemory;
    MemoryRegion* pRegion = findMatchingRegion(pThis, &baseAddress, 1);
    pRegion->isReadOnly = 1;
    allocateReadCountArrayForReadOnlyRegion(pRegion);
}

static MemoryRegion* findMatchingRegion(MemorySim* pThis, uint32_t* pAddress, uint32_t size)
{
    uint32_t address = *pAddress;
    MemoryRegion* pCurr = pThis->pHeadRegion;

    while (pCurr)
    {
        MemoryRegion* pNext = pCurr->pNext;
        if (address >= pCurr->baseAddress && (uint64_t)address + size <= (uint64_t)pCurr->baseAddress + pCurr->size)
        {
            if (pCurr->isAlias)
            {
                *pAddress = (address - pCurr->baseAddress) + pCurr->redirectAddress;
                return pCurr->pRedirect;
            }
            return pCurr;
        }
        pCurr = pNext;
    }
    __throw(busErrorException);
}

static void allocateReadCountArrayForReadOnlyRegion(MemoryRegion* pRegion)
{
    uint32_t halfWordCount = pRegion->size / sizeof(uint16_t);
    pRegion->pReadCounts = throwingZeroedMalloc(halfWordCount * sizeof(uint32_t));
    pRegion->readCounts = halfWordCount;
}


__throws void MemorySim_CreateRegionsFromFlashImage(IMemory* pMemory, const void* pFlashImage, uint32_t flashImageSize)
{
    MemorySim*      pThis = (MemorySim*)pMemory;

    if (flashImageSize < sizeof(uint32_t))
        __throw(bufferOverrunException);

    MemorySim_CreateRegion(pMemory, FLASH_BASE_ADDRESS, flashImageSize);
    MemorySim_LoadFromFlashImage(pMemory, FLASH_BASE_ADDRESS, pFlashImage, flashImageSize);

    __try
    {
        uint32_t endRAMAddress;
        uint32_t baseRAMAddress;

        MemorySim_MakeRegionReadOnly(pMemory, FLASH_BASE_ADDRESS);
        endRAMAddress = *(uint32_t*)pFlashImage;
        baseRAMAddress =  endRAMAddress & RAM_ADDRESS_MASK;
        MemorySim_CreateRegion(pMemory, baseRAMAddress, endRAMAddress - baseRAMAddress);
    }
    __catch
    {
        freeLastRegion(pThis);
        __rethrow;
    }
}

__throws void MemorySim_LoadFromFlashImage(IMemory* pMemory, uint32_t baseAddress, const void* pFlashImage, uint32_t flashImageSize)
{
    const uint32_t* pSrcWord = (uint32_t*)pFlashImage;
    uint32_t        address = baseAddress;
    const uint8_t*  pSrcByte;

    while (flashImageSize > sizeof(uint32_t))
    {
        load32(pMemory, address, *pSrcWord++);
        address += sizeof(uint32_t);
        flashImageSize -= sizeof(uint32_t);
    }

    pSrcByte = (const uint8_t*)pSrcWord;
    while (flashImageSize--)
        load8(pMemory, address++, *pSrcByte++);
}

static void load32(IMemory* pMemory, uint32_t address, uint32_t value)
{
    *(uint32_t*)getDataPointer((MemorySim*)pMemory, address, sizeof(uint32_t), LOADING, DISABLE_WATCHPOINT_CHECK) = value;
}

static void load8(IMemory* pMemory, uint32_t address, uint8_t value)
{
    *(uint8_t*)getDataPointer((MemorySim*)pMemory, address, sizeof(uint8_t), LOADING, DISABLE_WATCHPOINT_CHECK) = value;
}

static void freeLastRegion(MemorySim* pThis)
{
    MemoryRegion* pPrev = NULL;
    MemoryRegion* pCurr = pThis->pHeadRegion;

    while (pCurr && pCurr->pNext)
    {
        pPrev = pCurr;
        pCurr = pCurr->pNext;
    }
    if (!pPrev)
        pThis->pHeadRegion = NULL;
    else
        pPrev->pNext = NULL;
    pThis->pTailRegion = pPrev;
    freeRegion(pCurr);
}


const char* MemorySim_GetMemoryMapXML(IMemory* pMemory)
{
    static const char xmlExampleLine[] = "<memory type=\"flash\" start=\"0x00000000\" length=\"0xFFFFFFFF\"> <property name=\"blocksize\">1</property></memory>";
    MemorySim*        pThis = (MemorySim*)pMemory;
    size_t            regionCount = countRegions(pThis);
    size_t            allocSize = sizeof(g_xmlHeader) + sizeof(g_xmlTrailer) + regionCount * sizeof(xmlExampleLine);
    SizedBuffer       buffer;

    allocateMemoryMapXML(pThis, allocSize);
    buffer.pBuffer = pThis->pMemoryMapXML;
    buffer.size = allocSize;

    appendMemoryMapXmlHeader(pThis, &buffer);
    appendMemoryMapRegions(pThis, &buffer);
    appendMemoryMapXmlTrailer(pThis, &buffer);

    return pThis->pMemoryMapXML;
}

static size_t countRegions(MemorySim* pThis)
{
    size_t        count = 0;
    MemoryRegion* pCurr = pThis->pHeadRegion;
    while (pCurr)
    {
        count++;
        pCurr = pCurr->pNext;
    }

    return count;
}

static void allocateMemoryMapXML(MemorySim* pThis, size_t allocSize)
{
    char* pRealloc = realloc(pThis->pMemoryMapXML, allocSize);
    if (!pRealloc)
        __throw(outOfMemoryException);
    pThis->pMemoryMapXML = pRealloc;
}

static void appendMemoryMapXmlHeader(MemorySim* pThis, SizedBuffer* pBuffer)
{
    assert (pBuffer->size >= sizeof(g_xmlHeader));
    memcpy(pBuffer->pBuffer, g_xmlHeader, sizeof(g_xmlHeader) - 1);
    pBuffer->pBuffer += sizeof(g_xmlHeader) - 1;
    pBuffer->size -= sizeof(g_xmlHeader) - 1;
}

static void appendMemoryMapRegions(MemorySim* pThis, SizedBuffer* pBuffer)
{
    int           bytesUsed = 0;
    MemoryRegion* pCurr = pThis->pHeadRegion;

    while (pCurr)
    {
        bytesUsed = snprintf(pBuffer->pBuffer, pBuffer->size,
                             "<memory type=\"%s\" start=\"0x%X\" length=\"0x%X\">%s</memory>",
                             pCurr->isReadOnly ? "flash" : "ram",
                             pCurr->baseAddress,
                             pCurr->size,
                             pCurr->isReadOnly ? " <property name=\"blocksize\">1</property>" : "");
        assert (bytesUsed < (int)pBuffer->size);
        pBuffer->pBuffer += bytesUsed;
        pBuffer->size -= bytesUsed;

        pCurr = pCurr->pNext;
    }
}

static void appendMemoryMapXmlTrailer(MemorySim* pThis, SizedBuffer* pBuffer)
{
    assert (pBuffer->size >= sizeof(g_xmlTrailer));
    memcpy(pBuffer->pBuffer, g_xmlTrailer, sizeof(g_xmlTrailer));
    pBuffer->pBuffer += sizeof(g_xmlTrailer);
    pBuffer->size -= sizeof(g_xmlTrailer);
}


__throws void* MemorySim_MapSimulatedAddressToHostAddressForWrite(IMemory* pMemory, uint32_t address, uint32_t size)
{
    return getDataPointer((MemorySim*)pMemory, address, size, WRITING, DISABLE_WATCHPOINT_CHECK);
}


__throws const void* MemorySim_MapSimulatedAddressToHostAddressForRead(IMemory* pMemory, uint32_t address, uint32_t size)
{
    return getDataPointer((MemorySim*)pMemory, address, size, READING, DISABLE_WATCHPOINT_CHECK);
}


__throws uint32_t MemorySim_GetFlashReadCount(IMemory* pMemory, uint32_t address)
{
    MemorySim*    pThis = (MemorySim*)pMemory;
    MemoryRegion* pRegion = findMatchingRegion(pThis, &address, 2);
    uint32_t      regionOffset = address - pRegion->baseAddress;

    if (!pRegion->isReadOnly)
        __throw(busErrorException);
    return pRegion->pReadCounts[regionOffset / sizeof(uint16_t)];
}


__throws void MemorySim_SetHardwareBreakpoint(IMemory* pMemory, uint32_t address, uint32_t size)
{
    setWatchpoint(pMemory, address, size, WATCHPOINT_BREAKPOINT);
}

static void setWatchpoint(IMemory* pMemory, uint32_t address, uint32_t size, WatchpointType type)
{
    MemoryRegion* pRegion = findMatchingRegion((MemorySim*)pMemory, &address, size);
    uint32_t      endAddress = address + size;
    Watchpoint    watchpoint = {type, address, endAddress};
    uint32_t      i;
    MatchResult   match = findMatchingOrHigherWatchpoint(pRegion, &watchpoint, &i);

    if (match == FOUND)
        return;
    growWatchpointArrayIfNeeded(pRegion, pRegion->watchpointCount + 1);
    memmove(&pRegion->pWatchpoints[i+1],
            &pRegion->pWatchpoints[i],
            sizeof(*pRegion->pWatchpoints) * (pRegion->watchpointCount - i));
    pRegion->pWatchpoints[i] = watchpoint;
    pRegion->watchpointCount++;
}


static MatchResult findMatchingOrHigherWatchpoint(MemoryRegion* pRegion, Watchpoint* pKey, uint32_t* pIndex)
{
    uint32_t           i;

    for (i = 0 ; i < pRegion->watchpointCount ; i++)
    {
        int result = compareWatchpoints(pKey, &pRegion->pWatchpoints[i]);
        if (result == 0)
        {
            *pIndex = i;
            return FOUND;
        }
        else if (result < 0)
        {
            *pIndex = i;
            return FOUND_HIGHER;
        }
    }
    *pIndex = i;
    return NOT_FOUND;
}

static int compareWatchpoints(const void* pvKey, const void* pvCurr)
{
    const Watchpoint* pKey = (const Watchpoint*)pvKey;
    const Watchpoint* pCurr = (const Watchpoint*)pvCurr;

    if (watchpointsMatch(pKey, pCurr))
        return 0;
    else if (pKey->startAddress < pCurr->startAddress)
        return -1;
    else
        return 1;
}

static int watchpointsMatch(const Watchpoint* p1, const Watchpoint* p2)
{
    return 0 == memcmp(p1, p2, sizeof(*p1));
}

static void growWatchpointArrayIfNeeded(MemoryRegion* pRegion, uint32_t requiredSize)
{
    if (requiredSize > pRegion->watchpointAlloc)
    {
        Watchpoint* pRealloc = realloc(pRegion->pWatchpoints, sizeof(*pRegion->pWatchpoints) * requiredSize);
        if (!pRealloc)
            __throw(outOfMemoryException);
        pRegion->pWatchpoints = pRealloc;
        pRegion->watchpointAlloc = requiredSize;
    }
}


__throws void MemorySim_ClearHardwareBreakpoint(IMemory* pMemory, uint32_t address, uint32_t size)
{
    clearWatchpoint(pMemory, address, size, WATCHPOINT_BREAKPOINT);
}

static void clearWatchpoint(IMemory* pMemory, uint32_t address, uint32_t size, WatchpointType type)
{
    MemoryRegion* pRegion = findMatchingRegion((MemorySim*)pMemory, &address, size);
    uint32_t      endAddress = address + size;
    Watchpoint    watchpoint = {type, address, endAddress};
    uint32_t      i;
    MatchResult   match = findMatchingOrHigherWatchpoint(pRegion, &watchpoint, &i);

    if (match != FOUND)
        return;
    memmove(&pRegion->pWatchpoints[i],
            &pRegion->pWatchpoints[i+1],
            sizeof(*pRegion->pWatchpoints) * (pRegion->watchpointCount - i - 1));
    pRegion->watchpointCount--;
}


__throws void MemorySim_SetHardwareWatchpoint(IMemory* pMemory, uint32_t address, uint32_t size, WatchpointType type)
{
    setWatchpoint(pMemory, address, size, type);
}


__throws void MemorySim_ClearHardwareWatchpoint(IMemory* pMemory, uint32_t address, uint32_t size, WatchpointType type)
{
    clearWatchpoint(pMemory, address, size, type);
}


int  MemorySim_WasWatchpointEncountered(IMemory* pMemory)
{
    MemorySim* pThis = (MemorySim*)pMemory;
    int        wasEncountered = pThis->watchpointEncountered;

    pThis->watchpointEncountered = 0;
    return wasEncountered;
}



/* IMemory interface methods */
static uint32_t read32(IMemory* pMemory, uint32_t address)
{
    return *(uint32_t*)getDataPointer((MemorySim*)pMemory, address, sizeof(uint32_t), READING, ENABLE_WATCHPOINT_CHECK);
}

static uint16_t read16(IMemory* pMemory, uint32_t address)
{
    return *(uint16_t*)getDataPointer((MemorySim*)pMemory, address, sizeof(uint16_t), READING, ENABLE_WATCHPOINT_CHECK);
}

static uint8_t read8(IMemory* pMemory, uint32_t address)
{
    return *(uint8_t*)getDataPointer((MemorySim*)pMemory, address, sizeof(uint8_t), READING, ENABLE_WATCHPOINT_CHECK);
}

static void write32(IMemory* pMemory, uint32_t address, uint32_t value)
{
    *(uint32_t*)getDataPointer((MemorySim*)pMemory, address, sizeof(uint32_t), WRITING, ENABLE_WATCHPOINT_CHECK) = value;
}

static void write16(IMemory* pMemory, uint32_t address, uint16_t value)
{
    *(uint16_t*)getDataPointer((MemorySim*)pMemory, address, sizeof(uint16_t), WRITING, ENABLE_WATCHPOINT_CHECK) = value;
}

static void write8(IMemory* pMemory, uint32_t address, uint8_t value)
{
    *(uint8_t*)getDataPointer((MemorySim*)pMemory, address, sizeof(uint8_t), WRITING, ENABLE_WATCHPOINT_CHECK) = value;
}


static void* getDataPointer(MemorySim* pThis, uint32_t address, uint32_t size, AccessType type, int checkWatchpoints)
{
    MemoryRegion* pRegion = findMatchingRegion(pThis, &address, size);
    uint32_t regionOffset = address - pRegion->baseAddress;
    if (type == WRITING && pRegion->isReadOnly)
        __throw(busErrorException);
    if (type == READING && size == sizeof(uint16_t) && pRegion->pReadCounts)
        pRegion->pReadCounts[regionOffset / sizeof(uint16_t)]++;
    if (checkWatchpoints)
        checkForBreakWatchPoint(pThis, pRegion, address, size, type);
    return pRegion->pData + regionOffset;
}

static void checkForBreakWatchPoint(MemorySim* pThis,
                                    MemoryRegion* pRegion,
                                    uint32_t address, uint32_t size, AccessType type)
{
    uint32_t endAddress = address + size;
    uint32_t i;

    for (i = 0 ; i < pRegion->watchpointCount ; i++)
    {
        Watchpoint* pWatchpoint = &pRegion->pWatchpoints[i];

        if ((type & pWatchpoint->type) == 0)
            continue;
        if (pWatchpoint->type == WATCHPOINT_BREAKPOINT)
        {
            if (size == sizeof(uint16_t) && accessInRange(pWatchpoint, address, endAddress))
                __throw(hardwareBreakpointException);
        }
        else if (accessInRange(pWatchpoint, address, endAddress))
        {
            pThis->watchpointEncountered++;
        }
        else if (pWatchpoint->startAddress > address)
        {
            return;
        }
    }
}

static int accessInRange(Watchpoint* pWatchpoint, uint32_t startAddress, uint32_t endAddress)
{
    return startAddress >= pWatchpoint->startAddress && endAddress <= pWatchpoint->endAddress;
}
