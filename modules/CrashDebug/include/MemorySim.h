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
#ifndef _MEMORY_SIM_H_
#define _MEMORY_SIM_H_


#include <IMemory.h>


/* MemorySim_CreateRegionsFromFlashImage() will place read-only FLASH image contents at this address. */
#define FLASH_BASE_ADDRESS 0x00000000
/* MemorySim_CreateRegionsFromFlashImage() will create a read-write RAM region based on the initial stack pointer value
   in the first word of the FLASH image.  The region will start at initial_stack_pointer & RAM_ADDRESS_MASK. */
#define RAM_ADDRESS_MASK   0xF0000000

typedef enum WatchpointType
{
    WATCHPOINT_READ       = 1,
    WATCHPOINT_WRITE      = 2,
    WATCHPOINT_READ_WRITE = 3
} WatchpointType;


IMemory*                     MemorySim_Init(void);
void                         MemorySim_Uninit(IMemory* pMemory);
__throws void                MemorySim_CreateRegion(IMemory* pMemory, uint32_t baseAddress, uint32_t size);
__throws void                MemorySim_CreateAlias(IMemory* pMemory, uint32_t aliasAddress, uint32_t redirectAddress, uint32_t size);
void                         MemorySim_MakeRegionReadOnly(IMemory* pMemory, uint32_t baseAddress);
__throws void                MemorySim_LoadFromFlashImage(IMemory* pMemory, uint32_t baseAddress, const void* pFlashImage, uint32_t flashImageSize);
__throws void                MemorySim_CreateRegionsFromFlashImage(IMemory* pMemory, const void* pFlashImage, uint32_t flashImageSize);
__throws const char*         MemorySim_GetMemoryMapXML(IMemory* pMemory);
__throws void*               MemorySim_MapSimulatedAddressToHostAddressForWrite(IMemory* pMemory, uint32_t address, uint32_t size);
__throws const void*         MemorySim_MapSimulatedAddressToHostAddressForRead(IMemory* pMemory, uint32_t address, uint32_t size);
__throws uint32_t            MemorySim_GetFlashReadCount(IMemory* pMemory, uint32_t address);

__throws void MemorySim_SetHardwareBreakpoint(IMemory* pMemory, uint32_t address, uint32_t size);
__throws void MemorySim_ClearHardwareBreakpoint(IMemory* pMemory, uint32_t address, uint32_t size);

__throws void MemorySim_SetHardwareWatchpoint(IMemory* pMemory, uint32_t address, uint32_t size, WatchpointType type);
__throws void MemorySim_ClearHardwareWatchpoint(IMemory* pMemory, uint32_t address, uint32_t size, WatchpointType type);
         int  MemorySim_WasWatchpointEncountered(IMemory* pMemory);


#endif /* _MEMORY_SIM_H_ */
