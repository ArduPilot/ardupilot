/*  Copyright (C) 2014  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
#ifndef _IMEMORY_H_
#define _IMEMORY_H_

#include <stdint.h>
#include <try_catch.h>


typedef struct IMemory IMemory;

typedef struct IMemoryVTable
{
    __throws uint32_t (* read32)(IMemory* pThis, uint32_t address);
    __throws uint16_t (* read16)(IMemory* pThis, uint32_t address);
    __throws uint8_t  (* read8)(IMemory* pThis, uint32_t address);

    __throws void (* write32)(IMemory* pThis, uint32_t address, uint32_t value);
    __throws void (* write16)(IMemory* pThis, uint32_t address, uint16_t value);
    __throws void (* write8)(IMemory* pThis, uint32_t address, uint8_t value);
} IMemoryVTable;

struct IMemory
{
    IMemoryVTable* pVTable;
};


static __throws __inline uint32_t IMemory_Read32(IMemory* pThis, uint32_t address)
{
    return pThis->pVTable->read32(pThis, address);
}

static __throws __inline uint16_t IMemory_Read16(IMemory* pThis, uint32_t address)
{
    return pThis->pVTable->read16(pThis, address);
}

static __throws __inline uint8_t IMemory_Read8(IMemory* pThis, uint32_t address)
{
    return pThis->pVTable->read8(pThis, address);
}

static __throws __inline void IMemory_Write32(IMemory* pThis, uint32_t address, uint32_t value)
{
    pThis->pVTable->write32(pThis, address, value);
}

static __throws __inline void IMemory_Write16(IMemory* pThis, uint32_t address, uint16_t value)
{
    pThis->pVTable->write16(pThis, address, value);
}

static __throws __inline void IMemory_Write8(IMemory* pThis, uint32_t address, uint8_t value)
{
    pThis->pVTable->write8(pThis, address, value);
}


#endif /* _IMEMORY_H_ */
