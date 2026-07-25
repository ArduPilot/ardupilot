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
#ifndef _ELF_LOAD_H_
#define _ELF_LOAD_H_

#include <IMemory.h>
#include <try_catch.h>

__throws void ElfLoad_FromMemory(IMemory* pMemory, const void* pElf, size_t elfSize);

#endif /* _ELF_LOAD_H_ */
