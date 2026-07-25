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
#ifndef _CRASHDEBUG_COMMANDLINE_H_
#define _CRASHDEBUG_COMMANDLINE_H_

#include <mriPlatform.h>
#include <IMemory.h>
#include <try_catch.h>

typedef struct CrashDebugCommandLine
{
    const char*     pElfFilename;
    const char*     pBinFilename;
    const char*     pDumpFilename;
    IMemory*        pMemory;
    RegisterContext context;
    uint32_t        baseAddress;
} CrashDebugCommandLine;


__throws void CrashDebugCommandLine_Init(CrashDebugCommandLine* pThis, int argc, const char** argv);
         void CrashDebugCommandLine_Uninit(CrashDebugCommandLine* pThis);


#endif /* _CRASHDEBUG_COMMANDLINE_H_ */
