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
#include <CrashDebugCommandLine.h>
#include <mriPlatform.h>
#include <StandardIComm.h>
#include <stdio.h>


int main(int argc, const char** argv)
{
    volatile int          returnValue = 0;
    IComm* volatile       pComm = NULL;
    CrashDebugCommandLine commandLine;

    __try
    {
        CrashDebugCommandLine_Init(&commandLine, argc-1, argv+1);
        pComm = StandardIComm_Init();
        mriPlatform_Init(&commandLine.context, commandLine.pMemory);
        mriPlatform_Run(pComm);
    }
    __catch
    {
        switch (getExceptionCode())
        {
        case fileException:
        case elfFormatException:
        case invalidArgumentException:
            // Appropriate error message will already have been displayed by CrashDebugCommandLine module.
            break;
        default:
            fprintf(stderr, "Encountered unexpected error: %d\n", getExceptionCode());
            break;
        }
        returnValue = -1;
    }
    StandardIComm_Uninit(pComm);
    CrashDebugCommandLine_Uninit(&commandLine);

    return returnValue;
}
