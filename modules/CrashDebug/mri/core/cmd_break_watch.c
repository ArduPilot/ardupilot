/* Copyright 2014 Adam Green (http://mbed.org/users/AdamGreen/)

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
/* Handlers for gdb breakpoint and watchpoint commands. */
#include "platforms.h"
#include "core.h"
#include "mri.h"
#include "cmd_common.h"
#include "cmd_break_watch.h"

typedef struct
{
    uint32_t address;
    uint32_t kind;
    char     type;
} BreakpointWatchpointArguments;

static void parseBreakpointWatchpointCommandArguments(BreakpointWatchpointArguments* pArguments);
static void handleHardwareBreakpointSetCommand(BreakpointWatchpointArguments* pArguments);
static void handleBreakpointWatchpointException(void);
static void handleWatchpointSetCommand(PlatformWatchpointType type, BreakpointWatchpointArguments* pArguments);
/* Handle the '"Z*" commands used by gdb to set hardware breakpoints/watchpoints.

    Command Format:     Z*,AAAAAAAA,K
    Response Format:    OK
    Where * is 1 for hardware breakpoint.
               2 for write watchpoint.
               3 for read watchpoint.
               4 for read/write watchpoint.
          AAAAAAAA is the hexadecimal representation of the address on which the breakpoint should be set.
          K is either 2: 16-bit Thumb instruction.
                      3: 32-bit Thumb2 instruction.
                      4: 32-bit ARM insruction.
                      value: byte size for data watchpoint.
*/
uint32_t HandleBreakpointWatchpointSetCommand(void)
{
    BreakpointWatchpointArguments  arguments;
    
    __try
    {
        parseBreakpointWatchpointCommandArguments(&arguments);
    }
    __catch
    {
        PrepareStringResponse(MRI_ERROR_INVALID_ARGUMENT);
        return 0;
    }
    
    switch(arguments.type)
    {
    case '1':
        handleHardwareBreakpointSetCommand(&arguments);
        break;
    case '2':
        handleWatchpointSetCommand(MRI_PLATFORM_WRITE_WATCHPOINT, &arguments);
        break;
    case '3':
        handleWatchpointSetCommand(MRI_PLATFORM_READ_WATCHPOINT, &arguments);
        break;
    case '4':
        handleWatchpointSetCommand(MRI_PLATFORM_READWRITE_WATCHPOINT, &arguments);
        break;
    default:
        PrepareEmptyResponseForUnknownCommand();
        break;
    }
    
    return 0;
}

static void parseBreakpointWatchpointCommandArguments(BreakpointWatchpointArguments* pArguments)
{
    Buffer*    pBuffer = GetBuffer();
    
    __try
    {
        __throwing_func( pArguments->type = Buffer_ReadChar(pBuffer) );
        __throwing_func( ThrowIfNextCharIsNotEqualTo(pBuffer, ',') );
        __throwing_func( pArguments->address = ReadUIntegerArgument(pBuffer) );
        __throwing_func( ThrowIfNextCharIsNotEqualTo(pBuffer, ',') );
        __throwing_func( pArguments->kind = ReadUIntegerArgument(pBuffer) );
    }
    __catch
    {
        __rethrow;
    }
}

static void handleHardwareBreakpointSetCommand(BreakpointWatchpointArguments* pArguments)
{
    __try
    {
        Platform_SetHardwareBreakpoint(pArguments->address, pArguments->kind);
    }
    __catch
    {
        handleBreakpointWatchpointException();
        return;
    }
    PrepareStringResponse("OK");
}

static void handleBreakpointWatchpointException(void)
{
    switch(getExceptionCode())
    {
    case invalidArgumentException:
        PrepareStringResponse(MRI_ERROR_INVALID_ARGUMENT);
        break;
    case exceededHardwareResourcesException:
    default:
        PrepareStringResponse(MRI_ERROR_NO_FREE_BREAKPOINT);
        break;
    }
    
    return;
}

static void handleWatchpointSetCommand(PlatformWatchpointType type, BreakpointWatchpointArguments* pArguments)
{
    uint32_t        address = pArguments->address;
    uint32_t        size = pArguments->kind;

    __try
    {
        Platform_SetHardwareWatchpoint(address, size, type);
    }
    __catch
    {
        handleBreakpointWatchpointException();
        return;
    }
    PrepareStringResponse("OK");
}


static void handleHardwareBreakpointRemoveCommand(BreakpointWatchpointArguments* pArguments);
static void handleWatchpointRemoveCommand(PlatformWatchpointType type, BreakpointWatchpointArguments* pArguments);
/* Handle the '"z*" commands used by gdb to remove hardware breakpoints/watchpoints.

    Command Format:     z*,AAAAAAAA,K
    Response Format:    OK
    Where * is 1 for hardware breakpoint.
               2 for write watchpoint.
               3 for read watchpoint.
               4 for read/write watchpoint.
          AAAAAAAA is the hexadecimal representation of the address on which the breakpoint should be removed.
          K is either 2: 16-bit Thumb instruction.
                      3: 32-bit Thumb2 instruction.
                      4: 32-bit ARM insruction.
                      value: byte size for data watchpoint.
*/
uint32_t HandleBreakpointWatchpointRemoveCommand(void)
{
    BreakpointWatchpointArguments  arguments;
    
    __try
    {
        parseBreakpointWatchpointCommandArguments(&arguments);
    }
    __catch
    {
        PrepareStringResponse(MRI_ERROR_INVALID_ARGUMENT);
        return 0;
    }
    
    switch(arguments.type)
    {
    case '1':
        handleHardwareBreakpointRemoveCommand(&arguments);
        break;
    case '2':
        handleWatchpointRemoveCommand(MRI_PLATFORM_WRITE_WATCHPOINT, &arguments);
        break;
    case '3':
        handleWatchpointRemoveCommand(MRI_PLATFORM_READ_WATCHPOINT, &arguments);
        break;
    case '4':
        handleWatchpointRemoveCommand(MRI_PLATFORM_READWRITE_WATCHPOINT, &arguments);
        break;
    default:
        PrepareEmptyResponseForUnknownCommand();
        break;
    }
    
    return 0;
}

static void handleHardwareBreakpointRemoveCommand(BreakpointWatchpointArguments* pArguments)
{
    __try
    {
        Platform_ClearHardwareBreakpoint(pArguments->address, pArguments->kind);
    }
    __catch
    {
        handleBreakpointWatchpointException();
        return;
    }
    PrepareStringResponse("OK");
}

static void handleWatchpointRemoveCommand(PlatformWatchpointType type, BreakpointWatchpointArguments* pArguments)
{
    uint32_t        address = pArguments->address;
    uint32_t        size = pArguments->kind;

    __try
    {
        Platform_ClearHardwareWatchpoint(address, size, type);
    }
    __catch
    {
        handleBreakpointWatchpointException();
        return;
    }
    PrepareStringResponse("OK");
}
