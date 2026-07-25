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
#ifndef _CRASH_CATCHER_H_
#define _CRASH_CATCHER_H_


#include <stdint.h>
#include <stdlib.h>


#define TRUE  1
#define FALSE 0


/* The crash dump start with a four byte header.  The first two bytes are "cC", the third byte is the major version
   number, and the fourth bytes is the minor version number. */
#define CRASH_CATCHER_SIGNATURE_BYTE0 'c'
#define CRASH_CATCHER_SIGNATURE_BYTE1 'C'
#define CRASH_CATCHER_VERSION_MAJOR   3
#define CRASH_CATCHER_VERSION_MINOR   0

/* The second word of the dump contains flags.  These are the allowed flags. */
/* Flag to indicate that 32 single-precision floating point registers and FPSCR will follow integer registers. */
#define CRASH_CATCHER_FLAGS_FLOATING_POINT (1 << 0)


/* This magic value will be found as the last word in a crash dump if the fault handler overflowed the stack while
   generating the crash dump. */
#define CRASH_CATCHER_STACK_SENTINEL 0xACCE55ED


/* Particulars of crash provided to CrashCatcher_DumpStart(). */
typedef struct
{
    /* The SP, Stack Pointer, contained this value at the time of the fault. */
    uint32_t    sp;
    /* Was this fault actually just a hardcoded breakpoint from which it is safe to continue. */
    int         isBKPT;
} CrashCatcherInfo;


/* The crash dump will have one of these entries for each memory region included in the dump file. */
typedef struct
{
    uint32_t                 startAddress;
    uint32_t                 endAddress;
} CrashCatcherMemoryRegionInfo;

/* Supported element sizes to be used with CrashCatcher_DumpMemory calls. */
typedef enum
{
    CRASH_CATCHER_BYTE = 1,
    CRASH_CATCHER_HALFWORD = 2,
    CRASH_CATCHER_WORD = 4
} CrashCatcherElementSizes;


/* Codes to be returned from an implementation's CrashCathcer_DumpEnd() handler. */
typedef enum
{
    /* Crash Catcher should loop around and try dumping again incase user missed it previous time. */
    CRASH_CATCHER_TRY_AGAIN = 0,
    /* Crash Catcher should exit and return to caller.  This is used during unit testing or if the crash cause was 
       actually a hardcoded breakpoint that should be logged and then execution continued. */
    CRASH_CATCHER_EXIT
} CrashCatcherReturnCodes;


/* An array of these structures is returned from CrashCatcher_GetMemoryRegions() to indicate what regions of memory
   should be dumped as part of the crash dump.  The last entry should contain a starting address of 0xFFFFFFFF to
   indicate that the end of the list has been encountered. */
typedef struct
{
    /* The first address of the element to be dumped for this region of memory. */
    /* The last region in the array return from CrashCatcher_GetMemoryRegions() must set this to 0xFFFFFFFF */
    uint32_t                 startAddress;
    /* Stop dumping the region once this address is encountered.  The dump isn't inclusive of this address. */
    /* It must be greater than startAddress. */
    uint32_t                 endAddress;
    /* This should be set to CRASH_CATCHER_BYTE except for peripheral registers which don't support 8-bit reads. */
    CrashCatcherElementSizes elementSize;
} CrashCatcherMemoryRegion;


#ifdef __cplusplus
extern "C"
{
#endif
/* The following functions must be provided by a specific dumping implementation.  The Core CrashCatcher calls these
   routines to have an implementation dump the bytes associated with the current crash. */

/* Called at the beginning of crash dump. You should provide an implementation which prepares for the dump by opening
   a dump file, prompting the user to begin a crash dump, or whatever makes sense for your scenario. */
void CrashCatcher_DumpStart(const CrashCatcherInfo* pInfo);

/* Called to obtain an array of regions in memory that should be dumped as part of the crash.  This will typically
   be all RAM regions that contain volatile data.  For some crash scenarios, a user may decide to also add peripheral
   registers of interest (ie. dump some ethernet registers when you are encountering crashes in the network stack.)
   If NULL is returned from this function, the core will only dump the registers. */
const CrashCatcherMemoryRegion* CrashCatcher_GetMemoryRegions(void);

/* Called to dump the next chunk of memory to the dump (this memory may point to register contents which has been copied
   to memory by CrashCatcher already.  The element size will be 8-bits, 16-bits, or 32-bits.  The implementation should
   use reads of the specified size since some memory locations may only support the indicated size. */
void CrashCatcher_DumpMemory(const void* pvMemory, CrashCatcherElementSizes elementSize, size_t elementCount);

/* Called at the end of crash dump. You should provide an implementation which cleans up at the end of dump. This could
   include closing a dump file, blinking LEDs, infinite looping, and/or returning CRASH_CATCHER_TRY_AGAIN if
   CrashCatcher should prepare to dump again incase user missed the first attempt. */
CrashCatcherReturnCodes CrashCatcher_DumpEnd(void);


/* The following functions must be provided by a hex dumping implementation. Such implementations will also have to
   implement the core CrashCatcher_GetMemoryRegions() API as well.  The HexDump version of CrashCatcher calls these
   routines to have an implementation query the user when they are ready for the dump to start and actually dump the
   hex data to the user a character at a time. */

/* Called to receive a character of data from the user.  Typically this is in response to a "Press any key" type of
   prompt to the user.  This function should be blocking. */
int CrashCatcher_getc(void);

/* Called to send a character of hex dump data to the user. */
void CrashCatcher_putc(int c);

#ifdef __cplusplus
}
#endif


/* Macros which can generate a few forms of crashes. */
#define CRASH_CATCHER_READ_FAULT()          (*(volatile unsigned int*)0xFFFFFFF0)
#define CRASH_CATCHER_WRITE_FAULT()         (*(volatile unsigned int*)0xFFFFFFF0 = 0x0)
#define CRASH_CATCHER_INVALID_INSTRUCTION() { __asm volatile (".word 0xDE00"); }

/* Macro used to insert hardcoded breakpoint into user's code. */
#define CRASH_CATCHER_BREAKPOINT()          { __asm volatile ("bkpt #0"); }

/* Macro used to make some globals writeable from unit tests but constant when running on ARM processors. */
#ifdef RUNNING_HOST_TESTS
    #define CRASH_CATCHER_TEST_WRITEABLE
#else
    #define CRASH_CATCHER_TEST_WRITEABLE static const
#endif


#endif /* _CRASH_CATCHER_H_ */
