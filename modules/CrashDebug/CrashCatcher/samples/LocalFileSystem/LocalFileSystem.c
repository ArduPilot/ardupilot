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
#include <CrashCatcher.h>


/* Low level declarations for LocalFileSystem taken from mbed headers. */
#define OPEN_W          4

typedef int FILEHANDLE;

FILEHANDLE semihost_open(const char* name, int openmode);
int        semihost_close(FILEHANDLE fh);
int        semihost_write(FILEHANDLE fh, const unsigned char* buffer, unsigned int length, int mode);


static FILEHANDLE g_coreDumpFile = -1;


/* Forward Declarations */
static void dumpHalfWords(const uint16_t* pMemory, size_t elementCount);
static void dumpWords(const uint32_t* pMemory, size_t elementCount);
static void infiniteLoop(void);


void CrashCatcher_DumpStart(const CrashCatcherInfo* pInfo)
{
    (void)pInfo;
    g_coreDumpFile = semihost_open("crash.dmp", OPEN_W);
}


void CrashCatcher_DumpMemory(const void* pvMemory, CrashCatcherElementSizes elementSize, size_t elementCount)
{
    if (g_coreDumpFile < 0)
        return;
    switch (elementSize)
    {
    case CRASH_CATCHER_BYTE:
        semihost_write(g_coreDumpFile, pvMemory, elementCount, 0);
        break;
    case CRASH_CATCHER_HALFWORD:
        dumpHalfWords(pvMemory, elementCount);
        break;
    case CRASH_CATCHER_WORD:
        dumpWords(pvMemory, elementCount);
        break;
    }
}

static void dumpHalfWords(const uint16_t* pMemory, size_t elementCount)
{
    size_t i;
    for (i = 0 ; i < elementCount ; i++)
    {
        uint16_t val = *pMemory++;
        semihost_write(g_coreDumpFile, (void*)&val, sizeof(val), 0);
    }
}

static void dumpWords(const uint32_t* pMemory, size_t elementCount)
{
    size_t i;
    for (i = 0 ; i < elementCount ; i++)
    {
        uint32_t val = *pMemory++;
        semihost_write(g_coreDumpFile, (void*)&val, sizeof(val), 0);
    }
}


CrashCatcherReturnCodes CrashCatcher_DumpEnd(void)
{
    if (g_coreDumpFile < 0)
        infiniteLoop();

    semihost_close(g_coreDumpFile);
    g_coreDumpFile = -1;
    infiniteLoop();
    return CRASH_CATCHER_TRY_AGAIN;
}

static void infiniteLoop(void)
{
    while (1)
    {
    }
}
