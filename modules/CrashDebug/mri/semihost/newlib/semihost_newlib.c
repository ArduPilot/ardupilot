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
/* Semihost functionality for redirecting stdin/stdout/stderr I/O to the GNU console. */
#include <string.h>
#include <mri.h>
#include <semihost.h>
#include <cmd_file.h>


static int handleNewlibSemihostWriteRequest(PlatformSemihostParameters* pSemihostParameters);
static int handleNewlibSemihostReadRequest(PlatformSemihostParameters* pSemihostParameters);
static int handleNewlibSemihostOpenRequest(PlatformSemihostParameters* pSemihostParameters);
static int handleNewlibSemihostUnlinkRequest(PlatformSemihostParameters* pSemihostParameters);
static int handleNewlibSemihostLSeekRequest(PlatformSemihostParameters* pSemihostParameters);
static int handleNewlibSemihostCloseRequest(PlatformSemihostParameters* pSemihostParameters);
static int handleNewlibSemihostFStatRequest(PlatformSemihostParameters* pSemihostParameters);
static int handleNewlibSemihostStatRequest(PlatformSemihostParameters* pSemihostParameters);
static int handleNewlibSemihostRenameRequest(PlatformSemihostParameters* pSemihostParameters);
int Semihost_HandleNewlibSemihostRequest(PlatformSemihostParameters* pSemihostParameters)
{
    uint32_t semihostOperation;

    semihostOperation = Platform_GetProgramCounter() | 1;
    if (semihostOperation == (uint32_t)__mriNewlib_SemihostWrite)
        return handleNewlibSemihostWriteRequest(pSemihostParameters);
    else if (semihostOperation == (uint32_t)__mriNewlib_SemihostRead)
        return handleNewlibSemihostReadRequest(pSemihostParameters);
    else if (semihostOperation == (uint32_t)__mriNewLib_SemihostOpen)
        return handleNewlibSemihostOpenRequest(pSemihostParameters);
    else if (semihostOperation == (uint32_t)__mriNewLib_SemihostUnlink)
        return handleNewlibSemihostUnlinkRequest(pSemihostParameters);
    else if (semihostOperation == (uint32_t)__mriNewlib_SemihostLSeek)
        return handleNewlibSemihostLSeekRequest(pSemihostParameters);
    else if (semihostOperation == (uint32_t)__mriNewlib_SemihostClose)
        return handleNewlibSemihostCloseRequest(pSemihostParameters);
    else if (semihostOperation == (uint32_t)__mriNewlib_SemihostFStat)
        return handleNewlibSemihostFStatRequest(pSemihostParameters);
    else if (semihostOperation == (uint32_t)__mriNewLib_SemihostStat)
        return handleNewlibSemihostStatRequest(pSemihostParameters);
    else if (semihostOperation == (uint32_t)__mriNewLib_SemihostRename)
        return handleNewlibSemihostRenameRequest(pSemihostParameters);
    else
        return 0;
}

static int handleNewlibSemihostWriteRequest(PlatformSemihostParameters* pSemihostParameters)
{
    TransferParameters parameters;

    parameters.fileDescriptor = pSemihostParameters->parameter1;
    parameters.bufferAddress = pSemihostParameters->parameter2;
    parameters.bufferSize = pSemihostParameters->parameter3;
    
    return IssueGdbFileWriteRequest(&parameters);
}

static int handleNewlibSemihostReadRequest(PlatformSemihostParameters* pSemihostParameters)
{
    TransferParameters parameters;

    parameters.fileDescriptor = pSemihostParameters->parameter1;
    parameters.bufferAddress = pSemihostParameters->parameter2;
    parameters.bufferSize = pSemihostParameters->parameter3;
    
    return IssueGdbFileReadRequest(&parameters);
}

static int handleNewlibSemihostOpenRequest(PlatformSemihostParameters* pSemihostParameters)
{
    OpenParameters parameters;

    parameters.filenameAddress = pSemihostParameters->parameter1;
    parameters.filenameLength = strlen((const char*)parameters.filenameAddress) + 1;
    parameters.flags = pSemihostParameters->parameter2;
    parameters.mode = pSemihostParameters->parameter3;
    
    return IssueGdbFileOpenRequest(&parameters);
}

static int handleNewlibSemihostUnlinkRequest(PlatformSemihostParameters* pSemihostParameters)
{
    RemoveParameters parameters;

    parameters.filenameAddress = pSemihostParameters->parameter1;    
    parameters.filenameLength = strlen((const char*)parameters.filenameAddress) + 1;
    
    return IssueGdbFileUnlinkRequest(&parameters);
}

static int handleNewlibSemihostLSeekRequest(PlatformSemihostParameters* pSemihostParameters)
{
    SeekParameters parameters;

    parameters.fileDescriptor = pSemihostParameters->parameter1;    
    parameters.offset = pSemihostParameters->parameter2;
    parameters.whence = pSemihostParameters->parameter3;
    
    return IssueGdbFileSeekRequest(&parameters);
}

static int handleNewlibSemihostCloseRequest(PlatformSemihostParameters* pSemihostParameters)
{
    return IssueGdbFileCloseRequest(pSemihostParameters->parameter1);
}

static int handleNewlibSemihostFStatRequest(PlatformSemihostParameters* pSemihostParameters)
{
    return IssueGdbFileFStatRequest(pSemihostParameters->parameter1, pSemihostParameters->parameter2);
}

static int handleNewlibSemihostStatRequest(PlatformSemihostParameters* pSemihostParameters)
{
    StatParameters parameters;

    parameters.filenameAddress = pSemihostParameters->parameter1;
    parameters.filenameLength = strlen((const char*) parameters.filenameAddress) + 1;
    return IssueGdbFileStatRequest(&parameters);
}

static int handleNewlibSemihostRenameRequest(PlatformSemihostParameters* pSemihostParameters)
{
    RenameParameters parameters;

    parameters.origFilenameAddress = pSemihostParameters->parameter1;
    parameters.origFilenameLength = strlen((const char*)parameters.origFilenameAddress) + 1;
    parameters.newFilenameAddress = pSemihostParameters->parameter2;
    parameters.newFilenameLength = strlen((const char*)parameters.newFilenameAddress) + 1;
    return IssueGdbFileRenameRequest(&parameters);
}
