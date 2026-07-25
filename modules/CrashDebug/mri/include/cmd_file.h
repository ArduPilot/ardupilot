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
/* Handling and issuing routines for gdb file commands. */
#ifndef _CMD_FILE_H_
#define _CMD_FILE_H_

#include <stdint.h>
#include "buffer.h"

typedef struct
{
    uint32_t        filenameAddress;
    uint32_t        filenameLength;
    uint32_t        flags;
    uint32_t        mode;
} OpenParameters;

typedef struct 
{
    uint32_t        fileDescriptor;
    uint32_t        bufferAddress;
    int32_t         bufferSize;
} TransferParameters;

typedef struct
{
    uint32_t        fileDescriptor;
    int32_t         offset;
    int32_t         whence;
} SeekParameters;

typedef struct
{
    uint32_t        filenameAddress;
    uint32_t        filenameLength;
} RemoveParameters;

typedef struct
{
    uint32_t        filenameAddress;
    uint32_t        filenameLength;
    uint32_t        fileStatBuffer;
} StatParameters;

typedef struct
{
    uint32_t        origFilenameAddress;
    uint32_t        origFilenameLength;
    uint32_t        newFilenameAddress;
    uint32_t        newFilenameLength;
} RenameParameters;

/* Real name of functions are in __mri namespace. */
int      __mriIssueGdbFileOpenRequest(const OpenParameters* pParameters);
int      __mriIssueGdbFileWriteRequest(const TransferParameters* pParameters);
int      __mriIssueGdbFileReadRequest(const TransferParameters* pParameters);
int      __mriIssueGdbFileCloseRequest(uint32_t fileDescriptor);
int      __mriIssueGdbFileSeekRequest(const SeekParameters* pParameters);
int      __mriIssueGdbFileFStatRequest(uint32_t fileDescriptor, uint32_t fileStatBuffer);
int      __mriIssueGdbFileUnlinkRequest(const RemoveParameters* pParameters);
int      __mriIssueGdbFileStatRequest(const StatParameters* pParameters);
int      __mriIssueGdbFileRenameRequest(const RenameParameters* pParameters);
uint32_t __mriHandleFileIOCommand(void);

/* Macroes which allow code to drop the __mri namespace prefix. */
#define IssueGdbFileOpenRequest     __mriIssueGdbFileOpenRequest
#define IssueGdbFileWriteRequest    __mriIssueGdbFileWriteRequest
#define IssueGdbFileReadRequest     __mriIssueGdbFileReadRequest
#define IssueGdbFileCloseRequest    __mriIssueGdbFileCloseRequest
#define IssueGdbFileSeekRequest     __mriIssueGdbFileSeekRequest
#define IssueGdbFileFStatRequest    __mriIssueGdbFileFStatRequest
#define IssueGdbFileUnlinkRequest   __mriIssueGdbFileUnlinkRequest
#define IssueGdbFileStatRequest     __mriIssueGdbFileStatRequest
#define IssueGdbFileRenameRequest   __mriIssueGdbFileRenameRequest
#define HandleFileIOCommand         __mriHandleFileIOCommand

#endif /* _CMD_FILE_H_ */
