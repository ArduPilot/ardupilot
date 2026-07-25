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
#include <signal.h>
#include <string.h>
#include "fileio.h"
#include "core.h"
#include "cmd_common.h"
#include "cmd_file.h"


static int processGdbFileResponseCommands(void);
/* Send file open request to gdb on behalf of mbed LocalFileSystem.

    Data Format: Fopen,ff/nn,gg,mm
    
    Where ff is the hex representation of the address of the filename to be opened.
          nn is the hex value of the count of characters in the filename pointed to by ff.
          gg is the hex value of the flags to be used for the file open.
          mm is the hex value of the mode to be used for the file open.
*/
int IssueGdbFileOpenRequest(const OpenParameters* pParameters)
{
    static const char  gdbOpenCommand[] = "Fopen,";
    Buffer*            pBuffer = GetInitializedBuffer();

    Buffer_WriteString(pBuffer, gdbOpenCommand);
    Buffer_WriteUIntegerAsHex(pBuffer, pParameters->filenameAddress);
    Buffer_WriteChar(pBuffer, '/');
    Buffer_WriteUIntegerAsHex(pBuffer, pParameters->filenameLength);
    Buffer_WriteChar(pBuffer, ',');
    Buffer_WriteUIntegerAsHex(pBuffer, pParameters->flags);
    Buffer_WriteChar(pBuffer, ',');
    Buffer_WriteUIntegerAsHex(pBuffer, pParameters->mode);
    
    SendPacketToGdb();
    return processGdbFileResponseCommands();
}


/* Send file write request to gdb on behalf of mbed LocalFileSystem or stdout/stderr.

    Data Format: Fwrite,ff,pp,cc
    
    Where ff is the hex value of the file descriptor of the file to which the data should be written.
          pp is the hex representation of the buffer to be written to the specified file.
          cc is the hex value of the count of bytes in the buffer to be written to the specified file.
*/
int IssueGdbFileWriteRequest(const TransferParameters* pParameters)
{
    static const char  gdbWriteCommand[] = "Fwrite,";
    Buffer*            pBuffer = GetInitializedBuffer();

    Buffer_WriteString(pBuffer, gdbWriteCommand);
    Buffer_WriteUIntegerAsHex(pBuffer, pParameters->fileDescriptor);
    Buffer_WriteChar(pBuffer, ',');
    Buffer_WriteUIntegerAsHex(pBuffer, pParameters->bufferAddress);
    Buffer_WriteChar(pBuffer, ',');
    Buffer_WriteUIntegerAsHex(pBuffer, pParameters->bufferSize);
    
    SendPacketToGdb();
    return processGdbFileResponseCommands();
}


/* Send file read request to gdb on behalf of mbed LocalFileSystem or stdin.

    Data Format: Fread,ff,pp,cc
    
    Where ff is the hex value of the file descriptor of the file from which the data should be read.
          pp is the hex representation of the buffer to be read into.
          cc is the hex value of the count of bytes in the buffer to be read from the specified file.
*/
int IssueGdbFileReadRequest(const TransferParameters* pParameters)
{
    static const char  gdbReadCommand[] = "Fread,";
    Buffer*            pBuffer = GetInitializedBuffer();

    Buffer_WriteString(pBuffer, gdbReadCommand);
    Buffer_WriteUIntegerAsHex(pBuffer, pParameters->fileDescriptor);
    Buffer_WriteChar(pBuffer, ',');
    Buffer_WriteUIntegerAsHex(pBuffer, pParameters->bufferAddress);
    Buffer_WriteChar(pBuffer, ',');
    Buffer_WriteUIntegerAsHex(pBuffer, pParameters->bufferSize);
    
    SendPacketToGdb();
    return processGdbFileResponseCommands();
}


/* Send file close request to gdb on behalf of mbed LocalFileSystem.

    Data Format: Fclose,ff
    
    Where ff is the hex value of the file descriptor to be closed.
*/
int IssueGdbFileCloseRequest(uint32_t fileDescriptor)
{
    static const char  gdbCloseCommand[] = "Fclose,";
    Buffer*            pBuffer = GetInitializedBuffer();

    Buffer_WriteString(pBuffer, gdbCloseCommand);
    Buffer_WriteUIntegerAsHex(pBuffer, fileDescriptor);
    
    SendPacketToGdb();
    return processGdbFileResponseCommands();
}


/* Send file seek request to gdb on behalf of mbed LocalFileSystem.

    Data Format: Flseek,ff,oo,ww
    
    Where ff is the hex value of the file descriptor to be seeked within.
          oo is the hex value of the signed offset for the seek.
          ww is the hex value of the flag indicating from where the seek should be conducted (whence.)
*/
int IssueGdbFileSeekRequest(const SeekParameters* pParameters)
{
    static const char  gdbSeekCommand[] = "Flseek,";
    Buffer*            pBuffer = GetInitializedBuffer();

    Buffer_WriteString(pBuffer, gdbSeekCommand);
    Buffer_WriteUIntegerAsHex(pBuffer, pParameters->fileDescriptor);
    Buffer_WriteChar(pBuffer, ',');
    Buffer_WriteIntegerAsHex(pBuffer, pParameters->offset);
    Buffer_WriteChar(pBuffer, ',');
    Buffer_WriteIntegerAsHex(pBuffer, pParameters->whence);
    
    SendPacketToGdb();
    return processGdbFileResponseCommands();
}


/* Send file stat request to gdb on behalf of mbed LocalFileSystem to get file length.

    Data Format: Ffstat,ff,bb
    
    Where ff is the hex value of the file descriptor to be closed.
          bb is the hex representation of the address of the stat structure to be filled in.
*/
int IssueGdbFileFStatRequest(uint32_t fileDescriptor, uint32_t fileStatBuffer)
{
    static const char  gdbStatCommand[] = "Ffstat,";
    Buffer*            pBuffer = GetInitializedBuffer();

    Buffer_WriteString(pBuffer, gdbStatCommand);
    Buffer_WriteUIntegerAsHex(pBuffer, fileDescriptor);
    Buffer_WriteChar(pBuffer, ',');
    Buffer_WriteUIntegerAsHex(pBuffer, fileStatBuffer);
    
    SendPacketToGdb();
    return processGdbFileResponseCommands();
}


/* Send file unlink request to gdb on behalf of mbed LocalFileSystem.

    Data Format: Funlink,ff/nn
    
    Where ff is the hex representation of the address of the filename to be deleted.
          nn is the hex value of the count of characters in the filename pointed to by ff.
*/
int IssueGdbFileUnlinkRequest(const RemoveParameters* pParameters)
{
    static const char  gdbUnlinkCommand[] = "Funlink,";
    Buffer*            pBuffer = GetInitializedBuffer();

    Buffer_WriteString(pBuffer, gdbUnlinkCommand);
    Buffer_WriteUIntegerAsHex(pBuffer, pParameters->filenameAddress);
    Buffer_WriteChar(pBuffer, '/');
    Buffer_WriteUIntegerAsHex(pBuffer, pParameters->filenameLength);
    
    SendPacketToGdb();
    return processGdbFileResponseCommands();
}


/* Send file system level stat request to gdb.

    Data Format: Fstat,ff/nn,bb
    
    Where ff is the hex representation of the address of the filename.
          nn is the hex value of the count of characters in the filename pointed to by ff.
          bb is the hex representation of the address of the stat structure to be filled in.
*/
int IssueGdbFileStatRequest(const StatParameters* pParameters)
{
    static const char  gdbStatCommand[] = "Fstat,";
    Buffer*            pBuffer = GetInitializedBuffer();

    Buffer_WriteString(pBuffer, gdbStatCommand);
    Buffer_WriteUIntegerAsHex(pBuffer, pParameters->filenameAddress);
    Buffer_WriteChar(pBuffer, '/');
    Buffer_WriteUIntegerAsHex(pBuffer, pParameters->filenameLength);
    Buffer_WriteChar(pBuffer, ',');
    Buffer_WriteUIntegerAsHex(pBuffer, pParameters->fileStatBuffer);
    
    SendPacketToGdb();
    return processGdbFileResponseCommands();
}


/* Send file rename request to gdb.

    Data Format: Frename,oo/aa,nn/bb
    
    Where oo is the hex representation of the address of the original filename.
          aa is the hex value of the count of characters in the original filename pointed to by oo.
          nn is the hex representation of the address of the new filename.
          bb is the hex value of the count of characters in the new filename pointed to by nn.
*/
int IssueGdbFileRenameRequest(const RenameParameters* pParameters)
{
    static const char  gdbCommand[] = "Frename,";
    Buffer*            pBuffer = GetInitializedBuffer();

    Buffer_WriteString(pBuffer, gdbCommand);
    Buffer_WriteUIntegerAsHex(pBuffer, pParameters->origFilenameAddress);
    Buffer_WriteChar(pBuffer, '/');
    Buffer_WriteUIntegerAsHex(pBuffer, pParameters->origFilenameLength);
    Buffer_WriteChar(pBuffer, ',');
    Buffer_WriteUIntegerAsHex(pBuffer, pParameters->newFilenameAddress);
    Buffer_WriteChar(pBuffer, '/');
    Buffer_WriteUIntegerAsHex(pBuffer, pParameters->newFilenameLength);
    
    SendPacketToGdb();
    return processGdbFileResponseCommands();
}


/* Handle the 'F' command which is sent from gdb in response to a previously sent File I/O command from mri.
   
    Command Format:     Frr[,ee[,C]]

    Where rr is the signed hexadecimal representation of the return code from the last requested file I/O command.
          ee is the optional signed hexadecimal value for the errorno associated with the call if rr indicates error.
          C is the optional 'C' character sent by gdb to indicate that CTRL+C was pressed by user while gdb
            was processing the current file I/O request.
*/
uint32_t HandleFileIOCommand(void)
{
    static const char controlCFlag[] = ",C";
    Buffer*           pBuffer = GetBuffer();
    int               returnCode = -1;
    int               errNo = 0;
    int               controlC = 0;
    
    returnCode = Buffer_ReadIntegerAsHex(pBuffer);
    if (Buffer_IsNextCharEqualTo(pBuffer, ','))
    {
        errNo = Buffer_ReadIntegerAsHex(pBuffer);
        controlC = Buffer_MatchesString(pBuffer, controlCFlag, sizeof(controlCFlag)-1);
    }
    
    SetSemihostReturnValues(returnCode, errNo);
    RecordControlCFlagSentFromGdb(controlC);
    clearExceptionCode();

    return (HANDLER_RETURN_RESUME_PROGRAM | HANDLER_RETURN_RETURN_IMMEDIATELY);
}

static int processGdbFileResponseCommands(void)
{
    GdbCommandHandlingLoop();

    if (WasControlCFlagSentFromGdb())
    {
        if (!WasSemihostCallCancelledByGdb())
            FlagSemihostCallAsHandled();

        SetSignalValue(SIGINT);
        return 0;
    }
    else
    {
        FlagSemihostCallAsHandled();
        return 1;
    }
}
