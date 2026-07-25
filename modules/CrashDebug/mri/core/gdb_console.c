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
/* Routines to output text to stdout on the gdb console. */
#include <string.h>
#include "buffer.h"
#include "platforms.h"
#include "core.h"
#include "memory.h"
#include "gdb_console.h"


static void writeStringToSharedCommChannel(const char* pString);
static void writeStringToExclusiveGdbCommChannel(const char* pString);
void WriteStringToGdbConsole(const char* pString)
{
    if (Platform_CommSharingWithApplication() && IsFirstException())
        writeStringToSharedCommChannel(pString);
    else
        writeStringToExclusiveGdbCommChannel(pString);
}

static void writeStringToSharedCommChannel(const char* pString)
{
    while(*pString)
        Platform_CommSendChar(*pString++);
}

/* Send the 'O' command to gdb to output text to its console.

    Command Format: OXX...
    Where XX is the hexadecimal representation of each character in the string to be sent to the gdb console.
*/
static void writeStringToExclusiveGdbCommChannel(const char* pString)
{
    Buffer* pBuffer = GetInitializedBuffer();

    Buffer_WriteChar(pBuffer, 'O');
    while (*pString)
        Buffer_WriteByteAsHex(pBuffer, *pString++);
    if (!Buffer_OverrunDetected(pBuffer))
        SendPacketToGdb();
}


void WriteHexValueToGdbConsole(uint32_t Value)
{
    Buffer BufferObject;
    char   StringBuffer[11];
    
    Buffer_Init(&BufferObject, StringBuffer, sizeof(StringBuffer));
    Buffer_WriteString(&BufferObject, "0x");
    Buffer_WriteUIntegerAsHex(&BufferObject, Value);
    Buffer_WriteChar(&BufferObject, '\0');
    
    WriteStringToGdbConsole(StringBuffer);
}
