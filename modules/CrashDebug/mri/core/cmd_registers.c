/* Copyright 2012 Adam Green (http://mbed.org/users/AdamGreen/)

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
/* Command handler for gdb commands related to CPU registers. */
#include "cmd_common.h"
#include "platforms.h"
#include "buffer.h"
#include "core.h"
#include "mri.h"
#include "cmd_registers.h"


/* Sent when an exception occurs while program is executing because of previous 'c' (Continue) or 's' (Step) commands.

    Data Format: Tssii:xxxxxxxx;ii:xxxxxxxx;...
    
    Where ss is the hex value of the signal which caused the exception.
          ii is the hex offset of the 32-bit register value following the ':'  The offset is relative to the register
             contents in the g response packet and the SContext structure.
          xxxxxxxx is the 32-bit value of the specified register in hex format.
          The above ii:xxxxxxxx; patterns can be repeated for whichever register values should be sent with T repsonse.
*/
uint32_t Send_T_StopResponse(void)
{
    Buffer* pBuffer = GetInitializedBuffer();
    
    Buffer_WriteChar(pBuffer, 'T');
    Buffer_WriteByteAsHex(pBuffer, GetSignalValue());
    Platform_WriteTResponseRegistersToBuffer(pBuffer);

    SendPacketToGdb();
    return HANDLER_RETURN_RETURN_IMMEDIATELY;
}


/* Handle the 'g' command which is to send the contents of the registers back to gdb.

    Command Format:     g
    Response Format:    xxxxxxxxyyyyyyyy...
    
    Where xxxxxxxx is the hexadecimal representation of the 32-bit R0 register.
          yyyyyyyy is the hexadecimal representation of the 32-bit R1 register.
          ... and so on through the members of the SContext structure.
*/
uint32_t HandleRegisterReadCommand(void)
{
    Platform_CopyContextToBuffer(GetInitializedBuffer());

    return 0;
}

/* Handle the 'G' command which is to receive the new contents of the registers from gdb for the program to use when
   it resumes execution.
   
   Command Format:      Gxxxxxxxxyyyyyyyy...
   Response Format:     OK
   
    Where xxxxxxxx is the hexadecimal representation of the 32-bit R0 register.
          yyyyyyyy is the hexadecimal representation of the 32-bit R1 register.
          ... and so on through the members of the SContext structure.
*/
uint32_t HandleRegisterWriteCommand(void)
{
    Buffer*     pBuffer = GetBuffer();
    
    Platform_CopyContextFromBuffer(pBuffer);

    if (Buffer_OverrunDetected(pBuffer))
        PrepareStringResponse(MRI_ERROR_BUFFER_OVERRUN);
    else
        PrepareStringResponse("OK");

    return 0;
}
