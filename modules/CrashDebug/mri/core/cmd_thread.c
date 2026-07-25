/* Copyright 2020 Adam Green (https://github.com/adamgreen/)
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
/* Command handler for gdb commands related to threads. */
#include <mri.h>
#include <core.h>
#include <cmd_thread.h>
#include <platforms.h>
#include <try_catch.h>


/* Handle the 'H' command which is sent to switch thread register context.
    Command Format:     Hgxxxxxxxx
    Response Format:    OK
    Where xxxxxxxx is the hexadecimal representation of the ID for the thread to use for future register read/write
    commands.
*/
uint32_t mriCmd_HandleThreadContextCommand(void)
{
    Buffer*     pBuffer = GetBuffer();

    __try
    {
        uint32_t    threadId = 0;
        char        ch;

        __throwing_func( ch = Buffer_ReadChar(pBuffer) );
        if (ch != 'g')
        {
            setExceptionCode(invalidArgumentException);
            break;
        }
        __throwing_func( threadId = Buffer_ReadUIntegerAsHex(pBuffer) );
        pBuffer = GetInitializedBuffer();
        if (!Platform_RtosGetThreadContext(pBuffer, threadId))
        {
            setExceptionCode(invalidArgumentException);
            break;
        }
        PrepareStringResponse("OK");
    }
    __catch
    {
        PrepareStringResponse(MRI_ERROR_INVALID_ARGUMENT);
    }

    return 0;
}


/* Handle the 'T' command which is sent to see if a thread ID is still active.
    Command Format:     Txxxxxxxx
    Response Format:    OK if thread is still active.
                        E01 is thread isn't active.
    Where xxxxxxxx is the hexadecimal representation of the thread ID.
*/
uint32_t mriCmd_HandleIsThreadActiveCommand(void)
{
    // Buffer*     pBuffer = GetBuffer();

    __try
    {
        // uint32_t    threadId = 0;
        // int         isActive = 0;

        // __throwing_func( threadId = Buffer_ReadUIntegerAsHex(pBuffer) );
        // isActive = Platform_RtosIsThreadActive(threadId);
        // if (!isActive)
        // {
        //     setExceptionCode(invalidArgumentException);
        //     break;
        // }
        PrepareStringResponse("OK");
    }
    __catch
    {
        PrepareStringResponse(MRI_ERROR_INVALID_ARGUMENT);
    }

    return 0;
}
