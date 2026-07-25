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
/* Semihost functionality for redirecting operations such as file I/O to the GNU debugger. */
#include <platforms.h>
#include <semihost.h>


int Semihost_IsDebuggeeMakingSemihostCall(void)
{
    PlatformInstructionType instructionType = Platform_TypeOfCurrentInstruction();

    return (instructionType == MRI_PLATFORM_INSTRUCTION_MBED_SEMIHOST_CALL ||
            instructionType == MRI_PLATFORM_INSTRUCTION_NEWLIB_SEMIHOST_CALL);
}

int Semihost_HandleSemihostRequest(void)
{
    PlatformInstructionType    instructionType = Platform_TypeOfCurrentInstruction();
    PlatformSemihostParameters parameters = Platform_GetSemihostCallParameters();

    if (instructionType == MRI_PLATFORM_INSTRUCTION_MBED_SEMIHOST_CALL)
        return Semihost_HandleMbedSemihostRequest(&parameters);
    else if (instructionType == MRI_PLATFORM_INSTRUCTION_NEWLIB_SEMIHOST_CALL)
        return Semihost_HandleNewlibSemihostRequest(&parameters);
    else
        return 0;
}
