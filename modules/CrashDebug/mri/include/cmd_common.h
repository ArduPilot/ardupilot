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
/* Common functionality shared between gdb command handlers in mri. */
#ifndef _CMD_COMMON_H_
#define _CMD_COMMON_H_

#include <stdint.h>
#include "buffer.h"
#include "try_catch.h"

/* The bits that can be set in the return value from a command handler to indicate if the caller should return
   immediately or send the prepared response back to gdb.  It also indicates whether program execution should be
   resumed for commands like continue and single step. */
#define HANDLER_RETURN_RESUME_PROGRAM       1
#define HANDLER_RETURN_RETURN_IMMEDIATELY   2
#define HANDLER_RETURN_SKIPPED_OVER_BREAK   4

typedef struct
{
    uint32_t address;
    uint32_t length;
} AddressLength;

/* Real name of functions are in __mri namespace. */
__throws void     __mriCmd_ReadAddressAndLengthArguments(Buffer* pBuffer, AddressLength* pArguments);
__throws void     __mriCmd_ReadAddressAndLengthArgumentsWithColon(Buffer* pBuffer, AddressLength* pArguments);
__throws uint32_t __mriCmd_ReadUIntegerArgument(Buffer* pBuffer);
__throws void     __mriCmd_ThrowIfNextCharIsNotEqualTo(Buffer* pBuffer, char thisChar);

/* Macroes which allow code to drop the __mri namespace prefix. */
#define ReadAddressAndLengthArguments           __mriCmd_ReadAddressAndLengthArguments
#define ReadAddressAndLengthArgumentsWithColon  __mriCmd_ReadAddressAndLengthArgumentsWithColon
#define ReadUIntegerArgument                    __mriCmd_ReadUIntegerArgument
#define ThrowIfNextCharIsNotEqualTo             __mriCmd_ThrowIfNextCharIsNotEqualTo

#endif /* _CMD_COMMON_H_ */
