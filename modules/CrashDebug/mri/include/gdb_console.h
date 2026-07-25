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
#ifndef _GDB_CONSOLE_H_
#define _GDB_CONSOLE_H_

#include <stdint.h>

/* Real name of functions are in __mri namespace. */
void __mriGdbConsole_WriteString(const char* pString);
void __mriGdbConsole_WriteHexValue(uint32_t value);

/* Macroes which allow code to drop the __mri namespace prefix. */
#define WriteStringToGdbConsole     __mriGdbConsole_WriteString
#define WriteHexValueToGdbConsole   __mriGdbConsole_WriteHexValue

#endif /* _GDB_CONSOLE_H_ */
