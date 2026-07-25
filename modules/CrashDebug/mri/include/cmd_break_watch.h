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
/* Handlers for gdb breakpoint and watchpoint commands. */
#ifndef _CMD_BREAK_WATCH_H_
#define _CMD_BREAK_WATCH_H_

#include <stdint.h>

/* Real name of functions are in __mri namespace. */
uint32_t __mriCmd_HandleBreakpointWatchpointSetCommand(void);
uint32_t __mriCmd_HandleBreakpointWatchpointRemoveCommand(void);

/* Macroes which allow code to drop the __mri namespace prefix. */
#define HandleBreakpointWatchpointSetCommand    __mriCmd_HandleBreakpointWatchpointSetCommand
#define HandleBreakpointWatchpointRemoveCommand __mriCmd_HandleBreakpointWatchpointRemoveCommand

#endif /* _CMD_BREAK_WATCH_H_ */
