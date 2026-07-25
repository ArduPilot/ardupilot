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
#ifndef CMD_THREAD_H_
#define CMD_THREAD_H_

#include <stdint.h>

/* Real name of functions are in mri namespace. */
uint32_t mriCmd_HandleThreadContextCommand(void);
uint32_t mriCmd_HandleIsThreadActiveCommand(void);

/* Macroes which allow code to drop the mri namespace prefix. */
#define HandleThreadContextCommand          mriCmd_HandleThreadContextCommand
#define HandleIsThreadActiveCommand         mriCmd_HandleIsThreadActiveCommand

#endif /* CMD_THREAD_H_ */