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
/* Semihost functionality for redirecting operations such as file I/O to the GNU debugger. */
#ifndef _SEMIHOST_H_
#define _SEMIHOST_H_

#include "platforms.h"

/* Real name of functions are in __mri namespace. */
int __mriSemihost_IsDebuggeeMakingSemihostCall(void);
int __mriSemihost_HandleSemihostRequest(void);
int __mriSemihost_HandleNewlibSemihostRequest(PlatformSemihostParameters* pSemihostParameters);
int __mriSemihost_HandleMbedSemihostRequest(PlatformSemihostParameters* pParameters);

/* Macroes which allow code to drop the __mri namespace prefix. */
#define Semihost_IsDebuggeeMakingSemihostCall   __mriSemihost_IsDebuggeeMakingSemihostCall
#define Semihost_HandleSemihostRequest          __mriSemihost_HandleSemihostRequest
#define Semihost_HandleNewlibSemihostRequest    __mriSemihost_HandleNewlibSemihostRequest
#define Semihost_HandleMbedSemihostRequest      __mriSemihost_HandleMbedSemihostRequest

#endif /* _SEMIHOST_H_ */
