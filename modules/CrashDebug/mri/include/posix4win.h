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
/* Monitor for Remote Inspection. */
#ifndef _POSIX4WIN_H_
#define _POSIX4WIN_H_
#ifdef WIN32


#define SIGTRAP 5
#define SIGBUS  10
#define SIGSTOP 18


#endif /* WIN32 */
#endif /* _POSIX4WIN_H_ */
