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
/* Constants used by gdb for remote file APIs. */
#ifndef _FILEIO_H_
#define _FILEIO_H_

#define O_RDONLY    0x0
#define O_WRONLY    0x1
#define O_RDWR      0x2
#define O_APPEND    0x8
#define O_CREAT     0x200
#define O_TRUNC     0x400

#define S_IRUSR     0400
#define S_IWUSR     0200
#define S_IRGRP     040
#define S_IWGRP     020
#define S_IROTH     04
#define S_IWOTH     02

#define SEEK_SET    0
#define SEEK_CUR    1
#define SEEK_END    2

#endif /* _FILEIO_H_ */
