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
/* Implementation of semihost assembly language routines to be used to redirect standard I/O (stdin/stdout/stderr)
   from the newlib library to the MRI debug monitor. */
    .text
    .code 16
    .syntax unified

    .global __mriNewlib_SemihostWrite
    .type __mriNewlib_SemihostWrite, function
    /* extern "C" int __mriNewlib_SemihostWrite(int file, char *ptr, int len);
       Issues semihost write call to PC via gdb.
    */
__mriNewlib_SemihostWrite:
    bkpt    0xff
    bx      lr


    .global __mriNewlib_SemihostRead
    .type __mriNewlib_SemihostRead, function
    /* extern "C" int __mriNewlib_SemihostRead(int file, char *ptr, int len);
       Issues semihost read call to PC via gdb.
    */
__mriNewlib_SemihostRead:
    bkpt    0xff
    bx      lr


    .global __mriNewLib_SemihostOpen
    .type __mriNewLib_SemihostOpen, function
    /* extern "C" int __mriNewLib_SemihostOpen(const char *pFilename, int flags, int mode);
       Issues semihost open file call to PC via gdb.
    */
__mriNewLib_SemihostOpen:
    bkpt    0xff
    bx      lr

    
    .global __mriNewLib_SemihostRename
    .type __mriNewLib_SemihostRename, function
    /* extern "C" int __mriNewLib_SemihostRename(const char *pOldFilename, const char *pNewFilename);
       Issues file rename call to PC via GDB.
    */
__mriNewLib_SemihostRename:
    bkpt    0xff
    bx      lr
    
    .global __mriNewLib_SemihostUnlink
    .type __mriNewLib_SemihostUnlink, function
    /* extern "C" int __mriNewLib_SemihostUnlink(const char *pFilename);
       Issues file delete (unlink) call to PC via GDB.
    */
__mriNewLib_SemihostUnlink:
    bkpt    0xff
    bx      lr
    
    
    .global __mriNewLib_SemihostStat
    .type __mriNewLib_SemihostStat, function
    /* extern "C" int __mriNewLib_SemihostStat(const char *pFilename, struct stat *pStat);
       Issues stat call to PC via GDB.
    */
__mriNewLib_SemihostStat:
    bkpt    0xff
    bx      lr
    
    
    .global __mriNewlib_SemihostLSeek
    .type __mriNewlib_SemihostLSeek, function
    /* extern "C" int __mriNewlib_SemihostLSeek(int file, int offset, int whence);
       Issues seek call to PC via GDB.
    */
__mriNewlib_SemihostLSeek:
    bkpt    0xff
    bx      lr
    
    
    .global __mriNewlib_SemihostClose
    .type __mriNewlib_SemihostClose, function
    /* extern "C" int __mriNewlib_SemihostClose(int file);
       Issues file close call to PC via GDB.
    */
__mriNewlib_SemihostClose:
    bkpt    0xff
    bx      lr
    
    
    .global __mriNewlib_SemihostFStat
    .type __mriNewlib_SemihostFStat, function
    /* extern "C" int __mriNewlib_SemihostFStat(int file, struct stat *pStat);
       Issues stat call to PC via GDB. 
    */
__mriNewlib_SemihostFStat:
    bkpt    0xff
    bx      lr
