/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

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
/*
* **** This file incorporates work covered by the following copyright and ****
* **** permission notice:                                                 ****
*
*  Copyright (c) 2009 by Michael Fischer. All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*  3. Neither the name of the author nor the names of its contributors may
*     be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
*  SUCH DAMAGE.
*
****************************************************************************
*  History:
*
*  28.03.09  mifi       First Version, based on the original syscall.c from
*                       newlib version 1.17.0
*  17.08.09  gdisirio   Modified the file for use under ChibiOS/RT
*  15.11.09  gdisirio   Added read and write handling
****************************************************************************/

/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * Modified for use in AP_HAL by Andrew Tridgell and Siddharth Bharat Purohit
 */

#include <sys/unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "ch.h"
#if defined(STDOUT_SD) || defined(STDIN_SD)
#include "hal.h"
#endif

uint8_t _before_main = 1;

/***************************************************************************/

__attribute__((used))
int _read(struct _reent *r, int file, char * ptr, int len)
{
    (void)r;
    (void)file;
    (void)ptr;
    (void)len;
    return -1;
}

/***************************************************************************/

__attribute__((used))
int _lseek(struct _reent *r, int file, int ptr, int dir)
{
    (void)r;
    (void)file;
    (void)ptr;
    (void)dir;

    return 0;
}

/***************************************************************************/

__attribute__((used))
int _write(struct _reent *r, int file, char * ptr, int len)
{
    (void)r;
    (void)file;
    (void)ptr;
    (void)len;
    return -1;
}

/***************************************************************************/

__attribute__((used))
int _close(struct _reent *r, int file)
{
    (void)r;
    (void)file;
    return 0;
}

/***************************************************************************/

__attribute__((used))
caddr_t _sbrk(struct _reent *r, int incr)
{
#if CH_CFG_USE_MEMCORE && CH_CFG_USE_HEAP == TRUE
  void *p;

  chDbgCheck(incr >= 0);
  p = chHeapAlloc(NULL, (size_t)incr);
  if (p == NULL) {
    __errno_r(r) = ENOMEM;
    return (caddr_t)-1;
  }
  return (caddr_t)p;
#else
  (void)incr;
  __errno_r(r) = ENOMEM;
  return (caddr_t)-1;
#endif
}


/***************************************************************************/

__attribute__((used))
int _fstat(struct _reent *r, int file, struct stat * st)
{
    (void)r;
    (void)file;
    (void)st;
    return -1;
}

/***************************************************************************/

__attribute__((used))
int _isatty(struct _reent *r, int fd)
{
    (void)r;
    (void)fd;
    return 1;
}

__attribute__((used))
pid_t _getpid(void)
{
    return 0;
}

__attribute__((used))
void _exit( int status )
{
    (void)status;
    while ( 1 );
}

__attribute__((used))
void _fini(void)
{
}

__attribute__((used))
int _kill( int pid, int sig )
{
    (void)pid;
    (void)sig;
    return -1;
}

/*** EOF ***/
