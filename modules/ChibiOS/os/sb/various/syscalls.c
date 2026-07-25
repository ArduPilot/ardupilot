/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

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

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "sbuser.h"

#define MAKERR(e) (-(int)(e))

__attribute__((used))
int _close_r(struct _reent *r, int file) {
  uint32_t err;

  err = sbFileClose((uint32_t)file);
  if (SB_ERR_ISERROR(err)) {
    __errno_r(r) = MAKERR(err);
    return -1;
  }

  return 0;
}

__attribute__((used))
int _write_r(struct _reent *r, int file, char * ptr, int len) {
  uint32_t err;

  err = sbFileWrite((uint32_t)file, (const uint8_t *)ptr, (size_t)len);
  if (SB_ERR_ISERROR(err)) {
    __errno_r(r) = MAKERR(err);
    return -1;
  }

  return (int)err;
}

__attribute__((used))
int _read_r(struct _reent *r, int file, char * ptr, int len) {
  uint32_t err;

  err = sbFileRead((uint32_t)file, (uint8_t *)ptr, (size_t)len);
  if (SB_ERR_ISERROR(err)) {
    __errno_r(r) = MAKERR(err);
    return -1;
  }

  return (int)err;
}

__attribute__((used))
int _lseek_r(struct _reent *r, int file, int ptr, int dir) {
  uint32_t err;

  err = sbFileSeek((uint32_t)file, (uint32_t)ptr, (uint32_t)dir);
  if (SB_ERR_ISERROR(err)) {
    __errno_r(r) = MAKERR(err);
    return -1;
  }

  return (int)err;
}

__attribute__((used))
int _fstat_r(struct _reent *r, int file, struct stat * st) {
  (void)r;
  (void)file;

  memset(st, 0, sizeof(*st));
  st->st_mode = S_IFCHR;
  return 0;
}

__attribute__((used))
int _isatty_r(struct _reent *r, int fd) {
  (void)r;
  (void)fd;

  return 1;
}

__attribute__((used))
caddr_t _sbrk_r(struct _reent *r, int incr) {
  extern uint8_t __heap_end__, __heap_base__;
  static uint8_t *p = &__heap_base__;
  uint8_t *prevp;

  prevp = p;
  if ((p + incr > &__heap_end__) ||
      (p + incr < &__heap_base__)) {
    __errno_r(r)  = ENOMEM;
    return (caddr_t)-1;
  }

  p += incr;
  return (caddr_t)prevp;
}

__attribute__((used))
int _kill(int pid, int sig) {

  (void) pid;
  (void) sig;

  errno = EINVAL;
  return -1;
}

__attribute__((used))
void _exit(int code) {

  sbExit((msg_t)code);
  abort();
}

__attribute__((used))
int _getpid(void) {

  return 1;
  abort();
}

#ifdef __cplusplus
extern "C" {
  void __cxa_pure_virtual(void) {
    osalSysHalt("pure virtual");
  }
}
#endif

/*** EOF ***/
