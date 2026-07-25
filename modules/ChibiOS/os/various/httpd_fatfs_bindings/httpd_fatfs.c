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

/**
 * @file    httpd_fatfs.h
 * @brief   HTTPD FatFS bindings code.
 * @addtogroup LWIP_HTTPD_FATFS
 * @{
 */

#include <string.h>

#include "ch.h"
#include "hal.h"

#include "lwip/apps/fs.h"
#include "lwip/opt.h"
#include "lwip/mem.h"
#include "lwip/apps/httpd.h"

#include "ff.h"

static GUARDEDMEMORYPOOL_DECL (http_file_pool, sizeof(FIL), sizeof(int));
static FIL http_file_array[16];

void httpd_fatfs_init(void) {

  chGuardedPoolLoadArray(&http_file_pool,
                         http_file_array,
                         sizeof http_file_array / sizeof (FIL));
}

int fs_open_custom(struct fs_file *file, const char *name) {
  FIL *filp;

  filp = (FIL *) chGuardedPoolAllocTimeout(&http_file_pool, TIME_INFINITE);
  memset(file, '\0', sizeof (struct fs_file));

  if (f_open(filp, _T(name), FA_READ) != FR_OK) {
    chGuardedPoolFree(&http_file_pool, (void *)filp);
    return 0;
  }

  file->data       = NULL;
  file->len        = (FSIZE_t)f_size(filp);
  file->index      = 0;
  file->pextension = (fs_file_extension *)filp;
  return 1;
}

void fs_close_custom(struct fs_file *file) {

  if (file != NULL) {
    if (file->pextension != NULL) {
      f_close((FIL *)file->pextension);
      chGuardedPoolFree(&http_file_pool, (void *)file->pextension);
      file->pextension = NULL;
    }
  }
}

int fs_read_custom(struct fs_file *file, char *buffer, int count) {

  do {
    FRESULT res;
    UINT br;
    FIL *fil;

    if (file == NULL) {
      break;
    }

    if (file->pextension == NULL) {
      break;
    }

    fil = (FIL *)file->pextension;
    if (f_eof(fil)) {
      break;
    }

    res = f_read(fil, buffer, count, &br);
    if (res != FR_OK) {
      return 0;
    }

    file->index += (int)br;
    return (int)br;

  } while (false);

  return FS_READ_EOF;
}

/** @} */
