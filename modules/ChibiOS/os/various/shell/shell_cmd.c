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
 * @file    shell_cmd.c
 * @brief   Simple CLI shell common commands code.
 *
 * @addtogroup SHELL
 * @{
 */

#include <string.h>

#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "shell_cmd.h"
#include "chprintf.h"

#if (SHELL_CMD_FILES_ENABLED == TRUE) || defined(__DOXYGEN__)
#include <fcntl.h>
#include "vfs.h"
#endif

#if (SHELL_CMD_TEST_ENABLED == TRUE) || defined(__DOXYGEN__)
#include "rt_test_root.h"
#include "oslib_test_root.h"
#endif

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

#if ((SHELL_CMD_EXIT_ENABLED == TRUE) && !defined(__CHIBIOS_NIL__)) ||        \
    defined(__DOXYGEN__)
static void cmd_exit(BaseSequentialStream *chp, int argc, char *argv[]) {

  (void)argv;
  if (argc > 0) {
    shellUsage(chp, "exit");
    return;
  }

  shellExit(MSG_OK);
}
#endif

#if (SHELL_CMD_INFO_ENABLED == TRUE) || defined(__DOXYGEN__)
static void cmd_info(BaseSequentialStream *chp, int argc, char *argv[]) {

  (void)argv;
  if (argc > 0) {
    shellUsage(chp, "info");
    return;
  }

  chprintf(chp, "Kernel:       %s" SHELL_NEWLINE_STR, CH_KERNEL_VERSION);
#ifdef PORT_COMPILER_NAME
  chprintf(chp, "Compiler:     %s" SHELL_NEWLINE_STR, PORT_COMPILER_NAME);
#endif
  chprintf(chp, "Architecture: %s" SHELL_NEWLINE_STR, PORT_ARCHITECTURE_NAME);
#ifdef PORT_CORE_VARIANT_NAME
  chprintf(chp, "Core Variant: %s" SHELL_NEWLINE_STR, PORT_CORE_VARIANT_NAME);
#endif
#ifdef PORT_INFO
  chprintf(chp, "Port Info:    %s" SHELL_NEWLINE_STR, PORT_INFO);
#endif
#ifdef PLATFORM_NAME
  chprintf(chp, "Platform:     %s" SHELL_NEWLINE_STR, PLATFORM_NAME);
#endif
#ifdef BOARD_NAME
  chprintf(chp, "Board:        %s" SHELL_NEWLINE_STR, BOARD_NAME);
#endif
#ifdef __DATE__
#ifdef __TIME__
  chprintf(chp, "Build time:   %s%s%s" SHELL_NEWLINE_STR, __DATE__, " - ", __TIME__);
#endif
#endif
}
#endif

#if (SHELL_CMD_ECHO_ENABLED == TRUE) || defined(__DOXYGEN__)
static void cmd_echo(BaseSequentialStream *chp, int argc, char *argv[]) {

  (void)argv;
  if (argc != 1) {
    shellUsage(chp, "echo \"message\"");
    return;
  }
  chprintf(chp, "%s" SHELL_NEWLINE_STR, argv[0]);
}
#endif

#if (SHELL_CMD_SYSTIME_ENABLED == TRUE) || defined(__DOXYGEN__)
static void cmd_systime(BaseSequentialStream *chp, int argc, char *argv[]) {

  (void)argv;
  if (argc > 0) {
    shellUsage(chp, "systime");
    return;
  }
  chprintf(chp, "%lu" SHELL_NEWLINE_STR, (unsigned long)chVTGetSystemTimeX());
}
#endif

#if (SHELL_CMD_MEM_ENABLED == TRUE) || defined(__DOXYGEN__)
static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
  size_t n, total, largest;

  (void)argv;
  if (argc > 0) {
    shellUsage(chp, "mem");
    return;
  }
  n = chHeapStatus(NULL, &total, &largest);
  chprintf(chp, "core free memory : %u bytes" SHELL_NEWLINE_STR, chCoreGetStatusX());
  chprintf(chp, "heap fragments   : %u" SHELL_NEWLINE_STR, n);
  chprintf(chp, "heap free total  : %u bytes" SHELL_NEWLINE_STR, total);
  chprintf(chp, "heap free largest: %u bytes" SHELL_NEWLINE_STR, largest);
}
#endif

#if (SHELL_CMD_THREADS_ENABLED == TRUE) || defined(__DOXYGEN__)
static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
  static const char *states[] = {CH_STATE_NAMES};
  thread_t *tp;

  (void)argv;
  if (argc > 0) {
    shellUsage(chp, "threads");
    return;
  }
  chprintf(chp, "core stklimit    stack     addr refs prio     state         name" SHELL_NEWLINE_STR);
  tp = chRegFirstThread();
  do {
    core_id_t core_id;

#if !defined(__CHIBIOS_NIL__)
    core_id = tp->owner->core_id;
#else
    core_id = 0U;
#endif
#if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || (CH_CFG_USE_DYNAMIC == TRUE)
    uint32_t stklimit = (uint32_t)tp->wabase;
#else
    uint32_t stklimit = 0U;
#endif
    chprintf(chp, "%4lu %08lx %08lx %08lx %4lu %4lu %9s %12s" SHELL_NEWLINE_STR,
             core_id,
             stklimit,
             (uint32_t)tp->ctx.sp,
             (uint32_t)tp,
             (uint32_t)tp->refs - 1,
             (uint32_t)tp->hdr.pqueue.prio,
             states[tp->state],
             tp->name == NULL ? "" : tp->name);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}
#endif

#if (SHELL_CMD_TEST_ENABLED == TRUE) || defined(__DOXYGEN__)
static THD_FUNCTION(test_rt, arg) {
  BaseSequentialStream *chp = (BaseSequentialStream *)arg;
  test_execute(chp, &rt_test_suite);
}

static THD_FUNCTION(test_oslib, arg) {
  BaseSequentialStream *chp = (BaseSequentialStream *)arg;
  test_execute(chp, &oslib_test_suite);
}

static void cmd_test(BaseSequentialStream *chp, int argc, char *argv[]) {
  thread_t *tp;
  tfunc_t tfp;

  (void)argv;
  if (argc != 1) {
    shellUsage(chp, "test rt|oslib");
    return;
  }
  if (!strcmp(argv[0], "rt")) {
    tfp = test_rt;
  }
  else if (!strcmp(argv[0], "oslib")) {
    tfp = test_oslib;
  }
  else {
    shellUsage(chp, "test rt|oslib");
    return;
  }
  tp = chThdCreateFromHeap(NULL, SHELL_CMD_TEST_WA_SIZE,
                           "test", chThdGetPriorityX(),
                           tfp, chp);
  if (tp == NULL) {
    chprintf(chp, "out of memory" SHELL_NEWLINE_STR);
    return;
  }
  chThdWait(tp);
}
#endif

#if (SHELL_CMD_FILES_ENABLED == TRUE) || defined(__DOXYGEN__)
static void scan_nodes(BaseSequentialStream *chp,
                       char *path,
                       vfs_direntry_info_t *dip) {
  msg_t res;
  vfs_directory_node_c *dirp;

  chprintf(chp, "%s" SHELL_NEWLINE_STR, path);
  res = vfsOpenDirectory(path, &dirp);
  if (res == CH_RET_SUCCESS) {
    size_t i = strlen(path);

    while (true) {
      char *fn = dip->name;
      res = vfsReadDirectoryNext(dirp, dip);
      if (res < (msg_t)1) {
        break;
      }

      fn = dip->name;
//      if (dip->attr & VFS_NODE_ATTR_ISDIR) {
      if (VFS_MODE_S_ISDIR(dip->mode)) {
        strcpy(path + i, fn);
        strcat(path + i, "/");
        scan_nodes(chp, path, dip);
        path[i] = '\0';
      }
      else {
        chprintf(chp, "%s%s" SHELL_NEWLINE_STR, path, fn);
      }
    }

    vfsClose((vfs_node_c *)dirp);
  }
}

static void cmd_tree(BaseSequentialStream *chp, int argc, char *argv[]) {
  char *pathbuf = NULL;
  vfs_direntry_info_t *dip = NULL;

  (void)argv;

  if (argc > 0) {
    chprintf(chp, "Usage: tree" SHELL_NEWLINE_STR);
    return;
  }

  do {
    pathbuf = (char *)chHeapAlloc(NULL, 1024);
    dip = (vfs_direntry_info_t *)chHeapAlloc(NULL, 1024);
    if ((pathbuf == NULL) || (dip == NULL)) {
      chprintf(chp, "Out of memory" SHELL_NEWLINE_STR);
     break;
    }

    strcpy(pathbuf, "/");
    scan_nodes(chp, pathbuf, dip);
  }
  while (false);

  if (pathbuf != NULL) {
    chHeapFree((void *)pathbuf);
  }

  if (dip != NULL) {
    chHeapFree((void *)dip);
  }
}

static void cmd_cat(BaseSequentialStream *chp, int argc, char *argv[]) {
  char *buf = NULL;

  if (argc != 1) {
    chprintf(chp, "Usage: cat <filename>" SHELL_NEWLINE_STR);
    return;
  }

  do {
    int fd, n;

    buf = (char *)chHeapAlloc(NULL, 2048);
    if (buf == NULL) {
      chprintf(chp, "Out of memory" SHELL_NEWLINE_STR);
     break;
    }

    fd = open(argv[0], O_RDONLY);
    if(fd == -1) {
      chprintf(chp, "Cannot open file" SHELL_NEWLINE_STR);
      break;
    }

    while ((n = read(fd, buf, sizeof (2048))) > 0) {
      streamWrite(chp, (const uint8_t *)buf, n);
    }
    chprintf(chp, SHELL_NEWLINE_STR);

    (void) close(fd);
  }
  while (false);

  if (buf != NULL) {
    chHeapFree((void *)buf);
  }
}

static void cmd_cd(BaseSequentialStream *chp, int argc, char *argv[]) {

  if (argc != 1) {
    chprintf(chp, "Usage: cd <dirpath>" SHELL_NEWLINE_STR);
    return;
  }

  do {
    msg_t ret;

    ret = vfsChangeCurrentDirectory(argv[0]);
    if (CH_RET_IS_ERROR(ret)) {
      chprintf(chp, "failed (%d)" SHELL_NEWLINE_STR, ret);
    }
  }
  while (false);
}

static void cmd_ls(BaseSequentialStream *chp, int argc, char *argv[]) {
  vfs_direntry_info_t *dip = NULL;

  if (argc > 1) {
    chprintf(chp, "Usage: ls [<dirpath>]" SHELL_NEWLINE_STR);
    return;
  }

  do {
    msg_t ret;
    vfs_directory_node_c *dirp;

    dip = (vfs_direntry_info_t *)chHeapAlloc(NULL, sizeof (vfs_direntry_info_t));
    if (dip == NULL) {
      chprintf(chp, "Out of memory" SHELL_NEWLINE_STR);
     break;
    }

    /* Opening the (un)specified directory.*/
    ret = vfsOpenDirectory(argc == 1 ? argv[0] : ".", &dirp);
    if (!CH_RET_IS_ERROR(ret)) {

      while (vfsReadDirectoryNext(dirp, dip) > (msg_t)0) {
        chprintf(chp, "%s" SHELL_NEWLINE_STR, dip->name);
      }

      vfsClose((vfs_node_c *)dirp);
    }
    else {
      chprintf(chp, "Failed (%d)" SHELL_NEWLINE_STR, ret);
    }

  } while (false);

  if (dip != NULL) {
    chHeapFree((void *)dip);
  }
}

static void cmd_mkdir(BaseSequentialStream *chp, int argc, char *argv[]) {
  msg_t ret;

  if (argc != 1) {
    chprintf(chp, "Usage: mkdir <dirpath>" SHELL_NEWLINE_STR);
    return;
  }

  ret = vfsMkdir(argv[0], 0777U);
  if (CH_RET_IS_ERROR(ret)) {
    chprintf(chp, "Failed (%d)" SHELL_NEWLINE_STR, ret);
  }
}

static void cmd_mv(BaseSequentialStream *chp, int argc, char *argv[]) {
  msg_t ret;

  if (argc != 2) {
    chprintf(chp, "Usage: mv <oldpath> <newpath>" SHELL_NEWLINE_STR);
    return;
  }

  ret = vfsRename(argv[0], argv[1]);
  if (CH_RET_IS_ERROR(ret)) {
    chprintf(chp, "Failed (%d)" SHELL_NEWLINE_STR, ret);
  }
}

static void cmd_pwd(BaseSequentialStream *chp, int argc, char *argv[]) {
  char *buf = NULL;

  (void)argv;

  if (argc != 0) {
    chprintf(chp, "Usage: pwd" SHELL_NEWLINE_STR);
    return;
  }

  do {
    msg_t ret;

    buf = (char *)chHeapAlloc(NULL, VFS_CFG_PATHLEN_MAX + 1);
    if (buf == NULL) {
      chprintf(chp, "Out of memory" SHELL_NEWLINE_STR);
     break;
    }

    ret = vfsGetCurrentDirectory(buf, VFS_CFG_PATHLEN_MAX + 1);
    if (CH_RET_IS_ERROR(ret)) {
      chprintf(chp, "Failed (%d)" SHELL_NEWLINE_STR, ret);
    }
    else {
      chprintf(chp, "%s" SHELL_NEWLINE_STR, buf);
    }
  }
  while (false);

  if (buf != NULL) {
    chHeapFree((void *)buf);
  }
}

static void cmd_rm(BaseSequentialStream *chp, int argc, char *argv[]) {
  msg_t ret;

  if (argc != 1) {
    chprintf(chp, "Usage: rm <filepath>" SHELL_NEWLINE_STR);
    return;
  }

  ret = vfsUnlink(argv[0]);
  if (CH_RET_IS_ERROR(ret)) {
    chprintf(chp, "Failed (%d)" SHELL_NEWLINE_STR, ret);
  }
}

static void cmd_rmdir(BaseSequentialStream *chp, int argc, char *argv[]) {
  msg_t ret;

  if (argc != 1) {
    chprintf(chp, "Usage: rmdir <dirpath>" SHELL_NEWLINE_STR);
    return;
  }

  ret = vfsRmdir(argv[0]);
  if (CH_RET_IS_ERROR(ret)) {
    chprintf(chp, "Failed (%d)" SHELL_NEWLINE_STR, ret);
  }
}

static void cmd_stat(BaseSequentialStream *chp, int argc, char *argv[]) {
  msg_t ret;
  vfs_stat_t statbuf;

  if (argc != 1) {
    chprintf(chp, "Usage: stat <path>" SHELL_NEWLINE_STR);
    return;
  }

  ret = vfsStat(argv[0], &statbuf);
  if (CH_RET_IS_ERROR(ret)) {
    chprintf(chp, "Failed (%d)" SHELL_NEWLINE_STR, ret);
  }

  chprintf(chp, "Mode 0x%04lx Size %d" SHELL_NEWLINE_STR, statbuf.mode, statbuf.size);
}
#endif

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Array of the default commands.
 */
const ShellCommand shell_local_commands[] = {
#if (SHELL_CMD_EXIT_ENABLED == TRUE) && !defined(__CHIBIOS_NIL__)
  {"exit",      cmd_exit},
#endif
#if SHELL_CMD_INFO_ENABLED == TRUE
  {"info",      cmd_info},
#endif
#if SHELL_CMD_ECHO_ENABLED == TRUE
  {"echo",      cmd_echo},
#endif
#if SHELL_CMD_SYSTIME_ENABLED == TRUE
  {"systime",   cmd_systime},
#endif
#if SHELL_CMD_MEM_ENABLED == TRUE
  {"mem",       cmd_mem},
#endif
#if SHELL_CMD_THREADS_ENABLED == TRUE
  {"threads",   cmd_threads},
#endif
#if SHELL_CMD_FILES_ENABLED == TRUE
  {"cat",       cmd_cat},
  {"cd",        cmd_cd},
  {"ls",        cmd_ls},
  {"mkdir",     cmd_mkdir},
  {"mv",        cmd_mv},
  {"pwd",       cmd_pwd},
  {"rm",        cmd_rm},
  {"rmdir",     cmd_rmdir},
  {"stat",      cmd_stat},
  {"tree",      cmd_tree},
#endif
#if SHELL_CMD_TEST_ENABLED == TRUE
  {"test",      cmd_test},
#endif
  {NULL, NULL}
};

/** @} */
