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
 * @file    shell.c
 * @brief   Simple CLI shell code.
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

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

#if !defined(__CHIBIOS_NIL__) || defined(__DOXYGEN__)
/**
 * @brief   Shell termination event source.
 */
event_source_t shell_terminated;
#endif

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

static char *parse_arguments(char *str, char **saveptr) {
  char *p;

  if (str != NULL)
    *saveptr = str;

  p = *saveptr;
  if (!p) {
    return NULL;
  }

  /* Skipping white space.*/
  p += strspn(p, " \t");

  if (*p == '"') {
    /* If an argument starts with a double quote then its delimiter is another
       quote.*/
    p++;
    *saveptr = strpbrk(p, "\"");
  }
  else {
    /* The delimiter is white space.*/
    *saveptr = strpbrk(p, " \t");
  }

  /* Replacing the delimiter with a zero.*/
  if (*saveptr != NULL) {
    *(*saveptr)++ = '\0';
  }

  return *p != '\0' ? p : NULL;
}

static void list_commands(BaseSequentialStream *chp, const ShellCommand *scp) {

  while (scp->sc_name != NULL) {
    chprintf(chp, "%s ", scp->sc_name);
    scp++;
  }
}

static bool cmdexec(const ShellCommand *scp, BaseSequentialStream *chp,
                      char *name, int argc, char *argv[]) {

  while (scp->sc_name != NULL) {
    if (strcmp(scp->sc_name, name) == 0) {
      scp->sc_function(chp, argc, argv);
      return false;
    }
    scp++;
  }
  return true;
}

#if (SHELL_USE_HISTORY == TRUE) || defined(__DOXYGEN__)
static void del_histbuff_entry(ShellHistory *shp) {
  int pos = shp->sh_beg + *(shp->sh_buffer + shp->sh_beg) + 1;

  if (pos >= shp->sh_size)
    pos -= shp->sh_size;

  shp->sh_beg = pos;
}

static bool is_histbuff_space(ShellHistory *shp, int length) {

  if (shp->sh_end >= shp->sh_beg) {
    if (length < (shp->sh_size - (shp->sh_end - shp->sh_beg + 1)))
      return true;
  }
  else {
    if (length < (shp->sh_beg - shp->sh_end - 1))
      return true;
  }

  return false;
}

static void save_history(ShellHistory *shp, char *line, int length) {

  if (shp == NULL)
    return;

  if (length > shp->sh_size - 2)
    return;

  while ((*(line + length -1) == ' ') && (length > 0))
    length--;

  if (length <= 0)
    return;

  while (!is_histbuff_space(shp, length))
    del_histbuff_entry(shp);

  if (length < shp->sh_size - shp->sh_end - 1)
    memcpy(shp->sh_buffer + shp->sh_end + 1, line, length);
  else {
    /*
     * Since there isn't enough room left at the end of the buffer,
     * split the line to save up to the end of the buffer and then
     * wrap back to the beginning of the buffer.
     */
    int part_len = shp->sh_size - shp->sh_end - 1;
    memcpy(shp->sh_buffer + shp->sh_end + 1, line, part_len);
    memcpy(shp->sh_buffer, line + part_len, length - part_len);
  }

  /* Save the length of the current line and move the buffer end pointer */
  *(shp->sh_buffer + shp->sh_end) = (char)length;
  shp->sh_end += length + 1;
  if (shp->sh_end >= shp->sh_size)
    shp->sh_end -= shp->sh_size;
  *(shp->sh_buffer + shp->sh_end) = 0;
  shp->sh_cur = 0;
}

static int get_history(ShellHistory *shp, char *line, int dir) {
  int count=0;

  if (shp == NULL)
    return -1;

  /* Count the number of lines saved in the buffer */
  int idx = shp->sh_beg;
  while (idx != shp->sh_end) {
    idx += *(shp->sh_buffer + idx) + 1;
    if (idx >= shp->sh_size)
      idx -= shp->sh_size;
    count++;
  }

  if (dir == SHELL_HIST_DIR_FW) {
    if (shp->sh_cur > 0)
      shp->sh_cur -= 2;
    else
      return 0;
  }

  if (count >= shp->sh_cur) {
    idx = shp->sh_beg;
    int i = 0;
    while (idx != shp->sh_end && shp->sh_cur != (count - i - 1)) {
      idx += *(shp->sh_buffer + idx) + 1;
      if (idx >= shp->sh_size)
        idx -= shp->sh_size;
      i++;
    }

    int length = *(shp->sh_buffer + idx);

    if (length > 0) {
      shp->sh_cur++;

      memset(line, 0, SHELL_MAX_LINE_LENGTH);
      if ((idx + length) < shp->sh_size) {
        memcpy(line, (shp->sh_buffer + idx + 1), length);
      }
      else {
        /*
         * Since the saved line was split at the end of the buffer,
         * get the line in two parts.
         */
        int part_len = shp->sh_size - idx - 1;
        memcpy(line, shp->sh_buffer + idx + 1, part_len);
        memcpy(line + part_len, shp->sh_buffer, length - part_len);
      }
      return length;
    }
    else if (dir == SHELL_HIST_DIR_FW) {
      shp->sh_cur++;
      return 0;
    }
  }
  return -1;
}
#endif

#if (SHELL_USE_COMPLETION == TRUE) || defined(__DOXYGEN__)
static void get_completions(ShellConfig *scfg, char *line) {
  const ShellCommand *lcp = shell_local_commands;
  const ShellCommand *scp = scfg->sc_commands;
  char **scmp = scfg->sc_completion;
  char help_cmp[] = "help";

  if (strstr(help_cmp, line) == help_cmp) {
    *scmp++ = help_cmp;
  }
  while (lcp->sc_name != NULL) {
    if (strstr(lcp->sc_name, line) == lcp->sc_name) {
      *scmp++ = (char *)lcp->sc_name;
    }
    lcp++;
  }
  if (scp != NULL) {
    while (scp->sc_name != NULL) {
      if (strstr(scp->sc_name, line) == scp->sc_name) {
        *scmp++ = (char *)scp->sc_name;
      }
      scp++;
    }
  }

  *scmp = NULL;
}

static int process_completions(ShellConfig *scfg, char *line, int length, unsigned size) {
  char **scmp = scfg->sc_completion;
  char **cmp = scmp + 1;
  char *c = line + length;
  int clen = 0;

  if (*scmp != NULL) {
    if (*cmp == NULL) {
      clen = strlen(*scmp);
      int i = length;
      while ((c < line + clen) && (c < line + size - 1))
        *c++ = *(*scmp + i++);
      if (c < line + size -1) {
        *c = ' ';
        clen++;
      }
    }
    else {
      while (*(*scmp + clen) != 0) {
        while ((*(*scmp + clen) == *(*cmp + clen)) &&
               (*(*cmp + clen) != 0) && (*cmp != NULL)) {
          cmp++;
        }
        if (*cmp == NULL) {
          if ((c < line + size - 1) && (clen >= length))
            *c++ = *(*scmp + clen);
          cmp = scmp + 1;
          clen++;
        }
        else {
          break;
        }
      }
    }

    *(line + clen) = 0;
  }

  return clen;
}

static void write_completions(ShellConfig *scfg, char *line, int pos) {
  BaseSequentialStream *chp = scfg->sc_channel;
  char **scmp = scfg->sc_completion;

  if (*(scmp + 1) != NULL) {
    chprintf(chp, SHELL_NEWLINE_STR);
    while (*scmp != NULL)
      chprintf(chp, " %s", *scmp++);
    chprintf(chp, SHELL_NEWLINE_STR);

    chprintf(chp, SHELL_PROMPT_STR);
    chprintf(chp, "%s", line);
  }
  else {
    chprintf(chp, "%s", line + pos);
  }
}
#endif

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Shell thread function.
 *
 * @param[in] p         pointer to a @p BaseSequentialStream object
 */
THD_FUNCTION(shellThread, p) {
  int n;
  ShellConfig *scfg = p;
  BaseSequentialStream *chp = scfg->sc_channel;
  const ShellCommand *scp = scfg->sc_commands;
  char *lp, *cmd, *tokp, line[SHELL_MAX_LINE_LENGTH];
  char *args[SHELL_MAX_ARGUMENTS + 1];

#if !defined(__CHIBIOS_NIL__)
  chRegSetThreadName(SHELL_THREAD_NAME);
#endif

#if SHELL_USE_HISTORY == TRUE
  *(scfg->sc_histbuf) = 0;
  ShellHistory hist = {
                       scfg->sc_histbuf,
                       scfg->sc_histsize,
                       0,
                       0,
                       0
  };
  ShellHistory *shp = &hist;
#else
  ShellHistory *shp = NULL;
#endif

  chprintf(chp, SHELL_NEWLINE_STR);
  chprintf(chp, "ChibiOS/RT Shell" SHELL_NEWLINE_STR);
#if !defined(__CHIBIOS_NIL__)
  while (!chThdShouldTerminateX()) {
#else
  while (true) {
#endif
    chprintf(chp, SHELL_PROMPT_STR);
    if (shellGetLine(scfg, line, sizeof(line), shp)) {
#if (SHELL_CMD_EXIT_ENABLED == TRUE) && !defined(__CHIBIOS_NIL__)
      chprintf(chp, SHELL_NEWLINE_STR);
      chprintf(chp, "logout");
      break;
#else
      /* Putting a delay in order to avoid an endless loop trying to read
         an unavailable stream.*/
      osalThreadSleepMilliseconds(100);
      continue;
#endif
    }
    lp = parse_arguments(line, &tokp);
    cmd = lp;
    n = 0;
    while ((lp = parse_arguments(NULL, &tokp)) != NULL) {
      if (n >= SHELL_MAX_ARGUMENTS) {
        chprintf(chp, "too many arguments" SHELL_NEWLINE_STR);
        cmd = NULL;
        break;
      }
      args[n++] = lp;
    }
    args[n] = NULL;
    if (cmd != NULL) {
      if (strcmp(cmd, "help") == 0) {
        if (n > 0) {
          shellUsage(chp, "help");
          continue;
        }
        chprintf(chp, "Commands: help ");
        list_commands(chp, shell_local_commands);
        if (scp != NULL)
          list_commands(chp, scp);
        chprintf(chp, SHELL_NEWLINE_STR);
      }
      else if (cmdexec(shell_local_commands, chp, cmd, n, args) &&
          ((scp == NULL) || cmdexec(scp, chp, cmd, n, args))) {
        chprintf(chp, "%s", cmd);
        chprintf(chp, " ?" SHELL_NEWLINE_STR);
      }
    }
  }
#if !defined(__CHIBIOS_NIL__)
  shellExit(MSG_OK);
#endif
}

/**
 * @brief   Shell manager initialization.
 *
 * @api
 */
void shellInit(void) {

#if !defined(__CHIBIOS_NIL__)
  chEvtObjectInit(&shell_terminated);
#endif
}

#if !defined(__CHIBIOS_NIL__) || defined(__DOXYGEN__)
/**
 * @brief   Terminates the shell.
 * @note    Must be invoked from the command handlers.
 * @note    Does not return.
 *
 * @param[in] msg       shell exit code
 *
 * @api
 */
void shellExit(msg_t msg) {

  /* Atomically broadcasting the event source and terminating the thread,
     there is not a chSysUnlock() because the thread terminates upon return.*/
  chSysLock();
  chEvtBroadcastI(&shell_terminated);
  chThdExitS(msg);
}
#endif

/**
 * @brief   Reads a whole line from the input channel.
 * @note    Input chars are echoed on the same stream object with the
 *          following exceptions:
 *          - DEL and BS are echoed as BS-SPACE-BS.
 *          - CR is echoed as CR-LF.
 *          - 0x4 is echoed as "^D".
 *          - Other values below 0x20 are not echoed.
 *          .
 *
 * @param[in] scfg      pointer to a @p ShellConfig object
 * @param[in] line      pointer to the line buffer
 * @param[in] size      buffer maximum length
 * @param[in] shp       pointer to a @p ShellHistory object or NULL
 * @return              The operation status.
 * @retval true         the channel was reset or CTRL-D pressed.
 * @retval false        operation successful.
 *
 * @api
 */
bool shellGetLine(ShellConfig *scfg, char *line, unsigned size, ShellHistory *shp) {
  char *p = line;
  BaseSequentialStream *chp = scfg->sc_channel;
#if SHELL_USE_ESC_SEQ == TRUE
  bool escape = false;
  bool bracket = false;
#endif

#if SHELL_USE_HISTORY != TRUE
  (void) shp;
#endif

  while (true) {
    char c;

    if (streamRead(chp, (uint8_t *)&c, 1) == 0)
      return true;
#if SHELL_USE_ESC_SEQ == TRUE
    if (c == 27) {
      escape = true;
      continue;
    }
    if (escape) {
      escape = false;
      if (c == '[') {
        escape = true;
        bracket = true;
        continue;
      }
      if (bracket) {
        bracket = false;
#if SHELL_USE_HISTORY == TRUE
        if (c == 'A') {
          int len = get_history(shp, line, SHELL_HIST_DIR_BK);

          if (len > 0) {
            _shell_reset_cur(chp);
            _shell_clr_line(chp);
            chprintf(chp, "%s", line);
            p = line + len;
          }
          continue;
        }
        if (c == 'B') {
          int len = get_history(shp, line, SHELL_HIST_DIR_FW);

          if (len == 0)
            *line = 0;

          if (len >= 0) {
            _shell_reset_cur(chp);
            _shell_clr_line(chp);
            chprintf(chp, "%s", line);
            p = line + len;
          }
          continue;
        }
#endif
      }
      continue;
    }
#endif
#if (SHELL_CMD_EXIT_ENABLED == TRUE) && !defined(__CHIBIOS_NIL__)
    if (c == 4) {
      chprintf(chp, "^D");
      return true;
    }
#endif
    if ((c == 8) || (c == 127)) {
      if (p != line) {
        streamPut(chp, 0x08);
        streamPut(chp, 0x20);
        streamPut(chp, 0x08);
        p--;
      }
      continue;
    }
    if (c == '\r') {
      chprintf(chp, SHELL_NEWLINE_STR);
#if SHELL_USE_HISTORY == TRUE
      save_history(shp, line, p - line);
#endif
      *p = 0;
      return false;
    }
#if SHELL_USE_COMPLETION == TRUE
    if (c == '\t') {
      if (p < line + size - 1) {
        *p = 0;

        get_completions(scfg, line);
        int len = process_completions(scfg, line, p - line, size);
        if (len > 0) {
          write_completions(scfg, line, p - line);
          p = line + len;
        }
      }
      continue;
    }
#endif
#if SHELL_USE_HISTORY == TRUE
    if (c == 14) {
      int len = get_history(shp, line, SHELL_HIST_DIR_FW);

      if (len == 0)
        *line = 0;

      if (len >= 0) {
        _shell_reset_cur(chp);
        _shell_clr_line(chp);
        chprintf(chp, "%s", line);
        p = line + len;
      }
      continue;
    }
    if (c == 16) {
      int len = get_history(shp, line, SHELL_HIST_DIR_BK);

      if (len > 0) {
        _shell_reset_cur(chp);
        _shell_clr_line(chp);
        chprintf(chp, "%s", line);
        p = line + len;
      }
      continue;
    }
#endif
    if (c < 0x20)
      continue;
    if (p < line + size - 1) {
      streamPut(chp, c);
      *p++ = (char)c;
    }
  }
}

/** @} */
