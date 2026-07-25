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
 * @file    shell.h
 * @brief   Simple CLI shell header.
 *
 * @addtogroup SHELL
 * @{
 */

#ifndef SHELL_H
#define SHELL_H

#if defined(SHELL_CONFIG_FILE)
#include "shellconf.h"
#endif

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Shell History Constants
 */
#define SHELL_HIST_DIR_BK           0
#define SHELL_HIST_DIR_FW           1

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Shell maximum input line length.
 */
#if !defined(SHELL_MAX_LINE_LENGTH) || defined(__DOXYGEN__)
#define SHELL_MAX_LINE_LENGTH       64
#endif

/**
 * @brief   Shell maximum arguments per command.
 */
#if !defined(SHELL_MAX_ARGUMENTS) || defined(__DOXYGEN__)
#define SHELL_MAX_ARGUMENTS         4
#endif

/**
 * @brief   Shell maximum command history.
 */
#if !defined(SHELL_MAX_HIST_BUFF) || defined(__DOXYGEN__)
#define SHELL_MAX_HIST_BUFF         8 * SHELL_MAX_LINE_LENGTH
#endif

/**
 * @brief   Enable shell command history
 */
#if !defined(SHELL_USE_HISTORY) || defined(__DOXYGEN__)
#define SHELL_USE_HISTORY           FALSE
#endif

/**
 * @brief   Enable shell command completion
 */
#if !defined(SHELL_USE_COMPLETION) || defined(__DOXYGEN__)
#define SHELL_USE_COMPLETION        FALSE
#endif

/**
 * @brief   Shell Maximum Completions (Set to max commands with common prefix)
 */
#if !defined(SHELL_MAX_COMPLETIONS) || defined(__DOXYGEN__)
#define SHELL_MAX_COMPLETIONS       8
#endif

/**
 * @brief   Enable shell escape sequence processing
 */
#if !defined(SHELL_USE_ESC_SEQ) || defined(__DOXYGEN__)
#define SHELL_USE_ESC_SEQ           FALSE
#endif

/**
 * @brief   Prompt string
 */
#if !defined(SHELL_PROMPT_STR) || defined(__DOXYGEN__)
#define SHELL_PROMPT_STR            "ch> "
#endif

/**
 * @brief   Newline string
 */
#if !defined(SHELL_NEWLINE_STR) || defined(__DOXYGEN__)
#define SHELL_NEWLINE_STR            "\r\n"
#endif

/**
 * @brief   Default shell thread name.
 */
#if !defined(SHELL_THREAD_NAME) || defined(__DOXYGEN__)
#define SHELL_THREAD_NAME           "shell"
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Command handler function type.
 */
typedef void (*shellcmd_t)(BaseSequentialStream *chp, int argc, char *argv[]);

/**
 * @brief   Custom command entry type.
 */
typedef struct {
  const char            *sc_name;           /**< @brief Command name.       */
  shellcmd_t            sc_function;        /**< @brief Command function.   */
} ShellCommand;

/**
 * @brief   Shell history type.
 */
typedef struct {
  char                   *sh_buffer;        /**< @brief Buffer to store command
                                                 history.                   */
  const int              sh_size;           /**< @brief Shell history buffer
                                                 size.                      */
  int                    sh_beg;            /**< @brief Beginning command index
                                                 in buffer.                 */
  int                    sh_end;            /**< @brief Ending command index
                                                 in buffer.                 */
  int                    sh_cur;            /**< @brief Currently selected
                                                 command in buffer.         */
} ShellHistory;

/**
 * @brief   Shell descriptor type.
 */
typedef struct {
  BaseSequentialStream  *sc_channel;        /**< @brief I/O channel associated
                                                 to the shell.              */
  const ShellCommand    *sc_commands;       /**< @brief Shell extra commands
                                                 table.                     */
#if (SHELL_USE_HISTORY == TRUE) || defined(__DOXYGEN__)
  char                  *sc_histbuf;        /**< @brief Shell command history
                                                 buffer.                    */
  const int             sc_histsize;        /**< @brief Shell history buffer
                                                 size.                      */
#endif
#if (SHELL_USE_COMPLETION == TRUE) || defined(__DOXYGEN__)
  char                  **sc_completion;    /**< @brief Shell command completion
                                                 buffer.                    */
#endif
} ShellConfig;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Send escape codes to move cursor to the beginning of the line
 *
 * @param[in] stream    pointer to a @p BaseSequentialStream object
 *
 * @notapi
 */
#define _shell_reset_cur(stream) chprintf(stream, "\033[%dD\033[%dC",       \
                                          SHELL_MAX_LINE_LENGTH +           \
                                          strlen(SHELL_PROMPT_STR) + 2,     \
                                          strlen(SHELL_PROMPT_STR))

/**
 * @brief   Send escape codes to clear the rest of the line
 *
 * @param[in] stream    pointer to a @p BaseSequentialStream object
 *
 * @notapi
 */
#define _shell_clr_line(stream)   chprintf(stream, "\033[K")

/**
 * @brief   Prints out usage message
 *
 * @param[in] stream    pointer to a @p BaseSequentialStream object
 * @param[in] message   pointer to message string
 *
 * @api
 */
#define shellUsage(stream, message)                                         \
  chprintf(stream, "Usage: %s" SHELL_NEWLINE_STR, message)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
extern event_source_t shell_terminated;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void shellInit(void);
  THD_FUNCTION(shellThread, p);
  void shellExit(msg_t msg);
  bool shellGetLine(ShellConfig *scfg, char *line,
                    unsigned size, ShellHistory *shp);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* SHELL_H */

/** @} */
