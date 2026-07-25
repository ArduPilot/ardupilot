/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    ch_test.h
 * @brief   Unit Tests Engine Module macros and structures.
 *
 * @addtogroup CH_TEST
 * @{
 */

#ifndef CH_TEST_H
#define CH_TEST_H

#include <stdarg.h>

#if defined(TEST_USE_CFG_FILE)
#include "testconf.h"
#endif

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#if !defined(FALSE)
#define FALSE                               0
#endif

#if !defined(TRUE)
#define TRUE                                (!FALSE)
#endif

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Enables ChibiOS-awareness features.
 */
#if !defined(TEST_CFG_CHIBIOS_SUPPORT) || defined(__DOXYGEN__)
#define TEST_CFG_CHIBIOS_SUPPORT            TRUE
#endif

/**
 * @brief   Maximum number of entries in the tokens buffer.
 */
#if !defined(TEST_CFG_EOL_STRING) || defined(__DOXYGEN__)
#define TEST_CFG_EOL_STRING                 "\r\n"
#endif

/**
 * @brief   Maximum number of entries in the tokens buffer.
 */
#if !defined(TEST_CFG_MAX_TOKENS) || defined(__DOXYGEN__)
#define TEST_CFG_MAX_TOKENS                 16
#endif

/**
 * @brief   Delay inserted between test cases.
 * @note    Requires @p TEST_CFG_NO_OS_DEPENDENCIES set to @p TRUE.
 */
#if !defined(TEST_CFG_DELAY_BETWEEN_TESTS) || defined(__DOXYGEN__)
#define TEST_CFG_DELAY_BETWEEN_TESTS        200
#endif

/**
 * @brief   Shows a sequence header if enabled.
 */
#if !defined(TEST_CFG_SHOW_SEQUENCES) || defined(__DOXYGEN__)
#define TEST_CFG_SHOW_SEQUENCES             TRUE
#endif

/**
 * @brief   Print executable sizes.
 * @note    Requires specific linker scatter symbols.
 */
#if !defined(TEST_CFG_SIZE_REPORT) || defined(__DOXYGEN__)
#define TEST_CFG_SIZE_REPORT                TRUE
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if TEST_CFG_CHIBIOS_SUPPORT == TRUE
#include <stdarg.h>

#include "hal.h"

#else /* TEST_CFG_CHIBIOS_SUPPORT == FALSE */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#endif /* TEST_CFG_CHIBIOS_SUPPORT == FALSE */

#if (TEST_CFG_CHIBIOS_SUPPORT == FALSE) && (TEST_CFG_DELAY_BETWEEN_TESTS==TRUE)
#error "TEST_CFG_DELAY_BETWEEN_TESTS requires TEST_CFG_CHIBIOS_SUPPORT"
#endif

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a put char function.
 * @note    Follows the standard @p putchar() prototype.
 */
typedef int (*test_putchar_t)(int c);

/**
 * @brief   Type of a test engine context structure.
 */
typedef struct {
  /**
   * @brief   Test step being executed.
   */
  unsigned          current_step;
 /**
   * @brief   Global test result flag.
   */
  bool              global_fail;
  /**
   * @brief   Local test result flag.
   */
  bool              local_fail;
  /**
   * @brief   Last test failure message.
   */
  const char        *failure_message;
  /**
   * @brief   Pointer to the next token position.
   */
  char              *tokp;
  /**
   * @brief   Tokens buffer.
   */
  char              tokens_buffer[TEST_CFG_MAX_TOKENS];
  /**
   * @brief   Current output function.
   */
  test_putchar_t    putchar;
#if (TEST_CFG_CHIBIOS_SUPPORT == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Current output stream.
   */
  BaseSequentialStream *stream;
#endif
} ch_test_context_t;

/**
 * @brief   Structure representing a test case.
 */
typedef struct {
  const char *name;             /**< @brief Test case name.                 */
  void (*setup)(void);          /**< @brief Test case preparation function. */
  void (*teardown)(void);       /**< @brief Test case clean up function.    */
  void (*execute)(void);        /**< @brief Test case execution function.   */
} testcase_t;

/**
 * @brief   Structure representing a test sequence.
 */
typedef struct {
  const char        *name;          /**< @brief Name of the test sequence.  */
  const testcase_t * const * cases; /**< @brief Test cases array.           */
} testsequence_t;

/**
 * @brief   Type of a test suite.
 */
typedef struct {
  const char        *name;          /**< @brief Name of the test suite.     */
  const testsequence_t * const * sequences; /**< @brief Test sequences array.           */
} testsuite_t;

/**
 * @brief   Type of a test suite.
 */
//typedef const testcase_t * const *testsuite_t[];

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Sets the step identifier.
 *
 * @param[in] step      the step number
 */
#define test_set_step(step) chtest.current_step = (step)

/**
 * @brief   End step marker.
 *
 * @param[in] step      the step number
 */
#define test_end_step(step) (void)(step);

/**
 * @brief   Test failure enforcement.
 * @note    This function can only be called from test_case execute context.
 *
 * @param[in] msg       failure message as string
 *
 * @api
 */
#define test_fail(msg) {                                                    \
  __test_fail(msg);                                                         \
  return;                                                                   \
}

/**
 * @brief   Test assertion.
 * @note    This function can only be called from test_case execute context.
 *
 * @param[in] condition a boolean expression that must be verified to be true
 * @param[in] msg       failure message as string
 *
 * @api
 */
#define test_assert(condition, msg) {                                       \
  if (__test_assert(condition, msg))                                        \
    return;                                                                 \
}

/**
 * @brief   Test sequence assertion.
 * @note    This function can only be called from test_case execute context.
 *
 * @param[in] expected  string to be matched with the tokens buffer
 * @param[in] msg       failure message as string
 *
 * @api
 */
#define test_assert_sequence(expected, msg) {                               \
  if (__test_assert_sequence(expected, msg))                                \
    return;                                                                 \
}

#if (TEST_CFG_CHIBIOS_SUPPORT == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Test assertion with lock.
 * @note    This function can only be called from test_case execute context.
 *
 * @param[in] condition a boolean expression that must be verified to be true
 * @param[in] msg       failure message as string
 *
 * @api
 */
#define test_assert_lock(condition, msg) {                                  \
  osalSysLock();                                                            \
  if (__test_assert(condition, msg)) {                                      \
    osalSysUnlock();                                                        \
    return;                                                                 \
  }                                                                         \
  osalSysUnlock();                                                          \
}

/**
 * @brief   Test time window assertion.
 * @note    This function can only be called from test_case execute context.
 *
 * @param[in] start     initial time in the window (included)
 * @param[in] end       final time in the window (not included)
 * @param[in] msg       failure message as string
 *
 * @api
 */
#define test_assert_time_window(start, end, msg) {                          \
  if (__test_assert_time_window(start, end, msg))                           \
    return;                                                                 \
}
#endif /* TEST_CFG_CHIBIOS_SUPPORT == TRUE */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
extern ch_test_context_t chtest;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  bool __test_fail(const char *message);
  bool __test_assert(bool condition, const char *msg);
  bool __test_assert_sequence(char *expected, const char *msg);
#if TEST_CFG_CHIBIOS_SUPPORT == TRUE
  bool __test_assert_time_window(systime_t start,
                                 systime_t end,
                                 const char *msg);
#endif
  void test_putchar(char c);
  int test_vprintf(const char *fmt, va_list ap);
  int test_printf(const char *fmt, ...);
  void test_emit_token(char token);
  bool test_execute_putchar(test_putchar_t putfunc,
                            const testsuite_t *tsp);
#if TEST_CFG_CHIBIOS_SUPPORT == TRUE
  bool test_execute_stream(BaseSequentialStream *stream,
                           const testsuite_t *tsp);
#endif
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#if (TEST_CFG_CHIBIOS_SUPPORT == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Test execution thread function.
 * @note    This is a legacy version.
 *
 * @param[in] stream    pointer to a @p BaseSequentialStream object for test
 *                      output
 * @param[in] tsp       test suite to execute
 * @return              A failure boolean.
 * @retval false        if no errors occurred.
 * @retval true         if one or more tests failed.
 *
 * @api
 */
static inline msg_t test_execute(BaseSequentialStream *stream,
                                 const testsuite_t *tsp) {

  return (msg_t)test_execute_stream(stream, tsp);
}
#endif

/**
 * @brief   Prints a decimal unsigned number.
 *
 * @param[in] n         the number to be printed
 *
 * @api
 */
static inline void test_printn(uint32_t n) {

  test_printf("%u", n);
}

/**
 * @brief   Prints a line without final end-of-line.
 *
 * @param[in] msgp      the message
 *
 * @api
 */
static inline void test_print(const char *msgp) {

  test_printf("%s", msgp);
}

/**
 * @brief   Prints a line.
 *
 * @param[in] msgp      the message
 *
 * @api
 */
static inline void test_println(const char *msgp) {

  test_printf("%s" TEST_CFG_EOL_STRING, msgp);
}

#endif /* CH_TEST_H */

/** @} */
