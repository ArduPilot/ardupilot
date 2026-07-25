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

#include "hal.h"
#include "rt_test_root.h"

/**
 * @file    rt_test_sequence_002.c
 * @brief   Test Sequence 002 code.
 *
 * @page rt_test_sequence_002 [2] System layer and port interface
 *
 * File: @ref rt_test_sequence_002.c
 *
 * <h2>Description</h2>
 * The functionality of the system layer and port interface is tested.
 * Basic RT functionality is taken for granted or this test suite could
 * not even be executed. Errors in implementation are detected by
 * executing this sequence with the state checker enabled
 * (CH_DBG_STATE_CHECKER=TRUE).
 *
 * <h2>Test Cases</h2>
 * - @subpage rt_test_002_001
 * - @subpage rt_test_002_002
 * - @subpage rt_test_002_003
 * .
 */

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#if CH_PORT_SUPPORTS_RECURSIVE_LOCKS == TRUE
/* Timer callback for testing system functions in ISR context.*/
static void vtcb(virtual_timer_t *vtp, void *p) {
  syssts_t sts;

  (void)vtp;
  (void)p;

  /* Testing normal case.*/
  chSysLockFromISR();
  chSysUnlockFromISR();

  /* Reentrant case.*/
  chSysLockFromISR();
  sts = chSysGetStatusAndLockX();
  chSysRestoreStatusX(sts);
  chSysUnlockFromISR();
}
#endif

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page rt_test_002_001 [2.1] System integrity functionality
 *
 * <h2>Description</h2>
 * The system self-test functionality is invoked in order to make an
 * initial system state assessment and for coverage.
 *
 * <h2>Test Steps</h2>
 * - [2.1.1] Testing Ready List integrity.
 * - [2.1.2] Testing Virtual Timers List integrity.
 * - [2.1.3] Testing Registry List integrity.
 * - [2.1.4] Testing Port-defined integrity.
 * .
 */

static void rt_test_002_001_execute(void) {
  bool result;

  /* [2.1.1] Testing Ready List integrity.*/
  test_set_step(1);
  {
    chSysLock();
    result = chSysIntegrityCheckI(CH_INTEGRITY_RLIST);
    chSysUnlock();
    test_assert(result == false, "ready list check failed");
  }
  test_end_step(1);

  /* [2.1.2] Testing Virtual Timers List integrity.*/
  test_set_step(2);
  {
    chSysLock();
    result = chSysIntegrityCheckI(CH_INTEGRITY_VTLIST);
    chSysUnlock();
    test_assert(result == false, "virtual timers list check failed");
  }
  test_end_step(2);

  /* [2.1.3] Testing Registry List integrity.*/
  test_set_step(3);
  {
    chSysLock();
    result = chSysIntegrityCheckI(CH_INTEGRITY_REGISTRY);
    chSysUnlock();
    test_assert(result == false, "registry list check failed");
  }
  test_end_step(3);

  /* [2.1.4] Testing Port-defined integrity.*/
  test_set_step(4);
  {
    chSysLock();
    result = chSysIntegrityCheckI(CH_INTEGRITY_PORT);
    chSysUnlock();
    test_assert(result == false, "port layer check failed");
  }
  test_end_step(4);
}

static const testcase_t rt_test_002_001 = {
  "System integrity functionality",
  NULL,
  NULL,
  rt_test_002_001_execute
};

#if (CH_PORT_SUPPORTS_RECURSIVE_LOCKS == TRUE) || defined(__DOXYGEN__)
/**
 * @page rt_test_002_002 [2.2] Critical zones functionality
 *
 * <h2>Description</h2>
 * The critical zones API is invoked for coverage.
 *
 * <h2>Conditions</h2>
 * This test is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_PORT_SUPPORTS_RECURSIVE_LOCKS == TRUE
 * .
 *
 * <h2>Test Steps</h2>
 * - [2.2.1] Testing chSysGetStatusAndLockX() and
 *   chSysRestoreStatusX(), non reentrant case.
 * - [2.2.2] Testing chSysGetStatusAndLockX() and
 *   chSysRestoreStatusX(), reentrant case.
 * - [2.2.3] Testing chSysUnconditionalLock().
 * - [2.2.4] Testing chSysUnconditionalUnlock().
 * - [2.2.5] Testing from ISR context using a virtual timer.
 * .
 */

static void rt_test_002_002_execute(void) {
  syssts_t sts;
  virtual_timer_t vt;

  /* [2.2.1] Testing chSysGetStatusAndLockX() and
     chSysRestoreStatusX(), non reentrant case.*/
  test_set_step(1);
  {
    sts = chSysGetStatusAndLockX();
    chSysRestoreStatusX(sts);
  }
  test_end_step(1);

  /* [2.2.2] Testing chSysGetStatusAndLockX() and
     chSysRestoreStatusX(), reentrant case.*/
  test_set_step(2);
  {
    chSysLock();
    sts = chSysGetStatusAndLockX();
    chSysRestoreStatusX(sts);
    chSysUnlock();
  }
  test_end_step(2);

  /* [2.2.3] Testing chSysUnconditionalLock().*/
  test_set_step(3);
  {
    chSysUnconditionalLock();
    chSysUnconditionalLock();
    chSysUnlock();
  }
  test_end_step(3);

  /* [2.2.4] Testing chSysUnconditionalUnlock().*/
  test_set_step(4);
  {
    chSysLock();
    chSysUnconditionalUnlock();
    chSysUnconditionalUnlock();
  }
  test_end_step(4);

  /* [2.2.5] Testing from ISR context using a virtual timer.*/
  test_set_step(5);
  {
    chVTObjectInit(&vt);
    chVTSet(&vt, 1, vtcb, NULL);
    chThdSleep(10);

    test_assert(chVTIsArmed(&vt) == false, "timer still armed");
  }
  test_end_step(5);
}

static const testcase_t rt_test_002_002 = {
  "Critical zones functionality",
  NULL,
  NULL,
  rt_test_002_002_execute
};
#endif /* CH_PORT_SUPPORTS_RECURSIVE_LOCKS == TRUE */

/**
 * @page rt_test_002_003 [2.3] Interrupts handling functionality
 *
 * <h2>Description</h2>
 * The interrupts handling API is invoked for coverage.
 *
 * <h2>Test Steps</h2>
 * - [2.3.1] Testing chSysSuspend(), chSysDisable() and chSysEnable().
 * .
 */

static void rt_test_002_003_execute(void) {

  /* [2.3.1] Testing chSysSuspend(), chSysDisable() and
     chSysEnable().*/
  test_set_step(1);
  {
    chSysSuspend();
    chSysDisable();
    chSysSuspend();
    chSysEnable();
  }
  test_end_step(1);
}

static const testcase_t rt_test_002_003 = {
  "Interrupts handling functionality",
  NULL,
  NULL,
  rt_test_002_003_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const rt_test_sequence_002_array[] = {
  &rt_test_002_001,
#if (CH_PORT_SUPPORTS_RECURSIVE_LOCKS == TRUE) || defined(__DOXYGEN__)
  &rt_test_002_002,
#endif
  &rt_test_002_003,
  NULL
};

/**
 * @brief   System layer and port interface.
 */
const testsequence_t rt_test_sequence_002 = {
  "System layer and port interface",
  rt_test_sequence_002_array
};
