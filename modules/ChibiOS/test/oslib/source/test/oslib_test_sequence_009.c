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
#include "oslib_test_root.h"

/**
 * @file    oslib_test_sequence_009.c
 * @brief   Test Sequence 009 code.
 *
 * @page oslib_test_sequence_009 [9] Objects Factory
 *
 * File: @ref oslib_test_sequence_009.c
 *
 * <h2>Description</h2>
 * This sequence tests the ChibiOS library functionalities related to
 * the object factory.
 *
 * <h2>Conditions</h2>
 * This sequence is only executed if the following preprocessor condition
 * evaluates to true:
 * - (CH_CFG_USE_FACTORY == TRUE) && (CH_CFG_USE_MEMPOOLS == TRUE) && (CH_CFG_USE_HEAP == TRUE)
 * .
 *
 * <h2>Test Cases</h2>
 * - @subpage oslib_test_009_001
 * - @subpage oslib_test_009_002
 * - @subpage oslib_test_009_003
 * - @subpage oslib_test_009_004
 * - @subpage oslib_test_009_005
 * - @subpage oslib_test_009_006
 * .
 */

#if ((CH_CFG_USE_FACTORY == TRUE) && (CH_CFG_USE_MEMPOOLS == TRUE) && (CH_CFG_USE_HEAP == TRUE)) || defined(__DOXYGEN__)

/****************************************************************************
 * Shared code.
 ****************************************************************************/


/****************************************************************************
 * Test cases.
 ****************************************************************************/

#if (CH_CFG_FACTORY_OBJECTS_REGISTRY == TRUE) || defined(__DOXYGEN__)
/**
 * @page oslib_test_009_001 [9.1] Objects Registry
 *
 * <h2>Description</h2>
 * This test case verifies the static objects registry.
 *
 * <h2>Conditions</h2>
 * This test is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_FACTORY_OBJECTS_REGISTRY == TRUE
 * .
 *
 * <h2>Test Steps</h2>
 * - [9.1.1] Retrieving a registered object by name, must not exist.
 * - [9.1.2] Registering an object, it must not exists, must succeed.
 * - [9.1.3] Registering an object with the same name, must fail.
 * - [9.1.4] Retrieving the registered object by name, must exist, then
 *   increasing the reference counter, finally releasing both
 *   references.
 * - [9.1.5] Releasing the first reference to the object, must not
 *   trigger an assertion.
 * - [9.1.6] Retrieving the registered object by name again, must not
 *   exist.
 * .
 */

static void oslib_test_009_001_teardown(void) {
  registered_object_t *rop;

  rop = chFactoryFindObject("myobj");
  if (rop != NULL) {
    while (rop->element.refs > 0U) {
      chFactoryReleaseObject(rop);
    }
  }
}

static void oslib_test_009_001_execute(void) {
  registered_object_t *rop;

  /* [9.1.1] Retrieving a registered object by name, must not exist.*/
  test_set_step(1);
  {
    rop = chFactoryFindObject("myobj");
    test_assert(rop == NULL, "found");
  }
  test_end_step(1);

  /* [9.1.2] Registering an object, it must not exists, must succeed.*/
  test_set_step(2);
  {
    static uint32_t myobj = 0x55aa;

    rop = chFactoryRegisterObject("myobj", (void *)&myobj);
    test_assert(rop != NULL, "cannot register");
  }
  test_end_step(2);

  /* [9.1.3] Registering an object with the same name, must fail.*/
  test_set_step(3);
  {
    registered_object_t *rop1;
    static uint32_t myobj = 0x55aa;

    rop1 = chFactoryRegisterObject("myobj", (void *)&myobj);
    test_assert(rop1 == NULL, "can register");
  }
  test_end_step(3);

  /* [9.1.4] Retrieving the registered object by name, must exist, then
     increasing the reference counter, finally releasing both
     references.*/
  test_set_step(4);
  {
    registered_object_t *rop1, *rop2;

    rop1 = chFactoryFindObject("myobj");
    test_assert(rop1 != NULL, "not found");
    test_assert(*(uint32_t *)(rop1->objp) == 0x55aa, "object mismatch");
    test_assert(rop == rop1, "object reference mismatch");
    test_assert(rop1->element.refs == 2, "object reference mismatch");

    rop2 = (registered_object_t *)chFactoryDuplicateReference(&rop1->element);
    test_assert(rop1 == rop2, "object reference mismatch");
    test_assert(*(uint32_t *)(rop2->objp) == 0x55aa, "object mismatch");
    test_assert(rop2->element.refs == 3, "object reference mismatch");

    chFactoryReleaseObject(rop2);
    test_assert(rop1->element.refs == 2, "references mismatch");

    chFactoryReleaseObject(rop1);
    test_assert(rop->element.refs == 1, "references mismatch");
  }
  test_end_step(4);

  /* [9.1.5] Releasing the first reference to the object, must not
     trigger an assertion.*/
  test_set_step(5);
  {
    chFactoryReleaseObject(rop);
  }
  test_end_step(5);

  /* [9.1.6] Retrieving the registered object by name again, must not
     exist.*/
  test_set_step(6);
  {
    rop = chFactoryFindObject("myobj");
    test_assert(rop == NULL, "found");
  }
  test_end_step(6);
}

static const testcase_t oslib_test_009_001 = {
  "Objects Registry",
  NULL,
  oslib_test_009_001_teardown,
  oslib_test_009_001_execute
};
#endif /* CH_CFG_FACTORY_OBJECTS_REGISTRY == TRUE */

#if (CH_CFG_FACTORY_GENERIC_BUFFERS == TRUE) || defined(__DOXYGEN__)
/**
 * @page oslib_test_009_002 [9.2] Dynamic Buffers Factory
 *
 * <h2>Description</h2>
 * This test case verifies the dynamic buffers factory.
 *
 * <h2>Conditions</h2>
 * This test is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_FACTORY_GENERIC_BUFFERS == TRUE
 * .
 *
 * <h2>Test Steps</h2>
 * - [9.2.1] Retrieving a dynamic buffer by name, must not exist.
 * - [9.2.2] Creating a dynamic buffer it must not exists, must
 *   succeed.
 * - [9.2.3] Creating a dynamic buffer with the same name, must fail.
 * - [9.2.4] Retrieving the dynamic buffer by name, must exist, then
 *   increasing the reference counter, finally releasing both
 *   references.
 * - [9.2.5] Releasing the first reference to the dynamic buffer, must
 *   not trigger an assertion.
 * - [9.2.6] Retrieving the dynamic buffer by name again, must not
 *   exist.
 * .
 */

static void oslib_test_009_002_teardown(void) {
  dyn_buffer_t *dbp;

  dbp = chFactoryFindBuffer("mybuf");
  if (dbp != NULL) {
    while (dbp->element.refs > 0U) {
      chFactoryReleaseBuffer(dbp);
    }
  }
}

static void oslib_test_009_002_execute(void) {
  dyn_buffer_t *dbp;

  /* [9.2.1] Retrieving a dynamic buffer by name, must not exist.*/
  test_set_step(1);
  {
    dbp = chFactoryFindBuffer("mybuf");
    test_assert(dbp == NULL, "found");
  }
  test_end_step(1);

  /* [9.2.2] Creating a dynamic buffer it must not exists, must
     succeed.*/
  test_set_step(2);
  {
    dbp = chFactoryCreateBuffer("mybuf", 128U);
    test_assert(dbp != NULL, "cannot create");
  }
  test_end_step(2);

  /* [9.2.3] Creating a dynamic buffer with the same name, must fail.*/
  test_set_step(3);
  {
    dyn_buffer_t *dbp1;

    dbp1 = chFactoryCreateBuffer("mybuf", 128U);
    test_assert(dbp1 == NULL, "can create");
  }
  test_end_step(3);

  /* [9.2.4] Retrieving the dynamic buffer by name, must exist, then
     increasing the reference counter, finally releasing both
     references.*/
  test_set_step(4);
  {
    dyn_buffer_t *dbp1, *dbp2;

    dbp1 = chFactoryFindBuffer("mybuf");
    test_assert(dbp1 != NULL, "not found");
    test_assert(dbp == dbp1, "object reference mismatch");
    test_assert(dbp1->element.refs == 2, "object reference mismatch");

    dbp2 = (dyn_buffer_t *)chFactoryDuplicateReference(&dbp1->element);
    test_assert(dbp1 == dbp2, "object reference mismatch");
    test_assert(dbp2->element.refs == 3, "object reference mismatch");

    chFactoryReleaseBuffer(dbp2);
    test_assert(dbp1->element.refs == 2, "references mismatch");

    chFactoryReleaseBuffer(dbp1);
    test_assert(dbp->element.refs == 1, "references mismatch");
  }
  test_end_step(4);

  /* [9.2.5] Releasing the first reference to the dynamic buffer, must
     not trigger an assertion.*/
  test_set_step(5);
  {
    chFactoryReleaseBuffer(dbp);
  }
  test_end_step(5);

  /* [9.2.6] Retrieving the dynamic buffer by name again, must not
     exist.*/
  test_set_step(6);
  {
    dbp = chFactoryFindBuffer("mybuf");
    test_assert(dbp == NULL, "found");
  }
  test_end_step(6);
}

static const testcase_t oslib_test_009_002 = {
  "Dynamic Buffers Factory",
  NULL,
  oslib_test_009_002_teardown,
  oslib_test_009_002_execute
};
#endif /* CH_CFG_FACTORY_GENERIC_BUFFERS == TRUE */

#if (CH_CFG_FACTORY_SEMAPHORES == TRUE) || defined(__DOXYGEN__)
/**
 * @page oslib_test_009_003 [9.3] Dynamic Semaphores Factory
 *
 * <h2>Description</h2>
 * This test case verifies the dynamic semaphores factory.
 *
 * <h2>Conditions</h2>
 * This test is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_FACTORY_SEMAPHORES == TRUE
 * .
 *
 * <h2>Test Steps</h2>
 * - [9.3.1] Retrieving a dynamic semaphore by name, must not exist.
 * - [9.3.2] Creating a dynamic semaphore it must not exists, must
 *   succeed.
 * - [9.3.3] Creating a dynamic semaphore with the same name, must
 *   fail.
 * - [9.3.4] Retrieving the dynamic semaphore by name, must exist, then
 *   increasing the reference counter, finally releasing both
 *   references.
 * - [9.3.5] Releasing the first reference to the dynamic semaphore
 *   must not trigger an assertion.
 * - [9.3.6] Retrieving the dynamic semaphore by name again, must not
 *   exist.
 * .
 */

static void oslib_test_009_003_teardown(void) {
  dyn_semaphore_t *dsp;

  dsp = chFactoryFindSemaphore("mysem");
  if (dsp != NULL) {
    while (dsp->element.refs > 0U) {
      chFactoryReleaseSemaphore(dsp);
    }
  }
}

static void oslib_test_009_003_execute(void) {
  dyn_semaphore_t *dsp;

  /* [9.3.1] Retrieving a dynamic semaphore by name, must not exist.*/
  test_set_step(1);
  {
    dsp = chFactoryFindSemaphore("mysem");
    test_assert(dsp == NULL, "found");
  }
  test_end_step(1);

  /* [9.3.2] Creating a dynamic semaphore it must not exists, must
     succeed.*/
  test_set_step(2);
  {
    dsp = chFactoryCreateSemaphore("mysem", 0);
    test_assert(dsp != NULL, "cannot create");
  }
  test_end_step(2);

  /* [9.3.3] Creating a dynamic semaphore with the same name, must
     fail.*/
  test_set_step(3);
  {
    dyn_semaphore_t *dsp1;

    dsp1 = chFactoryCreateSemaphore("mysem", 0);
    test_assert(dsp1 == NULL, "can create");
  }
  test_end_step(3);

  /* [9.3.4] Retrieving the dynamic semaphore by name, must exist, then
     increasing the reference counter, finally releasing both
     references.*/
  test_set_step(4);
  {
    dyn_semaphore_t *dsp1, *dsp2;

    dsp1 = chFactoryFindSemaphore("mysem");
    test_assert(dsp1 != NULL, "not found");
    test_assert(dsp == dsp1, "object reference mismatch");
    test_assert(dsp1->element.refs == 2, "object reference mismatch");

    dsp2 = (dyn_semaphore_t *)chFactoryDuplicateReference(&dsp1->element);
    test_assert(dsp1 == dsp2, "object reference mismatch");
    test_assert(dsp2->element.refs == 3, "object reference mismatch");

    chFactoryReleaseSemaphore(dsp2);
    test_assert(dsp1->element.refs == 2, "references mismatch");

    chFactoryReleaseSemaphore(dsp1);
    test_assert(dsp->element.refs == 1, "references mismatch");
  }
  test_end_step(4);

  /* [9.3.5] Releasing the first reference to the dynamic semaphore
     must not trigger an assertion.*/
  test_set_step(5);
  {
    chFactoryReleaseSemaphore(dsp);
  }
  test_end_step(5);

  /* [9.3.6] Retrieving the dynamic semaphore by name again, must not
     exist.*/
  test_set_step(6);
  {
    dsp = chFactoryFindSemaphore("mysem");
    test_assert(dsp == NULL, "found");
  }
  test_end_step(6);
}

static const testcase_t oslib_test_009_003 = {
  "Dynamic Semaphores Factory",
  NULL,
  oslib_test_009_003_teardown,
  oslib_test_009_003_execute
};
#endif /* CH_CFG_FACTORY_SEMAPHORES == TRUE */

#if (CH_CFG_FACTORY_MAILBOXES == TRUE) || defined(__DOXYGEN__)
/**
 * @page oslib_test_009_004 [9.4] Dynamic Mailboxes Factory
 *
 * <h2>Description</h2>
 * This test case verifies the dynamic mailboxes factory.
 *
 * <h2>Conditions</h2>
 * This test is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_FACTORY_MAILBOXES == TRUE
 * .
 *
 * <h2>Test Steps</h2>
 * - [9.4.1] Retrieving a dynamic mailbox by name, must not exist.
 * - [9.4.2] Creating a dynamic mailbox it must not exists, must
 *   succeed.
 * - [9.4.3] Creating a dynamic mailbox with the same name, must fail.
 * - [9.4.4] Retrieving the dynamic mailbox by name, must exist, then
 *   increasing the reference counter, finally releasing both
 *   references.
 * - [9.4.5] Releasing the first reference to the dynamic mailbox must
 *   not trigger an assertion.
 * - [9.4.6] Retrieving the dynamic mailbox by name again, must not
 *   exist.
 * .
 */

static void oslib_test_009_004_teardown(void) {
  dyn_mailbox_t *dmp;

  dmp = chFactoryFindMailbox("mymbx");
  if (dmp != NULL) {
    while (dmp->element.refs > 0U) {
      chFactoryReleaseMailbox(dmp);
    }
  }
}

static void oslib_test_009_004_execute(void) {
  dyn_mailbox_t *dmp;

  /* [9.4.1] Retrieving a dynamic mailbox by name, must not exist.*/
  test_set_step(1);
  {
    dmp = chFactoryFindMailbox("mymbx");
    test_assert(dmp == NULL, "found");
  }
  test_end_step(1);

  /* [9.4.2] Creating a dynamic mailbox it must not exists, must
     succeed.*/
  test_set_step(2);
  {
    dmp = chFactoryCreateMailbox("mymbx", 16U);
    test_assert(dmp != NULL, "cannot create");
  }
  test_end_step(2);

  /* [9.4.3] Creating a dynamic mailbox with the same name, must
     fail.*/
  test_set_step(3);
  {
    dyn_mailbox_t *dmp1;

    dmp1 = chFactoryCreateMailbox("mymbx", 16U);
    test_assert(dmp1 == NULL, "can create");
  }
  test_end_step(3);

  /* [9.4.4] Retrieving the dynamic mailbox by name, must exist, then
     increasing the reference counter, finally releasing both
     references.*/
  test_set_step(4);
  {
    dyn_mailbox_t *dmp1, *dmp2;

    dmp1 = chFactoryFindMailbox("mymbx");
    test_assert(dmp1 != NULL, "not found");
    test_assert(dmp == dmp1, "object reference mismatch");
    test_assert(dmp1->element.refs == 2, "object reference mismatch");

    dmp2 = (dyn_mailbox_t *)chFactoryDuplicateReference(&dmp1->element);
    test_assert(dmp1 == dmp2, "object reference mismatch");
    test_assert(dmp2->element.refs == 3, "object reference mismatch");

    chFactoryReleaseMailbox(dmp2);
    test_assert(dmp1->element.refs == 2, "references mismatch");

    chFactoryReleaseMailbox(dmp1);
    test_assert(dmp->element.refs == 1, "references mismatch");
  }
  test_end_step(4);

  /* [9.4.5] Releasing the first reference to the dynamic mailbox must
     not trigger an assertion.*/
  test_set_step(5);
  {
    chFactoryReleaseMailbox(dmp);
  }
  test_end_step(5);

  /* [9.4.6] Retrieving the dynamic mailbox by name again, must not
     exist.*/
  test_set_step(6);
  {
    dmp = chFactoryFindMailbox("mymbx");
    test_assert(dmp == NULL, "found");
  }
  test_end_step(6);
}

static const testcase_t oslib_test_009_004 = {
  "Dynamic Mailboxes Factory",
  NULL,
  oslib_test_009_004_teardown,
  oslib_test_009_004_execute
};
#endif /* CH_CFG_FACTORY_MAILBOXES == TRUE */

#if (CH_CFG_FACTORY_OBJ_FIFOS == TRUE) || defined(__DOXYGEN__)
/**
 * @page oslib_test_009_005 [9.5] Dynamic Objects FIFOs Factory
 *
 * <h2>Description</h2>
 * This test case verifies the dynamic objects FIFOs factory.
 *
 * <h2>Conditions</h2>
 * This test is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_FACTORY_OBJ_FIFOS == TRUE
 * .
 *
 * <h2>Test Steps</h2>
 * - [9.5.1] Retrieving a dynamic objects FIFO by name, must not exist.
 * - [9.5.2] Creating a dynamic objects FIFO it must not exists, must
 *   succeed.
 * - [9.5.3] Creating a dynamic objects FIFO with the same name, must
 *   fail.
 * - [9.5.4] Retrieving the dynamic objects FIFO by name, must exist,
 *   then increasing the reference counter, finally releasing both
 *   references.
 * - [9.5.5] Releasing the first reference to the dynamic objects FIFO
 *   must not trigger an assertion.
 * - [9.5.6] Retrieving the dynamic objects FIFO by name again, must
 *   not exist.
 * .
 */

static void oslib_test_009_005_teardown(void) {
  dyn_objects_fifo_t *dofp;

  dofp = chFactoryFindObjectsFIFO("myfifo");
  if (dofp != NULL) {
    while (dofp->element.refs > 0U) {
      chFactoryReleaseObjectsFIFO(dofp);
    }
  }
}

static void oslib_test_009_005_execute(void) {
  dyn_objects_fifo_t *dofp;

  /* [9.5.1] Retrieving a dynamic objects FIFO by name, must not
     exist.*/
  test_set_step(1);
  {
    dofp = chFactoryFindObjectsFIFO("myfifo");
    test_assert(dofp == NULL, "found");
  }
  test_end_step(1);

  /* [9.5.2] Creating a dynamic objects FIFO it must not exists, must
     succeed.*/
  test_set_step(2);
  {
    dofp = chFactoryCreateObjectsFIFO("myfifo", 16U, 16U, PORT_NATURAL_ALIGN);
    test_assert(dofp != NULL, "cannot create");
  }
  test_end_step(2);

  /* [9.5.3] Creating a dynamic objects FIFO with the same name, must
     fail.*/
  test_set_step(3);
  {
    dyn_objects_fifo_t *dofp1;

    dofp1 = chFactoryCreateObjectsFIFO("myfifo", 16U, 16U, PORT_NATURAL_ALIGN);
    test_assert(dofp1 == NULL, "can create");
  }
  test_end_step(3);

  /* [9.5.4] Retrieving the dynamic objects FIFO by name, must exist,
     then increasing the reference counter, finally releasing both
     references.*/
  test_set_step(4);
  {
    dyn_objects_fifo_t *dofp1, *dofp2;

    dofp1 = chFactoryFindObjectsFIFO("myfifo");
    test_assert(dofp1 != NULL, "not found");
    test_assert(dofp == dofp1, "object reference mismatch");
    test_assert(dofp1->element.refs == 2, "object reference mismatch");

    dofp2 = (dyn_objects_fifo_t *)chFactoryDuplicateReference(&dofp1->element);
    test_assert(dofp1 == dofp2, "object reference mismatch");
    test_assert(dofp2->element.refs == 3, "object reference mismatch");

    chFactoryReleaseObjectsFIFO(dofp2);
    test_assert(dofp1->element.refs == 2, "references mismatch");

    chFactoryReleaseObjectsFIFO(dofp1);
    test_assert(dofp->element.refs == 1, "references mismatch");
  }
  test_end_step(4);

  /* [9.5.5] Releasing the first reference to the dynamic objects FIFO
     must not trigger an assertion.*/
  test_set_step(5);
  {
    chFactoryReleaseObjectsFIFO(dofp);
  }
  test_end_step(5);

  /* [9.5.6] Retrieving the dynamic objects FIFO by name again, must
     not exist.*/
  test_set_step(6);
  {
    dofp = chFactoryFindObjectsFIFO("myfifo");
    test_assert(dofp == NULL, "found");
  }
  test_end_step(6);
}

static const testcase_t oslib_test_009_005 = {
  "Dynamic Objects FIFOs Factory",
  NULL,
  oslib_test_009_005_teardown,
  oslib_test_009_005_execute
};
#endif /* CH_CFG_FACTORY_OBJ_FIFOS == TRUE */

#if (CH_CFG_FACTORY_PIPES == TRUE) || defined(__DOXYGEN__)
/**
 * @page oslib_test_009_006 [9.6] Dynamic Pipes Factory
 *
 * <h2>Description</h2>
 * This test case verifies the dynamic pipes factory.
 *
 * <h2>Conditions</h2>
 * This test is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_FACTORY_PIPES == TRUE
 * .
 *
 * <h2>Test Steps</h2>
 * - [9.6.1] Retrieving a dynamic pipe by name, must not exist.
 * - [9.6.2] Creating a dynamic pipe it must not exists, must succeed.
 * - [9.6.3] Creating a dynamic pipe with the same name, must fail.
 * - [9.6.4] Retrieving the dynamic pipe by name, must exist, then
 *   increasing the reference counter, finally releasing both
 *   references.
 * - [9.6.5] Releasing the first reference to the dynamic pipe must not
 *   trigger an assertion.
 * - [9.6.6] Retrieving the dynamic pipe by name again, must not exist.
 * .
 */

static void oslib_test_009_006_teardown(void) {
  dyn_pipe_t *dpp;

  dpp = chFactoryFindPipe("mypipe");
  if (dpp != NULL) {
    while (dpp->element.refs > 0U) {
      chFactoryReleasePipe(dpp);
    }
  }
}

static void oslib_test_009_006_execute(void) {
  dyn_pipe_t *dpp;

  /* [9.6.1] Retrieving a dynamic pipe by name, must not exist.*/
  test_set_step(1);
  {
    dpp = chFactoryFindPipe("mypipe");
    test_assert(dpp == NULL, "found");
  }
  test_end_step(1);

  /* [9.6.2] Creating a dynamic pipe it must not exists, must
     succeed.*/
  test_set_step(2);
  {
    dpp = chFactoryCreatePipe("mypipe", 16U);
    test_assert(dpp != NULL, "cannot create");
  }
  test_end_step(2);

  /* [9.6.3] Creating a dynamic pipe with the same name, must fail.*/
  test_set_step(3);
  {
    dyn_pipe_t *dpp1;

    dpp1 = chFactoryCreatePipe("mypipe", 16U);
    test_assert(dpp1 == NULL, "can create");
  }
  test_end_step(3);

  /* [9.6.4] Retrieving the dynamic pipe by name, must exist, then
     increasing the reference counter, finally releasing both
     references.*/
  test_set_step(4);
  {
    dyn_pipe_t *dpp1, *dpp2;

    dpp1 = chFactoryFindPipe("mypipe");
    test_assert(dpp1 != NULL, "not found");
    test_assert(dpp == dpp1, "object reference mismatch");
    test_assert(dpp1->element.refs == 2, "object reference mismatch");

    dpp2 = (dyn_pipe_t *)chFactoryDuplicateReference(&dpp1->element);
    test_assert(dpp1 == dpp2, "object reference mismatch");
    test_assert(dpp2->element.refs == 3, "object reference mismatch");

    chFactoryReleasePipe(dpp2);
    test_assert(dpp1->element.refs == 2, "references mismatch");

    chFactoryReleasePipe(dpp1);
    test_assert(dpp->element.refs == 1, "references mismatch");
  }
  test_end_step(4);

  /* [9.6.5] Releasing the first reference to the dynamic pipe must not
     trigger an assertion.*/
  test_set_step(5);
  {
    chFactoryReleasePipe(dpp);
  }
  test_end_step(5);

  /* [9.6.6] Retrieving the dynamic pipe by name again, must not
     exist.*/
  test_set_step(6);
  {
    dpp = chFactoryFindPipe("mypipe");
    test_assert(dpp == NULL, "found");
  }
  test_end_step(6);
}

static const testcase_t oslib_test_009_006 = {
  "Dynamic Pipes Factory",
  NULL,
  oslib_test_009_006_teardown,
  oslib_test_009_006_execute
};
#endif /* CH_CFG_FACTORY_PIPES == TRUE */

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const oslib_test_sequence_009_array[] = {
#if (CH_CFG_FACTORY_OBJECTS_REGISTRY == TRUE) || defined(__DOXYGEN__)
  &oslib_test_009_001,
#endif
#if (CH_CFG_FACTORY_GENERIC_BUFFERS == TRUE) || defined(__DOXYGEN__)
  &oslib_test_009_002,
#endif
#if (CH_CFG_FACTORY_SEMAPHORES == TRUE) || defined(__DOXYGEN__)
  &oslib_test_009_003,
#endif
#if (CH_CFG_FACTORY_MAILBOXES == TRUE) || defined(__DOXYGEN__)
  &oslib_test_009_004,
#endif
#if (CH_CFG_FACTORY_OBJ_FIFOS == TRUE) || defined(__DOXYGEN__)
  &oslib_test_009_005,
#endif
#if (CH_CFG_FACTORY_PIPES == TRUE) || defined(__DOXYGEN__)
  &oslib_test_009_006,
#endif
  NULL
};

/**
 * @brief   Objects Factory.
 */
const testsequence_t oslib_test_sequence_009 = {
  "Objects Factory",
  oslib_test_sequence_009_array
};

#endif /* (CH_CFG_USE_FACTORY == TRUE) && (CH_CFG_USE_MEMPOOLS == TRUE) && (CH_CFG_USE_HEAP == TRUE) */
