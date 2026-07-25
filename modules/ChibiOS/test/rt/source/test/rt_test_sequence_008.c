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
 * @file    rt_test_sequence_008.c
 * @brief   Test Sequence 008 code.
 *
 * @page rt_test_sequence_008 [8] Mutexes, Condition Variables and Priority Inheritance
 *
 * File: @ref rt_test_sequence_008.c
 *
 * <h2>Description</h2>
 * This sequence tests the ChibiOS/RT functionalities related to
 * mutexes, condition variables and priority inheritance algorithm.
 *
 * <h2>Conditions</h2>
 * This sequence is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_MUTEXES == TRUE
 * .
 *
 * <h2>Test Cases</h2>
 * - @subpage rt_test_008_001
 * - @subpage rt_test_008_002
 * - @subpage rt_test_008_003
 * - @subpage rt_test_008_004
 * - @subpage rt_test_008_005
 * - @subpage rt_test_008_006
 * - @subpage rt_test_008_007
 * - @subpage rt_test_008_008
 * - @subpage rt_test_008_009
 * .
 */

#if (CH_CFG_USE_MUTEXES == TRUE) || defined(__DOXYGEN__)

/****************************************************************************
 * Shared code.
 ****************************************************************************/

static MUTEX_DECL(m1);
static MUTEX_DECL(m2);
#if CH_CFG_USE_CONDVARS || defined(__DOXYGEN__)
static CONDVAR_DECL(c1);
#endif

#if CH_DBG_THREADS_PROFILING || defined(__DOXYGEN__)
/**
 * @brief   CPU pulse.
 * @note    The current implementation is not totally reliable.
 *
 * @param[in] duration      CPU pulse duration in milliseconds
 */
void test_cpu_pulse(time_msecs_t duration) {
  systime_t start, end, now;

  start = chThdGetTicksX(chThdGetSelfX());
  end = chTimeAddX(start, TIME_MS2I(duration));
  do {
    now = chThdGetTicksX(chThdGetSelfX());
#if defined(SIMULATOR)
    _sim_check_for_interrupts();
#endif
  }
  while (chTimeIsInRangeX(now, start, end));
}
#endif /* CH_DBG_THREADS_PROFILING */

static THD_FUNCTION(thread1, p) {

  chMtxLock(&m1);
  test_emit_token(*(char *)p);
  chMtxUnlock(&m1);
}

#if CH_DBG_THREADS_PROFILING || defined(__DOXYGEN__)
/* Low priority thread */
static THD_FUNCTION(thread2L, p) {

  (void)p;
  chMtxLock(&m1);
  test_cpu_pulse(40);
  chMtxUnlock(&m1);
  test_cpu_pulse(10);
  test_emit_token('C');
}

/* Medium priority thread */
static THD_FUNCTION(thread2M, p) {

  (void)p;
  chThdSleepMilliseconds(20);
  test_cpu_pulse(40);
  test_emit_token('B');
}

/* High priority thread */
static THD_FUNCTION(thread2H, p) {

  (void)p;
  chThdSleepMilliseconds(40);
  chMtxLock(&m1);
  test_cpu_pulse(10);
  chMtxUnlock(&m1);
  test_emit_token('A');
}

/* Lowest priority thread */
static THD_FUNCTION(thread3LL, p) {

  (void)p;
  chMtxLock(&m1);
  test_cpu_pulse(30);
  chMtxUnlock(&m1);
  test_emit_token('E');
}

/* Low priority thread */
static THD_FUNCTION(thread3L, p) {

  (void)p;
  chThdSleepMilliseconds(10);
  chMtxLock(&m2);
  test_cpu_pulse(20);
  chMtxLock(&m1);
  test_cpu_pulse(10);
  chMtxUnlock(&m1);
  test_cpu_pulse(10);
  chMtxUnlock(&m2);
  test_emit_token('D');
}

/* Medium priority thread */
static THD_FUNCTION(thread3M, p) {

  (void)p;
  chThdSleepMilliseconds(20);
  chMtxLock(&m2);
  test_cpu_pulse(10);
  chMtxUnlock(&m2);
  test_emit_token('C');
}

/* High priority thread */
static THD_FUNCTION(thread3H, p) {

  (void)p;
  chThdSleepMilliseconds(40);
  test_cpu_pulse(20);
  test_emit_token('B');
}

/* Highest priority thread */
static THD_FUNCTION(thread3HH, p) {

  (void)p;
  chThdSleepMilliseconds(50);
  chMtxLock(&m2);
  test_cpu_pulse(10);
  chMtxUnlock(&m2);
  test_emit_token('A');
}
#endif /* CH_DBG_THREADS_PROFILING */

static THD_FUNCTION(thread4A, p) {

  (void)p;
  chThdSleepMilliseconds(50);
  chMtxLock(&m1);
  chMtxUnlock(&m1);
}

static THD_FUNCTION(thread4B, p) {

  (void)p;
  chThdSleepMilliseconds(150);
  chSysLock();
  chMtxLockS(&m2);   /* For coverage of the chMtxLockS() function variant.*/
  chMtxUnlockS(&m2); /* For coverage of the chMtxUnlockS() function variant.*/
  chSchRescheduleS();
  chSysUnlock();
}

#if CH_CFG_USE_CONDVARS || defined(__DOXYGEN__)
static THD_FUNCTION(thread6, p) {

  chMtxLock(&m1);
  chCondWait(&c1);
  test_emit_token(*(char *)p);
  chMtxUnlock(&m1);
}

static THD_FUNCTION(thread8, p) {

  chMtxLock(&m2);
  chMtxLock(&m1);
#if CH_CFG_USE_CONDVARS_TIMEOUT || defined(__DOXYGEN__)
  chCondWaitTimeout(&c1, TIME_INFINITE);
#else
  chCondWait(&c1);
#endif
  test_emit_token(*(char *)p);
  chMtxUnlock(&m1);
  chMtxUnlock(&m2);
}

static THD_FUNCTION(thread9, p) {

  chMtxLock(&m2);
  test_emit_token(*(char *)p);
  chMtxUnlock(&m2);
}
#endif /* CH_CFG_USE_CONDVARS */

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page rt_test_008_001 [8.1] Priority enqueuing test
 *
 * <h2>Description</h2>
 * Five threads, with increasing priority, are enqueued on a locked
 * mutex then the mutex is unlocked. The test expects the threads to
 * perform their operations in increasing priority order regardless of
 * the initial order.
 *
 * <h2>Test Steps</h2>
 * - [8.1.1] Getting the initial priority.
 * - [8.1.2] Locking the mutex.
 * - [8.1.3] Five threads are created that try to lock and unlock the
 *   mutex then terminate. The threads are created in ascending
 *   priority order.
 * - [8.1.4] Unlocking the mutex, the threads will wakeup in priority
 *   order because the mutext queue is an ordered one.
 * .
 */

static void rt_test_008_001_setup(void) {
  chMtxObjectInit(&m1);
}

static void rt_test_008_001_execute(void) {
  tprio_t prio;

  /* [8.1.1] Getting the initial priority.*/
  test_set_step(1);
  {
    prio = chThdGetPriorityX();
  }
  test_end_step(1);

  /* [8.1.2] Locking the mutex.*/
  test_set_step(2);
  {
    chMtxLock(&m1);
  }
  test_end_step(2);

  /* [8.1.3] Five threads are created that try to lock and unlock the
     mutex then terminate. The threads are created in ascending
     priority order.*/
  test_set_step(3);
  {
    threads[0] = chThdCreateStatic(wa[0], WA_SIZE, prio+1, thread1, "E");
    threads[1] = chThdCreateStatic(wa[1], WA_SIZE, prio+2, thread1, "D");
    threads[2] = chThdCreateStatic(wa[2], WA_SIZE, prio+3, thread1, "C");
    threads[3] = chThdCreateStatic(wa[3], WA_SIZE, prio+4, thread1, "B");
    threads[4] = chThdCreateStatic(wa[4], WA_SIZE, prio+5, thread1, "A");
  }
  test_end_step(3);

  /* [8.1.4] Unlocking the mutex, the threads will wakeup in priority
     order because the mutext queue is an ordered one.*/
  test_set_step(4);
  {
    chMtxUnlock(&m1);
    test_wait_threads();
    test_assert(prio == chThdGetPriorityX(), "wrong priority level");
    test_assert_sequence("ABCDE", "invalid sequence");
  }
  test_end_step(4);
}

static const testcase_t rt_test_008_001 = {
  "Priority enqueuing test",
  rt_test_008_001_setup,
  NULL,
  rt_test_008_001_execute
};

#if (CH_DBG_THREADS_PROFILING == TRUE) || defined(__DOXYGEN__)
/**
 * @page rt_test_008_002 [8.2] Priority inheritance, simple case
 *
 * <h2>Description</h2>
 * Three threads are involved in the classic priority inversion
 * scenario, a medium priority thread tries to starve an high priority
 * thread by blocking a low priority thread into a mutex lock zone. The
 * test expects the threads to reach their goal in increasing priority
 * order by rearranging their priorities in order to avoid the priority
 * inversion trap.
 *
 * <h2>Conditions</h2>
 * This test is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_DBG_THREADS_PROFILING == TRUE
 * .
 *
 * <h2>Test Steps</h2>
 * - [8.2.1] Getting the system time for test duration measurement.
 * - [8.2.2] The three contenders threads are created and let run
 *   atomically, the goals sequence is tested, the threads must
 *   complete in priority order.
 * - [8.2.3] Testing that all threads completed within the specified
 *   time windows (100mS...100mS+ALLOWED_DELAY).
 * .
 */

static void rt_test_008_002_setup(void) {
  chMtxObjectInit(&m1);
}

static void rt_test_008_002_execute(void) {
  systime_t time;

  /* [8.2.1] Getting the system time for test duration measurement.*/
  test_set_step(1);
  {
    time = test_wait_tick();
  }
  test_end_step(1);

  /* [8.2.2] The three contenders threads are created and let run
     atomically, the goals sequence is tested, the threads must
     complete in priority order.*/
  test_set_step(2);
  {
    threads[0] = chThdCreateStatic(wa[0], WA_SIZE, chThdGetPriorityX()-1, thread2H, 0);
    threads[1] = chThdCreateStatic(wa[1], WA_SIZE, chThdGetPriorityX()-2, thread2M, 0);
    threads[2] = chThdCreateStatic(wa[2], WA_SIZE, chThdGetPriorityX()-3, thread2L, 0);
    test_wait_threads();
    test_assert_sequence("ABC", "invalid sequence");
  }
  test_end_step(2);

  /* [8.2.3] Testing that all threads completed within the specified
     time windows (100mS...100mS+ALLOWED_DELAY).*/
  test_set_step(3);
  {
    test_assert_time_window(chTimeAddX(time, TIME_MS2I(100)),
                            chTimeAddX(time, TIME_MS2I(100) + ALLOWED_DELAY),
                            "out of time window");
  }
  test_end_step(3);
}

static const testcase_t rt_test_008_002 = {
  "Priority inheritance, simple case",
  rt_test_008_002_setup,
  NULL,
  rt_test_008_002_execute
};
#endif /* CH_DBG_THREADS_PROFILING == TRUE */

#if (CH_DBG_THREADS_PROFILING == TRUE) || defined(__DOXYGEN__)
/**
 * @page rt_test_008_003 [8.3] Priority inheritance, complex case
 *
 * <h2>Description</h2>
 * Five threads are involved in the complex priority inversion
 * scenario, the priority inheritance algorithm is tested for depths
 * greater than one. The test expects the threads to perform their
 * operations in increasing priority order by rearranging their
 * priorities in order to avoid the priority inversion trap.
 *
 * <h2>Conditions</h2>
 * This test is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_DBG_THREADS_PROFILING == TRUE
 * .
 *
 * <h2>Test Steps</h2>
 * - [8.3.1] Getting the system time for test duration measurement.
 * - [8.3.2] The five contenders threads are created and let run
 *   atomically, the goals sequence is tested, the threads must
 *   complete in priority order.
 * - [8.3.3] Testing that all threads completed within the specified
 *   time windows (110mS...110mS+ALLOWED_DELAY).
 * .
 */

static void rt_test_008_003_setup(void) {
  chMtxObjectInit(&m1); /* Mutex B.*/
  chMtxObjectInit(&m2); /* Mutex A.*/
}

static void rt_test_008_003_execute(void) {
  systime_t time;

  /* [8.3.1] Getting the system time for test duration measurement.*/
  test_set_step(1);
  {
    time = test_wait_tick();
  }
  test_end_step(1);

  /* [8.3.2] The five contenders threads are created and let run
     atomically, the goals sequence is tested, the threads must
     complete in priority order.*/
  test_set_step(2);
  {
    threads[0] = chThdCreateStatic(wa[0], WA_SIZE, chThdGetPriorityX()-5, thread3LL, 0);
    threads[1] = chThdCreateStatic(wa[1], WA_SIZE, chThdGetPriorityX()-4, thread3L, 0);
    threads[2] = chThdCreateStatic(wa[2], WA_SIZE, chThdGetPriorityX()-3, thread3M, 0);
    threads[3] = chThdCreateStatic(wa[3], WA_SIZE, chThdGetPriorityX()-2, thread3H, 0);
    threads[4] = chThdCreateStatic(wa[4], WA_SIZE, chThdGetPriorityX()-1, thread3HH, 0);
    test_wait_threads();
    test_assert_sequence("ABCDE", "invalid sequence");
  }
  test_end_step(2);

  /* [8.3.3] Testing that all threads completed within the specified
     time windows (110mS...110mS+ALLOWED_DELAY).*/
  test_set_step(3);
  {
    test_assert_time_window(chTimeAddX(time, TIME_MS2I(110)),
                            chTimeAddX(time, TIME_MS2I(110) + ALLOWED_DELAY),
                            "out of time window");
  }
  test_end_step(3);
}

static const testcase_t rt_test_008_003 = {
  "Priority inheritance, complex case",
  rt_test_008_003_setup,
  NULL,
  rt_test_008_003_execute
};
#endif /* CH_DBG_THREADS_PROFILING == TRUE */

/**
 * @page rt_test_008_004 [8.4] Priority return verification
 *
 * <h2>Description</h2>
 * Two threads are spawned that try to lock the mutexes already locked
 * by the tester thread with precise timing. The test expects that the
 * priority changes caused by the priority inheritance algorithm happen
 * at the right moment and with the right values.<br> Thread A performs
 * wait(50), lock(m1), unlock(m1), exit. Thread B performs wait(150),
 * lock(m2), unlock(m2), exit.
 *
 * <h2>Test Steps</h2>
 * - [8.4.1] Getting current thread priority P(0) and assigning to the
 *   threads A and B priorities +1 and +2.
 * - [8.4.2] Spawning threads A and B at priorities P(A) and P(B).
 * - [8.4.3] Locking the mutex M1 before thread A has a chance to lock
 *   it. The priority must not change because A has not yet reached
 *   chMtxLock(M1). the mutex is not locked.
 * - [8.4.4] Waiting 100mS, this makes thread A reach chMtxLock(M1) and
 *   get the mutex. This must boost the priority of the current thread
 *   at the same level of thread A.
 * - [8.4.5] Locking the mutex M2 before thread B has a chance to lock
 *   it. The priority must not change because B has not yet reached
 *   chMtxLock(M2). the mutex is not locked.
 * - [8.4.6] Waiting 100mS, this makes thread B reach chMtxLock(M2) and
 *   get the mutex. This must boost the priority of the current thread
 *   at the same level of thread B.
 * - [8.4.7] Unlocking M2, the priority should fall back to P(A).
 * - [8.4.8] Unlocking M1, the priority should fall back to P(0).
 * .
 */

static void rt_test_008_004_setup(void) {
  chMtxObjectInit(&m1);
  chMtxObjectInit(&m2);
}

static void rt_test_008_004_teardown(void) {
  test_wait_threads();
}

static void rt_test_008_004_execute(void) {
  tprio_t p, pa, pb;

  /* [8.4.1] Getting current thread priority P(0) and assigning to the
     threads A and B priorities +1 and +2.*/
  test_set_step(1);
  {
    p = chThdGetPriorityX();
    pa = p + 1;
    pb = p + 2;
  }
  test_end_step(1);

  /* [8.4.2] Spawning threads A and B at priorities P(A) and P(B).*/
  test_set_step(2);
  {
    threads[0] = chThdCreateStatic(wa[0], WA_SIZE, pa, thread4A, "A");
    threads[1] = chThdCreateStatic(wa[1], WA_SIZE, pb, thread4B, "B");
  }
  test_end_step(2);

  /* [8.4.3] Locking the mutex M1 before thread A has a chance to lock
     it. The priority must not change because A has not yet reached
     chMtxLock(M1). the mutex is not locked.*/
  test_set_step(3);
  {
    chMtxLock(&m1);
    test_assert(chThdGetPriorityX() == p, "wrong priority level");
  }
  test_end_step(3);

  /* [8.4.4] Waiting 100mS, this makes thread A reach chMtxLock(M1) and
     get the mutex. This must boost the priority of the current thread
     at the same level of thread A.*/
  test_set_step(4);
  {
    chThdSleepMilliseconds(100);
    test_assert(chThdGetPriorityX() == pa, "wrong priority level");
  }
  test_end_step(4);

  /* [8.4.5] Locking the mutex M2 before thread B has a chance to lock
     it. The priority must not change because B has not yet reached
     chMtxLock(M2). the mutex is not locked.*/
  test_set_step(5);
  {
    chMtxLock(&m2);
    test_assert(chThdGetPriorityX() == pa, "wrong priority level");
  }
  test_end_step(5);

  /* [8.4.6] Waiting 100mS, this makes thread B reach chMtxLock(M2) and
     get the mutex. This must boost the priority of the current thread
     at the same level of thread B.*/
  test_set_step(6);
  {
    chThdSleepMilliseconds(100);
    test_assert(chThdGetPriorityX() == pb, "wrong priority level");
  }
  test_end_step(6);

  /* [8.4.7] Unlocking M2, the priority should fall back to P(A).*/
  test_set_step(7);
  {
    chMtxUnlock(&m2);
    test_assert(chThdGetPriorityX() == pa, "wrong priority level");
  }
  test_end_step(7);

  /* [8.4.8] Unlocking M1, the priority should fall back to P(0).*/
  test_set_step(8);
  {
    chMtxUnlock(&m1);
    test_assert(chThdGetPriorityX() == p, "wrong priority level");
  }
  test_end_step(8);
}

static const testcase_t rt_test_008_004 = {
  "Priority return verification",
  rt_test_008_004_setup,
  rt_test_008_004_teardown,
  rt_test_008_004_execute
};

#if (CH_CFG_USE_MUTEXES_RECURSIVE == FALSE) || defined(__DOXYGEN__)
/**
 * @page rt_test_008_005 [8.5] Repeated locks, non recursive scenario
 *
 * <h2>Description</h2>
 * The behavior of multiple mutex locks from the same thread is tested
 * when recursion is disabled.
 *
 * <h2>Conditions</h2>
 * This test is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_MUTEXES_RECURSIVE == FALSE
 * .
 *
 * <h2>Test Steps</h2>
 * - [8.5.1] Getting current thread priority for later checks.
 * - [8.5.2] Locking the mutex first time, it must be possible because
 *   it is not owned.
 * - [8.5.3] Locking the mutex second time, it must fail because it is
 *   already owned.
 * - [8.5.4] Unlocking the mutex then it must not be owned anymore and
 *   the queue must be empty.
 * - [8.5.5] Testing that priority has not changed after operations.
 * - [8.5.6] Testing chMtxUnlockAll() behavior.
 * - [8.5.7] Testing that priority has not changed after operations.
 * .
 */

static void rt_test_008_005_setup(void) {
  chMtxObjectInit(&m1);
}

static void rt_test_008_005_execute(void) {
  bool b;
  tprio_t prio;

  /* [8.5.1] Getting current thread priority for later checks.*/
  test_set_step(1);
  {
    prio = chThdGetPriorityX();
  }
  test_end_step(1);

  /* [8.5.2] Locking the mutex first time, it must be possible because
     it is not owned.*/
  test_set_step(2);
  {
    b = chMtxTryLock(&m1);
    test_assert(b, "already locked");
  }
  test_end_step(2);

  /* [8.5.3] Locking the mutex second time, it must fail because it is
     already owned.*/
  test_set_step(3);
  {
    b = chMtxTryLock(&m1);
    test_assert(!b, "not locked");
  }
  test_end_step(3);

  /* [8.5.4] Unlocking the mutex then it must not be owned anymore and
     the queue must be empty.*/
  test_set_step(4);
  {
    chMtxUnlock(&m1);
    test_assert(m1.owner == NULL, "still owned");
    test_assert(ch_queue_isempty(&m1.queue), "queue not empty");
  }
  test_end_step(4);

  /* [8.5.5] Testing that priority has not changed after operations.*/
  test_set_step(5);
  {
    test_assert(chThdGetPriorityX() == prio, "wrong priority level");
  }
  test_end_step(5);

  /* [8.5.6] Testing chMtxUnlockAll() behavior.*/
  test_set_step(6);
  {
    b = chMtxTryLock(&m1);
    test_assert(b, "already locked");
    b = chMtxTryLock(&m1);
    test_assert(!b, "not locked");

    chMtxUnlockAll();
    test_assert(m1.owner == NULL, "still owned");
    test_assert(ch_queue_isempty(&m1.queue), "queue not empty");
  }
  test_end_step(6);

  /* [8.5.7] Testing that priority has not changed after operations.*/
  test_set_step(7);
  {
    test_assert(chThdGetPriorityX() == prio, "wrong priority level");
  }
  test_end_step(7);
}

static const testcase_t rt_test_008_005 = {
  "Repeated locks, non recursive scenario",
  rt_test_008_005_setup,
  NULL,
  rt_test_008_005_execute
};
#endif /* CH_CFG_USE_MUTEXES_RECURSIVE == FALSE */

#if (CH_CFG_USE_MUTEXES_RECURSIVE == TRUE) || defined(__DOXYGEN__)
/**
 * @page rt_test_008_006 [8.6] Repeated locks using, recursive scenario
 *
 * <h2>Description</h2>
 * The behavior of multiple mutex locks from the same thread is tested
 * when recursion is enabled.
 *
 * <h2>Conditions</h2>
 * This test is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_MUTEXES_RECURSIVE == TRUE
 * .
 *
 * <h2>Test Steps</h2>
 * - [8.6.1] Getting current thread priority for later checks.
 * - [8.6.2] Locking the mutex first time, it must be possible because
 *   it is not owned.
 * - [8.6.3] Locking the mutex second time, it must be possible because
 *   it is recursive.
 * - [8.6.4] Unlocking the mutex then it must be still owned because
 *   recursivity.
 * - [8.6.5] Unlocking the mutex then it must not be owned anymore and
 *   the queue must be empty.
 * - [8.6.6] Testing that priority has not changed after operations.
 * - [8.6.7] Testing consecutive chMtxTryLock()/chMtxTryLockS() calls
 *   and a final chMtxUnlockAllS().
 * - [8.6.8] Testing consecutive chMtxLock()/chMtxLockS() calls and a
 *   final chMtxUnlockAll().
 * - [8.6.9] Testing that priority has not changed after operations.
 * .
 */

static void rt_test_008_006_setup(void) {
  chMtxObjectInit(&m1);
}

static void rt_test_008_006_execute(void) {
  bool b;
  tprio_t prio;

  /* [8.6.1] Getting current thread priority for later checks.*/
  test_set_step(1);
  {
    prio = chThdGetPriorityX();
  }
  test_end_step(1);

  /* [8.6.2] Locking the mutex first time, it must be possible because
     it is not owned.*/
  test_set_step(2);
  {
    b = chMtxTryLock(&m1);
    test_assert(b, "already locked");
  }
  test_end_step(2);

  /* [8.6.3] Locking the mutex second time, it must be possible because
     it is recursive.*/
  test_set_step(3);
  {
    b = chMtxTryLock(&m1);
    test_assert(b, "already locked");
  }
  test_end_step(3);

  /* [8.6.4] Unlocking the mutex then it must be still owned because
     recursivity.*/
  test_set_step(4);
  {
    chMtxUnlock(&m1);
    test_assert(m1.owner != NULL, "not owned");
  }
  test_end_step(4);

  /* [8.6.5] Unlocking the mutex then it must not be owned anymore and
     the queue must be empty.*/
  test_set_step(5);
  {
    chMtxUnlock(&m1);
    test_assert(m1.owner == NULL, "still owned");
    test_assert(ch_queue_isempty(&m1.queue), "queue not empty");
  }
  test_end_step(5);

  /* [8.6.6] Testing that priority has not changed after operations.*/
  test_set_step(6);
  {
    test_assert(chThdGetPriorityX() == prio, "wrong priority level");
  }
  test_end_step(6);

  /* [8.6.7] Testing consecutive chMtxTryLock()/chMtxTryLockS() calls
     and a final chMtxUnlockAllS().*/
  test_set_step(7);
  {
    b = chMtxTryLock(&m1);
    test_assert(b, "already locked");
    chSysLock();
    b = chMtxTryLockS(&m1);
    chSysUnlock();
    test_assert(b, "already locked");
    test_assert(m1.cnt == 2, "invalid recursion counter");
    chSysLock();
    chMtxUnlockAllS();
    chSysUnlock();
    test_assert(m1.owner == NULL, "still owned");
    test_assert(ch_queue_isempty(&m1.queue), "queue not empty");
    test_assert(m1.cnt == 0, "invalid recursion counter");
  }
  test_end_step(7);

  /* [8.6.8] Testing consecutive chMtxLock()/chMtxLockS() calls and a
     final chMtxUnlockAll().*/
  test_set_step(8);
  {
    chMtxLock(&m1);
    test_assert(m1.owner != NULL, "not owned");
    chSysLock();
    chMtxLockS(&m1);
    chSysUnlock();
    test_assert(m1.owner != NULL, "not owned");
    test_assert(m1.cnt == 2, "invalid recursion counter");
    chMtxUnlockAll();
    test_assert(m1.owner == NULL, "still owned");
    test_assert(ch_queue_isempty(&m1.queue), "queue not empty");
    test_assert(m1.cnt == 0, "invalid recursion counter");
  }
  test_end_step(8);

  /* [8.6.9] Testing that priority has not changed after operations.*/
  test_set_step(9);
  {
    test_assert(chThdGetPriorityX() == prio, "wrong priority level");
  }
  test_end_step(9);
}

static const testcase_t rt_test_008_006 = {
  "Repeated locks using, recursive scenario",
  rt_test_008_006_setup,
  NULL,
  rt_test_008_006_execute
};
#endif /* CH_CFG_USE_MUTEXES_RECURSIVE == TRUE */

#if (CH_CFG_USE_CONDVARS == TRUE) || defined(__DOXYGEN__)
/**
 * @page rt_test_008_007 [8.7] Condition Variable signal test
 *
 * <h2>Description</h2>
 * Five threads take a mutex and then enter a conditional variable
 * queue, the tester thread then proceeds to signal the conditional
 * variable five times atomically.<br> The test expects the threads to
 * reach their goal in increasing priority order regardless of the
 * initial order.
 *
 * <h2>Conditions</h2>
 * This test is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_CONDVARS == TRUE
 * .
 *
 * <h2>Test Steps</h2>
 * - [8.7.1] Starting the five threads with increasing priority, the
 *   threads will queue on the condition variable.
 * - [8.7.2] Atomically signaling the condition variable five times
 *   then waiting for the threads to terminate in priority order, the
 *   order is tested.
 * .
 */

static void rt_test_008_007_setup(void) {
  chCondObjectInit(&c1);
  chMtxObjectInit(&m1);
}

static void rt_test_008_007_execute(void) {

  /* [8.7.1] Starting the five threads with increasing priority, the
     threads will queue on the condition variable.*/
  test_set_step(1);
  {
    tprio_t prio = chThdGetPriorityX();
    threads[0] = chThdCreateStatic(wa[0], WA_SIZE, prio+1, thread6, "E");
    threads[1] = chThdCreateStatic(wa[1], WA_SIZE, prio+2, thread6, "D");
    threads[2] = chThdCreateStatic(wa[2], WA_SIZE, prio+3, thread6, "C");
    threads[3] = chThdCreateStatic(wa[3], WA_SIZE, prio+4, thread6, "B");
    threads[4] = chThdCreateStatic(wa[4], WA_SIZE, prio+5, thread6, "A");
  }
  test_end_step(1);

  /* [8.7.2] Atomically signaling the condition variable five times
     then waiting for the threads to terminate in priority order, the
     order is tested.*/
  test_set_step(2);
  {
    chSysLock();
    chCondSignalI(&c1);
    chCondSignalI(&c1);
    chCondSignalI(&c1);
    chCondSignalI(&c1);
    chCondSignalI(&c1);
    chSchRescheduleS();
    chSysUnlock();
    test_wait_threads();
    test_assert_sequence("ABCDE", "invalid sequence");
  }
  test_end_step(2);
}

static const testcase_t rt_test_008_007 = {
  "Condition Variable signal test",
  rt_test_008_007_setup,
  NULL,
  rt_test_008_007_execute
};
#endif /* CH_CFG_USE_CONDVARS == TRUE */

#if (CH_CFG_USE_CONDVARS == TRUE) || defined(__DOXYGEN__)
/**
 * @page rt_test_008_008 [8.8] Condition Variable broadcast test
 *
 * <h2>Description</h2>
 * Five threads take a mutex and then enter a conditional variable
 * queue, the tester thread then proceeds to broadcast the conditional
 * variable.<br> The test expects the threads to reach their goal in
 * increasing priority order regardless of the initial order.
 *
 * <h2>Conditions</h2>
 * This test is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_CONDVARS == TRUE
 * .
 *
 * <h2>Test Steps</h2>
 * - [8.8.1] Starting the five threads with increasing priority, the
 *   threads will queue on the condition variable.
 * - [8.8.2] Broarcasting on the condition variable then waiting for
 *   the threads to terminate in priority order, the order is tested.
 * .
 */

static void rt_test_008_008_setup(void) {
  chCondObjectInit(&c1);
  chMtxObjectInit(&m1);
}

static void rt_test_008_008_execute(void) {

  /* [8.8.1] Starting the five threads with increasing priority, the
     threads will queue on the condition variable.*/
  test_set_step(1);
  {
    tprio_t prio = chThdGetPriorityX();
    threads[0] = chThdCreateStatic(wa[0], WA_SIZE, prio+1, thread6, "E");
    threads[1] = chThdCreateStatic(wa[1], WA_SIZE, prio+2, thread6, "D");
    threads[2] = chThdCreateStatic(wa[2], WA_SIZE, prio+3, thread6, "C");
    threads[3] = chThdCreateStatic(wa[3], WA_SIZE, prio+4, thread6, "B");
    threads[4] = chThdCreateStatic(wa[4], WA_SIZE, prio+5, thread6, "A");
  }
  test_end_step(1);

  /* [8.8.2] Broarcasting on the condition variable then waiting for
     the threads to terminate in priority order, the order is tested.*/
  test_set_step(2);
  {
    chCondBroadcast(&c1);
    test_wait_threads();
    test_assert_sequence("ABCDE", "invalid sequence");
  }
  test_end_step(2);
}

static const testcase_t rt_test_008_008 = {
  "Condition Variable broadcast test",
  rt_test_008_008_setup,
  NULL,
  rt_test_008_008_execute
};
#endif /* CH_CFG_USE_CONDVARS == TRUE */

#if (CH_CFG_USE_CONDVARS == TRUE) || defined(__DOXYGEN__)
/**
 * @page rt_test_008_009 [8.9] Condition Variable priority boost test
 *
 * <h2>Description</h2>
 * This test case verifies the priority boost of a thread waiting on a
 * conditional variable queue. It tests this very specific situation in
 * order to improve code coverage. The created threads perform the
 * following operations: TA{lock(M2), lock(M1), wait(C1), unlock(M1),
 * unlock(M2)}, TB{lock(M2), wait(C1), unlock(M2)}. TC{lock(M1),
 * unlock(M1)}.
 *
 * <h2>Conditions</h2>
 * This test is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_CONDVARS == TRUE
 * .
 *
 * <h2>Test Steps</h2>
 * - [8.9.1] Reading current base priority.
 * - [8.9.2] Thread A is created at priority P(+1), it locks M2, locks
 *   M1 and goes to wait on C1.
 * - [8.9.3] Thread C is created at priority P(+2), it enqueues on M1
 *   and boosts TA priority at P(+2).
 * - [8.9.4] Thread B is created at priority P(+3), it enqueues on M2
 *   and boosts TA priority at P(+3).
 * - [8.9.5] Signaling C1: TA wakes up, unlocks M1 and priority goes to
 *   P(+2). TB locks M1, unlocks M1 and completes. TA unlocks M2 and
 *   priority goes to P(+1). TC waits on C1. TA completes.
 * - [8.9.6] Signaling C1: TC wakes up, unlocks M1 and completes.
 * - [8.9.7] Checking the order of operations.
 * .
 */

static void rt_test_008_009_setup(void) {
  chCondObjectInit(&c1);
  chMtxObjectInit(&m1);
  chMtxObjectInit(&m2);
}

static void rt_test_008_009_execute(void) {
  tprio_t prio;

  /* [8.9.1] Reading current base priority.*/
  test_set_step(1);
  {
    prio = chThdGetPriorityX();
  }
  test_end_step(1);

  /* [8.9.2] Thread A is created at priority P(+1), it locks M2, locks
     M1 and goes to wait on C1.*/
  test_set_step(2);
  {
    threads[0] = chThdCreateStatic(wa[0], WA_SIZE, prio+1, thread8, "A");
  }
  test_end_step(2);

  /* [8.9.3] Thread C is created at priority P(+2), it enqueues on M1
     and boosts TA priority at P(+2).*/
  test_set_step(3);
  {
    threads[1] = chThdCreateStatic(wa[1], WA_SIZE, prio+2, thread6, "C");
  }
  test_end_step(3);

  /* [8.9.4] Thread B is created at priority P(+3), it enqueues on M2
     and boosts TA priority at P(+3).*/
  test_set_step(4);
  {
    threads[2] = chThdCreateStatic(wa[2], WA_SIZE, prio+3, thread9, "B");
  }
  test_end_step(4);

  /* [8.9.5] Signaling C1: TA wakes up, unlocks M1 and priority goes to
     P(+2). TB locks M1, unlocks M1 and completes. TA unlocks M2 and
     priority goes to P(+1). TC waits on C1. TA completes.*/
  test_set_step(5);
  {
    chCondSignal(&c1);
  }
  test_end_step(5);

  /* [8.9.6] Signaling C1: TC wakes up, unlocks M1 and completes.*/
  test_set_step(6);
  {
    chCondSignal(&c1);
  }
  test_end_step(6);

  /* [8.9.7] Checking the order of operations.*/
  test_set_step(7);
  {
    test_wait_threads();
    test_assert_sequence("ABC", "invalid sequence");
  }
  test_end_step(7);
}

static const testcase_t rt_test_008_009 = {
  "Condition Variable priority boost test",
  rt_test_008_009_setup,
  NULL,
  rt_test_008_009_execute
};
#endif /* CH_CFG_USE_CONDVARS == TRUE */

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const rt_test_sequence_008_array[] = {
  &rt_test_008_001,
#if (CH_DBG_THREADS_PROFILING == TRUE) || defined(__DOXYGEN__)
  &rt_test_008_002,
#endif
#if (CH_DBG_THREADS_PROFILING == TRUE) || defined(__DOXYGEN__)
  &rt_test_008_003,
#endif
  &rt_test_008_004,
#if (CH_CFG_USE_MUTEXES_RECURSIVE == FALSE) || defined(__DOXYGEN__)
  &rt_test_008_005,
#endif
#if (CH_CFG_USE_MUTEXES_RECURSIVE == TRUE) || defined(__DOXYGEN__)
  &rt_test_008_006,
#endif
#if (CH_CFG_USE_CONDVARS == TRUE) || defined(__DOXYGEN__)
  &rt_test_008_007,
#endif
#if (CH_CFG_USE_CONDVARS == TRUE) || defined(__DOXYGEN__)
  &rt_test_008_008,
#endif
#if (CH_CFG_USE_CONDVARS == TRUE) || defined(__DOXYGEN__)
  &rt_test_008_009,
#endif
  NULL
};

/**
 * @brief   Mutexes, Condition Variables and Priority Inheritance.
 */
const testsequence_t rt_test_sequence_008 = {
  "Mutexes, Condition Variables and Priority Inheritance",
  rt_test_sequence_008_array
};

#endif /* CH_CFG_USE_MUTEXES == TRUE */
