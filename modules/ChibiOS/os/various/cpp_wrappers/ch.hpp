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
 * @file    ch.hpp
 * @brief   C++ wrapper classes and definitions.
 *
 * @addtogroup cpp_library
 * @{
 */

#include <ch.h>

#ifndef _CH_HPP_
#define _CH_HPP_

/**
 * @brief   ChibiOS-RT kernel-related classes and interfaces.
 */
namespace chibios_rt {

  /* Forward declaration of some classes.*/
  class ThreadReference;

  /*------------------------------------------------------------------------*
   * chibios_rt::System                                                     *
   *------------------------------------------------------------------------*/
  /**
   * @brief Class encapsulating the base system functionalities.
   */
  class System {
  public:
    /**
     * @brief   ChibiOS/RT initialization.
     * @details After executing this function the current instructions stream
     *          becomes the main thread.
     * @pre     Interrupts must be still disabled when @p chSysInit() is invoked
     *          and are internally enabled.
     * @post    The main thread is created with priority @p NORMALPRIO.
     * @note    This function has special, architecture-dependent, requirements,
     *          see the notes into the various port reference manuals.
     *
     * @special
     */
    static void init(void) {

      chSysInit();
    }

    /**
     * @brief   Halts the system.
     * @details This function is invoked by the operating system when an
     *          unrecoverable error is detected, for example because a programming
     *          error in the application code that triggers an assertion while
     *          in debug mode.
     * @note    Can be invoked from any system state.
     *
     * @param[in] reason        pointer to an error string
     *
     * @special
     */
    static void halt(const char *reason) {

      chSysHalt(reason);
    }

    /**
     * @brief   System integrity check.
     * @details Performs an integrity check of the important ChibiOS/RT data
     *          structures.
     * @note    The appropriate action in case of failure is to halt the system
     *          before releasing the critical zone.
     * @note    If the system is corrupted then one possible outcome of this
     *          function is an exception caused by @p nullptr or corrupted
     *          pointers in list elements. Exception vectors must be monitored
     *          as well.
     * @note    This function is not used internally, it is up to the
     *          application to define if and where to perform system
     *          checking.
     * @note    Performing all tests at once can be a slow operation and can
     *          degrade the system response time. It is suggested to execute
     *          one test at time and release the critical zone in between tests.
     *
     * @param[in] testmask  Each bit in this mask is associated to a test to be
     *                      performed.
     * @return              The test result.
     * @retval false        The test succeeded.
     * @retval true         Test failed.
     *
     * @iclass
     */
    static bool integrityCheckI(unsigned int testmask) {

      return chSysIntegrityCheckI(testmask);
    }

    /**
     * @brief   Raises the system interrupt priority mask to the maximum level.
     * @details All the maskable interrupt sources are disabled regardless their
     *          hardware priority.
     * @note    Do not invoke this API from within a kernel lock.
     *
     * @special
     */
    static void disable(void) {

      chSysDisable();
    }

    /**
     * @brief   Raises the system interrupt priority mask to system level.
     * @details The interrupt sources that should not be able to preempt the kernel
     *          are disabled, interrupt sources with higher priority are still
     *          enabled.
     * @note    Do not invoke this API from within a kernel lock.
     * @note    This API is no replacement for @p chSysLock(), the @p chSysLock()
     *          could do more than just disable the interrupts.
     *
     * @special
     */
    static void suspend(void) {

      chSysSuspend();
    }

    /**
     * @brief   Lowers the system interrupt priority mask to user level.
     * @details All the interrupt sources are enabled.
     * @note    Do not invoke this API from within a kernel lock.
     * @note    This API is no replacement for @p chSysUnlock(), the
     *          @p chSysUnlock() could do more than just enable the interrupts.
     *
     * @special
     */
    static void enable(void) {

      chSysEnable();
    }

    /**
     * @brief   Enters the kernel lock mode.
     *
     * @special
     */
    static void lock(void) {

      chSysLock();
    }

    /**
     * @brief   Leaves the kernel lock mode.
     *
     * @special
     */
    static void unlock(void) {

      chSysUnlock();
    }

    /**
     * @brief   Enters the kernel lock mode from within an interrupt handler.
     * @note    This API may do nothing on some architectures, it is required
     *          because on ports that support preemptable interrupt handlers
     *          it is required to raise the interrupt mask to the same level of
     *          the system mutual exclusion zone.<br>
     *          It is good practice to invoke this API before invoking any I-class
     *          syscall from an interrupt handler.
     * @note    This API must be invoked exclusively from interrupt handlers.
     *
     * @special
     */
    static void lockFromIsr(void) {

      chSysLockFromISR();
    }

    /**
     * @brief   Leaves the kernel lock mode from within an interrupt handler.
     *
     * @note    This API may do nothing on some architectures, it is required
     *          because on ports that support preemptable interrupt handlers
     *          it is required to raise the interrupt mask to the same level of
     *          the system mutual exclusion zone.<br>
     *          It is good practice to invoke this API after invoking any I-class
     *          syscall from an interrupt handler.
     * @note    This API must be invoked exclusively from interrupt handlers.
     *
     * @special
     */
    static void unlockFromIsr(void) {

      chSysUnlockFromISR();
    }

    /**
     * @brief   Unconditionally enters the kernel lock state.
     * @note    Can be called without previous knowledge of the current lock state.
     *          The final state is "s-locked".
     *
     * @special
     */
    static void unconditionalLock(void) {

      chSysUnconditionalLock();
    }

    /**
     * @brief   Unconditionally leaves the kernel lock state.
     * @note    Can be called without previous knowledge of the current lock state.
     *          The final state is "normal".
     *
     * @special
     */
    static void unconditionalUnlock(void) {

      chSysUnconditionalUnlock();
    }

    /**
     * @brief   Returns the execution status and enters a critical zone.
     * @details This functions enters into a critical zone and can be called
     *          from any context. Because its flexibility it is less efficient
     *          than @p chSysLock() which is preferable when the calling context
     *          is known.
     * @post    The system is in a critical zone.
     *
     * @return              The previous system status, the encoding of this
     *                      status word is architecture-dependent and opaque.
     *
     * @xclass
     */
    static syssts_t getStatusAndLockX(void) {

      return chSysGetStatusAndLockX();
    }

    /**
     * @brief   Restores the specified execution status and leaves a critical zone.
     * @note    A call to @p chSchRescheduleS() is automatically performed
     *          if exiting the critical zone and if not in ISR context.
     *
     * @param[in] sts       the system status to be restored.
     *
     * @xclass
     */
    static void restoreStatusX(syssts_t sts) {

      chSysRestoreStatusX(sts);
    }

#if (PORT_SUPPORTS_RT == TRUE) || defined(__DOXYGEN__)
    /**
     * @brief   Returns the current value of the system real time counter.
     * @note    This function is only available if the port layer supports the
     *          option @p PORT_SUPPORTS_RT.
     *
     * @return              The value of the system realtime counter of
     *                      type rtcnt_t.
     *
     * @xclass
     */
    static rtcnt_t getRealtimeCounterX(void) {

      return chSysGetRealtimeCounterX();
    }

    /**
     * @brief   Realtime window test.
     * @details This function verifies if the current realtime counter value
     *          lies within the specified range or not. The test takes care
     *          of the realtime counter wrapping to zero on overflow.
     * @note    When start==end then the function returns always true because the
     *          whole time range is specified.
     * @note    This function is only available if the port layer supports the
     *          option @p PORT_SUPPORTS_RT.
     *
     * @param[in] cnt       the counter value to be tested
     * @param[in] start     the start of the time window (inclusive)
     * @param[in] end       the end of the time window (non inclusive)
     * @retval true         current time within the specified time window.
     * @retval false        current time not within the specified time window.
     *
     * @xclass
     */
    static bool isCounterWithinX(rtcnt_t cnt,
                                       rtcnt_t start,
                                       rtcnt_t end) {

      return chSysIsCounterWithinX(cnt, start, end);
    }

   /**
    * @brief   Polled delay.
    * @note    The real delay is always few cycles in excess of the specified
    *          value.
    * @note    This function is only available if the port layer supports the
    *          option @p PORT_SUPPORTS_RT.
    *
    * @param[in] cycles    number of cycles
    *
    * @xclass
    */
    static void polledDelayX(rtcnt_t cycles) {

      chSysPolledDelayX(cycles);
    }
#endif /* PORT_SUPPORTS_RT == TRUE */

    /**
     * @brief   Returns the system time as system ticks.
     * @note    The system tick time interval is implementation dependent.
     *
     * @return          The system time.
     *
     * @api
     */
    static systime_t getTime(void) {

      return chVTGetSystemTime();
    }

    /**
     * @brief   Returns the system time as system ticks.
     * @note    The system tick time interval is implementation dependent.
     *
     * @return          The system time.
     *
     * @xclass
     */
    static systime_t getTimeX(void) {

      return chVTGetSystemTimeX();
    }

    /**
     * @brief   Checks if the current system time is within the specified time
     *          window.
     * @note    When start==end then the function returns always true because the
     *          whole time range is specified.
     *
     * @param[in] start     the start of the time window (inclusive)
     * @param[in] end       the end of the time window (non inclusive)
     * @retval true         current time within the specified time window.
     * @retval false        current time not within the specified time window.
     *
     * @api
     */
    static bool isSystemTimeWithin(systime_t start, systime_t end) {

      return chVTIsSystemTimeWithin(start, end);
    }

#if (CH_CFG_NO_IDLE_THREAD == FALSE) || defined(__DOXYGEN__)
    /**
     * @brief   Returns a reference to the idle thread.
     * @pre     In order to use this function the option @p CH_CFG_NO_IDLE_THREAD
     *          must be disabled.
     * @note    The reference counter of the idle thread is not incremented but
     *          it is not strictly required being the idle thread a static
     *          object.
     *
     * @return              Reference to the idle thread.
     *
     * @xclass
     */
    static ThreadReference getIdleThreadX(void);
#endif /* CH_CFG_NO_IDLE_THREAD == FALSE */
  };

  /*------------------------------------------------------------------------*
   * chibios_rt::CriticalSectionLocker                                      *
   *------------------------------------------------------------------------*/
  /**
   * @brief   RAII helper for reentrant critical sections.
   */
  class CriticalSectionLocker {
    volatile const syssts_t syssts = chSysGetStatusAndLockX();

  public:
    ~CriticalSectionLocker() {

      chSysRestoreStatusX(syssts);
    }
  };

  /*------------------------------------------------------------------------*
   * chibios_rt::Scheduler                                                  *
   *------------------------------------------------------------------------*/
  /**
   * @brief Class encapsulating the low level scheduler functionalities.
   */
  class Scheduler {
  public:
    /**
     * @brief   Performs a reschedule if a higher priority thread is runnable.
     * @details If a thread with a higher priority than the current thread is in
     *          the ready list then make the higher priority thread running.
     *
     * @sclass
     */
    static void rescheduleS(void) {

      void chSchRescheduleS();
    }
  };

#if (CH_CFG_USE_MEMCORE == TRUE) || defined(__DOXYGEN__)
  /*------------------------------------------------------------------------*
   * chibios_rt::Core                                                       *
   *------------------------------------------------------------------------*/
  /**
   * @brief Class encapsulating the base system functionalities.
   */
  class Core {
  public:
    /**
     * @brief   Allocates a memory block.
     * @details The size of the returned block is aligned to the alignment
     *          type so it is not possible to allocate less
     *          than <code>MEM_ALIGN_SIZE</code>.
     *
     * @param[in] size      the size of the block to be allocated
     * @return              A pointer to the allocated memory block.
     * @retval nullptr      allocation failed, core memory exhausted.
     *
     * @api
     */
    static void *alloc(size_t size) {

      return chCoreAlloc(size);
    }

    /**
     * @brief   Allocates a memory block.
     * @details The size of the returned block is aligned to the alignment
     *          type so it is not possible to allocate less than
     *          <code>MEM_ALIGN_SIZE</code>.
     *
     * @param[in] size      the size of the block to be allocated.
     * @return              A pointer to the allocated memory block.
     * @retval nullptr      allocation failed, core memory exhausted.
     *
     * @iclass
     */
    static void *allocI(size_t size) {

      return chCoreAllocI(size);
    }

    /**
     * @brief   Core memory status.
     *
     * @return              The size, in bytes, of the free core memory.
     *
     * @xclass
     */
    static size_t getStatusX(void) {

      return chCoreGetStatusX();
    }
  };
#endif /* CH_CFG_USE_MEMCORE == TRUE */

  /*------------------------------------------------------------------------*
   * chibios_rt::Timer                                                      *
   *------------------------------------------------------------------------*/
  /**
   * @brief   Timer class.
   */
  class Timer {
    /**
     * @brief   Embedded @p virtual_timer_t structure.
     */
    virtual_timer_t vt;

  public:
    /**
     * @brief  Construct a virtual timer.
     */
    Timer() : vt() {

      chVTObjectInit(&vt);
    }

    /* Prohibit copy construction and assignment.*/
    Timer(const Timer &) = delete;
    Timer &operator=(const Timer &) = delete;

    /**
     * @brief   Enables a virtual timer.
     * @note    The associated function is invoked from interrupt context.
     *
     * @param[in] timeout   the number of ticks before the operation timeouts,
     *                      the special values are handled as follow:
     *                      - @a TIME_INFINITE is allowed but interpreted as a
     *                        normal time specification.
     *                      - @a TIME_IMMEDIATE this value is not allowed.
     *                      .
     * @param[in] vtfunc    the timer callback function. After invoking the
     *                      callback the timer is disabled and the structure
     *                      can be disposed or reused.
     * @param[in] par       a parameter that will be passed to the callback
     *                      function
     *
     * @api
     */
    void set(sysinterval_t timeout, vtfunc_t vtfunc, void *par) {

      chVTSet(&vt, timeout, vtfunc, par);
    }

    /**
     * @brief   Enables a virtual timer.
     * @note    The associated function is invoked from interrupt context.
     *
     * @param[in] timeout   the number of ticks before the operation timeouts,
     *                      the special values are handled as follow:
     *                      - @a TIME_INFINITE is allowed but interpreted as a
     *                        normal time specification.
     *                      - @a TIME_IMMEDIATE this value is not allowed.
     *                      .
     * @param[in] vtfunc    the timer callback function. After invoking the
     *                      callback the timer is disabled and the structure
     *                      can be disposed or reused.
     * @param[in] par       a parameter that will be passed to the callback
     *                      function
     *
     * @iclass
     */
    void setI(sysinterval_t timeout, vtfunc_t vtfunc, void *par) {

      chVTSetI(&vt, timeout, vtfunc, par);
    }

    /**
     * @brief   Resets the timer, if armed.
     *
     * @api
     */
    void reset() {

      chVTReset(&vt);
    }

    /**
     * @brief   Resets the timer, if armed.
     *
     * @iclass
     */
    void resetI() {

      chVTResetI(&vt);
    }

    /**
     * @brief   Returns the timer status.
     *
     * @return              The timer status.
     * @retval true         If the timer is armed.
     * @retval false        If the timer already fired its callback.
     *
     * @iclass
     */
    bool isArmedI(void) const {

      return chVTIsArmedI(&vt);
    }
  };

  /*------------------------------------------------------------------------*
   * chibios_rt::ThreadReference                                            *
   *------------------------------------------------------------------------*/
  /**
   * @brief     Thread reference class.
   * @details   This class encapsulates a reference to a system thread. All
   *            operations involving another thread are performed through
   *            an object of this type.
   */
  class ThreadReference final {
    /**
     * @brief   Pointer to the system thread.
     */
    thread_t *thread_ref;

  public:
    /**
     * @brief   Thread reference constructor.
     * @note    Do not call this version directly, this constructor is empty
     *          and is here only to do nothing when an object of this kind
     *          is declared then assigned.
     * @note    Automatic instances of this object are not initialized
     *          because this constructor, this is intentional.
     *
     * @param[in] tp            the target thread
     *
     * @init
     */
    ThreadReference(void) {

    }

    /**
     * @brief   Thread reference constructor.
     *
     * @param[in] tp            the target thread
     *
     * @init
     */
    ThreadReference(thread_t *tp) : thread_ref(tp) {

    }

    /**
     * @brief   Returns the reference state.
     *
     * @return          The reference state.
     * @retval false    if the reference is still valid.
     * @retval true     if the reference is set to @p nullptr.
     */
    bool isNull(void) const {

      return (bool)(thread_ref == nullptr);
    }

    /**
     * @brief   Returns the low level pointer to the referenced thread.
     */
    thread_t *getInner(void) {

      return thread_ref;
    }

    /**
     * @brief   Requests a thread termination.
     * @pre     The target thread must be written to invoke periodically
     *          @p chThdShouldTerminate() and terminate cleanly if it returns
     *          @p TRUE.
     * @post    The specified thread will terminate after detecting the
     *          termination condition.
     *
     * @api
     */
    void requestTerminate(void) const {

      chThdTerminate(thread_ref);
    }

#if (CH_CFG_USE_REGISTRY == TRUE) || defined(__DOXYGEN__)
    /**
     * @brief   Adds a reference to a thread object.
     * @pre     The configuration option @p CH_CFG_USE_REGISTRY must be enabled
     *          in order to use this function.
     *
     * @return              A new thread reference.
     *
     * @api
     */
    ThreadReference addRef(void) const {

      return ThreadReference(chThdAddRef(thread_ref));
    }

    /**
     * @brief   Releases a reference to a thread object.
     * @details If the references counter reaches zero <b>and</b> the thread
     *          is in the @p CH_STATE_FINAL state then the thread's memory is
     *          returned to the proper allocator and the thread is removed
     *          from the registry.<br>
     *          Threads whose counter reaches zero and are still active become
     *          "detached". Detached static threads will be removed from the
     *          registry on termination. Detached non-static threads can only be
     *          removed by performing a registry scan operation.
     * @pre     The configuration option @p CH_CFG_USE_REGISTRY must be enabled in
     *          order to use this function.
     * @post    The reference is set to @p nullptr.
     * @note    Static threads are not affected, only removed from the registry.
     *
     * @api
     */
    void release(void) {
      thread_t *tp = thread_ref;
      thread_ref = nullptr;

      chThdRelease(tp);
    }
#endif /* CH_CFG_USE_REGISTRY == TRUE */

#if (CH_CFG_USE_WAITEXIT == TRUE) || defined(__DOXYGEN__)
    /**
     * @brief   Blocks the execution of the invoking thread until the specified
     *          thread terminates then the exit code is returned.
     * @details This function waits for the specified thread to terminate then
     *          decrements its reference counter, if the counter reaches zero
     *          then the thread working area is returned to the proper
     *          allocator.<br>
     *          The memory used by the exited thread is handled in different
     *          ways depending on the API that spawned the thread:
     *          - If the thread was spawned by @p chThdCreateStatic() or by
     *            @p chThdCreateI() then nothing happens and the thread working
     *            area is not released or modified in any way. This is the
     *            default, totally static, behavior.
     *          - If the thread was spawned by @p chThdCreateFromHeap() then
     *            the working area is returned to the system heap.
     *          - If the thread was spawned by @p chThdCreateFromMemoryPool()
     *            then the working area is returned to the owning memory pool.
     *          .
     * @pre     The configuration option @p CH_USE_WAITEXIT must be enabled in
     *          order to use this function.
     * @post    Enabling @p chThdWait() requires 2-4 (depending on the
     *          architecture) extra bytes in the @p Thread structure.
     * @post    The reference is set to @p nullptr.
     * @note    If @p CH_USE_DYNAMIC is not specified this function just waits
     *          for the thread termination, no memory allocators are involved.
     *
     * @return              The exit code from the terminated thread.
     *
     * @api
     */
    msg_t wait(void) {
      thread_t *tp = thread_ref;
      thread_ref = nullptr;

      msg_t msg = chThdWait(tp);
      return msg;
    }
#endif /* CH_CFG_USE_WAITEXIT == TRUE */

#if (CH_CFG_USE_MESSAGES == TRUE) || defined(__DOXYGEN__)
    /**
     * @brief   Sends a message to the thread and returns the answer.
     *
     * @param[in] msg           the sent message
     * @return                  The returned message.
     *
     * @api
     */
    msg_t sendMessage(msg_t msg) const {

      return chMsgSend(thread_ref, msg);
    }

    /**
     * @brief   Returns true if there is at least one message in queue.
     *
     * @retval true             A message is waiting in queue.
     * @retval false            A message is not waiting in queue.
     *
     * @api
     */
    bool isPendingMessage(void) const {

      return chMsgIsPendingI(thread_ref);
    }

    /**
     * @brief   Returns an enqueued message or @p nullptr.
     *
     * @return                  The incoming message.
     *
     * @api
     */
    msg_t getMessage(void) const {

      return chMsgGet(thread_ref);
    }

    /**
     * @brief   Releases the next message in queue with a reply.
     * @post    The reference is set to @p nullptr.
     *
     * @param[in] msg           the answer message
     *
     * @api
     */
    void releaseMessage(msg_t msg) {
      thread_t *tp = thread_ref;
      thread_ref = nullptr;

      chMsgRelease(tp, msg);
    }
#endif /* CH_CFG_USE_MESSAGES == TRUE */

#if (CH_CFG_USE_EVENTS == TRUE) || defined(__DOXYGEN__)
    /**
     * @brief   Adds a set of event flags directly to specified @p Thread.
     *
     * @param[in] mask      the event flags set to be ORed
     *
     * @api
     */
    void signalEvents(eventmask_t mask) const {

      chEvtSignal(thread_ref, mask);
    }

    /**
     * @brief   Adds a set of event flags directly to specified @p Thread.
     *
     * @param[in] mask      the event flags set to be ORed
     *
     * @iclass
     */
    void signalEventsI(eventmask_t mask) const {

      chEvtSignalI(thread_ref, mask);
    }
#endif /* CH_CFG_USE_EVENTS == TRUE */

#if (CH_DBG_THREADS_PROFILING == TRUE) || defined(__DOXYGEN__)
    /**
     * @brief   Returns the number of ticks consumed by the specified thread.
     * @note    This function is only available when the
     *          @p CH_DBG_THREADS_PROFILING configuration option is enabled.
     *
     * @param[in] tp        pointer to the thread
     * @return              The number of consumed system ticks.
     *
     * @xclass
     */
    systime_t getTicksX(void) const {

      return chThdGetTicksX(thread_ref);
    }
#endif /* CH_DBG_THREADS_PROFILING == TRUE */
  };

#if (CH_CFG_USE_REGISTRY == TRUE) || defined(__DOXYGEN__)
  /*------------------------------------------------------------------------*
   * chibios_rt::Registry                                                   *
   *------------------------------------------------------------------------*/
  class Registry {
  public:
    /**
     * @brief   Returns the first thread in the system.
     * @details Returns the most ancient thread in the system, usually this is
     *          the main thread unless it terminated. A reference is added to the
     *          returned thread in order to make sure its status is not lost.
     * @note    This function cannot return @p nullptr because there is always at
     *          least one thread in the system.
     *
     * @return              A reference to the most ancient thread.
     *
     * @api
     */
    static ThreadReference firstThread(void) {

      return ThreadReference(chRegFirstThread());
    }

    /**
     * @brief   Returns the thread next to the specified one.
     * @details The reference counter of the specified thread is decremented and
     *          the reference counter of the returned thread is incremented.
     *
     * @param[in] tref      reference to the thread
     * @return              A reference to the next thread. The reference is
     *                      set to @p nullptr if there is no next thread.
     *
     * @api
     */
    static ThreadReference nextThread(ThreadReference tref) {

      return ThreadReference(chRegNextThread(tref.getInner()));
    }

    /**
     * @brief   Retrieves a thread reference by name.
     * @note    The reference counter of the found thread is increased by one so
     *          it cannot be disposed incidentally after the pointer has been
     *          returned.
     *
     * @param[in] name      the thread name
     * @return              A pointer to the found thread.
     * @return              A reference to the found thread. The reference is
     *                      set to @p nullptr if no next thread is found.
     *
     * @api
     */
    static ThreadReference findThreadByName(const char *name) {

      return ThreadReference(chRegFindThreadByName(name));
    }
  };
#endif /* CH_CFG_USE_REGISTRY == TRUE */

  /*------------------------------------------------------------------------*
   * chibios_rt::BaseThread                                                 *
   *------------------------------------------------------------------------*/
  /**
   * @brief   Abstract base class for a ChibiOS/RT thread.
   * @details The thread body is the virtual function @p Main().
   */
  class BaseThread {
  public:
    /**
     * @brief   BaseThread constructor.
     *
     * @init
     */
    BaseThread(void) {

    }

    /**
     * @brief   Thread body function.
     *
     * @return                  The exit message.
     *
     * @api
     */
    virtual void main(void) = 0;

    /**
     * @brief   Creates and starts a system thread.
     *
     * @param[in] prio          thread priority
     * @return                  A reference to the created thread with
     *                          reference counter set to one.
     *
     * @api
     */
    virtual ThreadReference start(tprio_t prio) = 0;

    /**
     * @brief   Returns a reference to the current thread.
     *
     * @return             A reference to the current thread.
     *
     * @xclass
     */
    static ThreadReference getSelfX(void) {

      return ThreadReference(chThdGetSelfX());
    }

    /**
     * @brief   Sets the current thread name.
     * @pre     This function only stores the pointer to the name if the option
     *          @p CH_USE_REGISTRY is enabled else no action is performed.
     *
     * @param[in] tname         thread name as a zero terminated string
     *
     * @api
     */
    static void setName(const char *tname) {

      chRegSetThreadName(tname);
    }

    /**
     * @brief   Changes the running thread priority level then reschedules if
     *          necessary.
     * @note    The function returns the real thread priority regardless of the
     *          current priority that could be higher than the real priority
     *          because the priority inheritance mechanism.
     *
     * @param[in] newprio   the new priority level of the running thread
     * @return              The old priority level.
     *
     * @api
     */
    static tprio_t setPriority(tprio_t newprio) {

      return chThdSetPriority(newprio);
    }

    /**
     * @brief   Returns the current thread priority.
     * @note    Can be invoked in any context.
     *
     * @return              The current thread priority.
     *
     * @xclass
     */
    static tprio_t getPriorityX(void) {

      return chThdGetPriorityX();
    }

    /**
     * @brief   Terminates the current thread.
     * @details The thread goes in the @p THD_STATE_FINAL state holding the
     *          specified exit status code, other threads can retrieve the
     *          exit status code by invoking the function @p chThdWait().
     * @post    Eventual code after this function will never be executed,
     *          this function never returns. The compiler has no way to
     *          know this so do not assume that the compiler would remove
     *          the dead code.
     *
     * @param[in] msg       thread exit code
     *
     * @api
     */
    static void exit(msg_t msg) {

      chThdExit(msg);
    }

    /**
     * @brief   Terminates the current thread.
     * @details The thread goes in the @p THD_STATE_FINAL state holding the
     *          specified exit status code, other threads can retrieve the
     *          exit status code by invoking the function @p chThdWait().
     * @post    Eventual code after this function will never be executed,
     *          this function never returns. The compiler has no way to
     *          know this so do not assume that the compiler would remove
     *          the dead code.
     *
     * @param[in] msg       thread exit code
     *
     * @sclass
     */
    static void exitS(msg_t msg) {

      chThdExitS(msg);
    }

    /**
     * @brief   Verifies if the current thread has a termination request
     *          pending.
     * @note    Can be invoked in any context.
     *
     * @retval TRUE         termination request pending.
     * @retval FALSE        termination request not pending.
     *
     * @special
     */
    static bool shouldTerminate(void) {

      return chThdShouldTerminateX();
    }

    /**
     * @brief   Suspends the invoking thread for the specified time.
     *
     * @param[in] interval  the delay in system ticks, the special values are
     *                      handled as follow:
     *                      - @a TIME_INFINITE the thread enters an infinite
     *                        sleep state.
     *                      - @a TIME_IMMEDIATE this value is not allowed.
     *                      .
     *
     * @api
     */
    static void sleep(sysinterval_t interval) {

      chThdSleep(interval);
    }

    /**
     * @brief   Suspends the invoking thread until the system time arrives to
     *          the specified value.
     *
     * @param[in] time      absolute system time
     *
     * @api
     */
    static void sleepUntil(systime_t time) {

      chThdSleepUntil(time);
    }

    /**
     * @brief   Suspends the invoking thread until the system time arrives to the
     *          specified value.
     * @note    The system time is assumed to be between @p prev and @p time
     *          else the call is assumed to have been called outside the
     *          allowed time interval, in this case no sleep is performed.
     * @see     chThdSleepUntil()
     *
     * @param[in] prev      absolute system time of the previous deadline
     * @param[in] next      absolute system time of the next deadline
     * @return              the @p next parameter
     *
     * @api
     */
    static systime_t sleepUntilWindowed(systime_t prev, systime_t next) {

      return chThdSleepUntilWindowed(prev, next);
    }

    /**
     * @brief   Yields the time slot.
     * @details Yields the CPU control to the next thread in the ready list
     *          with equal priority, if any.
     *
     * @api
     */
    static void yield(void) {

      chThdYield();
    }

#if CH_CFG_USE_MESSAGES || defined(__DOXYGEN__)
    /**
     * @brief   Waits for a message.
     * @post    On the returned reference it is mandatory to call
     *          @p releaseMessage() or the sender thread would be waiting
     *          undefinitely.
     *
     * @return                  The sender thread reference.
     *
     * @api
     */
    static ThreadReference waitMessage(void) {

      return ThreadReference(chMsgWait());
    }
#endif /* CH_CFG_USE_MESSAGES == TRUE */

#if CH_CFG_USE_EVENTS || defined(__DOXYGEN__)
    /**
     * @brief   Clears the pending events specified in the mask.
     *
     * @param[in] mask      the events to be cleared
     * @return              The pending events that were cleared.
     *
     * @api
     */
    static eventmask_t getAndClearEvents(eventmask_t mask) {

      return chEvtGetAndClearEvents(mask);
    }

    /**
     * @brief   Adds (OR) a set of event flags on the current thread, this is
     *          @b much faster than using @p chEvtBroadcast() or
     *          @p chEvtSignal().
     *
     * @param[in] mask      the event flags to be added
     * @return              The current pending events mask.
     *
     * @api
     */
    static eventmask_t addEvents(eventmask_t mask) {

      return chEvtAddEvents(mask);
    }

    /**
     * @brief   Waits for a single event.
     * @details A pending event among those specified in @p ewmask is selected,
     *          cleared and its mask returned.
     * @note    One and only one event is served in the function, the one with
     *          the lowest event id. The function is meant to be invoked into
     *          a loop in order to serve all the pending events.<br>
     *          This means that Event Listeners with a lower event identifier
     *          have an higher priority.
     *
     * @param[in] ewmask        mask of the events that the function should
     *                          wait for, @p ALL_EVENTS enables all the events
     * @return                  The mask of the lowest id served and cleared
     *                          event.
     *
     * @api
     */
    static eventmask_t waitOneEvent(eventmask_t ewmask) {

      return chEvtWaitOne(ewmask);
    }

    /**
     * @brief   Waits for any of the specified events.
     * @details The function waits for any event among those specified in
     *          @p ewmask to become pending then the events are cleared and
     *          returned.
     *
     * @param[in] ewmask        mask of the events that the function should
     *                          wait for, @p ALL_EVENTS enables all the events
     * @return                  The mask of the served and cleared events.
     *
     * @api
     */
    static eventmask_t waitAnyEvent(eventmask_t ewmask) {

      return chEvtWaitAny(ewmask);
    }

    /**
     * @brief   Waits for all the specified event flags then clears them.
     * @details The function waits for all the events specified in @p ewmask
     *          to become pending then the events are cleared and returned.
     *
     * @param[in] ewmask        mask of the event ids that the function should
     *                          wait for
     * @return                  The mask of the served and cleared events.
     *
     * @api
     */
    static eventmask_t waitAllEvents(eventmask_t ewmask) {

      return chEvtWaitAll(ewmask);
    }

#if CH_CFG_USE_EVENTS_TIMEOUT || defined(__DOXYGEN__)
    /**
     * @brief   Waits for a single event.
     * @details A pending event among those specified in @p ewmask is selected,
     *          cleared and its mask returned.
     * @note    One and only one event is served in the function, the one with
     *          the lowest event id. The function is meant to be invoked into
     *          a loop in order to serve all the pending events.<br>
     *          This means that Event Listeners with a lower event identifier
     *          have an higher priority.
     *
     * @param[in] ewmask        mask of the events that the function should
     *                          wait for, @p ALL_EVENTS enables all the events
     *
     * @param[in] timeout       the number of ticks before the operation
     *                          timouts
     * @return                  The mask of the lowest id served and cleared
     *                          event.
     * @retval 0                if the specified timeout expired.
     *
     * @api
     */
    static eventmask_t waitOneEventTimeout(eventmask_t ewmask,
                                           sysinterval_t timeout) {

      return chEvtWaitOneTimeout(ewmask, timeout);
    }

    /**
     * @brief   Waits for any of the specified events.
     * @details The function waits for any event among those specified in
     *          @p ewmask to become pending then the events are cleared and
     *          returned.
     *
     * @param[in] ewmask        mask of the events that the function should
     *                          wait for, @p ALL_EVENTS enables all the events
     * @param[in] timeout       the number of ticks before the operation
     *                          timouts
     * @return                  The mask of the served and cleared events.
     * @retval 0                if the specified timeout expired.
     *
     * @api
     */
    static eventmask_t waitAnyEventTimeout(eventmask_t ewmask,
                                           sysinterval_t timeout) {

      return chEvtWaitAnyTimeout(ewmask, timeout);
    }

    /**
     * @brief   Waits for all the specified event flags then clears them.
     * @details The function waits for all the events specified in @p ewmask
     *          to become pending then the events are cleared and returned.
     *
     * @param[in] ewmask        mask of the event ids that the function should
     *                          wait for
     * @param[in] timeout       the number of ticks before the operation
     *                          timouts
     * @return                  The mask of the served and cleared events.
     * @retval 0                if the specified timeout expired.
     *
     * @api
     */
    static eventmask_t waitAllEventsTimeout(eventmask_t ewmask,
                                            sysinterval_t timeout) {

      return chEvtWaitAllTimeout(ewmask, timeout);
    }
#endif /* CH_CFG_USE_EVENTS_TIMEOUT == TRUE */

    /**
     * @brief   Invokes the event handlers associated to an event flags mask.
     *
     * @param[in] mask      mask of the event flags to be dispatched
     * @param[in] handlers  an array of @p evhandler_t. The array must have
     *                      size equal to the number of bits in eventmask_t.
     *
     * @api
     */
    static void dispatchEvents(const evhandler_t handlers[],
                               eventmask_t mask) {

      chEvtDispatch(handlers, mask);
    }
#endif /* CH_CFG_USE_EVENTS == TRUE */

#if CH_CFG_USE_MUTEXES || defined(__DOXYGEN__)
    /**
     * @brief   Unlocks all the mutexes owned by the invoking thread.
     * @post    The stack of owned mutexes is emptied and all the found
     *          mutexes are unlocked.
     * @note    This function is <b>MUCH MORE</b> efficient than releasing the
     *          mutexes one by one and not just because the call overhead,
     *          this function does not have any overhead related to the
     *          priority inheritance mechanism.
     *
     * @api
     */
    static void unlockAllMutexes(void) {

      chMtxUnlockAll();
    }
#endif /* CH_CFG_USE_MUTEXES == TRUE */
  };

  /*------------------------------------------------------------------------*
   * chibios_rt::BaseStaticThread                                           *
   *------------------------------------------------------------------------*/
  /**
   * @brief   Static threads template base class.
   * @details This class introduces static working area instantiation.
   *
   * @param N               the working area size for the thread class
   */
  template <int N>
  class BaseStaticThread : public BaseThread {
  protected:
    THD_WORKING_AREA(wa, N);

  public:
    /**
     * @brief   Starts a static thread.
     *
     * @param[in] prio          thread priority
     * @return                  A reference to the created thread with
     *                          reference counter set to one.
     *
     * @api
     */
    ThreadReference start(tprio_t prio) override {
      void _thd_start(void *arg);

      return ThreadReference(chThdCreateStatic(wa, sizeof(wa), prio,
                                               _thd_start, this));
    }
  };

  /*------------------------------------------------------------------------*
   * chibios_rt::BaseDynamicThread                                          *
   *------------------------------------------------------------------------*/
  /**
   * @brief   Dynamic threads base class.
   */
  class BaseDynamicThread : public BaseThread {
  };

  /*------------------------------------------------------------------------*
   * chibios_rt::ThreadStayPoint                                            *
   *------------------------------------------------------------------------*/
  /**
   * @brief     Thread suspension point class.
   * @details   This class encapsulates a reference to a suspended thread.
   */
  class ThreadStayPoint {
    /**
     * @brief   Pointer to the suspended thread.
     */
    thread_reference_t thread_ref = nullptr;

  public:
    /**
     * @brief   Thread stay point constructor.
     *
     * @init
     */
    ThreadStayPoint() {

    }

    /* Prohibit copy construction and assignment.*/
    ThreadStayPoint(const ThreadStayPoint &) = delete;
    ThreadStayPoint &operator=(const ThreadStayPoint &) = delete;

    /**
     * @brief   Suspends the current thread on the stay point.
     * @details The suspended thread becomes the referenced thread. It is
     *          possible to use this method only if the thread reference
     *          was set to @p nullptr.
     *
     * @return                  The incoming message.
     *
     * @sclass
     */
    msg_t suspendS(void) {

      return chThdSuspendS(&thread_ref);
    }

    /**
     * @brief   Suspends the current thread on the stay point with timeout.
     * @details The suspended thread becomes the referenced thread. It is
     *          possible to use this method only if the thread reference
     *          was set to @p nullptr.
     *
     *
     * @param[in] timeout   the number of ticks before the operation timeouts,
     *                      the following special values are allowed:
     *                      - @a TIME_IMMEDIATE immediate timeout.
     *                      - @a TIME_INFINITE no timeout.
     *                      .
     * @return              A message specifying how the invoking thread has
     *                      been released from the semaphore.
     * @retval MSG_OK       if the binary semaphore has been successfully
     *                      taken.
     * @retval MSG_RESET    if the binary semaphore has been reset using
     *                      @p chBSemReset().
     * @retval MSG_TIMEOUT  if the binary semaphore has not been signaled
     *                      or reset within the specified timeout.
     *
     * @sclass
     */
    msg_t suspendS(sysinterval_t timeout) {

      return chThdSuspendTimeoutS(&thread_ref, timeout);
    }

    /**
     * @brief   Resumes the currently referenced thread, if any.
     *
     * @param[in] msg       the wakeup message
     *
     * @iclass
     */
    void resumeI(msg_t msg) {

      chThdResumeI(&thread_ref, msg);
    }

    /**
     * @brief   Resumes the currently referenced thread, if any.
     *
     * @param[in] msg       the wakeup message
     *
     * @sclass
     */
    void resumeS(msg_t msg) {

      chThdResumeS(&thread_ref, msg);
    }
  };

  /*------------------------------------------------------------------------*
   * chibios_rt::SynchronizationObject                                      *
   *------------------------------------------------------------------------*/
  /**
   * @brief     Base class for all synchronization objects.
   * @note      No other uses.
   */
  class SynchronizationObject {
  };

  /*------------------------------------------------------------------------*
   * chibios_rt::ThreadsQueue                                               *
   *------------------------------------------------------------------------*/
  /**
   * @brief     Threads queue class.
   * @details   This class encapsulates a queue of threads.
   */
  class ThreadsQueue : public SynchronizationObject {
    /**
     * @brief   Pointer to the system thread.
     */
    threads_queue_t threads_queue;

  public:
    /**
     * @brief   Threads queue constructor.
     *
     * @init
     */
    ThreadsQueue() {

      chThdQueueObjectInit(&threads_queue);
    }

    /* Prohibit copy construction and assignment.*/
    ThreadsQueue(const ThreadsQueue &) = delete;
    ThreadsQueue &operator=(const ThreadsQueue &) = delete;

    /**
     * @brief   Enqueues the caller thread on a threads queue object.
     * @details The caller thread is enqueued and put to sleep until it is
     *          dequeued or the specified timeouts expires.
     *
     * @param[in] timeout   the timeout in system ticks, the special values are
     *                      handled as follow:
     *                      - @a TIME_INFINITE the thread enters an infinite sleep
     *                        state.
     *                      - @a TIME_IMMEDIATE the thread is not enqueued and
     *                        the function returns @p MSG_TIMEOUT as if a timeout
     *                        occurred.
     *                      .
     * @return              The message from @p osalQueueWakeupOneI() or
     *                      @p osalQueueWakeupAllI() functions.
     * @retval MSG_TIMEOUT  if the thread has not been dequeued within the
     *                      specified timeout or if the function has been
     *                      invoked with @p TIME_IMMEDIATE as timeout
     *                      specification.
     *
     * @sclass
     */
    msg_t enqueueSelfS(sysinterval_t timeout) {

      return chThdEnqueueTimeoutS(&threads_queue, timeout);
    }

    /**
     * @brief   Dequeues and wakes up one thread from the threads queue object,
     *          if any.
     *
     * @param[in] msg       the message code
     *
     * @iclass
     */
    void dequeueNextI(msg_t msg) {

      chThdDequeueNextI(&threads_queue, msg);
    }

    /**
     * @brief   Dequeues and wakes up all threads from the threads queue object.
     *
     * @param[in] msg       the message code
     *
     * @iclass
     */
    void chdequeueAllI(msg_t msg) {

      chThdDequeueAllI(&threads_queue, msg);
    }
};

#if (CH_CFG_USE_SEMAPHORES == TRUE) || defined(__DOXYGEN__)
  /*------------------------------------------------------------------------*
   * chibios_rt::CounterSemaphore                                           *
   *------------------------------------------------------------------------*/
  /**
   * @brief   Class encapsulating a semaphore.
   */
  class CounterSemaphore : public SynchronizationObject {
    /**
     * @brief   Embedded @p semaphore_t structure.
     */
    semaphore_t sem;

  public:
    /**
     * @brief   CounterSemaphore constructor.
     * @details The embedded @p Semaphore structure is initialized.
     *
     * @param[in] n             the semaphore counter value, must be greater
     *                          or equal to zero
     *
     * @init
     */
    CounterSemaphore(cnt_t n) {

      chSemObjectInit(&sem, n);
    }

    /**
     * @brief   Performs a reset operation on the semaphore.
     * @post    After invoking this function all the threads waiting on the
     *          semaphore, if any, are released and the semaphore counter is
     *          set to the specified, non negative, value.
     * @note    The released threads can recognize they were waked up by a
     *          reset rather than a signal because the @p chSemWait() will
     *          return @p MSG_RESET instead of @p MSG_OK.
     *
     * @param[in] n         the new value of the semaphore counter. The value
     *                      must be non-negative.
     *
     * @api
     */
    void reset(cnt_t n) {

      chSemReset(&sem, n);
    }

    /**
     * @brief   Performs a reset operation on the semaphore.
     * @post    After invoking this function all the threads waiting on the
     *          semaphore, if any, are released and the semaphore counter is
     *          set to the specified, non negative, value.
     * @post    This function does not reschedule so a call to a rescheduling
     *          function must be performed before unlocking the kernel. Note
     *          that interrupt handlers always reschedule on exit so an
     *          explicit reschedule must not be performed in ISRs.
     * @note    The released threads can recognize they were waked up by a
     *          reset rather than a signal because the @p chSemWait() will
     *          return @p MSG_RESET instead of @p MSG_OK.
     *
     * @param[in] n         the new value of the semaphore counter. The value
     *                      must be non-negative.
     *
     * @iclass
     */
    void resetI(cnt_t n) {

      chSemResetI(&sem, n);
    }

    /**
     * @brief   Performs a wait operation on a semaphore.
     *
     * @return              A message specifying how the invoking thread has
     *                      been released from the semaphore.
     * @retval MSG_OK       if the thread has not stopped on the semaphore or
     *                      the semaphore has been signaled.
     * @retval MSG_RESET    if the semaphore has been reset using
     *                      @p chSemReset().
     *
     * @api
     */
    msg_t wait(void) {

      return chSemWait(&sem);
    }

    /**
     * @brief   Performs a wait operation on a semaphore.
     *
     * @return              A message specifying how the invoking thread has
     *                      been released from the semaphore.
     * @retval MSG_OK       if the thread has not stopped on the semaphore or
     *                      the semaphore has been signaled.
     * @retval MSG_RESET    if the semaphore has been reset using
     *                      @p chSemReset().
     *
     * @sclass
     */
    msg_t waitS(void) {

      return chSemWaitS(&sem);
    }

    /**
     * @brief   Performs a wait operation on a semaphore with timeout
     *          specification.
     *
     * @param[in] timeout   the number of ticks before the operation timeouts,
     *                      the following special values are allowed:
     *                      - @a TIME_IMMEDIATE immediate timeout.
     *                      - @a TIME_INFINITE no timeout.
     *                      .
     * @return              A message specifying how the invoking thread has
     *                      been released from the semaphore.
     * @retval MSG_OK       if the thread has not stopped on the semaphore or
     *                      the semaphore has been signaled.
     * @retval MSG_RESET    if the semaphore has been reset using
     *                      @p chSemReset().
     * @retval MSG_TIMEOUT  if the semaphore has not been signaled or reset
     *                      within the specified timeout.
     *
     * @api
     */
    msg_t wait(sysinterval_t timeout) {

      return chSemWaitTimeout(&sem, timeout);
    }

    /**
     * @brief   Performs a wait operation on a semaphore with timeout
     *          specification.
     *
     * @param[in] timeout   the number of ticks before the operation timeouts,
     *                      the following special values are allowed:
     *                      - @a TIME_IMMEDIATE immediate timeout.
     *                      - @a TIME_INFINITE no timeout.
     *                      .
     * @return              A message specifying how the invoking thread has
     *                      been released from the semaphore.
     * @retval MSG_OK       if the thread has not stopped on the semaphore or
     *                      the semaphore has been signaled.
     * @retval MSG_RESET    if the semaphore has been reset using
     *                      @p chSemReset().
     * @retval MSG_TIMEOUT  if the semaphore has not been signaled or reset
     *                      within the specified timeout.
     *
     * @sclass
     */
    msg_t waitS(sysinterval_t timeout) {

      return chSemWaitTimeoutS(&sem, timeout);
    }

    /**
     * @brief   Performs a signal operation on a semaphore.
     *
     * @api
     */
    void signal(void) {

      chSemSignal(&sem);
    }

    /**
     * @brief   Performs a signal operation on a semaphore.
     * @post    This function does not reschedule so a call to a rescheduling
     *          function must be performed before unlocking the kernel. Note
     *          that interrupt handlers always reschedule on exit so an
     *          explicit reschedule must not be performed in ISRs.
     *
     * @iclass
     */
    void signalI(void) {

      chSemSignalI(&sem);
    }

    /**
     * @brief   Adds the specified value to the semaphore counter.
     * @post    This function does not reschedule so a call to a rescheduling
     *          function must be performed before unlocking the kernel. Note
     *          that interrupt handlers always reschedule on exit so an explicit
     *          reschedule must not be performed in ISRs.
     *
     * @param[in] n         value to be added to the semaphore counter. The
     *                      value must be positive.
     *
     * @iclass
     */
    void addCounterI(cnt_t n) {

      chSemAddCounterI(&sem, n);
    }

    /**
     * @brief   Returns the semaphore counter value.
     *
     * @return                  The semaphore counter value.
     *
     * @iclass
     */
    cnt_t getCounterI(void) const {

      return chSemGetCounterI(&sem);
    }

    /**
     * @brief   Atomic signal and wait operations.
     *
     * @param[in] ssem          @p Semaphore object to be signaled
     * @param[in] wsem          @p Semaphore object to wait on
     * @return                  A message specifying how the invoking thread
     *                          has been released from the semaphore.
     * @retval MSG_OK           if the thread has not stopped on the semaphore
     *                          or the semaphore has been signaled.
     * @retval MSG_RESET        if the semaphore has been reset using
     *                          @p chSemReset().
     *
     * @api
     */
    static msg_t signalWait(CounterSemaphore *ssem,
                            CounterSemaphore *wsem) {

      return chSemSignalWait(&ssem->sem, &wsem->sem);
    }
  };

  /*------------------------------------------------------------------------*
   * chibios_rt::BinarySemaphore                                            *
   *------------------------------------------------------------------------*/
  /**
   * @brief   Class encapsulating a binary semaphore.
   */
  class BinarySemaphore : public SynchronizationObject {
    /**
     * @brief   Embedded @p binary_semaphore_t structure.
     */
    binary_semaphore_t bsem;

  public:
    /**
     * @brief   BinarySemaphore constructor.
     * @details The embedded @p ::BinarySemaphore structure is initialized.
     *
     * @param[in] taken     initial state of the binary semaphore:
     *                      - @a false, the initial state is not taken.
     *                      - @a true, the initial state is taken.
     *                      .
     *
     * @init
     */
    BinarySemaphore(bool taken) {

      chBSemObjectInit(&bsem, taken);
    }

    /**
     * @brief   Wait operation on the binary semaphore.
     *
     * @return              A message specifying how the invoking thread has
     *                      been released from the semaphore.
     * @retval MSG_OK       if the binary semaphore has been successfully
     *                      taken.
     * @retval MSG_RESET    if the binary semaphore has been reset using
     *                      @p chBSemReset().
     *
     * @api
     */
    msg_t wait(void) {

      return chBSemWait(&bsem);
    }

    /**
     * @brief   Wait operation on the binary semaphore.
     *
     * @return              A message specifying how the invoking thread has
     *                      been released from the semaphore.
     * @retval MSG_OK       if the binary semaphore has been successfully
     *                      taken.
     * @retval MSG_RESET    if the binary semaphore has been reset using
     *                      @p chBSemReset().
     *
     * @sclass
     */
    msg_t waitS(void) {

      return chBSemWaitS(&bsem);
    }

    /**
     * @brief   Wait operation on the binary semaphore.
     *
     * @param[in] timeout   the number of ticks before the operation timeouts,
     *                      the following special values are allowed:
     *                      - @a TIME_IMMEDIATE immediate timeout.
     *                      - @a TIME_INFINITE no timeout.
     *                      .
     * @return              A message specifying how the invoking thread has
     *                      been released from the semaphore.
     * @retval MSG_OK       if the binary semaphore has been successfully
     *                      taken.
     * @retval MSG_RESET    if the binary semaphore has been reset using
     *                      @p chBSemReset().
     * @retval MSG_TIMEOUT  if the binary semaphore has not been signaled
     *                      or reset within the specified timeout.
     *
     * @api
     */
    msg_t wait(sysinterval_t timeout) {

      return chBSemWaitTimeout(&bsem, timeout);
    }

    /**
     * @brief   Wait operation on the binary semaphore.
     *
     * @param[in] timeout   the number of ticks before the operation timeouts,
     *                      the following special values are allowed:
     *                      - @a TIME_IMMEDIATE immediate timeout.
     *                      - @a TIME_INFINITE no timeout.
     *                      .
     * @return              A message specifying how the invoking thread has
     *                      been released from the semaphore.
     * @retval MSG_OK       if the binary semaphore has been successfully
     *                      taken.
     * @retval MSG_RESET    if the binary semaphore has been reset using
     *                      @p chBSemReset().
     * @retval MSG_TIMEOUT  if the binary semaphore has not been signaled
     *                      or reset within the specified timeout.
     *
     * @sclass
     */
    msg_t waitS(sysinterval_t timeout) {

      return chBSemWaitTimeoutS(&bsem, timeout);
    }

    /**
     * @brief   Reset operation on the binary semaphore.
     * @note    The released threads can recognize they were waked up by a
     *          reset rather than a signal because the @p chBSemWait() will
     *          return @p MSG_RESET instead of @p MSG_OK.
     *
     * @param[in] taken     new state of the binary semaphore
     *                      - @a FALSE, the new state is not taken.
     *                      - @a TRUE, the new state is taken.
     *                      .
     *
     * @api
     */
    void reset(bool taken) {

      chBSemReset(&bsem, taken);
    }

    /**
     * @brief   Reset operation on the binary semaphore.
     * @note    The released threads can recognize they were waked up by a
     *          reset rather than a signal because the @p chBSemWait() will
     *          return @p MSG_RESET instead of @p MSG_OK.
     * @note    This function does not reschedule.
     *
     * @param[in] taken     new state of the binary semaphore
     *                      - @a FALSE, the new state is not taken.
     *                      - @a TRUE, the new state is taken.
     *                      .
     *
     * @iclass
     */
    void resetI(bool taken) {

      chBSemResetI(&bsem, taken);
    }

    /**
     * @brief   Performs a signal operation on a binary semaphore.
     *
     * @api
     */
    void signal(void) {

      chBSemSignal(&bsem);
    }

    /**
     * @brief   Performs a signal operation on a binary semaphore.
     * @note    This function does not reschedule.
     *
     * @iclass
     */
    void signalI(void) {

      chBSemSignalI(&bsem);
    }

    /**
     * @brief   Returns the binary semaphore current state.
     *
     * @return              The binary semaphore current state.
     * @retval false        if the binary semaphore is not taken.
     * @retval true         if the binary semaphore is taken.
     *
     * @iclass
     */
    bool getStateI(void) const {

      return (bool)chBSemGetStateI(&bsem);
    }
};
#endif /* CH_CFG_USE_SEMAPHORES == TRUE */

#if (CH_CFG_USE_MUTEXES == TRUE) || defined(__DOXYGEN__)
  /*------------------------------------------------------------------------*
   * chibios_rt::Mutex                                                      *
   *------------------------------------------------------------------------*/
  /**
   * @brief   Class encapsulating a mutex.
   */
  class Mutex : public SynchronizationObject {
    /**
     * @brief   Embedded @p mutex_t structure.
     */
    mutex_t mutex;

  public:
    /**
     * @brief   Mutex object constructor.
     * @details The embedded @p mutex_t structure is initialized.
     *
     * @init
     */
    Mutex(void) {

      chMtxObjectInit(&mutex);
    }

    /**
     * @brief   Tries to lock a mutex.
     * @details This function attempts to lock a mutex, if the mutex is already
     *          locked by another thread then the function exits without
     *          waiting.
     * @post    The mutex is locked and inserted in the per-thread stack of
     *          owned mutexes.
     * @note    This function does not have any overhead related to the
     *          priority inheritance mechanism because it does not try to
     *          enter a sleep state.
     *
     * @return              The operation status.
     * @retval TRUE         if the mutex has been successfully acquired
     * @retval FALSE        if the lock attempt failed.
     *
     * @api
     */
    bool tryLock(void) {

      return chMtxTryLock(&mutex);
    }

    /**
     * @brief   Tries to lock a mutex.
     * @details This function attempts to lock a mutex, if the mutex is already
     *          taken by another thread then the function exits without
     *          waiting.
     * @post    The mutex is locked and inserted in the per-thread stack of
     *          owned mutexes.
     * @note    This function does not have any overhead related to the
     *          priority inheritance mechanism because it does not try to
     *          enter a sleep state.
     *
     * @return              The operation status.
     * @retval TRUE         if the mutex has been successfully acquired
     * @retval FALSE        if the lock attempt failed.
     *
     * @sclass
     */
    bool tryLockS(void) {

      return chMtxTryLockS(&mutex);
    }

    /**
     * @brief   Locks the specified mutex.
     * @post    The mutex is locked and inserted in the per-thread stack of
     *          owned mutexes.
     *
     * @api
     */
    void lock(void) {

      chMtxLock(&mutex);
    }

    /**
     * @brief   Locks the specified mutex.
     * @post    The mutex is locked and inserted in the per-thread stack of
     *          owned mutexes.
     *
     * @sclass
     */
    void lockS(void) {

      chMtxLockS(&mutex);
    }

    /**
     * @brief   Unlocks the next owned mutex in reverse lock order.
     * @pre     The invoking thread <b>must</b> have at least one owned mutex.
     * @post    The mutex is unlocked and removed from the per-thread stack of
     *          owned mutexes.
     *
     * @api
     */
    void unlock(void) {

      chMtxUnlock(&mutex);
    }

    /**
     * @brief   Unlocks the next owned mutex in reverse lock order.
     * @pre     The invoking thread <b>must</b> have at least one owned mutex.
     * @post    The mutex is unlocked and removed from the per-thread stack of
     *          owned mutexes.
     * @post    This function does not reschedule so a call to a rescheduling
     *          function must be performed before unlocking the kernel.
     *
     * @sclass
     */
    void unlockS(void) {

      chMtxUnlockS(&mutex);
    }

    /**
     * @brief   Unlocks the next owned mutex in reverse lock order.
     * @pre     The invoking thread <b>must</b> have at least one owned mutex.
     * @post    The mutex is unlocked and removed from the per-thread stack of
     *          owned mutexes.
     *
     * @return              A pointer to the unlocked mutex.
     *
     * @api
     */
    void unlockMutex(void) {

      chMtxUnlock(&mutex);
    }

    /**
     * @brief   Unlocks the next owned mutex in reverse lock order.
     * @pre     The invoking thread <b>must</b> have at least one owned mutex.
     * @post    The mutex is unlocked and removed from the per-thread stack of
     *          owned mutexes.
     * @post    This function does not reschedule so a call to a rescheduling
     *          function must be performed before unlocking the kernel.
     *
     * @return              A pointer to the unlocked mutex.
     *
     * @sclass
     */
    void unlockMutexS(void) {

      chMtxUnlockS(&mutex);
    }
  };

  /*------------------------------------------------------------------------*
   * chibios_rt::MutexLocker                                                *
   *------------------------------------------------------------------------*/
  /**
   * @brief   RAII helper for mutexes.
   */
  class MutexLocker
  {
    Mutex& mutex;

  public:
      MutexLocker(Mutex& m) : mutex(m) {

        mutex.lock();
      }

      ~MutexLocker() {

        mutex.unlock();
      }
  };

#if (CH_CFG_USE_CONDVARS == TRUE) || defined(__DOXYGEN__)
  /*------------------------------------------------------------------------*
   * chibios_rt::Monitor                                                    *
   *------------------------------------------------------------------------*/
  /**
   * @brief   Template class to be used for implementing a monitor.
   */
  template <unsigned N>
  class Monitor: protected Mutex {
    condition_variable_t condvars[N];

  protected:
    /**
     * @brief   Waits on the condition variable releasing the mutex lock.
     *
     * @param[in] var       the condition variable index
     * @return              A message specifying how the invoking thread has
     *                      been released from the condition variable.
     * @retval MSG_OK       if the condition variable has been signaled using
     *                      @p signal().
     * @retval MSG_RESET    if the condition variable has been signaled using
     *                      @p broadcast().
     *
     * @api
     */
    msg_t wait(unsigned var) {

      chDbgCheck(var < N);

      return chCondWait(&condvars[var]);
    }

    /**
     * @brief   Waits on the condition variable releasing the mutex lock.
     *
     * @param[in] var       the condition variable index
     * @return              A message specifying how the invoking thread has
     *                      been released from the condition variable.
     * @retval MSG_OK       if the condition variable has been signaled using
     *                      @p signal().
     * @retval MSG_RESET    if the condition variable has been signaled using
     *                      @p broadcast().
     *
     * @sclass
     */
    msg_t waitS(unsigned var) {

      chDbgCheck(var < N);

      return chCondWaitS(&condvars[var]);
    }

#if (CH_CFG_USE_CONDVARS_TIMEOUT == TRUE) || defined(__DOXYGEN__)
    /**
     * @brief   Waits on the CondVar while releasing the controlling mutex.
     *
     * @param[in] var       the condition variable index
     * @param[in] timeout   the number of ticks before the operation fails
     * @return              The wakep mode.
     * @retval MSG_OK       if the condition variable was signaled using
     *                      @p signal().
     * @retval MSG_RESET    if the condition variable was signaled using
     *                      @p broadcast().
     * @retval MSG_TIMEOUT  if the condition variable was not signaled
     *                      within the specified timeout.
     *
     * @api
     */
    msg_t wait(unsigned var, sysinterval_t timeout) {

      chDbgCheck(var < N);

      return chCondWaitTimeout(&condvars[var], timeout);
    }

    /**
     * @brief   Waits on the CondVar while releasing the controlling mutex.
     *
     * @param[in] var       the condition variable index
     * @param[in] timeout   the number of ticks before the operation fails
     * @return              The wakep mode.
     * @retval MSG_OK       if the condition variable was signaled using
     *                      @p signal().
     * @retval MSG_RESET    if the condition variable was signaled using
     *                      @p broadcast().
     * @retval MSG_TIMEOUT  if the condition variable was not signaled
     *                      within the specified timeout.
     *
     * @sclass
     */
    msg_t waitS(unsigned var, sysinterval_t timeout) {

      chDbgCheck(var < N);

      return chCondWaitTimeoutS(&condvars[var], timeout);
    }
#endif /* CH_CFG_USE_CONDVARS_TIMEOUT == TRUE */

  public:
    /**
     * @brief   Monitor object constructor.
     *
     * @init
     */
    Monitor(void) : Mutex() {

      for (unsigned i = 0; i < N; i++) {
        chCondObjectInit(&condvars[i]);
      }
    }

    /**
     * @brief   Signals one thread that is waiting on the condition variable.
     *
     * @param[in] var       the condition variable index
     *
     * @api
     */
    void signal(unsigned var) {

      chDbgCheck(var < N);

      chCondSignal(&condvars[var]);
    }

    /**
     * @brief   Signals one thread that is waiting on the condition variable.
     * @post    This function does not reschedule so a call to a rescheduling
     *          function must be performed before unlocking the kernel. Note
     *          that interrupt handlers always reschedule on exit so an
     *          explicit reschedule must not be performed in ISRs.
     *
     * @param[in] var       the condition variable index
     *
     * @iclass
     */
    void signalI(unsigned var) {

      chDbgCheck(var < N);

      chCondSignalI(&condvars[var]);
    }

    /**
     * @brief   Signals all threads that are waiting on the condition variable.
     *
     * @param[in] var       the condition variable index
     *
     * @api
     */
    void broadcast(unsigned var) {

      chDbgCheck(var < N);

      chCondBroadcast(&condvars[var]);
    }

    /**
     * @brief   Signals all threads that are waiting on the condition variable.
     * @post    This function does not reschedule so a call to a rescheduling
     *          function must be performed before unlocking the kernel. Note
     *          that interrupt handlers always reschedule on exit so an
     *          explicit reschedule must not be performed in ISRs.
     *
     * @param[in] var       the condition variable index
     *
     * @iclass
     */
    void broadcastI(unsigned var) {

      chDbgCheck(var < N);

      chCondBroadcastI(&condvars[var]);
    }
  };

#endif /* CH_CFG_USE_CONDVARS == TRUE */
#endif /* CH_CFG_USE_MUTEXES == TRUE */

#if (CH_CFG_USE_EVENTS == TRUE) || defined(__DOXYGEN__)
  /*------------------------------------------------------------------------*
   * chibios_rt::EvtListener                                                *
   *------------------------------------------------------------------------*/
  /**
   * @brief   Class encapsulating an event listener.
   */
  class EventListener {
  public:
    /**
     * @brief   Embedded @p event_listener_t structure.
     */
    event_listener_t ev_listener;

    /**
     * @brief   Returns the pending flags from the listener and clears them.
     *
     * @return              The flags added to the listener by the
     *                      associated event source.
     *
     * @api
     */
    eventflags_t getAndClearFlags(void) {

      return chEvtGetAndClearFlags(&ev_listener);
    }

    /**
     * @brief   Returns the flags associated to an @p event_listener_t.
     * @details The flags are returned and the @p event_listener_t flags mask is
     *          cleared.
     *
     * @return              The flags added to the listener by the associated
     *                      event source.
     *
     * @iclass
     */
    eventflags_t getAndClearFlagsI(void) {

      return chEvtGetAndClearFlagsI(&ev_listener);
    }
  };

  /*------------------------------------------------------------------------*
   * chibios_rt::EvtSource                                                  *
   *------------------------------------------------------------------------*/
  /**
   * @brief   Class encapsulating an event source.
   */
  class EventSource {
    /**
     * @brief   Embedded @p event_source_t structure.
     */
    event_source_t ev_source;

   public:
   /**
     * @brief   EvtSource object constructor.
     * @details The embedded @p event_source_t structure is initialized.
     *
     * @init
     */
    EventSource(void) {

      chEvtObjectInit(&ev_source);
    }

    /**
     * @brief   Registers a listener on the event source.
     *
     * @param[in] elp       pointer to the @p EvtListener object
     * @param[in] eid       numeric identifier assigned to the Event
     *                      Listener
     *
     * @api
     */
    void registerOne(EventListener *elp,
                     eventid_t eid) {

      chEvtRegister(&ev_source, &elp->ev_listener, eid);
    }

    /**
     * @brief   Registers an Event Listener on an Event Source.
     * @note    Multiple Event Listeners can specify the same bits to be added.
     *
     * @param[in] elp       pointer to the @p EvtListener object
     * @param[in] emask     the mask of event flags to be pended to the
     *                      thread when the event source is broadcasted
     *
     * @api
     */
    void registerMask(EventListener *elp,
                      eventmask_t emask) {

      chEvtRegisterMask(&ev_source, &elp->ev_listener, emask);
    }

    /**
     * @brief   Unregisters a listener.
     * @details The specified listeners is no more signaled by the event
     *          source.
     *
     * @param[in] elp       the listener to be unregistered
     *
     * @api
     */
    void unregister(EventListener *elp) {

      chEvtUnregister(&ev_source, &elp->ev_listener);
    }

    /**
     * @brief   Broadcasts on an event source.
     * @details All the listeners registered on the event source are signaled
     *          and the flags are added to the listener's flags mask.
     *
     * @param[in] flags     the flags set to be added to the listener
     *                      flags mask
     *
     * @api
     */
    void broadcastFlags(eventflags_t flags) {

      chEvtBroadcastFlags(&ev_source, flags);
    }

    /**
     * @brief   Broadcasts on an event source.
     * @details All the listeners registered on the event source are signaled
     *          and the flags are added to the listener's flags mask.
     *
     * @param[in] flags     the flags set to be added to the listener
     *                      flags mask
     *
     * @iclass
     */
    void broadcastFlagsI(eventflags_t flags) {

      chEvtBroadcastFlagsI(&ev_source, flags);
    }
  };
#endif /* CH_CFG_USE_EVENTS == TRUE */

#if (CH_CFG_USE_MAILBOXES == TRUE) || defined(__DOXYGEN__)
  /*------------------------------------------------------------------------*
   * chibios_rt::Mailbox                                                    *
   *------------------------------------------------------------------------*/
  /**
   * @brief   Base mailbox class.
   *
   * @param T               type of objects that mailbox able to handle
   */
  template <typename T>
  class MailboxBase {

    /**
     * @brief   Embedded @p mailbox_t structure.
     */
    mailbox_t mb;

   public:
   /**
     * @brief   Mailbox constructor.
     * @details The embedded @p mailbox_t structure is initialized.
     *
     * @param[in] buf       pointer to the messages buffer as an array of
     *                      @p msg_t
     * @param[in] n         number of elements in the buffer array
     *
     * @init
     */
    MailboxBase(msg_t *buf, cnt_t n) {

      chMBObjectInit(&mb, buf, n);
    }

    /**
     * @brief   Resets a Mailbox object.
     * @details All the waiting threads are resumed with status @p MSG_RESET
     *          and the queued messages are lost.
     *
     * @api
     */
    void reset(void) {

      chMBReset(&mb);
    }

    /**
     * @brief   Terminates the reset state.
     *
     * @xclass
     */
    void resumeX(void) {

      chMBResumeX(&mb);
    }

    /**
     * @brief   Posts a message into a mailbox.
     * @details The invoking thread waits until a empty slot in the mailbox
     *          becomes available or the specified time runs out.
     *
     * @param[in] msg       the message to be posted on the mailbox
     * @param[in] timeout   the number of ticks before the operation timeouts,
     *                      the following special values are allowed:
     *                      - @a TIME_IMMEDIATE immediate timeout.
     *                      - @a TIME_INFINITE no timeout.
     *                      .
     * @return              The operation status.
     * @retval MSG_OK       if a message has been correctly posted.
     * @retval MSG_RESET    if the mailbox has been reset while waiting.
     * @retval MSG_TIMEOUT  if the operation has timed out.
     *
     * @api
     */
    msg_t post(T msg, sysinterval_t timeout) {

      return chMBPostTimeout(&mb, reinterpret_cast<msg_t>(msg), timeout);
    }

    /**
     * @brief   Posts a message into a mailbox.
     * @details The invoking thread waits until a empty slot in the mailbox
     *          becomes available or the specified time runs out.
     *
     * @param[in] msg       the message to be posted on the mailbox
     * @param[in] timeout   the number of ticks before the operation timeouts,
     *                      the following special values are allowed:
     *                      - @a TIME_IMMEDIATE immediate timeout.
     *                      - @a TIME_INFINITE no timeout.
     *                      .
     * @return              The operation status.
     * @retval MSG_OK       if a message has been correctly posted.
     * @retval MSG_RESET    if the mailbox has been reset while waiting.
     * @retval MSG_TIMEOUT  if the operation has timed out.
     *
     * @sclass
     */
    msg_t postS(T msg, sysinterval_t timeout) {

      return chMBPostTimeoutS(&mb, reinterpret_cast<msg_t>(msg), timeout);
    }

    /**
     * @brief   Posts a message into a mailbox.
     * @details This variant is non-blocking, the function returns a timeout
     *          condition if the queue is full.
     *
     * @param[in] msg       the message to be posted on the mailbox
     * @return              The operation status.
     * @retval MSG_OK       if a message has been correctly posted.
     * @retval MSG_TIMEOUT  if the mailbox is full and the message cannot be
     *                      posted.
     *
     * @iclass
     */
    msg_t postI(T msg) {

      return chMBPostI(&mb, reinterpret_cast<msg_t>(msg));
    }

    /**
     * @brief   Posts an high priority message into a mailbox.
     * @details The invoking thread waits until a empty slot in the mailbox
     *          becomes available or the specified time runs out.
     *
     * @param[in] msg       the message to be posted on the mailbox
     * @param[in] timeout   the number of ticks before the operation timeouts,
     *                      the following special values are allowed:
     *                      - @a TIME_IMMEDIATE immediate timeout.
     *                      - @a TIME_INFINITE no timeout.
     *                      .
     * @return              The operation status.
     * @retval MSG_OK       if a message has been correctly posted.
     * @retval MSG_RESET    if the mailbox has been reset while waiting.
     * @retval MSG_TIMEOUT  if the operation has timed out.
     *
     * @api
     */
    msg_t postAhead(T msg, sysinterval_t timeout) {

      return chMBPostAheadTimeout(&mb, reinterpret_cast<msg_t>(msg), timeout);
    }

    /**
     * @brief   Posts an high priority message into a mailbox.
     * @details The invoking thread waits until a empty slot in the mailbox
     *          becomes available or the specified time runs out.
     *
     * @param[in] msg       the message to be posted on the mailbox
     * @param[in] timeout   the number of ticks before the operation timeouts,
     *                      the following special values are allowed:
     *                      - @a TIME_IMMEDIATE immediate timeout.
     *                      - @a TIME_INFINITE no timeout.
     *                      .
     * @return              The operation status.
     * @retval MSG_OK       if a message has been correctly posted.
     * @retval MSG_RESET    if the mailbox has been reset while waiting.
     * @retval MSG_TIMEOUT  if the operation has timed out.
     *
     * @sclass
     */
    msg_t postAheadS(T msg, sysinterval_t timeout) {

      return chMBPostAheadTimeoutS(&mb, reinterpret_cast<msg_t>(msg), timeout);
    }

    /**
     * @brief   Posts an high priority message into a mailbox.
     * @details This variant is non-blocking, the function returns a timeout
     *          condition if the queue is full.
     *
     * @param[in] msg       the message to be posted on the mailbox
     * @return              The operation status.
     * @retval MSG_OK       if a message has been correctly posted.
     * @retval MSG_TIMEOUT  if the mailbox is full and the message cannot be
     *                      posted.
     *
     * @iclass
     */
    msg_t postAheadI(T msg) {

      return chMBPostAheadI(&mb, reinterpret_cast<msg_t>(msg));
    }

    /**
     * @brief   Retrieves a message from a mailbox.
     * @details The invoking thread waits until a message is posted in the
     *          mailbox or the specified time runs out.
     *
     * @param[out] msgp     pointer to a message variable for the received
     * @param[in] timeout   message the number of ticks before the operation
     *                      timeouts, the following special values are allowed:
     *                      - @a TIME_IMMEDIATE immediate timeout.
     *                      - @a TIME_INFINITE no timeout.
     *                      .
     * @return              The operation status.
     * @retval MSG_OK       if a message has been correctly fetched.
     * @retval MSG_RESET    if the mailbox has been reset while waiting.
     * @retval MSG_TIMEOUT  if the operation has timed out.
     *
     * @api
     */
    msg_t fetch(T *msgp, sysinterval_t timeout) {

      return chMBFetchTimeout(&mb, reinterpret_cast<msg_t*>(msgp), timeout);
    }

    /**
     * @brief   Retrieves a message from a mailbox.
     * @details The invoking thread waits until a message is posted in the
     *          mailbox or the specified time runs out.
     *
     * @param[out] msgp     pointer to a message variable for the received
     * @param[in] timeout   message the number of ticks before the operation
     *                      timeouts, the following special values are allowed:
     *                      - @a TIME_IMMEDIATE immediate timeout.
     *                      - @a TIME_INFINITE no timeout.
     *                      .
     * @return              The operation status.
     * @retval MSG_OK       if a message has been correctly fetched.
     * @retval MSG_RESET    if the mailbox has been reset while waiting.
     * @retval MSG_TIMEOUT  if the operation has timed out.
     *
     * @sclass
     */
    msg_t fetchS(T *msgp, sysinterval_t timeout) {

      return chMBFetchTimeoutS(&mb, reinterpret_cast<msg_t*>(msgp), timeout);
    }

    /**
     * @brief   Retrieves a message from a mailbox.
     * @details This variant is non-blocking, the function returns a timeout
     *          condition if the queue is empty.
     *
     * @param[out] msgp     pointer to a message variable for the received
     *                      message
     * @return              The operation status.
     * @retval MSG_OK       if a message has been correctly fetched.
     * @retval MSG_TIMEOUT  if the mailbox is empty and a message cannot be
     *                      fetched.
     *
     * @iclass
     */
    msg_t fetchI(T *msgp) {

      return chMBFetchI(&mb, reinterpret_cast<msg_t*>(msgp));
    }

    /**
     * @brief   Returns the next message in the queue without removing it.
     * @pre     A message must be waiting in the queue for this function to work
     *          or it would return garbage. The correct way to use this macro is
     *          to use @p getUsedCountI() and then use this macro, all within
     *          a lock state.
     *
     * @return              The next message in queue.
     *
     * @iclass
     */
    T peekI(const mailbox_t *mbp) const {

      return chMBPeekI(&mb);
    }

    /**
     * @brief   Returns the number of free message slots into a mailbox.
     * @note    Can be invoked in any system state but if invoked out of a
     *          locked state then the returned value may change after reading.
     * @note    The returned value can be less than zero when there are waiting
     *          threads on the internal semaphore.
     *
     * @return              The number of empty message slots.
     *
     * @iclass
     */
    cnt_t getFreeCountI(void) const {

      return chMBGetFreeCountI(&mb);
    }

    /**
     * @brief   Returns the number of used message slots into a mailbox.
     * @note    Can be invoked in any system state but if invoked out of a
     *          locked state then the returned value may change after reading.
     * @note    The returned value can be less than zero when there are waiting
     *          threads on the internal semaphore.
     *
     * @return              The number of queued messages.
     *
     * @iclass
     */
    cnt_t getUsedCountI(void) const {

      return chMBGetUsedCountI(&mb);
    }
  };

  /*------------------------------------------------------------------------*
   * chibios_rt::Mailbox                                                    *
   *------------------------------------------------------------------------*/
  /**
   * @brief     Template class encapsulating a mailbox and its messages buffer.
   *
   * @param N               length of the mailbox buffer
   */
  template <typename T, int N>
  class Mailbox : public MailboxBase<T> {

    static_assert(sizeof(T) <= sizeof(msg_t),
                  "Mailbox type does not fit in msg_t");

  private:
    msg_t   mb_buf[N];

  public:
    /**
     * @brief   Mailbox constructor.
     *
     * @init
     */
    Mailbox(void) :
      MailboxBase<T>(mb_buf, (cnt_t)(sizeof mb_buf / sizeof (msg_t))) {
    }
  };
#endif /* CH_CFG_USE_MAILBOXES == TRUE */

#if (CH_CFG_USE_MEMPOOLS == TRUE) || defined(__DOXYGEN__)
  /*------------------------------------------------------------------------*
   * chibios_rt::MemoryPool                                                 *
   *------------------------------------------------------------------------*/
  /**
   * @brief   Class encapsulating a memory pool.
   */
  class MemoryPool {
    /**
     * @brief   Embedded @p memory_pool_t structure.
     */
    memory_pool_t pool;

  public:
    /**
     * @brief   MemoryPool constructor.
     *
     * @param[in] size      the size of the objects contained in this memory
     *                      pool, the minimum accepted size is the size of
     *                      a pointer to void.
     * @param[in] provider  memory provider function for the memory pool or
     *                      @p nullptr if the pool is not allowed to grow
     *                      automatically
     *
     * @init
     */
    MemoryPool(size_t size, memgetfunc_t provider=0) : pool() {

      chPoolObjectInit(&pool, size, provider);
    }

    /**
     * @brief   MemoryPool constructor.
     *
     * @param[in] size      the size of the objects contained in this memory
     *                      pool, the minimum accepted size is the size of
     *                      a pointer to void.
     * @param[in] provider  memory provider function for the memory pool or
     *                      @p nullptr if the pool is not allowed to grow
     *                      automatically
     * @param[in] p         pointer to the array first element
     * @param[in] n         number of elements in the array
     *
     * @init
     */
    MemoryPool(size_t size, void* p, size_t n,
               memgetfunc_t provider=0) : pool() {

      chPoolObjectInit(&pool, size, provider);
      chPoolLoadArray(&pool, p, n);
    }

    /* Prohibit copy construction and assignment, but allow move.*/
    MemoryPool(const MemoryPool &) = delete;
    MemoryPool &operator=(const MemoryPool &) = delete;
    MemoryPool(MemoryPool &&) = default;
    MemoryPool &operator=(MemoryPool &&) = default;

    /**
     * @brief   Loads a memory pool with an array of static objects.
     * @pre     The memory pool must be already been initialized.
     * @pre     The array elements must be of the right size for the specified
     *          memory pool.
     * @post    The memory pool contains the elements of the input array.
     *
     * @param[in] p         pointer to the array first element
     * @param[in] n         number of elements in the array
     *
     * @api
     */
    void loadArray(void *p, size_t n) {

      chPoolLoadArray(&pool, p, n);
    }

    /**
     * @brief   Allocates an object from a memory pool.
     * @pre     The memory pool must be already been initialized.
     *
     * @return              The pointer to the allocated object.
     * @retval nullptr      if pool is empty.
     *
     * @iclass
     */
    void *allocI(void) {

      return chPoolAllocI(&pool);
    }

    /**
     * @brief   Allocates an object from a memory pool.
     * @pre     The memory pool must be already been initialized.
     *
     * @return              The pointer to the allocated object.
     * @retval nullptr      if pool is empty.
     *
     * @api
     */
    void *alloc(void) {

      return chPoolAlloc(&pool);
    }

    /**
     * @brief   Releases an object into a memory pool.
     * @pre     The memory pool must be already been initialized.
     * @pre     The freed object must be of the right size for the specified
     *          memory pool.
     * @pre     The object must be properly aligned to contain a pointer to
     *          void.
     *
     * @param[in] objp      the pointer to the object to be released
     *
     * @iclass
     */
    void free(void *objp) {

      chPoolFree(&pool, objp);
    }

    /**
     * @brief   Adds an object to a memory pool.
     * @pre     The memory pool must be already been initialized.
     * @pre     The added object must be of the right size for the specified
     *          memory pool.
     * @pre     The added object must be memory aligned to the size of
     *          @p stkalign_t type.
     * @note    This function is just an alias for @p chPoolFree() and has been
     *          added for clarity.
     *
     * @param[in] objp      the pointer to the object to be added
     *
     * @iclass
     */
    void freeI(void *objp) {

      chPoolFreeI(&pool, objp);
    }
  };

  /*------------------------------------------------------------------------*
   * chibios_rt::ObjectsPool                                                *
   *------------------------------------------------------------------------*/
  /**
   * @brief   Template class encapsulating a memory pool and its elements.
   */
  template<class T, size_t N>
  class ObjectsPool : public MemoryPool {
    /* The buffer is declared as an array of pointers to void for two
       reasons:
       1) The objects must be properly aligned to hold a pointer as
          first field.
       2) Objects are dirtied when loaded in the pool.*/
    void *pool_buf[(N * sizeof (T)) / sizeof (void *)];

  public:
    /**
     * @brief   ObjectsPool constructor.
     *
     * @init
     */
    ObjectsPool(void) : MemoryPool(sizeof (T), nullptr) {

      loadArray(pool_buf, N);
    }
  };

  /*------------------------------------------------------------------------*
   * chibios_rt::ThreadsPool                                                *
   *------------------------------------------------------------------------*/
  /**
   * @brief   Template class encapsulating a pool of threads.
   */
  template<size_t S, size_t N, const char *C>
  class ThreadsPool : public BaseDynamicThread {
    THD_WORKING_AREA(working_areas, S)[N];
    MemoryPool threads_pool;

  public:
    /**
     * @brief   ThreadsPool constructor.
     *
     * @init
     */
    ThreadsPool(void) : threads_pool(THD_WORKING_AREA_SIZE(S)) {

      threads_pool.loadArray(working_areas, N);
    }

    /**
     * @brief   Starts a dynamic thread from the pool.
     *
     * @param[in] prio          thread priority
     * @return                  A reference to the created thread with
     *                          reference counter set to one.
     *
     * @api
     */
    ThreadReference start(tprio_t prio) override {
      void _thd_start(void *arg);

      return ThreadReference(chThdCreateFromMemoryPool(&threads_pool.pool,
                                                       C,
                                                       prio,
                                                       _thd_start,
                                                       this));
     }
   };
#endif /* CH_CFG_USE_MEMPOOLS == TRUE */

#if (CH_CFG_USE_HEAP == TRUE) || defined(__DOXYGEN__)
  /*------------------------------------------------------------------------*
   * chibios_rt::Heap                                                       *
   *------------------------------------------------------------------------*/
  /**
   * @brief   Class encapsulating a heap.
   */
  class Heap {
    /**
     * @brief   Embedded @p memory_heap_t structure.
     */
    memory_heap_t heap;

  public:
    /**
     * @brief   Heap constructor.
     * @pre     Both the heap buffer base and the heap size must be aligned to
     *          the @p stkalign_t type size.
     *
     * @param[in] buffer    heap buffer base
     * @param[in] size      the size of the memory area located at \e buffer
     * @init
     */
    Heap(void *buffer, const size_t size) : heap() {

      chHeapObjectInit(&heap, buffer, size);
    }

    /* Prohibit copy construction and assignment, but allow move.*/
    Heap(const Heap &) = delete;
    Heap &operator=(const Heap &) = delete;
    Heap(Heap &&) = default;
    Heap &operator=(Heap &&) = default;

    /**
     * @brief   Allocates an object from a heap.
     * @pre     The heap must be already been initialized.
     *
     * @return              The pointer to the allocated object.
     * @retval nullptr      if pool is empty.
     *
     * @api
     */
    void *alloc(const size_t size) {

      return chHeapAlloc(&heap, size);
    }

    /**
     * @brief   Releases an object into the heap.
     *
     * @param[in] objp      the pointer to the object to be released
     *
     * @api
     */
    void free(void *objp) {

      chHeapFree(objp);
    }

    /**
     * @brief   Reports the heap status.
     *
     * @param[out] frag     the size of total fragmented free space
     * @param[in] largestp  pointer to a variable that will receive the largest
     *                      free free block found space or @p nullptr
     * @return              the number of fragments in the heap
     *
     * @api
     */
    size_t status(size_t &frag, size_t *largestp=0) {

      return chHeapStatus(&heap, &frag, largestp);
    }
  };
#endif /* CH_CFG_USE_MEMPOOLS == TRUE */

  /*------------------------------------------------------------------------*
   * chibios_rt::BaseSequentialStreamInterface                              *
   *------------------------------------------------------------------------*/
  /**
   * @brief   Interface of a BaseSequentialStream.
   * @note    You can cast a BaseSequentialStream to this interface and use
   *          it, the memory layout is the same.
   */
  class BaseSequentialStreamInterface {
  public:
    /**
     * @brief   Sequential Stream write.
     * @details The function writes data from a buffer to a stream.
     *
     * @param[in] bp        pointer to the data buffer
     * @param[in] n         the maximum amount of data to be transferred
     * @return              The number of bytes transferred. The return value
     *                      can be less than the specified number of bytes if
     *                      an end-of-file condition has been met.
     *
     * @api
     */
    virtual size_t write(const uint8_t *bp, const size_t n) = 0;

    /**
     * @brief   Sequential Stream read.
     * @details The function reads data from a stream into a buffer.
     *
     * @param[out] bp       pointer to the data buffer
     * @param[in] n         the maximum amount of data to be transferred
     * @return              The number of bytes transferred. The return value
     *                      can be less than the specified number of bytes if
     *                      an end-of-file condition has been met.
     *
     * @api
     */
    virtual size_t read(uint8_t *bp, const size_t n) = 0;

    /**
     * @brief   Sequential Stream blocking byte write.
     * @details This function writes a byte value to a channel. If the channel
     *          is not ready to accept data then the calling thread is
     *          suspended.
     *
     * @param[in] b         the byte value to be written to the channel
     *
     * @return              The operation status.
     * @retval Q_OK         if the operation succeeded.
     * @retval Q_RESET      if an end-of-file condition has been met.
     *
     * @api
     */
    virtual msg_t put(const uint8_t b) = 0;

    /**
     * @brief   Sequential Stream blocking byte read.
     * @details This function reads a byte value from a channel. If the data
     *          is not available then the calling thread is suspended.
     *
     * @return              A byte value from the queue.
     * @retval Q_RESET      if an end-of-file condition has been met.
     *
     * @api
     */
    virtual msg_t get(void) = 0;
  };
}

#endif /* _CH_HPP_ */

/** @} */
