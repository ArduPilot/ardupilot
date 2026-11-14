#pragma once

#include <stdint.h>

#include <AP_Common/AP_Common.h>

#include "AP_HAL_Boards.h"
#include "AP_HAL_Namespace.h"


class AP_HAL::Scheduler {
public:
    Scheduler() {}
    virtual void     init() = 0;
    virtual void     delay(uint16_t ms) = 0;

    /*
      delay for the given number of microseconds. This needs to be as
      accurate as possible - preferably within 100 microseconds.
     */
    virtual void     delay_microseconds(uint16_t us) = 0;

    /*
      delay for the given number of microseconds. On supported
      platforms this boosts the priority of the main thread for a
      short time when the time completes. The aim is to ensure the
      main thread runs at a higher priority than drivers for the start
      of each loop
     */
    virtual void     delay_microseconds_boost(uint16_t us) { delay_microseconds(us); }

    /*
      inform the scheduler that we are calling an operation from the
      main thread that may take an extended amount of time. This can
      be used to prevent watchdog reset during expected long delays
      A value of zero cancels the previous expected delay
     */
    virtual void     expect_delay_ms(uint32_t ms) { }

    /*
      return true if we are in a period of expected delay. This can be
      used to suppress error messages
     */
    virtual bool     in_expected_delay(void) const { return false; }

    /*
      end the priority boost from delay_microseconds_boost()
     */
    virtual void     boost_end(void) {}

    // register a function to be called by the scheduler if it needs
    // to sleep for more than min_time_ms
    virtual void     register_delay_callback(AP_HAL::Proc,
                                             uint16_t min_time_ms);
    // returns true if the Scheduler has called the delay callback
    // function.  If you are on the main thread that means your call
    // stack has the scheduler on it somewhere.
    virtual bool     in_delay_callback() const { return _in_delay_callback; }

    // register a high priority timer task
    virtual void     register_timer_process(AP_HAL::MemberProc) = 0;

    // register a low priority IO task
    virtual void     register_io_process(AP_HAL::MemberProc) = 0;

    virtual void     register_timer_failsafe(AP_HAL::Proc,
                                             uint32_t period_us) = 0;

    // check and set the startup state
    virtual void     set_system_initialized() = 0;
    virtual bool     is_system_initialized() = 0;

    virtual void     reboot(bool hold_in_bootloader = false) = 0;

    /**
       optional function to stop clock at a given time, used by log replay
     */
    virtual void     stop_clock(uint64_t time_usec) {}

    virtual bool     in_main_thread() const = 0;

    /*
      disable interrupts and return a context that can be used to
      restore the interrupt state. This can be used to protect
      critical regions

      Warning: may not be implemented on all HALs
     */
    virtual void *disable_interrupts_save(void) { return nullptr; }

    /*
      restore interrupt state from disable_interrupts_save()
     */
    virtual void restore_interrupts(void *) {}

    // called by subclasses when they need to delay for some time
    virtual void call_delay_cb();
    uint16_t _min_delay_cb_ms;

    /*
      priority_base is used to select what the priority for a new
      thread is relative to
     */
    enum priority_base {
        PRIORITY_BOOST,
        PRIORITY_MAIN,
        PRIORITY_SPI,
        PRIORITY_I2C,
        PRIORITY_CAN,
        PRIORITY_TIMER,
        PRIORITY_RCOUT,
        PRIORITY_LED,
        PRIORITY_RCIN,
        PRIORITY_IO,
        PRIORITY_UART,
        PRIORITY_STORAGE,
        PRIORITY_SCRIPTING,
        PRIORITY_NET,
        PRIORITY_ABSOLUTE, // Owned threads: native priority, not a base-relative offset.
    };
    
    /*
      create a new thread
     */
    virtual bool thread_create(AP_HAL::MemberProc proc, const char *name,
                               uint32_t stack_size, priority_base base, int8_t priority) {
        return false;
    }

#if HAL_OWNED_THREADS_ENABLED
    // Native application-priority range. Zero means unsupported. SITL exposes
    // nominal Linux priorities but retains the host's SCHED_OTHER policy.
    virtual uint8_t thread_priority_min() const { return 0; }
    virtual uint8_t thread_priority_max() const { return 0; }
    // Retain a failed worker's frames and resources without returning to it.
    [[noreturn]] virtual void thread_park() {
        while (true) {
            delay(1000);
        }
    }
    // Single-owner handle. Only thread_reap may destroy it; never cancel a
    // running thread or release its callback's context before successful reap.
    class OwnedThread {
    public:
        CLASS_NO_COPY(OwnedThread);
        // Reserved stack bytes, including platform minimum/alignment rounding.
        // On POSIX this includes in-stack libc/TLS bookkeeping, not just the
        // bytes available to the callback. Callers must leave adequate margin.
        virtual uint32_t stack_size() const = 0;
        // Known reservation including stack, handle and guard/working-area bytes.
        // Excludes allocator headers and opaque OS/libc thread bookkeeping.
        virtual uint32_t allocation_size() const = 0;
        // Percentage of one CPU; false when the HAL has no measurement.
        // Must not reset profiler counters shared with other diagnostics.
        virtual bool cpu_load(float &percent) const { return false; }

    protected:
        OwnedThread() = default;
        virtual ~OwnedThread() = default;
        // Return false while the callback runs. Once it returns, join the OS
        // thread before allowing destruction (may wait for its exit epilogue).
        virtual bool join_if_finished() = 0;
        friend class Scheduler;
    };

    // name must remain alive until reap. The returned handle has one owner;
    // creation and reaping must not be called from interrupts.
    // Stacks need not be DMA-capable. Device bindings must copy stack buffers
    // to suitable host storage when their driver requires DMA-safe memory.
    virtual OwnedThread *thread_create_owned(AP_HAL::MemberProc proc, const char *name,
                                            uint32_t stack_size, priority_base base, int16_t priority) {
        return nullptr;
    }

    struct StackFaultHandler {
        void *context;
        // Thread-context notification on a fresh stack, immediately before
        // exit. It must be lock-free and must not block or allocate.
        void (*fault)(void *context);
    };
    // On supported HALs, recover an attributable MPU stack-guard fault. The
    // backend must notify on a fresh stack, release thread-owned mutexes, then
    // exit; it cannot repair partially updated state protected by the mutexes.
    // Other HALs retain ordinary owned-thread behaviour (not fault isolation).
    virtual OwnedThread *thread_create_owned_guarded(AP_HAL::MemberProc proc, const char *name,
                                                    uint32_t stack_size, priority_base base, int16_t priority,
                                                    StackFaultHandler handler) {
        return thread_create_owned(proc, name, stack_size, base, priority);
    }

    // Call from a low-priority owner, not from the vehicle main loop. On true,
    // the thread can no longer access its context, code or stack; clears handle.
    static bool thread_reap(OwnedThread *&thread);
#endif

private:

    AP_HAL::Proc _delay_cb;
    bool _in_delay_callback;

};

/*
  helper macro and class to make using expect_delay_ms() safer and easier
 */
class ExpectDelay {
public:
    ExpectDelay(uint32_t ms);
    ~ExpectDelay();
};

#define EXPECT_DELAY_MS(ms) DELAY_JOIN( ms, __COUNTER__ )
#define DELAY_JOIN( ms, counter) _DO_DELAY_JOIN( ms, counter )
#define _DO_DELAY_JOIN( ms, counter ) ExpectDelay _getdelay ## counter(ms)


/*
  TIME_CHECK() can be used to unexpected detect long delays. Scatter
  them in likely places and any long delays will be printed
 */

class TimeCheck {
public:
    TimeCheck(uint32_t limit_ms, const char *file, uint32_t line);
    ~TimeCheck();
private:
    const uint32_t limit_ms;
    const uint32_t line;
    const char *file;
    uint32_t start_ms;
};

#define TIME_CHECK(limit_ms) JOIN_TC(limit_ms, __FILE__, __LINE__, __COUNTER__ )
#define JOIN_TC(limit_ms, file, line, counter ) _DO_JOIN_TC( limit_ms, file, line, counter )
#define _DO_JOIN_TC(limit_ms, file, line, counter ) TimeCheck _gettc ## counter(limit_ms, file, line)
