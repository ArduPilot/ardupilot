/*
  test of recursive HAL_Semaphore behaviour
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <stdio.h>

void setup();
void loop();

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

class SemaphoreRecursionTest {
public:
    HAL_Semaphore mtx;
    HAL_BinarySemaphore worker_request{false};
    HAL_BinarySemaphore worker_done{false};

    void setup(void);
    void worker(void);

private:
    void expect_bool(const char *label, bool actual, bool expected);
    void worker_attempt(const char *label, bool expected);

    volatile bool worker_result = false;
public:
    bool test_ok = true;
    bool complete = false;
    uint32_t complete_ms = 0;
};

void SemaphoreRecursionTest::expect_bool(const char *label, bool actual, bool expected)
{
    const bool pass = (actual == expected);
    if (!pass) {
        test_ok = false;
    }
    ::printf("%s: %s (actual=%u expected=%u)\n",
             label,
             pass ? "PASS" : "FAIL",
             unsigned(actual),
             unsigned(expected));
}

void SemaphoreRecursionTest::worker(void)
{
    while (true) {
        worker_request.wait_blocking();
        const bool locked = mtx.take_nonblocking();
        worker_result = locked;
        if (locked) {
            mtx.give();
        }
        worker_done.signal();
    }
}

void SemaphoreRecursionTest::worker_attempt(const char *label, bool expected)
{
    worker_request.signal();
    const bool done = worker_done.wait(1000000);
    expect_bool("worker completion", done, true);
    if (done) {
        expect_bool(label, worker_result, expected);
    }
}

void SemaphoreRecursionTest::setup(void)
{
    hal.scheduler->thread_create(
        FUNCTOR_BIND_MEMBER(&SemaphoreRecursionTest::worker, void),
        "semrec",
        2048,
        AP_HAL::Scheduler::PRIORITY_IO,
        0);

    ::printf("Starting recursive semaphore test at %lu ms\n", (unsigned long)AP_HAL::millis());

    expect_bool("main first take", mtx.take_nonblocking(), true);
    expect_bool("main recursive take", mtx.take_nonblocking(), true);

    worker_attempt("worker blocked while held twice", false);

    expect_bool("main first give", mtx.give(), true);
    worker_attempt("worker blocked while held once", false);

    expect_bool("main second give", mtx.give(), true);
    worker_attempt("worker acquires after full release", true);

    {
        WITH_SEMAPHORE(mtx);
        {
            WITH_SEMAPHORE(mtx);
            ::printf("nested WITH_SEMAPHORE: PASS\n");
        }
    }

    complete_ms = AP_HAL::millis();
    ::printf("Semaphore recursion result: %s at %lu ms\n",
             test_ok ? "PASS" : "FAIL",
             (unsigned long)complete_ms);
    complete = true;
}

static SemaphoreRecursionTest *test;

void setup(void)
{
    test = new SemaphoreRecursionTest;
    test->setup();
}

void loop(void)
{
    static uint32_t last_report_ms;

    hal.scheduler->delay(20);

    if (test && test->complete) {
        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_report_ms >= 2000U) {
            last_report_ms = now_ms;
            ::printf("Semaphore recursion result: %s complete=%lu now=%lu\n",
                     test->test_ok ? "PASS" : "FAIL",
                     (unsigned long)test->complete_ms,
                     (unsigned long)now_ms);
        }
    }
}

AP_HAL_MAIN();