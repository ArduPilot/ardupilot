/*
  simple test of UART interfaces
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/EventHandle.h>

#include <stdio.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class EventTest {
public:
    HAL_EventSource src;
    HAL_EventHandle handle;

    void setup(void);
    void loop(void);
    void thread1(void);
    void thread2(void);

    HAL_Semaphore sem;
    uint32_t ops, timeouts;
    uint32_t last_print_us;
};

void EventTest::setup(void)
{
    handle.register_event(1);
    handle.set_source(&src);

    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&EventTest::thread1, void), "thd1", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0);
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&EventTest::thread2, void), "thd2", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0);
    ::printf("Setup event threads\n");
}

void EventTest::thread2(void)
{
    while (true) {
        auto ret = handle.wait(0xFFFFU);
        WITH_SEMAPHORE(sem);
        if (ret) {
            ops++;
        } else {
            timeouts++;
        }
    }
}

void EventTest::thread1(void)
{
    while (true) {
        hal.scheduler->delay_microseconds(100);
        // change the signal to send in this line to test timeout
        // handling and correct differentiation of events
        src.signal(1);
    }
}

void EventTest::loop(void)
{
    uint32_t now_us = AP_HAL::micros();
    float dt = (now_us - last_print_us)*1.0e-6;
    last_print_us = now_us;
    WITH_SEMAPHORE(sem);
    ::printf("tick %u %.3f ops/s %.3f timeouts/s\n",
             unsigned(AP_HAL::millis()),
             ops/dt,
             timeouts/dt);
    ops = 0;
    timeouts = 0;
}

static EventTest *et;

void setup(void)
{
    et = new EventTest;
    et->setup();
}

void loop(void)
{
    et->loop();
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
