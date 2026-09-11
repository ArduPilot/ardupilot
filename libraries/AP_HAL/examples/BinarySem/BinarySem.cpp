/*
  test of HAL_BinarySemaphore
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>

#include <stdio.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class BinarySemTest {
public:
    HAL_BinarySemaphore sem1{1};
    HAL_BinarySemaphore sem2{0};

    void setup(void);
    void thread1(void);
    void thread2(void);
    void update(bool ok);

    uint32_t ops, timeouts;
    uint32_t last_print_us;
    HAL_Semaphore mtx;
};

void BinarySemTest::setup(void)
{
    hal.scheduler->thread_create(
        FUNCTOR_BIND_MEMBER(&BinarySemTest::thread1, void), "thd1", 2048, AP_HAL::Scheduler::PRIORITY_IO, 0);
    hal.scheduler->thread_create(
        FUNCTOR_BIND_MEMBER(&BinarySemTest::thread2, void), "thd2", 2048, AP_HAL::Scheduler::PRIORITY_IO, 0);
    ::printf("Setup threads\n");
}

void BinarySemTest::thread2(void)
{
    while (true) {
        bool ok = sem2.wait(50000);
        sem1.signal();
        update(ok);
    }
}

void BinarySemTest::thread1(void)
{
    while (true) {
        bool ok = sem1.wait(50000);
        sem2.signal();
        update(ok);
    }
}

void BinarySemTest::update(bool ok)
{
    WITH_SEMAPHORE(mtx);
    if (ok) {
        ops++;
    } else {
        timeouts++;
    }
    uint32_t now_us = AP_HAL::micros();
    float dt = (now_us - last_print_us)*1.0e-6;
    if (dt >= 1.0) {
        last_print_us = now_us;
        ::printf("tick %u %.3f ops/s %.3f timeouts/s\n",
                 unsigned(AP_HAL::millis()),
                 ops/dt,
                 timeouts/dt);
        ops = 0;
        timeouts = 0;
    }
}

static BinarySemTest *ct;

void setup(void)
{
    ct = new BinarySemTest;
    ct->setup();
}

void loop(void)
{
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
