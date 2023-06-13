/*
  test deadlock behaviour
*/

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <hal.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

extern uint32_t sem_error_count;

class DeadlockTest {
public:
    void setup();
    void loop();

private:
    uint32_t count1;
    uint32_t count2;

    void thread1(void);
    void thread2(void);

    mutex_t sem[4];
    uint32_t last_print_ms;
};

void DeadlockTest::thread1(void)
{
    while (true) {
        {
            chMtxLock(&sem[0]);
            chThdSleep(CH_CFG_ST_TIMEDELTA+1);
            chMtxLock(&sem[1]);
            chThdSleep(CH_CFG_ST_TIMEDELTA+1);
            chMtxLock(&sem[2]);
            chThdSleep(CH_CFG_ST_TIMEDELTA+1);
            chMtxLock(&sem[3]);

            chMtxUnlock(&sem[3]);
            chMtxUnlock(&sem[2]);
            chMtxUnlock(&sem[1]);
            chMtxUnlock(&sem[0]);
            count1++;
        }
        hal.scheduler->delay(1);
    }
}

void DeadlockTest::thread2(void)
{
    while (true) {
        {
            chMtxLock(&sem[3]);
            chThdSleep(CH_CFG_ST_TIMEDELTA+1);
            chMtxLock(&sem[0]);
            chThdSleep(CH_CFG_ST_TIMEDELTA+1);
            chMtxLock(&sem[1]);
            chThdSleep(CH_CFG_ST_TIMEDELTA+1);
            chMtxLock(&sem[2]);

            chMtxUnlock(&sem[2]);
            chMtxUnlock(&sem[1]);
            chMtxUnlock(&sem[0]);
            chMtxUnlock(&sem[3]);
            count2++;
        }
        hal.scheduler->delay(1);
    }
}

void DeadlockTest::setup(void)
{
    hal.console->begin(57600);

    for (auto &m : sem) {
        chMtxObjectInit(&m);
    }

    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&DeadlockTest::thread1, void),
                                 "THREAD1", 2560, AP_HAL::Scheduler::PRIORITY_IO, 0);
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&DeadlockTest::thread2, void),
                                 "THREAD2", 2560, AP_HAL::Scheduler::PRIORITY_IO, 0);
}

void DeadlockTest::loop(void)
{
    const uint32_t now = AP_HAL::millis();
    if (now - last_print_ms >= 1000U) {
        last_print_ms = now;
        hal.console->printf("count1=%u count2=%u err=%u\n", unsigned(count1), unsigned(count2), unsigned(sem_error_count));
    }
    hal.scheduler->delay(10);
}

static DeadlockTest test;

void setup() {
    test.setup();
}

void loop()
{
    test.loop();
}

AP_HAL_MAIN();
