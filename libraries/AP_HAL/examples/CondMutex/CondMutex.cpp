/*
  test of HAL_BinarySemaphore
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Math/AP_Math.h>

#include <stdio.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class ProducerConsumerTest {
public:
    HAL_BinarySemaphore sem1;

    void setup(void);
    void producer(void);
    void consumer(void);
    void producer_tick(void);
    void consumer_tick(void);
    bool update();
    void update_sent();
    bool check() { return bsize>0; }

    uint32_t ops, timeouts, sent, recv;
    uint32_t bsize;
    uint32_t last_print_us;
    uint32_t last_sent_us;
    uint32_t max_delayed_us;
    HAL_Semaphore mtx;
    uint32_t delay_count;
};

void ProducerConsumerTest::setup(void)
{
    hal.scheduler->thread_create(
        FUNCTOR_BIND_MEMBER(&ProducerConsumerTest::producer, void), "producer", 2048, AP_HAL::Scheduler::PRIORITY_IO, 0);
    hal.scheduler->thread_create(
        FUNCTOR_BIND_MEMBER(&ProducerConsumerTest::consumer, void), "consumer", 2048, AP_HAL::Scheduler::PRIORITY_IO, 1);
    ::printf("Setup threads\n");
}

void ProducerConsumerTest::consumer(void)
{
    while (true) {
        consumer_tick();
    }
}

void ProducerConsumerTest::consumer_tick(void)
{
    if (!check()) {
        sem1.wait_blocking();
    }
    // we are avoiding wait_blocking() here to cope with double notifications
    // however it also means that, pre-existing notifications will not
    // be waited for, this means that we can exhaust the pending data
    // and the wait_blocking() above will immediately return. This is why
    // the availability of data must also be checked inside the lock
    // it also means you have to go around the loop twice to get to a blocking state
    // when going from some data to no data
    if (!update()) {
        // we thought we had a sample, but concurrency means we actually do not
        // the pattern requires that we are able to exit early here without processing
        // it should only ever happen two cycles at a time so is not a busy wait
        return; 
    }
    hal.scheduler->delay_microseconds(100); // simluate processing delay
}

void ProducerConsumerTest::producer(void)
{
    while (true) {
        // simulate fifo burst
        producer_tick();
        producer_tick();
        producer_tick();
        hal.scheduler->delay_microseconds(750);
    }
}

void ProducerConsumerTest::producer_tick(void)
{
    update_sent();
    sem1.signal();
}

bool ProducerConsumerTest::update()
{
    WITH_SEMAPHORE(mtx);
    // see the comment in consumer_tick() as to why this is necessary
    if (!check()) {
        return false;
    }
    ops++;
    recv++;
    bsize--;
    uint32_t now_us = AP_HAL::micros();
    max_delayed_us = MAX(max_delayed_us, now_us - last_sent_us);
    float dt = (now_us - last_print_us)*1.0e-6;
    if (dt >= 1.0) {
        last_print_us = now_us;
        ::printf("tick %u %.3f ops/s, dt %uus missing %d max_delay %uus queue length %u\n",
                 unsigned(AP_HAL::millis()),
                 ops/dt,
                 uint32_t(dt * 1.0e6 / ops),
                 int32_t(sent) - int32_t(recv),
                 max_delayed_us,
                 bsize);
        ops = 0;
        max_delayed_us = 0;
    }
    return true;
}

void ProducerConsumerTest::update_sent()
{
    WITH_SEMAPHORE(mtx);
    sent++;
    bsize++;
    last_sent_us = AP_HAL::micros();
}

static ProducerConsumerTest *ct;

void setup(void)
{
    ct = new ProducerConsumerTest;
    ct->setup();
}

void loop(void)
{
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
