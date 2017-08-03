#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

#include <AP_HAL/utility/RingBuffer.h>

using namespace std;

struct stress_pairs {
    uint8_t pair_id;
    uint8_t reader_buf[32];
    ByteBuffer buf{128};
    thread consumer;
    thread producer;
};

//This buffer is shared among producers.
static uint8_t writer_buf[256];

mutex error_mtx;
condition_variable error_cond;

static void consumer_thread(struct stress_pairs *pair) {
    uint32_t ret;
    uint8_t last = 0;

    for (;;) {
        ret = pair->buf.read(pair->reader_buf, sizeof(pair->reader_buf));

        for (uint32_t i = 0; i < ret; i++) {
            if (pair->reader_buf[i] != last) {
                fprintf(stderr, "[id=%u read=%u] Expected=%u Got=%u\n",
                       pair->pair_id, ret, last, pair->reader_buf[i]);
                unique_lock<mutex> lck(error_mtx);
                error_cond.notify_all();
            }
            last++;
            last %= sizeof(writer_buf);
        }
    }
}

static void producer_thread(struct stress_pairs *pair) {
    uint8_t next = 0;

    for (;;) {
        // Overflow will do the magic
        next += pair->buf.write(writer_buf + next, sizeof(writer_buf) - next);
        next %= sizeof(writer_buf);
    }
}

static void
usage(const char *program)
{
    cout << "Usage: " << program << " [pairs]\n"
        "   pair - number of <producer,consumer> pairs that should be created. [Default=1]\n";
}

int main(int argc, char const **argv) {
    unsigned int i, pairs = 1;
    struct stress_pairs *list;

    if (argc > 1 && (!sscanf(argv[1], "%u", &pairs) || !pairs)) {
        usage(argv[0]);
        return EXIT_SUCCESS;
    }

    for (i = 0; i < sizeof(writer_buf); i++)
        writer_buf[i] = i;

    cout << "Hello Threaded World!\n";
    unique_lock<mutex> lck(error_mtx);

    list = new struct stress_pairs[pairs];
    for (i = 0; i < pairs; i++) {
        cout << "Launching pair "<< i << "... ";
        list[i].pair_id = i;
        list[i].consumer = thread(consumer_thread, list + i);
        list[i].producer = thread(producer_thread, list + i);
        cout << "done!" << endl;
    }

    error_cond.wait(lck);

    // Let the OS do all the cleaning
    cout << "Aborting: Good bye **failure** World!\n";
    return EXIT_FAILURE;
}

#else

#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static void loop() { }
static void setup()
{
    printf("Board not currently supported\n");
}

AP_HAL_MAIN();

#endif
