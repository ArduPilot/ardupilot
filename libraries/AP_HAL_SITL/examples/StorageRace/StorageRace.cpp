/*
  Check that concurrent writes and flushes persist the final value.
  Run the SITL example in a disposable directory: it overwrites eeprom.bin.
  Build: ./waf --targets examples/StorageRace
  Run: /path/to/build/sitl/examples/StorageRace --model rover --serial0=null
*/
#include <AP_HAL/AP_HAL.h>
#include <atomic>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
static std::atomic<unsigned> requested{0}, completed{0};
static void *writer(void *)
{
    for (unsigned round = 1; round <= 100000; round++) {
        while (requested.load() != round) {}
        hal.storage->write_block(320, &round, sizeof(round));
        completed.store(round);
    }
    return nullptr;
}
void setup();
void loop();
void setup()
{
    unsigned value;
    hal.storage->read_block(&value, 320, sizeof(value));
    int fd = open("eeprom.bin", O_RDONLY);
    if (fd < 0) {
        perror("eeprom.bin");
        exit(2);
    }
    pthread_t thread;
    if (pthread_create(&thread, nullptr, writer, nullptr) != 0) {
        exit(2);
    }
    for (unsigned round = 1; round <= 100000; round++) {
        // Ensure the line is already queued when the worker changes it again.
        unsigned previous = round + 100000;
        hal.storage->write_block(320, &previous, sizeof(previous));
        requested.store(round);
        do {
            hal.storage->_timer_tick();
        } while (completed.load() != round);
        for (unsigned i = 0; i < 10; i++) {
            hal.storage->_timer_tick();
        }
        if (pread(fd, &value, sizeof(value), 320) != sizeof(value)) {
            exit(3);
        }
        if (value != round) {
            printf("LOST WRITE round=%u disk=%u\n", round, value);
            fflush(stdout);
            exit(1);
        }
    }
    pthread_join(thread, nullptr);
    close(fd);
    printf("PASS: 100000 concurrent storage writes persisted\n");
    exit(0);
}
void loop() {}
AP_HAL_MAIN();
