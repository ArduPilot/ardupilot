/*
  Check that concurrent writes and flushes persist the final value.
  Use disposable storage: the test overwrites the first two storage lines.
  Build: ./waf --targets examples/StorageRace
  SITL: run /path/to/build/sitl/examples/StorageRace in a temporary directory.
  Linux: run /path/to/build/linux/examples/StorageRace --storage-directory DIR
*/
#include <AP_HAL/AP_HAL.h>
#include <atomic>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#if defined(__linux__)
#include <sys/stat.h>
#include <errno.h>
#endif

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
static constexpr unsigned num_rounds = 1000000;
static constexpr uint16_t storage_offset = 0;
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
static constexpr uint16_t second_storage_offset = 512;
#else
static constexpr uint16_t second_storage_offset = 8;
#endif
#if defined(__linux__)
// Intercept only this test's storage file, after initialization has finished.
static dev_t storage_device;
static ino_t storage_inode;
static std::atomic<bool> stall_armed{false}, io_entered{false}, io_release{false};
static std::atomic<bool> access_completed{false};
static bool fail_write;

static bool stall_storage_io(int fd)
{
    if (!stall_armed.load()) {
        return false;
    }
    struct stat st;
    if (fstat(fd, &st) != 0 || st.st_dev != storage_device || st.st_ino != storage_inode ||
        !stall_armed.exchange(false)) {
        return false;
    }
    io_entered.store(true);
    while (!io_release.load()) {
        usleep(1000);
    }
    if (fail_write) {
        errno = EIO;
        return true;
    }
    return false;
}

extern "C" ssize_t __real_pwrite(int fd, const void *data, size_t size, off_t offset);
extern "C" ssize_t __wrap_pwrite(int fd, const void *data, size_t size, off_t offset);
ssize_t __wrap_pwrite(int fd, const void *data, size_t size, off_t offset)
{
    return stall_storage_io(fd) ? -1 : __real_pwrite(fd, data, size, offset);
}

extern "C" ssize_t __real_write(int fd, const void *data, size_t size);
extern "C" ssize_t __wrap_write(int fd, const void *data, size_t size);
ssize_t __wrap_write(int fd, const void *data, size_t size)
{
    return stall_storage_io(fd) ? -1 : __real_write(fd, data, size);
}

static void *flush_once(void *)
{
    hal.storage->_timer_tick();
    return nullptr;
}

static void *access_during_io(void *)
{
    for (const uint16_t offset : {storage_offset, second_storage_offset}) {
        unsigned value;
        hal.storage->read_block(&value, offset, sizeof(value));
        if (value != 123) {
            exit(1);
        }
        if (!fail_write) {
            value = 456;
            hal.storage->write_block(offset, &value, sizeof(value));
        }
    }
    access_completed.store(true);
    return nullptr;
}

static bool wait_for(const std::atomic<bool> &flag)
{
    // Wall time, independent of simulated time and scheduler progress.
    for (unsigned i = 0; i < 2000; i++) {
        if (flag.load()) {
            return true;
        }
        usleep(1000);
    }
    return false;
}

static void check_access_during_io(int fd, bool inject_failure)
{
    struct stat st;
    if (fstat(fd, &st) != 0) {
        exit(2);
    }
    storage_device = st.st_dev;
    storage_inode = st.st_ino;
    for (const uint16_t offset : {storage_offset, second_storage_offset}) {
        unsigned value = 123;
        hal.storage->write_block(offset, &value, sizeof(value));
    }
    io_entered.store(false);
    io_release.store(false);
    access_completed.store(false);
    fail_write = inject_failure;
    stall_armed.store(true);
    pthread_t flusher, accessor;
    if (pthread_create(&flusher, nullptr, flush_once, nullptr) != 0 || !wait_for(io_entered)) {
        fprintf(stderr, "FAIL: storage IO hook did not trigger\n");
        exit(1);
    }
    if (pthread_create(&accessor, nullptr, access_during_io, nullptr) != 0) {
        exit(2);
    }
    const bool accessible = wait_for(access_completed);
    io_release.store(true);
    pthread_join(flusher, nullptr);
    pthread_join(accessor, nullptr);
    if (!accessible) {
        fprintf(stderr, "FAIL: buffer access blocked behind storage IO\n");
        exit(1);
    }
    hal.storage->_timer_tick();
    hal.storage->_timer_tick();
    for (const uint16_t offset : {storage_offset, second_storage_offset}) {
        unsigned value;
        const unsigned expected = inject_failure ? 123 : 456;
        if (pread(fd, &value, sizeof(value), offset) != sizeof(value) || value != expected) {
            fprintf(stderr, "FAIL: write during pending IO was lost\n");
            exit(1);
        }
    }
    printf("PASS: buffer access during %s IO, pending writes persisted\n",
           inject_failure ? "failed" : "successful");
}
#endif

static std::atomic<unsigned> requested{0}, completed{0};
static void *writer(void *)
{
    for (unsigned round = 1; round <= num_rounds; round++) {
        while (requested.load() != round) {
            sched_yield();
        }
        // Queue another line while the flusher updates the same dirty mask.
        hal.storage->write_block(second_storage_offset, &round, sizeof(round));
        hal.storage->write_block(storage_offset, &round, sizeof(round));
        completed.store(round);
    }
    return nullptr;
}
void setup();
void loop();
void setup()
{
    unsigned value;
    hal.storage->read_block(&value, storage_offset, sizeof(value));
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    const char *directory = hal.util->get_custom_storage_directory();
    if (directory == nullptr) {
        fprintf(stderr, "Use --storage-directory with a disposable directory\n");
        exit(2);
    }
    char *path = nullptr;
    // Examples link the common "ap" library built for the UNKNOWN vehicle.
    if (asprintf(&path, "%s/UNKNOWN.stg", directory) == -1) {
        exit(2);
    }
    int fd = open(path, O_RDONLY);
    free(path);
#else
    int fd = open("eeprom.bin", O_RDONLY);
#endif
    if (fd < 0) {
        perror("storage file");
        exit(2);
    }
#if defined(__linux__)
    check_access_during_io(fd, false);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // Linux intentionally closes its storage file on an IO failure.
    check_access_during_io(fd, true);
#endif
#endif
    pthread_t thread;
    if (pthread_create(&thread, nullptr, writer, nullptr) != 0) {
        exit(2);
    }
    for (unsigned round = 1; round <= num_rounds; round++) {
        // Ensure the line is already queued when the worker changes it again.
        unsigned previous = round + num_rounds;
        hal.storage->write_block(storage_offset, &previous, sizeof(previous));
        requested.store(round);
        do {
            hal.storage->_timer_tick();
            sched_yield();
        } while (completed.load() != round);
        // The first two lines are flushed before any later dirty lines.
        // The worker has completed its last write, so two ticks suffice.
        hal.storage->_timer_tick();
        hal.storage->_timer_tick();
        for (const uint16_t offset : {storage_offset, second_storage_offset}) {
            if (pread(fd, &value, sizeof(value), offset) != sizeof(value)) {
                exit(3);
            }
            if (value != round) {
                printf("LOST WRITE round=%u offset=%u disk=%u\n", round, offset, value);
                fflush(stdout);
                exit(1);
            }
        }
    }
    pthread_join(thread, nullptr);
    close(fd);
    printf("PASS: %u concurrent storage rounds persisted\n", num_rounds);
    exit(0);
}
void loop() {}
AP_HAL_MAIN();
