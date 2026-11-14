// SPDX-License-Identifier: GPL-3.0-or-later
// Exercise the actual manager using an emulator-owned SD card.
#include <AP_HAL/AP_HAL.h>
#include <AP_Module/Runtime.h>
#include <AP_Module/Bindings.h>
#include <AP_Module/BindingConfig.h>
#include <AP_Module/HostApi.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_Common/ExpandingString.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <atomic>
#include <stdio.h>
#include <stdlib.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

volatile uint32_t rust_runtime_test_result;
volatile uint32_t rust_runtime_test_cycles;
volatile uint32_t rust_runtime_test_before;
volatile uint32_t rust_runtime_test_after;
volatile uint32_t rust_runtime_test_empty_before;
volatile uint32_t rust_runtime_test_empty_after;
volatile uint32_t rust_runtime_test_generation_free[21];
volatile uint32_t rust_runtime_test_stack_free;
volatile uint32_t rust_runtime_test_stack_total;
volatile uint32_t rust_runtime_test_manager_priority;
volatile uint32_t rust_runtime_test_worker_priority;
volatile uint32_t rust_runtime_test_priority_max = 2;
volatile uint32_t rust_runtime_test_manager_stack;
volatile uint32_t rust_runtime_test_worker_stack;
volatile uint32_t rust_runtime_test_worker_stack_total;
volatile uint32_t rust_runtime_test_timeouts;
volatile uint32_t rust_runtime_test_timeout_elapsed;
volatile uint32_t rust_runtime_test_panics;
volatile uint32_t rust_runtime_test_overflows;
volatile uint32_t rust_runtime_test_loaded_checksum;
volatile uint32_t rust_runtime_test_running_checksum;
volatile uint32_t rust_runtime_test_romfs_rx;
volatile uint32_t rust_runtime_test_romfs_rw;
volatile uint32_t rust_runtime_test_sd_rx;
volatile uint32_t rust_runtime_test_sd_rw;
char rust_runtime_test_error[100];
char rust_runtime_test_memory[256];

#if AP_MODULE_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
class RuntimeGCS : public GCS_Dummy
{
public:
    std::atomic<uint32_t> loaded_reports{0};
    std::atomic<uint32_t> loaded_count{0};
    std::atomic<uint32_t> timeouts{0};
    std::atomic<uint32_t> panics{0};
    std::atomic<uint32_t> overflows{0};
    std::atomic<uint32_t> parameter_ready{0};
    std::atomic<uint32_t> parameter_restored{0};
    std::atomic<uint32_t> ram_reports{0};
    std::atomic<uint32_t> first_rx{0};
    std::atomic<uint32_t> first_rw{0};
    std::atomic<uint32_t> second_rx{0};
    std::atomic<uint32_t> second_rw{0};
    std::atomic<uint32_t> ignored_writable{0};

    void send_textv(MAV_SEVERITY severity, const char *format, va_list args,
                    mavlink_channel_mask_t destination) override
    {
        char message[160];
        vsnprintf(message, sizeof(message), format, args);
        if (severity <= MAV_SEVERITY_ERROR) {
            hal.util->snprintf(rust_runtime_test_error, sizeof(rust_runtime_test_error), "%s", message);
            ExpandingString memory(rust_runtime_test_memory, sizeof(rust_runtime_test_memory));
            hal.util->mem_info(memory);
        }
        constexpr char loaded_prefix[] = "Module: loaded ";
        constexpr size_t loaded_length = sizeof(loaded_prefix) - 1;
        if (strncmp(message, loaded_prefix, loaded_length) == 0 &&
            message[loaded_length] >= '0' && message[loaded_length] <= '9') {
            loaded_count.store(strtoul(message + loaded_length, nullptr, 10));
            loaded_reports.fetch_add(1);
        }
        if (strstr(message, "stop timeout") != nullptr) {
            timeouts.fetch_add(1);
        }
        if (strstr(message, "panic; reboot required") != nullptr) {
            panics.fetch_add(1);
        }
        if (strstr(message, "stack overflow") != nullptr) {
            overflows.fetch_add(1);
        }
        if (strstr(message, "PARAM ready") != nullptr) {
            parameter_ready.fetch_add(1);
        }
        if (strstr(message, "PARAM restored") != nullptr) {
            parameter_restored.fetch_add(1);
        }
        if (strstr(message, "ignored writable romfs.apmod; ROMFS wins") != nullptr) {
            ignored_writable.fetch_add(1);
        }
        constexpr char memory_prefix[] = "Module RAM: RX ";
        if (strncmp(message, memory_prefix, sizeof(memory_prefix) - 1) == 0) {
            char *separator;
            const uint32_t rx = strtoul(message + sizeof(memory_prefix) - 1, &separator, 10);
            const uint32_t rw = strncmp(separator, " RW ", 4) == 0 ?
                                strtoul(separator + 4, &separator, 10) : 0;
            if (rw != 0 && strncmp(separator, " stack ", 7) == 0) {
                const uint32_t report = ram_reports.fetch_add(1);
                if (report == 0) {
                    first_rx.store(rx);
                    first_rw.store(rw);
                } else if (report == 1) {
                    second_rx.store(rx);
                    second_rw.store(rw);
                }
            }
        }
    }
};

static RuntimeGCS test_gcs;
// Binding cleanup takes the vehicle scheduler semaphore even in this minimal
// harness, which otherwise has no AP_Vehicle to create the scheduler.
static AP_Scheduler vehicle_scheduler;
static AP_Module::Runtime runtime;
static const AP_Param::Info parameter_info[] = { AP_VAREND };
static AP_Param parameter_loader{parameter_info};

class RuntimeHarness
{
public:
    void run();

private:
    bool set(const char *name, float value)
    {
        return AP_Param::set_object_value(&runtime, AP_Module::Runtime::var_info, name, value);
    }
    bool wait_loaded(uint32_t previous, uint32_t count)
    {
        const uint32_t start = AP_HAL::millis();
        while (AP_HAL::millis() - start < 5000) {
            if (test_gcs.loaded_reports.load() != previous) {
                return test_gcs.loaded_count.load() == count;
            }
            hal.scheduler->delay(1);
        }
        return false;
    }
    bool restart(uint32_t count)
    {
        const uint32_t previous = test_gcs.loaded_reports.load();
        const uint32_t ready = test_gcs.parameter_ready.load();
        if (!set("RESTART", ++_request)) {
            return false;
        }
        runtime.update();
        if (!wait_loaded(previous, count)) {
            return false;
        }
        if (count == 2) {
            const uint32_t start = AP_HAL::millis();
            while (test_gcs.parameter_ready.load() == ready) {
                if (AP_HAL::millis() - start >= 5000) {
                    return false;
                }
                hal.scheduler->delay(1);
            }
        }
        return true;
    }
    bool exercise();
    bool exercise_overflow();
    bool exercise_romfs();
    bool exercise_many(uint8_t count);
    uint16_t _request{};
};

static RuntimeHarness harness;

static bool thread_values(const char *line, uint32_t &priority, uint32_t &free_stack,
                          uint32_t &total_stack, uint32_t &address)
{
    if (line == nullptr) {
        return false;
    }
    const char *end = strchr(line, '\n');
    const char *pri = strstr(line, "PRI=");
    const char *stack = strstr(line, "STACK=");
    const char *pointer = strstr(line, "sp=");
    if (end == nullptr || pri == nullptr || stack == nullptr || pointer == nullptr ||
        pri >= end || stack >= end || pointer >= end) {
        return false;
    }
    priority = strtoul(pri + 4, nullptr, 10);
    address = strtoul(pointer + 3, nullptr, 16);
    char *separator;
    free_stack = strtoul(stack + 6, &separator, 10);
    if (*separator != '/' || separator >= end) {
        return false;
    }
    total_stack = strtoul(separator + 1, nullptr, 10);
    return true;
}

static bool copy_file(const char *source, const char *destination)
{
    auto &fs = AP::FS();
    const int input = fs.open(source, O_RDONLY);
    if (input < 0) {
        return false;
    }
    const int output = fs.open(destination, O_WRONLY | O_CREAT | O_TRUNC);
    if (output < 0) {
        fs.close(input);
        return false;
    }
    bool success = true;
    uint8_t buffer[256];
    int32_t count;
    while ((count = fs.read(input, buffer, sizeof(buffer))) > 0) {
        if (fs.write(output, buffer, count) != count) {
            success = false;
            break;
        }
    }
    success = success && count == 0;
    success = fs.close(output) == 0 && success;
    success = fs.close(input) == 0 && success;
    if (!success) {
        fs.unlink(destination);
    }
    return success;
}

static int32_t runtime_int32(const char *name)
{
    for (uint8_t i = 0; AP_Module::Runtime::var_info[i].type != AP_PARAM_NONE; i++) {
        const auto &info = AP_Module::Runtime::var_info[i];
        if (info.type == AP_PARAM_INT32 && strcmp(info.name, name) == 0) {
            const uintptr_t address = reinterpret_cast<uintptr_t>(&runtime) + info.offset;
            auto *value = reinterpret_cast<AP_Int32 *>(address);
            return value->get();
        }
    }
    return INT32_MIN;
}

static bool ensure_directory(const char *path)
{
    auto &fs = AP::FS();
    if (fs.mkdir(path) == 0) {
        return true;
    }
    struct stat status;
    return fs.stat(path, &status) == 0 && S_ISDIR(status.st_mode);
}

static bool verify_module_report()
{
    auto &fs = AP::FS();
    const int fd = fs.open("@SYS/modules.txt", O_RDONLY);
    if (fd < 0) {
        return false;
    }
    char report[768];
    const int32_t count = fs.read(fd, report, sizeof(report) - 1);
    const bool closed = fs.close(fd) == 0;
    if (!closed || count <= 0 || count >= int32_t(sizeof(report) - 1)) {
        return false;
    }
    report[count] = '\0';
    if (strncmp(report, "ModulesV1\n", 10) != 0) {
        return false;
    }
    char *line = report + 10;
    for (const char *name : {"one.apmod ", "two.apmod "}) {
        char *end = strchr(line, '\n');
        if (end == nullptr || strncmp(line, name, strlen(name)) != 0) {
            return false;
        }
        *end = '\0';
        const char *heap = strstr(line, " HEAP=");
        if (heap == nullptr || strstr(line, "STATE=RUN ") == nullptr ||
            strstr(line, " SRC=") == nullptr || strstr(line, " STK=") == nullptr) {
            return false;
        }
        char *separator;
        const uint32_t free_bytes = strtoul(heap + 6, &separator, 10);
        if (*separator != '/') {
            return false;
        }
        const uint32_t capacity = strtoul(separator + 1, nullptr, 10);
        if (capacity != 4096 || free_bytes > capacity ||
            strstr(line, " USED=") != nullptr || strstr(line, " FREE=") != nullptr) {
            return false;
        }
#if HAL_ENABLE_THREAD_STATISTICS
        if (strstr(line, " LOAD=") == nullptr || strstr(line, "LOAD=?") != nullptr || strchr(line, '%') == nullptr) {
            return false;
        }
#else
        if (strstr(line, " LOAD=") != nullptr) {
            return false;
        }
#endif
        line = end + 1;
    }
    return *line == '\0';
}

static void romfs_stage(uint32_t stage)
{
    rust_runtime_test_result = stage;
}

bool RuntimeHarness::exercise_romfs()
{
    romfs_stage(20);
    if (!ensure_directory("/APM") || !ensure_directory("/APM/modules")) {
        return false;
    }
    romfs_stage(21);
    if (!copy_file("@ROMFS/modules/romfs.apmod", "/fixtures/readback.apmod") ||
        !copy_file("/fixtures/sd.apmod", "/APM/modules/sd.apmod") ||
        !copy_file("/fixtures/shadow.apmod", "/APM/modules/romfs.apmod")) {
        return false;
    }
    romfs_stage(22);
    if (!set("ENABLE", 1) || !set("HEAP_MAX", 4096) || !set("CODE_MAXSIZE", 65535) ||
        !set("STACK_MAX", 16385)) {
        return false;
    }
    romfs_stage(23);
    runtime.init();
    if (!wait_loaded(0, 2)) {
        return false;
    }
    romfs_stage(24);
    const uint32_t ready_start = AP_HAL::millis();
    while (test_gcs.parameter_ready.load() == 0) {
        if (AP_HAL::millis() - ready_start > 5000) {
            return false;
        }
        hal.scheduler->delay(1);
    }
    rust_runtime_test_romfs_rx = test_gcs.first_rx.load();
    rust_runtime_test_romfs_rw = test_gcs.first_rw.load();
    rust_runtime_test_sd_rx = test_gcs.second_rx.load();
    rust_runtime_test_sd_rw = test_gcs.second_rw.load();
    if (test_gcs.ram_reports.load() < 2 || rust_runtime_test_romfs_rx != 0 || rust_runtime_test_romfs_rw == 0 ||
        rust_runtime_test_sd_rx == 0 || rust_runtime_test_sd_rw == 0) {
        return false;
    }

    if (!set("DEBUG_OPTS", 1U << 5)) {
        return false;
    }
    runtime.update();
    rust_runtime_test_loaded_checksum = runtime_int32("LD_CSUM");
    rust_runtime_test_running_checksum = runtime_int32("RUN_CSUM");
    const uint32_t expected_checksum = rust_runtime_test_loaded_checksum;
    char message[100];
    if (expected_checksum == 0 || rust_runtime_test_running_checksum != expected_checksum ||
        !runtime.arming_checks(sizeof(message), message)) {
        return false;
    }
    if (!set("LD_CSUM", expected_checksum ^ 1U) ||
        runtime.arming_checks(sizeof(message), message) || strstr(message, "loaded checksum mismatch") == nullptr ||
        !set("LD_CSUM", expected_checksum)) {
        return false;
    }

    if (!restart(2)) {
        return false;
    }
    hal.scheduler->delay(100);
    rust_runtime_test_before = hal.util->available_memory();
    for (unsigned cycle = 0; cycle < 10; cycle++) {
        if (!restart(2)) {
            return false;
        }
        hal.scheduler->delay(100);
        rust_runtime_test_cycles = rust_runtime_test_cycles + 1;
    }
    rust_runtime_test_after = hal.util->available_memory();
    ExpandingString threads;
    hal.util->thread_info(threads);
    const char *all_threads = threads.get_string();
    const char *manager = all_threads == nullptr ? nullptr : strstr(all_threads, "ModMgr");
    uint32_t priority, free_stack, total_stack, address;
    if (!thread_values(manager, priority, free_stack, total_stack, address)) {
        return false;
    }
    rust_runtime_test_stack_free = free_stack;
    rust_runtime_test_stack_total = total_stack;
    return rust_runtime_test_before == rust_runtime_test_after &&
           test_gcs.parameter_ready.load() == rust_runtime_test_cycles + 2 &&
           test_gcs.ignored_writable.load() == rust_runtime_test_cycles + 2 &&
           runtime.arming_checks(sizeof(message), message) && free_stack > 1024;
}

bool RuntimeHarness::exercise_many(uint8_t count)
{
    if (count == 0 || !ensure_directory("/APM") || !ensure_directory("/APM/modules") ||
        !ensure_directory("/fixtures/many")) {
        return false;
    }
    runtime.init();
    if (!wait_loaded(0, 0)) {
        return false;
    }
    hal.scheduler->delay(100);
    if (AP::FS().unlink("/APM/modules") != 0) {
        return false;
    }
    // Reverse order exercises sorted insertion without moving callback owners.
    for (uint16_t i = count; i > 0; i--) {
        char destination[48];
        hal.util->snprintf(destination, sizeof(destination), "/fixtures/many/m%03u.apmod", unsigned(i - 1));
        if (!copy_file("/fixtures/many.apmod", destination)) {
            return false;
        }
    }
    const auto move_files = [](bool install) {
        // Move the directory, avoiding count-dependent FAT metadata IO on
        // every cycle. A missing writable directory is a valid empty generation.
        const char *active = "/APM/modules";
        const char *parked = "/fixtures/many";
        return AP::FS().rename(install ? parked : active, install ? active : parked) == 0;
    };
    // Warm the complete load/unload path: the default ChibiOS heap grows
    // provider slabs on first use. Require exact recovery of both loaded and
    // empty baselines once that allocator bookkeeping has settled.
    for (uint8_t warmup = 0; warmup < 4; warmup++) {
        if (!move_files(true) || !restart(count)) {
            return false;
        }
        hal.scheduler->delay(100);
        if (!move_files(false) || !restart(0)) {
            return false;
        }
        hal.scheduler->delay(100);
    }
    rust_runtime_test_empty_before = hal.util->available_memory();
    if (!move_files(true) || !restart(count)) {
        return false;
    }
    hal.scheduler->delay(100);
    rust_runtime_test_before = hal.util->available_memory();
    rust_runtime_test_generation_free[0] = rust_runtime_test_before;
    char message[100];
    for (uint8_t cycle = 0; cycle < 20; cycle++) {
        if (!restart(count)) {
            return false;
        }
        hal.scheduler->delay(100);
        rust_runtime_test_generation_free[cycle + 1] = hal.util->available_memory();
        rust_runtime_test_after = hal.util->available_memory();
        if (!runtime.arming_checks(sizeof(message), message) || !move_files(false) || !restart(0)) {
            return false;
        }
        hal.scheduler->delay(100);
        rust_runtime_test_empty_after = hal.util->available_memory();
        if (rust_runtime_test_empty_before != rust_runtime_test_empty_after ||
            (cycle < 19 && !move_files(true))) {
            return false;
        }
        rust_runtime_test_cycles = rust_runtime_test_cycles + 1;
    }
    if (!runtime.arming_checks(sizeof(message), message) ||
        rust_runtime_test_before != rust_runtime_test_after) {
        return false;
    }
    ExpandingString threads;
    hal.util->thread_info(threads);
    const char *all_threads = threads.get_string();
    uint32_t priority, address, free_stack, total_stack;
    if (!thread_values(all_threads == nullptr ? nullptr : strstr(all_threads, "ModMgr"),
                       priority, free_stack, total_stack, address)) {
        return false;
    }
    rust_runtime_test_stack_free = free_stack;
    rust_runtime_test_stack_total = total_stack;
    return free_stack > 1024;
}

bool RuntimeHarness::exercise()
{
    // A wire pointer with a nonzero high word must not truncate to a valid
    // STM32 address. No C++ object is constructed on a rejected receiver.
    uint8_t input[25] {};
    input[4] = 1;
    input[8] = 'A';
    uint8_t output = 0xFF;
    if (AP_Module::binding_dispatch(nullptr, static_cast<AP_Module::BindingId>(AP_MODULE_BINDING_PARAMETER_INIT),
                                  input, sizeof(input), &output, sizeof(output)) != AP_MODULE_STATUS_INVALID || output != 0) {
        return false;
    }
    for (unsigned retry = 0; !AP::FS().retry_mount(); retry++) {
        if (retry == 5) {
            return false;
        }
        hal.scheduler->delay(100);
    }
    rust_runtime_test_result = 2;
    // Optional emulator fixture; read after startup rather than patching a
    // RAM initializer which the C runtime would overwrite during boot.
    const int priority_file = AP::FS().open("/fixtures/priority.bin", O_RDONLY);
    if (priority_file >= 0) {
        uint8_t priority = 0;
        const auto count = AP::FS().read(priority_file, &priority, sizeof(priority));
        AP::FS().close(priority_file);
        if (count != sizeof(priority)) {
            return false;
        }
        rust_runtime_test_priority_max = priority;
    }
    // Odd byte counts exercise image-budget and HAL stack rounding.
    if (!set("ENABLE", 1) || !set("HEAP_MAX", 4096) || !set("CODE_MAXSIZE", 65535) || !set("STACK_MAX", 16385) ||
        !set("THD_PRIO_MAX", rust_runtime_test_priority_max)) {
        return false;
    }
    const int many_file = AP::FS().open("/fixtures/many.count", O_RDONLY);
    if (many_file >= 0) {
        uint8_t count = 0;
        const auto length = AP::FS().read(many_file, &count, sizeof(count));
        AP::FS().close(many_file);
        return length == sizeof(count) && exercise_many(count);
    }
    struct stat embedded_stat;
    if (AP::FS().stat("@ROMFS/modules/romfs.apmod", &embedded_stat) == 0) {
        return exercise_romfs();
    }
    runtime.init();
    char message[100];
    if (!wait_loaded(0, 0) || !runtime.arming_checks(sizeof(message), message)) {
        return false;
    }
    rust_runtime_test_result = 3;
    auto &fs = AP::FS();
    if ((fs.mkdir("/APM") != 0 && errno != EEXIST) ||
        (fs.mkdir("/APM/modules") != 0 && errno != EEXIST)) {
        return false;
    }
    struct stat overflow_stat;
    if (fs.stat("/fixtures/overflow.apmod", &overflow_stat) != 0) {
        struct stat romfs_stat;
        if (fs.stat("@ROMFS/overflow.apmod", &romfs_stat) == 0 &&
            ((fs.mkdir("/fixtures") != 0 && errno != EEXIST) ||
             !copy_file("@ROMFS/overflow.apmod", "/fixtures/overflow.apmod"))) {
            return false;
        }
    }
    if (fs.stat("/fixtures/overflow.apmod", &overflow_stat) == 0) {
        return exercise_overflow();
    }
    rust_runtime_test_result = 30;
    if (fs.rename("/fixtures/one.apmod", "/APM/modules/one.apmod") != 0 ||
        fs.rename("/fixtures/two.apmod", "/APM/modules/two.apmod") != 0) {
        return false;
    }
    rust_runtime_test_result = 31;
    // The larger module metadata/header changes allocator placement. Three
    // complete generations settle the observed one-time 8-byte free-list
    // difference; every subsequent generation must match the exact baseline.
    constexpr uint8_t warmup_generations = 3;
    for (uint8_t generation = 0; generation < warmup_generations; generation++) {
        if (!restart(2) || !verify_module_report()) {
            return false;
        }
    }
    hal.scheduler->delay(100);
    rust_runtime_test_before = hal.util->available_memory();
    rust_runtime_test_generation_free[0] = rust_runtime_test_before;
    for (unsigned cycle = 0; cycle < 20; cycle++) {
        rust_runtime_test_result = 4;
        if (!restart(2) || !verify_module_report()) {
            return false;
        }
        hal.scheduler->delay(100);
        rust_runtime_test_cycles = rust_runtime_test_cycles + 1;
        rust_runtime_test_generation_free[cycle + 1] = hal.util->available_memory();
    }
    rust_runtime_test_after = hal.util->available_memory();
    if (rust_runtime_test_before != rust_runtime_test_after || !runtime.arming_checks(sizeof(message), message) ||
        test_gcs.parameter_restored.load() != rust_runtime_test_cycles + warmup_generations - 1) {
        return false;
    }
    rust_runtime_test_result = 5;
    if (fs.unlink("/APM/modules/one.apmod") != 0 || fs.unlink("/APM/modules/two.apmod") != 0 ||
        fs.rename("/fixtures/end.apmod", "/APM/modules/one.apmod") != 0 || !restart(1)) {
        return false;
    }
    hal.scheduler->delay(100);
    const uint32_t loaded = test_gcs.loaded_reports.load();
    const uint32_t retained = hal.util->available_memory();
    if (!set("RESTART", ++_request)) {
        return false;
    }
    runtime.update();
    const uint32_t start = AP_HAL::millis();
    while (test_gcs.timeouts.load() == 0 && AP_HAL::millis() - start < 12000) {
        hal.scheduler->delay(10);
    }
    rust_runtime_test_timeout_elapsed = AP_HAL::millis() - start;
    rust_runtime_test_timeouts = test_gcs.timeouts.load();
    rust_runtime_test_panics = test_gcs.panics.load();
    rust_runtime_test_result = 6;
    if (rust_runtime_test_timeouts != 1 || test_gcs.loaded_reports.load() != loaded ||
        runtime.arming_checks(sizeof(message), message) || hal.util->available_memory() != retained) {
        return false;
    }
    if (rust_runtime_test_panics != 0 && strstr(message, "one.apmod panic") == nullptr) {
        return false; // preserve the original module failure across stop timeout
    }
    // Read the HAL's real thread registry; no private Runtime test access.
    ExpandingString threads;
    hal.util->thread_info(threads);
    const char *all_threads = threads.get_string();
    const char *manager = all_threads == nullptr ? nullptr : strstr(all_threads, "ModMgr");
    const char *worker = all_threads == nullptr ? nullptr : strstr(all_threads, "one.apmod");
    uint32_t manager_priority, worker_priority, free_stack, total_stack, worker_free, worker_total;
    uint32_t manager_address, worker_address;
    rust_runtime_test_result = 7;
    if (!thread_values(manager, manager_priority, free_stack, total_stack, manager_address) ||
        !thread_values(worker, worker_priority, worker_free, worker_total, worker_address)) {
        return false;
    }
    rust_runtime_test_stack_free = free_stack;
    rust_runtime_test_stack_total = total_stack;
    rust_runtime_test_manager_priority = manager_priority;
    rust_runtime_test_worker_priority = worker_priority;
    rust_runtime_test_manager_stack = manager_address;
    rust_runtime_test_worker_stack = worker_address;
    rust_runtime_test_worker_stack_total = worker_total;
    return manager_priority == rust_runtime_test_priority_max + 1 &&
           worker_priority <= rust_runtime_test_priority_max && free_stack > 1024;
}

bool RuntimeHarness::exercise_overflow()
{
    auto &fs = AP::FS();
    char message[100];
    // Warm up twice: the default ChibiOS heap may need a second provider slab
    // before it can reuse freed blocks without growing. Then require exact
    // release of image, stack and live heap allocations for 100 generations.
    // An overflow latches pre-arm failure; a deliberate restart with the faulty
    // image removed clears it.
    for (unsigned cycle = 0; cycle <= 101; cycle++) {
        rust_runtime_test_result = 80;
        const uint32_t previous = test_gcs.overflows.load();
        if (fs.rename("/fixtures/overflow.apmod", "/APM/modules/overflow.apmod") != 0 || !restart(1)) {
            return false;
        }
        const uint32_t start = AP_HAL::millis();
        while (test_gcs.overflows.load() == previous) {
            if (AP_HAL::millis() - start > 5000) {
                rust_runtime_test_result = 81;
                return false;
            }
            hal.scheduler->delay(1);
        }
        if (runtime.arming_checks(sizeof(message), message) || strstr(message, "stack overflow") == nullptr) {
            rust_runtime_test_result = 82;
            return false;
        }
        if (fs.rename("/APM/modules/overflow.apmod", "/fixtures/overflow.apmod") != 0 || !restart(0) ||
            !runtime.arming_checks(sizeof(message), message)) {
            rust_runtime_test_result = 83;
            return false;
        }
        hal.scheduler->delay(20);
        if (cycle == 1) {
            rust_runtime_test_before = hal.util->available_memory();
        } else if (cycle > 1) {
            rust_runtime_test_cycles = cycle - 1;
        }
    }
    rust_runtime_test_overflows = test_gcs.overflows.load();
    rust_runtime_test_after = hal.util->available_memory();
    return rust_runtime_test_before == rust_runtime_test_after;
}

void RuntimeHarness::run()
{
    if (exercise()) {
        rust_runtime_test_result = 0x600DF00D;
    }
    while (true) {
        hal.scheduler->delay(1000);
    }
}
#endif

void setup();
void loop();

void setup()
{
    rust_runtime_test_result = 1;
#if AP_MODULE_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    AP_Param::setup();
    AP_Param::load_all();
    // Test driver only: it sleeps while waiting for the lower-priority manager.
    if (!hal.scheduler->thread_create(FUNCTOR_BIND(&harness, &RuntimeHarness::run, void), "RustTest", 8192,
                                      AP_HAL::Scheduler::PRIORITY_SCRIPTING, 8)) {
        rust_runtime_test_result = 0xBAD0;
    }
#else
    rust_runtime_test_result = 0xBAD1;
#endif
}

void loop()
{
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
