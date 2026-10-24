// SPDX-License-Identifier: GPL-3.0-or-later
// SD-loaded SDK value math on a real HAL worker, without a vehicle instance.
#include <AP_HAL/AP_HAL.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_Module/Bindings.h>
#include <AP_Module/Module.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_Notify/AP_Notify_config.h>
#include <AP_Notify/ScriptingLED.h>
#include <AP_Common/ExpandingString.h>
#include <AP_Mount/AP_Mount.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Logger/AP_Logger.h>
#include <fcntl.h>
#include <memory>
#include <stdlib.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#ifndef AP_MODULE_MATH_HEAP_SIZE
#define AP_MODULE_MATH_HEAP_SIZE 4096
#endif
#ifndef AP_MODULE_MATH_CODE_SIZE
#define AP_MODULE_MATH_CODE_SIZE 65536
#endif
#ifndef AP_MODULE_MATH_TIMEOUT_MS
#define AP_MODULE_MATH_TIMEOUT_MS 5000
#endif
#ifndef AP_MODULE_MATH_CYCLES
#define AP_MODULE_MATH_CYCLES 20
#endif

volatile uint32_t rust_math_test_result;
volatile uint32_t rust_math_test_cycles;
volatile uint32_t rust_math_test_before;
volatile uint32_t rust_math_test_after;
volatile uint32_t rust_math_test_heap_peak;
volatile int32_t rust_math_test_status;
volatile uint32_t rust_math_test_stack_used;
volatile uint32_t rust_math_test_stack_total;
volatile uint32_t rust_math_test_warmup_before;
volatile uint32_t rust_math_test_heap_limit;
volatile uint32_t rust_math_test_code_limit;
volatile uint32_t rust_math_test_cycle_limit;
volatile uint32_t rust_math_test_timeout_ms;
char rust_math_test_error[96];

#if AP_MODULE_ENABLED && HAL_OWNED_THREADS_ENABLED && HAL_EXECUTABLE_MEMORY_ENABLED
using AP_Module::Module;
#if HAL_LOGGING_ENABLED
// Registry/record assembly without a logging backend; SITL checks disk records.
static AP_Logger logger;
#endif
#if AP_RTC_ENABLED
static AP_RTC rtc;
#endif
#if AP_NOTIFY_SCRIPTING_LED_ENABLED
static ScriptingLED led;
#endif
#if HAL_MOUNT_ENABLED
static AP_Mount mount;
#endif
#if AP_RC_CHANNEL_ENABLED && AP_SCHEDULER_ENABLED
static AP_Scheduler scheduler;
class HarnessRCChannels : public RC_Channels
{
public:
    RC_Channel obj_channels[NUM_RC_CHANNELS];
    RC_Channel *channel(uint8_t index) override
    {
        return index < NUM_RC_CHANNELS ? &obj_channels[index] : nullptr;
    }
    const RC_Channel *channel(uint8_t index) const override
    {
        return index < NUM_RC_CHANNELS ? &obj_channels[index] : nullptr;
    }
    int8_t flight_mode_channel_number() const override
    {
        return 0;
    }
};
#define RC_CHANNELS_SUBCLASS HarnessRCChannels
#define RC_CHANNEL_SUBCLASS RC_Channel
#include <RC_Channel/RC_Channels_VarInfo.h>
#undef RC_CHANNELS_SUBCLASS
#undef RC_CHANNEL_SUBCLASS
static HarnessRCChannels rc_channels;
#endif

class MathHarness
{
public:
    void run()
    {
        rust_math_test_heap_limit = AP_MODULE_MATH_HEAP_SIZE;
        rust_math_test_code_limit = AP_MODULE_MATH_CODE_SIZE;
        rust_math_test_cycle_limit = AP_MODULE_MATH_CYCLES;
        rust_math_test_timeout_ms = AP_MODULE_MATH_TIMEOUT_MS;
#if AP_RC_CHANNEL_ENABLED && AP_SCHEDULER_ENABLED
        // Real RC methods with a fixed input, deliberately without init()'s
        // ch_in assignment: handle encoding must use pointer identity.
        rc_channels.channel(0)->set_radio_in(1600);
#endif
        const bool mounted = AP::FS().retry_mount();
        rust_math_test_warmup_before = hal.util->available_memory();
        if (mounted && cycle()) {
            rust_math_test_before = hal.util->available_memory();
            for (unsigned i = 0; i < AP_MODULE_MATH_CYCLES && cycle(); i++) {
                rust_math_test_cycles = rust_math_test_cycles + 1;
            }
            rust_math_test_after = hal.util->available_memory();
            if (rust_math_test_cycles == AP_MODULE_MATH_CYCLES && rust_math_test_before == rust_math_test_after) {
                rust_math_test_result = 0x600DF00D;
            }
        }
        // A failed test may leave a worker. Its callback context and module
        // remain alive until the emulator tears down the whole machine.
        while (true) {
            hal.scheduler->delay(1000);
        }
    }

private:
    static void text(void *context, uint8_t severity, const char *message, uint32_t length)
    {
        auto &self = *static_cast<MathHarness *>(context);
        if (severity <= 3 || (length >= 7 && memcmp(message, "oracle ", 7) == 0)) {
            const uint32_t n = MIN(length, sizeof(rust_math_test_error) - 1);
            memcpy(rust_math_test_error, message, n);
            rust_math_test_error[n] = 0;
        }
        if (severity == 6 && length == 7 && memcmp(message, "MATH ok", 7) == 0) {
            self._messages.fetch_add(1);
            // Keep the worker alive while the manager samples its high-water
            // mark. Do not allocate the thread-info buffer on the worker.
            while (!self._sampled.load()) {
                hal.scheduler->delay(1);
            }
        }
    }

    bool sample_stack()
    {
        ExpandingString threads;
        hal.util->thread_info(threads);
        const char *all = threads.get_string();
        // Include padding to distinguish the worker from rust-math-manager.
        const char *worker = all == nullptr ? nullptr : strstr(all, "rust-math ");
        const char *end = worker == nullptr ? nullptr : strchr(worker, '\n');
        const char *stack = worker == nullptr ? nullptr : strstr(worker, "STACK=");
        if (end == nullptr || stack == nullptr || stack >= end) {
            return false;
        }
        char *separator;
        const uint32_t free = strtoul(stack + 6, &separator, 10);
        if (separator >= end || *separator != '/') {
            return false;
        }
        const uint32_t total = strtoul(separator + 1, nullptr, 10);
        if (free > total || free < 4096) {
            return false;
        }
        rust_math_test_stack_total = total;
        if (total - free > rust_math_test_stack_used) {
            rust_math_test_stack_used = total - free;
        }
        return true;
    }

    bool load()
    {
        auto &fs = AP::FS();
        const int fd = fs.open("/APM/modules/math.apmod", O_RDONLY);
        if (fd < 0) {
            rust_math_test_result = 5;
            return false;
        }
        const int32_t length = fs.lseek(fd, 0, SEEK_END);
        const uint32_t size = length > 0 ? uint32_t(length) : 0;
        const auto free_input = [size](uint8_t *buffer) {
            hal.util->free_type(buffer, size, AP_HAL::Util::MEM_FAST);
        };
        std::unique_ptr<uint8_t, decltype(free_input)> input(nullptr, free_input);
        // Match the runtime: loading scratch must not crowd code out of F405 SRAM.
        if (size > 0 && size <= AP_MODULE_MATH_CODE_SIZE && fs.lseek(fd, 0, SEEK_SET) == 0) {
            input.reset(static_cast<uint8_t *>(hal.util->malloc_type(size, AP_HAL::Util::MEM_FAST)));
        }
        uint8_t *bytes = input.get();
        int32_t offset = 0;
        while (bytes != nullptr && offset < length) {
            const int32_t count = fs.read(fd, bytes + offset, length - offset);
            if (count <= 0) {
                break;
            }
            offset += count;
        }
        const bool closed = fs.close(fd) == 0;
        if (bytes == nullptr || offset != length || !closed) {
            rust_math_test_result = 6;
            return false;
        }
        Module::Error error;
        _module = Module::load(bytes, length, "rust-math", {AP_MODULE_MATH_HEAP_SIZE, AP_MODULE_MATH_CODE_SIZE, 16384}, {this, text}, error,
        {nullptr, AP_Module::binding_dispatch, AP_Module::binding_resolve});
        if (_module == nullptr) {
            rust_math_test_result = 10 + uint8_t(error);
        }
        return _module != nullptr;
    }

    bool cycle()
    {
        const uint32_t expected = _messages.load() + 1;
        _sampled.store(false);
        struct ReleaseSample {
            std::atomic<bool> &sampled;
            ~ReleaseSample()
            {
                sampled.store(true);
            }
        } release_sample{_sampled};
        bool sampled_ok = false;
        rust_math_test_result = 2;
        if (!load()) {
            return false;
        }
        if (!_module->start()) {
            rust_math_test_result = 7;
            return false;
        }
        rust_math_test_result = 3;
        for (unsigned i = 0; i < AP_MODULE_MATH_TIMEOUT_MS; i++) {
            if (_messages.load() >= expected && !_sampled.load()) {
                sampled_ok = sample_stack();
                _sampled.store(true);
            }
            if (_module->poll_join()) {
                rust_math_test_status = _module->result();
                rust_math_test_heap_peak = _module->stats().heap.peak;
                const bool ok = rust_math_test_status == AP_MODULE_STATUS_OK && _module->stats().heap.used == 0 &&
                                _messages.load() == expected && sampled_ok;
                return Module::release(_module) && ok;
            }
            hal.scheduler->delay(1);
        }
        // A per-load deadline, distinct from an unfinished emulator run.
        // ReleaseSample unblocks late text; never free a live worker here.
        rust_math_test_result = 8;
        return false;
    }

    Module *_module{};
    std::atomic<uint32_t> _messages{0};
    std::atomic<bool> _sampled{false};
};

static MathHarness harness;
#endif

void setup();
void loop();

void setup()
{
    rust_math_test_result = 1;
#if AP_MODULE_ENABLED && HAL_OWNED_THREADS_ENABLED && HAL_EXECUTABLE_MEMORY_ENABLED
    if (!hal.scheduler->thread_create(FUNCTOR_BIND(&harness, &MathHarness::run, void), "rust-math-manager", 16384,
                                      AP_HAL::Scheduler::PRIORITY_SCRIPTING, 1)) {
        rust_math_test_result = 4;
    }
#endif
}

void loop()
{
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
