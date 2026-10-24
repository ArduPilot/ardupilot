// SPDX-License-Identifier: GPL-3.0-or-later
// Real HAL/SD-card lifecycle harness; not the vehicle module manager.
#include <AP_HAL/AP_HAL.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_Module/Module.h>
#include <fcntl.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

volatile uint32_t rust_module_test_result;
volatile uint32_t rust_module_test_cycles;
volatile uint32_t rust_module_test_before;
volatile uint32_t rust_module_test_after;
volatile uint32_t rust_module_test_retained;
volatile uint32_t rust_module_test_loads;
volatile uint32_t rust_module_test_free_on_failure;
volatile uint32_t rust_module_test_overflows;

#if AP_MODULE_ENABLED && HAL_OWNED_THREADS_ENABLED && HAL_EXECUTABLE_MEMORY_ENABLED
using AP_Module::Module;

class ModuleHarness
{
public:
    void run();

private:
    static void text(void *context, uint8_t severity, const char *message, uint32_t length)
    {
        auto &self = *static_cast<ModuleHarness *>(context);
        if (severity == 6 && length == 5 && memcmp(message, "ready", 5) == 0) {
            self._messages.fetch_add(1);
        }
    }
    Module *load(const char *path)
    {
        auto &fs = AP::FS();
        const int fd = fs.open(path, O_RDONLY);
        if (fd < 0) {
            rust_module_test_result = 2;
            return nullptr;
        }
        const int32_t length = fs.lseek(fd, 0, SEEK_END);
        AP_Module::Heap scratch;
        uint8_t *bytes = nullptr;
        if (length > 0 && length <= 65536 && fs.lseek(fd, 0, SEEK_SET) == 0 && scratch.create(length + 128)) {
            bytes = static_cast<uint8_t *>(scratch.allocate(length, 8));
        }
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
            rust_module_test_result = 3;
            return nullptr;
        }
        Module::Error error;
        auto *module = Module::load(bytes, length, "rust-test",
                                    {
                                        4096, 65536, 16384
                                    }, {this, text}, error,
                                    {nullptr, AP_Module::binding_dispatch, AP_Module::binding_resolve});
        if (module == nullptr) {
            rust_module_test_result = 10 + uint8_t(error);
            rust_module_test_free_on_failure = hal.util->available_memory();
        } else {
            rust_module_test_loads = rust_module_test_loads + 1;
        }
        return module;
    }
    bool wait_messages(uint32_t expected)
    {
        for (unsigned i = 0; i < 2000; i++) {
            if (_messages.load() == expected) {
                return true;
            }
            hal.scheduler->delay(1);
        }
        rust_module_test_result = 4;
        return false;
    }
    bool cycle()
    {
        const uint32_t expected = _messages.load() + 2;
        for (auto &module : _modules) {
            module = load("/APM/modules/cleanup.apmod");
            if (module == nullptr) {
                return false;
            }
        }
        // Reserve executable-capable RAM before stacks can consume it. F405
        // stacks may fall back to CCM, whereas executable images cannot.
        for (auto &module : _modules) {
            if (!module->start()) {
                rust_module_test_result = 5;
                return false;
            }
        }
        if (!wait_messages(expected)) {
            return false;
        }
        for (auto &module : _modules) {
            if (Module::release(module)) {
                rust_module_test_result = 6;
                return false;
            }
            module->request_stop();
        }
        for (auto &module : _modules) {
            bool joined = false;
            for (unsigned i = 0; i < 2000; i++) {
                if (module->poll_join()) {
                    joined = true;
                    break;
                }
                hal.scheduler->delay(1);
            }
            if (!joined || module->result() != AP_MODULE_STATUS_STOPPED || module->stats().heap.used != 0 ||
                !Module::release(module)) {
                rust_module_test_result = 7;
                return false;
            }
        }
        return true;
    }
    bool overflow_cycle()
    {
        Module *module = load("@ROMFS/overflow.apmod");
        if (module == nullptr || !module->start()) {
            rust_module_test_result = 30;
            return false;
        }
        for (unsigned i = 0; i < 5000 && module->state() != Module::State::Returned; i++) {
            hal.scheduler->delay(1);
        }
        if (module->state() != Module::State::Returned || !module->stack_overflowed() ||
            module->result() != AP_MODULE_STATUS_HOST_ERROR || module->stats().heap.used == 0) {
            rust_module_test_result = 31;
            return false;
        }
        for (unsigned i = 0; i < 5000 && !module->poll_join(); i++) {
            hal.scheduler->delay(1);
        }
        if (!Module::release(module)) {
            rust_module_test_result = 32;
            return false;
        }
        rust_module_test_overflows++;
        return true;
    }
    bool exercise_overflow()
    {
        if (!overflow_cycle()) {
            return false;
        }
        rust_module_test_before = hal.util->available_memory();
        for (unsigned i = 0; i < 100; i++) {
            if (!overflow_cycle()) {
                return false;
            }
            rust_module_test_cycles++;
        }
        rust_module_test_after = hal.util->available_memory();
        return rust_module_test_before == rust_module_test_after;
    }
    bool exercise()
    {
        struct stat overflow_stat;
        if (AP::FS().stat("@ROMFS/overflow.apmod", &overflow_stat) == 0) {
            return exercise_overflow();
        }
        bool mounted = false;
        for (unsigned i = 0; i < 5; i++) {
            if (AP::FS().retry_mount()) {
                mounted = true;
                break;
            }
            hal.scheduler->delay(100);
        }
        if (!mounted) {
            rust_module_test_result = 24;
            return false;
        }
        if (!cycle()) {
            return false;
        }
        rust_module_test_before = hal.util->available_memory();
        for (unsigned i = 0; i < 100; i++) {
            if (!cycle()) {
                return false;
            }
            rust_module_test_cycles = rust_module_test_cycles + 1;
        }
        rust_module_test_after = hal.util->available_memory();
        if (rust_module_test_after != rust_module_test_before) {
            rust_module_test_result = 8;
            return false;
        }
        _modules[0] = load("/APM/modules/panic.apmod");
        if (_modules[0] == nullptr || !_modules[0]->start()) {
            rust_module_test_result = 9;
            return false;
        }
        for (unsigned i = 0; i < 2000 && _modules[0]->state() != Module::State::Panicked; i++) {
            hal.scheduler->delay(1);
        }
        if (_modules[0]->state() != Module::State::Panicked) {
            rust_module_test_result = 20;
            return false;
        }
        _modules[0]->request_stop();
        const auto before = _modules[0]->stats();
        // The -1 manager offset clamps to the workers' LOWPRIO on ChibiOS.
        // Without time slicing, a busy-looping panic would starve this harness.
        // Require progress across several parked wakeups.
        for (unsigned i = 0; i < 30; i++) {
            if (Module::release(_modules[0])) {
                rust_module_test_result = 21;
                return false;
            }
            hal.scheduler->delay(100);
        }
        const auto after = _modules[0]->stats();
        if (before.heap.used == 0 || before.heap.used != after.heap.used ||
            before.rx_reserved != after.rx_reserved || before.rw_reserved != after.rw_reserved ||
            before.thread_reserved != after.thread_reserved) {
            rust_module_test_result = 22;
            return false;
        }
        rust_module_test_retained = after.heap.used + after.rx_reserved + after.rw_reserved +
                                    after.thread_reserved + after.metadata;
        return true;
    }

    Module *_modules[2] {};
    std::atomic<uint32_t> _messages{0};
};

static ModuleHarness harness;

void ModuleHarness::run()
{
    if (exercise()) {
        rust_module_test_result = 0x600DF00D;
    }
    // Failure can leave a deliberately live worker. Keep its sink/context and
    // every handle alive even on failure; the emulator owns process teardown.
    while (true) {
        hal.scheduler->delay(1000);
    }
}
#endif

void setup();
void loop();

void setup()
{
    rust_module_test_result = 1;
#if AP_MODULE_ENABLED && HAL_OWNED_THREADS_ENABLED && HAL_EXECUTABLE_MEMORY_ENABLED
    if (!hal.scheduler->thread_create(FUNCTOR_BIND(&harness, &ModuleHarness::run, void), "rust-manager", 8192,
                                      AP_HAL::Scheduler::PRIORITY_SCRIPTING, -1)) {
        rust_module_test_result = 23;
    }
#else
    rust_module_test_result = 0xBAD0;
#endif
}

void loop()
{
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
