// SPDX-License-Identifier: GPL-3.0-or-later
// Execute the C Lua module on an H7 HAL, including repeated unload/reload.
#include <AP_HAL/AP_HAL.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_Module/Bindings.h>
#include <AP_Module/Module.h>
#include <AP_Common/ExpandingString.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <atomic>
#include <fcntl.h>
#include <stdlib.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

volatile uint32_t lua_module_test_result;
volatile uint32_t lua_module_test_cycles;
volatile uint32_t lua_module_test_before;
volatile uint32_t lua_module_test_after;
volatile uint32_t lua_module_test_heap_peak;
volatile uint32_t lua_module_test_stack_used;
volatile uint32_t lua_module_test_stack_total;
char lua_module_test_error[256];

#if AP_MODULE_ENABLED && HAL_OWNED_THREADS_ENABLED && HAL_EXECUTABLE_MEMORY_ENABLED
static std::atomic<bool> sampled;
static std::atomic<bool> finished;
class LuaGCS : public GCS_Dummy
{
public:
    void send_textv(MAV_SEVERITY severity, const char *format, va_list args, mavlink_channel_mask_t) override
    {
        char text[256];
        hal.util->vsnprintf(text, sizeof(text), format, args);
        if (severity <= MAV_SEVERITY_ERROR) {
            hal.util->snprintf(lua_module_test_error, sizeof(lua_module_test_error), "%s", text);
        }
        if (strcmp(text, "Lua native test OK") == 0) {
            finished.store(true);
            while (!sampled.load()) {
                hal.scheduler->delay(1);
            }
        }
    }
};
static LuaGCS test_gcs;

static bool cycle()
{
    sampled.store(false);
    finished.store(false);
    const int fd = AP::FS().open("/APM/modules/lua.apmod", O_RDONLY);
    if (fd < 0) {
        return false;
    }
    const int32_t length = AP::FS().lseek(fd, 0, SEEK_END);
    uint8_t *source = length > 0 && length <= 131072 ?
                      static_cast<uint8_t *>(hal.util->malloc_type(length, AP_HAL::Util::MEM_FAST)) : nullptr;
    const bool read = source != nullptr && AP::FS().lseek(fd, 0, SEEK_SET) == 0 &&
                      AP::FS().read(fd, source, length) == length;
    AP::FS().close(fd);
    AP_Module::Module::Error error;
    AP_Module::Module *module = nullptr;
    if (read) {
        const AP_Module::Module::Limits limits(131072, 131072, 32768, false, 2);
        module = AP_Module::Module::load(source, length, "lua-test", limits, {}, error,
        {nullptr, AP_Module::binding_dispatch, AP_Module::binding_resolve});
    }
    hal.util->free_type(source, length > 0 ? length : 0, AP_HAL::Util::MEM_FAST);
    if (module == nullptr) {
        return false;
    }
    if (!module->start()) {
        AP_Module::Module::release(module);
        return false;
    }
    for (unsigned n = 0; n < 10000; n++) {
        if (finished.load() && !sampled.load()) {
            ExpandingString threads;
            hal.util->thread_info(threads);
            const char *worker = threads.get_string() == nullptr ? nullptr : strstr(threads.get_string(), "lua-test ");
            const char *stack = worker == nullptr ? nullptr : strstr(worker, "STACK=");
            if (stack != nullptr) {
                char *separator;
                const uint32_t free = strtoul(stack + 6, &separator, 10);
                const uint32_t total = *separator == '/' ? strtoul(separator + 1, nullptr, 10) : 0;
                lua_module_test_stack_total = total;
                if (free <= total && total - free > lua_module_test_stack_used) {
                    lua_module_test_stack_used = total - free;
                }
            }
            sampled.store(true);
        }
        if (module->poll_join()) {
            const auto stats = module->stats();
            lua_module_test_heap_peak = MAX(lua_module_test_heap_peak, stats.heap.peak);
            const bool ok = finished.load() && module->result() == AP_MODULE_STATUS_OK && stats.heap.used == 0;
            return AP_Module::Module::release(module) && ok;
        }
        hal.scheduler->delay(1);
    }
    sampled.store(true);
    module->request_stop();
    // Never release a worker that has not joined.
    return false;
}
static void run(void *)
{
    lua_module_test_result = 1;
    if (!AP::FS().retry_mount() || !cycle()) {
        return;
    }
    lua_module_test_before = hal.util->available_memory();
    for (unsigned n = 0; n < 10; n++) {
        if (!cycle()) {
            return;
        }
        lua_module_test_cycles++;
    }
    lua_module_test_after = hal.util->available_memory();
    if (lua_module_test_before == lua_module_test_after) {
        lua_module_test_result = 0x600df00d;
    }
}
#endif

void setup();
void loop();
void setup()
{
#if AP_MODULE_ENABLED && HAL_OWNED_THREADS_ENABLED && HAL_EXECUTABLE_MEMORY_ENABLED
    if (!hal.scheduler->thread_create({nullptr, run}, "lua-manager", 8192,
                                      AP_HAL::Scheduler::PRIORITY_SCRIPTING, 1)) {
        lua_module_test_result = 2;
    }
#endif
}
void loop()
{
    hal.scheduler->delay(1000);
}
AP_HAL_MAIN();
