// SPDX-License-Identifier: GPL-3.0-or-later
#include "Runtime.h"
#include <AP_Common/ExpandingString.h>

#if AP_MODULE_ENABLED
#include "BindingAnalogPWM.h"
#include "BindingArming.h"
#include "BindingCAN.h"
#include "BindingCRSF.h"
#include "BindingCustomMode.h"
#include "Bindings.h"
#include "BindingDroneCAN.h"
#include "BindingI2C.h"
#include "BindingSharedState.h"
#include "BindingMAVLink.h"
#include "BindingNetwork.h"
#include "BindingONVIF.h"
#include "BindingSerialLED.h"
#include <AP_Filesystem/AP_Filesystem.h>
#include <GCS_MAVLink/GCS.h>
#ifdef HAL_HAVE_AP_ROMFS_EMBEDDED_APMOD
#include <AP_ROMFS/AP_ROMFS.h>
#endif
#include <fcntl.h>
#include <memory>

extern const AP_HAL::HAL &hal;

namespace AP_Module
{

#if HAL_OS_FATFS_IO || HAL_OS_LITTLEFS_IO
static constexpr const char *DIRECTORY = "/APM/modules";
#else
static constexpr const char *DIRECTORY = "./modules";
#endif

Runtime *Runtime::_singleton;

const AP_Param::GroupInfo Runtime::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Native modules enable
    // @Description: Enable experimental trusted native modules. Native code is not isolated from the autopilot.
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, Runtime, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: HEAP_MAX
    // @DisplayName: Native module heap capacity limit
    // @Description: Maximum backing heap arena capacity per module. Initial size comes from the module file. Armed expansion is capped here and retains 20 KiB of system memory unless disabled by DEBUG_OPTS bit 6. Excludes code, stack and allocator metadata.
    // @Units: B
    // @Range: 256 16777216
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("HEAP_MAX", 2, Runtime, _heap_size, 32768),

    // @Param: CODE_MAXSIZE
    // @DisplayName: Native module image limit
    // @Description: Per-module code and data image limit in bytes, rounded up to a multiple of 4096. Only the loaded image is allocated, not this entire limit. Alignment and allocator overhead are additional.
    // @Units: B
    // @Range: 4096 16777216
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("CODE_MAXSIZE", 3, Runtime, _code_size, 131072),

    // @Param: STACK_MAX
    // @DisplayName: Native module stack limit
    // @Description: Maximum logical worker stack per module, including the declared module requirement plus host callback margin. Only the requested stack is allocated, rounded to platform minimum and alignment. OS and guard overhead are additional.
    // @Units: B
    // @Range: 256 16777216
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("STACK_MAX", 4, Runtime, _stack_size, 32768),

    // Index 5 was MAX_MOD; do not reuse its saved parameter identity.

    // @Param: RESTART
    // @DisplayName: Native module restart request counter
    // @Description: Change this value to request cooperative restart of all Native modules, including while armed. Non-cooperating workers prevent reload and retain their memory.
    // @Range: 0 32767
    // @User: Advanced
    AP_GROUPINFO("RESTART", 6, Runtime, _restart, 0),

    // @Param: LD_CSUM
    // @DisplayName: Required loaded Native module checksum
    // @Description: Required masked XOR of whole-file CRC32 values of loaded modules. Minus one disables this pre-arm requirement.
    // @Range: -1 8388607
    // @User: Advanced
    AP_GROUPINFO("LD_CSUM", 7, Runtime, _loaded_required, -1),

    // @Param: RUN_CSUM
    // @DisplayName: Required running Native module checksum
    // @Description: Required masked XOR of whole-file CRC32 values of running modules. Minus one disables this pre-arm requirement.
    // @Range: -1 8388607
    // @User: Advanced
    AP_GROUPINFO("RUN_CSUM", 8, Runtime, _running_required, -1),

    // @Param: DEBUG_OPTS
    // @DisplayName: Native module debug options
    // @Description: Disable module pre-arm checks, save current loaded and running checksums, or disable heap expansion. The save bit clears after storing both values. Changes to the heap expansion option take effect on module restart.
    // @Bitmask: 4:Disable pre-arm checks,5:Save checksums,6:Disable heap expansion
    // @User: Advanced
    AP_GROUPINFO("DEBUG_OPTS", 9, Runtime, _debug_options, 0),

    // @Param: THD_PRIO_MAX
    // @DisplayName: Maximum native module thread priority
    // @Description: Maximum native OS priority allowed by a module header. Defaults to 2 on STM32 and 1 on other HALs. The manager runs one level above this limit. The limit must leave room for the manager within the HAL priority range. Higher numbers mean higher priority. SITL and non-realtime Linux retain host time slicing.
    // @Range: 1 254
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("THD_PRIO_MAX", 10, Runtime, _thread_priority_max, AP_MODULE_DEFAULT_THREAD_PRIORITY),
    AP_GROUPEND
};

Runtime::Runtime()
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("Duplicate module runtime");
    }
    _singleton = this;
}

void Runtime::fail(const char *reason, bool preserve_error, const char *prearm_reason)
{
    {
        WITH_SEMAPHORE(_sem);
        if (!preserve_error || _error[0] == '\0') {
            hal.util->snprintf(_error, sizeof(_error), "%s", prearm_reason != nullptr ? prearm_reason : reason);
        }
    }
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Module: %s", reason);
}

void Runtime::init()
{
    if (_initialized) {
        return;
    }
    _initialized = true;
    _observed_restart = _restart.get();
    if (_enable == 0) {
        return;
    }
    constexpr uint32_t maximum = 16U * 1024 * 1024;
    const uint32_t code_size = Module::rounded_code_size(_code_size.get());
    if (_heap_size < 256 || _heap_size > maximum || code_size == 0 ||
        _stack_size < 256 || _stack_size > maximum) {
        fail("invalid memory limits; reboot required");
        return;
    }
    const uint16_t priority_max = _thread_priority_max.get();
    if (hal.scheduler->thread_priority_min() == 0 ||
        priority_max < hal.scheduler->thread_priority_min() ||
        priority_max >= hal.scheduler->thread_priority_max()) {
        fail("invalid MOD_THD_PRIO_MAX; reboot required");
        return;
    }
    uint32_t manager_stack;
    switch (hal.util->executable_memory_target()) {
    case AP_HAL::ExecutableMemory::Target::CortexM4HardFloat:
    case AP_HAL::ExecutableMemory::Target::CortexM7HardFloat:
        // Module code and binding calls run on worker stacks. The manager
        // only loads images, handles messages and reaps stopped workers.
        manager_stack = 4096;
        break;
    case AP_HAL::ExecutableMemory::Target::X86_64Baseline:
        manager_stack = 32768;
        break;
    default:
        fail("unsupported native execution target");
        return;
    }
    _limits = {_heap_size.get(), code_size, _stack_size.get(), true, priority_max};
    {
        WITH_SEMAPHORE(_sem);
        _phase = Phase::Loading;
    }
    // Preempt non-yielding workers on ChibiOS and Linux FIFO. SITL and
    // unprivileged Linux use the host's time-sliced SCHED_OTHER instead.
    _manager_thread = hal.scheduler->thread_create_owned(FUNCTOR_BIND_MEMBER(&Runtime::manager, void), "ModMgr",
                      manager_stack, AP_HAL::Scheduler::PRIORITY_ABSOLUTE, _limits.thread_priority_max + 1);
    if (_manager_thread == nullptr) {
        fail("manager thread creation failed");
        WITH_SEMAPHORE(_sem);
        _phase = Phase::Failed;
    }
}

void Runtime::update()
{
    if (!_initialized) {
        return;
    }
    const uint16_t request = _restart.get();
    if (request != _observed_restart) {
        _observed_restart = request;
        WITH_SEMAPHORE(_sem);
        _restart_pending = true;
    }
    if (_enable == 0) {
        return;
    }
    if ((uint8_t(_debug_options.get()) & (1U << 5)) != 0) {
        uint32_t loaded, running;
        {
            WITH_SEMAPHORE(_sem);
            loaded = _loaded_checksum & CHECKSUM_MASK;
            running = _running_checksum & CHECKSUM_MASK;
        }
        _loaded_required.set_and_save(loaded);
        _running_required.set_and_save(running);
        _debug_options.set_and_save(uint8_t(_debug_options.get()) & ~(1U << 5));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Module: saved checksums");
    }
    uint32_t dropped;
    {
        WITH_SEMAPHORE(_sem);
        dropped = _messages_dropped;
        _messages_dropped = 0;
    }
    if (dropped != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Module: dropped %lu text messages", (unsigned long)dropped);
    }
}

bool Runtime::arming_checks(size_t length, char *message) const
{
    if (_enable == 0 || (uint8_t(_debug_options.get()) & (1U << 4)) != 0) {
        return true;
    }
    WITH_SEMAPHORE(_sem);
    if (_error[0] != '\0') {
        hal.util->snprintf(message, length, "Module: %s", _error);
        return false;
    }
    if (_heap_required != 0) {
        hal.util->snprintf(message, length, "Module: rebuild with initial heap at least %u", unsigned(_heap_required));
        return false;
    }
    if (_loaded_required != -1 && (uint32_t(_loaded_required.get()) & CHECKSUM_MASK) != (_loaded_checksum & CHECKSUM_MASK)) {
        hal.util->snprintf(message, length, "Module: loaded checksum mismatch");
        return false;
    }
    if (_running_required != -1 && (uint32_t(_running_required.get()) & CHECKSUM_MASK) != (_running_checksum & CHECKSUM_MASK)) {
        hal.util->snprintf(message, length, "Module: running checksum mismatch");
        return false;
    }
    return true;
}

#if AP_MISSION_ENABLED
void Runtime::handle_mission_command(uint16_t p1, float content_p1, float content_p2, float content_p3)
{
    if (_enable == 0) {
        return;
    }
    WITH_SEMAPHORE(_mission_sem);
    if (_mission_count >= MISSION_QUEUE_SIZE) {
        // Match Lua's ObjectBuffer: a full queue drops the newest command.
        return;
    }
    const uint8_t tail = (_mission_head + _mission_count) % MISSION_QUEUE_SIZE;
    _mission_commands[tail] = {p1, content_p1, content_p2, content_p3, AP_HAL::millis()};
    _mission_count++;
}

bool Runtime::pop_mission_command(MissionCommand &command)
{
    WITH_SEMAPHORE(_mission_sem);
    if (_mission_count == 0) {
        return false;
    }
    command = _mission_commands[_mission_head];
    _mission_head = (_mission_head + 1) % MISSION_QUEUE_SIZE;
    _mission_count--;
    return true;
}
#endif // AP_MISSION_ENABLED

Module *Runtime::load_bytes(const uint8_t *bytes, uint32_t length, const ModuleFile &file, Slot &slot)
{
    Module::Error error;
    uint64_t missing_import = 0;
    const char *missing_import_name = nullptr;
    const char *error_detail = nullptr;
    auto *module = Module::load(bytes, length, file.name, _limits,
                                {
                                    &slot, text
                                }, error, {&slot, binding_dispatch, binding_resolve}, &missing_import,
                                &missing_import_name,
                                file.source == ModuleSource::ROMFS ? Module::Source::ROMFS : Module::Source::File,
                                &error_detail);
    if (module == nullptr) {
        char reason[144];
        char prearm_reason[sizeof(_error)];
        if (error == Module::Error::Imports) {
            if (missing_import_name != nullptr) {
                hal.util->snprintf(reason, sizeof(reason), "missing API %.63s in %.31s [%08x%08x]",
                                   missing_import_name, file.name, unsigned(missing_import >> 32),
                                   unsigned(missing_import));
            } else {
                hal.util->snprintf(reason, sizeof(reason), "%.31s missing API %08x%08x", file.name,
                                   unsigned(missing_import >> 32), unsigned(missing_import));
            }
        } else {
            const char *detail = error_detail != nullptr ? error_detail : Module::error_string(error);
            hal.util->snprintf(reason, sizeof(reason), "%.31s load failed: %s", file.name,
                               detail);
            hal.util->snprintf(prearm_reason, sizeof(prearm_reason), "%.15s: %s", file.name, detail);
        }
        fail(reason, false, error == Module::Error::Imports ? nullptr : prearm_reason);
    }
    return module;
}

Module *Runtime::load_file(const ModuleFile &file, Slot &slot)
{
#ifdef HAL_HAVE_AP_ROMFS_EMBEDDED_APMOD
    if (file.source == ModuleSource::ROMFS) {
        char path[sizeof(file.name) + sizeof("modules/")];
        hal.util->snprintf(path, sizeof(path), "modules/%s", file.name);
        uint32_t length = 0;
        const uint8_t *bytes = AP_ROMFS::find_uncompressed(path, length);
        if (bytes == nullptr) {
            char reason[96];
            hal.util->snprintf(reason, sizeof(reason), "%.31s ROMFS module unavailable", file.name);
            fail(reason);
            return nullptr;
        }
        // find_uncompressed() borrows immutable firmware storage, which
        // remains valid for every module generation and worker lifetime.
        return load_bytes(bytes, length, file, slot);
    }
#endif
    char path[96];
    hal.util->snprintf(path, sizeof(path), "%s/%s", DIRECTORY, file.name);
    auto &fs = AP::FS();
    const int fd = fs.open(path, O_RDONLY);
    if (fd < 0) {
        fail("module open failed");
        return nullptr;
    }
    const int32_t length = fs.lseek(fd, 0, SEEK_END);
    // Image, worst-case 32-bit relocations, and capped import records/names.
    const uint32_t maximum = _limits.code * 3U + 4096U * (16U + 64U) + 88U;
    const uint32_t input_size = length > 0 ? uint32_t(length) : 0;
    const auto free_input = [input_size](uint8_t *buffer) {
        hal.util->free_type(buffer, input_size, AP_HAL::Util::MEM_FAST);
    };
    std::unique_ptr<uint8_t, decltype(free_input)> input(nullptr, free_input);
    // Firmware loading scratch is separate from each module's AP_MultiHeap.
    // Prefer non-executable fast RAM so the input does not crowd out its image.
    if (input_size > 0 && input_size <= maximum && input_size <= 16 * 1024 * 1024 &&
        fs.lseek(fd, 0, SEEK_SET) == 0) {
        input.reset(static_cast<uint8_t *>(hal.util->malloc_type(input_size, AP_HAL::Util::MEM_FAST)));
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
    uint8_t extra;
    const bool complete = bytes != nullptr && offset == length && fs.read(fd, &extra, 1) == 0;
    const bool closed = fs.close(fd) == 0;
    if (!complete || !closed) {
        fail("module read/size/scratch allocation failed");
        return nullptr;
    }
    return load_bytes(bytes, input_size, file, slot);
}

bool Runtime::add_file(const char *name, ModuleSource source)
{
    const size_t length = strlen(name);
    if (length >= sizeof(ModuleFile::name)) {
        char reason[112];
        hal.util->snprintf(reason, sizeof(reason), "module filename exceeds 63 characters: %.63s", name);
        fail(reason);
        return false;
    }
    // Insert in basename order without moving a callback context. ROMFS is
    // discovered first and takes precedence over writable duplicates.
    Slot **link = &_slots;
    while (*link != nullptr && strcmp((*link)->file.name, name) < 0) {
        link = &(*link)->next;
    }
    if (*link != nullptr && strcmp((*link)->file.name, name) == 0) {
        if ((*link)->file.source == ModuleSource::ROMFS && source == ModuleSource::Writable) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Module: ignored writable %.31s; ROMFS wins", name);
        }
        return true;
    }
    auto *slot = NEW_NOTHROW Slot{};
    if (slot == nullptr) {
        char reason[112];
        hal.util->snprintf(reason, sizeof(reason), "module metadata allocation failed: %.63s", name);
        fail(reason);
        return false;
    }
    slot->owner = this;
    memcpy(slot->file.name, name, length + 1);
    slot->file.source = source;
    slot->next = *link;
    {
        WITH_SEMAPHORE(_modules_sem);
        *link = slot;
    }
    return true;
}

void Runtime::clear_slots()
{
    // Only called after all_released(): no worker, owned-thread epilogue or
    // binding registry may still reference any of these contexts.
    Slot *slots;
    {
        WITH_SEMAPHORE(_modules_sem);
        slots = _slots;
        _slots = nullptr;
    }
    while (slots != nullptr) {
        Slot *slot = slots;
        slots = slot->next;
        delete slot;
    }
}

bool Runtime::load_generation()
{
    if (_slots != nullptr) {
        fail("previous generation still retained");
        return false;
    }
    {
        WITH_SEMAPHORE(_sem);
        _phase = Phase::Loading;
        _error[0] = '\0';
        _heap_required = 0;
        _loaded_checksum = _running_checksum = 0;
    }
    _limits.allow_heap_expansion = (uint8_t(_debug_options.get()) & (1U << 6)) == 0;
#ifdef HAL_HAVE_AP_ROMFS_EMBEDDED_APMOD
    uint16_t romfs_offset = 0;
    while (const char *path = AP_ROMFS::dir_list("modules", romfs_offset)) {
        constexpr size_t prefix_length = sizeof("modules/") - 1;
        if (strncmp(path, "modules/", prefix_length) != 0) {
            continue;
        }
        const char *name = path + prefix_length;
        const size_t length = strlen(name);
        // Runtime module discovery is deliberately non-recursive, matching
        // the writable module directory.
        if (strchr(name, '/') != nullptr || length < 6 || strcmp(name + length - 6, ".apmod") != 0) {
            continue;
        }
        if (!add_file(name, ModuleSource::ROMFS)) {
            return false;
        }
    }
#endif
    auto &fs = AP::FS();
    errno = 0;
    auto *directory = fs.opendir(DIRECTORY);
    if (directory == nullptr) {
        bool romfs_only = false;
#ifdef HAL_HAVE_AP_ROMFS_EMBEDDED_APMOD
        // A missing or unmounted SD card must not prevent embedded startup.
        romfs_only = _slots != nullptr && (errno == EIO || errno == ENOSPC || errno == ENODEV);
#endif
        if (errno != ENOENT && !romfs_only) {
            fail("module directory unavailable");
            return false;
        }
    } else {
        bool valid = true;
        int read_error = 0;
        while (true) {
            // Allocation and diagnostic calls below may change errno. Only
            // the immediately preceding readdir determines a read failure.
            errno = 0;
            const auto *entry = fs.readdir(directory);
            if (entry == nullptr) {
                read_error = errno;
                break;
            }
            const size_t length = strlen(entry->d_name);
            if (length < 6 || strcmp(entry->d_name + length - 6, ".apmod") != 0) {
                continue;
            }
            if (!add_file(entry->d_name, ModuleSource::Writable)) {
                valid = false;
                break;
            }
        }
        const bool closed = fs.closedir(directory) == 0;
        const int close_error = errno;
        if (!valid) {
            return false;
        }
        if (read_error != 0 || !closed) {
            char reason[96];
            hal.util->snprintf(reason, sizeof(reason), "module directory %s failed (errno %d)",
                               read_error != 0 ? "read" : "close", read_error != 0 ? read_error : close_error);
            fail(reason);
            return false;
        }
    }
    size_t count = 0;
    for (Slot *item = _slots; item != nullptr; item = item->next) {
        auto &slot = *item;
        Module *prepared = load_file(slot.file, slot);
        {
            WITH_SEMAPHORE(_modules_sem);
            slot.module = prepared;
        }
        if (slot.module == nullptr) {
            return false;
        }
        WITH_SEMAPHORE(_sem);
        _loaded_checksum ^= slot.module->checksum();
        count++;
    }
    for (Slot *item = _slots; item != nullptr; item = item->next) {
        auto &slot = *item;
        bool started;
        {
            WITH_SEMAPHORE(_modules_sem);
            started = slot.module->start();
        }
        if (!started) {
            char reason[112];
            hal.util->snprintf(reason, sizeof(reason), "%.63s heap/thread creation failed", slot.file.name);
            fail(reason);
            return false;
        }
        slot.running = true;
        {
            WITH_SEMAPHORE(_sem);
            _running_checksum ^= slot.module->checksum();
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Module: started %s", slot.module->name());
        report_memory(*slot.module);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Module: loaded %u modules", unsigned(count));
    return true;
}

void Runtime::report_memory(Module &module)
{
    // Separate reservations/counters, not total RAM: backend heap overhead,
    // manager storage and temporary input buffers are not included here.
    const auto stats = module.stats();
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Module RAM: RX %u RW %u stack %u",
                  unsigned(stats.rx_reserved), unsigned(stats.rw_reserved), unsigned(stats.stack_reserved));
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Module RAM: heap %u used %u peak %u thread %u", unsigned(stats.heap_initial),
                  unsigned(stats.heap.used), unsigned(stats.heap.peak), unsigned(stats.thread_reserved));
    if (stats.heap.expanded_to != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Module RAM: heap expanded to %u", unsigned(stats.heap.expanded_to));
    }
}

void Runtime::poll_modules()
{
    for (Slot *item = _slots; item != nullptr; item = item->next) {
        auto &slot = *item;
        auto *module = slot.module;
        if (module == nullptr) {
            continue;
        }
        const auto state = module->state();
        if (state == Module::State::Panicked && !slot.panic_reported) {
            slot.panic_reported = true;
            char reason[80];
            hal.util->snprintf(reason, sizeof(reason), "%.24s panic; reboot required", module->name());
            fail(reason);
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Module fatal: %.37s", module->panic_message());
            report_memory(*module);
        }
        bool joined;
        {
            WITH_SEMAPHORE(_modules_sem);
            joined = module->poll_join();
        }
        // Sample after joining too: a worker can expand and return between
        // manager polls. Its final requirement must survive heap destruction.
        const auto heap_stats = module->stats().heap;
        const uint32_t expanded_to = heap_stats.expanded_to;
        if (expanded_to > slot.heap_expansion_reported) {
            slot.heap_expansion_reported = expanded_to;
            {
                WITH_SEMAPHORE(_sem);
                _heap_required = MAX(_heap_required, expanded_to);
            }
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Module: %.24s initial heap needs %u",
                          module->name(), unsigned(expanded_to));
        }
        const uint8_t failures = heap_stats.expansion_failures & ~slot.heap_failures_reported;
        slot.heap_failures_reported |= failures;
        if ((failures & MultiHeap::Capacity) != 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Module: %.24s MOD_HEAP_MAX reached", module->name());
        }
        if ((failures & MultiHeap::Arenas) != 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Module: %.24s heap arena slots exhausted", module->name());
        }
        if (slot.running && (joined || state == Module::State::Returned || state == Module::State::Panicked)) {
            slot.running = false;
            WITH_SEMAPHORE(_sem);
            _running_checksum ^= module->checksum();
        }
        if (!joined) {
            continue;
        }
        if (state != Module::State::Prepared) {
            const int32_t result = module->result();
            if (module->stack_overflowed()) {
                char reason[80];
                hal.util->snprintf(reason, sizeof(reason), "%.31s stack overflow", module->name());
                fail(reason);
            } else if (result != AP_MODULE_STATUS_OK && result != AP_MODULE_STATUS_STOPPED) {
                char reason[80];
                hal.util->snprintf(reason, sizeof(reason), "%.31s returned %ld", module->name(), long(result));
                fail(reason);
            }
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Module: exited %s", module->name());
            report_memory(*module);
        }
        binding_dronecan_cleanup(&slot);
        binding_i2c_cleanup(&slot);
        binding_shared_state_cleanup(&slot);
        binding_can_cleanup(&slot);
        binding_crsf_cleanup(&slot);
        binding_custom_mode_cleanup(&slot);
        binding_analog_pwm_cleanup(&slot);
        binding_mavlink_cleanup(&slot);
        binding_network_cleanup(&slot);
        binding_onvif_cleanup(&slot);
        binding_serialled_cleanup(&slot);
        binding_arming_cleanup(&slot);
        {
            WITH_SEMAPHORE(_modules_sem);
            Module::release(slot.module);
        }
    }
}

static const char *module_state_name(Module::State state)
{
    switch (state) {
    case Module::State::Prepared: return "PREPARED";
    case Module::State::Running: return "RUN";
    case Module::State::Returned: return "RETURNED";
    case Module::State::Joined: return "JOINED";
    case Module::State::Panicked: return "PANICKED";
    }
    return "?";
}

void Runtime::module_info(ExpandingString &str)
{
    str.printf("ModulesV1\n");
    Runtime *runtime = get_singleton();
    if (runtime == nullptr) {
        return;
    }
    WITH_SEMAPHORE(runtime->_modules_sem);
    for (const Slot *slot = runtime->_slots; slot != nullptr; slot = slot->next) {
        Module *module = slot->module;
        if (module == nullptr) {
            continue;
        }
        const auto stats = module->stats(true);
        str.printf("%s STATE=%s SRC=%s PRI=%u RX=%u RW=%u STK=%u "
                   "HEAP=%u/%u ALLOC=%u PEAK=%u ALLOCS=%u",
                   slot->file.name, module_state_name(module->state()),
                   slot->file.source == ModuleSource::ROMFS ? "ROMFS" : "FILE",
                   unsigned(module->thread_priority()), unsigned(stats.rx_reserved), unsigned(stats.rw_reserved),
                   unsigned(stats.stack_reserved), unsigned(stats.heap.free), unsigned(stats.heap.capacity),
                   unsigned(stats.heap.used), unsigned(stats.heap.peak), unsigned(stats.heap.allocations));
        float load;
        if (module->cpu_load(load)) {
            str.printf(" LOAD=%.1f%%", double(load));
        }
        str.printf("\n");
    }
}

bool Runtime::all_released() const
{
    for (const Slot *item = _slots; item != nullptr; item = item->next) {
        const auto &slot = *item;
        if (slot.module != nullptr) {
            return false;
        }
    }
    return true;
}

void Runtime::begin_stop(bool reload)
{
    _reload_after_stop = reload;
    _stop_start_ms = AP_HAL::millis();
    for (Slot *item = _slots; item != nullptr; item = item->next) {
        auto &slot = *item;
        if (slot.module != nullptr) {
            slot.module->request_stop();
        }
    }
    WITH_SEMAPHORE(_sem);
    _phase = Phase::Stopping;
}

void Runtime::manager()
{
    if (!load_generation()) {
        begin_stop(false);
    } else {
        WITH_SEMAPHORE(_sem);
        _phase = Phase::Running;
    }
    while (true) {
        drain_messages();
        poll_modules();
        Phase phase;
        bool restart;
        {
            WITH_SEMAPHORE(_sem);
            phase = _phase;
            restart = _restart_pending;
            _restart_pending = false;
        }
        if (restart && phase != Phase::Stopping) {
            begin_stop(true);
            phase = Phase::Stopping;
        } else if (restart) {
            _reload_after_stop = true;
        }
        if (phase == Phase::Stopping) {
            if (all_released()) {
                clear_slots();
                if (_reload_after_stop) {
                    drain_messages();
                    if (load_generation()) {
                        WITH_SEMAPHORE(_sem);
                        _phase = Phase::Running;
                    } else {
                        begin_stop(false);
                    }
                } else {
                    WITH_SEMAPHORE(_sem);
                    _phase = Phase::Failed;
                }
            } else if (uint32_t(AP_HAL::millis() - _stop_start_ms) >= 10000) {
                bool panicked = false;
                for (const Slot *slot = _slots; slot != nullptr; slot = slot->next) {
                    panicked |= slot->module != nullptr && slot->module->state() == Module::State::Panicked;
                }
                fail(panicked ? "stop timeout: panic requires reboot" : "stop timeout: memory retained; retry/reboot", true);
                WITH_SEMAPHORE(_sem);
                _phase = Phase::Failed;
            }
        }
        hal.scheduler->delay(10);
    }
}

void Runtime::text(void *context, uint8_t severity, const char *message, uint32_t length)
{
    auto &runtime = *static_cast<Slot *>(context)->owner;
    WITH_SEMAPHORE(runtime._sem);
    if (runtime._message_count == ARRAY_SIZE(runtime._messages)) {
        runtime._messages_dropped++;
        return;
    }
    auto &entry = runtime._messages[(runtime._message_head + runtime._message_count) % ARRAY_SIZE(runtime._messages)];
    entry.severity = severity;
    const uint32_t count = length < sizeof(entry.text) - 1 ? length : sizeof(entry.text) - 1;
    memcpy(entry.text, message, count);
    entry.text[count] = '\0';
    runtime._message_count++;
}

void Runtime::drain_messages()
{
    // Bound work per iteration even when a module continuously sends text.
    for (uint8_t i = 0; i < ARRAY_SIZE(_messages); i++) {
        Message message;
        {
            WITH_SEMAPHORE(_sem);
            if (_message_count == 0) {
                break;
            }
            message = _messages[_message_head];
            _message_head = (_message_head + 1) % ARRAY_SIZE(_messages);
            _message_count--;
        }
        GCS_SEND_TEXT(MAV_SEVERITY(message.severity), "Module: %s", message.text);
    }
}

}
#endif // AP_MODULE_ENABLED
