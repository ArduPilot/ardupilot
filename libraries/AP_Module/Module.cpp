// SPDX-License-Identifier: GPL-3.0-or-later
#include "Module.h"

#if AP_MODULE_ENABLED && HAL_OWNED_THREADS_ENABLED && HAL_EXECUTABLE_MEMORY_ENABLED
#include "Image.h"
#include <string.h>

extern const AP_HAL::HAL &hal;

namespace AP_Module
{

const AP_ModuleHostApi Module::_api {
    AP_MODULE_ABI_VERSION, sizeof(AP_ModuleHostApi), AP_MODULE_API_FINGERPRINT,
    allocate, deallocate, time_us, sleep_ms, stop_requested, text, panic, binding_call
};

uint32_t Module::rounded_code_size(uint32_t requested)
{
    if (requested < 4096 || requested > 16U * 1024 * 1024) {
        return 0;
    }
    return (requested + 4095U) & ~4095U;
}

const char *Module::error_string(Error error)
{
    switch (error) {
    case Error::None:
        return "no error";
    case Error::Limits:
        return "invalid memory limits";
    case Error::Target:
        return "unsupported native execution target";
    case Error::Image:
        return "invalid module image";
    case Error::Memory:
        return "insufficient image memory";
    case Error::Publication:
        return "image relocation/publication failed";
    case Error::Imports:
        return "missing API import";
    }
    return "unknown error";
}

Module *Module::load(const uint8_t *source, size_t length, const char *name,
                     const Limits &limits, TextSink sink, Error &error, BindingSink bindings,
                     uint64_t *missing_import, const char **missing_import_name, Source source_type,
                     const char **error_detail)
{
    if (error_detail != nullptr) {
        *error_detail = nullptr;
    }
    constexpr uint32_t maximum = 16U * 1024 * 1024;
    const uint32_t code_size = rounded_code_size(limits.code);
    error = Error::Limits;
    if (name == nullptr || name[0] == '\0' || limits.heap < 256 || limits.heap > maximum ||
        code_size == 0 ||
        limits.stack < 256 || limits.stack > maximum) {
        return nullptr;
    }
    Image::Compatibility expected{0, AP_MODULE_ABI_VERSION, 0, 0, AP_MODULE_API_FINGERPRINT};
    uint32_t margin = 0;
    error = Error::Target;
    switch (hal.util->executable_memory_target()) {
    case AP_HAL::ExecutableMemory::Target::CortexM4HardFloat:
    case AP_HAL::ExecutableMemory::Target::CortexM7HardFloat:
        expected.machine = 40;
        expected.cpu_profile = hal.util->executable_memory_target() ==
                               AP_HAL::ExecutableMemory::Target::CortexM7HardFloat ? 3 : 1;
        // Provisional host callback margin, including the TextSink contract.
        // Measure the complete binding set before enabling vehicle modules.
        margin = 4096;
        break;
    case AP_HAL::ExecutableMemory::Target::X86_64Baseline:
        expected.machine = 62;
        expected.cpu_profile = 2;
        // Includes libc/TLS use and host callbacks, not a stack-usage proof.
        margin = 16384;
        break;
    default:
        return nullptr;
    }
    error = Error::Limits;
    if (limits.stack <= margin) {
        return nullptr;
    }
    expected.stack_limit = limits.stack - margin;
    expected.heap_limit = limits.heap;
    expected.priority_min = hal.scheduler->thread_priority_min();
    expected.priority_limit = limits.thread_priority_max == 0 ? expected.priority_min : limits.thread_priority_max;
    // Leave a higher priority for the manager; never silently clamp a native
    // priority or truncate it through the scheduler's relative-offset API.
    if (expected.priority_min == 0 || expected.priority_limit < expected.priority_min ||
        expected.priority_limit >= hal.scheduler->thread_priority_max()) {
        return nullptr;
    }
    Image image;
    error = Error::Image;
    const Image::SourcePolicy first_policy = source_type == Source::ROMFS ?
                                            Image::SourcePolicy::ROMFSPrelinked : Image::SourcePolicy::Canonical;
    if (!image.parse(source, length, code_size, expected, first_policy)) {
        // Prefer the prelinked policy even when there are no RX fixups: those
        // images are already position independent and may execute from flash.
        // Raw ROMFS images with canonical fixups retain the copied fallback.
        if (source_type != Source::ROMFS ||
            !image.parse(source, length, code_size, expected, Image::SourcePolicy::Canonical)) {
            if (error_detail != nullptr) {
                *error_detail = image.parse_error();
            }
            return nullptr;
        }
    }
    error = Error::Imports;
    if (!image.check_imports(bindings.resolve, bindings.context, missing_import, missing_import_name)) {
        return nullptr;
    }
    error = Error::Memory;
    auto *module = NEW_NOTHROW Module(limits, sink, bindings);
    if (module == nullptr) {
        return nullptr;
    }
    module->_initial_heap = image.initial_heap_size();
    module->_worker_stack = image.required_stack_size() + margin;
    module->_thread_priority = image.thread_priority();
    strncpy(module->_name, name, sizeof(module->_name) - 1);
    if (image.is_split()) {
        const bool arm = expected.machine == 40;
        const bool borrow_rx = arm && source_type == Source::ROMFS && image.can_execute_in_place() &&
                               hal.util->is_executable(image.code_source(), image.code_size());
        if (borrow_rx) {
            module->_rx = const_cast<uint8_t *>(image.code_source());
        } else {
            // Reserve the more restricted executable-capable RAM first.
            // Ordinary RW storage may use memory (for example F405 CCM) from
            // which instructions cannot be fetched.
            const uint32_t executable_size = arm ? image.code_size() : image.memory_size();
            module->_rx_memory = hal.util->allocate_executable(executable_size, image.rx_alignment());
            if (module->_rx_memory == nullptr) {
                delete module;
                return nullptr;
            }
            module->_rx = static_cast<uint8_t *>(module->_rx_memory->data());
        }
        if (arm && image.rw_memory_size() != 0) {
            // RWPI statics may request more than the allocator's natural
            // alignment. Retain the original block for free_type().
            const uint32_t alignment = image.rw_alignment();
            const uint32_t padding = alignment > alignof(uintptr_t) ? alignment - 1 : 0;
            module->_rw_allocation_size = image.rw_memory_size() + padding;
            module->_rw_allocation = hal.util->malloc_type(module->_rw_allocation_size,
                                                            AP_HAL::Util::MEM_FAST);
            if (module->_rw_allocation == nullptr ||
                reinterpret_cast<uintptr_t>(module->_rw_allocation) % alignof(uintptr_t) != 0) {
                delete module;
                return nullptr;
            }
            module->_rw = reinterpret_cast<uint8_t *>((reinterpret_cast<uintptr_t>(module->_rw_allocation) +
                                                      alignment - 1) & ~(uintptr_t(alignment) - 1));
        } else if (!arm) {
            module->_rw = module->_rx + image.code_size();
        }
        error = Error::Publication;
        if (!image.load_split(module->_rx, image.code_size(), module->_rw, image.rw_memory_size(),
                              bindings.resolve, bindings.context) ||
            (module->_rx_memory != nullptr && !module->_rx_memory->publish(image.code_size()))) {
            delete module;
            return nullptr;
        }
    }
    module->_entry = reinterpret_cast<Start>(module->_rx + image.entry_offset());
    module->_checksum = image.checksum();
    error = Error::None;
    return module;
}

Module::~Module()
{
    if (_rw_allocation != nullptr) {
        hal.util->free_type(_rw_allocation, _rw_allocation_size, AP_HAL::Util::MEM_FAST);
    }
    delete _rx_memory;
}

bool Module::start()
{
    if (_started || _stop.load()) {
        return false;
    }
    // Prepare every generation's images before starting its workers. Deferring
    // ordinary heaps until here keeps them out of scarce executable SRAM while
    // those images are being reserved. A failed thread start can reuse its heap.
    if (!_heap_ready) {
        if (!_heap.create(_initial_heap, _limits.allow_heap_expansion, _limits.heap)) {
            return false;
        }
        _heap_ready = true;
    }
    _thread = hal.scheduler->thread_create_owned_guarded(FUNCTOR_BIND_MEMBER(&Module::run, void), _name,
              _worker_stack, AP_HAL::Scheduler::PRIORITY_ABSOLUTE, _thread_priority,
              {this, stack_fault});
    _started = _thread != nullptr;
    return _started;
}

void Module::run()
{
    _result = invoke(_entry, &_api, this, _rw);
    _returned.store(true, std::memory_order_release);
}

#if defined(__ARM_ARCH_7EM__)
__attribute__((naked, noinline)) int32_t Module::invoke(Start entry, const AP_ModuleHostApi *api,
                                                       void *context, void *rw_base)
{
    // The module's RWPI static base lives in r9. Preserve the firmware value
    // and LR across the native call while retaining 8-byte stack alignment.
    // There are no firmware-to-module callbacks: the single initial entry
    // passes this gate. Module-to-firmware callbacks obey AAPCS and therefore
    // preserve r9. A module fatal callback never returns or unwinds this
    // frame, so all module resources remain owned.
    asm volatile(
        "push {r9, lr}\n"
        "mov r12, r0\n"
        "mov r0, r1\n"
        "mov r1, r2\n"
        "mov r9, r3\n"
        "blx r12\n"
        "pop {r9, pc}\n");
}
#else
int32_t Module::invoke(Start entry, const AP_ModuleHostApi *api, void *context, void *rw_base)
{
    (void)rw_base;
    return entry(api, context);
}
#endif

void Module::stack_fault(void *context)
{
    auto &module = *static_cast<Module *>(context);
    module._stack_overflow.store(true, std::memory_order_release);
    module._result = AP_MODULE_STATUS_HOST_ERROR;
    module._returned.store(true, std::memory_order_release);
}

void Module::request_stop()
{
    _stop.store(true);
}

bool Module::poll_join()
{
    return _thread == nullptr || AP_HAL::Scheduler::thread_reap(_thread);
}

bool Module::release(Module *&module)
{
    if (module == nullptr) {
        return true;
    }
    if (!module->poll_join()) {
        return false;
    }
    delete module;
    module = nullptr;
    return true;
}

Module::State Module::state() const
{
    if (_panicked.load(std::memory_order_acquire)) {
        return State::Panicked;
    }
    if (!_started) {
        return State::Prepared;
    }
    if (_thread == nullptr) {
        return State::Joined;
    }
    return _returned.load(std::memory_order_acquire) ? State::Returned : State::Running;
}

int32_t Module::result() const
{
    return _returned.load(std::memory_order_acquire) ? _result : AP_MODULE_STATUS_UNAVAILABLE;
}

const char *Module::panic_message() const
{
    return _panicked.load(std::memory_order_acquire) ? _panic_message : nullptr;
}

Module::Stats Module::stats(bool include_free)
{
    return {_heap.stats(include_free), _initial_heap,
            _rx_memory == nullptr ? 0 : _rx_memory->allocation_size(), _rw_allocation_size,
            _thread == nullptr ? 0 : _thread->stack_size(),
            _thread == nullptr ? 0 : _thread->allocation_size(), sizeof(*this)};
}

uint8_t *Module::allocate(void *context, uint32_t size, uint32_t alignment)
{
    auto &module = *static_cast<Module *>(context);
    // Module destructors and in-flight work can still allocate after stop. A
    // synthetic OOM here would turn cooperative exit into a permanent panic.
    return static_cast<uint8_t *>(module._heap.allocate(size, alignment));
}

void Module::deallocate(void *context, uint8_t *allocation)
{
    auto &module = *static_cast<Module *>(context);
    if (!module._heap.deallocate(allocation)) {
        static const uint8_t message[] = "invalid deallocation";
        panic(context, message, sizeof(message) - 1);
    }
}

uint64_t Module::time_us(void *context)
{
    (void)context;
    return AP_HAL::micros64();
}

int32_t Module::sleep_ms(void *context, uint32_t milliseconds)
{
    auto &module = *static_cast<Module *>(context);
    const uint64_t start = AP_HAL::millis64();
    do {
        if (module._stop.load()) {
            return AP_MODULE_STATUS_STOPPED;
        }
        const uint64_t elapsed = AP_HAL::millis64() - start;
        if (elapsed >= milliseconds) {
            return AP_MODULE_STATUS_OK;
        }
        const uint32_t remaining = milliseconds - uint32_t(elapsed);
        hal.scheduler->delay(remaining > 10 ? 10 : remaining);
    } while (true);
}

uint8_t Module::stop_requested(void *context)
{
    return static_cast<Module *>(context)->_stop.load();
}

int32_t Module::text(void *context, uint8_t severity, const uint8_t *message, uint32_t length)
{
    auto &module = *static_cast<Module *>(context);
    if (module._stop.load()) {
        return AP_MODULE_STATUS_STOPPED;
    }
    if (message == nullptr || severity > 7 || length > 512) {
        return AP_MODULE_STATUS_INVALID;
    }
    if (module._sink.send == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    module._sink.send(module._sink.context, severity, reinterpret_cast<const char *>(message), length);
    return AP_MODULE_STATUS_OK;
}

int32_t Module::binding_call(void *context, uint32_t id, const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size)
{
    auto &module = *static_cast<Module *>(context);
    if (module._stop.load()) {
        return AP_MODULE_STATUS_STOPPED;
    }
    if (module._bindings.call == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    return module._bindings.call(module._bindings.context, static_cast<BindingId>(id),
                                 input, input_size, output, output_size);
}

void Module::panic(void *context, const uint8_t *message, uint32_t length)
{
    auto &module = *static_cast<Module *>(context);
    const uint32_t count = length < sizeof(module._panic_message) - 1 ? length : sizeof(module._panic_message) - 1;
    if (message != nullptr) {
        memcpy(module._panic_message, message, count);
    }
    module._panicked.store(true, std::memory_order_release);
    // This callback never returns. In particular it must not use sleep_ms(),
    // which returns immediately after a stop request. Keep module frames alive
    // at its declared priority, without polling the simulation clock on SITL.
    while (true) {
        hal.scheduler->thread_park();
    }
}

}
#endif
