// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED && HAL_OWNED_THREADS_ENABLED && HAL_EXECUTABLE_MEMORY_ENABLED
#include "Heap.h"
#include <AP_Module/Bindings.h>
#include <AP_Module/HostApi.h>
#include <AP_HAL/AP_HAL.h>
#include <atomic>

namespace AP_Module
{

// Single manager owns this object. Only the host callbacks run on its worker.
// The private destructor prevents deletion without joining the worker first.
class Module
{
public:
    CLASS_NO_COPY(Module);
    struct Limits {
        Limits(uint32_t heap_size = 0, uint32_t code_size = 0, uint32_t stack_size = 0,
               bool allow_expansion = true, uint16_t priority_limit = 0) :
            heap(heap_size), code(code_size), stack(stack_size), allow_heap_expansion(allow_expansion),
            thread_priority_max(priority_limit) {}
        uint32_t heap;
        uint32_t code;
        uint32_t stack;
        bool allow_heap_expansion;
        uint16_t thread_priority_max; // Zero selects the HAL's lowest application priority.
    };
    enum class Error : uint8_t {
        None,
        Limits,
        Target,
        Image,
        Memory,
        Publication,
        Imports,
    };
    enum class State : uint8_t {
        Prepared,
        Running,
        Returned,
        Joined,
        Panicked,
    };
    enum class Source : uint8_t {
        File,
        ROMFS,
    };
    // Firmware callback only, called on the module worker: must be thread-safe,
    // must not retain message, throw, or hold a lock across a return to module code.
    // context must outlive this Module. Budget at most 2 KiB of additional
    // stack for this callback on Cortex-M; queue text for formatting elsewhere.
    struct TextSink {
        void *context;
        void (*send)(void *context, uint8_t severity, const char *message, uint32_t length);
    };
    // Firmware-only delegate; same lifetime and no-retained-pointers contract
    // as TextSink. An absent delegate keeps standalone loaders independent.
    struct BindingSink {
        void *context;
        int32_t (*call)(void *context, BindingId id, const uint8_t *input, uint32_t input_size,
                        uint8_t *output, uint32_t output_size);
        uint32_t (*resolve)(void *context, uint64_t hash);
    };
    struct Stats {
        Heap::Stats heap;
        uint32_t heap_initial;
        // Owned reservations. Borrowed ROMFS RX reports zero; a platform which
        // owns RX and RW in one executable mapping charges it all to RX.
        uint32_t rx_reserved;
        uint32_t rw_reserved;
        uint32_t stack_reserved;
        uint32_t thread_reserved;
        uint32_t metadata;
    };

    // Zero means outside the supported range; otherwise round up safely.
    static uint32_t rounded_code_size(uint32_t requested);
    static const char *error_string(Error error);

    // File source may be released on return. Borrowed ROMFS has firmware
    // lifetime. A returned missing_import_name aliases source and must be
    // consumed before a file source is released. error_detail, when supplied,
    // receives a static string describing an image validation failure.
    static Module *load(const uint8_t *source, size_t length, const char *name,
                        const Limits &limits, TextSink sink, Error &error, BindingSink bindings = {},
                        uint64_t *missing_import = nullptr, const char **missing_import_name = nullptr,
                        Source source_type = Source::File, const char **error_detail = nullptr);
    // Creates the heap and worker. Refuses repeated or stopped starts. Failed
    // creation leaves no worker; the prepared module can be retried or released.
    bool start();
    void request_stop();
    // May join the exit epilogue; call only from the low-priority manager.
    bool poll_join();
    // False leaves the handle and ALL resources intact. Never cancels a worker.
    static bool release(Module *&module);
    State state() const;
    int32_t result() const;
    const char *panic_message() const;
    bool stack_overflowed() const { return _stack_overflow.load(std::memory_order_acquire); }
    const char *name() const
    {
        return _name;
    }
    uint32_t checksum() const
    {
        return _checksum;
    }
    Stats stats(bool include_free = false);
    uint8_t thread_priority() const { return _thread_priority; }
    bool cpu_load(float &percent) const { return _thread != nullptr && _thread->cpu_load(percent); }

private:
    using Start = int32_t (*)(const AP_ModuleHostApi *, void *);
    Module(const Limits &limits, TextSink sink, BindingSink bindings) :
        _limits(limits), _sink(sink), _bindings(bindings) {}
    ~Module();
    void run();
    static int32_t invoke(Start entry, const AP_ModuleHostApi *api, void *context, void *rw_base);
    static void stack_fault(void *context);

    static uint8_t *allocate(void *context, uint32_t size, uint32_t alignment);
    static void deallocate(void *context, uint8_t *allocation);
    static uint64_t time_us(void *context);
    static int32_t sleep_ms(void *context, uint32_t milliseconds);
    static uint8_t stop_requested(void *context);
    static int32_t text(void *context, uint8_t severity, const uint8_t *message, uint32_t length);
    [[noreturn]] static void panic(void *context, const uint8_t *message, uint32_t length);
    static int32_t binding_call(void *context, uint32_t id, const uint8_t *input, uint32_t input_size,
                                uint8_t *output, uint32_t output_size);
    static const AP_ModuleHostApi _api;

    const Limits _limits;
    uint32_t _initial_heap;
    uint32_t _worker_stack;
    uint8_t _thread_priority;
    const TextSink _sink;
    const BindingSink _bindings;
    Heap _heap;
    AP_HAL::ExecutableMemory *_rx_memory{};
    void *_rw_allocation{};
    uint8_t *_rx{};
    uint8_t *_rw{};
    uint32_t _rw_allocation_size{};
    AP_HAL::Scheduler::OwnedThread *_thread{};
    Start _entry{};
    char _name[32] {};
    char _panic_message[193] {};
    uint32_t _checksum{};
    bool _heap_ready{};
    bool _started{};
    std::atomic<bool> _stop{false};
    std::atomic<bool> _returned{false};
    std::atomic<bool> _panicked{false};
    std::atomic<bool> _stack_overflow{false};
    int32_t _result{AP_MODULE_STATUS_HOST_ERROR};
};

}
#endif
