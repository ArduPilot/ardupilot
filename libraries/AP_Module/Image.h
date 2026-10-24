// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include <stddef.h>
#include <stdint.h>

namespace AP_Module
{

class Image
{
public:
    enum class SourcePolicy : uint8_t {
        // Unmodified container encoding: target offsets remain in fixup slots.
        Canonical,
        // ROMFS has resolved RX-to-RX fixup slots at firmware link time.
        // RW slots retain target offsets and RX bytes have firmware lifetime.
        ROMFSPrelinked,
    };

    struct Compatibility {
        uint32_t machine;
        uint32_t abi_version;
        uint32_t cpu_profile; // Board capability: 1=M4/SP, 2=x86-64, 3=M7/DP.
        uint32_t stack_limit;
        uint8_t fingerprint[16];
        uint32_t heap_limit;
        uint32_t priority_min; // Zero selects the container minimum, 1.
        uint32_t priority_limit; // Zero selects the container maximum, 254.
    };

    // Profile IDs are not ordered: M7/DP also supports the M4/SP baseline,
    // while neither ARM capability may admit x86-64 or unknown profiles.
    static constexpr bool supports_cpu_profile(uint32_t machine, uint32_t required, uint32_t available)
    {
        return (machine == 40 && ((required == 1 && (available == 1 || available == 3)) ||
                                 (required == 3 && available == 3))) ||
               (machine == 62 && required == 2 && available == 2);
    }

    // Source bytes must stay alive and immutable through load(). No allocation
    // or executable-memory permission changes are performed by this class.
    // A failed load may leave partial output; never publish or execute it.
    bool parse(const uint8_t *source, size_t length, uint32_t memory_limit, const Compatibility &compatibility,
               SourcePolicy policy = SourcePolicy::Canonical);
    // Static diagnostic string, valid after the source buffer is released.
    const char *parse_error() const
    {
        return _parse_error;
    }
#if AP_MODULE_LEGACY_PROBE_ENABLED
    // Legacy relocation probes have no API identity. Only test builds expose
    // this path; runtime module discovery must use the strict parse().
    bool parse_probe(const uint8_t *source, size_t length, uint32_t memory_limit, uint32_t machine);
#endif
    using Resolver = uint32_t (*)(void *context, uint64_t hash);
    // missing_name aliases the parsed source buffer.
    bool check_imports(Resolver resolver, void *context = nullptr, uint64_t *missing = nullptr,
                       const char **missing_name = nullptr) const;
    // UINT32_MAX rejects an unknown contract. Resolve only before publication.
    bool load(void *destination, size_t size, Resolver resolver = nullptr, void *context = nullptr) const;
    // Load an APMOD07 split image. rx_base is either writable destination
    // storage or code_source() for a prelinked ROMFS image. RW is always
    // copied, zero-filled and relocated in writable memory.
    bool load_split(void *rx_base, size_t rx_size, void *rw_base, size_t rw_size,
                    Resolver resolver = nullptr, void *context = nullptr) const;

    bool is_split() const
    {
        return _split;
    }
    const uint8_t *code_source() const
    {
        return _split && _source != nullptr ? _source + _header_size : nullptr;
    }
    bool can_execute_in_place() const
    {
        return _split && _machine == 40 && _source_policy == SourcePolicy::ROMFSPrelinked &&
               reinterpret_cast<uintptr_t>(_source + _header_size) % _rx_alignment == 0;
    }
    uint32_t rw_file_size() const
    {
        return _split ? _file_size - _code_size : 0;
    }
    uint32_t rw_memory_size() const
    {
        return _split ? _memory_size - _code_size : 0;
    }
    uint32_t rw_alignment() const
    {
        return _rw_alignment;
    }
    uint32_t rx_alignment() const
    {
        return _rx_alignment;
    }

    uint32_t memory_size() const
    {
        return _memory_size;
    }
    uint32_t code_size() const
    {
        return _code_size;
    }
    uint32_t entry_offset() const
    {
        return _entry;
    }
    // Declared module stack use, not a measured bound. The caller must budget
    // backend overhead (including POSIX TLS/libc) separately. Zero for probes.
    uint32_t required_stack_size() const
    {
        return _stack_size;
    }
    uint32_t initial_heap_size() const { return _heap_size; }
    uint32_t thread_priority() const { return _thread_priority; }
    // Whole-file CRC32 of the canonical unresolved container. This remains
    // identical when ROMFS firmware linking has resolved RX fixup slots.
    uint32_t checksum() const
    {
        return _checksum;
    }

private:
    bool reject(const char *reason)
    {
        _parse_error = reason;
        return false;
    }
    static uint32_t word(const uint8_t *pointer);
    bool parse_internal(const uint8_t *source, size_t length, uint32_t memory_limit,
                        uint32_t machine, const Compatibility *compatibility, SourcePolicy policy);
    const uint8_t *_source{};
    size_t _length{};
    uint32_t _file_size{};
    uint32_t _memory_size{};
    uint32_t _code_size{};
    uint32_t _entry{};
    uint32_t _count{};
    uint32_t _width{};
    uint32_t _header_size{};
    uint32_t _stack_size{};
    uint32_t _heap_size{};
    uint32_t _thread_priority{};
    uint32_t _import_count{};
    uint32_t _import_names_offset{};
    uint32_t _checksum{};
    uint32_t _machine{};
    uint32_t _rw_alignment{};
    uint32_t _rx_alignment{};
    SourcePolicy _source_policy{SourcePolicy::Canonical};
    bool _split{};
    const char *_parse_error{};
};

}
#endif // AP_MODULE_ENABLED
