// SPDX-License-Identifier: GPL-3.0-or-later
#include "Image.h"

#if AP_MODULE_ENABLED
#include <AP_Math/crc.h>
#include <string.h>

namespace AP_Module
{

static constexpr uint32_t PROBE_HEADER_SIZE = 56;
static constexpr uint32_t RUNTIME_HEADER_SIZE = 88;
static constexpr uint32_t ALIGNMENT = 4096;
static constexpr uint32_t MAX_IMAGE = 16U * 1024 * 1024;
static constexpr uint32_t MAX_IMPORTS = 4096;
static constexpr uint32_t IMPORT_RECORD_SIZE = 16;
static constexpr uint32_t MAX_IMPORT_NAME = 63;

uint32_t Image::word(const uint8_t *pointer)
{
    return uint32_t(pointer[0]) | uint32_t(pointer[1]) << 8 |
           uint32_t(pointer[2]) << 16 | uint32_t(pointer[3]) << 24;
}

bool Image::parse(const uint8_t *source, size_t length, uint32_t memory_limit, const Compatibility &compatibility,
                  SourcePolicy policy)
{
    return parse_internal(source, length, memory_limit, compatibility.machine, &compatibility, policy);
}

#if AP_MODULE_LEGACY_PROBE_ENABLED
bool Image::parse_probe(const uint8_t *source, size_t length, uint32_t memory_limit, uint32_t machine)
{
    return parse_internal(source, length, memory_limit, machine, nullptr, SourcePolicy::Canonical);
}
#endif

bool Image::parse_internal(const uint8_t *source, size_t length, uint32_t memory_limit,
                           uint32_t machine, const Compatibility *compatibility, SourcePolicy policy)
{
    *this = Image{};
    const uint32_t header_size = compatibility == nullptr ? PROBE_HEADER_SIZE : RUNTIME_HEADER_SIZE;
    if (source == nullptr || length < header_size) {
        return reject("truncated image header");
    }
    bool split = false;
    if (compatibility == nullptr) {
        if (memcmp(source, "APMOD01\0", 8) != 0 || word(source + 8) != 1 ||
            policy != SourcePolicy::Canonical) {
            return reject("unsupported image format");
        }
    } else if (memcmp(source, "APMOD07\0", 8) == 0 && word(source + 8) == 7) {
        split = true;
    } else {
        return reject("unsupported image format");
    }
    if (word(source + 12) != machine) {
        return reject("wrong target architecture");
    }
    uint32_t stack_size = 0;
    uint32_t heap_size = 0;
    uint32_t thread_priority = 0;
    uint32_t import_count = 0;
    if (compatibility != nullptr) {
        const uint32_t profile = word(source + 48);
        stack_size = word(source + 52);
        heap_size = word(source + 56);
        thread_priority = word(source + 80);
        import_count = word(source + 60);
        const uint8_t empty_fingerprint[16] {};
        if (word(source + 44) == 0 || word(source + 44) != compatibility->abi_version) {
            return reject("ABI version mismatch");
        }
        if (!supports_cpu_profile(machine, profile, compatibility->cpu_profile)) {
            if (machine == 40 && profile == 3 && compatibility->cpu_profile == 1) {
                return reject("requires Cortex-M7 with double-precision FPU");
            }
            return reject("CPU profile mismatch");
        }
        if (stack_size < 256 || stack_size > MAX_IMAGE) {
            return reject("invalid stack requirement");
        }
        if (stack_size > compatibility->stack_limit) {
            return reject("stack exceeds MOD_STACK_MAX");
        }
        if (heap_size < 256 || heap_size > MAX_IMAGE) {
            return reject("invalid initial heap requirement");
        }
        if (compatibility->heap_limit != 0 && heap_size > compatibility->heap_limit) {
            return reject("initial heap exceeds MOD_HEAP_MAX");
        }
        if (thread_priority < 1 || thread_priority > 254 || thread_priority < compatibility->priority_min) {
            return reject("invalid module thread priority");
        }
        if (compatibility->priority_limit != 0 && thread_priority > compatibility->priority_limit) {
            return reject("thread priority exceeds MOD_THD_PRIO_MAX");
        }
        if (memcmp(source + 64, empty_fingerprint, sizeof(empty_fingerprint)) == 0 ||
            memcmp(source + 64, compatibility->fingerprint, sizeof(compatibility->fingerprint)) != 0) {
            return reject("API fingerprint mismatch");
        }
    } else if (word(source + 44) != 0 || word(source + 48) != 0) {
        return reject("invalid probe metadata");
    }
    const uint32_t width = word(source + 16);
    const uint32_t alignment = word(source + 20);
    const uint32_t rw_alignment = split ? alignment & 0xffffU : ALIGNMENT;
    const uint32_t rx_alignment = split ? alignment >> 16 : ALIGNMENT;
    const uint32_t file_size = word(source + 24);
    const uint32_t memory_size = word(source + 28);
    const uint32_t code_size = word(source + 32);
    const uint32_t entry = word(source + 36);
    const uint32_t count = word(source + 40);
    const uint32_t boundary = split && machine == 40 ? 4 : ALIGNMENT;
    if (memory_size > memory_limit) {
        return reject("image exceeds MOD_CODE_MAXSIZE");
    }
    if (!((machine == 40 && width == 4) || (machine == 62 && width == 8)) ||
        width != sizeof(uintptr_t) ||
        (split ? (rw_alignment < width || rw_alignment > ALIGNMENT || (rw_alignment & (rw_alignment - 1)) != 0 ||
                  rx_alignment < width || rx_alignment > ALIGNMENT || (rx_alignment & (rx_alignment - 1)) != 0) :
         alignment != ALIGNMENT) ||
        file_size == 0 ||
        file_size > memory_size || memory_size > MAX_IMAGE ||
        code_size == 0 || code_size % boundary != 0 ||
        code_size > ((memory_size + boundary - 1) & ~(boundary - 1)) ||
        (split && code_size > file_size) ||
        entry >= code_size || entry >= file_size || (machine == 40 && (entry & 1) == 0) ||
        count > file_size / width || import_count > file_size / 4 || import_count > MAX_IMPORTS) {
        return reject("invalid image layout");
    }
    const uint32_t fixup_record_size = split ? 4 : 8;
    const size_t import_names_offset = header_size + size_t(file_size) + size_t(count) * fixup_record_size +
                                       size_t(import_count) * IMPORT_RECORD_SIZE;
    if (import_names_offset > length) {
        return reject("truncated relocation/import tables");
    }
    uint32_t previous_end = 0;
    for (uint32_t i = 0; i < count; i++) {
        const uint8_t *fixup = source + header_size + file_size + i * fixup_record_size;
        const uint32_t offset = word(fixup);
        if (offset < previous_end || offset % width != 0 || file_size < width || offset > file_size - width) {
            return reject("invalid relocation offset");
        }
        const uint8_t *slot = source + header_size + offset;
        uint32_t target;
        if (split && policy == SourcePolicy::ROMFSPrelinked && offset < code_size) {
            uintptr_t value = 0;
            for (uint32_t j = 0; j < width; j++) {
                value |= uintptr_t(slot[j]) << (j * 8);
            }
            const uintptr_t source_rx = reinterpret_cast<uintptr_t>(source + header_size);
            if (value < source_rx || value - source_rx > UINT32_MAX) {
                return reject("invalid ROMFS relocation");
            }
            target = uint32_t(value - source_rx);
        } else if (split) {
            target = word(slot);
            for (uint32_t j = 4; j < width; j++) {
                if (slot[j] != 0) {
                    return reject("invalid relocation slot");
                }
            }
        } else {
            target = word(fixup + 4);
            for (uint32_t j = 0; j < width; j++) {
                if (slot[j] != 0) {
                    return reject("invalid relocation slot");
                }
            }
        }
        // An odd target can be a byte pointer into RW as well as a Thumb
        // function pointer. Segment boundaries are even; keep all target bits.
        if (target > memory_size ||
            (machine == 40 && split && offset < code_size &&
             (offset + width > code_size || target >= code_size))) {
            return reject("invalid relocation target");
        }
        previous_end = offset + width;
    }
    previous_end = code_size;
    uint32_t previous_name_end = 0;
    uint32_t fixup_index = 0;
    const uint8_t *imports = source + header_size + file_size + count * fixup_record_size;
    for (uint32_t i = 0; i < import_count; i++) {
        const uint8_t *record = imports + i * IMPORT_RECORD_SIZE;
        const uint32_t offset = word(record + 8);
        const uint32_t name_offset = word(record + 12);
        if (offset < previous_end || offset % 4 != 0 || file_size < 4 || offset > file_size - 4 ||
            word(source + header_size + offset) != UINT32_MAX || name_offset != previous_name_end ||
            import_names_offset + name_offset >= length) {
            return reject("invalid import record");
        }
        const uint8_t *name = source + import_names_offset + name_offset;
        uint32_t name_length = 0;
        while (import_names_offset + name_offset + name_length < length &&
               name[name_length] != 0 && name_length <= MAX_IMPORT_NAME) {
            const uint8_t c = name[name_length];
            if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                  (c >= '0' && c <= '9') || c == '_' || c == '.')) {
                return reject("invalid import name");
            }
            name_length++;
        }
        if (name_length == 0 || name_length > MAX_IMPORT_NAME ||
            import_names_offset + name_offset + name_length >= length || name[name_length] != 0) {
            return reject("invalid import name");
        }
        previous_name_end = name_offset + name_length + 1;
        for (uint32_t j = 0; j < i; j++) {
            if (memcmp(imports + j * IMPORT_RECORD_SIZE, record, 8) == 0) {
                return reject("duplicate API import");
            }
        }
        // Both lists are offset-sorted. Slots must not overlap pointer fixups.
        while (fixup_index < count &&
               word(source + header_size + file_size + fixup_index * fixup_record_size) + width <= offset) {
            fixup_index++;
        }
        if (fixup_index < count &&
            word(source + header_size + file_size + fixup_index * fixup_record_size) < offset + 4) {
            return reject("import overlaps relocation");
        }
        previous_end = offset + 4;
    }
    if (import_names_offset + previous_name_end != length) {
        return reject("invalid image length");
    }
    // The container checksum describes the unresolved file. ROMFS firmware
    // linking changes only declared fixup slots, so canonicalise exactly those
    // bytes while retaining the relocation table itself in the checksum.
    const uint8_t zeros[8] {};
    auto canonical_crc = [&](bool zero_header_crc) {
        uint32_t crc = crc32_small(UINT32_MAX, source, header_size - 4);
        crc = crc32_small(crc, zero_header_crc ? zeros : source + header_size - 4, 4);
        size_t image_position = 0;
        for (uint32_t i = 0; i < count; i++) {
            const uint8_t *fixup = source + header_size + file_size + i * fixup_record_size;
            const uint32_t offset = word(fixup);
            crc = crc32_small(crc, source + header_size + image_position, offset - image_position);
            if (split) {
                uint32_t target;
                const uint8_t *slot = source + header_size + offset;
                if (policy == SourcePolicy::ROMFSPrelinked && offset < code_size) {
                    uintptr_t value = 0;
                    for (uint32_t j = 0; j < width; j++) {
                        value |= uintptr_t(slot[j]) << (j * 8);
                    }
                    target = uint32_t(value - reinterpret_cast<uintptr_t>(source + header_size));
                } else {
                    target = word(slot);
                }
                uint8_t encoded[8] {};
                for (uint32_t j = 0; j < width; j++) {
                    encoded[j] = j < 4 ? uint8_t(target >> (j * 8)) : 0;
                }
                crc = crc32_small(crc, encoded, width);
            } else {
                crc = crc32_small(crc, zeros, width);
            }
            image_position = offset + width;
        }
        crc = crc32_small(crc, source + header_size + image_position, file_size - image_position);
        crc = crc32_small(crc, source + header_size + file_size, length - header_size - file_size);
        return crc ^ UINT32_MAX;
    };
    const uint32_t crc = canonical_crc(true);
    if (crc != word(source + header_size - 4)) {
        return reject("image checksum mismatch");
    }
    _source = source;
    _length = length;
    _file_size = file_size;
    _memory_size = memory_size;
    _code_size = code_size;
    _entry = entry;
    _count = count;
    _width = width;
    _rw_alignment = rw_alignment;
    _rx_alignment = rx_alignment;
    _header_size = header_size;
    _stack_size = stack_size;
    _heap_size = heap_size;
    _thread_priority = thread_priority;
    _import_count = import_count;
    _import_names_offset = import_names_offset;
    _checksum = canonical_crc(false);
    _machine = machine;
    _source_policy = policy;
    _split = split;
    return true;
}

bool Image::check_imports(Resolver resolver, void *context, uint64_t *missing, const char **missing_name) const
{
    if (_source == nullptr) {
        return false;
    }
    if (missing_name != nullptr) {
        *missing_name = nullptr;
    }
    const uint32_t fixup_record_size = _split ? 4 : 8;
    for (uint32_t i = 0; i < _import_count; i++) {
        const uint8_t *record = _source + _header_size + _file_size + _count * fixup_record_size +
                                i * IMPORT_RECORD_SIZE;
        const uint64_t hash = uint64_t(word(record)) | (uint64_t(word(record + 4)) << 32);
        if (resolver == nullptr || resolver(context, hash) == UINT32_MAX) {
            if (missing != nullptr) {
                *missing = hash;
            }
            if (missing_name != nullptr) {
                *missing_name = reinterpret_cast<const char *>(_source + _import_names_offset + word(record + 12));
            }
            return false;
        }
    }
    return true;
}

bool Image::load(void *destination, size_t size, Resolver resolver, void *context) const
{
    const uintptr_t address = reinterpret_cast<uintptr_t>(destination);
    const uintptr_t source_address = reinterpret_cast<uintptr_t>(_source);
    if (_source == nullptr || _split || destination == nullptr || size < _memory_size || address % ALIGNMENT != 0 ||
        address > UINTPTR_MAX - _memory_size || source_address > UINTPTR_MAX - _length ||
        (address < source_address + _length && source_address < address + _memory_size) ||
        (_import_count != 0 && resolver == nullptr)) {
        return false;
    }
    auto *bytes = static_cast<uint8_t *>(destination);
    // A missing import must reject the image before any output is modified.
    // Resolvers are immutable firmware metadata; the second lookup below must
    // return the same slot. No allocation or per-module dispatch table needed.
    if (!check_imports(resolver, context)) {
        return false;
    }
    memcpy(bytes, _source + _header_size, _file_size);
    memset(bytes + _file_size, 0, _memory_size - _file_size);
    for (uint32_t i = 0; i < _count; i++) {
        const uint8_t *fixup = _source + _header_size + _file_size + i * 8;
        const uint32_t offset = word(fixup);
        const uint32_t target = word(fixup + 4);
        if (_file_size < _width || offset > _file_size - _width || offset % _width != 0 || target > _memory_size) {
            return false;
        }
        const uintptr_t value = address + target;
        for (uint32_t j = 0; j < _width; j++) {
            bytes[offset + j] = uint8_t(value >> (j * 8));
        }
    }
    for (uint32_t i = 0; i < _import_count; i++) {
        const uint8_t *record = _source + _header_size + _file_size + _count * 8 + i * IMPORT_RECORD_SIZE;
        const uint64_t hash = uint64_t(word(record)) | (uint64_t(word(record + 4)) << 32);
        const uint32_t slot = resolver(context, hash);
        if (slot == UINT32_MAX) {
            return false;
        }
        const uint32_t offset = word(record + 8);
        for (uint32_t j = 0; j < 4; j++) {
            bytes[offset + j] = uint8_t(slot >> (j * 8));
        }
    }
    return true;
}

bool Image::load_split(void *rx_base, size_t rx_size, void *rw_base, size_t rw_size,
                       Resolver resolver, void *context) const
{
    if (_source == nullptr || !_split || rx_base == nullptr || rx_size < _code_size ||
        (_memory_size > _code_size && rw_base == nullptr) || rw_size < rw_memory_size() ||
        (_import_count != 0 && resolver == nullptr)) {
        return false;
    }
    const auto *source_rx = _source + _header_size;
    const bool borrowed_rx = rx_base == source_rx;
    if (borrowed_rx && !can_execute_in_place()) {
        return false;
    }
    const uintptr_t rx_address = reinterpret_cast<uintptr_t>(rx_base);
    const uintptr_t rw_address = reinterpret_cast<uintptr_t>(rw_base);
    const uintptr_t source_address = reinterpret_cast<uintptr_t>(_source);
    const uint32_t rx_alignment = _machine == 40 ? _rx_alignment : ALIGNMENT;
    if (rx_address % rx_alignment != 0 ||
        (_memory_size > _code_size && rw_address % _rw_alignment != 0) ||
        (_machine == 62 && rw_address != rx_address + _code_size) ||
        rx_address > UINTPTR_MAX - _code_size ||
        (_memory_size > _code_size && rw_address > UINTPTR_MAX - rw_memory_size()) ||
        source_address > UINTPTR_MAX - _length) {
        return false;
    }
    const auto overlaps = [](uintptr_t first, size_t first_size, uintptr_t second, size_t second_size) {
        return first < second + second_size && second < first + first_size;
    };
    if ((!borrowed_rx && overlaps(rx_address, _code_size, source_address, _length)) ||
        (_memory_size > _code_size && overlaps(rw_address, rw_memory_size(), source_address, _length)) ||
        (_memory_size > _code_size && overlaps(rx_address, _code_size, rw_address, rw_memory_size()))) {
        return false;
    }
    if (!check_imports(resolver, context)) {
        return false;
    }
    auto *rx = static_cast<uint8_t *>(rx_base);
    auto *rw = static_cast<uint8_t *>(rw_base);
    if (!borrowed_rx) {
        memcpy(rx, source_rx, _code_size);
    }
    if (rw_file_size() != 0) {
        memcpy(rw, source_rx + _code_size, rw_file_size());
    }
    if (rw_memory_size() > rw_file_size()) {
        memset(rw + rw_file_size(), 0, rw_memory_size() - rw_file_size());
    }
    for (uint32_t i = 0; i < _count; i++) {
        const uint8_t *fixup = _source + _header_size + _file_size + i * 4;
        const uint32_t offset = word(fixup);
        const uint8_t *source_slot = _source + _header_size + offset;
        uint32_t target;
        if (_source_policy == SourcePolicy::ROMFSPrelinked && offset < _code_size) {
            uintptr_t value = 0;
            for (uint32_t j = 0; j < _width; j++) {
                value |= uintptr_t(source_slot[j]) << (j * 8);
            }
            target = uint32_t(value - reinterpret_cast<uintptr_t>(source_rx));
        } else {
            target = word(source_slot);
        }
        if (borrowed_rx && offset < _code_size) {
            continue;
        }
        uint8_t *slot = offset < _code_size ? rx + offset : rw + offset - _code_size;
        const bool target_rx = target < _code_size ||
                               (target == _code_size && rw_memory_size() == 0);
        const uintptr_t value = target_rx ? rx_address + target : rw_address + target - _code_size;
        for (uint32_t j = 0; j < _width; j++) {
            slot[j] = uint8_t(value >> (j * 8));
        }
    }
    for (uint32_t i = 0; i < _import_count; i++) {
        const uint8_t *record = _source + _header_size + _file_size + _count * 4 + i * IMPORT_RECORD_SIZE;
        const uint64_t hash = uint64_t(word(record)) | (uint64_t(word(record + 4)) << 32);
        const uint32_t slot = resolver(context, hash);
        if (slot == UINT32_MAX) {
            return false;
        }
        const uint32_t offset = word(record + 8) - _code_size;
        for (uint32_t j = 0; j < 4; j++) {
            rw[offset + j] = uint8_t(slot >> (j * 8));
        }
    }
    return true;
}

}
#endif // AP_MODULE_ENABLED
