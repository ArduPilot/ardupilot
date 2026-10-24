// SPDX-License-Identifier: GPL-3.0-or-later
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include <AP_Module/Image.h>
#include <AP_Module/Heap.h>

#if AP_MODULE_ENABLED && defined(__linux__) && defined(__x86_64__)
#include <memory>
#include <vector>
#endif

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if AP_MODULE_ENABLED
namespace
{

struct Container {
    uint8_t bytes[56 + 4104 + 8] {};

    void put(uint32_t offset, uint32_t value)
    {
        for (uint8_t i = 0; i < 4; i++) {
            bytes[offset + i] = uint8_t(value >> (8 * i));
        }
    }

    void checksum()
    {
        put(52, 0);
        put(52, crc32_small(UINT32_MAX, bytes, sizeof(bytes)) ^ UINT32_MAX);
    }

    Container()
    {
        memcpy(bytes, "APMOD01\0", 8);
        put(8, 1);
        put(12, sizeof(uintptr_t) == 8 ? 62 : 40);
        put(16, sizeof(uintptr_t));
        put(20, 4096);
        put(24, 4104);
        put(28, 4120);
        put(32, 4096);
        put(36, sizeof(uintptr_t) == 8 ? 0 : 1);
        put(40, 1);
        bytes[56] = 0xA5;
        put(56 + 4104, 4096);
        put(56 + 4104 + 4, 8);
        checksum();
    }

#if AP_MODULE_LEGACY_PROBE_ENABLED
    bool parse(AP_Module::Image &image, uint32_t limit = 8192)
    {
        return image.parse_probe(bytes, sizeof(bytes), limit, sizeof(uintptr_t) == 8 ? 62 : 40);
    }
#endif
};

struct RuntimeContainer {
    uint8_t bytes[88 + 4104 + 4] {};

    static AP_Module::Image::Compatibility compatibility()
    {
        return {sizeof(uintptr_t) == 8 ? 62U : 40U, 1,
                sizeof(uintptr_t) == 8 ? 2U : 1U, 8192,
        {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}};
    }

    void put(uint32_t offset, uint32_t value)
    {
        for (uint8_t i = 0; i < 4; i++) {
            bytes[offset + i] = uint8_t(value >> (8 * i));
        }
    }

    void checksum()
    {
        put(84, 0);
        put(84, crc32_small(UINT32_MAX, bytes, sizeof(bytes)) ^ UINT32_MAX);
    }

    RuntimeContainer()
    {
        Container probe;
        memcpy(bytes, probe.bytes, 44);
        memcpy(bytes, "APMOD07\0", 8);
        put(8, 7);
        put(20, sizeof(uintptr_t) | sizeof(uintptr_t) << 16);
        const auto expected = compatibility();
        put(44, expected.abi_version);
        put(48, expected.cpu_profile);
        put(52, 4096);
        put(56, 32768);
        put(80, sizeof(uintptr_t) == 4 ? 2 : 1);
        memcpy(bytes + 64, expected.fingerprint, sizeof(expected.fingerprint));
        memcpy(bytes + 88, probe.bytes + 56, 4104);
        put(88 + 4096, 8);
        put(88 + 4104, 4096);
        checksum();
    }
};

struct alignas(uintptr_t) SplitContainer {
    static constexpr uint32_t header_size = 88;
    static constexpr uint32_t code_size = 4096;
    static constexpr uint32_t file_size = code_size + 2 * sizeof(uintptr_t);
    static constexpr uint32_t memory_size = file_size + 16;
    static constexpr uint32_t fixup_count = 2;
    uint8_t bytes[header_size + file_size + fixup_count * 4] {};

    void put(uint32_t offset, uint32_t value)
    {
        for (uint8_t i = 0; i < 4; i++) {
            bytes[offset + i] = uint8_t(value >> (8 * i));
        }
    }

    void put_pointer(uint32_t offset, uintptr_t value)
    {
        for (uint8_t i = 0; i < sizeof(uintptr_t); i++) {
            bytes[offset + i] = uint8_t(value >> (8 * i));
        }
    }

    void checksum()
    {
        put(84, 0);
        put(84, crc32_small(UINT32_MAX, bytes, sizeof(bytes)) ^ UINT32_MAX);
    }

    SplitContainer()
    {
        memcpy(bytes, "APMOD07\0", 8);
        put(8, 7);
        const auto expected = RuntimeContainer::compatibility();
        put(12, expected.machine);
        put(16, sizeof(uintptr_t));
        put(20, sizeof(uintptr_t) | sizeof(uintptr_t) << 16);
        put(24, file_size);
        put(28, memory_size);
        put(32, code_size);
        put(36, 1);
        put(40, fixup_count);
        put(44, expected.abi_version);
        put(48, expected.cpu_profile);
        put(52, 4096);
        put(56, 32768);
        put(80, sizeof(uintptr_t) == 4 ? 2 : 1);
        memcpy(bytes + 64, expected.fingerprint, sizeof(expected.fingerprint));
        bytes[header_size] = 0xA5;
        put_pointer(header_size + sizeof(uintptr_t), 17);
        put_pointer(header_size + code_size, 17);
        const uint32_t fixups = header_size + file_size;
        put(fixups, sizeof(uintptr_t));
        put(fixups + 4, code_size);
        checksum();
    }

    void prelink()
    {
        const uintptr_t value = reinterpret_cast<uintptr_t>(bytes + header_size) + 17;
        put_pointer(header_size + sizeof(uintptr_t), value);
    }
};

struct ImportContainer {
    uint8_t bytes[88 + 4112 + 4 + 32 + 21] {};

    void put(uint32_t offset, uint32_t value)
    {
        for (uint8_t i = 0; i < 4; i++) {
            bytes[offset + i] = uint8_t(value >> (8 * i));
        }
    }

    void checksum()
    {
        put(84, 0);
        put(84, crc32_small(UINT32_MAX, bytes, sizeof(bytes)) ^ UINT32_MAX);
    }

    ImportContainer()
    {
        RuntimeContainer base;
        memcpy(bytes, base.bytes, 88 + 4104);
        put(24, 4112);
        put(28, 4128);
        put(60, 2);
        put(88 + 4104, UINT32_MAX);
        put(88 + 4108, UINT32_MAX);
        put(88 + 4112, 4096);
        const uint32_t records = 88 + 4112 + 4;
        put(records, 0x55667788);
        put(records + 4, 0x11223344);
        put(records + 8, 4104);
        put(records + 12, 0);
        put(records + 16, 0x44332211);
        put(records + 20, 0x88776655);
        put(records + 24, 4108);
        put(records + 28, 10);
        memcpy(bytes + records + 32, "first.api\0second.api", 21);
        checksum();
    }

    static uint32_t resolve(void *context, uint64_t hash)
    {
        const uint32_t offset = *static_cast<uint32_t *>(context);
        if (hash == UINT64_C(0x1122334455667788)) {
            return offset + 77;
        }
        if (hash == UINT64_C(0x8877665544332211) && offset != UINT16_MAX) {
            return offset + 88;
        }
        return UINT32_MAX;
    }
};

struct SplitImportContainer {
    static constexpr uint32_t header_size = 88;
    static constexpr uint32_t code_size = 4096;
    static constexpr uint32_t file_size = 4104;
    static constexpr uint32_t memory_size = 4112;
    static constexpr uint32_t record = header_size + file_size;
    uint8_t bytes[header_size + file_size + 16 + 10] {};

    void put(uint32_t offset, uint32_t value)
    {
        for (uint8_t i = 0; i < 4; i++) {
            bytes[offset + i] = uint8_t(value >> (8 * i));
        }
    }

    void checksum()
    {
        put(84, 0);
        put(84, crc32_small(UINT32_MAX, bytes, sizeof(bytes)) ^ UINT32_MAX);
    }

    SplitImportContainer()
    {
        memcpy(bytes, "APMOD07\0", 8);
        put(8, 7);
        const auto expected = RuntimeContainer::compatibility();
        put(12, expected.machine);
        put(16, sizeof(uintptr_t));
        put(20, sizeof(uintptr_t) | sizeof(uintptr_t) << 16);
        put(24, file_size);
        put(28, memory_size);
        put(32, code_size);
        put(36, 1);
        put(44, expected.abi_version);
        put(48, expected.cpu_profile);
        put(52, 4096);
        put(60, 1);
        put(56, 32768);
        put(80, sizeof(uintptr_t) == 4 ? 2 : 1);
        memcpy(bytes + 64, expected.fingerprint, sizeof(expected.fingerprint));
        put(header_size + code_size, UINT32_MAX);
        put(record, 0x55667788);
        put(record + 4, 0x11223344);
        put(record + 8, code_size);
        memcpy(bytes + record + 16, "first.api", 10);
        checksum();
    }
};

}

TEST(RustImage, ResolveImportsBeforePublication)
{
    ImportContainer container;
    AP_Module::Image image;
    ASSERT_TRUE(image.parse(container.bytes, sizeof(container.bytes), 8192, RuntimeContainer::compatibility()));
    alignas(4096) uint8_t destination[8192];
    memset(destination, 0x5A, sizeof(destination));
    EXPECT_FALSE(image.load_split(destination, 4096, destination + 4096, sizeof(destination) - 4096));
    uint32_t offset = UINT16_MAX; // first known, second unknown
    uint64_t missing = 0;
    const char *missing_name = nullptr;
    EXPECT_FALSE(image.check_imports(ImportContainer::resolve, &offset, &missing, &missing_name));
    EXPECT_EQ(missing, UINT64_C(0x8877665544332211));
    EXPECT_STREQ(missing_name, "second.api");
    EXPECT_FALSE(image.load_split(destination, 4096, destination + 4096, sizeof(destination) - 4096, ImportContainer::resolve, &offset));
    for (const auto byte : destination) {
        ASSERT_EQ(byte, 0x5A);
    }
    // Exactly the same source image works with changed firmware-private IDs.
    for (const uint32_t firmware_offset : {
             0U, 1000U
         }) {
        offset = firmware_offset;
        ASSERT_TRUE(image.load_split(destination, 4096, destination + 4096, sizeof(destination) - 4096, ImportContainer::resolve, &offset));
        uint32_t first, second;
        memcpy(&first, destination + 4104, 4);
        memcpy(&second, destination + 4108, 4);
        EXPECT_EQ(first, offset + 77);
        EXPECT_EQ(second, offset + 88);
        EXPECT_EQ(destination[4112], 0);
    }
}

TEST(RustImage, RejectImportMetadataErrors)
{
    const uint32_t records = 88 + 4112 + 4;
    const struct {
        uint32_t offset;
        uint32_t value;
    } mutations[] = {
        {60, 0}, {60, UINT32_MAX}, {60, 4097},
        {records + 8, 4092}, {records + 8, 4096}, {records + 8, 4105},
        {records + 12, 1}, {records + 12, UINT32_MAX},
        {records + 24, 4104}, {records + 24, 4112}, {records + 24, UINT32_MAX},
        {records + 28, 0}, {records + 28, 9}, {records + 28, UINT32_MAX},
        {88 + 4104, 0}, {88 + 4108, 0},
    };
    AP_Module::Image image;
    for (const auto &mutation : mutations) {
        ImportContainer container;
        container.put(mutation.offset, mutation.value);
        container.checksum();
        EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192, RuntimeContainer::compatibility()));
    }
    ImportContainer duplicate;
    memcpy(duplicate.bytes + records + 16, duplicate.bytes + records, 8);
    duplicate.checksum();
    EXPECT_FALSE(image.parse(duplicate.bytes, sizeof(duplicate.bytes), 8192, RuntimeContainer::compatibility()));
    ImportContainer invalid_name;
    invalid_name.bytes[records + 32] = '-';
    invalid_name.checksum();
    EXPECT_FALSE(image.parse(invalid_name.bytes, sizeof(invalid_name.bytes), 8192,
                             RuntimeContainer::compatibility()));
    ImportContainer corrupt;
    corrupt.bytes[records] ^= 1;
    EXPECT_FALSE(image.parse(corrupt.bytes, sizeof(corrupt.bytes), 8192, RuntimeContainer::compatibility()));
    ImportContainer old;
    memcpy(old.bytes, "APRUST03", 8);
    old.put(8, 3);
    old.checksum();
    EXPECT_FALSE(image.parse(old.bytes, sizeof(old.bytes), 8192, RuntimeContainer::compatibility()));
}

TEST(RustImage, RequireRuntimeCompatibility)
{
    AP_Module::Image image;
    auto expected = RuntimeContainer::compatibility();
    Container probe;
    EXPECT_FALSE(image.parse(probe.bytes, sizeof(probe.bytes), 8192, expected));
    RuntimeContainer container;
#if AP_MODULE_LEGACY_PROBE_ENABLED
    EXPECT_FALSE(image.parse_probe(container.bytes, sizeof(container.bytes), 8192, expected.machine));
#endif
    ASSERT_TRUE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
    EXPECT_EQ(image.required_stack_size(), 4096U);
    alignas(4096) uint8_t destination[8192];
    ASSERT_TRUE(image.load_split(destination, 4096, destination + 4096, sizeof(destination) - 4096));
    EXPECT_EQ(destination[0], 0xA5);
    uintptr_t pointer;
    memcpy(&pointer, destination + 4096, sizeof(pointer));
    EXPECT_EQ(pointer, reinterpret_cast<uintptr_t>(destination + 8));
    EXPECT_EQ(destination[4104], 0);

    expected.stack_limit = 4095;
    EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
    EXPECT_FALSE(image.load_split(destination, 4096, destination + 4096, sizeof(destination) - 4096));
    expected.stack_limit = 4096;
    EXPECT_TRUE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
    expected.fingerprint[15] ^= 1;
    EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
}

TEST(RustImage, ParseFailureDiagnostics)
{
    const auto expected = RuntimeContainer::compatibility();
    const struct {
        uint32_t offset;
        uint32_t value;
        const char *reason;
    } mutations[] = {
        {0, 0, "unsupported image format"},
        {12, 0, "wrong target architecture"},
        {44, 0, "ABI version mismatch"},
        {48, 0, "CPU profile mismatch"},
        {52, 0, "invalid stack requirement"},
        {52, 8193, "stack exceeds MOD_STACK_MAX"},
        {64, 0, "API fingerprint mismatch"},
        {28, 8193, "image exceeds MOD_CODE_MAXSIZE"},
        {16, 0, "invalid image layout"},
        {40, 2, "truncated relocation/import tables"},
        {88 + 4104, 4103, "invalid relocation offset"},
        {88 + 4096, 4121, "invalid relocation target"},
        {84, 0, "image checksum mismatch"},
    };
    AP_Module::Image image;
    for (const auto &mutation : mutations) {
        RuntimeContainer container;
        container.put(mutation.offset, mutation.value);
        EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected)) << mutation.offset;
        EXPECT_STREQ(image.parse_error(), mutation.reason) << mutation.offset;
    }
    RuntimeContainer container;
    EXPECT_FALSE(image.parse(container.bytes, 79, 8192, expected));
    EXPECT_STREQ(image.parse_error(), "truncated image header");
    ASSERT_TRUE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
    EXPECT_EQ(image.parse_error(), nullptr);
}

TEST(RustImage, LoadSplitRawAndPrelinkedImages)
{
    SplitContainer raw;
    AP_Module::Image image;
    ASSERT_TRUE(image.parse(raw.bytes, sizeof(raw.bytes), 8192, RuntimeContainer::compatibility()));
    const uint32_t raw_checksum = image.checksum();
    EXPECT_EQ(raw_checksum, crc32_small(UINT32_MAX, raw.bytes, sizeof(raw.bytes)) ^ UINT32_MAX);
    EXPECT_TRUE(image.is_split());
    EXPECT_FALSE(image.can_execute_in_place());
    EXPECT_EQ(image.code_source(), raw.bytes + SplitContainer::header_size);
    EXPECT_EQ(image.code_size(), uint32_t(SplitContainer::code_size));
    EXPECT_EQ(image.rw_file_size(), 2U * sizeof(uintptr_t));
    EXPECT_EQ(image.rw_memory_size(), 2U * sizeof(uintptr_t) + 16U);

    alignas(4096) uint8_t memory[SplitContainer::code_size + 4096];
    uint8_t *const rx = memory;
    uint8_t *const rw = memory + SplitContainer::code_size;
    memset(memory, 0x5A, sizeof(memory));
    ASSERT_TRUE(image.load_split(rx, SplitContainer::code_size, rw, 4096));
    EXPECT_EQ(rx[0], 0xA5);
    uintptr_t rx_pointer;
    uintptr_t rw_pointer;
    memcpy(&rx_pointer, rx + sizeof(uintptr_t), sizeof(rx_pointer));
    memcpy(&rw_pointer, rw, sizeof(rw_pointer));
    EXPECT_EQ(rx_pointer, reinterpret_cast<uintptr_t>(rx) + 17);
    EXPECT_EQ(rw_pointer, reinterpret_cast<uintptr_t>(rx) + 17);
    for (uint32_t i = image.rw_file_size(); i < image.rw_memory_size(); i++) {
        EXPECT_EQ(rw[i], 0);
    }
    EXPECT_FALSE(image.load_split(rx, SplitContainer::code_size - 1, rw, 4096));
    EXPECT_FALSE(image.load_split(rx, SplitContainer::code_size, rw, image.rw_memory_size() - 1));
    EXPECT_FALSE(image.load_split(rx + 1, SplitContainer::code_size, rw + 1, 4095));
    EXPECT_FALSE(image.load_split(const_cast<uint8_t *>(image.code_source()), image.code_size(), rw, 4096));

    if (RuntimeContainer::compatibility().machine == 62) {
        // x86 remains contiguous but may contain absolute RX slots which
        // target RW. ARM ROPI explicitly forbids this case.
        SplitContainer rx_to_rw;
        rx_to_rw.put_pointer(SplitContainer::header_size + sizeof(uintptr_t), SplitContainer::code_size);
        rx_to_rw.checksum();
        ASSERT_TRUE(image.parse(rx_to_rw.bytes, sizeof(rx_to_rw.bytes), 8192,
                                RuntimeContainer::compatibility()));
        ASSERT_TRUE(image.load_split(rx, SplitContainer::code_size, rw, 4096));
        memcpy(&rx_pointer, rx + sizeof(uintptr_t), sizeof(rx_pointer));
        EXPECT_EQ(rx_pointer, reinterpret_cast<uintptr_t>(rw));
    }

    SplitContainer prelinked;
    prelinked.prelink();
    EXPECT_FALSE(image.parse(prelinked.bytes, sizeof(prelinked.bytes), 8192,
                             RuntimeContainer::compatibility()));
    ASSERT_TRUE(image.parse(prelinked.bytes, sizeof(prelinked.bytes), 8192, RuntimeContainer::compatibility(),
                            AP_Module::Image::SourcePolicy::ROMFSPrelinked));
    EXPECT_EQ(image.can_execute_in_place(), sizeof(uintptr_t) == 4);
    EXPECT_EQ(image.checksum(), raw_checksum);
    memset(rw, 0x5A, 4096);
    if (image.can_execute_in_place()) {
        alignas(uintptr_t) uint8_t direct_rw[64];
        memset(direct_rw, 0x5A, sizeof(direct_rw));
        ASSERT_TRUE(image.load_split(const_cast<uint8_t *>(image.code_source()), image.code_size(),
                                     direct_rw, sizeof(direct_rw)));
        memcpy(&rw_pointer, direct_rw, sizeof(rw_pointer));
        EXPECT_EQ(rw_pointer, reinterpret_cast<uintptr_t>(image.code_source()) + 17);
        uintptr_t source_pointer;
        memcpy(&source_pointer, image.code_source() + sizeof(uintptr_t), sizeof(source_pointer));
        EXPECT_EQ(source_pointer, reinterpret_cast<uintptr_t>(image.code_source()) + 17);
    }

    // A prelinked source can also fall back to copied executable storage. RX
    // fixups must then be rebased rather than retaining the ROMFS address.
    memset(memory, 0x5A, sizeof(memory));
    ASSERT_TRUE(image.load_split(rx, SplitContainer::code_size, rw, 4096));
    memcpy(&rx_pointer, rx + sizeof(uintptr_t), sizeof(rx_pointer));
    EXPECT_EQ(rx_pointer, reinterpret_cast<uintptr_t>(rx) + 17);
}

TEST(RustImage, RejectMalformedSplitImages)
{
    const uint32_t fixups = SplitContainer::header_size + SplitContainer::file_size;
    const struct {
        uint32_t offset;
        uint32_t value;
    } mutations[] = {
        {20, 0},
        {20, uint32_t(sizeof(uintptr_t)) / 2},
        {20, 24},
        {20, 8192},
        {20, uint32_t(sizeof(uintptr_t)) | 24U << 16},
        {20, uint32_t(sizeof(uintptr_t)) | 8192U << 16},
        {24, SplitContainer::code_size - 1},
        {28, SplitContainer::file_size - 1},
        {32, SplitContainer::code_size - 1},
        {fixups, SplitContainer::code_size - uint32_t(sizeof(uintptr_t)) + 1},
        {fixups + 4, SplitContainer::file_size - uint32_t(sizeof(uintptr_t)) + 1},
    };
    AP_Module::Image image;
    for (const auto &mutation : mutations) {
        SplitContainer container;
        container.put(mutation.offset, mutation.value);
        container.checksum();
        EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192,
                                 RuntimeContainer::compatibility())) << mutation.offset;
    }

    if (sizeof(uintptr_t) == 8) {
        // A word-aligned but non-page RX boundary remains invalid on POSIX:
        // otherwise mprotect could make the first RW bytes read-only too.
        SplitContainer short_page;
        short_page.put(32, SplitContainer::code_size - sizeof(uintptr_t));
        short_page.checksum();
        EXPECT_FALSE(image.parse(short_page.bytes, sizeof(short_page.bytes), 8192,
                                 RuntimeContainer::compatibility()));
    }

    SplitContainer nonzero;
    nonzero.put_pointer(SplitContainer::header_size + SplitContainer::code_size,
                        SplitContainer::memory_size + 1);
    nonzero.checksum();
    EXPECT_FALSE(image.parse(nonzero.bytes, sizeof(nonzero.bytes), 8192,
                             RuntimeContainer::compatibility()));

    SplitContainer bad_prelink;
    bad_prelink.prelink();
    bad_prelink.bytes[SplitContainer::header_size + sizeof(uintptr_t)] ^= 1;
    EXPECT_FALSE(image.parse(bad_prelink.bytes, sizeof(bad_prelink.bytes), 8192,
                             RuntimeContainer::compatibility(),
                             AP_Module::Image::SourcePolicy::ROMFSPrelinked));

    // The relocation metadata remains covered by CRC even though its image
    // slot is canonicalised for a prelinked source.
    SplitContainer changed_table;
    changed_table.prelink();
    changed_table.put(fixups + 4, SplitContainer::code_size + sizeof(uintptr_t));
    EXPECT_FALSE(image.parse(changed_table.bytes, sizeof(changed_table.bytes), 8192,
                             RuntimeContainer::compatibility(),
                             AP_Module::Image::SourcePolicy::ROMFSPrelinked));
}

TEST(RustImage, SplitRwAlignmentAndOddDataPointers)
{
    SplitContainer container;
    container.put(20, 64 | uint32_t(sizeof(uintptr_t)) << 16);
    container.put_pointer(SplitContainer::header_size + SplitContainer::code_size,
                          SplitContainer::code_size + 1);
    container.checksum();
    AP_Module::Image image;
    ASSERT_TRUE(image.parse(container.bytes, sizeof(container.bytes), 8192,
                            RuntimeContainer::compatibility()));
    EXPECT_EQ(image.rw_alignment(), 64U);
    alignas(4096) uint8_t memory[8192];
    ASSERT_TRUE(image.load_split(memory, 4096, memory + 4096, 4096));
    uintptr_t pointer;
    memcpy(&pointer, memory + 4096, sizeof(pointer));
    EXPECT_EQ(pointer, reinterpret_cast<uintptr_t>(memory + 4097));
    EXPECT_FALSE(image.load_split(memory, 4096, memory + 4096 + sizeof(uintptr_t),
                                 4096 - sizeof(uintptr_t)));
}

TEST(RustImage, PrelinkedWithoutRxFixups)
{
    SplitContainer container;
    const uint32_t table = SplitContainer::header_size + SplitContainer::file_size;
    // Both relocations are RW-resident, so canonical and prelinked containers
    // have identical bytes. The source policy must still allow ARM XIP.
    container.put(table, SplitContainer::code_size);
    container.put(table + 4, SplitContainer::code_size + sizeof(uintptr_t));
    container.put_pointer(SplitContainer::header_size + SplitContainer::code_size + sizeof(uintptr_t),
                          SplitContainer::code_size + 1);
    container.checksum();
    AP_Module::Image image;
    ASSERT_TRUE(image.parse(container.bytes, sizeof(container.bytes), 8192,
                            RuntimeContainer::compatibility(), AP_Module::Image::SourcePolicy::ROMFSPrelinked));
    EXPECT_EQ(image.can_execute_in_place(), sizeof(uintptr_t) == 4);
    if (image.can_execute_in_place()) {
        alignas(uintptr_t) uint8_t rw[64];
        ASSERT_TRUE(image.load_split(const_cast<uint8_t *>(image.code_source()), image.code_size(), rw, sizeof(rw)));
        uintptr_t pointer;
        memcpy(&pointer, rw + sizeof(uintptr_t), sizeof(pointer));
        EXPECT_EQ(pointer, reinterpret_cast<uintptr_t>(rw + 1));
    }
}

TEST(RustImage, SplitImportsRemainInRw)
{
    SplitImportContainer container;
    AP_Module::Image image;
    ASSERT_TRUE(image.parse(container.bytes, sizeof(container.bytes), 8192,
                            RuntimeContainer::compatibility()));
    alignas(4096) uint8_t memory[8192];
    uint32_t firmware_offset = 1000;
    ASSERT_TRUE(image.load_split(memory, SplitImportContainer::code_size,
                                 memory + SplitImportContainer::code_size, 4096,
                                 ImportContainer::resolve, &firmware_offset));
    uint32_t slot;
    memcpy(&slot, memory + SplitImportContainer::code_size, sizeof(slot));
    EXPECT_EQ(slot, firmware_offset + 77);

    container.put(SplitImportContainer::record + 8, SplitImportContainer::code_size - 4);
    container.put(SplitImportContainer::header_size + SplitImportContainer::code_size - 4, UINT32_MAX);
    container.checksum();
    EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192,
                             RuntimeContainer::compatibility()));
}

TEST(RustImage, AcceptByteGranularStackRequirements)
{
    auto expected = RuntimeContainer::compatibility();
    for (const uint32_t size : {
             256U, 257U, 4095U, 4097U, 16U * 1024 * 1024
         }) {
        RuntimeContainer container;
        container.put(52, size);
        container.checksum();
        AP_Module::Image image;
        expected.stack_limit = size;
        ASSERT_TRUE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
        EXPECT_EQ(image.required_stack_size(), size);
        expected.stack_limit = size - 1;
        EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
    }
}

TEST(RustImage, RejectRuntimeMetadataErrors)
{
    const auto expected = RuntimeContainer::compatibility();
    const struct {
        uint32_t offset;
        uint32_t value;
    } mutations[] = {
        {8, 1}, {44, 0}, {44, 2}, {48, 0}, {48, 3U - expected.cpu_profile},
        {52, 0}, {52, 255}, {52, 8193}, {52, UINT32_MAX}, {60, 1},
        {64, 0}, {88 + 4104, 4103}, {88 + 4096, 4121}, {60, UINT32_MAX},
    };
    AP_Module::Image image;
    for (const auto &mutation : mutations) {
        RuntimeContainer container;
        container.put(mutation.offset, mutation.value);
        container.checksum();
        EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected)) << mutation.offset;
    }
    RuntimeContainer container;
    for (const size_t length : {
             size_t(0), size_t(55), size_t(79), sizeof(container.bytes) - 1
         }) {
        EXPECT_FALSE(image.parse(container.bytes, length, 8192, expected));
    }
    container.bytes[88] ^= 1;
    EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
    EXPECT_FALSE(image.parse(nullptr, sizeof(container.bytes), 8192, expected));
}

TEST(ModuleImage, CpuProfileCapabilities)
{
    for (const uint32_t available : {0U, 1U, 2U, 3U, 4U, UINT32_MAX}) {
        for (const uint32_t required : {0U, 1U, 2U, 3U, 4U, UINT32_MAX}) {
            const bool arm = (required == 1 && (available == 1 || available == 3)) ||
                             (required == 3 && available == 3);
            EXPECT_EQ(AP_Module::Image::supports_cpu_profile(40, required, available), arm);
            EXPECT_EQ(AP_Module::Image::supports_cpu_profile(62, required, available),
                      required == 2 && available == 2);
            EXPECT_FALSE(AP_Module::Image::supports_cpu_profile(0, required, available));
        }
    }
}

TEST(ModuleImage, RejectM7OnM4BeforeImageAllocation)
{
    RuntimeContainer container;
    auto expected = RuntimeContainer::compatibility();
    expected.machine = 40;
    expected.cpu_profile = 1;
    container.put(12, 40);
    container.put(48, 3);
    container.checksum();
    AP_Module::Image image;
    EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
    EXPECT_STREQ(image.parse_error(), "requires Cortex-M7 with double-precision FPU");
}

TEST(ModuleImage, InitialHeapAdmissionAndOldFormats)
{
    RuntimeContainer container;
    auto expected = RuntimeContainer::compatibility();
    AP_Module::Image image;
    for (const uint32_t requested : {256U, 257U, 32768U, 16U * 1024 * 1024}) {
        container.put(56, requested);
        container.checksum();
        expected.heap_limit = requested;
        ASSERT_TRUE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
        EXPECT_EQ(image.initial_heap_size(), requested);
        expected.heap_limit = requested - 1;
        EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
        EXPECT_STREQ(image.parse_error(), "initial heap exceeds MOD_HEAP_MAX");
    }
    expected.heap_limit = 32768;
    for (const uint32_t invalid : {0U, 255U, 16U * 1024 * 1024 + 1, UINT32_MAX}) {
        container.put(56, invalid);
        container.checksum();
        EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
        EXPECT_STREQ(image.parse_error(), "invalid initial heap requirement");
    }
    container.put(56, 32768);
    container.checksum();
    container.put(56, 32767);
    EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
    EXPECT_STREQ(image.parse_error(), "image checksum mismatch");
    for (const char *old_magic : {"APRUST01", "APRUST04", "APRUST05", "APMOD06"}) {
        memcpy(container.bytes, old_magic, 8);
        container.checksum();
        EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
        EXPECT_STREQ(image.parse_error(), "unsupported image format");
    }
}

TEST(ModuleImage, ThreadPriorityAdmission)
{
    RuntimeContainer container;
    auto expected = RuntimeContainer::compatibility();
    AP_Module::Image image;
    for (const uint32_t priority : {1U, 2U, 128U, 254U}) {
        container.put(80, priority);
        container.checksum();
        expected.priority_min = 1;
        expected.priority_limit = priority;
        ASSERT_TRUE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
        EXPECT_EQ(image.thread_priority(), priority);
        if (priority > 1) {
            expected.priority_limit = priority - 1;
            EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
            EXPECT_STREQ(image.parse_error(), "thread priority exceeds MOD_THD_PRIO_MAX");
        }
    }
    expected.priority_limit = 254;
    for (const uint32_t priority : {0U, 255U, UINT32_MAX}) {
        container.put(80, priority);
        container.checksum();
        EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
        EXPECT_STREQ(image.parse_error(), "invalid module thread priority");
    }
    expected.priority_min = 2;
    container.put(80, 1);
    container.checksum();
    EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
    EXPECT_STREQ(image.parse_error(), "invalid module thread priority");
}

#if AP_MODULE_LEGACY_PROBE_ENABLED
TEST(RustImage, RelocateAndZeroBss)
{
    Container container;
    AP_Module::Image image;
    ASSERT_TRUE(container.parse(image));
    alignas(4096) uint8_t destination[8192];
    memset(destination, 0xFF, sizeof(destination));
    EXPECT_FALSE(image.load(destination + 1, sizeof(destination) - 1));
    EXPECT_FALSE(image.load(destination, 4000));
    ASSERT_TRUE(image.load(destination, sizeof(destination)));
    EXPECT_EQ(destination[0], 0xA5);
    uintptr_t pointer;
    memcpy(&pointer, destination + 4096, sizeof(pointer));
    EXPECT_EQ(pointer, reinterpret_cast<uintptr_t>(destination + 8));
    for (uint16_t i = 4104; i < 4120; i++) {
        EXPECT_EQ(destination[i], 0);
    }
    EXPECT_EQ(destination[4120], 0xFF);
}

TEST(RustImage, RejectMalformed)
{
    AP_Module::Image image;
    Container container;
    EXPECT_FALSE(container.parse(image, 4119));
    container.bytes[56] ^= 1;
    EXPECT_FALSE(container.parse(image));
    container.checksum();
    EXPECT_TRUE(container.parse(image));
    container.put(56 + 4104, 4103);
    container.checksum();
    EXPECT_FALSE(container.parse(image));
    alignas(4096) uint8_t destination[8192];
    EXPECT_FALSE(image.load(destination, sizeof(destination)));
    container.put(56 + 4104, 4096);
    container.put(56 + 4104 + 4, 4121);
    container.checksum();
    EXPECT_FALSE(container.parse(image));
}

TEST(RustImage, RejectBoundsAndProfileErrors)
{
    const struct {
        uint32_t offset;
        uint32_t value;
    } mutations[] = {
        {0, 0},
        {12, 0},
        {16, sizeof(uintptr_t) == 8 ? 4U : 8U},
        {24, UINT32_MAX},
        {36, UINT32_MAX},
        {56 + 4104, 4104},
        {56 + 4096, 1},
    };
    AP_Module::Image image;
    for (const auto &mutation : mutations) {
        Container container;
        container.put(mutation.offset, mutation.value);
        container.checksum();
        EXPECT_FALSE(container.parse(image)) << mutation.offset;
    }
    alignas(4096) Container container;
    EXPECT_FALSE(image.parse_probe(container.bytes, sizeof(container.bytes) - 1, 8192, sizeof(uintptr_t) == 8 ? 62 : 40));
    ASSERT_TRUE(container.parse(image));
    EXPECT_FALSE(image.load(container.bytes, sizeof(container.bytes)));
    EXPECT_FALSE(image.load(reinterpret_cast<void *>(UINTPTR_MAX & ~uintptr_t(4095)), 8192));
    container.put(56 + 4104, UINT32_MAX);
    alignas(4096) uint8_t destination[8192];
    EXPECT_FALSE(image.load(destination, sizeof(destination)));
}

#endif // AP_MODULE_LEGACY_PROBE_ENABLED

#if HAL_EXECUTABLE_MEMORY_ENABLED && defined(__linux__) && defined(__x86_64__)
namespace
{
AP_Module::Heap *probe_heap;
uint32_t outputs;
uint32_t sleeps;
bool callback_error;

void probe_output(const uint8_t *text, size_t size)
{
    callback_error |= size != 17 || memcmp(text, "Native module probe", 17) != 0;
    outputs++;
}

void probe_sleep(uint32_t ms)
{
    sleeps += ms;
}

void *probe_allocate(size_t size, size_t alignment)
{
    if (size > UINT32_MAX || alignment > UINT32_MAX) {
        return nullptr;
    }
    return probe_heap->allocate(size, alignment);
}

void probe_deallocate(void *pointer, size_t size, size_t alignment)
{
    callback_error |= !probe_heap->deallocate(pointer);
}
}

static void run_toolchain_probe(bool runtime)
{
    const char *path = getenv(runtime ? "AP_MODULE_TEST_RUNTIME_MODULE" : "AP_MODULE_TEST_MODULE");
    if (path == nullptr) {
        GTEST_SKIP() << "Set AP_MODULE_TEST_MODULE and AP_MODULE_TEST_RUNTIME_MODULE to the packed x86-64 probes";
    }
    FILE *file = fopen(path, "rb");
    ASSERT_NE(file, nullptr);
    ASSERT_EQ(fseek(file, 0, SEEK_END), 0);
    const long length = ftell(file);
    ASSERT_GT(length, 0);
    ASSERT_LT(length, 1024 * 1024);
    rewind(file);
    std::vector<uint8_t> source(length);
    const size_t received = fread(source.data(), 1, source.size(), file);
    fclose(file);
    ASSERT_EQ(received, source.size());
    AP_Module::Image image;
    if (runtime) {
        ASSERT_TRUE(image.parse(source.data(), source.size(), 1024 * 1024, RuntimeContainer::compatibility()));
    } else {
#if AP_MODULE_LEGACY_PROBE_ENABLED
        ASSERT_TRUE(image.parse_probe(source.data(), source.size(), 1024 * 1024, 62));
#else
        GTEST_SKIP() << "Requires AP_MODULE_LEGACY_PROBE_ENABLED";
#endif
    }
    AP_Module::Heap heap;
    ASSERT_TRUE(heap.create(32768));
    probe_heap = &heap;
    outputs = sleeps = 0;
    callback_error = false;
    const struct {
        decltype(&probe_output) output;
        decltype(&probe_sleep) sleep;
        decltype(&probe_allocate) allocate;
        decltype(&probe_deallocate) deallocate;
    } host{probe_output, probe_sleep, probe_allocate, probe_deallocate};
    std::unique_ptr<AP_HAL::ExecutableMemory> memory(hal.util->allocate_executable(image.memory_size()));
    ASSERT_NE(memory, nullptr);
    auto *rx = static_cast<uint8_t *>(memory->data());
    const bool loaded = image.is_split() ?
                        image.load_split(rx, image.code_size(), rx + image.code_size(), image.rw_memory_size()) :
                        image.load(rx, memory->size());
    const bool published = loaded && memory->publish(image.code_size());
    EXPECT_TRUE(published);
    if (published) {
        using Start = uint32_t (*)(const void *);
        const auto start = reinterpret_cast<Start>(static_cast<uint8_t *>(memory->data()) + image.entry_offset());
        EXPECT_EQ(start(&host), 31U);
        EXPECT_EQ(start(&host), 26U);
        EXPECT_EQ(outputs, 2U);
        EXPECT_EQ(sleeps, 20U);
        EXPECT_FALSE(callback_error);
        EXPECT_EQ(heap.stats().used, 0U);
        EXPECT_GT(heap.stats().peak, 256U);
    }
    probe_heap = nullptr;
}

TEST(RustImage, ToolchainProbeWithMultiHeap)
{
    run_toolchain_probe(false);
}

TEST(RustImage, ToolchainRuntimeContainerWithMultiHeap)
{
    run_toolchain_probe(true);
}
#elif defined(__linux__) && defined(__x86_64__)
TEST(RustImage, ToolchainProbeWithMultiHeap)
{
    GTEST_SKIP() << "Requires HAL_EXECUTABLE_MEMORY_ENABLED";
}
#endif
#else
TEST(RustImage, RequiresEnabledBuild)
{
    GTEST_SKIP() << "Configure with --extra-hwdef libraries/AP_Module/tests/enabled.hwdef";
}
#endif // AP_MODULE_ENABLED

AP_GTEST_MAIN()
