// SPDX-License-Identifier: GPL-3.0-or-later
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if HAL_EXECUTABLE_MEMORY_ENABLED && defined(__linux__)
#include <memory>
#include <signal.h>
#include <unistd.h>

TEST(ExecutableMemory, BoundsAndPublication)
{
#if defined(__x86_64__) && !defined(__ILP32__)
    EXPECT_EQ(hal.util->executable_memory_target(), AP_HAL::ExecutableMemory::Target::X86_64Baseline);
#else
    EXPECT_EQ(hal.util->executable_memory_target(), AP_HAL::ExecutableMemory::Target::Unsupported);
#endif
    EXPECT_EQ(hal.util->allocate_executable(0), nullptr);
    EXPECT_EQ(hal.util->allocate_executable(UINT32_MAX), nullptr);
    EXPECT_EQ(hal.util->allocate_executable(64, 3), nullptr);
    EXPECT_EQ(hal.util->allocate_executable(64, 8192), nullptr);
    EXPECT_FALSE(hal.util->is_executable(nullptr, 1));
    EXPECT_FALSE(hal.util->is_executable(reinterpret_cast<const void *>(UINTPTR_MAX), 2));
    std::unique_ptr<AP_HAL::ExecutableMemory> memory(hal.util->allocate_executable(4097));
    ASSERT_NE(memory, nullptr);
    // POSIX mappings are deliberately not borrowable: the backend owns W^X
    // publication and ROMFS data is not an executable mapping.
    EXPECT_FALSE(hal.util->is_executable(memory->data(), memory->size()));
    EXPECT_EQ(reinterpret_cast<uintptr_t>(memory->data()) % 4096, 0U);
    ASSERT_EQ(memory->size(), 8192U);
    EXPECT_GT(memory->allocation_size(), memory->size());
    auto *bytes = static_cast<uint8_t *>(memory->data());
    for (uint32_t i = 0; i < memory->size(); i++) {
        ASSERT_EQ(bytes[i], 0U);
    }
    EXPECT_FALSE(memory->publish(0));
    EXPECT_FALSE(memory->publish(1));
    EXPECT_FALSE(memory->publish(12288));
    bytes[0] = 0xA5;
    ASSERT_TRUE(memory->publish(4096));
    EXPECT_FALSE(memory->publish(4096));
    EXPECT_EQ(bytes[0], 0xA5);
    bytes[4096] = 0x5A;
    EXPECT_EQ(bytes[4096], 0x5A);
    // Code is no longer writable, while the data page above remains writable.
    EXPECT_EXIT({
        // Test the OS protection fault, not ASan's SIGSEGV-to-exit conversion.
        signal(SIGSEGV, SIG_DFL);
        *static_cast<volatile uint8_t *>(memory->data()) = 0;
        _exit(0);
    }, ::testing::KilledBySignal(SIGSEGV), "");
}

#if defined(__x86_64__)
TEST(ExecutableMemory, ExecutesCodeButNotData)
{
    std::unique_ptr<AP_HAL::ExecutableMemory> memory(hal.util->allocate_executable(8192));
    ASSERT_NE(memory, nullptr);
    // x86-64: mov eax, 42; ret. Both pages have identical instructions.
    const uint8_t code[] = {0xB8, 42, 0, 0, 0, 0xC3};
    auto *bytes = static_cast<uint8_t *>(memory->data());
    memcpy(bytes, code, sizeof(code));
    memcpy(bytes + 4096, code, sizeof(code));
    ASSERT_TRUE(memory->publish(4096));
    using Entry = uint32_t (*)();
    EXPECT_EQ(reinterpret_cast<Entry>(bytes)(), 42U);
    EXPECT_EXIT({
        signal(SIGSEGV, SIG_DFL);
        reinterpret_cast<Entry>(bytes + 4096)();
        _exit(0);
    }, ::testing::KilledBySignal(SIGSEGV), "");
}
#endif
#else
TEST(ExecutableMemory, RequiresEnabledLinuxBuild)
{
    GTEST_SKIP() << "Requires HAL_EXECUTABLE_MEMORY_ENABLED on a Linux host";
}
#endif

AP_GTEST_MAIN()
