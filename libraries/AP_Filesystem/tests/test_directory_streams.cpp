// SPDX-License-Identifier: GPL-3.0-or-later
#include <AP_gtest.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/AP_HAL.h>

#include <string>
#include <vector>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if AP_FILESYSTEM_FILE_READING_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL
TEST(DirectoryStreams, IndependentVirtualEntries)
{
    auto &fs = AP::FS();
    auto *first = fs.opendir("/");
    ASSERT_NE(first, nullptr);
    std::vector<std::string> expected;
    while (const auto *entry = fs.readdir(first)) {
        if (entry->d_name[0] == '@') {
            expected.emplace_back(entry->d_name);
        }
    }
    fs.closedir(first);
    ASSERT_GT(expected.size(), 1U);

    first = fs.opendir("/");
    ASSERT_NE(first, nullptr);
    // A non-root listing must not disable virtual entries in another stream.
    auto *other = fs.opendir("@ROMFS");
    ASSERT_NE(other, nullptr);
    fs.closedir(other);
    std::vector<std::string> actual;
    while (const auto *entry = fs.readdir(first)) {
        if (entry->d_name[0] == '@') {
            actual.emplace_back(entry->d_name);
        }
    }
    EXPECT_EQ(actual, expected);
    fs.closedir(first);

    first = fs.opendir("/");
    ASSERT_NE(first, nullptr);
    const struct dirent *saved = nullptr;
    while ((saved = fs.readdir(first)) != nullptr && saved->d_name[0] != '@') {
    }
    ASSERT_NE(saved, nullptr);
    const std::string name = saved->d_name;
    other = fs.opendir("/");
    ASSERT_NE(other, nullptr);
    while (fs.readdir(other) != nullptr) {
    }
    EXPECT_EQ(std::string(saved->d_name), name);
    fs.closedir(other);
    fs.closedir(first);
}
#endif

AP_GTEST_MAIN()
