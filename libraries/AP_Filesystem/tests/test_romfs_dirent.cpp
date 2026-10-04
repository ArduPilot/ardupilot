/*
  a ROMFS entry named longer than a directory entry holds must come back cut
  short and terminated, and reading it must not write outside its own entry.
  SITL embeds two such entries under autotest_fixtures/long_names, a file and
  a directory, each with a 300 character name (see Tools/ardupilotwaf/boards.py)
 */

#include <AP_gtest.h>

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/AP_HAL.h>

#include <string.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// the fixtures these need are embedded by SITL's board configuration, and
// ROMFS itself is enabled wherever a board embeds anything at all
#if AP_FILESYSTEM_ROMFS_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL

TEST(ROMFSDirent, LongNamesAreTerminated)
{
    auto *dir = AP::FS().opendir("@ROMFS/autotest_fixtures/long_names");
    ASSERT_NE(dir, nullptr);
    unsigned count = 0;
    const struct dirent *de;
    while ((de = AP::FS().readdir(dir)) != nullptr) {
        count++;
        EXPECT_LT(strnlen(de->d_name, sizeof(de->d_name)), sizeof(de->d_name));
    }
    AP::FS().closedir(dir);
    EXPECT_EQ(count, 2U);
}

/*
  the directory's name is ended at its separator, 300 characters in, which
  is past the end of d_name.  with another listing open alongside, that
  write lands in the entry the other listing last returned.  the other
  listing is of autotest_fixtures, whose first entry is long_names, a name
  long enough to show the write cutting it short
 */
TEST(ROMFSDirent, ALongDirectoryNameLeavesOtherListingsAlone)
{
    auto *long_names = AP::FS().opendir("@ROMFS/autotest_fixtures/long_names");
    ASSERT_NE(long_names, nullptr);
    auto *other = AP::FS().opendir("@ROMFS/autotest_fixtures");
    ASSERT_NE(other, nullptr);

    const struct dirent *other_de = AP::FS().readdir(other);
    ASSERT_NE(other_de, nullptr);
    char other_name[sizeof(other_de->d_name)];
    strncpy(other_name, other_de->d_name, sizeof(other_name));
    other_name[sizeof(other_name)-1] = 0;
    ASSERT_STREQ(other_name, "long_names");

    while (AP::FS().readdir(long_names) != nullptr) {
    }
    EXPECT_STREQ(other_de->d_name, other_name);

    AP::FS().closedir(other);
    AP::FS().closedir(long_names);
}

#endif  // AP_FILESYSTEM_ROMFS_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL

AP_GTEST_MAIN()
