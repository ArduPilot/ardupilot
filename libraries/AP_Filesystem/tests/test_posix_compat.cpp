/*
  Unit tests for the stdio emulation in posix_compat, which Lua's io
  library uses on boards.  The modes given to apfs_fopen must keep or
  discard a file's contents as C's fopen does.
 */

#include <AP_gtest.h>

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/AP_HAL.h>

#include <string.h>

#include <AP_Filesystem/posix_compat.h>

#if AP_FILESYSTEM_POSIX_ENABLED

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static const char *test_path = "test_posix_compat.txt";

// write contents to the test file with the given mode
static void write_file(const char *mode, const char *contents)
{
    APFS_FILE *f = apfs_fopen(test_path, mode);
    ASSERT_NE(f, nullptr);
    const size_t len = strlen(contents);
    EXPECT_EQ(apfs_fwrite(contents, 1, len, f), len);
    EXPECT_EQ(apfs_fclose(f), 0);
}

// the test file's whole contents
static std::string read_file(void)
{
    APFS_FILE *f = apfs_fopen(test_path, "r");
    if (f == nullptr) {
        return "<missing>";
    }
    std::string ret;
    char buf[64];
    size_t n;
    while ((n = apfs_fread(buf, 1, sizeof(buf), f)) > 0) {
        ret.append(buf, n);
    }
    apfs_fclose(f);
    return ret;
}

class PosixCompat : public ::testing::Test
{
protected:
    void SetUp(void) override {
        apfs_remove(test_path);
    }
    void TearDown(void) override {
        apfs_remove(test_path);
    }
};

// "r+" opens an existing file for update, keeping what is in it
TEST_F(PosixCompat, ReadPlusKeepsContents)
{
    write_file("w", "hello world");

    APFS_FILE *f = apfs_fopen(test_path, "r+");
    ASSERT_NE(f, nullptr);
    char buf[16] {};
    EXPECT_EQ(apfs_fread(buf, 1, 11, f), 11U);
    EXPECT_STREQ(buf, "hello world");
    EXPECT_EQ(apfs_fseek(f, 0, SEEK_SET), 0);
    EXPECT_EQ(apfs_fwrite("J", 1, 1, f), 1U);
    EXPECT_EQ(apfs_fclose(f), 0);

    EXPECT_EQ(read_file(), "Jello world");
}

// "r+" doesn't create a file
TEST_F(PosixCompat, ReadPlusNeedsFile)
{
    EXPECT_EQ(apfs_fopen(test_path, "r+"), nullptr);
    EXPECT_EQ(read_file(), "<missing>");
}

// "w" and "w+" discard the contents; "a" and "a+" add to them
TEST_F(PosixCompat, WriteTruncatesAppendAppends)
{
    write_file("w", "abc");
    write_file("a", "def");
    EXPECT_EQ(read_file(), "abcdef");
    write_file("a+", "g");
    EXPECT_EQ(read_file(), "abcdefg");
    write_file("w", "x");
    EXPECT_EQ(read_file(), "x");
    write_file("w+", "yz");
    EXPECT_EQ(read_file(), "yz");
}

#endif  // AP_FILESYSTEM_POSIX_ENABLED

AP_GTEST_MAIN()
