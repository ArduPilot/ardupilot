/*
  Unit tests for the @MAV_LOG alias.

  The alias is a row in AP_Filesystem's backend table rather than a
  filesystem of its own: the resolver rewrites a path under the prefix to
  sit under the board's log directory and hands it to the local filesystem.
  These tests drive that through the public AP_Filesystem interface.
 */

#include <AP_gtest.h>

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/AP_HAL.h>

#include <string>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_Filesystem fs;

/*
  create a file with the OS rather than AP_Filesystem.  the tests below need
  files at names AP_Filesystem's own resolver would divert, which is the very
  thing under test, so they cannot be made through it
 */
static bool create_directly(const char *path, const char *contents)
{
    int fd = ::open(path, O_WRONLY|O_CREAT|O_TRUNC, 0666);
    if (fd < 0) {
        return false;
    }
    const ssize_t len = strlen(contents);
    const bool ok = ::write(fd, contents, len) == len;
    ::close(fd);
    return ok;
}

#if AP_FILESYSTEM_MAVLOG_ENABLED

// the longest path an FTP request can carry: the transaction holds 239
// bytes and the last of them is the null
#define FTP_MAX_REQUEST_PATH 238
// the longest name an FTP listing will stat an entry by
#define FTP_MAX_NAME_LEN 255

class MavLogAlias : public ::testing::Test
{
protected:
    void SetUp(void) override {
        fs.mkdir(HAL_BOARD_LOG_DIRECTORY);
        int fd = fs.open(path_under_root, O_WRONLY|O_CREAT|O_TRUNC);
        ASSERT_GE(fd, 0);
        EXPECT_EQ(fs.write(fd, "alias", 5), 5);
        fs.close(fd);
    }
    void TearDown(void) override {
        fs.unlink(path_under_root);
    }
    const char *name = "alias_unit_test.txt";
    char path_under_root[128] = HAL_BOARD_LOG_DIRECTORY "/alias_unit_test.txt";
};

TEST_F(MavLogAlias, ResolvesToTheLogDirectory)
{
    char aliased[128];
    snprintf(aliased, sizeof(aliased), "@MAV_LOG/%s", name);

    struct stat direct {};
    struct stat through_alias {};
    ASSERT_EQ(fs.stat(path_under_root, &direct), 0);
    ASSERT_EQ(fs.stat(aliased, &through_alias), 0);
    EXPECT_EQ(direct.st_size, through_alias.st_size);
    EXPECT_EQ(direct.st_size, 5);
}

TEST_F(MavLogAlias, LeadingSlashIsAccepted)
{
    char aliased[128];
    snprintf(aliased, sizeof(aliased), "/@MAV_LOG/%s", name);
    struct stat st {};
    EXPECT_EQ(fs.stat(aliased, &st), 0);
}

/*
  the prefix has to be the whole of the first component.  "@MAV_LOG_x" is an
  ordinary local file, not "_x" under the log directory; one of those is
  planted too, with different contents, so that reaching it is told apart
  from reaching the right file
 */
TEST_F(MavLogAlias, APrefixOfTheFirstComponentIsNotTheAlias)
{
    ASSERT_TRUE(create_directly("@MAV_LOG_x", "local"));
    ASSERT_TRUE(create_directly(HAL_BOARD_LOG_DIRECTORY "/_x", "aliased file"));

    struct stat st {};
    EXPECT_EQ(fs.stat("@MAV_LOG_x", &st), 0);
    EXPECT_EQ(st.st_size, 5);

    ::unlink("@MAV_LOG_x");
    ::unlink(HAL_BOARD_LOG_DIRECTORY "/_x");
}

/*
  rename is the one call with two paths alive at once.  with both under the
  alias, each needs a rewritten path of its own for the whole call
 */
TEST_F(MavLogAlias, RenameWithinTheAlias)
{
    char from[128];
    snprintf(from, sizeof(from), "@MAV_LOG/%s", name);
    const char *to = "@MAV_LOG/alias_unit_test_renamed.txt";
    const char *to_under_root = HAL_BOARD_LOG_DIRECTORY "/alias_unit_test_renamed.txt";

    ASSERT_EQ(fs.rename(from, to), 0);

    struct stat st {};
    EXPECT_EQ(::stat(path_under_root, &st), -1);
    EXPECT_EQ(::stat(to_under_root, &st), 0);
    EXPECT_EQ(st.st_size, 5);

    ::unlink(to_under_root);
}

TEST_F(MavLogAlias, PrefixOnItsOwnIsTheDirectory)
{
    struct stat st {};
    ASSERT_EQ(fs.stat("@MAV_LOG", &st), 0);
    EXPECT_TRUE(S_ISDIR(st.st_mode));
}

/*
  the buffer the rewritten path is built in has to hold this board's log
  directory as well as the path, or a path a listing is entitled to stat
  would be refused and the file would vanish from the listing with no error.
  a listing stats each entry by the path it was asked for, a separator and
  the entry's name.  AP_Filesystem.cpp static_asserts this for the
  compile-time directory; this states the same requirement where a reader of
  the tests will see it
 */
TEST(MavLogAliasSizing, BufferHoldsTheLongestListingPathUnderThisRoot)
{
    EXPECT_LE(strlen(HAL_BOARD_LOG_DIRECTORY) + 1 + FTP_MAX_REQUEST_PATH + 1 + FTP_MAX_NAME_LEN + 1,
              (size_t)AP_FILESYSTEM_ALIAS_PATH_MAX);
}

/*
  and that longest path really does resolve: a directory whose aliased path
  is the longest a request can carry, holding a file with the longest name.
  paths this long are built as strings; there is no fixed buffer the
  compiler could decide they overflow
 */
TEST_F(MavLogAlias, TheLongestListingPathResolves)
{
    const std::string dir(FTP_MAX_REQUEST_PATH - strlen("@MAV_LOG/"), 'd');
    const std::string leaf(FTP_MAX_NAME_LEN, 'f');

    const std::string dir_under_root = std::string(HAL_BOARD_LOG_DIRECTORY) + "/" + dir;
    const std::string leaf_under_root = dir_under_root + "/" + leaf;
    ASSERT_EQ(::mkdir(dir_under_root.c_str(), 0777), 0);
    ASSERT_TRUE(create_directly(leaf_under_root.c_str(), "alias"));

    const std::string aliased_dir = "@MAV_LOG/" + dir;
    ASSERT_EQ(aliased_dir.size(), (size_t)FTP_MAX_REQUEST_PATH);
    const std::string aliased = aliased_dir + "/" + leaf;

    struct stat st {};
    EXPECT_EQ(fs.stat(aliased.c_str(), &st), 0);
    EXPECT_EQ(st.st_size, 5);

    ::unlink(leaf_under_root.c_str());
    ::rmdir(dir_under_root.c_str());
}

/*
  A path too long to rewrite must not resolve to something else. There are
  three files it could wrongly reach: the path with the prefix stripped, the
  path exactly as it arrived, and the rewritten path cut short to fit the
  buffer. All three are created here, on the local filesystem where only a
  mistake in the resolver would reach them.
 */
TEST_F(MavLogAlias, APathTooLongToRewriteReachesNothing)
{
    // three components, each short enough for any filesystem, but together
    // longer than a rewritten path can be
    const std::string dir(200, 'd');
    const std::string subdir(200, 'e');
    const std::string leaf(200, 'f');

    const std::string aliased = "@MAV_LOG/" + dir + "/" + subdir + "/" + leaf;
    ASSERT_GT(aliased.size(), (size_t)AP_FILESYSTEM_ALIAS_PATH_MAX);

    // each is created with the OS rather than AP_Filesystem, because the
    // alias would intercept "@MAV_LOG" itself.  "base" is the directory
    // under which the whole of the stripped path is made
    struct Candidate {
        std::string base;
        std::string dir;
        std::string subdir;
        std::string leaf;
    };
    Candidate candidates[] {
        // the prefix-stripped path, which the resolver must not fall back to
        { "." },
        // the whole path as it arrives
        { "@MAV_LOG" },
        // and the rewritten path, whose leaf is truncated to what the buffer holds
        { HAL_BOARD_LOG_DIRECTORY },
    };

    for (auto &c : candidates) {
        c.dir = c.base + "/" + dir;
        c.subdir = c.dir + "/" + subdir;
        c.leaf = c.subdir + "/" + leaf;
        ::mkdir(c.base.c_str(), 0777);
        ::mkdir(c.dir.c_str(), 0777);
        ::mkdir(c.subdir.c_str(), 0777);
    }
    Candidate &truncated = candidates[ARRAY_SIZE(candidates) - 1];
    truncated.leaf.resize(AP_FILESYSTEM_ALIAS_PATH_MAX - 1);
    ASSERT_GT(truncated.leaf.size(), truncated.subdir.size() + 1);

    // all three exist where the local filesystem can see them...
    for (auto &c : candidates) {
        EXPECT_TRUE(create_directly(c.leaf.c_str(), ""));
        struct stat st {};
        EXPECT_EQ(::stat(c.leaf.c_str(), &st), 0);
    }
    // ...and none may be reached by asking for the overlong alias path
    struct stat st {};
    EXPECT_EQ(fs.stat(aliased.c_str(), &st), -1);

    for (auto &c : candidates) {
        ::unlink(c.leaf.c_str());
        ::rmdir(c.subdir.c_str());
        ::rmdir(c.dir.c_str());
    }
    ::rmdir("@MAV_LOG");
}

#endif  // AP_FILESYSTEM_MAVLOG_ENABLED

#if AP_FILESYSTEM_SYS_ENABLED
/*
  the whole-component prefix match is not particular to aliases.
  "@SYSfoo" names a local file; it is not "foo" served by @SYS
 */
TEST(BackendPrefix, APrefixOfTheFirstComponentIsNotTheBackend)
{
    ASSERT_TRUE(create_directly("@SYSfoo", "local"));

    struct stat st {};
    EXPECT_EQ(fs.stat("@SYSfoo", &st), 0);
    EXPECT_EQ(st.st_size, 5);

    ::unlink("@SYSfoo");
}
#endif  // AP_FILESYSTEM_SYS_ENABLED

AP_GTEST_MAIN()
