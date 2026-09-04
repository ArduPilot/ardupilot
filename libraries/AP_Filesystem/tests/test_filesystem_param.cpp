#include <AP_gtest.h>

#include <AP_Filesystem/AP_Filesystem_Param.h>

#include <fcntl.h>

#if AP_FILESYSTEM_PARAM_ENABLED

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// the upload path allocates no cursors of its own, so a slot it reuses from a
// failed open() is closed on a pointer that has already been freed
TEST(FilesystemParam, OpenFailureLeavesNoStaleCursors)
{
    static AP_Filesystem_Param fs;

    // start >= UINT16_MAX reaches failed:, after the cursors are allocated
    EXPECT_EQ(-1, fs.open("param.pck?start=99999", O_RDONLY));

    const int fd = fs.open("param.pck", O_WRONLY);
    ASSERT_GE(fd, 0);

    // an upload of no parameters legitimately fails to parse, so no EXPECT here
    fs.close(fd);
}

#endif  // AP_FILESYSTEM_PARAM_ENABLED

AP_GTEST_MAIN()
