#include <AP_gtest.h>
#include <string.h>
#include <stdarg.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/NMEA.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// wrapper to make testing nmea_vaprintf_buffer() easy
static uint16_t test_vaprintf_buffer(char *buf, const uint16_t buf_max_len, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    uint16_t len = nmea_vaprintf_buffer(buf, buf_max_len, fmt, ap);
    va_end(ap);
    return len;
}

TEST(NMEA, VAPrintfBuffer)
{
    // normal formatting, checksum and CR/LF appended, NUL terminated
    char buf[32];
    uint16_t len = test_vaprintf_buffer(buf, sizeof(buf), "$TEST");
    EXPECT_EQ(len, 10U);
    EXPECT_STREQ(buf, "$TEST*16\r\n");

    // return value matches the number of bytes in the buffer
    EXPECT_EQ(strlen(buf), len);

    // the message should be unchanged (still NUL terminated)
    EXPECT_EQ(buf[len], 0);

    // if there is not enough room for the checksum/CR/LF trailer it returns 0
    char too_small_buf[6];
    len = test_vaprintf_buffer(too_small_buf, sizeof(too_small_buf), "$TEST");
    EXPECT_EQ(len, 0U);

    // if the message body does not fit it returns 0 and leaves the buffer untouched
    char tight_buf[10];
    memset(tight_buf, 'x', sizeof(tight_buf));
    len = test_vaprintf_buffer(tight_buf, sizeof(tight_buf), "$TEST");
    EXPECT_EQ(len, 0U);
    for (size_t i = 0; i < sizeof(tight_buf); i++) {
        EXPECT_EQ(tight_buf[i], 'x');
    }

    // nmea_printf_buffer formats the same way
    len = nmea_printf_buffer(buf, sizeof(buf), "$TEST");
    EXPECT_EQ(len, 10U);
    EXPECT_STREQ(buf, "$TEST*16\r\n");
}

AP_GTEST_MAIN()
