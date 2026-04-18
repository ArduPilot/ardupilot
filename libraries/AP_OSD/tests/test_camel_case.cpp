/*
 * Regression tests for AP_OSD_ParamSetting::copy_name_camel_case().
 */

#include <AP_gtest.h>
#include <string.h>
#include <ctype.h>
#include <stddef.h>
#include <stdint.h>
#include <limits.h>

int hal = 0;

static void camel_case_buggy(char *name, const char *src, size_t len)
{
    name[0] = src[0];
    for (uint8_t i = 1, n = 1; i < len - 1 && src[i] != '\0'; i++, n++) {
        if (src[i] == '_' && src[i + 1] != '\0') {
            name[n] = src[i + 1];
            i++;
        } else {
            name[n] = (char)tolower((unsigned char)src[i]);
        }
    }
}

static void camel_case_fixed(char *name, const char *src, size_t len)
{
    if (len == 0) {
        return;
    }
    name[0] = src[0];
    uint8_t n = 1;
    for (uint8_t i = 1; i < len - 1 && src[i] != '\0'; i++, n++) {
        if (src[i] == '_' && src[i + 1] != '\0') {
            name[n] = src[i + 1];
            i++;
        } else {
            name[n] = (char)tolower((unsigned char)src[i]);
        }
    }
    name[n] = '\0';
}

TEST(CamelCase, BugA_null_terminator_written_by_fixed_version)
{
    const char src[17] = "FOO_BAR";
    const size_t len = 16;
    char name[16];

    memset(name, 0xFF, sizeof(name));
    camel_case_buggy(name, src, len);
    EXPECT_NE('\0', name[6]);

    memset(name, 0xFF, sizeof(name));
    camel_case_fixed(name, src, len);
    EXPECT_EQ('\0', name[6]);
    EXPECT_EQ('F', name[0]);
    EXPECT_EQ('o', name[1]);
    EXPECT_EQ('o', name[2]);
    EXPECT_EQ('B', name[3]);
    EXPECT_EQ('a', name[4]);
    EXPECT_EQ('r', name[5]);
}

TEST(CamelCase, BugB_len_zero_does_not_write_on_fixed_version)
{
    const char src[17] = "FOO";
    const char sentinel = (char)0xFF;
    char name[8];
    size_t len_zero = 0;

    EXPECT_EQ(SIZE_MAX, len_zero - 1);

    memset(name, (int)(unsigned char)sentinel, sizeof(name));
    camel_case_buggy(name, src, 0);
    EXPECT_NE(sentinel, name[0]);

    memset(name, (int)(unsigned char)sentinel, sizeof(name));
    camel_case_fixed(name, src, 0);
    for (size_t i = 0; i < sizeof(name); i++) {
        EXPECT_EQ(sentinel, name[i]);
    }
}

TEST(CamelCase, normal_conversion_correct)
{
    const char src[17] = "AHR_TRIM_X";
    char name[16];

    memset(name, 0, sizeof(name));
    camel_case_fixed(name, src, sizeof(name));
    EXPECT_STREQ("AhrTrimX", name);
}

AP_GTEST_MAIN()