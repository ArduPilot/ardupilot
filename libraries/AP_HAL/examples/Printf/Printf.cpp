#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup(void) {
    hal.console->printf("Starting Printf test\n");
}

static const struct {
    const char *fmt;
    float v;
    const char *result;
} float_tests[] = {
    { "%f", 3.71f, "3.710000" },
    { "%.1f", 3.71f, "3.7" },
    { "%.1f", 3.75f, "3.8" },
    { "%.2f", 3.75f, "3.75" },
    { "%.7f", 3.75f, "3.750000" },
    { "%f", 10.4f, "10.40000" },
    { "%f", 10.6f, "10.60000" },
    { "%f", 1020.4f, "1020.400" },
    { "%f", 1030.6f, "1030.600" },
    { "%f", 10.123456f, "10.12346" },
    { "%f", 102.123456f, "102.1235" },
    { "%f", 1020.123456f, "1020.123" },
    { "%.6f", 10.123456f, "10.12346" },
    { "%.6f", 102.123456f, "102.1235" },
    { "%.6f", 1020.123456f, "1020.123" },
    { "%f", 10304052.6f, "1.030405e+07" },
    { "%f", 103040501.6f, "1.030405e+08" },
    { "%f", 1030405023.6f, "1.030405e+09" },
    { "%f", -1030.6f, "-1030.600" },
    { "%f", -10304052.6f, "-1.030405e+07" },
    { "%f", -103040501.6f, "-1.030405e+08" },
    { "%f", -1030405023.6f, "-1.030405e+09" },
    { "%e", 103040501.6f, "1.030405e+08" },
    { "%g", 103040501.6f, "1.03041e+08" },
    { "%e", -103040501.6f, "-1.030405e+08" },
    { "%g", -103040501.6f, "-1.03041e+08" },
    { "%.0f", 10.4f, "10" },
    { "%.0f", 10.6f, "11" },
    { "%.1f", 10.4f, "10.4" },
    { "%.1f", 10.6f, "10.6" },
};

static void test_printf_floats(void)
{
    hal.console->printf("Starting Printf floats test\n");

    uint8_t i;
    char buf[30];
    uint8_t failures = 0;
    hal.console->printf("Running printf tests\n");
    for (i=0; i < ARRAY_SIZE(float_tests); i++) {
        int ret = hal.util->snprintf(buf, sizeof(buf), float_tests[i].fmt, (double)float_tests[i].v);
        if (strcmp(buf, float_tests[i].result) != 0) {
            hal.console->printf("Failed float_tests[%u] '%s' -> '%s' should be '%s'\n",
                                (unsigned)i,
                                float_tests[i].fmt,
                                buf,
                                float_tests[i].result);
            failures++;
        }
        if (ret != (int)strlen(float_tests[i].result)) {
            hal.console->printf("Failed float_tests[%u] ret=%d/%d '%s' should be '%s'\n",
                                (unsigned)i,
                                ret, (int)strlen(float_tests[i].result),
                                float_tests[i].fmt,
                                float_tests[i].result);
            failures++;
        }
    }
    hal.console->printf("%u failures\n", (unsigned)failures);
}

static void test_printf_null_termination(void)
{
    hal.console->printf("Starting Printf null-termination tests\n");

    {
        char buf[10];
        int ret = hal.util->snprintf(buf,sizeof(buf), "%s", "ABCDEABCDE");
        const int want = 9;
        if (ret != want) {
            hal.console->printf("snprintf returned %d expected %d\n", ret, want);
        }
        if (!strncmp(buf, "ABCDEABCD", sizeof(buf))) {
            hal.console->printf("Bad snprintf string (%s)\n", buf);
        }
    }
    {
        char buf[10];
        int ret = hal.util->snprintf(buf,sizeof(buf), "ABCDEABCDE");
        const int want = 9;
        if (ret != want) {
            hal.console->printf("snprintf returned %d expected %d\n", ret, want);
        }
        if (!strncmp(buf, "ABCDEABCD", sizeof(buf))) {
            hal.console->printf("Bad snprintf string (%s)\n", buf);
        }
    }
}

static void test_printf(void)
{
    test_printf_floats();
    test_printf_null_termination();
}

void loop(void)
{
    test_printf();
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
