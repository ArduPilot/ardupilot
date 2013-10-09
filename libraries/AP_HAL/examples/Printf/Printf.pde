#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>
#include <AP_Common.h>
#include <AP_Baro.h>
#include <AP_ADC.h>
#include <GCS_MAVLink.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <SITL.h>
#include <Filter.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_Math.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

void setup(void) {
    hal.console->println_P(PSTR("Starting Printf test"));
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

static void test_printf(void)
{
    uint8_t i;
    char buf[30];
    uint8_t failures = 0;
    hal.console->printf("Running printf tests\n");
    for (i=0; i<sizeof(float_tests)/sizeof(float_tests[0]); i++) {
        int ret = hal.util->snprintf(buf, sizeof(buf), float_tests[i].fmt, float_tests[i].v);
        if (strcmp(buf, float_tests[i].result) != 0) {
            hal.console->printf("Failed float_tests[%u] '%s' -> '%s' should be '%s'\n", 
                                (unsigned)i, 
                                float_tests[i].fmt,
                                buf,
                                float_tests[i].result);
            failures++;
        }
        if (ret != strlen(float_tests[i].result)) {
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

void loop(void) 
{	
    test_printf();
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
