/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
    Printf.cpp: We demonstrate the use of the printf() and snprintf() functions
*/
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

void setup();    // declaration of the setup() function
void loop();     // declaration of the loop() function

const AP_HAL::HAL& hal = AP_HAL::get_HAL();    // create a reference to AP_HAL::HAL object to get access to hardware specific functions. For more info see <https://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html/>

// the setup function runs once when the board powers up
void setup(void) {
    hal.console->printf("Starting Printf test\n");    // print a starting message
}

//create a struct array float_tests
static const struct {
    // holds the char sequence representing the format specifier
    const char *fmt;
    // holds a float value    
    float v;    
    // holds the char sequence that would be printed in accordance with the format specifier defined(fmt)  
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

// test_printf_floats(void) : tests the printf() and snprintf() function against the float numbers defined in float_tests
static void test_printf_floats(void)
{
    hal.console->printf("Starting Printf floats test\n");
    uint8_t i;
    // 30 bytes long char buffer(Expected length of string : 29)
    char buf[30];
    uint8_t failures = 0;
    hal.console->printf("Running printf tests\n");
    for (i=0; i < ARRAY_SIZE(float_tests); i++) {
        // create a format string(buf) in accordance with the format specifier and the float values defined for every element of float_tests[]   
        int ret = hal.util->snprintf(buf, sizeof(buf), float_tests[i].fmt, (double)float_tests[i].v);    //For more info, see : http://www.cplusplus.com/reference/cstdio/snprintf/
        // comparing the results of the snprintf() and results defined in the float_tests[] :
        //  1. check whether the strings are equal or not
        if (strcmp(buf, float_tests[i].result) != 0) {
            hal.console->printf("Failed float_tests[%u] '%s' -> '%s' should be '%s'\n",
                                (unsigned)i,
                                float_tests[i].fmt,
                                buf,
                                float_tests[i].result);
            failures++;
        }
        //  2. check whether the len of the strings is equal or not
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

// test_printf_null_termination(void) : tests the printf() and the snprintf() function against a char sequence and whether they consider the terminating null character '\0' 
static void test_printf_null_termination(void)
{
    hal.console->printf("Starting Printf null-termination tests\n");
    // 10 bytes long char buffer(Expected length of string : 9)
    char buf[10];
    // create a format string(buf) in accordance with "ABCDEABCDE" and length of buf[]  
    int ret = hal.util->snprintf(buf,sizeof(buf), "%s", "ABCDEABCDE");    //For more info, see : http://www.cplusplus.com/reference/cstdio/snprintf/
    // store the expected length of string
    const int want = 9;
    // comparing the results of the snprintf() function :
    //  1. check whether the expected length of the string is equal to the buffer(buf) length or not
    if (ret != want) {
        hal.console->printf("snprintf returned %d expected %d\n", ret, want);
    }
    //  2. check whether the buffer(buf) is equal to "ABCDEABCD" or not
    if (!strncmp(buf, "ABCDEABCD", sizeof(buf))) {
        hal.console->printf("Bad snprintf string (%s)\n", buf);
    }
}
 
static void test_printf(void)
{
    test_printf_floats();
    test_printf_null_termination();
}

// the loop function runs over and over again forever
void loop(void)
{
    test_printf();
    // give a delay of 1000ms or 1s
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();    // HAL Macro that declares the main function. For more info see <https://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html/>
