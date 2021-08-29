/*
 * Utility header for unit tests with gtest.
 */

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include <gtest/gtest.h>
#include <gtest/gtest-spi.h>


#define AP_GTEST_PRINTATBLE_PARAM_MEMBER(class_name_, printable_member_) \
::std::ostream& operator<<(::std::ostream& os, const class_name_& param); \
::std::ostream& operator<<(::std::ostream& os, const class_name_& param) \
{ \
    return os << param.printable_member_; \
}

/*
* Override the WEAK version of AP_HAL_SITL/system.cpp panic() instead of staying in an infinite loop
* This is used by the gtest suite to test for an exit signal caused by a test statement and continue testing
* Printing to stderr  is required for gtest matching
*/
#define AP_GTEST_PANIC() \
void AP_HAL::panic(const char *errormsg, ...) \
{ \
    va_list ap; \
    auto outputs = {stdout, stderr}; \
    for( auto output : outputs) { \
        fflush(stdout); \
        fprintf(output, "PANIC: "); \
        va_start(ap, errormsg); \
        vfprintf(output, errormsg, ap); \
        va_end(ap); \
        fprintf(output, "\n"); \
    } \
    abort(); \
}

#define AP_GTEST_MAIN() \
int main(int argc, char *argv[]) \
{ \
    ::testing::InitGoogleTest(&argc, argv); \
    return RUN_ALL_TESTS(); \
}
