/*
 * Utility header for unit tests with gtest.
 */

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include <gtest/gtest.h>


#define AP_GTEST_PRINTATBLE_PARAM_MEMBER(class_name_, printable_member_) \
::std::ostream& operator<<(::std::ostream& os, const class_name_& param); \
::std::ostream& operator<<(::std::ostream& os, const class_name_& param) \
{ \
    return os << param.printable_member_; \
}

#define AP_GTEST_MAIN() \
int main(int argc, char *argv[]) \
{ \
    ::testing::InitGoogleTest(&argc, argv); \
    return RUN_ALL_TESTS(); \
}
