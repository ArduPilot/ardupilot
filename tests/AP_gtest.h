/*
 * Utility header for unit tests with gtest.
 */
#include <gtest/gtest.h>

#define AP_GTEST_MAIN() \
int main(int argc, char *argv[]) \
{ \
    ::testing::InitGoogleTest(&argc, argv); \
    return RUN_ALL_TESTS(); \
}
