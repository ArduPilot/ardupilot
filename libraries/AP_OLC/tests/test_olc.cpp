#include <AP_gtest.h>
#include "AP_Math/AP_Math.h"
#include "AP_OLC/AP_OLC.h"

AP_OLC olc;
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Test the olc_encode function of AP_OLC
TEST(AP_OLCTest, OLC_EncodeTest) {
    // Test cases
    struct TestCase {
        int32_t lat;
        int32_t lon;
        size_t length;
        std::string expectedCode;
    };

    // Define the test cases
    std::vector<TestCase> testCases = {
            {500000000, 1000000000, 6, "9P2222"},
            {900000000, 2000000000, 8, "C3X2X2X2"},
            {000000000, 500000000, 9, "6HGG2222+"},
            {300000000, 500000000, 10, "8H2G2222+2"},
            {300000000, 500000000, 11, "8H2G2222+22"},
            {300000000, 500000000, 12, "8H2G2222+22"},
            {300000000, -1000000000, 10, "86222222+2"},
            {950000000, -1900000000, 12, "CVXGX2X2+X2"},
            // Add more test cases as needed
    };
    // Run the test cases
    for (const auto& testCase : testCases) {
        std::string buf(testCase.length + 2, '\0');  // buf should be > length +1 as the coding add \0
        uint32_t result = olc.olc_encode(testCase.lat, testCase.lon, testCase.length, &buf[0], buf.size());
        if (testCase.length < 9) {
            EXPECT_EQ(result, 9u);  // 9 is separator min position
        } else {
            EXPECT_EQ(result, 11u);  // length max is 11
        }
        EXPECT_EQ(buf.substr(0, MIN(testCase.length, 11u)), testCase.expectedCode);
    }
    std::string buf(2, '\0');
    uint32_t result = olc.olc_encode(testCases[0].lat, testCases[0].lon, testCases[0].length, &buf[0], buf.size());
    EXPECT_EQ(result, 0u);
    std::string empty_code(2, '\0');
    EXPECT_EQ(buf, empty_code);
    std::string buf2(16, '\0');  // buf should be > length +1 as the coding add \0
    result = olc.olc_encode(testCases[0].lat, testCases[0].lon, 11, &buf2[0], buf2.size());
    EXPECT_EQ(result, 12u);
    EXPECT_EQ(buf2.substr(0, 12), "9P222222+222");
}

AP_GTEST_MAIN()
