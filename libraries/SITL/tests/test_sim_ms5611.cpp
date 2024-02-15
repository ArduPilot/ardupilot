#include <AP_gtest.h>

#include <SITL/SIM_MS5611.h>
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace SITL;

// Dummy class to access protected functions through a public interface for unit tests
class dummy : public SITL::MS5611
{
public:
    void convert_forward_pub(int32_t D1, int32_t D2, float &P_Pa, float &Temp_C);
    void convert_pub(float P_Pa, float Temp_C, uint32_t &D1, uint32_t &D2);

};

void dummy::convert_forward_pub(int32_t D1, int32_t D2, float &P_Pa, float &Temp_C)
{
    dummy::convert_forward(D1, D2, P_Pa, Temp_C);
}

void dummy::convert_pub(float P_Pa, float Temp_C, uint32_t &D1, uint32_t &D2)
{
    dummy::convert(P_Pa, Temp_C, D1, D2);
}

const struct forward_check {
        uint32_t D1; // pressure bytes
        uint32_t D2; // temperature bytes
        float P_Pa;
        float Temp_C;
    } conversion_table [] = {
        {3894332, 7304992, 1200, -30},          // < -15 deg C Temperature, Low Pressure
        {10566581, 7304992, 115200, -30},       // < -15 deg C Temperatures, High Pressure

        {3919542, 8425824, 1200, 15.15},        // < 20 deg C Temperatures, Low Pressure
        {9937566, 8425824, 115200, 15.15},      // < 20 deg C Temperatures, High Pressure

        {3930802, 9048258, 1200, 36.25},        // > 20 deg C Temperature, Low Pressure
        {9696473, 9048258, 115200, 36.25},      // > 20 deg C Temperature, High Pressure
    };

TEST(MS5611, convert_forward)
{
    dummy ms5611;
    float accuracy_Pa = 0.2;
    float accuracy_Temp_C = 0.02;

    for(auto elem: conversion_table) {
        float P_Pa;
        float Temp_C;
        ms5611.convert_forward_pub(elem.D1, elem.D2, P_Pa, Temp_C);
        EXPECT_NEAR(P_Pa, elem.P_Pa, accuracy_Pa);
        EXPECT_NEAR(Temp_C, elem.Temp_C, accuracy_Temp_C);
    }
}

TEST(MS5611, convert)
{
    dummy ms5611;

    for(auto elem: conversion_table) {
        uint32_t D1;
        uint32_t D2;
        ms5611.convert_pub(elem.P_Pa, elem.Temp_C, D1, D2);

        // Expect NEAR here instead of EQ because in 32bit they are off by 1
        EXPECT_NEAR(D1, elem.D1, 1);
        EXPECT_NEAR(D2, elem.D2, 1);
    }
}

AP_GTEST_MAIN()
