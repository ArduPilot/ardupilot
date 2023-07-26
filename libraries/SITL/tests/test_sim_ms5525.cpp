#include <AP_gtest.h>

#include <SITL/SIM_MS5525.h>
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace SITL;

// Dummy class to access protected functions through a public interface for unit tests
class dummy : public SITL::MS5525
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
        {9453147, 3438567, 245, -20},           // Negative Temperatures, Positive Pressure

        {9407591, 3438567, -245, -20},          // Negative Temperatures, Negative Pressure

        {9153883, 3774904, 245, 15.15},         // < 20 deg C Temperatures, Positive Pressure

        {9112431, 3774904, -245, 15.15},        // < 20 deg C Temperatures, Negative Pressure

        {8979245, 3976802, 1.2250, 36.25},      // > 20 deg C Temperature, Positive Pressure
        {8998809, 3976802, 245, 36.25},         // > 20 deg C Temperature, Positive Pressure
        {9500811, 3976802, 6500, 36.25},        // > 20 deg C Temperature, Positive Pressure

        {8959483, 3976802, -245, 36.25},        // > 20 deg C Temperature, Negative Pressure
    };

TEST(MS5525, convert_forward)
{
    dummy ms5525;
    float accuracy_Pa = 0.05;
    float accuracy_Temp_C = 0.01;

    for(auto elem: conversion_table) {
        float P_Pa;
        float Temp_C;
        ms5525.convert_forward_pub(elem.D1, elem.D2, P_Pa, Temp_C);
        EXPECT_NEAR(P_Pa, elem.P_Pa, accuracy_Pa);
        EXPECT_NEAR(Temp_C, elem.Temp_C, accuracy_Temp_C);
    }
}

TEST(MS5525, convert)
{
    dummy ms5525;

    for(auto elem: conversion_table) {
        uint32_t D1;
        uint32_t D2;
        ms5525.convert_pub(elem.P_Pa, elem.Temp_C, D1, D2);

        // Expect NEAR here instead of EQ because in 32bit they are off by 1
        EXPECT_NEAR(D1, elem.D1, 1);
        EXPECT_NEAR(D2, elem.D2, 1);
    }
}

AP_GTEST_MAIN()
