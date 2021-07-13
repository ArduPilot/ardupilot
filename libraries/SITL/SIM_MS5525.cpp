#include "SIM_MS5525.h"

#include <SITL/SITL.h>

using namespace SITL;

void MS5525::convert_forward(int32_t D1, int32_t D2, float &P_Pa, float &Temp_C)
{
    const uint8_t Q1 = Qx_coeff[0];
    const uint8_t Q2 = Qx_coeff[1];
    const uint8_t Q3 = Qx_coeff[2];
    const uint8_t Q4 = Qx_coeff[3];
    const uint8_t Q5 = Qx_coeff[4];
    const uint8_t Q6 = Qx_coeff[5];

    // this is the forward conversion copied from the driver:
    int64_t dT = D2 - int64_t(prom[5]) * (1UL<<Q5);
    int64_t TEMP = 2000 + (dT*int64_t(prom[6]))/(1UL<<Q6);
    int64_t OFF =  int64_t(prom[2])*(1UL<<Q2) + (int64_t(prom[4])*dT)/(1UL<<Q4);
    int64_t SENS = int64_t(prom[1])*(1UL<<Q1) + (int64_t(prom[3])*dT)/(1UL<<Q3);
    int64_t P = (D1*SENS/(1UL<<21)-OFF)/(1UL<<15);
    const float PSI_to_Pa = 6894.757f;
    P_Pa = PSI_to_Pa * 1.0e-4 * P;
    Temp_C = TEMP * 0.01;
}

void MS5525::convert(float P_Pa, float Temp_C, uint32_t &D1, uint32_t &D2)
{
    const uint8_t Q1 = Qx_coeff[0];
    const uint8_t Q2 = Qx_coeff[1];
    const uint8_t Q3 = Qx_coeff[2];
    const uint8_t Q4 = Qx_coeff[3];
    const uint8_t Q5 = Qx_coeff[4];
    const uint8_t Q6 = Qx_coeff[5];

    const int64_t TEMP = Temp_C * 100.0f;
    const float dT = ((TEMP-2000)*(1UL<<Q6))/prom[6];
    const float PSI_to_Pa = 6894.757f;
    const float P = P_Pa / (PSI_to_Pa * 1.0e-4);
    const int64_t SENS = int64_t(prom[1])*(1UL<<Q1) + (int64_t(prom[3])*dT)/(1UL<<Q3);
    const int64_t OFF =  int64_t(prom[2])*(1UL<<Q2) + (int64_t(prom[4])*dT)/(1UL<<Q4);
    D1 = (((uint64_t(P*(1U<<15)))+OFF)<<21)/SENS;
    D2 = dT + int64_t(prom[5]) * (1UL<<Q5);


    float f_P_Pa;
    float f_Temp_C;
    convert_forward(D1, D2, f_P_Pa, f_Temp_C);
    if (fabs(f_P_Pa - P_Pa) > 1) {
        AP_HAL::panic("Invalid conversion");
    }
    if (fabs(f_Temp_C - Temp_C) > 0.1) {
        AP_HAL::panic("Invalid conversion");
    }
}

void MS5525::get_pressure_temperature_readings(float &P_Pa, float &Temp_C)
{
    Temp_C = 25.0f;
    P_Pa = AP::sitl()->state.airspeed_raw_pressure[0];
}
