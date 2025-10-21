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
    float dT = float(D2) - int64_t(prom[5]) * (1L<<Q5);
    float TEMP = 2000 + (dT*int64_t(prom[6]))/(1L<<Q6);
    float OFF  = int64_t(prom[2])*(1L<<Q2) + (int64_t(prom[4])*dT)/(1L<<Q4);
    float SENS = int64_t(prom[1])*(1L<<Q1) + (int64_t(prom[3])*dT)/(1L<<Q3);
    float P = (float(D1)*SENS/(1L<<21)-OFF)/(1L<<15);
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

    const float TEMP = Temp_C * 100.0f;
    const float dT = ((TEMP - 2000.0) * (1L<<Q6)) / int64_t(prom[6]);
    const float PSI_to_Pa = 6894.757f;
    const float P = P_Pa / (PSI_to_Pa * 1.0e-4);
    const float OFF  = int64_t(prom[2]) * (1L<<Q2) + (int64_t(prom[4]) * dT) / (1L<<Q4);
    const float SENS = int64_t(prom[1]) * (1L<<Q1) + (int64_t(prom[3]) * dT) / (1L<<Q3);

    D1 = ((P * (1L<<15) + OFF) * (1L<<21)) / SENS;
    D2 = dT + int64_t(prom[5]) * (1L<<Q5);
}

void MS5525::check_conversion_accuracy(float P_Pa, float Temp_C, uint32_t D1, uint32_t D2)
{
    float f_P_Pa;
    float f_Temp_C;
    convert_forward(D1, D2, f_P_Pa, f_Temp_C);

    const float p_error = fabs(f_P_Pa - P_Pa);
    const float p_error_max = 0.1;
    if (p_error > p_error_max) {
        AP_HAL::panic("Invalid pressure conversion: error %f Pa > %f Pa error max", p_error, p_error_max);
    }

    const float t_error = fabs(f_Temp_C - Temp_C);
    const float t_error_max = 0.01;
    if (t_error > t_error_max) {
        AP_HAL::panic("Invalid temperature conversion: error %f degC > %f degC error max", t_error, t_error_max);
    }
}

void MS5525::get_pressure_temperature_readings(float &P_Pa, float &Temp_C)
{
    float sim_alt = AP::sitl()->state.altitude;
    sim_alt += 2 * rand_float();

    Temp_C = AP_Baro::get_temperatureC_for_alt_amsl(sim_alt);
    const uint8_t instance = 0;  // TODO: work out which sensor this is
    P_Pa = AP::sitl()->state.airspeed_raw_pressure[instance] + AP::sitl()->airspeed[instance].offset;
}
