#include "SIM_MS5611.h"

#include <SITL/SITL.h>

#include <stdio.h>

using namespace SITL;

// forward conversion, copied from driver:
void MS5611::convert_forward(int32_t D1, int32_t D2, float &P_Pa, float &Temp_C)
{
    // _D1 and _D2 are stored as floats in driver
    const float _D1 = D1;
    const float _D2 = D2;

    float dT;
    float TEMP;
    float OFF;
    float SENS;

    dT = _D2-(((uint32_t)prom[5])<<8);
    TEMP = (dT * prom[6])/8388608;
    OFF = prom[2] * 65536.0f + (prom[4] * dT) / 128;
    SENS = prom[1] * 32768.0f + (prom[3] * dT) / 256;

    TEMP += 2000;

    if (TEMP < 2000) {
        // second order temperature compensation when under 20 degrees C
        float T2 = (dT*dT) / 0x80000000;
        float Aux = sq(TEMP-2000.0);
        float OFF2 = 2.5f*Aux;
        float SENS2 = 1.25f*Aux;
        if (TEMP < -1500) {
            // extra compensation for temperatures below -15C
            OFF2 += 7 * sq(TEMP+1500);
            SENS2 += sq(TEMP+1500) * 11.0*0.5;
        }
        TEMP = TEMP - T2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    P_Pa = (_D1*SENS/2097152 - OFF)/32768;
    Temp_C = TEMP * 0.01f;
}

void MS5611::convert(float P_Pa, float Temp_C, uint32_t &D1, uint32_t &D2)
{
    const uint8_t Q1 = Qx_coeff[0];
    const uint8_t Q2 = Qx_coeff[1];
    const uint8_t Q3 = Qx_coeff[2];
    const uint8_t Q4 = Qx_coeff[3];
    const uint8_t Q5 = Qx_coeff[4];
    const uint8_t Q6 = Qx_coeff[5];

    const float TEMP = Temp_C * 100;

    // second order temperature compensation when under 20 degrees C
    if (TEMP < 2000) {
        // Solve the quadratic equation for D2 when TEMP < 2000
        D2 = 128 * (2 * int64_t(prom[5]) - sqrt(sq(int64_t(prom[6])) - 131072 * (TEMP - 2000)) + int64_t(prom[6]));

        // Must compute the pressure compensation values using D2
        const float dT = float(D2) - (int64_t(prom[5]) << Q5);
        float TEMP_forward = 2000 + (dT * int64_t(prom[6])) / (1L << Q6);
        float OFF  = int64_t(prom[2]) * (1L << Q2) + (int64_t(prom[4]) * dT) / (1L << Q4);
        float SENS = int64_t(prom[1]) * (1L << Q1) + (int64_t(prom[3]) * dT) / (1L << Q3);

        const float Aux = sq(TEMP_forward - 2000);
        float OFF2  = 2.5 * Aux;
        float SENS2 = 1.25 * Aux;
        if (TEMP < -1500) {
            // extra compensation for temperatures below -15C
            OFF2  += 7 * sq(TEMP_forward + 1500);
            SENS2 += sq(TEMP_forward + 1500) * 11.0 * 0.5;
        }

        OFF = OFF - OFF2;
        SENS = SENS - SENS2;

        D1 = ((P_Pa * float(1L << 15) + OFF) * float(1L << 21)) / SENS;
    } else {
        const float dT = (TEMP - 2000) * (1L << Q6) / int64_t(prom[6]);
        const float OFF  = int64_t(prom[2]) * (1L << Q2) + (int64_t(prom[4]) * dT) / (1L << Q4);
        const float SENS = int64_t(prom[1]) * (1L << Q1) + (int64_t(prom[3]) * dT) / (1L << Q3);

        D1 = ((P_Pa * float(1L << 15) + OFF) * float(1L << 21)) / SENS;
        D2 = dT + (int64_t(prom[5]) << Q5);
    }
}

void MS5611::check_conversion_accuracy(float P_Pa, float Temp_C, uint32_t D1, uint32_t D2)
{
    float f_P_Pa;
    float f_Temp_C;
    convert_forward(D1, D2, f_P_Pa, f_Temp_C);

    if (fabs(f_P_Pa - P_Pa) > 0.2) {
        AP_HAL::panic("Invalid pressure conversion");
    }
    if (fabs(f_Temp_C - Temp_C) > 0.02) {
        AP_HAL::panic("Invalid temperature conversion");
    }
}

void MS5611::get_pressure_temperature_readings(float &P_Pa, float &Temp_C)
{
    float sim_alt = AP::sitl()->state.altitude;
    sim_alt += 2 * rand_float();

    float p, T_K;
    AP_Baro::get_pressure_temperature_for_alt_amsl(sim_alt, p, T_K);

    P_Pa = p;
    Temp_C = KELVIN_TO_C(T_K) + AP::sitl()->temp_board_offset;

    // TO DO add in temperature adjustment by inheritting from AP_Baro_SITL_Generic?
    // AP_Baro_SITL::temperature_adjustment(P_Pa, Temp_C);

    // TO DO add in wind correction by inheritting from AP_Baro_SITL_Generic?
    // P_Pa += AP_Baro_SITL::wind_pressure_correction(instance);
}
