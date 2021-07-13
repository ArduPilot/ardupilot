#include "SIM_MS5611.h"

#include <SITL/SITL.h>

#include <stdio.h>

using namespace SITL;

// forward conversion, copied from driver:
void MS5611::convert_forward(int32_t D1, int32_t D2, float &P_Pa, float &Temp_C)
{
    float dT;
    float TEMP;
    float OFF;
    float SENS;

    dT = D2-(((uint32_t)prom[5])<<8);
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


    P_Pa = (D1*SENS/2097152 - OFF)/32768;
    Temp_C = TEMP * 0.01f;
}

void MS5611::convert(float P_Pa, float Temp_C, uint32_t &D1, uint32_t &D2)
{
    int64_t TEMP = Temp_C * 100.0f;
    const float dT = (TEMP-2000.0) / (prom[6]/8388608.0);
    float OFF = prom[2] * 65536.0f + (prom[4] * dT) / 128;
    float SENS = prom[1] * 32768.0f + (prom[3] * dT) / 256;
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

    D1 = ((P_Pa*32768.0)+OFF) / (SENS/2097152.0);
    D2 = dT + (((uint32_t)prom[5])<<8);

    float f_P_Pa;
    float f_Temp_C;
    convert_forward(D1, D2, f_P_Pa, f_Temp_C);
    if (fabs(f_P_Pa - P_Pa) > 1) {
        AP_HAL::panic("Invalid pressure conversion");
    }
    if (fabs(f_Temp_C - Temp_C) > 0.1) {
        AP_HAL::panic("Invalid temperature conversion");
    }
}


void MS5611::get_pressure_temperature_readings(float &P_Pa, float &Temp_C)
{
    float sigma, delta, theta;

    float sim_alt = AP::sitl()->state.altitude;
    sim_alt += 2 * rand_float();

    AP_Baro::SimpleAtmosphere(sim_alt * 0.001f, sigma, delta, theta);
    P_Pa = SSL_AIR_PRESSURE * delta;

    Temp_C = (30.0 + C_TO_KELVIN) * theta - C_TO_KELVIN;  // Assume 30 degrees at sea level - converted to degrees Kelvin
}
