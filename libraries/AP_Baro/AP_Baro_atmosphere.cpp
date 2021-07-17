#include "AP_Baro.h"
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  atmospheric model options
 */

#ifndef HAL_BARO_ATMOSPHERIC_TABLE
// default to using tables when doing double precision EKF (which implies more flash space and faster MCU)
// this allows for non-table testing with --ekf-single configure option
#define HAL_BARO_ATMOSPHERIC_TABLE HAL_WITH_EKF_DOUBLE
#endif

/*
  return altitude difference in meters between current pressure and a
  given base_pressure in Pascal. This is a simple atmospheric model
  good to about 11km AMSL. Enable the table based function for higher altitudes
*/
float AP_Baro::get_altitude_difference_function(float base_pressure, float pressure) const
{
    float ret;
    float temp    = get_ground_temperature() + C_TO_KELVIN;
    float scaling = pressure / base_pressure;

    // This is an exact calculation that is within +-2.5m of the standard
    // atmosphere tables in the troposphere (up to 11,000 m amsl).
    ret = 153.8462f * temp * (1.0f - expf(0.190259f * logf(scaling)));

    return ret;
}

#if HAL_BARO_ATMOSPHERIC_TABLE || CONFIG_HAL_BOARD == HAL_BOARD_SITL
/*
  MIL-STD 3013 (1962 U.S. Standard Atmosphere) in Tabular Form.

  Using this table is more accurate over a much wider range of
  altitudes than using the function
 */
static const struct {
    float amsl_feet;
    float pressure_Pa;
    float density;
    float temp_K;
} atmospheric_table[] = {
    {-15000, 169732.9, 1.86000, 357.14 },
    {-14000, 164245.9, 1.81156, 356.01 },
    {-13000, 158903.4, 1.76363, 354.91 },
    {-12000, 153702.6, 1.71673, 353.79 },
    {-11000, 148640.3, 1.67086, 352.66 },
    {-10000, 143714.4, 1.62550, 351.53 },
    {-9000, 138921.1, 1.58170, 350.41 },
    {-8000, 134258.0, 1.53841, 349.25 },
    {-7000, 129722.3, 1.49614, 348.12 },
    {-6000, 125311.6, 1.45491, 346.96 },
    {-5000, 121023.5, 1.41471, 345.84 },
    {-4000, 116854.5, 1.37503, 344.68 },
    {-3000, 112802.9, 1.33638, 343.52 },
    {-2000, 108865.7, 1.29824, 342.37 },
    {-1000, 105040.6, 1.26113, 341.21 },
    {0, 101325.1, 1.22506, 340.02 },
    {1000, 97716.8, 1.18949, 338.86 },
    {2000, 94212.9, 1.15496, 337.68 },
    {3000, 90811.5, 1.12095, 336.52 },
    {4000, 87510.7, 1.08796, 335.33 },
    {5000, 84307.5, 1.05550, 334.14 },
    {6000, 81199.6, 1.02406, 332.95 },
    {7000, 78185.5, 0.99313, 331.74 },
    {8000, 75262.4, 0.96273, 330.55 },
    {9000, 72428.4, 0.93335, 329.33 },
    {10000, 69681.5, 0.90449, 328.14 },
    {11000, 67019.8, 0.87666, 326.92 },
    {12000, 64440.5, 0.84934, 325.71 },
    {13000, 61942.6, 0.82254, 324.49 },
    {14000, 59523.7, 0.79626, 323.27 },
    {15000, 57181.9, 0.77101, 322.02 },
    {16000, 54915.2, 0.74575, 320.77 },
    {17000, 52721.9, 0.72153, 319.55 },
    {18000, 50599.8, 0.69834, 318.30 },
    {19000, 48547.7, 0.67515, 317.06 },
    {20000, 46563.0, 0.65247, 315.78 },
    {21000, 44644.9, 0.63082, 314.53 },
    {22000, 42791.5, 0.60969, 313.25 },
    {23000, 41000.3, 0.58856, 312.00 },
    {24000, 39270.9, 0.56846, 310.72 },
    {25000, 37600.8, 0.54888, 309.44 },
    {26000, 35988.7, 0.52981, 308.13 },
    {27000, 34433.1, 0.51120, 306.85 },
    {28000, 32932.0, 0.49306, 305.54 },
    {29000, 31484.6, 0.47544, 304.26 },
    {31000, 28744.4, 0.44163, 301.61 },
    {32000, 27448.8, 0.42545, 300.30 },
    {33000, 26200.5, 0.40973, 298.96 },
    {34000, 24998.7, 0.39442, 297.65 },
    {35000, 23841.9, 0.37958, 296.31 },
    {36000, 22729.2, 0.36520, 294.97 },
    {36089, 22632.0, 0.36391, 294.85 },
    {37000, 21662.4, 0.34834, 294.85 },
    {38000, 20645.9, 0.33201, 294.85 },
    {39000, 19677.3, 0.31639, 294.85 },
    {40000, 18753.7, 0.30155, 294.85 },
    {41000, 17873.7, 0.28743, 294.85 },
    {42000, 17034.8, 0.27392, 294.85 },
    {43000, 16235.7, 0.26109, 294.85 },
    {44000, 15473.9, 0.24882, 294.85 },
    {45000, 14747.6, 0.23713, 294.85 },
    {46000, 14055.7, 0.22599, 294.85 },
    {47000, 13395.9, 0.21543, 294.85 },
    {48000, 12767.3, 0.20528, 294.85 },
    {49000, 12168.3, 0.19564, 294.85 },
    {50000, 11597.1, 0.18646, 294.85 },
    {51000, 11053.1, 0.17775, 294.85 },
    {52000, 10534.1, 0.16941, 294.85 },
    {53000, 10040.0, 0.16142, 294.85 },
    {54000, 9568.9, 0.15384, 294.85 },
    {55000, 9119.7, 0.14663, 294.85 },
    {56000, 8691.7, 0.13977, 294.85 },
    {57000, 8283.8, 0.13323, 294.85 },
    {58000, 7895.0, 0.12694, 294.85 },
    {59000, 7524.9, 0.12101, 294.85 },
    {60000, 7171.5, 0.11534, 294.85 },
    {61000, 6834.9, 0.10993, 294.85 },
    {62000, 6514.1, 0.10472, 294.85 },
    {63000, 6208.6, 0.09983, 294.85 },
    {64000, 5917.0, 0.09514, 294.85 },
    {65000, 5639.3, 0.09071, 294.85 },
    {65617, 5474.6, 0.08803, 294.85 },
    {66000, 5375.0, 0.08638, 294.91 },
    {67000, 5123.2, 0.08220, 295.13 },
    {68000, 4883.3, 0.07823, 295.34 },
    {69000, 4654.9, 0.07447, 295.55 },
    {70000, 4437.5, 0.07092, 295.74 },
    {71000, 4230.7, 0.06751, 295.95 },
    {72000, 4033.9, 0.06427, 296.16 },
    {73000, 3846.2, 0.06123, 296.38 },
    {74000, 3667.6, 0.05829, 296.59 },
    {76000, 3335.8, 0.05288, 296.98 },
    {77000, 3181.6, 0.05035, 297.20 },
    {78000, 3034.6, 0.04796, 297.41 },
    {79000, 2894.8, 0.04569, 297.59 },
    {80000, 2761.3, 0.04352, 297.81 },
    {81000, 2634.4, 0.04146, 298.02 },
    {82000, 2513.2, 0.03950, 298.23 },
    {83000, 2398.3, 0.03764, 298.42 },
    {84000, 2288.2, 0.03587, 298.63 },
    {85000, 2183.8, 0.03418, 298.84 },
    {86000, 2083.7, 0.03258, 299.06 },
    {87000, 1988.9, 0.03105, 299.24 },
    {88000, 1898.5, 0.02959, 299.45 },
    {89000, 1811.8, 0.02821, 299.66 },
    {90000, 1729.4, 0.02689, 299.85 },
    {91000, 1650.9, 0.02563, 300.06 },
    {92000, 1576.2, 0.02444, 300.27 },
    {93000, 1504.9, 0.02330, 300.46 },
    {94000, 1436.9, 0.02222, 300.67 },
    {95000, 1372.2, 0.02119, 300.88 },
    {96000, 1310.0, 0.02020, 301.07 },
    {97000, 1251.1, 0.01927, 301.28 },
    {98000, 1195.1, 0.01838, 301.49 },
    {99000, 1141.5, 0.01753, 301.68 },
    {100000, 1090.2, 0.01672, 301.89 },
    {105000, 867.6, 0.01321, 302.89 },
    {110000, 692.3, 0.01035, 305.73 },
    {115000, 554.9, 0.00815, 308.50 },
    {120000, 446.2, 0.00644, 311.27 },
    {125000, 360.5, 0.00511, 314.01 },
    {130000, 292.1, 0.00407, 316.72 },
    {135000, 237.5, 0.00326, 319.40 },
    {140000, 193.9, 0.00261, 322.08 },
    {145000, 159.0, 0.00211, 324.73 },
    {150000, 130.7, 0.00170, 327.35 },
};

/*
  find table entry given pressure. This returns at least 1
 */
static uint32_t lookup_atmospheric_table_by_pressure(float pressure)
{
    const uint32_t table_size = ARRAY_SIZE(atmospheric_table);
    uint32_t idx_min = 1;
    uint32_t idx_max = table_size-1;
    if (pressure > atmospheric_table[0].pressure_Pa) {
        // off the bottom of the table
        idx_max = 1;
    }
    while (idx_min < idx_max) {
        const uint32_t mid = (idx_max+idx_min)/2;
        if (atmospheric_table[mid].pressure_Pa < pressure) {
            idx_max = mid;
        } else {
            idx_min = mid+1;
        }
    }
    return idx_min;
}

/*
  find table entry given altitude in meters. This returns at least 1
 */
static uint32_t lookup_atmospheric_table_by_alt(float alt_m)
{
    const uint32_t table_size = ARRAY_SIZE(atmospheric_table);
    uint32_t idx_min = 1;
    uint32_t idx_max = table_size-1;
    float alt_feet = alt_m / FEET_TO_METERS;
    if (alt_feet <= atmospheric_table[0].amsl_feet) {
        // off the bottom of the table
        idx_max = 1;
    }
    while (idx_min < idx_max) {
        const uint32_t mid = (idx_max+idx_min)/2;
        if (atmospheric_table[mid].amsl_feet > alt_feet) {
            idx_max = mid;
        } else {
            idx_min = mid+1;
        }
    }
    return idx_min;
}

/*
  return an interpolated altitude in meters given a pressure
 */
static float lookup_atmospheric_table_alt(const float pressure)
{
    uint32_t idx = lookup_atmospheric_table_by_pressure(pressure);
    const float slope =
        (atmospheric_table[idx].amsl_feet - atmospheric_table[idx-1].amsl_feet) /
        (atmospheric_table[idx-1].pressure_Pa - atmospheric_table[idx].pressure_Pa);
    const float feet = atmospheric_table[idx-1].amsl_feet - slope * (pressure - atmospheric_table[idx-1].pressure_Pa);
    return FEET_TO_METERS * feet;
}

/*
  return an interpolated density in meters given a pressure
 */
static float lookup_atmospheric_table_density_by_pressure(const float pressure)
{
    uint32_t idx = lookup_atmospheric_table_by_pressure(pressure);
    const float slope =
        (atmospheric_table[idx].density - atmospheric_table[idx-1].density) /
        (atmospheric_table[idx-1].pressure_Pa - atmospheric_table[idx].pressure_Pa);
    return atmospheric_table[idx-1].density - slope * (pressure - atmospheric_table[idx-1].pressure_Pa);
}

/*
  return an interpolated density in meters given an altitude
 */
static float lookup_atmospheric_table_density_by_alt(const float alt_amsl)
{
    uint32_t idx = lookup_atmospheric_table_by_alt(alt_amsl);
    const float alt_feet = alt_amsl / FEET_TO_METERS;
    const float slope =
        (atmospheric_table[idx].density - atmospheric_table[idx-1].density) /
        (atmospheric_table[idx-1].amsl_feet - atmospheric_table[idx].amsl_feet);
    return atmospheric_table[idx-1].density - slope * (alt_feet - atmospheric_table[idx-1].amsl_feet);
}


/*
  return altitude difference in meters between current pressure and a
  given base_pressure in Pascal. This is a simple atmospheric model
  good to about 11km AMSL. Enable the table based function for higher altitudes
*/
float AP_Baro::get_altitude_difference_table(float base_pressure, float pressure) const
{
    const float alt1 = lookup_atmospheric_table_alt(base_pressure);
    const float alt2 = lookup_atmospheric_table_alt(pressure);
    return alt2 - alt1;
}

/*
  lookup expected pressure for a given altitude. Used for SITL
*/
void AP_Baro::get_pressure_temperature_for_alt_amsl(float alt_m, float &pressure, float &temperature_K)
{
    const float alt_feet = alt_m / FEET_TO_METERS;
    uint32_t idx = lookup_atmospheric_table_by_alt(alt_m);
    const float temp_zero = 340.02;
    const float temp_sea_level = C_TO_KELVIN + 30;
    const float slopeP =
        (atmospheric_table[idx-1].pressure_Pa - atmospheric_table[idx].pressure_Pa) /
        (atmospheric_table[idx].amsl_feet - atmospheric_table[idx-1].amsl_feet);
    const float slopeT =
        (atmospheric_table[idx-1].temp_K - atmospheric_table[idx].temp_K) /
        (atmospheric_table[idx].amsl_feet - atmospheric_table[idx-1].amsl_feet);
    pressure = atmospheric_table[idx-1].pressure_Pa - slopeP * (alt_feet - atmospheric_table[idx-1].amsl_feet);
    temperature_K = atmospheric_table[idx-1].temp_K - slopeT * (alt_feet - atmospheric_table[idx-1].amsl_feet);
    temperature_K += (temp_sea_level - temp_zero);
}

/*
  return current scale factor that converts from equivalent to true airspeed
  uses atmospheric tables
*/
float AP_Baro::get_EAS2TAS_table(float pressure)
{
    float density = lookup_atmospheric_table_density_by_pressure(pressure);
    if (!is_positive(density)) {
        // we really are not likely to get this high
        const uint32_t table_size = ARRAY_SIZE(atmospheric_table);
        density = atmospheric_table[table_size-1].density;
    }
    return sqrtf(SSL_AIR_DENSITY / density);
}

/*
  return air density for altitude, used for SITL
*/
float AP_Baro::get_air_density_for_alt_amsl(float alt_amsl)
{
    return lookup_atmospheric_table_density_by_alt(alt_amsl);
}

/*
  return scale factor that converts from equivalent to true airspeed
  used by SITL only
*/
float AP_Baro::get_EAS2TAS_for_alt_amsl(float alt_amsl)
{
    const float density = get_air_density_for_alt_amsl(alt_amsl);
    return sqrtf(SSL_AIR_DENSITY / MAX(0.00001,density));
}

#endif // HAL_BARO_ATMOSPHERIC_TABLE || CONFIG_HAL_BOARD == HAL_BOARD_SITL

/*
  return altitude difference given two pressures
 */
float AP_Baro::get_altitude_difference(float base_pressure, float pressure) const
{
#if HAL_BARO_ATMOSPHERIC_TABLE
    return get_altitude_difference_table(base_pressure, pressure);
#else
    return get_altitude_difference_function(base_pressure, pressure);
#endif
}


/*
  return current scale factor that converts from equivalent to true airspeed
  valid for altitudes up to 10km AMSL
  assumes standard atmosphere lapse rate
*/
float AP_Baro::get_EAS2TAS_function(float altitude, float pressure)
{
    if (is_zero(pressure)) {
        return 1.0f;
    }

    // only estimate lapse rate for the difference from the ground location
    // provides a more consistent reading then trying to estimate a complete
    // ISA model atmosphere
    float tempK = get_ground_temperature() + C_TO_KELVIN - ISA_LAPSE_RATE * altitude;
    const float eas2tas_squared = SSL_AIR_DENSITY / (pressure / (ISA_GAS_CONSTANT * tempK));
    if (!is_positive(eas2tas_squared)) {
        return 1.0f;
    }
    return sqrtf(eas2tas_squared);
}


/*
  return current scale factor that converts from equivalent to true airspeed
*/
float AP_Baro::get_EAS2TAS(void)
{
    const float pressure = get_pressure();

#if HAL_BARO_ATMOSPHERIC_TABLE
    return get_EAS2TAS_table(pressure);
#else
    // otherwise use function
    return get_EAS2TAS_function(get_altitude(), pressure);
#endif
}

