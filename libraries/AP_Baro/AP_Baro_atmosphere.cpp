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
    int16_t amsl_kfeet;
    int16_t temp_cK; // in centi-degrees kelvin
    float density;
    float temp_K(void) const {
        return temp_cK * 0.1;
    }
    float amsl_meters(void) const {
        return amsl_kfeet * (1000.0 * FEET_TO_METERS);
    }
    float pressure_Pa(void) const {
        constexpr float Rho_specific = 287.053;
        return density * Rho_specific * temp_K();
    }
} atmospheric_table[] = {
    { -15, 3179, 1.86000},
    { -14, 3159, 1.81156},
    { -13, 3139, 1.76363},
    { -12, 3119, 1.71673},
    { -11, 3099, 1.67086},
    { -10, 3080, 1.62550},
    { -9, 3060, 1.58170},
    { -8, 3040, 1.53841},
    { -7, 3020, 1.49614},
    { -6, 3000, 1.45491},
    { -5, 2981, 1.41471},
    { -4, 2961, 1.37503},
    { -3, 2941, 1.33638},
    { -2, 2921, 1.29824},
    { -1, 2901, 1.26113},
    { 0, 2882, 1.22506},
    { 1, 2862, 1.18949},
    { 2, 2842, 1.15496},
    { 3, 2822, 1.12095},
    { 4, 2802, 1.08796},
    { 5, 2782, 1.05550},
    { 6, 2763, 1.02406},
    { 7, 2743, 0.99313},
    { 8, 2723, 0.96273},
    { 9, 2703, 0.93335},
    { 10, 2683, 0.90449},
    { 11, 2664, 0.87666},
    { 12, 2644, 0.84934},
    { 13, 2624, 0.82254},
    { 14, 2604, 0.79626},
    { 15, 2584, 0.77101},
    { 16, 2565, 0.74575},
    { 17, 2545, 0.72153},
    { 18, 2525, 0.69834},
    { 19, 2505, 0.67515},
    { 20, 2485, 0.65247},
    { 21, 2465, 0.63082},
    { 22, 2446, 0.60969},
    { 23, 2426, 0.58856},
    { 24, 2406, 0.56846},
    { 25, 2386, 0.54888},
    { 26, 2366, 0.52981},
    { 27, 2347, 0.51120},
    { 28, 2327, 0.49306},
    { 29, 2307, 0.47544},
    { 31, 2267, 0.44163},
    { 32, 2248, 0.42545},
    { 33, 2228, 0.40973},
    { 34, 2208, 0.39442},
    { 35, 2188, 0.37958},
    { 36, 2168, 0.36520},
    { 37, 2167, 0.34834},
    { 38, 2167, 0.33201},
    { 39, 2167, 0.31639},
    { 40, 2167, 0.30155},
    { 41, 2167, 0.28743},
    { 42, 2167, 0.27392},
    { 43, 2167, 0.26109},
    { 44, 2167, 0.24882},
    { 45, 2167, 0.23713},
    { 46, 2167, 0.22599},
    { 47, 2167, 0.21543},
    { 48, 2167, 0.20528},
    { 49, 2167, 0.19564},
    { 50, 2167, 0.18646},
    { 51, 2167, 0.17775},
    { 52, 2167, 0.16941},
    { 53, 2167, 0.16142},
    { 54, 2167, 0.15384},
    { 55, 2167, 0.14663},
    { 56, 2167, 0.13977},
    { 57, 2167, 0.13323},
    { 58, 2167, 0.12694},
    { 59, 2167, 0.12101},
    { 60, 2167, 0.11534},
    { 61, 2167, 0.10993},
    { 62, 2167, 0.10472},
    { 63, 2167, 0.09983},
    { 64, 2167, 0.09514},
    { 65, 2167, 0.09071},
    { 66, 2168, 0.08638},
    { 67, 2171, 0.08220},
    { 68, 2174, 0.07823},
    { 69, 2177, 0.07447},
    { 70, 2180, 0.07092},
    { 71, 2183, 0.06751},
    { 72, 2186, 0.06427},
    { 73, 2189, 0.06123},
    { 74, 2192, 0.05829},
    { 76, 2198, 0.05288},
    { 77, 2201, 0.05035},
    { 78, 2204, 0.04796},
    { 79, 2207, 0.04569},
    { 80, 2210, 0.04352},
    { 81, 2213, 0.04146},
    { 82, 2216, 0.03950},
    { 83, 2220, 0.03764},
    { 84, 2223, 0.03587},
    { 85, 2226, 0.03418},
    { 86, 2229, 0.03258},
    { 87, 2232, 0.03105},
    { 88, 2235, 0.02959},
    { 89, 2238, 0.02821},
    { 90, 2241, 0.02689},
    { 91, 2244, 0.02563},
    { 92, 2247, 0.02444},
    { 93, 2250, 0.02330},
    { 94, 2253, 0.02222},
    { 95, 2256, 0.02119},
    { 96, 2259, 0.02020},
    { 97, 2262, 0.01927},
    { 98, 2265, 0.01838},
    { 99, 2268, 0.01753},
    { 100, 2271, 0.01672},
    { 105, 2287, 0.01321},
    { 110, 2329, 0.01035},
    { 115, 2372, 0.00815},
    { 120, 2415, 0.00644},
    { 125, 2457, 0.00511},
    { 130, 2500, 0.00407},
    { 135, 2543, 0.00326},
    { 140, 2585, 0.00261},
    { 145, 2628, 0.00211},
    { 150, 2671, 0.00170},
};

/*
  find table entry given pressure. This returns at least 1
 */
static uint32_t lookup_atmospheric_table_by_pressure(float pressure)
{
    const uint32_t table_size = ARRAY_SIZE(atmospheric_table);
    uint32_t idx_min = 1;
    uint32_t idx_max = table_size-1;
    if (pressure > atmospheric_table[0].pressure_Pa()) {
        // off the bottom of the table
        idx_max = 1;
    }
    while (idx_min < idx_max) {
        const uint32_t mid = (idx_max+idx_min)/2;
        if (atmospheric_table[mid].pressure_Pa() < pressure) {
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
    if (alt_m <= atmospheric_table[0].amsl_meters()) {
        // off the bottom of the table
        idx_max = 1;
    }
    while (idx_min < idx_max) {
        const uint32_t mid = (idx_max+idx_min)/2;
        if (atmospheric_table[mid].amsl_meters() > alt_m) {
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
        (atmospheric_table[idx].amsl_meters() - atmospheric_table[idx-1].amsl_meters()) /
        (atmospheric_table[idx-1].pressure_Pa() - atmospheric_table[idx].pressure_Pa());
    return atmospheric_table[idx-1].amsl_meters() - slope * (pressure - atmospheric_table[idx-1].pressure_Pa());
}

/*
  return an interpolated density in meters given a pressure
 */
static float lookup_atmospheric_table_density_by_pressure(const float pressure)
{
    uint32_t idx = lookup_atmospheric_table_by_pressure(pressure);
    const float slope =
        (atmospheric_table[idx].density - atmospheric_table[idx-1].density) /
        (atmospheric_table[idx-1].pressure_Pa() - atmospheric_table[idx].pressure_Pa());
    return atmospheric_table[idx-1].density - slope * (pressure - atmospheric_table[idx-1].pressure_Pa());
}

/*
  return an interpolated density in meters given an altitude
 */
static float lookup_atmospheric_table_density_by_alt(const float alt_amsl)
{
    uint32_t idx = lookup_atmospheric_table_by_alt(alt_amsl);
    const float slope =
        (atmospheric_table[idx].density - atmospheric_table[idx-1].density) /
        (atmospheric_table[idx-1].amsl_meters() - atmospheric_table[idx].amsl_meters());
    return atmospheric_table[idx-1].density - slope * (alt_amsl - atmospheric_table[idx-1].amsl_meters());
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
    uint32_t idx = lookup_atmospheric_table_by_alt(alt_m);
    const float temp_zero = 288.2;
    // to match SITL assumed board temperature behaviour we start at 30C
    const float temp_sea_level = C_TO_KELVIN + 30;
    const float slopeP =
        (atmospheric_table[idx-1].pressure_Pa() - atmospheric_table[idx].pressure_Pa()) /
        (atmospheric_table[idx].amsl_meters() - atmospheric_table[idx-1].amsl_meters());
    const float slopeT =
        (atmospheric_table[idx-1].temp_K() - atmospheric_table[idx].temp_K()) /
        (atmospheric_table[idx].amsl_meters() - atmospheric_table[idx-1].amsl_meters());
    pressure = atmospheric_table[idx-1].pressure_Pa() - slopeP * (alt_m - atmospheric_table[idx-1].amsl_meters());
    temperature_K = atmospheric_table[idx-1].temp_K() - slopeT * (alt_m - atmospheric_table[idx-1].amsl_meters());
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

