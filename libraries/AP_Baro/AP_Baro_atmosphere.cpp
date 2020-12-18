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
    float temp_K;
    float pressure_Pa;
    float density;
} atmospheric_table[] = {
    { -15000, 317.9, 169732.9, 1.86000},
    { -14000, 315.9, 164245.9, 1.81156},
    { -13000, 313.9, 158903.4, 1.76363},
    { -12000, 311.9, 153702.6, 1.71673},
    { -11000, 309.9, 148640.3, 1.67086},
    { -10000, 308.0, 143714.4, 1.62550},
    { -9000, 306.0, 138921.1, 1.58170},
    { -8000, 304.0, 134258.0, 1.53841},
    { -7000, 302.0, 129722.3, 1.49614},
    { -6000, 300.0, 125311.6, 1.45491},
    { -5000, 298.1, 121023.5, 1.41471},
    { -4000, 296.1, 116854.5, 1.37503},
    { -3000, 294.1, 112802.9, 1.33638},
    { -2000, 292.1, 108865.7, 1.29824},
    { -1000, 290.1, 105040.6, 1.26113},
    { 0, 288.2, 101325.1, 1.22506},
    { 1000, 286.2, 97716.8, 1.18949},
    { 2000, 284.2, 94212.9, 1.15496},
    { 3000, 282.2, 90811.5, 1.12095},
    { 4000, 280.2, 87510.7, 1.08796},
    { 5000, 278.2, 84307.5, 1.05550},
    { 6000, 276.3, 81199.6, 1.02406},
    { 7000, 274.3, 78185.5, 0.99313},
    { 8000, 272.3, 75262.4, 0.96273},
    { 9000, 270.3, 72428.4, 0.93335},
    { 10000, 268.3, 69681.5, 0.90449},
    { 11000, 266.4, 67019.8, 0.87666},
    { 12000, 264.4, 64440.5, 0.84934},
    { 13000, 262.4, 61942.6, 0.82254},
    { 14000, 260.4, 59523.7, 0.79626},
    { 15000, 258.4, 57181.9, 0.77101},
    { 16000, 256.5, 54915.2, 0.74575},
    { 17000, 254.5, 52721.9, 0.72153},
    { 18000, 252.5, 50599.8, 0.69834},
    { 19000, 250.5, 48547.7, 0.67515},
    { 20000, 248.5, 46563.0, 0.65247},
    { 21000, 246.5, 44644.9, 0.63082},
    { 22000, 244.6, 42791.5, 0.60969},
    { 23000, 242.6, 41000.3, 0.58856},
    { 24000, 240.6, 39270.9, 0.56846},
    { 25000, 238.6, 37600.8, 0.54888},
    { 26000, 236.6, 35988.7, 0.52981},
    { 27000, 234.7, 34433.1, 0.51120},
    { 28000, 232.7, 32932.0, 0.49306},
    { 29000, 230.7, 31484.6, 0.47544},
    { 31000, 226.7, 28744.4, 0.44163},
    { 32000, 224.8, 27448.8, 0.42545},
    { 33000, 222.8, 26200.5, 0.40973},
    { 34000, 220.8, 24998.7, 0.39442},
    { 35000, 218.8, 23841.9, 0.37958},
    { 36000, 216.8, 22729.2, 0.36520},
    { 36089, 216.7, 22632.0, 0.36391},
    { 37000, 216.7, 21662.4, 0.34834},
    { 38000, 216.7, 20645.9, 0.33201},
    { 39000, 216.7, 19677.3, 0.31639},
    { 40000, 216.7, 18753.7, 0.30155},
    { 41000, 216.7, 17873.7, 0.28743},
    { 42000, 216.7, 17034.8, 0.27392},
    { 43000, 216.7, 16235.7, 0.26109},
    { 44000, 216.7, 15473.9, 0.24882},
    { 45000, 216.7, 14747.6, 0.23713},
    { 46000, 216.7, 14055.7, 0.22599},
    { 47000, 216.7, 13395.9, 0.21543},
    { 48000, 216.7, 12767.3, 0.20528},
    { 49000, 216.7, 12168.3, 0.19564},
    { 50000, 216.7, 11597.1, 0.18646},
    { 51000, 216.7, 11053.1, 0.17775},
    { 52000, 216.7, 10534.1, 0.16941},
    { 53000, 216.7, 10040.0, 0.16142},
    { 54000, 216.7, 9568.9, 0.15384},
    { 55000, 216.7, 9119.7, 0.14663},
    { 56000, 216.7, 8691.7, 0.13977},
    { 57000, 216.7, 8283.8, 0.13323},
    { 58000, 216.7, 7895.0, 0.12694},
    { 59000, 216.7, 7524.9, 0.12101},
    { 60000, 216.7, 7171.5, 0.11534},
    { 61000, 216.7, 6834.9, 0.10993},
    { 62000, 216.7, 6514.1, 0.10472},
    { 63000, 216.7, 6208.6, 0.09983},
    { 64000, 216.7, 5917.0, 0.09514},
    { 65000, 216.7, 5639.3, 0.09071},
    { 65617, 216.7, 5474.6, 0.08803},
    { 66000, 216.8, 5375.0, 0.08638},
    { 67000, 217.1, 5123.2, 0.08220},
    { 68000, 217.4, 4883.3, 0.07823},
    { 69000, 217.7, 4654.9, 0.07447},
    { 70000, 218.0, 4437.5, 0.07092},
    { 71000, 218.3, 4230.7, 0.06751},
    { 72000, 218.6, 4033.9, 0.06427},
    { 73000, 218.9, 3846.2, 0.06123},
    { 74000, 219.2, 3667.6, 0.05829},
    { 76000, 219.8, 3335.8, 0.05288},
    { 77000, 220.1, 3181.6, 0.05035},
    { 78000, 220.4, 3034.6, 0.04796},
    { 79000, 220.7, 2894.8, 0.04569},
    { 80000, 221.0, 2761.3, 0.04352},
    { 81000, 221.3, 2634.4, 0.04146},
    { 82000, 221.6, 2513.2, 0.03950},
    { 83000, 222.0, 2398.3, 0.03764},
    { 84000, 222.3, 2288.2, 0.03587},
    { 85000, 222.6, 2183.8, 0.03418},
    { 86000, 222.9, 2083.7, 0.03258},
    { 87000, 223.2, 1988.9, 0.03105},
    { 88000, 223.5, 1898.5, 0.02959},
    { 89000, 223.8, 1811.8, 0.02821},
    { 90000, 224.1, 1729.4, 0.02689},
    { 91000, 224.4, 1650.9, 0.02563},
    { 92000, 224.7, 1576.2, 0.02444},
    { 93000, 225.0, 1504.9, 0.02330},
    { 94000, 225.3, 1436.9, 0.02222},
    { 95000, 225.6, 1372.2, 0.02119},
    { 96000, 225.9, 1310.0, 0.02020},
    { 97000, 226.2, 1251.1, 0.01927},
    { 98000, 226.5, 1195.1, 0.01838},
    { 99000, 226.8, 1141.5, 0.01753},
    { 100000, 227.1, 1090.2, 0.01672},
    { 105000, 228.7, 867.6, 0.01321},
    { 110000, 232.9, 692.3, 0.01035},
    { 115000, 237.2, 554.9, 0.00815},
    { 120000, 241.5, 446.2, 0.00644},
    { 125000, 245.7, 360.5, 0.00511},
    { 130000, 250.0, 292.1, 0.00407},
    { 135000, 254.3, 237.5, 0.00326},
    { 140000, 258.5, 193.9, 0.00261},
    { 145000, 262.8, 159.0, 0.00211},
    { 150000, 267.1, 130.7, 0.00170},
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
    const float temp_zero = 288.2;
    // to match SITL assumed board temperature behaviour we start at 30C
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

