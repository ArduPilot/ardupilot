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
 *	Adam M Rivera
 *	With direction from: Andrew Tridgell, Jason Short, Justin Beech
 *
 *	Adapted from: http://www.societyofrobots.com/robotforum/index.php?topic=11855.0
 *	Scott Ferguson
 *	scottfromscott@gmail.com
 *
 */
#include "AP_Declination.h"

#include <cmath>

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

/*
  calculate magnetic field intensity and orientation
*/
bool AP_Declination::get_mag_field_ef(float latitude_deg, float longitude_deg, float &intensity_gauss, float &declination_deg, float &inclination_deg)
{
    bool valid_input_data = true;

    /* round down to nearest sampling resolution */
    int32_t min_lat = static_cast<int32_t>(static_cast<int32_t>(floorf(latitude_deg / SAMPLING_RES)) * SAMPLING_RES);
    int32_t min_lon = static_cast<int32_t>(static_cast<int32_t>(floorf(longitude_deg / SAMPLING_RES)) * SAMPLING_RES);

    /* for the rare case of hitting the bounds exactly
     * the rounding logic wouldn't fit, so enforce it.
     */

    /* limit to table bounds - required for maxima even when table spans full globe range */
    if (latitude_deg <= SAMPLING_MIN_LAT) {
        min_lat = static_cast<int32_t>(SAMPLING_MIN_LAT);
        valid_input_data = false;
    }

    if (latitude_deg >= SAMPLING_MAX_LAT) {
        min_lat = static_cast<int32_t>(static_cast<int32_t>(latitude_deg / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES);
        valid_input_data = false;
    }

    if (longitude_deg <= SAMPLING_MIN_LON) {
        min_lon = static_cast<int32_t>(SAMPLING_MIN_LON);
        valid_input_data = false;
    }

    if (longitude_deg >= SAMPLING_MAX_LON) {
        min_lon = static_cast<int32_t>(static_cast<int32_t>(longitude_deg / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES);
        valid_input_data = false;
    }

    /* find index of nearest low sampling point */
    uint32_t min_lat_index = static_cast<uint32_t>((-(SAMPLING_MIN_LAT) + min_lat)  / SAMPLING_RES);
    uint32_t min_lon_index = static_cast<uint32_t>((-(SAMPLING_MIN_LON) + min_lon) / SAMPLING_RES);

    /* calculate intensity */

    float data_sw = intensity_table[min_lat_index][min_lon_index];
    float data_se = intensity_table[min_lat_index][min_lon_index + 1];;
    float data_ne = intensity_table[min_lat_index + 1][min_lon_index + 1];
    float data_nw = intensity_table[min_lat_index + 1][min_lon_index];

    /* perform bilinear interpolation on the four grid corners */

    float data_min = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_se - data_sw) + data_sw;
    float data_max = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_ne - data_nw) + data_nw;

    intensity_gauss = ((latitude_deg - min_lat) / SAMPLING_RES) * (data_max - data_min) + data_min;

    /* calculate declination */

    data_sw = declination_table[min_lat_index][min_lon_index];
    data_se = declination_table[min_lat_index][min_lon_index + 1];;
    data_ne = declination_table[min_lat_index + 1][min_lon_index + 1];
    data_nw = declination_table[min_lat_index + 1][min_lon_index];

    /* perform bilinear interpolation on the four grid corners */

    data_min = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_se - data_sw) + data_sw;
    data_max = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_ne - data_nw) + data_nw;

    declination_deg = ((latitude_deg - min_lat) / SAMPLING_RES) * (data_max - data_min) + data_min;

    /* calculate inclination */

    data_sw = inclination_table[min_lat_index][min_lon_index];
    data_se = inclination_table[min_lat_index][min_lon_index + 1];;
    data_ne = inclination_table[min_lat_index + 1][min_lon_index + 1];
    data_nw = inclination_table[min_lat_index + 1][min_lon_index];

    /* perform bilinear interpolation on the four grid corners */

    data_min = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_se - data_sw) + data_sw;
    data_max = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_ne - data_nw) + data_nw;

    inclination_deg = ((latitude_deg - min_lat) / SAMPLING_RES) * (data_max - data_min) + data_min;

    return valid_input_data;
}


/*
 calculate magnetic field intensity and orientation
*/
float AP_Declination::get_declination(float latitude_deg, float longitude_deg)
{
    float declination_deg=0, inclination_deg=0, intensity_gauss=0;

    get_mag_field_ef(latitude_deg, longitude_deg, intensity_gauss, declination_deg, inclination_deg);

    return declination_deg;
}

/*
  get earth field as a Vector3f in Gauss given a Location
*/
Vector3f AP_Declination::get_earth_field_ga(const Location &loc)
{
    float declination_deg=0, inclination_deg=0, intensity_gauss=0;
    get_mag_field_ef(loc.lat*1.0e-7f, loc.lng*1.0e-7f, intensity_gauss, declination_deg, inclination_deg);

    // create earth field
    Vector3f mag_ef = Vector3f(intensity_gauss, 0.0, 0.0);
    Matrix3f R;

    R.from_euler(0.0f, -ToRad(inclination_deg), ToRad(declination_deg));
    mag_ef = R * mag_ef;
    return mag_ef;
}
