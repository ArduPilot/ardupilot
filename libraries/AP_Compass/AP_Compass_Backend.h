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
  Compass driver backend class. Each supported compass sensor type
  needs to have an object derived from this class.
 */
#pragma once

#include "AP_Compass.h"

class Compass;  // forward declaration
class AP_Compass_Backend
{
public:
    AP_Compass_Backend();

    // we declare a virtual destructor so that drivers can
    // override with a custom destructor if need be.
    virtual ~AP_Compass_Backend(void) {}

    // read sensor data
    virtual void read(void) = 0;

    /*
      device driver IDs. These are used to fill in the devtype field
      of the device ID, which shows up as COMPASS*ID* parameters to
      users. The values are chosen for compatibility with existing PX4
      drivers.
      If a change is made to a driver that would make existing
      calibration values invalid then this number must be changed.
     */
    enum DevTypes {
        DEVTYPE_HMC5883_OLD = 0x01,
        DEVTYPE_HMC5883 = 0x07,
        DEVTYPE_LSM303D = 0x02,
        DEVTYPE_AK8963  = 0x04,
        DEVTYPE_BMM150  = 0x05,
        DEVTYPE_LSM9DS1 = 0x06,
        DEVTYPE_LIS3MDL = 0x08,
        DEVTYPE_AK09916 = 0x09,
        DEVTYPE_IST8310 = 0x0A,
        DEVTYPE_ICM20948 = 0x0B,
        DEVTYPE_MMC3416 = 0x0C,
        DEVTYPE_QMC5883L = 0x0D,
        DEVTYPE_MAG3110  = 0x0E,
        DEVTYPE_SITL  = 0x0F,
        DEVTYPE_IST8308 = 0x10,
		DEVTYPE_RM3100 = 0x11,
    };


protected:

    /*
     * A compass measurement is expected to pass through the following functions:
     * 1. rotate_field - this rotates the measurement in-place from sensor frame
     *      to body frame
     * 2. publish_raw_field - this provides an uncorrected point-sample for
     *      calibration libraries
     * 3. correct_field - this corrects the measurement in-place for hard iron,
     *      soft iron, motor interference, and non-orthagonality errors
     * 4. publish_filtered_field - legacy filtered magnetic field
     *
     * All those functions expect the mag field to be in milligauss.
     */

    void rotate_field(Vector3f &mag, uint8_t instance);
    void publish_raw_field(const Vector3f &mag, uint8_t instance);
    void correct_field(Vector3f &mag, uint8_t i);
    void publish_filtered_field(const Vector3f &mag, uint8_t instance);
    void set_last_update_usec(uint32_t last_update, uint8_t instance);

    void accumulate_sample(Vector3f &field, uint8_t instance,
                           uint32_t max_samples = 10);
    void drain_accumulated_samples(uint8_t instance, const Vector3f *scale = NULL);

    // register a new compass instance with the frontend
    uint8_t register_compass(void) const;

    // set dev_id for an instance
    void set_dev_id(uint8_t instance, uint32_t dev_id);

    // save dev_id, used by SITL
    void save_dev_id(uint8_t instance);

    // set external state for an instance
    void set_external(uint8_t instance, bool external);

    // tell if instance is an external compass
    bool is_external(uint8_t instance);

    // set rotation of an instance
    void set_rotation(uint8_t instance, enum Rotation rotation);

    // get board orientation (for SITL)
    enum Rotation get_board_orientation(void) const;
    
    // access to frontend
    Compass &_compass;

    // semaphore for access to shared frontend data
    HAL_Semaphore_Recursive _sem;

    // Check that the compass field is valid by using a mean filter on the vector length
    bool field_ok(const Vector3f &field);
    
    uint32_t get_error_count() const { return _error_count; }
private:
    void apply_corrections(Vector3f &mag, uint8_t i);
    
    // mean field length for range filter
    float _mean_field_length;
    // number of dropped samples. Not used for now, but can be usable to choose more reliable sensor
    uint32_t _error_count;
};
