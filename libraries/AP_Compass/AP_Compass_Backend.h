// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
#ifndef __AP_COMPASS_BACKEND_H__
#define __AP_COMPASS_BACKEND_H__

#include "Compass.h"

class Compass;  // forward declaration
class AP_Compass_Backend
{
public:
    AP_Compass_Backend(Compass &compass);

    // we declare a virtual destructor so that drivers can
    // override with a custom destructor if need be.
    virtual ~AP_Compass_Backend(void) {}

    // initialize the magnetometers
    virtual bool init(void) = 0;

    // read sensor data
    virtual void read(void) = 0;

    // accumulate a reading from the magnetometer. Optional in
    // backends
    virtual void accumulate(void) {};

protected:
    // publish a magnetic field vector to the frontend
    void publish_field(const Vector3f &mag, uint8_t instance);

    // register a new compass instance with the frontend
    uint8_t register_compass(void) const;

    // set dev_id for an instance
    void set_dev_id(uint8_t instance, uint32_t dev_id);

    // set external state for an instance
    void set_external(uint8_t instance, bool external);

    // access to frontend
    Compass &_compass;

private:
    void apply_corrections(Vector3f &mag, uint8_t i);
};

#endif // __AP_COMPASS_BACKEND_H__
