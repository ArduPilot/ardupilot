/*
 *       APM_AHRS_HIL.cpp
 *
 *       Hardware in the loop AHRS object

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

#include <AP_AHRS.h>

/**************************************************/
void
AP_AHRS_HIL::setHil(float _roll, float _pitch, float _yaw,
                    float _rollRate, float _pitchRate, float _yawRate)
{
    roll = _roll;
    pitch = _pitch;
    yaw = _yaw;

    _omega(_rollRate, _pitchRate, _yawRate);

    roll_sensor  = ToDeg(roll)*100;
    pitch_sensor = ToDeg(pitch)*100;
    yaw_sensor   = ToDeg(yaw)*100;

    _dcm_matrix.from_euler(roll, pitch, yaw);
}
