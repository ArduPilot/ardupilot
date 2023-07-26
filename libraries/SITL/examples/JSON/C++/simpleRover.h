/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

class simpleRover {
public:
    struct simpleRoverState {
        double timestamp = 0;
        double gyro_x, gyro_y, gyro_z = 0; // rad/sec
        double accel_x, accel_y, accel_z = 0; // m/s^2
        double pos_x, pos_y, pos_z = 0; // m in inertial frame
        double phi, theta, psi = 0; // attitude radians
        double V_x, V_y, V_z = 0; // m/s in inertial frame
    } state, old_state;

    bool update(simpleRover &rover, uint16_t servo_out[]);
private:
    double _interp1D(const double &x, const double &x0, const double &x1, const double &y0, const double &y1);
};
