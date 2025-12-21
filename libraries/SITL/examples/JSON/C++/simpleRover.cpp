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

// This is a very simple 1-D rover model using libAP_JSON for C++

#include <math.h>
#include <time.h>
#include <chrono>
#include <stdlib.h>

#include "libAP_JSON.h"
#include "simpleRover.h"

uint16_t servo_out[16];

uint64_t micros() {
    uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::
                  now().time_since_epoch()).count();
    return us; 
}

bool simpleRover::update(simpleRover &rover, uint16_t servo_out[]) {
    // returns true is the update went well. false means something went wrong and the physics model should exit
    // this is main portion of the physics

    // ideal vehicle model
    /* servos are defined as
        1. throttle (really just velocity control)
        2. steering (really just turn rate omega)
     */

    double timestep = rover.state.timestamp - rover.old_state.timestamp;
    if (timestep < 0) {
        // the sim is trying to go backwards in time
        std::cout << "[simpleRover] Error: Time went backwards" << std::endl;
        return false;
    } else if (timestep == 0) {
        // time did not advance. no physics step
        std::cout << "[simpleRover] Warning: Time did not step forward" << std::endl;
        return true;
    } else if (timestep > 60) {
        // limiting timestep to less than 1 minute
        std::cout << "[simpleRover] Warning: Time step was very large" << std::endl;
        return true;
    }

    // how fast is the rover moving
    double max_velocity = 1; // m/s
    double body_v = _interp1D(servo_out[2], 1100, 1900, -max_velocity, max_velocity);

    // how fast is the rover turning
    // Just doing 1-D right now. This Needs a bit of system dynamics math and geometry to get to 2-D.
    // double max_turn_rate = 5 * M_PI / 180;
    // double body_omega_z = _interp1D(servo_out[1], 1100, 1900, -max_turn_rate, max_turn_rate);

    // update the state
    rover.state.V_x = body_v;
    rover.state.accel_x = (rover.state.V_x - rover.old_state.V_x) / timestep; // derivative for accel
    double delta_pos_x = (rover.state.V_x) * timestep; // integrate for position change
    rover.state.pos_x = delta_pos_x + rover.old_state.pos_x; // plus c

    // update successful
    return true;
}

int main() {
    // init the ArduPilot connection
    libAP_JSON ap;
    if (ap.InitSockets("127.0.0.1", 9002))
    {
        std::cout << "started socket" << std::endl;
    }

    // init a simpleRover
    simpleRover rover;

    // send and receive data from AP
    while (true)
    {
        rover.state.timestamp = (double) micros() / 1000000.0;

        if (ap.ReceiveServoPacket(servo_out))
        {
#if DEBUG_ENABLED
            std::cout << "servo_out PWM: [";
            for (int i = 0; i < MAX_SERVO_CHANNELS - 1; ++i)
            {
                std::cout << servo_out[i] << ", ";
            }
            std::cout << servo_out[MAX_SERVO_CHANNELS - 1] << "]" << std::endl;
#endif
        }

        if (!ap.ap_online) {
            continue;
        }

        // calc rover physics
        if (!rover.update(rover, servo_out)) {
            // something went wrong with the physics
            std::cout << "[simpleRover] Error: Physics update has caused an exit" << std::endl;
            return 1;
        };

        // step the sim forward
        rover.old_state = rover.state;

        // send with the required state
        ap.SendState(rover.state.timestamp,
                     0, 0, 0,    // gyro
                     rover.state.accel_x, 0, -9.81, // accel
                     rover.state.pos_x, 0, 0,    // position
                     0, 0, 0,    // attitude
                     rover.state.V_x, 0, 0);    // velocity
    }
    return 0;
}

double simpleRover::_interp1D(const double &x, const double &x0, const double &x1, const double &y0, const double &y1)
{
    return ((y0 * (x1 - x)) + (y1 * (x - x0))) / (x1 - x0);
}
