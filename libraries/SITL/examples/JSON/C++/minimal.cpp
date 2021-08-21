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

// This is very minimal example of using libAP_JSON to send simulator data to ArduPilot SITL

#include <time.h>
#include <chrono>
#include <stdlib.h>

#include "libAP_JSON.cpp"

uint16_t servo_out[16];

uint64_t micros() {
    uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::
                  now().time_since_epoch()).count();
    return us; 
}

int main() {
    // init the ArduPilot connection
    libAP_JSON ap;
    if (ap.InitSockets("127.0.0.1", 9002))
    {
        std::cout << "started socket" << std::endl;
    }

    // send and receive data from AP
    while (true)
    {
        double timestamp = (double) micros() / 1000000.0;

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

        // example rangefinder data
        double rangefinder_example[] = {1, 2, 3, 4, 5, 6};

        // set the optionals
        ap.setAirspeed(1);
        ap.setWindvane(1 , 1);
        ap.setRangefinder(rangefinder_example, 6);

        // send with the required
        ap.SendState(timestamp,
                     0, 0, 0,    // gyro
                     0, 0, -9.81, // accel
                     0, 0, 0,    // position
                     0, 0, 0,    // attitude
                     0, 0, 0);    // velocity

        usleep(1000); // run this example at about 1000 Hz. Realistically it is about 800 Hz.
    }
    return 0;
}
