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

    HC-SR04 Ultrasonic Distance Sensor by Mirko Denecke <mirkix@gmail.com>

*/

#include <stdint.h>

#include "pru_ctrl.h"

volatile register uint32_t __R30;
volatile register uint32_t __R31;

// Trigger pin
#define TRIGGER 1<<14

// Echo pin
#define ECHO 1<<14

#define TICKS_PER_US 200
#define TICKS_PER_MS (1000 * TICKS_PER_US)

#define TICKS_PER_CM 11600

#define COUNTER_MIN_DISTANCE (2 * TICKS_PER_CM)
#define COUNTER_MAX_DISTANCE (400 * TICKS_PER_CM)

enum RangeFinder_Status {
        RangeFinder_NotConnected = 0,
        RangeFinder_NoData,
        RangeFinder_OutOfRangeLow,
        RangeFinder_OutOfRangeHigh,
        RangeFinder_Good
};

struct range {
    uint32_t distance;
    uint32_t status;
};

#pragma LOCATION(ranger, 0x0)
volatile struct range ranger;

main()
{
    // Init data
    ranger.distance = 0;
    ranger.status = RangeFinder_NoData;

    // Reset trigger
    __R30 &= ~(TRIGGER);

    // Wait 100ms
    __delay_cycles(250 * TICKS_PER_MS);

    // Disable counter
    PRU0_CTRL.CONTROL_bit.CTR_EN = 0;

    // Clear counter
    PRU0_CTRL.CYCLE_bit.CYCLECOUNT = 0xFFFFFFFF;

    while(1)
    {
        // Enable trigger
        __R30 |= TRIGGER;

        // Wait 15us
        __delay_cycles(3000);

        // Reset trigger
        __R30 &= ~(TRIGGER);

        // Wait for echo
        while((__R31 & ECHO) == 0)
        {
            if(PRU0_CTRL.CYCLE_bit.CYCLECOUNT > (600 * TICKS_PER_CM))
            {
                ranger.status = RangeFinder_NoData;
            }
        }

        // Start counter
        PRU0_CTRL.CONTROL_bit.CTR_EN = 1;

        // Count echo length
        while(__R31 & ECHO)
        {
            ;
        }

        // Stop counter
        PRU0_CTRL.CONTROL_bit.CTR_EN = 0;

        // Check status
        if(PRU0_CTRL.CYCLE_bit.CYCLECOUNT < COUNTER_MIN_DISTANCE)
        {
            ranger.distance = 0;

            // Set status out of range low
            ranger.status = RangeFinder_OutOfRangeLow;
        }
        else if(PRU0_CTRL.CYCLE_bit.CYCLECOUNT > COUNTER_MAX_DISTANCE)
        {
            ranger.distance = 0;

            // Set stauts
            ranger.status = RangeFinder_OutOfRangeHigh;
        }
        else
        {
            // Calculate distance in cm
            ranger.distance = PRU0_CTRL.CYCLE_bit.CYCLECOUNT / TICKS_PER_CM;

            //  Set status out of range high
            ranger.status = RangeFinder_Good;
        }

        // Clear counter
        PRU0_CTRL.CYCLE_bit.CYCLECOUNT = 0xFFFFFFFF;

        // Wait 20ms
        __delay_cycles(20 * TICKS_PER_MS);
    }
}

