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

   HC-SR04 Ultrasonic Distance Sensor connected to BeagleBone Black
   by Mirko Denecke <mirkix@gmail.com>
 */

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI

#include "AP_RangeFinder_BBB_PRU.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>

extern const AP_HAL::HAL& hal;

volatile struct range *rangerpru;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_BBB_PRU::AP_RangeFinder_BBB_PRU(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
}

/*
   Stop PRU, load firmware (check if firmware is present), start PRU.
   If we get a result the sensor seems to be there.
*/
bool AP_RangeFinder_BBB_PRU::detect(RangeFinder &_ranger, uint8_t instance)
{
    bool result = true;
    uint32_t mem_fd;
    uint32_t *ctrl;
    void *ram;

    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    ctrl = (uint32_t*)mmap(0, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, PRU0_CTRL_BASE);
    ram = mmap(0, PRU0_IRAM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, PRU0_IRAM_BASE);

    // Reset PRU 0
    *ctrl = 0;
    hal.scheduler->delay(1);

    // Load firmware (.text)
    FILE *file = fopen("/lib/firmware/rangefinderprutext.bin", "rb");
    if(file == NULL)
    {
        result = false;
    }

    if(fread(ram, PRU0_IRAM_SIZE, 1, file) != 1)
    {
        result = false;
    }

    fclose(file);

    munmap(ram, PRU0_IRAM_SIZE);

    ram = mmap(0, PRU0_DRAM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, PRU0_DRAM_BASE);

    // Load firmware (.data)
    file = fopen("/lib/firmware/rangefinderprudata.bin", "rb");
    if(file == NULL)
    {
        result = false;
    }

    if(fread(ram, PRU0_DRAM_SIZE, 1, file) != 1)
    {
        result = false;
    }

    fclose(file);

    munmap(ram, PRU0_DRAM_SIZE);

    // Map PRU RAM
    ram = mmap(0, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, PRU0_DRAM_BASE);
    close(mem_fd);

    // Start PRU 0
    *ctrl = 2;

    rangerpru = (volatile struct range*)ram;

    return result;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_BBB_PRU::update(void)
{
    state.status = (RangeFinder::RangeFinder_Status)rangerpru->status;
    state.distance_cm = rangerpru->distance;
}
#endif // CONFIG_HAL_BOARD_SUBTYPE
