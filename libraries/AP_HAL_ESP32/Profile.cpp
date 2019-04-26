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
#ifdef ENABLE_PROFILE
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <errno.h>

#define ARRAY_SIZE(_arr) (sizeof(_arr) / sizeof(_arr[0]))

struct FI {
    uint32_t address;
    uint32_t count;
    struct FI * next;
};

FI fi[8192];

extern "C" __attribute__((section(".iram1"))) __attribute__((no_instrument_function))
void _mcount()
{
    void *a = __builtin_return_address(0);
    a = __builtin_extract_return_addr(a);
    uint32_t address = (uint32_t) a;
    uint32_t hash = address;
    hash = ((hash >> 16)^hash)*0x45d9f3b;
    hash = ((hash >> 16)^hash)*0x45d9f3b;
    uint32_t index = ((hash >> 16)^hash)%ARRAY_SIZE(fi);
    FI *current = &(fi[index]);
    while (current->next != nullptr && current->address != address) {
        current = current->next;
    }
    if (current->address == address) {
        current->count++;
    } else {
        if(current->address == 0) {
           current->address = address;
           current->count = 1;
        } else {
           FI* next = new FI();
           next->address = address;
           next->count = 1;
           next->next = nullptr;
           current->next = next;
        }
    }
}

void print_profile()
{
    static int64_t last_run = 0;
    static int counter = 0;
    if (AP_HAL::millis64() - last_run > 60000) {
        char fname[50];
        snprintf(fname, sizeof(fname), "/SDCARD/APM/PROF%03d.TXT", counter);
        ++counter;
        FILE *f = fopen(fname, "w");
        if (f != nullptr) {
            for (size_t i=0; i < ARRAY_SIZE(fi); i++) {
                FI *current = &(fi[i]);
                do {
                    if (current->address != 0) {
                        fprintf(f, "0x%016x 0x%x\n", current->address , current->count);
                    }
                    current->count = 0;
                    current = current->next;
                } while (current != nullptr);
            }
            fclose(f);
            printf("profile dumped %s\n", fname);
        } else {
            printf("profile open error %s %d\n", fname, errno);
        }
        last_run = AP_HAL::millis64();
    }
}
#else
void print_profile()
{
}
#endif

