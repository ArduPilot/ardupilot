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
  Flymaple port by Mike McCauley
 */

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

#include <stdarg.h>
#include "Console.h"
#include "FlymapleWirish.h"

// Flymaple: Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects (eg SerialUSB) that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
}

using namespace AP_HAL_FLYMAPLE_NS;

FLYMAPLEConsoleDriver::FLYMAPLEConsoleDriver(void* notused)
{}

void FLYMAPLEConsoleDriver::init(void* machtnichts)
{
}

void FLYMAPLEConsoleDriver::backend_open()
{}

void FLYMAPLEConsoleDriver::backend_close()
{}

size_t FLYMAPLEConsoleDriver::backend_read(uint8_t *data, size_t len) {
    return 0;
}

size_t FLYMAPLEConsoleDriver::backend_write(const uint8_t *data, size_t len) {
    return 0;
}

int16_t FLYMAPLEConsoleDriver::available() {
    return SerialUSB.available();
}

int16_t FLYMAPLEConsoleDriver::txspace() {
    // REVISIT mikem
    return 0;
}

int16_t FLYMAPLEConsoleDriver::read() {
    return SerialUSB.read();
}

size_t FLYMAPLEConsoleDriver::write(uint8_t c) {
    SerialUSB.write(c);
    return 1;
}

#endif
