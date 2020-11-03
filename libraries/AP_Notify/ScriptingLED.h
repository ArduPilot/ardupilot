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
#pragma once

#include "RGBLed.h"
#include <AP_Common/AP_Common.h>

#ifdef ENABLE_SCRIPTING

class ScriptingLED: public RGBLed {
public:
    ScriptingLED();

    /* Do not allow copies */
    ScriptingLED(const AP_Notify &other) = delete;
    ScriptingLED &operator=(const AP_Notify&) = delete;

    // get singleton instance
    static ScriptingLED *get_singleton(void) {
        return _singleton;
    }

    void get_rgb(uint8_t& red, uint8_t& green, uint8_t& blue);

protected:

    bool hw_init() override  {return true;};
    bool hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue) override {return true;}

private:
    static ScriptingLED *_singleton;

};

#endif
