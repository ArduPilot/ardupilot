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

#pragma once

#include "AP_RCProtocol.h"
#include "AP_RCProtocol_SBUS.h"

class AP_RCProtocol_SBUS_NI : public AP_RCProtocol_SBUS {
public:
    AP_RCProtocol_SBUS_NI(AP_RCProtocol &_frontend) : AP_RCProtocol_SBUS(_frontend), saved_width(0) {}
    void process_pulse(uint32_t width_s0, uint32_t width_s1) override {
        AP_RCProtocol_SBUS::process_pulse(saved_width, width_s0);
        saved_width = width_s1;
    }
private:
    uint32_t saved_width;
};
