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

#include <AP_Rally/AP_Rally.h>
#include <AP_AHRS/AP_AHRS.h>

class AP_Rally_Copter : public AP_Rally
{
public:
    static AP_Rally_Copter create(AP_AHRS &ahrs) {
        return AP_Rally_Copter{ahrs};
    }

    constexpr AP_Rally_Copter(AP_Rally_Copter &&other) = default;

    /* Do not allow copies */
    AP_Rally_Copter(const AP_Rally_Copter &other) = delete;
    AP_Rally_Copter &operator=(const AP_Rally_Copter&) = delete;

private:
    AP_Rally_Copter(AP_AHRS &ahrs) : AP_Rally(ahrs) { }

    bool is_valid(const Location &rally_point) const override;
};
