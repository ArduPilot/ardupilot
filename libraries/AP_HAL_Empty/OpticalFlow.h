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

class Empty::OpticalFlow : public AP_HAL::OpticalFlow {
public:
    void init() override { }
    bool read(Data_Frame& frame) override { return false; }
    void push_gyro(float gyro_x, float gyro_y, float dt) override { };
    void push_gyro_bias(float gyro_bias_x, float gyro_bias_y) override { }
};
