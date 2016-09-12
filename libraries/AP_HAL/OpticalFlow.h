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

class AP_HAL::OpticalFlow {
public:
    class Data_Frame {
    public:
        float pixel_flow_x_integral;
        float pixel_flow_y_integral;
        float gyro_x_integral;
        float gyro_y_integral;
        uint32_t delta_time;
        uint8_t quality;
    };

    FUNCTOR_TYPEDEF(Gyro_Cb, void, float&, float&, float&);

    virtual void init(Gyro_Cb) = 0;
    virtual bool read(Data_Frame& frame) = 0;
};
