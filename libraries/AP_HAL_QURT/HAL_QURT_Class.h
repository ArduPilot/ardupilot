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

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_QURT_Namespace.h"
#include "interface.h"

class HAL_QURT : public AP_HAL::HAL
{
public:
    HAL_QURT();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
    void start_main_thread(Callbacks* callbacks);
    void main_thread(void);
};
