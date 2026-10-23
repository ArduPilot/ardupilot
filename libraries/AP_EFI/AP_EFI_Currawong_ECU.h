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
 * AP_EFI_Currawong_ECU.h
 *
 *      Author: Reilly Callaway / Currawong Engineering Pty Ltd
 */
 
#pragma once

#include "AP_EFI_config.h"

#if AP_EFI_CURRAWONG_ECU_ENABLED

#include "AP_EFI.h"
#include "AP_EFI_Backend.h"

class AP_EFI_Currawong_ECU : public AP_EFI_Backend {
public:
    AP_EFI_Currawong_ECU(AP_EFI &_frontend);
    
    void update() override;

    static AP_EFI_Currawong_ECU* get_instance(void)
    {
        return _singleton;
    }

private:
    bool handle_message(AP_HAL::CANFrame &frame);

    static AP_EFI_Currawong_ECU* _singleton;

    friend class AP_PiccoloCAN;
};

#endif // AP_EFI_CURRAWONG_ECU_ENABLED

