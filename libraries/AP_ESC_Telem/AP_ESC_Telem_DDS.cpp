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

#include "AP_ESC_Telem_DDS.h"

#if AP_ESC_TELEM_DDS_ENABLED

AP_ESC_Telem_DDS::AP_ESC_Telem_DDS()
{
}

void AP_ESC_Telem_DDS::handle_external(/*todo_add_data_type*/)
{
}

void AP_ESC_Telem_DDS::update()
{
    // backend protected methods
    // update_rpm();
    // update_telem_data();
}

#endif  // AP_ESC_TELEM_DDS_ENABLED
