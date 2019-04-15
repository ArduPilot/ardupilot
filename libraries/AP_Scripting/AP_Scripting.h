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

#ifdef ENABLE_SCRIPTING

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class AP_Scripting
{
public:
    AP_Scripting();

    /* Do not allow copies */
    AP_Scripting(const AP_Scripting &other) = delete;
    AP_Scripting &operator=(const AP_Scripting&) = delete;

    bool init(void);

    static AP_Scripting * get_singleton(void) { return _singleton; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    void load_script(const char *filename); // load a script from a file

    void thread(void); // main script execution thread

    AP_Int8 _enable;
    AP_Int32 _script_vm_exec_count;
    AP_Int32 _script_heap_size;
    AP_Int8 _debug_level;

    static AP_Scripting *_singleton;

};

namespace AP {
    AP_Scripting * scripting(void);
};

#endif // ENABLE_SCRIPTING
