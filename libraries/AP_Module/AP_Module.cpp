
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 *  Frontend Callback broker for APM
 *  Author: Siddharth B Purohit
 *
 */
 
#include <AP_Module/AP_Module.h>

int32_t AP_Module::add_callback(AP_HAL::MemberProc cfunc, uint8_t e)
{
    Callback *cb;
    cb = new Callback;
    // Check if allocation was successful
    if(cb == NULL) {
      return -1;
    }
    // Initialise callback node
    cb->init(cfunc,e,last_callback_id);
    calllist.push_back(cb);
    return last_callback_id++;
}

void AP_Module::modify_callback_trigger(uint32_t callback_id, uint8_t new_e)
{
    Callback *cb;
    // Get to the tip of the list
    cb = calllist.get_head();
    while(cb != NULL) {
        if(cb->is_callback_id(callback_id)) {
            cb->set_event(new_e);
            break;
        }
        // scroll through callback lisr
        cb = cb->get_prev();
    }
}

void AP_Module::invoke_callbacks(uint64_t &event_bitmask)
{
    Callback *cb;
    for(uint8_t i=0; i<MAX_EVENTS; i++) {
        if(event_bitmask & 1<<i) {
            // Get to the tip of the list
            cb = calllist.get_head();
            while(cb != NULL) {
                if(cb->is_event(i)) {
                    cb->callfunc();
                    cb->callperf();
                }
                // scroll through callback lisr
                cb = cb->get_prev();
            }
        }
    }
    event_bitmask = 0;
}