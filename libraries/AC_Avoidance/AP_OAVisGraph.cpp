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

#include "AC_Avoidance_config.h"

#if AP_OAPATHPLANNER_ENABLED

#include "AP_OAVisGraph.h"

// constructor initialises expanding array to use 20 elements per chunk
AP_OAVisGraph::AP_OAVisGraph() :
    _items(20)
{
}

// add item to visiblity graph, returns true on success, false if graph is full
bool AP_OAVisGraph::add_item(const OAItemID &id1, const OAItemID &id2, float distance_cm)
{
    // no more than 65k items
    if (_num_items == UINT16_MAX) {
        return false;
    }

    // ensure there is space in the array
    if (!_items.expand_to_hold(_num_items+1)) {
        return false;
    }

    // add item
    _items[_num_items] = {id1, id2, distance_cm};
    _num_items++;
    return true;
}

#endif  // AP_OAPATHPLANNER_ENABLED
