/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "BEV_SQueue.h"

BEV_SQueue::BEV_SQueue(uint8_t size):
    _size(size+1), //necessary for some reason
    _values(new bev_payload_struct[_size]),
    _front(0),
    _back(0)
{ 
	;
}
 
bool BEV_SQueue::isFull()
{
    if((_back + 1) %  _size == _front )
        return 1;
    else
        return 0;
}
 
bool BEV_SQueue::push(struct bev_payload_struct & s)
{
	bool b = 0;
	if(!isFull())
	{
		_values[_back] = s;
		_back = (_back + 1) % _size;
		b = 1;
	}
	return b;
}
 
bool BEV_SQueue::isEmpty()
{
    if( _back  == _front )//is empty
        return 1;
    else
		return 0; //is not empty
}
 
bev_payload_struct BEV_SQueue::pop()
{
    bev_payload_struct val = BEV_PAYLOAD_STRUCT_DEFAULT;
    if(!isEmpty())
    {
        val = _values[_front];
        _front = ( _front + 1 ) % _size;
    }
	
return val;
 
}