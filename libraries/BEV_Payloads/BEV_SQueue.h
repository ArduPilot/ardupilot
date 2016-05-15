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

#ifndef __BEV_SQUEUE_H__
#define __BEV_SQUEUE_H__

#include <stdint.h>

//the standard payload message
struct bev_payload_struct {
	uint8_t payload_id; //which payload this message is indented for
	uint8_t message_id; //message that is being sent. Message is payload specific
	int32_t d1; //data
	int32_t d2;
	int32_t d3;
	int32_t d4;
};

#define BEV_PAYLOAD_STRUCT_DEFAULT {0,0,0,0,0,0}
	
class BEV_SQueue
{
    public:
        BEV_SQueue(uint8_t size);
        //~BEV_SQueue() {delete [] _values;}
        bool push(struct bev_payload_struct & s);
        bev_payload_struct pop();
        bool isEmpty();
        bool isFull();
    private:
        uint8_t _size;
        bev_payload_struct * _values;
        uint8_t _front;
        uint8_t _back;
	
};

#endif //__BEV_SQUEUE_H__