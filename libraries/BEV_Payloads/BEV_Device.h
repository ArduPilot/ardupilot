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

#ifndef __BEV_DEVICE_H__
#define __BEV_DEVICE_H__

#include "BEV_PayloadCommunicator.h"

class BEV_Device {
    //Methods
public:
    BEV_Device(BEV_PayloadCommunicator &p, int32_t payloadId) :
        _payloadCommunicator(p),
        _payloadId(payloadId),
        _attached(false) {};
    void init() {;}
    void attached(bool a) {_attached = a;}
    bool attached() {return _attached;}
    virtual void update() = 0;
    virtual void process_message(const bev_payload_struct& s) = 0;
protected:
    BEV_PayloadCommunicator& _payloadCommunicator;
    int32_t _payloadId;
    bool _attached;
};

#endif /* __BEV_DEVICE_H__ */
