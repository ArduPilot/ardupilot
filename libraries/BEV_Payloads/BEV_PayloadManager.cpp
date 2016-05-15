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
 
#include "BEV_PayloadManager.h"
 
extern const AP_HAL::HAL& hal;
 
BEV_PayloadManager::BEV_PayloadManager() :
    cameraTrigger(_payloadCommunicator, PAYLOAD_ID_CAMERATRIGGER),
    gimbal(_payloadCommunicator, PAYLOAD_ID_GIMBAL),
    _payloadCommunicator()
{
    ;
}

void BEV_PayloadManager::init(void)
{
    _payloadCommunicator.init();

    //initialize each device.
    cameraTrigger.init();
    gimbal.init();

    //ask for attached payloads. This message should be the first thing through regardless but it never hurts to ask
    find_payloads();
}

void BEV_PayloadManager::update()
{
    _payloadCommunicator.update();
    gimbal.update();

    if(_payloadCommunicator.has_new_message_from_px4io()) {
        process_message(_payloadCommunicator.get_new_message_from_px4io());
    }
}

void BEV_PayloadManager::process_message(bev_payload_struct s)
{
    //figure out which device to send it to
    switch(s.payload_id) {
    case PAYLOAD_ID_MANAGER:
        process_my_message(s);
        break;
    case PAYLOAD_ID_CAMERATRIGGER:
        cameraTrigger.process_message(s);
        break;
    case PAYLOAD_ID_GIMBAL:
        gimbal.process_message(s);
        break;
    default :
        break;
    }
}

void BEV_PayloadManager::process_my_message(bev_payload_struct s)
{
    switch(s.message_id) {
    case MSG_ID_PAYLOADS_ATTACHED:
        report_payloads(s.d1);
        break;
    default:
        break;
    }
}

void BEV_PayloadManager::find_payloads()
{
    bev_payload_struct s = BEV_PAYLOAD_STRUCT_DEFAULT;
    s.payload_id = PAYLOAD_ID_MANAGER;
    s.message_id = MSG_ID_PAYLOADS_ATTACHED;
    _payloadCommunicator.push_important_message(s);
}
void BEV_PayloadManager::report_payloads(int32_t bits)
{
    //tell children if they're connected or not
    cameraTrigger.attached(bits & DEVICE_BITMASK_CAMERATRIGGGER);
    gimbal.attached(bits & DEVICE_BITMASK_GIMBAL);
}

void BEV_PayloadManager::send_bwtest_message()
{
    static int32_t test_value = 0;
    test_value++;

    bev_payload_struct s = BEV_PAYLOAD_STRUCT_DEFAULT;
    s.payload_id = PAYLOAD_ID_MANAGER; //
    s.message_id = 0; //bwtest msgid
    s.d1 = test_value;
    _payloadCommunicator.push_important_message(s);
}
