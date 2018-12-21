/*
 * TMRS_Payload_Controller.cpp
 *
 *  Created on: Sep 16, 2018
 *      Author: Rob
 */

#include "TMRS_Payload_Controller.h"

TMRS_Payload_Controller::TMRS_Payload_Controller()
{

}

void TMRS_Payload_Controller::init()
{
    this->payloadControlDevice = new AP_TCA9534A_Device();

    this->payloadControlDevice->init();
}

TMRS_Payload_Controller::~TMRS_Payload_Controller()
{
    // TODO Auto-generated destructor stub
}

void TMRS_Payload_Controller::send_mavlink_tmrs_payload_status(mavlink_channel_t chan)
{
    uint8_t control_bitmask = this->payloadControlDevice->get_payload();
    mavlink_msg_tmrs_payload_control_send(chan, control_bitmask);
}

void TMRS_Payload_Controller::set_payload_settings(uint8_t control)
{
    this->payloadControlDevice->set_payload(control);
}

/**
 * 1.   I/O Chip
1.1.    TCA9534A
1.2.    Address 0111000
1.3.    8 bits
1.4.    Enable +5V for sightline video processor, 500mA
1.4.1.  Bit 0
1.4.2.  Output
1.4.3.  1 = ON
1.4.4.  0 = OFF
1.5.    Enable +12VDC Power for External Payload, 4A
1.5.1.  Bit 1
1.5.2.  Output
1.5.3.  1 = ON
1.5.4.  0 = OFF
1.6.    Enable +5V for Spare power connector, 1.5A
1.6.1.  Bit 2
1.6.2.  Output
1.6.3.  1 = ON
1.6.4.  0 = OFF
1.7.    Enable +12V for internal Payload, 4A
1.7.1.  Bit 3
1.7.2.  Output
1.7.3.  1 = ON
1.7.4.  0 = OFF
1.8.    Not used
1.8.1.  Bit 4
1.8.2.  Output
1.8.3.  1 = N/A
1.8.4.  0 = N/A
1.9.    Enable +5V for LNA, 200mA
1.9.1.  Bit 5
1.9.2.  Output
1.9.3.  1 = ON
1.9.4.  0 = OFF
1.10.   RF switch position control
1.10.1. Bit 6
1.10.2. Output
1.10.3. 1 = TCA9534A P7
1.10.4. 0 = Power board comparator output (115VDC falling)
1.11.   RF switch position
1.11.1. Bit 7
1.11.2. Output
1.11.3. 1 = RF to tether
1.11.4. 0 = RF to external antenna
 */
