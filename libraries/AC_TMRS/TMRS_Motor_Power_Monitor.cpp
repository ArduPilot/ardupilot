/*
 * TMRSMotorPowerMonitor.cpp
 *
 *  Created on: Sep 16, 2018
 *      Author: Rob
 */

#include "TMRS_Motor_Power_Monitor.h"

TMRS_Motor_Power_Monitor::TMRS_Motor_Power_Monitor()
{
    voltage_reference=30;
    current_reference=20;
    bus_id = 0;
}

TMRS_Motor_Power_Monitor::~TMRS_Motor_Power_Monitor()
{
}

void TMRS_Motor_Power_Monitor::init() {
    _init_sensors();
}


void TMRS_Motor_Power_Monitor::_init_sensors() {
    sensors[0] = new AP_ADC121C021_Device(bus_id,0x52,this->current_reference,false);
    sensors[1] = new AP_ADC121C021_Device(bus_id,0x54,this->current_reference,false);
    sensors[2] = new AP_ADC121C021_Device(bus_id,0x55,this->current_reference,false);
    sensors[3] = new AP_ADC121C021_Device(bus_id,0x56,this->current_reference,false);
    sensors[4] = new AP_ADC121C021_Device(bus_id,0x58,this->current_reference,false);
    sensors[5] = new AP_ADC121C021_Device(bus_id,0x59,this->current_reference,false);
    for(int i = 0; i < 6; i++) {
        sensor_loaded[i] = sensors[i]->init();
    }
}

void TMRS_Motor_Power_Monitor::get_scaled_currents(float currents[]) {
//    for(int i = 0; i < 6; i++) {
//       currents[i] = this->sensors[i]->get_scaled_value();
//    }
}

void TMRS_Motor_Power_Monitor::send_mavlink_tmrs_motor_status( mavlink_channel_t chan)
{
    float currents[6];
    for(int i = 0; i < 6; i++) {
       currents[i] = this->sensors[i]->get_scaled_value();
    }
//    get_scaled_currents(currents);
    mavlink_msg_tmrs_motor_status_send(chan,currents);
}


/**
 * 2.   26 VDC POWER SUPPLY VOLTAGE MONITOR
2.1.    ADC121C021
2.2.    Address 1010000 // 0x50
2.3.    12bits
2.4.    V = 30VDC/4096 = 7.3mV/bit

3.  26 VDC POWER SUPPLY CURRENT MONITOR
3.1.    ADC121C021
3.2.    Address 1010001 //  0x51
3.3.    12bits
3.4.    I = 100A/4096 = 4.8mA/bit

4.  MOTOR 1 CURRENT MONITOR
4.1.    ADC121C021
4.2.    Address 1010010 //  0x52
4.3.    12bits
4.4.    I = 20A/4096 = 24.4mA/bit

5.  MOTOR 2 CURRENT MONITOR
5.1.    ADC121C021
5.2.    Address 1010100 // 0x54
5.3.    12bits
5.4.    I = 20A/4096 = 24.4mA/bit

6.  MOTOR 3 CURRENT MONITOR
6.1.    ADC121C021
6.2.    Address 1010101 // 0x55
6.3.    12bits
6.4.    I = 20A/4096 = 24.4mA/bit

7.  MOTOR 4 CURRENT MONITOR
7.1.    ADC121C021
7.2.    Address 1010110 // 0x56
7.3.    12bits
7.4.    I = 20A/4096 = 24.4mA/bit

8.  MOTOR 5 CURRENT MONITOR
8.1.    ADC121C021
8.2.    Address 1011000 // 0x58
8.3.    12bits
8.4.    I = 20A/4096 = 24.4mA/bit

9.  MOTOR 6 CURRENT MONITOR
9.1.    ADC121C021
9.2.    Address 1011001 0x59
9.3.    12bits
9.4.    I = 20A/4096 = 24.4mA/bit

10. 400 VDC POWER SUPPLY VOLTAGE MONITOR
10.1.   ADC121C021
10.2.   Address 1011010 // 0x5A
10.3.   12bits
10.4.   V = 525VDC/4096 = 128mV/bit
 *
 */
