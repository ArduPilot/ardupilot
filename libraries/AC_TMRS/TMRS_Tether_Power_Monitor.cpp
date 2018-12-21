/*
 * TMRS_Tether_Power_Monitor.cpp
 *
 *  Created on: Sep 16, 2018
 *      Author: Rob
 */

#include "TMRS_Tether_Power_Monitor.h"

TMRS_Tether_Power_Monitor::TMRS_Tether_Power_Monitor()
{
    /**
     * Will add these as configuration parameters eventually
     */
    this->high_voltage_reference = 525.0;
    this->low_voltage_reference = 30.0;
    this->low_voltage_current_reference = 100.0;
    this->bus_id = 0;
    this->high_voltage_voltage_monitor_load_status = false;
    this->low_voltage_current_monitor_load_status = false;
    this->low_voltage_voltage_monitor_load_status = false;

    this->lowVoltageVoltageMonitor = new AP_ADC121C021_Device(bus_id,0x51,this->low_voltage_reference);
    this->lowVoltageCurrentMonitor = new AP_ADC121C021_Device(bus_id,0x50,this->low_voltage_current_reference,false);
    this->highVoltageVoltageMonitor = new AP_ADC121C021_Device(bus_id,0x5A,this->high_voltage_reference);
}

TMRS_Tether_Power_Monitor::~TMRS_Tether_Power_Monitor()
{
    delete this->lowVoltageVoltageMonitor;
    delete this->lowVoltageCurrentMonitor;
    delete this->highVoltageVoltageMonitor;
}

void TMRS_Tether_Power_Monitor::init()
{
    this->low_voltage_voltage_monitor_load_status = this->lowVoltageVoltageMonitor->init();
    this->low_voltage_current_monitor_load_status = this->lowVoltageCurrentMonitor->init();
    this->high_voltage_voltage_monitor_load_status = this->highVoltageVoltageMonitor->init();
}

float TMRS_Tether_Power_Monitor::getHighVoltageVoltage()
{
    return this->highVoltageVoltageMonitor->get_scaled_value();
}

float TMRS_Tether_Power_Monitor::getLowVoltageVoltage()
{
    return this->lowVoltageVoltageMonitor->get_scaled_value();
}

float TMRS_Tether_Power_Monitor::getLowVoltageCurrent()
{
    return this->lowVoltageCurrentMonitor->get_scaled_value();
}

void TMRS_Tether_Power_Monitor::send_mavlink_tether_power_status( mavlink_channel_t chan)
{
    float high_voltage_voltage = this->highVoltageVoltageMonitor->get_scaled_value();
    float low_voltage_voltage = this->lowVoltageVoltageMonitor->get_scaled_value();
    float low_voltage_current = this->lowVoltageCurrentMonitor->get_scaled_value();
    mavlink_msg_tmrs_tether_status_send(chan,high_voltage_voltage,low_voltage_voltage,low_voltage_current);
}
