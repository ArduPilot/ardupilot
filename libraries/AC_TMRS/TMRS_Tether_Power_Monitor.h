/*
 * TMRS_Tether_Power_Monitor.h
 *
 *  Created on: Sep 16, 2018
 *      Author: Rob
 */

#ifndef TMRS_TETHER_POWER_MONITOR_H_
#define TMRS_TETHER_POWER_MONITOR_H_

#include "AP_ADC121C021_Device.h"

class TMRS_Tether_Power_Monitor
{
public:
    TMRS_Tether_Power_Monitor();
    virtual ~TMRS_Tether_Power_Monitor();
    void init();
	float getHighVoltageVoltage();
	float getLowVoltageVoltage();
	float getLowVoltageCurrent();
	void send_mavlink_tether_power_status(mavlink_channel_t chan);

private:
    int bus_id;
    float low_voltage_reference;
    float high_voltage_reference;
    float low_voltage_current_reference;
    AP_ADC121C021_Device* lowVoltageVoltageMonitor;
    AP_ADC121C021_Device* lowVoltageCurrentMonitor;
    AP_ADC121C021_Device* highVoltageVoltageMonitor;
    bool low_voltage_voltage_monitor_load_status;
    bool low_voltage_current_monitor_load_status;
    bool high_voltage_voltage_monitor_load_status;
};

#endif /* TMRS_TETHER_POWER_MONITOR_H_ */
