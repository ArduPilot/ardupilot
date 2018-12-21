/*
 * TMRS_Motor_Power_Monitor.h
 *
 */

#ifndef TMRS_MOTOR_POWER_MONITOR_H_
#define TMRS_MOTOR_POWER_MONITOR_H_
#include "AP_ADC121C021_Device.h"

class TMRS_Motor_Power_Monitor
{
public:
    TMRS_Motor_Power_Monitor();
    virtual ~TMRS_Motor_Power_Monitor();
	void get_scaled_currents(float currents[]);
	void init();
	void send_mavlink_tmrs_motor_status(mavlink_channel_t chan);

private:
    AP_ADC121C021_Device* sensors[6];
    float voltage_reference;
    float current_reference;
    bool sensor_loaded[6];
    int bus_id;

	void _init_sensors();


};

#endif /* TMRS_MOTOR_POWER_MONITOR_H_ */
