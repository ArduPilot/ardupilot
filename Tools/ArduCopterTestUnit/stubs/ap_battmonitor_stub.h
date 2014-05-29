/*
 * ap_battmmonitor_stub.h
 *
 *  Created on: 28 mai 2014
 *      Author: valentin
 */

#ifndef AP_BATTMONITOR_STUB_H_
#define AP_BATTMONITOR_STUB_H_

#define AP_BATT_MONITOR_DISABLED            0
#define AP_BATT_MONITOR_VOLTAGE_ONLY        3
#define AP_BATT_MONITOR_VOLTAGE_AND_CURRENT 4

class AP_BattMonitor
{
public:
	int8_t monitoring() {return AP_BATT_MONITOR_DISABLED;}
	uint8_t capacity_remaining_pct () {return 50;};
	float current_amps () {return 1.56;};
	float voltage () {return 11.56;};
};//AP_BattMonitor

#endif /* AP_BATTMONITOR_STUB_H_ */
