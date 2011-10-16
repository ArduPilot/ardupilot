/*
 * AP_Autopilot.cpp
 *
 *  Created on: Apr 30, 2011
 *      Author: jgoppert
 */

#include "AP_Autopilot.h"
#include "AP_BatteryMonitor.h"

namespace apo {

class AP_HardwareAbstractionLayer;

AP_Autopilot::AP_Autopilot(AP_Navigator * navigator, AP_Guide * guide,
		AP_Controller * controller, AP_HardwareAbstractionLayer * hal,
		float loopRate, float loop0Rate, float loop1Rate, float loop2Rate, float loop3Rate) :
	Loop(loopRate, callback, this), _navigator(navigator), _guide(guide),
			_controller(controller), _hal(hal),
			callbackCalls(0) {

	hal->setState(MAV_STATE_BOOT);
	hal->gcs->sendMessage(MAVLINK_MSG_ID_HEARTBEAT);
	hal->gcs->sendMessage(MAVLINK_MSG_ID_SYS_STATUS);

	/*
	 * Radio setup
	 */
	hal->debug->println_P(PSTR("initializing radio"));
	APM_RC.Init(); // APM Radio initialization,

	/*
	 * Calibration
	 */
	hal->setState(MAV_STATE_CALIBRATING);
	hal->gcs->sendMessage(MAVLINK_MSG_ID_HEARTBEAT);
	hal->gcs->sendMessage(MAVLINK_MSG_ID_SYS_STATUS);

	if (navigator) navigator->calibrate();

	/*
	 * Look for valid initial state
	 */
	while (_navigator) {
		// letc gcs known we are alive
		hal->gcs->sendMessage(MAVLINK_MSG_ID_HEARTBEAT);
		hal->gcs->sendMessage(MAVLINK_MSG_ID_SYS_STATUS);
		if (hal->getMode() == MODE_LIVE) {
			_navigator->updateSlow(0);
			if (hal->gps) {
				if (hal->gps->fix) {
					break;
				} else {
					hal->gps->update();
					hal->gcs->sendText(SEVERITY_LOW,
							PSTR("waiting for gps lock\n"));
					hal->debug->printf_P(PSTR("waiting for gps lock\n"));
				}
			} else { // no gps, can skip
				break;
			}
		} else if (hal->getMode() == MODE_HIL_CNTL) { // hil
			hal->hil->sendMessage(MAVLINK_MSG_ID_HEARTBEAT);
			hal->hil->receive();
			Serial.println("HIL Receive Called");
			if (_navigator->getTimeStamp() != 0) {
				// give hil a chance to send some packets
				for (int i = 0; i < 5; i++) {
					hal->debug->println_P(PSTR("reading initial hil packets"));
					hal->gcs->sendText(SEVERITY_LOW,
							PSTR("reading initial hil packets"));
					delay(1000);
				}
				break;
			}
			hal->debug->println_P(PSTR("waiting for hil packet"));
		}
		delay(500);
	}
	
	AP_MavlinkCommand::home.setAlt(_navigator->getAlt());
	AP_MavlinkCommand::home.setLat(_navigator->getLat());
	AP_MavlinkCommand::home.setLon(_navigator->getLon());
	AP_MavlinkCommand::home.setCommand(MAV_CMD_NAV_WAYPOINT);
	AP_MavlinkCommand::home.save();
	_hal->debug->printf_P(PSTR("\nhome before load lat: %f deg, lon: %f deg, cmd: %d\n"),
			AP_MavlinkCommand::home.getLat()*rad2Deg,
			AP_MavlinkCommand::home.getLon()*rad2Deg,
			AP_MavlinkCommand::home.getCommand());
	AP_MavlinkCommand::home.load();
	_hal->debug->printf_P(PSTR("\nhome after load lat: %f deg, lon: %f deg, cmd: %d\n"),
			AP_MavlinkCommand::home.getLat()*rad2Deg,
			AP_MavlinkCommand::home.getLon()*rad2Deg,
			AP_MavlinkCommand::home.getCommand());

	/*
	 * Attach loops
	 */
	hal->debug->println_P(PSTR("attaching loops"));
	subLoops().push_back(new Loop(loop0Rate, callback0, this));
	subLoops().push_back(new Loop(loop1Rate, callback1, this));
	subLoops().push_back(new Loop(loop2Rate, callback2, this));
	subLoops().push_back(new Loop(loop3Rate, callback3, this));

	hal->debug->println_P(PSTR("running"));
	hal->gcs->sendText(SEVERITY_LOW, PSTR("running"));
	hal->setState(MAV_STATE_STANDBY);
}

void AP_Autopilot::callback(void * data) {
	AP_Autopilot * apo = (AP_Autopilot *) data;
	//apo->hal()->debug->println_P(PSTR("callback"));

	/*
	 * ahrs update
	 */
	apo->callbackCalls++;
	if (apo->getNavigator())
		apo->getNavigator()->updateFast(apo->dt());
}

void AP_Autopilot::callback0(void * data) {
	AP_Autopilot * apo = (AP_Autopilot *) data;
	//apo->getHal()->debug->println_P(PSTR("callback 0"));

	/*
	 * hardware in the loop
	 */
	if (apo->getHal()->hil && apo->getHal()->getMode() != MODE_LIVE) {
		apo->getHal()->hil->receive();
		apo->getHal()->hil->sendMessage(MAVLINK_MSG_ID_RC_CHANNELS_SCALED);
	}

    /*
	 * update control laws
	 */
	if (apo->getController()) {
		//apo->getHal()->debug->println_P(PSTR("updating controller"));
		apo->getController()->update(apo->subLoops()[0]->dt());
	}
	/*
	 char msg[50];
	 sprintf(msg, "c_hdg: %f, c_thr: %f", apo->guide()->headingCommand, apo->guide()->groundSpeedCommand);
	 apo->hal()->gcs->sendText(AP_CommLink::SEVERITY_LOW, msg);
	 */
}

void AP_Autopilot::callback1(void * data) {
	AP_Autopilot * apo = (AP_Autopilot *) data;
	//apo->getHal()->debug->println_P(PSTR("callback 1"));
	
	/*
	 * update guidance laws
	 */
	if (apo->getGuide())
	{
		//apo->getHal()->debug->println_P(PSTR("updating guide"));
		apo->getGuide()->update();
	}

	/*
	 * slow navigation loop update
	 */
	if (apo->getNavigator()) {
		apo->getNavigator()->updateSlow(apo->subLoops()[1]->dt());
	}

	/*
	 * send telemetry
	 */
	if (apo->getHal()->gcs) {
		apo->getHal()->gcs->sendMessage(MAVLINK_MSG_ID_ATTITUDE);
	}

	/*
	 * handle ground control station communication
	 */
	if (apo->getHal()->gcs) {
		// send messages
		apo->getHal()->gcs->requestCmds();
		apo->getHal()->gcs->sendParameters();

		// receive messages
		apo->getHal()->gcs->receive();
	}

	/*
	 * navigator debug
	 */
	/*
	 if (apo->navigator()) {
	 apo->getHal()->debug->printf_P(PSTR("roll: %f deg\tpitch: %f deg\tyaw: %f deg\n"),
	 apo->navigator()->getRoll()*rad2Deg,
	 apo->navigator()->getPitch()*rad2Deg,
	 apo->navigator()->getYaw()*rad2Deg);
	 apo->getHal()->debug->printf_P(PSTR("lat: %f deg\tlon: %f deg\talt: %f m\n"),
	 apo->navigator()->getLat()*rad2Deg,
	 apo->navigator()->getLon()*rad2Deg,
	 apo->navigator()->getAlt());
	 }
	 */
}

void AP_Autopilot::callback2(void * data) {
	AP_Autopilot * apo = (AP_Autopilot *) data;
	//apo->getHal()->debug->println_P(PSTR("callback 2"));
	
	/*
	 * send telemetry
	 */
	if (apo->getHal()->gcs) {
		// send messages
		apo->getHal()->gcs->sendMessage(MAVLINK_MSG_ID_GPS_RAW);
		//apo->getHal()->gcs->sendMessage(MAVLINK_MSG_ID_RC_CHANNELS_SCALED);
		//apo->getHal()->gcs->sendMessage(MAVLINK_MSG_ID_GLOBAL_POSITION);
		//apo->getHal()->gcs->sendMessage(MAVLINK_MSG_ID_RC_CHANNELS_RAW);
		//apo->getHal()->gcs->sendMessage(MAVLINK_MSG_ID_SCALED_IMU);
	}

	/*
	 * update battery monitor
	 */
	if (apo->getHal()->batteryMonitor) apo->getHal()->batteryMonitor->update();

	/*
	 * send heartbeat
	 */
	apo->getHal()->gcs->sendMessage(MAVLINK_MSG_ID_HEARTBEAT);

	/*
	 * load/loop rate/ram debug
	 */
	apo->getHal()->load = apo->load();
	apo->getHal()->debug->printf_P(PSTR("callback calls: %d\n"),apo->callbackCalls);
	apo->callbackCalls = 0;
	apo->getHal()->debug->printf_P(PSTR("load: %d%%\trate: %f Hz\tfree ram: %d bytes\n"),
			apo->load(),1.0/apo->dt(),freeMemory());
	apo->getHal()->gcs->sendMessage(MAVLINK_MSG_ID_SYS_STATUS);

	/*
	 * adc debug
	 */
	//apo->getDebug().printf_P(PSTR("adc: %d %d %d %d %d %d %d %d\n"),
	//apo->adc()->Ch(0), apo->adc()->Ch(1), apo->adc()->Ch(2),
	//apo->adc()->Ch(3), apo->adc()->Ch(4), apo->adc()->Ch(5),
	//apo->adc()->Ch(6), apo->adc()->Ch(7), apo->adc()->Ch(8));
}

void AP_Autopilot::callback3(void * data) {
	//AP_Autopilot * apo = (AP_Autopilot *) data;
	//apo->getHal()->debug->println_P(PSTR("callback 3"));
}

} // apo
