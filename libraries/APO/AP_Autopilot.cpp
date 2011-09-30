/*
 * AP_Autopilot.cpp
 *
 *  Created on: Apr 30, 2011
 *      Author: jgoppert
 */

#include "AP_Autopilot.h"

namespace apo {

class AP_HardwareAbstractionLayer;

AP_Autopilot::AP_Autopilot(AP_Navigator * navigator, AP_Guide * guide,
		AP_Controller * controller, AP_HardwareAbstractionLayer * hal,
		float loop0Rate, float loop1Rate, float loop2Rate, float loop3Rate) :
	Loop(loop0Rate, callback0, this), _navigator(navigator), _guide(guide),
			_controller(controller), _hal(hal), _loop0Rate(loop0Rate),
			_loop1Rate(loop1Rate), _loop2Rate(loop2Rate), _loop3Rate(loop3Rate),
			_loop4Rate(loop3Rate) {

	hal->setState(MAV_STATE_BOOT);
	hal->gcs->sendMessage(MAVLINK_MSG_ID_HEARTBEAT);
	hal->gcs->sendMessage(MAVLINK_MSG_ID_SYS_STATUS);

	/*
	 * Calibration
	 */
	hal->setState(MAV_STATE_CALIBRATING);
	hal->gcs->sendMessage(MAVLINK_MSG_ID_HEARTBEAT);
	hal->gcs->sendMessage(MAVLINK_MSG_ID_SYS_STATUS);
	navigator->calibrate();

	// start clock
	//uint32_t timeStart = millis();
	//uint16_t gpsWaitTime = 5000; // 5 second wait for gps

	/*
	 * Look for valid initial state
	 */
	while (1) {
		// letc gcs known we are alive
		hal->gcs->sendMessage(MAVLINK_MSG_ID_HEARTBEAT);
		hal->gcs->sendMessage(MAVLINK_MSG_ID_SYS_STATUS);
		hal->hil->sendMessage(MAVLINK_MSG_ID_HEARTBEAT);
		delay(1000);
		if (hal->getMode() == MODE_LIVE) {
			_navigator->updateSlow(0);
			if (_hal->gps) {
				if (hal->gps->fix) {
					break;
				} else {
					hal->gcs->sendText(SEVERITY_LOW,
							PSTR("waiting for gps lock\n"));
					hal->debug->printf_P(PSTR("waiting for gps lock\n"));
				}
			} else { // no gps, can skip
				break;
			}
		} else if (hal->getMode() == MODE_HIL_CNTL) { // hil
			_hal->hil->receive();
			Serial.println("HIL Recieve Called");
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
	}

	AP_MavlinkCommand::home.setAlt(_navigator->getAlt());
	AP_MavlinkCommand::home.setLat(_navigator->getLat());
	AP_MavlinkCommand::home.setLon(_navigator->getLon());
	AP_MavlinkCommand::home.save();
	_hal->debug->printf_P(PSTR("\nhome before load lat: %f deg, lon: %f deg\n"),
			AP_MavlinkCommand::home.getLat()*rad2Deg,
			AP_MavlinkCommand::home.getLon()*rad2Deg);
	AP_MavlinkCommand::home.load();
	_hal->debug->printf_P(PSTR("home after load lat: %f deg, lon: %f deg\n"),
			AP_MavlinkCommand::home.getLat()*rad2Deg,
			AP_MavlinkCommand::home.getLon()*rad2Deg);

	/*
	 * Attach loops
	 */
	hal->debug->println_P(PSTR("attaching loops"));
	subLoops().push_back(new Loop(getLoopRate(1), callback1, this));
	subLoops().push_back(new Loop(getLoopRate(2), callback2, this));
	subLoops().push_back(new Loop(getLoopRate(3), callback3, this));
	subLoops().push_back(new Loop(getLoopRate(4), callback4, this));

	hal->debug->println_P(PSTR("running"));
	hal->gcs->sendText(SEVERITY_LOW, PSTR("running"));

	if (hal->getMode() == MODE_LIVE) {
		hal->setState(MAV_STATE_ACTIVE);
	} else {
		hal->setState(MAV_STATE_HILSIM);
	}

	/*
	 * Radio setup
	 */
	hal->debug->println_P(PSTR("initializing radio"));
	APM_RC.Init(); // APM Radio initialization,
	// start this after control loop is running
}

void AP_Autopilot::callback0(void * data) {
	AP_Autopilot * apo = (AP_Autopilot *) data;
	//apo->hal()->debug->println_P(PSTR("callback 0"));

	/*
	 * ahrs update
	 */
	if (apo->getNavigator())
		apo->getNavigator()->updateFast(1.0 / apo->getLoopRate(0));
}

void AP_Autopilot::callback1(void * data) {
	AP_Autopilot * apo = (AP_Autopilot *) data;
	//apo->getHal()->debug->println_P(PSTR("callback 1"));

	/*
	 * hardware in the loop
	 */
	if (apo->getHal()->hil && apo->getHal()->getMode() != MODE_LIVE) {
		apo->getHal()->hil->receive();
		apo->getHal()->hil->sendMessage(MAVLINK_MSG_ID_RC_CHANNELS_SCALED);
	}

	/*
	 * update guidance laws
	 */
	if (apo->getGuide())
	{
		//apo->getHal()->debug->println_P(PSTR("updating guide"));
		apo->getGuide()->update();
	}

	/*
	 * update control laws
	 */
	if (apo->getController()) {
		//apo->getHal()->debug->println_P(PSTR("updating controller"));
		apo->getController()->update(1. / apo->getLoopRate(1));
	}
	/*
	 char msg[50];
	 sprintf(msg, "c_hdg: %f, c_thr: %f", apo->guide()->headingCommand, apo->guide()->groundSpeedCommand);
	 apo->hal()->gcs->sendText(AP_CommLink::SEVERITY_LOW, msg);
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
		apo->getHal()->gcs->sendMessage(MAVLINK_MSG_ID_ATTITUDE);
		//apo->getHal()->gcs->sendMessage(MAVLINK_MSG_ID_RC_CHANNELS_SCALED);
		//apo->getHal()->gcs->sendMessage(MAVLINK_MSG_ID_GLOBAL_POSITION);
		apo->getHal()->gcs->sendMessage(MAVLINK_MSG_ID_RC_CHANNELS_RAW);
		//apo->getHal()->gcs->sendMessage(MAVLINK_MSG_ID_SCALED_IMU);
	}

	/*
	 * slow navigation loop update
	 */
	if (apo->getNavigator()) {
		apo->getNavigator()->updateSlow(1.0 / apo->getLoopRate(2));
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

void AP_Autopilot::callback3(void * data) {
	AP_Autopilot * apo = (AP_Autopilot *) data;
	//apo->getHal()->debug->println_P(PSTR("callback 3"));

	/*
	 * send heartbeat
	 */
	apo->getHal()->gcs->sendMessage(MAVLINK_MSG_ID_HEARTBEAT);

	/*
	 * load/loop rate/ram debug
	 */
	apo->getHal()->load = apo->load();
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

void AP_Autopilot::callback4(void * data) {
	//AP_Autopilot * apo = (AP_Autopilot *) data;
	//apo->getHal()->debug->println_P(PSTR("callback 4"));
}

} // apo
