/*
 * AP_Autopilot.cpp
 *
 *  Created on: Apr 30, 2011
 *      Author: jgoppert
 */

#include "AP_Autopilot.h"

namespace apo {

class AP_HardwareAbstractionLayer;

AP_Autopilot::AP_Autopilot(AP_Navigator * navigator, AP_Guide * guide, AP_Controller * controller,
		AP_HardwareAbstractionLayer * hal) :
	Loop(loop0Rate, callback0, this),
			_navigator(navigator), _guide(guide), _controller(controller), _hal(hal) {

	/*
	 * Calibration
	 */
	navigator->calibrate();

	// start clock
	uint32_t timeStart = millis();
	uint16_t gpsWaitTime = 5000; // 5 second wait for gps

	/*
	 * Look for valid initial state
	 */
	while (1) {
		// letc gcs known we are alive
		hal->gcs->sendMessage(MAVLINK_MSG_ID_HEARTBEAT);
		hal->hil->sendMessage(MAVLINK_MSG_ID_HEARTBEAT);
		delay(1000);
		if (hal->mode() == MODE_LIVE) {
			_navigator->updateSlow(0);
			if (_hal->gps) {
				if (hal->gps->fix) {
					break;
				} else {
					hal->gcs->sendText(SEVERITY_LOW,PSTR("waiting for gps lock\n"));
					hal->debug->printf_P(PSTR("waiting for gps lock\n"));
				}
			} else { // no gps, can skip
				break;
			}
		} else if(hal->mode() == MODE_HIL_CNTL){ // hil
			_hal->hil->receive();
			Serial.println("HIL Recieve Called");
			if (_navigator->getTimeStamp() != 0) {
				// give hil a chance to send some packets
				for (int i=0;i<5;i++) {
					hal->debug->println_P(PSTR("reading initial hil packets"));
					hal->gcs->sendText(SEVERITY_LOW,PSTR("reading initial hil packets"));
					delay(1000);
				}
				break;
			}
			hal->debug->println_P(PSTR("waiting for hil packet"));
		}
	}
	AP_MavlinkCommand home(0);
	home.setAlt(_navigator->getAlt());
	home.setLat(_navigator->getLat());
	home.setLon(_navigator->getLon());
	home.save();
	_hal->debug->printf_P(PSTR("home before load lat: %f deg, lon: %f deg\n"), home.getLat()*rad2Deg,home.getLon()*rad2Deg);
	home.load();
	_hal->debug->printf_P(PSTR("home after load lat: %f deg, lon: %f deg\n"), home.getLat()*rad2Deg,home.getLon()*rad2Deg);

	/*
	 * Attach loops
	 */
	hal->debug->println_P(PSTR("attaching loops"));
	subLoops().push_back(new Loop(loop1Rate, callback1, this));
	subLoops().push_back(new Loop(loop2Rate, callback2, this));
	subLoops().push_back(new Loop(loop3Rate, callback3, this));
	subLoops().push_back(new Loop(loop4Rate, callback4, this));

	hal->debug->println_P(PSTR("running"));
	hal->gcs->sendText(SEVERITY_LOW,PSTR("running"));
}

void AP_Autopilot::callback0(void * data) {
	AP_Autopilot * apo = (AP_Autopilot *) data;
	//apo->hal()->debug->println_P(PSTR("callback 0"));

	/*
	 * ahrs update
	 */
	if (apo->navigator())
		apo->navigator()->updateFast(1.0/loop0Rate);
}

void AP_Autopilot::callback1(void * data) {
	AP_Autopilot * apo = (AP_Autopilot *) data;
	//apo->hal()->debug->println_P(PSTR("callback 1"));

	/*
	 * hardware in the loop
	 */
	if (apo->hal()->hil && apo->hal()->mode()!=MODE_LIVE)
	{
		apo->hal()->hil->receive();
		apo->hal()->hil->sendMessage(MAVLINK_MSG_ID_RC_CHANNELS_SCALED);
	}

	/*
     * update control laws
	 */
	if (apo->guide())
		apo->guide()->update();

	/*
	 * update control laws
	 */
	if (apo->controller())
	{
		//apo->hal()->debug->println_P(PSTR("updating controller"));
		apo->controller()->update(1./loop1Rate);
	}
	/*
	char msg[50];
	sprintf(msg, "c_hdg: %f, c_thr: %f", apo->guide()->headingCommand, apo->guide()->groundSpeedCommand);
	apo->hal()->gcs->sendText(AP_CommLink::SEVERITY_LOW, msg);
	*/
}

void AP_Autopilot::callback2(void * data) {
	AP_Autopilot * apo = (AP_Autopilot *) data;
	//apo->hal()->debug->println_P(PSTR("callback 2"));

	/*
	 * send telemetry
	 */
	if (apo->hal()->gcs) {
		// send messages
		apo->hal()->gcs->sendMessage(MAVLINK_MSG_ID_GPS_RAW);
		apo->hal()->gcs->sendMessage(MAVLINK_MSG_ID_ATTITUDE);
		//apo->hal()->gcs->sendMessage(MAVLINK_MSG_ID_RC_CHANNELS_SCALED);
		//apo->hal()->gcs->sendMessage(MAVLINK_MSG_ID_GLOBAL_POSITION);
		apo->hal()->gcs->sendMessage(MAVLINK_MSG_ID_RC_CHANNELS_RAW);
		//apo->hal()->gcs->sendMessage(MAVLINK_MSG_ID_SCALED_IMU);
	}

	/*
	 * slow navigation loop update
	 */
	if (apo->navigator()) {
		apo->navigator()->updateSlow(1.0/loop2Rate);
	}

	/*
	 * handle ground control station communication
	 */
	if (apo->hal()->gcs) {
		// send messages
		apo->hal()->gcs->requestCmds();
		apo->hal()->gcs->sendParameters();

		// receive messages
		apo->hal()->gcs->receive();
	}

	/*
	 * navigator debug
	 */
	/*
	 if (apo->navigator()) {
		 apo->hal()->debug->printf_P(PSTR("roll: %f deg\tpitch: %f deg\tyaw: %f deg\n"),
				 apo->navigator()->getRoll()*rad2Deg,
				 apo->navigator()->getPitch()*rad2Deg,
				 apo->navigator()->getYaw()*rad2Deg);
		 apo->hal()->debug->printf_P(PSTR("lat: %f deg\tlon: %f deg\talt: %f m\n"),
				 apo->navigator()->getLat()*rad2Deg,
				 apo->navigator()->getLon()*rad2Deg,
				 apo->navigator()->getAlt());
	 }
	 */
}

void AP_Autopilot::callback3(void * data) {
	AP_Autopilot * apo = (AP_Autopilot *) data;
	//apo->hal()->debug->println_P(PSTR("callback 3"));

	/*
	 * send heartbeat
	 */
	apo->hal()->gcs->sendMessage(MAVLINK_MSG_ID_HEARTBEAT);

	/*
	 * load/loop rate/ram debug
	 */

	apo->hal()->load = apo->load();
	apo->hal()->debug->printf_P(PSTR("load: %d%%\trate: %f Hz\tfree ram: %d bytes\n"),
			apo->load(),1.0/apo->dt(),freeMemory());

	apo->hal()->gcs->sendMessage(MAVLINK_MSG_ID_SYS_STATUS);

	/*
	 * adc debug
	 */
	//apo->getDebug().printf_P(PSTR("adc: %d %d %d %d %d %d %d %d\n"),
	//apo->adc()->Ch(0), apo->adc()->Ch(1), apo->adc()->Ch(2),
	//apo->adc()->Ch(3), apo->adc()->Ch(4), apo->adc()->Ch(5),
	//apo->adc()->Ch(6), apo->adc()->Ch(7), apo->adc()->Ch(8));
}

void AP_Autopilot::callback4(void * data) {
	AP_Autopilot * apo = (AP_Autopilot *) data;
	//apo->hal()->debug->println_P(PSTR("callback 4"));
}

} // apo
