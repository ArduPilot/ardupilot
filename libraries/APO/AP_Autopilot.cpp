/*
 * AP_Autopilot.cpp
 *
 *  Created on: Apr 30, 2011
 *      Author: jgoppert
 */

#include "../FastSerial/FastSerial.h"
#include "AP_Autopilot.h"
#include "../AP_GPS/AP_GPS.h"
#include "../APM_RC/APM_RC.h"
#include "AP_Board.h"
#include "AP_CommLink.h"
#include "AP_MavlinkCommand.h"
#include "AP_Navigator.h"
#include "AP_Controller.h"
#include "AP_Guide.h"
#include "AP_BatteryMonitor.h"

namespace apo {

class AP_Board;

AP_Autopilot::AP_Autopilot(AP_Navigator * navigator, AP_Guide * guide,
                           AP_Controller * controller, AP_Board * board,
                           float loopRate, float loop0Rate, float loop1Rate, float loop2Rate, float loop3Rate) :
    Loop(loopRate, callback, this), _navigator(navigator), _guide(guide),
    _controller(controller), _board(board),
    callbackCalls(0) {

    board->debug->printf_P(PSTR("initializing autopilot\n"));
    board->debug->printf_P(PSTR("free ram: %d bytes\n"),freeMemory());

    /*
     * Comm links
     */
    board->gcs = new MavlinkComm(board->gcsPort, navigator, guide, controller, board, 3);
    if (board->getMode() != AP_Board::MODE_LIVE) {
        board->hil = new MavlinkComm(board->hilPort, navigator, guide, controller, board, 3);
    }
    board->gcsPort->printf_P(PSTR("gcs hello\n"));

    board->gcs->sendMessage(MAVLINK_MSG_ID_HEARTBEAT);
    board->gcs->sendMessage(MAVLINK_MSG_ID_SYS_STATUS);

    /*
     * Calibration
     */
    controller->setState(MAV_STATE_CALIBRATING);
    board->gcs->sendMessage(MAVLINK_MSG_ID_HEARTBEAT);
    board->gcs->sendMessage(MAVLINK_MSG_ID_SYS_STATUS);

    if (navigator) navigator->calibrate();

    /*
     * Look for valid initial state
     */
    while (_navigator) {

        // letc gcs known we are alive
        board->gcs->sendMessage(MAVLINK_MSG_ID_HEARTBEAT);
        board->gcs->sendMessage(MAVLINK_MSG_ID_SYS_STATUS);
        if (board->getMode() == AP_Board::MODE_LIVE) {
            _navigator->updateSlow(0);
            if (board->gps) {
                if (board->gps->fix) {
                    break;
                } else {
                    board->gps->update();
                    board->gcs->sendText(SEVERITY_LOW,
                                       PSTR("waiting for gps lock\n"));
                    board->debug->printf_P(PSTR("waiting for gps lock\n"));
                }
            } else { // no gps, can skip
                break;
            }
        } else if (board->getMode() == AP_Board::MODE_HIL_CNTL) { // hil
            board->hil->sendMessage(MAVLINK_MSG_ID_HEARTBEAT);
            board->hil->receive();
            if (_navigator->getTimeStamp() != 0) {
                // give hil a chance to send some packets
                for (int i = 0; i < 5; i++) {
                    board->debug->println_P(PSTR("reading initial hil packets"));
                    board->gcs->sendText(SEVERITY_LOW, PSTR("reading initial hil packets"));
                    delay(1000);
                }
                break;
            }
            board->debug->println_P(PSTR("waiting for hil packet"));
            board->gcs->sendText(SEVERITY_LOW, PSTR("waiting for hil packets"));
        }
        delay(500);
    }

    AP_MavlinkCommand::home.setAlt(_navigator->getAlt());
    AP_MavlinkCommand::home.setLat(_navigator->getLat());
    AP_MavlinkCommand::home.setLon(_navigator->getLon());
    AP_MavlinkCommand::home.setCommand(MAV_CMD_NAV_WAYPOINT);
    AP_MavlinkCommand::home.save();
    _board->debug->printf_P(PSTR("\nhome before load lat: %f deg, lon: %f deg, cmd: %d\n"),
                          AP_MavlinkCommand::home.getLat()*rad2Deg,
                          AP_MavlinkCommand::home.getLon()*rad2Deg,
                          AP_MavlinkCommand::home.getCommand());
    AP_MavlinkCommand::home.load();
    _board->debug->printf_P(PSTR("\nhome after load lat: %f deg, lon: %f deg, cmd: %d\n"),
                          AP_MavlinkCommand::home.getLat()*rad2Deg,
                          AP_MavlinkCommand::home.getLon()*rad2Deg,
                          AP_MavlinkCommand::home.getCommand());

    guide->setCurrentIndex(0);
    controller->setMode(MAV_MODE_LOCKED);
    controller->setState(MAV_STATE_STANDBY);
  
    /*
     * Attach loops, stacking for priority
     */
    board->debug->println_P(PSTR("attaching loops"));
    subLoops().push_back(new Loop(loop0Rate, callback0, this));
    subLoops().push_back(new Loop(loop1Rate, callback1, this));
    subLoops().push_back(new Loop(loop2Rate, callback2, this));
    subLoops().push_back(new Loop(loop3Rate, callback3, this));

    board->debug->println_P(PSTR("running"));
    board->gcs->sendText(SEVERITY_LOW, PSTR("running"));
}

void AP_Autopilot::callback(void * data) {
    AP_Autopilot * apo = (AP_Autopilot *) data;
    //apo->getBoard()->debug->println_P(PSTR("callback"));

    /*
     * ahrs update
     */
    apo->callbackCalls++;
    if (apo->getNavigator())
        apo->getNavigator()->updateFast(apo->dt());
}

void AP_Autopilot::callback0(void * data) {
    AP_Autopilot * apo = (AP_Autopilot *) data;
    //apo->getBoard()->debug->println_P(PSTR("callback 0"));

    /*
     * hardware in the loop
     */
    if (apo->getBoard()->hil && apo->getBoard()->getMode() != AP_Board::MODE_LIVE) {
        apo->getBoard()->hil->receive();
        apo->getBoard()->hil->sendMessage(MAVLINK_MSG_ID_RC_CHANNELS_SCALED);
    }

    /*
     * update control laws
     */
    if (apo->getController()) {
        //apo->getBoard()->debug->println_P(PSTR("updating controller"));
        apo->getController()->update(apo->subLoops()[0]->dt());
    }
    /*
     char msg[50];
     sprintf(msg, "c_hdg: %f, c_thr: %f", apo->guide()->headingCommand, apo->guide()->groundSpeedCommand);
     apo->board()->gcs->sendText(AP_CommLink::SEVERITY_LOW, msg);
     */
}

void AP_Autopilot::callback1(void * data) {
    AP_Autopilot * apo = (AP_Autopilot *) data;
    //apo->getBoard()->debug->println_P(PSTR("callback 1"));

    /*
     * update guidance laws
     */
    if (apo->getGuide())
    {
        //apo->getBoard()->debug->println_P(PSTR("updating guide"));
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
    if (apo->getBoard()->gcs) {
        apo->getBoard()->gcs->sendMessage(MAVLINK_MSG_ID_ATTITUDE);
        apo->getBoard()->gcs->sendMessage(MAVLINK_MSG_ID_GLOBAL_POSITION);
    }

    /*
     * handle ground control station communication
     */
    if (apo->getBoard()->gcs) {
        // send messages
        apo->getBoard()->gcs->requestCmds();
        apo->getBoard()->gcs->sendParameters();

        // receive messages
        apo->getBoard()->gcs->receive();
    }

    /*
     * navigator debug
     */
    /*
     if (apo->navigator()) {
     apo->getBoard()->debug->printf_P(PSTR("roll: %f deg\tpitch: %f deg\tyaw: %f deg\n"),
     apo->navigator()->getRoll()*rad2Deg,
     apo->navigator()->getPitch()*rad2Deg,
     apo->navigator()->getYaw()*rad2Deg);
     apo->getBoard()->debug->printf_P(PSTR("lat: %f deg\tlon: %f deg\talt: %f m\n"),
     apo->navigator()->getLat()*rad2Deg,
     apo->navigator()->getLon()*rad2Deg,
     apo->navigator()->getAlt());
     }
     */
}

void AP_Autopilot::callback2(void * data) {
    AP_Autopilot * apo = (AP_Autopilot *) data;
    //apo->getBoard()->debug->println_P(PSTR("callback 2"));

    /*
     * send telemetry
     */
    if (apo->getBoard()->gcs) {
        // send messages
        //apo->getBoard()->gcs->sendMessage(MAVLINK_MSG_ID_GPS_RAW_INT);
        //apo->getBoard()->gcs->sendMessage(MAVLINK_MSG_ID_LOCAL_POSITION);
        apo->getBoard()->gcs->sendMessage(MAVLINK_MSG_ID_RC_CHANNELS_SCALED);
        apo->getBoard()->gcs->sendMessage(MAVLINK_MSG_ID_RC_CHANNELS_RAW);
        apo->getBoard()->gcs->sendMessage(MAVLINK_MSG_ID_SCALED_IMU);
    }

    /*
     * update battery monitor
     */
    if (apo->getBoard()->batteryMonitor) apo->getBoard()->batteryMonitor->update();

    /*
     * send heartbeat
     */
    apo->getBoard()->gcs->sendMessage(MAVLINK_MSG_ID_HEARTBEAT);

    /*
     * load/loop rate/ram debug
     */
    apo->getBoard()->load = apo->load();
    apo->getBoard()->debug->printf_P(PSTR("callback calls: %d\n"),apo->callbackCalls);
    apo->callbackCalls = 0;
    apo->getBoard()->debug->printf_P(PSTR("load: %d%%\trate: %f Hz\tfree ram: %d bytes\n"),
                                   apo->load(),1.0/apo->dt(),freeMemory());
    apo->getBoard()->gcs->sendMessage(MAVLINK_MSG_ID_SYS_STATUS);

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
    //apo->getBoard()->debug->println_P(PSTR("callback 3"));
}

} // apo
// vim:ts=4:sw=4:expandtab
