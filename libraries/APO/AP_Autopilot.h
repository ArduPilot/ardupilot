/*
 * AP_Autopilot.h
 *
 *  Created on: Apr 30, 2011
 *      Author: jgoppert
 */

#ifndef AP_AUTOPILOT_H_
#define AP_AUTOPILOT_H_

#include "../AP_Common/AP_Loop.h"

/**
 * ArduPilotOne namespace to protect variables
 * from overlap with avr and libraries etc.
 * ArduPilotOne does not use any global
 * variables.
 */

namespace apo {

// enumerations
// forward declarations
class AP_Navigator;
class AP_Guide;
class AP_Controller;
class AP_Board;

/**
 * This class encapsulates the entire autopilot system
 * The constructor takes guide, navigator, and controller
 * as well as the hardware abstraction layer.
 *
 * It inherits from loop to manage
 * the sub-loops and sets the overall
 * frequency for the autopilot.
 *

 */
class AP_Autopilot: public Loop {
public:
    /**
     * Default constructor
     */
    AP_Autopilot(AP_Navigator * navigator, AP_Guide * guide,
                 AP_Controller * controller, AP_Board * board,
                 float loopRate, float loop0Rate, float loop1Rate, float loop2Rate, float loop3Rate);

    /**
     * Accessors
     */
    AP_Navigator * getNavigator() {
        return _navigator;
    }
    AP_Guide * getGuide() {
        return _guide;
    }
    AP_Controller * getController() {
        return _controller;
    }
    AP_Board * getBoard() {
        return _board;
    }

    /**
     * Loop Monitoring
     */
    uint32_t callbackCalls;

private:

    /**
     * Loop Callbacks (fastest)
     * - inertial navigation
     * @param data A void pointer used to pass the apo class
     *  so that the apo public interface may be accessed.
     */
    static void callback(void * data);

    /**
     * Loop 0 Callbacks
     * - control
     * - compass reading
     * @see callback
     */
    static void callback0(void * data);

    /**
     * Loop 1 Callbacks
     * - gps sensor fusion
     * - compass sensor fusion
     * @see callback
     */
    static void callback1(void * data);

    /**
     * Loop 2 Callbacks
     * - slow messages
     * @see callback
     */
    static void callback2(void * data);

    /**
     * Loop 3 Callbacks
     * - super slow messages
     * - log writing
     * @see callback
     */
    static void callback3(void * data);

    /**
     * Components
     */
    AP_Navigator * _navigator;
    AP_Guide * _guide;
    AP_Controller * _controller;
    AP_Board * _board;
};

} // namespace apo

#endif /* AP_AUTOPILOT_H_ */
// vim:ts=4:sw=4:expandtab
