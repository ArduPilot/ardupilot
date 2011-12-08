/*
 * AP_Controller.h
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef AP_Controller_H
#define AP_Controller_H

// inclusions
#include "../AP_Common/AP_Common.h"
#include "../AP_Common/AP_Var.h"
#include <inttypes.h>
#include <math.h>
#include "../GCS_MAVLink/GCS_MAVLink.h"

namespace apo {

// forward declarations within apo
class AP_Board;
class AP_Guide;
class AP_Navigator;
class Menu;
class AP_ArmingMechanism;

/// 
// The control system class.
// Given where the vehicle wants to go and where it is, 
// this class is responsible for sending commands to the 
// motors. It is also responsible for monitoring manual
// input.
class AP_Controller {
public:
    ///
    // The controller constructor.
    // Creates the control system.
    // @nav the navigation system
    // @guide the guidance system
    // @board the hardware abstraction layer
    // @armingMechanism the device that controls arming/ disarming
    // @chMode the channel that the mode switch is on
    // @key the unique key for the control system saved AP_Var variables
    // @name the name of the control system
    AP_Controller(AP_Navigator * nav, AP_Guide * guide,
                  AP_Board * board,
                  AP_ArmingMechanism * armingMechanism,
                  const uint8_t _chMode,
                  const uint16_t key,
                  const prog_char_t * name = NULL);

    ///
    // The loop callback function.
    // The callback function for the controller loop.
    // This is inherited from loop.
    // This function cannot be overriden.
    // @dt The loop update interval.
    void update(const float dt);

    ///
    // This sets all radio outputs to neutral.
    // This function cannot be overriden.
    void setAllRadioChannelsToNeutral();

    ///
    // This sets all radio outputs using the radio input.
    // This function cannot be overriden.
    void setAllRadioChannelsManually();

    ///
    // Sets the motor pwm outputs.
    // This function sets the motors given the control system outputs.
    // This function must be defined. There is no default implementation.
    virtual void setMotors() = 0;

    ///
    // The manual control loop function.
    // This uses radio to control the aircraft. 
    // This function must be defined. There is no default implementation.
    // @dt The loop update interval.
    virtual void manualLoop(const float dt) = 0;

    ///
    // The automatic control update function.
    // This loop is responsible for taking the 
    // vehicle to a waypoint.
    // This function must be defined. There is no default implementation.
    // @dt The loop update interval.
    virtual void autoLoop(const float dt) = 0;

    ///
    // Handles failsafe events.
    // This function is responsible for setting the mode of the vehicle during
    // a failsafe event (low battery, loss of gcs comms, ...).
    // This function must be defined. There is no default implementation.
    virtual void handleFailsafe() = 0;

    ///
    // The mode accessor.
    // @return The current vehicle mode.
    MAV_MODE getMode() {
        return _mode;
    }
    ///
    // The mode setter.
    // @mode The  mode to set the vehicle to.
    void setMode(MAV_MODE mode) {
        _mode = mode;
    }
    ///
    // The state acessor.
    // @return The current state of the vehicle.
    MAV_STATE getState() const { 
        return _state;
    }
    ///
    // state setter
    // @sate The state to set the vehicle to.
    void setState(MAV_STATE state) {
        _state = state;
    }

protected:
    AP_Navigator * _nav;                    /// navigator
    AP_Guide * _guide;                      /// guide
    AP_Board * _board;     /// hardware abstraction layer
    AP_ArmingMechanism * _armingMechanism;  /// controls arming/ disarming
    uint8_t _chMode;                        /// the channel the mode switch is on
    AP_Var_group _group;                    /// holds controller parameters
    MAV_MODE _mode;                         /// vehicle mode (auto, guided, manual, failsafe, ...)
    MAV_STATE _state;                       /// vehicle state (active, standby, boot, calibrating ...)
};

} // apo

#endif // AP_Controller_H
// vim:ts=4:sw=4:expandtab
