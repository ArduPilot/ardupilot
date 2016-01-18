// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  All APM Project credits from the original work are kept intact below as a
 *  courtesy.
 */

/*
 * AP_Relay.h
 *
 *  Created on: Oct 2, 2011
 *      Author: Amilcar Lucas
 */

/// @file	AP_Relay.h
/// @brief	APM relay control class

#ifndef __AP_RELAY_H__
#define __AP_RELAY_H__


#define AP_RELAY_NUM_RELAYS 2

/// @class	AP_Relay
/// @brief	Class to manage the APM relay
class AP_Relay {
public:
    AP_Relay();

    // setup the relay pin
    void        init();

    // activate the relay
    void        on();

    // de-activate the relay
    void        off();

    // see if the relay is enabled
    bool        enabled() { return true; }

    // toggle the relay status
    void        toggle();

private:

};

#endif /* AP_RELAY_H_ */
