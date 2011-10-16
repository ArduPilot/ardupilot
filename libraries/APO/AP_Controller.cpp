/*
 * AP_Controller.cpp
 *
 *  Created on: Apr 30, 2011
 *      Author: jgoppert
 */

#include "AP_Controller.h"
#include "AP_HardwareAbstractionLayer.h"
#include "../AP_Common/include/menu.h"
#include "AP_RcChannel.h"

namespace apo {

AP_Controller::AP_Controller(AP_Navigator * nav, AP_Guide * guide,
		AP_HardwareAbstractionLayer * hal) :
	_nav(nav), _guide(guide), _hal(hal) {
	setAllRadioChannelsToNeutral();
}

void AP_Controller::setAllRadioChannelsToNeutral() {
	for (uint8_t i = 0; i < _hal->rc.getSize(); i++) {
		_hal->rc[i]->setPosition(0.0);
	}
}

void AP_Controller::setAllRadioChannelsManually() {
	for (uint8_t i = 0; i < _hal->rc.getSize(); i++) {
		_hal->rc[i]->setUsingRadio();
	}
}

}
