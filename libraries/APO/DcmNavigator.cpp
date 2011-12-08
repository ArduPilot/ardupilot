/*
 * DcmNavigator.cpp
 *
 *  Created on: Dec 6, 2011
 *      Author: jgoppert/ wenyaoxie
 */

#include "../FastSerial/FastSerial.h"
#include "DcmNavigator.h"
#include "AP_CommLink.h"
#include "AP_HardwareAbstractionLayer.h"
#include "../AP_DCM/AP_DCM.h"
#include "../AP_Math/AP_Math.h"
#include "../AP_Compass/AP_Compass.h"
#include "AP_MavlinkCommand.h"
#include "AP_Var_keys.h"
#include "../AP_RangeFinder/RangeFinder.h"
#include "../AP_IMU/AP_IMU.h"
#include "../AP_InertialSensor/AP_InertialSensor.h"
#include "../APM_BMP085/APM_BMP085_hil.h"
#include "../APM_BMP085/APM_BMP085.h"

namespace apo {

DcmNavigator::DcmNavigator(AP_HardwareAbstractionLayer * hal, const uint16_t key, const prog_char_t * name) :
    AP_Navigator(hal), _imuOffsetAddress(0),
    _dcm(_hal->imu, _hal->gps, _hal->compass),
    _rangeFinderDown(),
    _group(key, name ? : PSTR("NAV_")),
    _baroLowPass(&_group,1,10,PSTR("BAROLP")),
    _groundTemperature(&_group,2, 25.0,PSTR("GNDT")), // TODO check temp
    _groundPressure(&_group,3,0.0,PSTR("GNDP")) {

    // if orientation equal to front, store as front
    /**
     * rangeFinder<direction> is assigned values based on orientation which
     * is specified in ArduPilotOne.pde.
     */
    for (uint8_t i = 0; i < _hal-> rangeFinders.getSize(); i++) {
        if (_hal->rangeFinders[i] == NULL)
            continue;
        if (_hal->rangeFinders[i]->orientation_x == 0
                && _hal->rangeFinders[i]->orientation_y == 0
                && _hal->rangeFinders[i]->orientation_z == 1)
            _rangeFinderDown = _hal->rangeFinders[i];
    }

    // tune down dcm
    _dcm.kp_roll_pitch(0.030000);
    _dcm.ki_roll_pitch(0.00001278),	// 50 hz I term

    // tune down compass in dcm
    _dcm.kp_yaw(0.08);
    _dcm.ki_yaw(0);

    if (_hal->compass) {
        _dcm.set_compass(_hal->compass);
    }
}
void DcmNavigator::calibrate() {

    AP_Navigator::calibrate();

    // TODO: handle cold/warm restart
    if (_hal->imu) {
        _hal->imu->init(IMU::COLD_START,delay,_hal->scheduler);
    }

    if (_hal->baro) {

        int flashcount = 0;

        while(_groundPressure == 0){
            _hal->baro->Read(); 					// Get initial data from absolute pressure sensor
            _groundPressure = _hal->baro->Press;
            _groundTemperature = _hal->baro->Temp/10.0;
            delay(20);
        }

        for(int i = 0; i < 30; i++){		// We take some readings...

            // set using low pass filters
            _groundPressure = _groundPressure * 0.9   + _hal->baro->Press * 0.1;
            _groundTemperature = _groundTemperature * 0.9   + (_hal->baro->Temp/10.0) * 0.1;

            //mavlink_delay(20);
            delay(20);
            if(flashcount == 5) {
                digitalWrite(_hal->cLedPin, LOW);
                digitalWrite(_hal->aLedPin, HIGH);
                digitalWrite(_hal->bLedPin, LOW);
            }

            if(flashcount >= 10) {
                flashcount = 0;
                digitalWrite(_hal->cLedPin, LOW);
                digitalWrite(_hal->aLedPin, HIGH);
                digitalWrite(_hal->bLedPin, LOW);
            }
            flashcount++;
        }
        
        _groundPressure.save();
        _groundTemperature.save();

        _hal->debug->printf_P(PSTR("ground pressure: %ld ground temperature: %d\n"),_groundPressure.get(), _groundTemperature.get());
        _hal->gcs->sendText(SEVERITY_LOW, PSTR("barometer calibration complete\n"));
    }
}

void DcmNavigator::updateFast(float dt) {

    if (_hal->getMode() != MODE_LIVE)
        return;

    setTimeStamp(micros()); // if running in live mode, record new time stamp

    // use range finder if attached and close to the ground
    if (_rangeFinderDown != NULL && _rangeFinderDown->distance <= 695) {
        setAlt(_rangeFinderDown->distance);

    // otherwise if you have a baro attached, use it
    } else if (_hal->baro) {
        /**
         * The altitued is read off the barometer by implementing the following formula:
         * altitude (in m) = 44330*(1-(p/po)^(1/5.255)),
         * where, po is pressure in Pa at sea level (101325 Pa).
         * See http://www.sparkfun.com/tutorials/253 or type this formula
         * in a search engine for more information.
         * altInt contains the altitude in meters.
         *
         * pressure input is in pascals
         * temp input is in deg C *10
         */
        _hal->baro->Read();		// Get new data from absolute pressure sensor
        float reference = 44330 * (1.0 - (pow(_groundPressure.get()/101325.0,0.190295)));
        setAlt(_baroLowPass.update((44330 * (1.0 - (pow((_hal->baro->Press/101325.0),0.190295)))) - reference,dt));
        //_hal->debug->printf_P(PSTR("Ground Pressure %f\tAltitude = %f\tGround Temperature = %f\tPress = %ld\tTemp = %d\n"),_groundPressure.get(),getAlt(),_groundTemperature.get(),_hal->baro->Press,_hal->baro->Temp);
        
    // last resort, use gps altitude
    } else if (_hal->gps && _hal->gps->fix) {
        setAlt_intM(_hal->gps->altitude * 10); // gps in cm, intM in mm
    }

    // update dcm calculations and navigator data
    //
    _dcm.update_DCM_fast();
    setRoll(_dcm.roll);
    setPitch(_dcm.pitch);
    setYaw(_dcm.yaw);
    setRollRate(_dcm.get_gyro().x);
    setPitchRate(_dcm.get_gyro().y);
    setYawRate(_dcm.get_gyro().z);
    setXAccel(_dcm.get_accel().x);
    setYAccel(_dcm.get_accel().y);
    setZAccel(_dcm.get_accel().z);

    /*
     * accel/gyro debug
     */
    /*
     Vector3f accel = _hal->imu->get_accel();
     Vector3f gyro = _hal->imu->get_gyro();
     Serial.printf_P(PSTR("accel: %f %f %f gyro: %f %f %f\n"),
     accel.x,accel.y,accel.z,gyro.x,gyro.y,gyro.z);
     */
}

void DcmNavigator::updateSlow(float dt) {
    if (_hal->getMode() != MODE_LIVE)
        return;

    setTimeStamp(micros()); // if running in live mode, record new time stamp

    if (_hal->gps) {
        _hal->gps->update();
        updateGpsLight();
        if (_hal->gps->fix && _hal->gps->new_data) {
            setLat_degInt(_hal->gps->latitude);
            setLon_degInt(_hal->gps->longitude);
            setGroundSpeed(_hal->gps->ground_speed / 100.0); // gps is in cm/s
        }
    }

    if (_hal->compass) {
        _hal->compass->read();
        _hal->compass->calculate(_dcm.get_dcm_matrix());
        _hal->compass->null_offsets(_dcm.get_dcm_matrix());
        //_hal->debug->printf_P(PSTR("heading: %f"), _hal->compass->heading);
    }
}
void DcmNavigator::updateGpsLight(void) {
    // GPS LED on if we have a fix or Blink GPS LED if we are receiving data
    // ---------------------------------------------------------------------
    static bool GPS_light = false;
    switch (_hal->gps->status()) {
    case (2):
        //digitalWrite(C_LED_PIN, HIGH); //Turn LED C on when gps has valid fix.
        break;

    case (1):
        if (_hal->gps->valid_read == true) {
            GPS_light = !GPS_light; // Toggle light on and off to indicate gps messages being received, but no GPS fix lock
            if (GPS_light) {
                digitalWrite(_hal->cLedPin, LOW);
            } else {
                digitalWrite(_hal->cLedPin, HIGH);
            }
            _hal->gps->valid_read = false;
        }
        break;

    default:
        digitalWrite(_hal->cLedPin, LOW);
        break;
    }
}

} // namespace apo
// vim:ts=4:sw=4:expandtab
