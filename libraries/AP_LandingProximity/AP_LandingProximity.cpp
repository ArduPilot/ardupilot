#include "AP_LandingProximity.h"

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo AP_LandingProximity::var_info[] = {
    // @Param: ENABLED
    // @DisplayName: enable land proximity
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLED", 0, AP_LandingProximity, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: THRESHOLD
    // @DisplayName: proximity threshold
    // @User: Advanced
    AP_GROUPINFO("THRESHOLD", 1, AP_LandingProximity, _thd, DEFAULT_PIHT),

    // @Param: GAIN
    // @DisplayName: proximity gain
    // @User: Advanced
    AP_GROUPINFO("GAIN", 2, AP_LandingProximity, _gain, DEFAULT_PGAIN),

    // @Param: PERSISTENCE
    // @DisplayName: proximity persistence
    // @User: Advanced
    AP_GROUPINFO("PERSISTENCE", 3, AP_LandingProximity, _pers, DEFAULT_PPERS),

    AP_GROUPEND
};

AP_LandingProximity::AP_LandingProximity()
{
    proximity = false;
}

bool AP_LandingProximity::setProximityGain(uint8_t drive)
{
    uint8_t val;
    uint8_t send = APDS9930_CONTROL | AUTO_INCREMENT;
    
    /* Read value from CONTROL register */
    if (!_dev->transfer(&send, 1, &val, 1)) {
        return false;
    }
    
    /* Set bits in register to given value */
    drive &= 0b00000011;
    drive = drive << 2;
    val &= 0b11110011;
    val |= drive;
    
    /* Write register value back into CONTROL register */
    if (!_dev->write_register(APDS9930_CONTROL | AUTO_INCREMENT, val)) {
        return false;
    }
    
    return true;
}

bool AP_LandingProximity::setLEDDrive(uint8_t drive)
{
    uint8_t val;
    uint8_t send = APDS9930_CONTROL | AUTO_INCREMENT;
    
    /* Read value from CONTROL register */
    if (!_dev->transfer(&send, 1, &val, 1)) {
        return false;
    }
    
    /* Set bits in register to given value */
    drive &= 0b00000011;
    drive = drive << 6;
    val &= 0b00111111;
    val |= drive;
    
    /* Write register value back into CONTROL register */
    if (!_dev->write_register(APDS9930_CONTROL | AUTO_INCREMENT, val)) {
        return false;
    }
    
    return true;
}

bool AP_LandingProximity::setProximityDiode(uint8_t drive)
{
    uint8_t val;
    uint8_t send = APDS9930_CONTROL | AUTO_INCREMENT;
    
    /* Read value from CONTROL register */
    if (!_dev->transfer(&send, 1, &val, 1)) {
        return false;
    }
    
    /* Set bits in register to given value */
    drive &= 0b00000011;
    drive = drive << 4;
    val &= 0b11001111;
    val |= drive;
    
    /* Write register value back into CONTROL register */
    if (!_dev->write_register(APDS9930_CONTROL | AUTO_INCREMENT, val)) {
        return false;
    }
    
    return true;
}

bool AP_LandingProximity::enableProximitySensor()
{
    /* Set default gain, LED, interrupts, enable power, and enable sensor */
    if( !setProximityGain(_gain) ) {
        return false;
    }
    if( !setLEDDrive(DEFAULT_PDRIVE) ) {
        return false;
    }
    if (!_dev->write_register(APDS9930_ENABLE | AUTO_INCREMENT, 0x25)) {//power, proximity, proximity interrupt
        return false;
    }
    
    return true;
}

bool AP_LandingProximity::checkId() {
    uint8_t send = APDS9930_ID | AUTO_INCREMENT;
    uint8_t val;
    if (_dev->transfer(&send, 1, &val, 1) && val == 0x39) return true;
    return false;
}

void AP_LandingProximity::timer() {
    uint8_t send = APDS9930_STATUS | AUTO_INCREMENT;
    uint8_t val;
    proximity = false;
    if (!_enabled) return;
    if (!_dev->transfer(&send, 1, &val, 1)) return;
    if (val & 0x20) {
        proximity = true;
        send = CLEAR_PROX_INT;
        _dev->transfer(&send, 1, 0, 0);
    }
}

bool AP_LandingProximity::setProximityIntHighThreshold(uint16_t threshold)
{
    uint8_t lo;
    uint8_t hi;
    hi = threshold >> 8;
    lo = threshold & 0x00FF;

    if( !_dev->write_register(APDS9930_PIHTL | AUTO_INCREMENT, lo) ) {
        return false;
    }
    if( !_dev->write_register(APDS9930_PIHTH | AUTO_INCREMENT, hi) ) {
        return false;
    }
    
    return true;
}

void AP_LandingProximity::init()
{
    if (!_enabled) return;
    _dev = hal.i2c_mgr->get_device(0, 0x39);
    if (_dev && _dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (!checkId()) {
            _dev->get_semaphore()->give();
            return;
        }
        if (!_dev->write_register(APDS9930_ENABLE | AUTO_INCREMENT, 0)) {
            _dev->get_semaphore()->give();
            return;
        }
        if (!_dev->write_register(APDS9930_ATIME | AUTO_INCREMENT, DEFAULT_ATIME)) {
            _dev->get_semaphore()->give();
            return;
        }
        if (!_dev->write_register(APDS9930_WTIME | AUTO_INCREMENT, DEFAULT_WTIME)) {
            _dev->get_semaphore()->give();
            return;
        }
        if (!_dev->write_register(APDS9930_PPULSE | AUTO_INCREMENT, DEFAULT_PPULSE)) {
            _dev->get_semaphore()->give();
            return;
        }
        if (!_dev->write_register(APDS9930_POFFSET | AUTO_INCREMENT, DEFAULT_POFFSET)) {
            _dev->get_semaphore()->give();
            return;
        }
        if (!_dev->write_register(APDS9930_CONFIG | AUTO_INCREMENT, DEFAULT_CONFIG)) {
            _dev->get_semaphore()->give();
            return;
        }
        if (!setProximityDiode(DEFAULT_PDIODE)) {
            _dev->get_semaphore()->give();
            return;
        }
        if( !setProximityIntHighThreshold(_thd) ) {
            _dev->get_semaphore()->give();
            return;
        }
        if( !_dev->write_register(APDS9930_PERS | AUTO_INCREMENT, _pers << 4) ) {
            _dev->get_semaphore()->give();
            return;
        }
        if (!enableProximitySensor()) {
            _dev->get_semaphore()->give();
            return;
        }            
        _dev->get_semaphore()->give();
        _dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_LandingProximity::timer, void));
    }    
}
