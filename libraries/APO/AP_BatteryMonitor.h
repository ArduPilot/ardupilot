/*
 * AP_BatteryMonitor.h
 *
 */

#ifndef AP_BATTERYMONITOR_H_
#define AP_BATTERYMONITOR_H_

#include <inttypes.h>
#include <wiring.h>

namespace apo {

class AP_BatteryMonitor {
public:
    AP_BatteryMonitor(uint8_t pin, float voltageDivRatio, float minVolt, float maxVolt) :
        _pin(pin), _voltageDivRatio(voltageDivRatio),
        _minVolt(minVolt), _maxVolt(maxVolt), _voltage(maxVolt) {
    }

    void update() {
        // low pass filter on voltage
        _voltage = _voltage*.9 + (analogRead(_pin)/255)*_voltageDivRatio*0.1;
    }

    /**
     * Accessors
     */
    float getVoltage() {
        return _voltage;
    }

    uint8_t getPercentage() {
        float norm = (_voltage-_minVolt)/(_maxVolt-_minVolt);
        if (norm < 0) norm = 0;
        else if (norm > 1) norm = 1;
        return 100*norm;
    }

private:

    uint8_t _pin;
    float _voltageDivRatio;
    float _voltage;
    float _minVolt;
    float _maxVolt;
};

} // namespace apo

#endif /* AP_BATTERYMONITOR_H_ */
// vim:ts=4:sw=4:expandtab
