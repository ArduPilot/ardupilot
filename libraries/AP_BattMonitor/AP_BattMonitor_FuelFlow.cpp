#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor_FuelFlow.h"
#include <GCS_MAVLink/GCS.h>

/*
  "battery" monitor for liquid fuel flow systems that give a pulse on
  a pin for fixed volumes of fuel.

  this driver assumes that BATTx_AMP_PERVLT is set to give the
  number of millilitres per pulse.

  Output is:

    - current in Amps maps to in litres/hour
    - consumed mAh is in consumed millilitres
    - fixed 1.0v voltage
 */
extern const AP_HAL::HAL& hal;

#define FUELFLOW_MIN_PULSE_DELTA_US 10

/// Constructor
AP_BattMonitor_FuelFlow::AP_BattMonitor_FuelFlow(AP_BattMonitor &mon,
                                                 AP_BattMonitor::BattMonitor_State &mon_state,
                                                 AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    _state.voltage = 1.0; // show a fixed voltage of 1v

    // we can't tell if it is healthy as we expect zero pulses when no
    // fuel is flowing
    _state.healthy = true;
}

/*
  handle interrupt on an instance
 */
void AP_BattMonitor_FuelFlow::irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp)
{
    if (irq_state.last_pulse_us == 0) {
        irq_state.last_pulse_us = timestamp;
        return;
    }
    uint32_t delta = timestamp - irq_state.last_pulse_us;
    if (delta < FUELFLOW_MIN_PULSE_DELTA_US) {
        // simple de-bounce
        return;
    }
    irq_state.pulse_count++;
    irq_state.total_us += delta;
    irq_state.last_pulse_us = timestamp;
}

/*
  read - read the "voltage" and "current"
*/
void AP_BattMonitor_FuelFlow::read()
{
    int8_t pin = _params._curr_pin;
    if (last_pin != pin) {
        // detach from last pin
        if (last_pin != -1) {
            hal.gpio->detach_interrupt(last_pin);
        }
        // attach to new pin
        last_pin = pin;
        if (last_pin > 0) {
            hal.gpio->pinMode(last_pin, HAL_GPIO_INPUT);
            if (!hal.gpio->attach_interrupt(
                    last_pin,
                    FUNCTOR_BIND_MEMBER(&AP_BattMonitor_FuelFlow::irq_handler, void, uint8_t, bool, uint32_t),
                    AP_HAL::GPIO::INTERRUPT_RISING)) {
                gcs().send_text(MAV_SEVERITY_WARNING, "FuelFlow: Failed to attach to pin %u", last_pin);
            }
        }
    }

    uint32_t now_us = AP_HAL::micros();
    if (_state.last_time_micros == 0) {
        // need initial time, so we can work out expected pulse rate
        _state.last_time_micros = now_us;
        return;
    }
    float dt = (now_us - _state.last_time_micros) * 1.0e-6;

    if (dt < 1 && irq_state.pulse_count == 0) {
        // we allow for up to 1 second with no pulses to cope with low
        // flow idling. After that we will start reading zero current
        return;
    }
    
    // get the IRQ state with interrupts disabled
    struct IrqState state;
    void *irqstate = hal.scheduler->disable_interrupts_save();
    state = irq_state;
    irq_state.pulse_count = 0;
    irq_state.total_us = 0;
    hal.scheduler->restore_interrupts(irqstate);

    /*
      this driver assumes that BATTx_AMP_PERVLT is set to give the
      number of millilitres per pulse.
     */
    float irq_dt = state.total_us * 1.0e-6;
    float litres, litres_pec_sec;
    if (state.pulse_count == 0) {
        litres = 0;
        litres_pec_sec = 0;
    } else {
        litres = state.pulse_count * _params._curr_amp_per_volt * 0.001;
        litres_pec_sec = litres / irq_dt;
    }

    _state.last_time_micros = now_us;

    // map amps to litres/hour
    _state.current_amps = litres_pec_sec * (60*60);

    // map consumed_mah to consumed millilitres
    _state.consumed_mah += litres * 1000;

    // map consumed_wh using fixed voltage of 1
    _state.consumed_wh = _state.consumed_mah;
}
