#include "AP_Periph.h"

#if AP_PERIPH_NOTIFY_ENABLED || AP_PERIPH_BUZZER_WITHOUT_NOTIFY_ENABLED

/*
  buzzer support
 */

#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

static uint32_t buzzer_start_ms;
static uint32_t buzzer_len_ms;

/*
  handle BeepCommand
 */
void AP_Periph_FW::handle_beep_command(CanardInstance* canard_instance, CanardRxTransfer* transfer)
{
    uavcan_equipment_indication_BeepCommand req;
    if (uavcan_equipment_indication_BeepCommand_decode(transfer, &req)) {
        return;
    }
    static bool initialised;
    if (!initialised) {
        initialised = true;
        hal.rcout->init();
        // just one buzzer type supported:
        hal.util->toneAlarm_init(uint8_t(AP_Notify::BuzzerType::BUILTIN));
    }
    buzzer_start_ms = AP_HAL::millis();
    buzzer_len_ms = req.duration*1000;
#if AP_PERIPH_BUZZER_WITHOUT_NOTIFY_ENABLED
    float volume = constrain_float(periph.g.buzz_volume*0.01f, 0, 1);
#elif AP_PERIPH_NOTIFY_ENABLED
    float volume = constrain_float(periph.notify.get_buzz_volume()*0.01f, 0, 1);
#endif
    hal.util->toneAlarm_set_buzzer_tone(req.frequency, volume, uint32_t(req.duration*1000));
}

/*
  update buzzer
 */
void AP_Periph_FW::can_buzzer_update(void)
{
    if (buzzer_start_ms != 0) {
        uint32_t now = AP_HAL::millis();
        if (now - buzzer_start_ms > buzzer_len_ms) {
            hal.util->toneAlarm_set_buzzer_tone(0, 0, 0);
            buzzer_start_ms = 0;
        }
    }
}

#endif // AP_PERIPH_BUZZER_WITHOUT_NOTIFY_ENABLED || AP_PERIPH_NOTIFY_ENABLED
