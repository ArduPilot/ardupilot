#include "AP_Periph.h"
#include "elrs_receiver.h"

#if AP_PERIPH_ELRS_ENABLED

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo Parameters_ELRS::var_info[] {
    // Internal binding storage; hidden from parameter downloads.
    AP_GROUPINFO_FLAGS("UID1", 1, Parameters_ELRS, uid1, 0, AP_PARAM_FLAG_HIDDEN),
    AP_GROUPINFO_FLAGS("UID2", 2, Parameters_ELRS, uid2, 0, AP_PARAM_FLAG_HIDDEN),

    // @Param: MODEL
    // @DisplayName: ELRS model match ID
    // @Description: Model ID used to validate ELRS synchronization packets. Set 255 when transmitter model match is disabled.
    // @Range: 0 255
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("MODEL", 3, Parameters_ELRS, model_id, 255),

    // @Param: BIND
    // @DisplayName: ELRS bind on next boot
    // @Description: Set to 1 and reboot to listen for a 2.4 GHz ELRS binding packet. The request clears after startup version validation. No RC data is forwarded while binding. A learned UID is saved and the receiver reboots automatically. Reboot again to cancel without changing the old UID. Model Match setting is retained.
    // @Values: 0:Normal,1:Bind on next boot
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("BIND", 4, Parameters_ELRS, bind, 0),

    // @Param: VER
    // @DisplayName: ELRS compatibility version
    // @Description: Select the ExpressLRS compatibility version implemented by this receiver. Only version 4 is supported. Other values prevent reception and binding until corrected and rebooted.
    // @Values: 4:ExpressLRS 4
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("VER", 5, Parameters_ELRS, version, 4),

    AP_GROUPEND
};

Parameters_ELRS::Parameters_ELRS()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void Parameters_ELRS::convert_parameters()
{
    // Preserve values stored in the former flat RC parameter table.
    static const AP_Param::ConversionInfoNoKey conversion_info[] {
        {6, AP_PARAM_INT32, "RC_ELRS_UID1"},
        {7, AP_PARAM_INT32, "RC_ELRS_UID2"},
        {8, AP_PARAM_INT16, "RC_ELRS_MODEL"},
        {9, AP_PARAM_INT8, "RC_ELRS_BIND"},
        {10, AP_PARAM_INT8, "RC_ELRS_VER"},
    };
    AP_Param::convert_old_parameters(Parameters::k_param_g_rcin, conversion_info, ARRAY_SIZE(conversion_info));
}

static AP_ELRSReceiver elrs_receiver;

bool AP_Periph_FW::elrs_binding_active() const
{
    return elrs_receiver.binding_active();
}

const char *AP_Periph_FW::elrs_error_message() const
{
    if (elrs_version_error) {
        return "ELRS: unsupported RC_ELRS_VER; select 4 and reboot";
    }
    return elrs_receiver.error_message();
}

void AP_Periph_FW::elrs_init()
{
    if (g_rcin.elrs.version != 4) {
        elrs_version_error = true;
        return;
    }
    bool bind = g_rcin.elrs.bind == 1;
    if (bind) {
        g_rcin.elrs.bind.set_and_save(0);
    }
#ifdef HAL_ELRS_BUTTON_PIN
    if (!bind &&
        hal.gpio->read(HAL_ELRS_BUTTON_PIN) == HAL_ELRS_BUTTON_ACTIVE_HIGH) {
        bind = true;
        // Accept only a button held throughout the startup debounce interval.
        for (uint8_t i = 0; i < 4; i++) {
            hal.scheduler->delay(5);
            if (hal.gpio->read(HAL_ELRS_BUTTON_PIN) != HAL_ELRS_BUTTON_ACTIVE_HIGH) {
                bind = false;
                break;
            }
        }
    }
#endif
    elrs_receiver.init(g_rcin.elrs.uid1, g_rcin.elrs.uid2, g_rcin.elrs.model_id, bind);
}

void AP_Periph_FW::elrs_update()
{
    elrs_receiver.update();
    uint8_t learned_uid[ELRS::UID_LENGTH];
    if (elrs_receiver.read_bound_uid(learned_uid)) {
        g_rcin.elrs.uid1.set_and_save((uint32_t(learned_uid[0]) << 16) |
                                      (uint32_t(learned_uid[1]) << 8) | learned_uid[2]);
        g_rcin.elrs.uid2.set_and_save((uint32_t(learned_uid[3]) << 16) |
                                      (uint32_t(learned_uid[4]) << 8) | learned_uid[5]);
        reboot(false);
        return;
    }
    const uint8_t rate_hz = g_rcin.rcin_rate_hz;
    const uint32_t now_ms = AP_HAL::millis();
    if (rate_hz == 0 || now_ms - rcin_last_sent_RCInput_ms < 1000U / rate_hz) {
        return;
    }
    uint16_t channels[ELRS::Tracker::OUTPUT_CHANNELS];
    uint8_t quality, count;
    bool failsafe;
    if (elrs_receiver.read_input(channels, count, quality, failsafe)) {
        rcin_last_sent_RCInput_ms = now_ms;
        can_send_RCInput(quality, channels, count, failsafe, true);
    }
}

#endif // AP_PERIPH_ELRS_ENABLED
