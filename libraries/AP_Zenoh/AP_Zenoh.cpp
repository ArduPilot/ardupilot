#include "AP_Zenoh_config.h"

#if AP_ZENOH_ENABLED

#include "AP_Zenoh.h"

extern const AP_HAL::HAL& hal;

AP_Zenoh *AP_Zenoh::_singleton = nullptr;

AP_Zenoh::AP_Zenoh(const AP_Zenoh_ConfigItem *config_items, uint8_t config_count) :
    _config_items(config_items),
    _config_count(config_count),
    _initialized(false)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Zenoh must be singleton");
    }
#endif
    _singleton = this;
}

bool AP_Zenoh::init()
{
    if (_initialized) {
        return true;
    }

    z_owned_config_t config;
    z_config_default(&config);

    for (uint8_t i = 0; i < _config_count; i++) {
        if (zp_config_insert(z_loan_mut(config), _config_items[i].key, _config_items[i].value) < 0) {
            hal.console->printf("%s: failed to set config key %u\n",
                                msg_prefix, _config_items[i].key);
            z_drop(z_move(config));
            return false;
        }
    }

    if (z_open(&_session, z_move(config), NULL) < 0) {
        hal.console->printf("%s: failed to open session\n", msg_prefix);
        return false;
    }

    if (zp_start_read_task(z_loan_mut(_session), NULL) < 0 ||
        zp_start_lease_task(z_loan_mut(_session), NULL) < 0) {
        hal.console->printf("%s: failed to start background tasks\n", msg_prefix);
        z_drop(z_move(_session));
        return false;
    }

    _initialized = true;
    return true;
}

bool AP_Zenoh::declare_publisher(const char *key_expr, AP_Zenoh_Publisher &pub)
{
    if (!_initialized) {
        hal.console->printf("%s: not initialized\n", msg_prefix);
        return false;
    }

    z_view_keyexpr_t ke;
    z_view_keyexpr_from_str(&ke, key_expr);

    if (z_declare_publisher(z_loan(_session), &pub._pub, z_loan(ke), NULL) < 0) {
        hal.console->printf("%s: failed to declare publisher on '%s'\n",
                            msg_prefix, key_expr);
        return false;
    }

    pub._valid = true;
    return true;
}

bool AP_Zenoh::declare_subscriber(const char *key_expr,
                                  AP_Zenoh_Subscriber &sub,
                                  AP_Zenoh_Subscriber::callback_t callback)
{
    if (!_initialized) {
        hal.console->printf("%s: not initialized\n", msg_prefix);
        return false;
    }

    sub._callback = callback;

    z_view_keyexpr_t ke;
    z_view_keyexpr_from_str(&ke, key_expr);

    z_closure(&sub._closure, AP_Zenoh_Subscriber::_trampoline, NULL, &sub);

    if (z_declare_subscriber(z_loan(_session), &sub._sub,
                             z_loan(ke), z_move(sub._closure), NULL) < 0) {
        hal.console->printf("%s: failed to declare subscriber on '%s'\n",
                            msg_prefix, key_expr);
        return false;
    }

    sub._valid = true;
    return true;
}

#endif // AP_ZENOH_ENABLED
