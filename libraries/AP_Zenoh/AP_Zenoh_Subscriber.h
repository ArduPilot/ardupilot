#pragma once

#include "AP_Zenoh_config.h"

#if AP_ZENOH_ENABLED

#include <AP_HAL/utility/functor.h>
#include <zenoh-pico.h>
#include "AP_Zenoh_Payload.h"

class AP_Zenoh_Subscriber
{
public:
    FUNCTOR_TYPEDEF(callback_t, void, const AP_Zenoh_Payload &);

    bool valid() const
    {
        return _valid;
    }

private:
    z_owned_subscriber_t _sub;
    z_owned_closure_sample_t _closure;
    callback_t _callback;
    bool _valid{false};

    static void _trampoline(z_loaned_sample_t *sample, void *arg);

    friend class AP_Zenoh;
};

#endif // AP_ZENOH_ENABLED
