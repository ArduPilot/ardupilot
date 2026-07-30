#include "AP_Zenoh_config.h"

#if AP_ZENOH_ENABLED

#include "AP_Zenoh_Subscriber.h"

void AP_Zenoh_Subscriber::_trampoline(z_loaned_sample_t *sample, void *arg)
{
    auto *self = static_cast<AP_Zenoh_Subscriber *>(arg);
    if (self == nullptr || !self->_callback) {
        return;
    }
    AP_Zenoh_Payload payload(sample);
    self->_callback(payload);
}

#endif // AP_ZENOH_ENABLED
