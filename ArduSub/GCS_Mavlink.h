#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_Sub : public GCS_MAVLINK {

public:

    void data_stream_send(void) override;

protected:

    uint32_t telem_delay() const override {
        return 0;
    };

private:

    void handleMessage(mavlink_message_t * msg) override;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd) override;
    bool try_send_message(enum ap_message id) override;

};
