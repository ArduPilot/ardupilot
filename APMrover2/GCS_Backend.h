#include "GCS_MAVLink/GCS.h"

class GCS_Backend_Rover : public GCS_MAVLINK {

public:

    bool try_send_message(enum ap_message id) override;
    bool stream_trigger(enum streams stream_num) override;
    void data_stream_send(void) override;
    void handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd) override;
    void handleMessage(mavlink_message_t* msg) override;

protected:

private:

};
