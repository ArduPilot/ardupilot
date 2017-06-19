#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_Mavlink.h"
#include "config.h" // for CLI_ENABLED

class GCS_Plane : public GCS
{
    friend class Plane; // for temporary access to num_gcs and links

public:

    FUNCTOR_TYPEDEF(run_cli_fn, void, AP_HAL::UARTDriver*);

    // return the number of valid GCS objects
    uint8_t num_gcs() const { return _num_gcs; };

    // return GCS link at offset ofs
    GCS_MAVLINK_Plane &chan(const uint8_t ofs) { return _chan[ofs]; };

    void reset_cli_timeout();
    void send_message(enum ap_message id);
    void send_mission_item_reached_message(uint16_t mission_index);
    void data_stream_send();
    void update();
    void send_airspeed_calibration(const Vector3f &vg);

    void set_run_cli_func(run_cli_fn run_cli) { _run_cli = run_cli; }
    void setup_uarts(AP_SerialManager &serial_manager);
#if CLI_ENABLED == ENABLED
    void handle_interactive_setup();
#endif

private:

    uint8_t _num_gcs = MAVLINK_COMM_NUM_BUFFERS;
    GCS_MAVLINK_Plane _chan[MAVLINK_COMM_NUM_BUFFERS];
    run_cli_fn _run_cli;

};
