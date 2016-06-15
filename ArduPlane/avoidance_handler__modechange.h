// this file is #included in avoidance_handler.h

#include "defines.h" // for control_mode_t

class AvoidanceHandler__ModeChange : public AvoidanceHandler {

public:

    bool update() override;
    virtual bool enter(class AP_Avoidance::Obstacle &threat, class AvoidanceHandler *old_handler) override;
    void exit() override;

protected:

    virtual FlightMode mode() const = 0;
    Location guided_WP_loc;

private:

    FlightMode _old_flight_mode;
    uint32_t _last_mode_change; // timestamp

    static const uint16_t _mode_change_hysteresis = 20000; // milliseconds

    // change the flight mode back to what it was before entering AVOID
    void exit_revert_flight_mode();

    bool set_mode(const FlightMode _mode);

    // keep track of various things to make sure we don't surprise the
    // user with a mode change when they've taken control of the
    // aircraft:
    uint8_t old_switch_position;

    // returns true if it looks like the user has decided they can
    // resolve this situation better than we can
    virtual bool user_has_taken_control();

};
