// this file is #included in avoidance_handler.h

#include "defines.h" // for control_mode_t

class AvoidanceHandler__ModeChange : public AvoidanceHandler {

public:

    bool update() override;
    virtual bool enter(class AP_Avoidance::Obstacle &threat, class AvoidanceHandler *old_handler) override;
    void exit() override;

protected:

    virtual control_mode_t mode() const = 0;

private:

    control_mode_t _old_flight_mode;
    uint32_t _last_mode_change; // timestamp

    static const uint16_t _mode_change_hysteresis = 10000; // milliseconds

    // change the flight mode back to what it was before entering AVOID
    void exit_revert_flight_mode();

};
