#include "Sub.h"

// set_home_state - update home state
void Sub::set_home_state(enum HomeState new_home_state)
{
    // if no change, exit immediately
    if (ap.home_state == new_home_state) {
        return;
    }

    // update state
    ap.home_state = new_home_state;

    // log if home has been set
    if (new_home_state == HOME_SET_NOT_LOCKED || new_home_state == HOME_SET_AND_LOCKED) {
        Log_Write_Event(DATA_SET_HOME);
    }
}

// home_is_set - returns true if home positions has been set (to GPS location, armed location or EKF origin)
bool Sub::home_is_set()
{
    return (ap.home_state == HOME_SET_NOT_LOCKED || ap.home_state == HOME_SET_AND_LOCKED);
}
