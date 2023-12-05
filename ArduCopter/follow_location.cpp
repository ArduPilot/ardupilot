#include "Copter.h"
#include "AP_BattMonitor/AP_BattMonitor.h"

void Copter::follow_location(){
    static bool setupDone = false;    
    if (flightmode->is_taking_off()){ // If the drone is taking off, set the start location
        if (!setupDone){
            followLocation._init();   // Setting the start coordinates
            setupDone = true;
            return;
        }
    }

    if(setupDone){
        if ((!flightmode->is_taking_off()) && followLocation.get_location()){ //If the drone has taken off and the locations are valid
            if (followLocation.check_location()){
                // followLocation.update_destination(); // Outdated and is not nedded.
                followLocation.update_velocity();       // Update the velocity and the direction of the drone
                return;
            }
            return;
        }
        return;
    }
    /*
    // get the battery level and land if it is below 10%
    if (battery.get_cell_voltages()) {
        copter.set_mode(Mode::Number::LAND, ModeReason::GCS_COMMAND);
        return;
    }
    */
}