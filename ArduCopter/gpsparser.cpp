#include "Copter.h"

void Copter::gps_parser_task() {
    
    // Perform the setup only once
    gpsParser.setup();

    // Process the GPS parser in each iteration
    gpsParser.process();
}




