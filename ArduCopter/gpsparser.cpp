#include "Copter.h"

void Copter::gpsparser_init() {
    
    // Perform the setup only once
    gpsParser.setup();

    // Process the GPS parser in each iteration
    gpsParser.process();
}




