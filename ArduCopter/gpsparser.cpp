#include "Copter.h"

void Copter::gpsparser_init() {
    hal.console->println("Entering gpsparser_init()");
    
    // Check if the GPS parser is already ready
    if (gpsParser.get_isReady()) {
        hal.console->println("GPS parser is already ready");
    } else {
        // Perform the setup only once
        gpsParser.setup();
        hal.console->println("Setup completed");
    }

    // Process the GPS parser in each iteration
    gpsParser.process();
    hal.console->println("Exiting gpsparser_init()");
}




