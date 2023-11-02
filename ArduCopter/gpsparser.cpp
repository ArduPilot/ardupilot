#include "Copter.h"

void Copter::gpsparser_init(){
static bool setupDone = false;

if (!setupDone)
{
    gpsParser.setup();  
    setupDone = true;    
    hal.console->printf("Setup completed");
}
    gpsParser.process();
}

  