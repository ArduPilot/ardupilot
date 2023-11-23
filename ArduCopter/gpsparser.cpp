#include "Copter.h"

void Copter::gpsparser_init(){
static bool setupDone = false;
hal.console->println("Task in scheduler runs");
if (!setupDone)
{
    gpsParser.setup();  
    setupDone = true;    
    hal.console->println("Setup completed");
}
    gpsParser.process();
    hal.console->println("Process igang");
    
}


  