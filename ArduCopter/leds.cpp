#include "Copter.h"


// updates the status of notify
// should be called at 50hz
void Copter::update_notify()
{
    notify.update();
}

