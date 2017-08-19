/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"


// updates the status of notify
// should be called at 50hz
void Copter::update_notify()
{
    notify.update();
}

