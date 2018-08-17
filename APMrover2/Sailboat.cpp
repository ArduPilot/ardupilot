#include "Rover.h"

// returns true if vehicle is a Sailboat
bool Rover::is_Sailboat() const
{
    return ((enum frame_class)g2.frame_class.get() == FRAME_SAILBOAT);
}



