#include "Tracker.h"

#include "RC_Channel_Tracker.h"

// defining these two macros and including the RC_Channels_VarInfo
// header defines the parameter information common to all vehicle
// types
#define RC_CHANNELS_SUBCLASS RC_Channels_Tracker
#define RC_CHANNEL_SUBCLASS RC_Channel_Tracker

#include <RC_Channel/RC_Channels_VarInfo.h>
