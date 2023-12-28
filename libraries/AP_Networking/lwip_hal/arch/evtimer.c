/*
  wrapper around evtimer.c so we only build when events are
  enabled. This prevents a complex check in the ChibiOS mk layer
 */
#include <hal.h>

#if CH_CFG_USE_EVENTS
#include "../../modules/ChibiOS/os/various/evtimer.c"
#endif

