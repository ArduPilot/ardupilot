/*
  wrapper around evtimer.c so we only build when events are
  enabled. This prevents a complex check in the ChibiOS mk layer
 */
#include <hal.h>

#if CH_CFG_USE_EVENTS
// this include relies on -I for modules/ChibiOS/os/various/cpp_wrappers
#include <../evtimer.c>
#endif

