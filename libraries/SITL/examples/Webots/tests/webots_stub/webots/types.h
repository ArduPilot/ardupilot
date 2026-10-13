/* Minimal stand-in for the Webots C API, used only to compile-check the
   ArduPilot controllers on a machine without Webots installed.
   It is NOT a simulator: every function is a stub. */
#ifndef WEBOTS_STUB_TYPES_H
#define WEBOTS_STUB_TYPES_H

#include <stdbool.h>
#include <stddef.h>

typedef int WbDeviceTag;
typedef void *WbNodeRef;
typedef void *WbFieldRef;

#endif
