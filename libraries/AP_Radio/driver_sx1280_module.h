// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "driver_sx1280.h"
#if defined(AP_MODULE_SX1280_ENABLED) && AP_MODULE_SX1280_ENABLED

// Native radio event/action ABI. No protocol parsing or RF scheduling policy.
// State is opaque to the host; calls are serialized and pinned until return.
namespace SX1280Module
{
constexpr uint8_t ERROR_DETACHED = 255;
using Entry = uint32_t (*)(uint8_t *, const uint8_t *, uint8_t *);
bool prepare(void *owner, const uint8_t settings[12]);
bool install(void *owner, Entry entry, void *rw);
void remove(void *owner);
void counters(uint32_t words[5]);
void update();
void report(uint8_t output[256]);
bool snapshot(void *owner, uint32_t offset, uint8_t output[16]);
}
#endif
