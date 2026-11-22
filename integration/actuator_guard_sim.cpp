// SPDX-License-Identifier: GPL-3.0-or-later
// Stdin/stdout test harness for ActuatorGuard. No transport or GPIO access.
#include "actuator_guard.h"

#include <cstdio>
#include <cstring>

int main() {
    ActuatorGuard guard(0x1234ABCD);
    char line[256];
    while (fgets(line, sizeof(line), stdin) != nullptr) {
        unsigned long now_ms;
        char command[160];
        char extra;
        if (sscanf(line, "%lu %159s %c", &now_ms, command, &extra) != 2 || now_ms > 0xFFFFFFFFUL) {
            return 2;
        }
        const bool accepted = strcmp(command, "TICK") == 0 ?
            (guard.tick(static_cast<uint32_t>(now_ms)), true) :
            guard.accept(command, static_cast<uint32_t>(now_ms));
        const char *state = guard.state() == ActuatorGuard::State::Armed ? "armed" :
            guard.state() == ActuatorGuard::State::Fault ? "fault" : "disarmed";
        printf("%u %s", accepted ? 1 : 0, state);
        for (unsigned i = 0; i < ActuatorGuard::MOTOR_COUNT; ++i) {
            printf(" %u", guard.outputs()[i]);
        }
        putchar('\n');
    }
    return ferror(stdin) ? 2 : 0;
}
