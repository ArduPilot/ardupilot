// Staged ArduSub ESP32 endpoint. GPIO output is intentionally absent until
// motor wiring, direction and emergency power isolation are verified on hardware.
#include "actuator_guard.h"
#ifndef ATAL_HOST_TEST
#include <esp_system.h>
#endif

static ActuatorGuard guard(0);
static char line[128];
static size_t used;
static bool overflow;

static void report_new_fault(ActuatorGuard::State before) {
    if (before != ActuatorGuard::State::Fault &&
            guard.state() == ActuatorGuard::State::Fault) {
        Serial.printf("A1,FAULT\n");
    }
}

static void new_session(const char *challenge) {
    uint32_t session = esp_random();
    if (session == 0) session = 1;
    guard.restart_session(session);
    Serial.printf("A1,SESSION,%08lX,%s\n", (unsigned long)session, challenge);
}

void setup() {
    Serial.begin(115200);
    guard.restart_session(esp_random() | 1U);
    // No LEDC attachment or ESC pin writes in this staged firmware.
}

void loop() {
    const ActuatorGuard::State before_tick = guard.state();
    guard.tick(millis());
    report_new_fault(before_tick);
    while (Serial.available()) {
        const char c = char(Serial.read());
        if (c == '\n') {
            const ActuatorGuard::State before_frame = guard.state();
            if (overflow || used == 0) {
                guard.accept(nullptr, millis());
            } else {
                if (used && line[used - 1] == '\r') --used;
                line[used] = '\0';
                // The only unguarded command can only force neutral and rotate
                // the session. Challenge echo rejects stale serial banners.
                if (used == 17 && strncmp(line, "A1,HELLO,", 9) == 0) {
                    bool valid = true;
                    for (size_t i = 9; i < 17; ++i) {
                        const char h = line[i];
                        valid &= (h >= '0' && h <= '9') ||
                                 (h >= 'A' && h <= 'F');
                    }
                    if (valid) new_session(line + 9);
                    else guard.accept(nullptr, millis());
                } else {
                    guard.accept(line, millis());
                }
            }
            report_new_fault(before_frame);
            used = 0;
            overflow = false;
        } else if (!overflow) {
            if (used < sizeof(line) - 1) line[used++] = c;
            else overflow = true;
        }
    }
}
