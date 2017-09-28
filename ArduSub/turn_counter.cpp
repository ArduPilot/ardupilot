// Code by Rustom Jehangir: rusty@bluerobotics.com

#include "Sub.h"

// Count total vehicle turns to avoid tangling tether
void Sub::update_turn_counter()
{
    // Determine state
    // 0: 0-90 deg, 1: 90-180 deg, 2: -180--90 deg, 3: -90--0 deg
    uint8_t turn_state;
    if (ahrs.yaw >= 0.0f && ahrs.yaw < radians(90)) {
        turn_state = 0;
    } else if (ahrs.yaw > radians(90)) {
        turn_state = 1;
    } else if (ahrs.yaw < -radians(90)) {
        turn_state = 2;
    } else {
        turn_state = 3;
    }

    // If yaw went from negative to positive (right turn)
    switch (last_turn_state) {
    case 0:
        if (turn_state == 1) {
            quarter_turn_count++;
        }
        if (turn_state == 3) {
            quarter_turn_count--;
        }
        break;
    case 1:
        if (turn_state == 2) {
            quarter_turn_count++;
        }
        if (turn_state == 0) {
            quarter_turn_count--;
        }
        break;
    case 2:
        if (turn_state == 3) {
            quarter_turn_count++;
        }
        if (turn_state == 1) {
            quarter_turn_count--;
        }
        break;
    case 3:
        if (turn_state == 0) {
            quarter_turn_count++;
        }
        if (turn_state == 2) {
            quarter_turn_count--;
        }
        break;
    }
    static int32_t last_turn_count_printed;
    static uint32_t last_turn_announce_ms = 0;
    uint32_t tnow = AP_HAL::millis();
    if (quarter_turn_count/4 != last_turn_count_printed  && tnow > last_turn_announce_ms + 2000) {
        last_turn_announce_ms = tnow;
        gcs().send_text(MAV_SEVERITY_INFO,"Tether is turned %i turns %s",int32_t(abs(quarter_turn_count)/4),(quarter_turn_count>0)?"to the right":"to the left");
        last_turn_count_printed = quarter_turn_count/4;
    }
    last_turn_state = turn_state;
}
