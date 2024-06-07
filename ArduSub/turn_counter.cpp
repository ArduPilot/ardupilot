// Code by Rustom Jehangir: rusty@bluerobotics.com

#include "Sub.h"

// Count total vehicle turns to avoid tangling tether
void Sub::update_turn_counter()
{
    // Determine state
    // 0: 0-90 deg, 1: 90-180 deg, 2: -180--90 deg, 3: -90--0 deg
    uint8_t turn_state;
    if (ahrs.get_yaw() >= 0.0f && ahrs.get_yaw() < radians(90)) {
        turn_state = 0;
    } else if (ahrs.get_yaw() > radians(90)) {
        turn_state = 1;
    } else if (ahrs.get_yaw() < -radians(90)) {
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
    last_turn_state = turn_state;
}
