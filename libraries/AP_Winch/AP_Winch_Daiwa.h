/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
   The Daiwa winch is produced by a Japanese company called Okaya.
   There are two PWM controls supported:
     - the rate control for releasing (high PWM) or retracting (low PWM) the line
     - the clutch control has three settings:
         - released (high PWM) lets the winch spin freely
         - engaged soft (middle PWM) allows the rate control to work but it may slip
           if too much tension is required.  This driver does not use this setting.
         - engaged hard (low PWM) allows the rate control to work regardless of tension.
   A telemetry output from the winch is connected to the autopilot and provides
   the amount of line released, tension, clutch setting, etc.
*/

#pragma once

#include <AP_Winch/AP_Winch_Backend.h>

#if AP_WINCH_DAIWA_ENABLED

class AP_Winch_Daiwa : public AP_Winch_Backend {
public:

    using AP_Winch_Backend::AP_Winch_Backend;

    // true if winch is healthy
    bool healthy() const override;

    // initialise the winch
    void init() override;

    // read telemetry from the winch and output controls
    void update() override;

    // returns current length of line deployed
    float get_current_length() const override { return latest.line_length; }

    // send status to ground station
    void send_status(const GCS_MAVLINK &channel) override;

#if HAL_LOGGING_ENABLED
    // write log
    void write_log() override;
#endif

private:

    // read incoming data from winch and update intermediate and latest structures
    void read_data_from_winch();

    // update pwm outputs to control winch
    void control_winch();

    // returns the rate which may be modified to unstick the winch
    // if the winch stops, the rate is temporarily set to zero
    // now_ms should be set to the current system time
    // rate should be the rate used to calculate the final PWM output to the winch
    float get_stuck_protected_rate(uint32_t now_ms, float rate);

    static const uint8_t buff_len_max = 20; // buffer maximum length
    static const int16_t output_dz = 100;   // output deadzone in scale of -1000 to +1000
    const float line_length_correction_factor = 0.003333f;  // convert winch counter to meters

    AP_HAL::UARTDriver *uart;
    char buff[buff_len_max];    // buffer holding latest data from winch
    uint8_t buff_len;           // number of bytes in buff

    // winch data
    // latest holds most recent complete data received
    // intermediate holds partial results currently being processed
    struct WinchData {
        uint32_t time_ms;               // winch system time in milliseconds
        float line_length;              // length of line released in meters
        uint16_t tension_uncorrected;   // uncorrected tension in grams (0 to 1024)
        uint16_t tension_corrected;     // corrected tension in grams (0 to 1024)
        bool thread_end;                // true if end of thread has been detected
        uint8_t moving;                 // 0:stopped, 1:retracting line, 2:extending line, 3:clutch engaged, 4:zero reset
        uint8_t clutch;                 // 0:clutch off, 1:clutch engaged weakly, 2:clutch engaged strongly, motor can spin freely
        uint8_t speed_pct;              // speed motor is moving as a percentage
        float voltage;                  // battery voltage (in voltes)
        float current;                  // current draw (in amps)
        float pcb_temp;                 // PCB temp in C
        float motor_temp;               // motor temp in C
    } latest, intermediate;
    uint32_t data_update_ms;            // system time that latest was last updated
    uint32_t control_update_ms;         // last time control_winch was called

    // parsing state
    enum class ParseState : uint8_t {
        WAITING_FOR_TIME = 0,
        WAITING_FOR_SPOOL,
        WAITING_FOR_TENSION1,
        WAITING_FOR_TENSION2,
        WAITING_FOR_THREAD_END,
        WAITING_FOR_MOVING,
        WAITING_FOR_CLUTCH,
        WAITING_FOR_SPEED,
        WAITING_FOR_VOLTAGE,
        WAITING_FOR_CURRENT,
        WAITING_FOR_PCB_TEMP,
        WAITING_FOR_MOTOR_TEMP
    } parse_state;

    // update user with changes to winch state via send text messages
    static const char* send_text_prefix;// send text prefix string to reduce flash cost
    void update_user();
    struct {
        uint32_t last_ms;               // system time of last update to user
        bool healthy;                   // latest reported health
        float line_length;
        bool thread_end;                // true if end of thread has been detected
        uint8_t moving;                 // 0:stopped, 1:retracting line, 2:extending line, 3:clutch engaged, 4:zero reset
        uint8_t clutch;                 // 0:clutch off, 1:clutch engaged weakly, 2:clutch engaged strongly, motor can spin freely
    } user_update;

    // stuck protection
    struct {
        uint32_t last_update_ms;        // system time that stuck protection was last called
        uint32_t stuck_start_ms;        // system time that winch became stuck (0 if not stuck)
        bool user_notified;             // true if user has been notified that winch is stuck
    } stuck_protection;
};

#endif  // AP_WINCH_DAIWA_ENABLED
