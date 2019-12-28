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
#pragma once

#include "AP_EFI.h"
#include "AP_EFI_Backend.h"

class AP_EFI_EMUECU: public AP_EFI_Backend {
    
public:
    // Constructor
    AP_EFI_EMUECU(AP_EFI &_frontend);

    // Update the state structure
    void update() override;

private:
    AP_HAL::UARTDriver *port;

    void process_line(void);
    void send_throttle(void);

    enum class ECU_State {
        INIT = 0,
        PRIME,
        STOPPED,
        CRANK,
        START,
        RUNNING,
    };

    struct {
        ECU_State state;
        float throttle_in;
        float throttle_out;
        uint16_t rpm;
        int16_t cht;   // -? - ~100
        int16_t iat;   // -? - 80
        uint32_t egt;  // 0 - 1024
        int16_t ecut;  // -? -
        uint32_t baro; // 0 - ~101300pa
        uint16_t humidity;
        // internal vars
        float pt_c;
        uint8_t starts;
        uint16_t engine_stop_ms;
        uint16_t engine_start_ms;
        uint16_t engine_prime_ms;
        // outputs
        uint16_t inj_ticks;
        uint16_t thr_in;
        uint16_t pwm0_out;
        uint16_t pwm1_out;
    } status;

    enum class DataType {
        FLOAT = 0,
        UINT8,
        UINT16,
        INT16,
        UINT32,
    };

    // table to aid parsing of JSON sensor data
    const struct keytable {
        const char *key;
        void *ptr;
        DataType type;
    } keytable[8] {
                  { "rpm",    &status.rpm, DataType::UINT16 },
                  { "cht",    &status.cht, DataType::INT16 },
                  { "egt",    &status.egt, DataType::UINT32 },
                  { "iat",    &status.iat, DataType::INT16 },
                  { "throttle_out", &status.throttle_out, DataType::FLOAT },
                  { "pt_c", &status.pt_c, DataType::FLOAT },
                  { "baro", &status.baro, DataType::UINT32 },
                  { "inj_ticks", &status.inj_ticks, DataType::UINT16 },
    };

    char linebuf[1200];
    uint16_t line_len;
    char last_log_msg[64];
    bool done_config_log;
};
