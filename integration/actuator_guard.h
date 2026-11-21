// SPDX-License-Identifier: GPL-3.0-or-later
// Software-only protocol core for a future ESP32 output path. No GPIO or serial I/O.
#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

class ActuatorGuard {
public:
    static constexpr unsigned MOTOR_COUNT = 8;
    static constexpr uint16_t NEUTRAL_US = 1500;
    static constexpr uint16_t MIN_US = 1300;
    static constexpr uint16_t MAX_US = 1700;
    static constexpr uint32_t TIMEOUT_MS = 250;

    enum class State : uint8_t { Disarmed, Armed, Fault };

    explicit ActuatorGuard(uint32_t session) : session_(session) {
        neutralize();
    }

    State state() const { return state_; }
    const uint16_t *outputs() const { return outputs_; }
    uint32_t last_sequence() const { return last_sequence_; }

    void tick(uint32_t now_ms) {
        if (state_ == State::Armed && uint32_t(now_ms - last_frame_ms_) > TIMEOUT_MS) {
            fault();
        }
    }

    // Caller supplies a NUL-terminated complete ASCII line, without CR/LF:
    // A1,ARM|PWM|DISARM|RESET,session_hex,sequence[,eight_pwm],crc16_hex
    // CRC-16/CCITT-FALSE covers bytes preceding the final comma.
    bool accept(const char *line, uint32_t now_ms) {
        tick(now_ms);
        if (line == nullptr) {
            fault();
            return false;
        }
        size_t length = 0;
        while (length < 128 && line[length] != '\0') ++length;
        if (length < 13 || length >= 128 || strchr(line, '\r') || strchr(line, '\n')) {
            fault();
            return false;
        }
        const char *last_comma = strrchr(line, ',');
        uint16_t received_crc;
        if (last_comma == nullptr || strlen(last_comma + 1) != 4 ||
                !hex16(last_comma + 1, received_crc) ||
                crc16(line, size_t(last_comma - line)) != received_crc) {
            fault();
            return false;
        }

        char body[128];
        const size_t body_length = size_t(last_comma - line);
        if (body_length == 0 || line[0] == ',' || last_comma[-1] == ',' ||
                strstr(line, ",,") != nullptr) {
            fault();
            return false;
        }
        memcpy(body, line, body_length);
        body[body_length] = '\0';
        char *parts[13] = {};
        unsigned count = 0;
        char *save = nullptr;
        for (char *part = strtok_r(body, ",", &save); part != nullptr;
             part = strtok_r(nullptr, ",", &save)) {
            if (count >= 13 || *part == '\0') {
                fault();
                return false;
            }
            parts[count++] = part;
        }
        if (count < 4 || strcmp(parts[0], "A1") != 0) {
            fault();
            return false;
        }
        uint32_t session, sequence;
        if (!hex32(parts[2], session) || session != session_ ||
                !decimal32(parts[3], sequence) || sequence <= last_sequence_) {
            fault();
            return false;
        }

        if (strcmp(parts[1], "ARM") == 0 && count == 4 && state_ == State::Disarmed) {
            state_ = State::Armed;
            last_sequence_ = sequence;
            last_frame_ms_ = now_ms;
            return true;
        }
        if (strcmp(parts[1], "DISARM") == 0 && count == 4) {
            neutralize();
            if (state_ != State::Fault) {
                state_ = State::Disarmed;
            }
            last_sequence_ = sequence;
            return true;
        }
        if (strcmp(parts[1], "RESET") == 0 && count == 4 && state_ == State::Fault) {
            neutralize();
            state_ = State::Disarmed;
            last_sequence_ = sequence;
            return true;
        }
        if (strcmp(parts[1], "PWM") == 0 && count == 12 && state_ == State::Armed) {
            uint16_t next[MOTOR_COUNT];
            for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
                uint32_t value;
                if (!decimal32(parts[i + 4], value) || value < MIN_US || value > MAX_US) {
                    fault();
                    return false;
                }
                next[i] = uint16_t(value);
            }
            memcpy(outputs_, next, sizeof(outputs_));
            last_sequence_ = sequence;
            last_frame_ms_ = now_ms;
            return true;
        }
        fault();
        return false;
    }

    static uint16_t crc16(const char *data, size_t length) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < length; ++i) {
            crc ^= uint16_t(uint8_t(data[i])) << 8;
            for (unsigned bit = 0; bit < 8; ++bit) {
                crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
            }
        }
        return crc;
    }

private:
    static bool decimal32(const char *text, uint32_t &value) {
        if (*text == '\0') return false;
        for (const char *p = text; *p; ++p) {
            if (*p < '0' || *p > '9') return false;
        }
        char *end;
        const unsigned long parsed = strtoul(text, &end, 10);
        if (*end != '\0' || parsed > 0xFFFFFFFFUL) return false;
        value = uint32_t(parsed);
        return true;
    }

    static bool hex32(const char *text, uint32_t &value) {
        if (strlen(text) != 8) return false;
        for (const char *p = text; *p; ++p) {
            if (!((*p >= '0' && *p <= '9') || (*p >= 'a' && *p <= 'f') ||
                  (*p >= 'A' && *p <= 'F'))) return false;
        }
        char *end;
        const unsigned long parsed = strtoul(text, &end, 16);
        if (*end != '\0' || parsed > 0xFFFFFFFFUL) return false;
        value = uint32_t(parsed);
        return true;
    }

    static bool hex16(const char *text, uint16_t &value) {
        for (const char *p = text; *p; ++p) {
            if (!((*p >= '0' && *p <= '9') || (*p >= 'a' && *p <= 'f') ||
                  (*p >= 'A' && *p <= 'F'))) return false;
        }
        char *end;
        const unsigned long parsed = strtoul(text, &end, 16);
        if (*end != '\0' || parsed > 0xFFFFUL) return false;
        value = uint16_t(parsed);
        return true;
    }

    void neutralize() {
        for (unsigned i = 0; i < MOTOR_COUNT; ++i) outputs_[i] = NEUTRAL_US;
    }

    void fault() {
        neutralize();
        state_ = State::Fault;
    }

    const uint32_t session_;
    State state_ = State::Disarmed;
    uint16_t outputs_[MOTOR_COUNT];
    uint32_t last_sequence_ = 0;
    uint32_t last_frame_ms_ = 0;
};
