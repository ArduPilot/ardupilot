#include "AP_Baro_ATAL_Depth_Parser.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

bool AP_Baro_ATAL_Depth_Parser::feed(uint8_t byte, Sample &sample)
{
    if (byte == '\n') {
        line[length] = 0;
        const bool valid = !discarding && parse(sample);
        length = 0;
        discarding = false;
        return valid;
    }
    if (byte == 0) {
        discarding = true;
    }
    if (discarding) {
        return false;
    }
    if (length >= sizeof(line) - 1) {
        length = 0;
        discarding = true;
        return false;
    }
    line[length++] = char(byte);
    return false;
}

bool AP_Baro_ATAL_Depth_Parser::parse(Sample &sample)
{
    static constexpr char prefix[] = "DEPTH:";
    if (strncmp(line, prefix, sizeof(prefix) - 1) != 0) {
        return false;
    }
    char *cursor = line + sizeof(prefix) - 1;
    char *end;
    const float pressure_mbar = strtof(cursor, &end);
    if (end == cursor || *end != ',') {
        return false;
    }
    cursor = end + 1;
    const float temperature_c = strtof(cursor, &end);
    if (end == cursor || *end != ',') {
        return false;
    }
    cursor = end + 1;
    const float depth_m = strtof(cursor, &end);
    if (end == cursor || (*end != 0 && !(end[0] == '\r' && end[1] == 0))) {
        return false;
    }
    // Bar30 is a 30 bar sensor. Reject impossible or nonfinite readings.
    if (!std::isfinite(pressure_mbar) || pressure_mbar < 300.0f || pressure_mbar > 30000.0f ||
        !std::isfinite(temperature_c) || temperature_c < -40.0f || temperature_c > 85.0f ||
        !std::isfinite(depth_m)) {
        return false;
    }
    sample = {pressure_mbar * 100.0f, temperature_c, depth_m};
    return true;
}
