// Standalone protocol checks: compile with AP_Baro_ATAL_Depth_Parser.cpp.
#include <AP_Baro/AP_Baro_ATAL_Depth_Parser.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>

namespace {
bool feed(AP_Baro_ATAL_Depth_Parser &parser, const char *line,
          AP_Baro_ATAL_Depth_Parser::Sample &sample)
{
    bool accepted = false;
    for (size_t i = 0; i < strlen(line); i++) {
        accepted |= parser.feed(uint8_t(line[i]), sample);
    }
    return accepted;
}

bool near(float value, float expected)
{
    return fabsf(value - expected) < 0.01f;
}
}

int main()
{
    AP_Baro_ATAL_Depth_Parser parser;
    AP_Baro_ATAL_Depth_Parser::Sample sample {};
    if (feed(parser, ">> MS5837 initialized\n", sample) ||
        !feed(parser, "DEPTH:1025.50,23.40,1.2500\r\n", sample) ||
        !near(sample.pressure_pa, 102550.0f) ||
        !near(sample.temperature_c, 23.4f) ||
        !near(sample.diagnostic_depth_m, 1.25f)) {
        fputs("valid pressure conversion failed\n", stderr);
        return 1;
    }
    for (const char *line : {
             "DEPTH:0,20,0\n", "DEPTH:30001,20,0\n",
             "DEPTH:nan,20,0\n", "DEPTH:1000,inf,0\n",
             "DEPTH:1000,20,nan\n", "DEPTH:1000,90,0\n",
             "DEPTH:1000,20\n", "DEPTH:1000,20,0,extra\n",
             "noise DEPTH:1000,20,0\n"}) {
        if (feed(parser, line, sample)) {
            fprintf(stderr, "accepted invalid line: %s", line);
            return 1;
        }
    }
    char oversized[150];
    memset(oversized, 'X', sizeof(oversized) - 2);
    oversized[sizeof(oversized) - 2] = '\n';
    oversized[sizeof(oversized) - 1] = 0;
    if (feed(parser, oversized, sample) ||
        feed(parser, "DEPTH:1013.25,", sample) ||
        !feed(parser, "20.00,0.0000\n", sample) ||
        !near(sample.pressure_pa, 101325.0f)) {
        fputs("stream recovery failed\n", stderr);
        return 1;
    }
    puts("ATAL depth parser checks passed");
    return 0;
}
