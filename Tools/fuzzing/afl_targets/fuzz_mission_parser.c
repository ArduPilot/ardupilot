/*
 * AFL++ Fuzzing Target for Mission File Parser
 * 
 * Usage:
 *   export CC=afl-clang-fast
 *   export CXX=afl-clang-fast++
 *   ./waf configure --board sitl
 *   ./waf fuzz_mission_parser
 *   
 *   afl-fuzz -i seeds/mission -o output/mission ./build/sitl/bin/fuzz_mission_parser @@
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <AP_Mission/AP_Mission.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Maximum mission file size for fuzzing (1MB)
#define MAX_MISSION_SIZE (1024 * 1024)

/**
 * Parse mission file data
 * 
 * @param data Input mission file data (WP file format)
 * @param size Size of input data
 * @return 0 on success, -1 on parse error
 */
static int parse_mission_data(const uint8_t *data, size_t size)
{
    if (data == nullptr || size == 0) {
        return -1;
    }
    
    // Limit size to prevent DoS
    if (size > MAX_MISSION_SIZE) {
        size = MAX_MISSION_SIZE;
    }
    
    // Create null-terminated string for parsing
    char *mission_str = (char *)malloc(size + 1);
    if (mission_str == nullptr) {
        return -1;
    }
    
    memcpy(mission_str, data, size);
    mission_str[size] = '\0';
    
    // Parse mission file
    // Format: QGC WPL 110
    //         <seq> <current> <frame> <command> <param1> <param2> <param3> <param4> <x> <y> <z> <autocontinue>
    
    char *line = strtok(mission_str, "\n");
    int line_num = 0;
    int waypoint_count = 0;
    
    while (line != nullptr && line_num < 1000) {
        line_num++;
        
        // Skip header line
        if (line_num == 1) {
            if (strncmp(line, "QGC WPL", 7) != 0) {
                // Invalid header, but don't crash
                free(mission_str);
                return -1;
            }
            line = strtok(nullptr, "\n");
            continue;
        }
        
        // Skip empty lines
        if (strlen(line) < 10) {
            line = strtok(nullptr, "\n");
            continue;
        }
        
        // Parse waypoint fields
        int seq, current, frame, command;
        float p1, p2, p3, p4, x, y, z;
        int autocontinue;
        
        int parsed = sscanf(line, "%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d",
                           &seq, &current, &frame, &command,
                           &p1, &p2, &p3, &p4, &x, &y, &z, &autocontinue);
        
        if (parsed >= 12) {
            waypoint_count++;
            
            // Validate ranges
            if (seq < 0 || seq > 10000) {
                free(mission_str);
                return -1;
            }
            
            if (frame < 0 || frame > 5) {
                free(mission_str);
                return -1;
            }
            
            // Check for NaN/Inf in coordinates
            if (isnan(x) || isnan(y) || isnan(z) ||
                isinf(x) || isinf(y) || isinf(z)) {
                free(mission_str);
                return -1;
            }
            
            // Check for extreme values
            if (fabsf(x) > 90.0f || fabsf(y) > 180.0f) {
                free(mission_str);
                return -1;
            }
        }
        
        line = strtok(nullptr, "\n");
    }
    
    free(mission_str);
    
    // Must have at least one waypoint
    if (waypoint_count == 0) {
        return -1;
    }
    
    return 0;
}

/**
 * AFL++ fuzzing entry point
 */
extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    // Parse mission data
    parse_mission_data(data, size);
    
    return 0;
}

/**
 * Main function for non-AFL testing
 */
int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <mission_file>\n", argv[0]);
        return 1;
    }
    
    FILE *f = fopen(argv[1], "rb");
    if (!f) {
        fprintf(stderr, "Error: Cannot open %s\n", argv[1]);
        return 1;
    }
    
    fseek(f, 0, SEEK_END);
    size_t size = ftell(f);
    fseek(f, 0, SEEK_SET);
    
    uint8_t *data = (uint8_t *)malloc(size);
    if (!data) {
        fclose(f);
        return 1;
    }
    
    fread(data, 1, size, f);
    fclose(f);
    
    int result = parse_mission_data(data, size);
    
    free(data);
    
    return result;
}
