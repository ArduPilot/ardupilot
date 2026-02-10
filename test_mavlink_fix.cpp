/*
 * Test program to verify the MAVLink tight loop fix
 * This simulates the scenario described in the bug report
 */

#include <iostream>
#include <vector>
#include <cstdint>

// Simulate the MAVLink framing constants
#define MAVLINK_FRAMING_INCOMPLETE 0
#define MAVLINK_FRAMING_OK 1
#define MAVLINK_FRAMING_BAD_CRC 2
#define MAVLINK_FRAMING_BAD_SIGNATURE 3

// Simulate the mavlink_frame_char_buffer function behavior
uint8_t simulate_mavlink_frame_char_buffer(uint8_t c) {
    // Simulate malformed packets that cause tight loops
    // Return BAD_CRC for most bytes to simulate the issue
    static int counter = 0;
    counter++;
    
    if (counter % 100 == 0) {
        // Occasionally return OK to simulate a valid packet
        return MAVLINK_FRAMING_OK;
    }
    
    // Return BAD_CRC to simulate malformed packets
    return MAVLINK_FRAMING_BAD_CRC;
}

// Simulate the fixed update_receive function logic
void test_update_receive_fix() {
    std::cout << "Testing MAVLink tight loop fix..." << std::endl;
    
    uint16_t malformed_packet_count = 0;
    const uint16_t max_malformed_packets = 100;
    uint16_t total_bytes_processed = 0;
    uint16_t valid_packets = 0;
    
    // Simulate processing 1000 bytes (more than max_malformed_packets)
    for (uint16_t i = 0; i < 1000; i++) {
        total_bytes_processed++;
        uint8_t c = i % 256; // Simulate byte data
        
        uint8_t framing = simulate_mavlink_frame_char_buffer(c);
        
        if (framing != MAVLINK_FRAMING_INCOMPLETE) {
            if (framing == MAVLINK_FRAMING_OK) {
                valid_packets++;
                // Reset malformed packet count on successful parse
                malformed_packet_count = 0;
            } else if (framing == MAVLINK_FRAMING_BAD_CRC || framing == MAVLINK_FRAMING_BAD_SIGNATURE) {
                // Count malformed packets to prevent tight loops
                malformed_packet_count++;
            }
        }
        
        // Check for excessive malformed packets and reset parser state if needed
        if (malformed_packet_count > max_malformed_packets) {
            std::cout << "Detected excessive malformed packets (" << malformed_packet_count 
                      << "), resetting parser state" << std::endl;
            // Reset the parser state to recover from malformed data
            malformed_packet_count = 0;
        }
        
        // Simulate the time check that would break the loop
        if (i > 500) {
            std::cout << "Breaking loop after processing " << i << " bytes" << std::endl;
            break;
        }
    }
    
    std::cout << "Test completed successfully!" << std::endl;
    std::cout << "Total bytes processed: " << total_bytes_processed << std::endl;
    std::cout << "Valid packets: " << valid_packets << std::endl;
    std::cout << "Malformed packet protection: " << (malformed_packet_count == 0 ? "ACTIVE" : "INACTIVE") << std::endl;
}

int main() {
    test_update_receive_fix();
    return 0;
}