@{from dronecan_dsdlc_helpers import *}@
@{indent = 0}@{ind = '    '*indent}@
#include <dronecan_msgs.h>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <bits/stdc++.h>
#include <stdlib.h>
#include <canard.h>
#include <test_helpers.h>

extern "C" float random_float16_val();
extern "C" float random_float16_val();
extern "C" uint32_t random_range_unsigned_val(uint32_t min, uint32_t max);
extern "C" int64_t random_bitlen_signed_val(uint8_t bitlen);
extern "C" uint64_t random_bitlen_unsigned_val(uint8_t bitlen);

// Random Value Functions

// Generate Random half precision float Value
float random_float16_val()
{
    float r = random_float_val();
    return canardConvertFloat16ToNativeFloat(canardConvertNativeFloatToFloat16(r));
}

// Generate a random float value
float random_float_val()
{
    return -512.0 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(1024.0)));
}

// Generate a random unsigned integer value from range, inclusive
uint32_t random_range_unsigned_val(uint32_t min, uint32_t max)
{
    return min + static_cast <uint32_t> (rand()) % (static_cast <uint32_t> (max - min + 1));
}


// get unsigned random number by bit length
uint64_t random_bitlen_unsigned_val(uint8_t bitlen) {
    uint64_t random_number = 0;
    for (int i = 0; i < bitlen; i++) {
        random_number = random_number << 1;
        random_number = random_number | (rand() % 2);
    }
    return random_number;
}

// get signed random number by bit length
int64_t random_bitlen_signed_val(uint8_t bitlen) {
    int64_t random_number = 0;
    for (uint8_t i = 0; i < (bitlen-1); i++) {
        random_number = random_number << 1;
        random_number = random_number | (rand() % 2);
    }
    // randomise sign
    if (rand() % 2 == 0) {
        random_number *= -1;
    }
    return random_number;
}

// Convert Binary data to Hex string
std::string hex_string(const std::vector<uint8_t>& data) {
    std::stringstream ss;
    for (size_t i = 0; i < data.size(); ++i) {
        ss << std::hex << std::setw(2) << std::setfill('0') << (int)data[i];
    }
    return ss.str();
}

// Convert Hex string to Binary data
std::vector<uint8_t> hex_data(const std::string& str) {
    std::vector<uint8_t> data;
    for (size_t i = 0; i < str.size(); i += 2) {
        std::string byte = str.substr(i, 2);
        data.push_back(static_cast<uint8_t>(std::stoul(byte, nullptr, 16)));
    }
    return data;
}

int main() {
@{indent += 1}@{ind = '    '*indent}@
    // seed random number generator
    srand(time(NULL));

    // Create a UAVCAN message
    @(msg_underscored_name) _msg = sample_@(msg_underscored_name)_msg();

    uint8_t buffer[@(msg_define_name.upper())_MAX_SIZE];

    // encode the message
    uint32_t data_len = @(msg_underscored_name)_encode(&_msg, buffer);

    // Convert Binary data to Hex string
    std::string hex_str = hex_string(std::vector<uint8_t>(buffer, buffer + data_len));
    std::cout << hex_str << std::endl;
    // get hex string from stdin
    std::string hex_str_input;
    std::getline(std::cin, hex_str_input);
    // Convert Hex string to Binary data
    std::vector<uint8_t> data = hex_data(hex_str_input);

    // decode the message
    @(msg_underscored_name) decoded_msg;
    CanardRxTransfer rx_transfer = {
        .timestamp_usec = 0,
        .payload_head = data.data(),
        .payload_middle = NULL,
        .payload_tail = NULL,
        .payload_len = (uint16_t)data.size(),
        .data_type_id = @(msg_define_name.upper())_ID,
        .transfer_type = 0,
        .transfer_id = 0,
        .priority = 0,
        .source_node_id = 125,
    };
    if (@(msg_underscored_name)_decode(&rx_transfer, &decoded_msg)) {
        std::cout << "decode failed" << std::endl;
        return 1;
    }

    // compare decoded message with original message
    data_len = @(msg_underscored_name)_encode(&decoded_msg, buffer);

    // Convert Binary data to Hex string
    std::string reencoded_hex_str = hex_string(std::vector<uint8_t>(buffer, buffer + data_len));

    // Compare the two strings
    if (hex_str.compare(reencoded_hex_str) == 0) {
        std::cout << "Messages are equal" << std::endl;
    } else {
       std::cout << "Messages are not equal" << std::endl;
       return 1;
    }

    return 0;
}
