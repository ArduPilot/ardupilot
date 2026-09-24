#pragma once

#include <stdint.h>

namespace AP_DDS {

// Preserve the legacy AD prefix for small IDs without discarding high bits.
// Fix zero and key_base in place so nonzero system IDs never use the reserved
// zero client key. All other keys form pairs under XOR, making this bijective.
constexpr uint32_t key_base = 0xAD000000;
constexpr uint32_t client_key_from_sysid(uint32_t sysid)
{
    return (sysid == 0 || sysid == key_base) ? sysid : sysid ^ key_base;
}

}
