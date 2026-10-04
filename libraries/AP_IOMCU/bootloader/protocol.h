/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */
#pragma once

#include <stdint.h>

namespace IOMCU_BL
{

constexpr uint32_t APP_BASE = 0x08001000;
constexpr uint32_t APP_SIZE = 60 * 1024;
constexpr uint32_t ERASED = 0xffffffff;

inline bool valid_app_vectors(uint32_t sp, uint32_t pc)
{
    // One image serves F100 (8 KiB RAM) and F103 (20 KiB RAM) IOMCUs.
    // This checks the combined address range, not the installed RAM size.
    return !(sp & 7) && sp > 0x20000000 && sp <= 0x20005000 &&
           (pc & 1) && pc >= APP_BASE && pc < APP_BASE + APP_SIZE;
}

// The transport and flash operations are implemented by the ChibiOS port.
int receive(uint32_t timeout_ms);
void send(uint8_t byte);
uint32_t read_word(uint32_t offset);
bool erase_application();
bool write_word(uint32_t offset, uint32_t word);

class Protocol
{
public:
    // Returns true only for a completed BOOT command.
    bool command(uint8_t opcode);
    bool active() const
    {
        return stay;
    }

private:
    uint32_t address = APP_SIZE;
    uint32_t first_word = ERASED;
    bool synced = false;
    bool crc_requested = false;
    bool stay = false;

    void send_word(uint32_t word);
    void reply(uint8_t status);
};

}
