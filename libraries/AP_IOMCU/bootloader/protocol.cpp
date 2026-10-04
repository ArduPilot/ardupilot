/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */
#include "protocol.h"

namespace IOMCU_BL
{

static constexpr uint8_t EOC = 0x20;
static constexpr uint8_t OK = 0x10;
static constexpr uint8_t FAILED = 0x11;
static constexpr uint8_t INVALID = 0x13;

void Protocol::send_word(uint32_t word)
{
    for (uint8_t i = 0; i < 4; i++) {
        send(word & 0xff);
        word >>= 8;
    }
}

void Protocol::reply(uint8_t status)
{
    send(0x12); // INSYNC
    send(status);
}

bool Protocol::command(uint8_t opcode)
{
    uint32_t data[63]; // accepts the FMU uploader's 248-byte packets
    int arg;
    switch (opcode) {
    case 0x21: // GET_SYNC
        if (receive(2) != EOC) {
            goto invalid;
        }
        synced = true;
        break;

    case 0x22: // GET_DEVICE
        arg = receive(1000);
        if (receive(2) != EOC) {
            goto invalid;
        }
        switch (arg) {
        case 1: send_word(5); break; // legacy protocol revision
        case 2: send_word(10); break; // PX4IO board ID, independent of MCU
        case 3: send_word(0); break;
        case 4: send_word(APP_SIZE); break;
        case 5:
            for (uint32_t offset = 28; offset <= 40; offset += 4) {
                send_word(read_word(offset));
            }
            break;
        default: goto invalid;
        }
        break;

    case 0x23: // CHIP_ERASE
        if (receive(2) != EOC || !synced) {
            goto invalid;
        }
        // Discard the previous upload even when a second erase fails.
        address = APP_SIZE;
        first_word = ERASED;
        crc_requested = false;
        if (!erase_application()) {
            goto failed;
        }
        address = 0;
        break;

    case 0x27: // PROG_MULTI
        arg = receive(50);
        if (arg <= 0 || arg > int(sizeof(data)) || (arg & 3) || uint32_t(arg) > APP_SIZE - address) {
            goto invalid;
        }
        for (int i = 0; i < arg; i++) {
            const int c = receive(1000);
            if (c < 0) {
                goto invalid;
            }
            reinterpret_cast<uint8_t *>(data)[i] = c;
        }
        if (receive(200) != EOC) {
            goto invalid;
        }
        crc_requested = false;
        if (address == 0) {
            first_word = data[0];
            data[0] = ERASED;
        }
        for (int i = 0; i < arg / 4; i++) {
            if (!write_word(address, data[i])) {
                goto failed;
            }
            address += 4;
        }
        break;

    case 0x29: { // GET_CRC: reflected CRC32, seed zero, no final XOR
        if (receive(2) != EOC) {
            goto invalid;
        }
        uint32_t crc = 0;
        for (uint32_t offset = 0; offset < APP_SIZE; offset += 4) {
            uint32_t word = offset == 0 && first_word != ERASED ? first_word : read_word(offset);
            for (uint8_t byte = 0; byte < 4; byte++) {
                crc ^= word & 0xff;
                word >>= 8;
                for (uint8_t bit = 0; bit < 8; bit++) {
                    crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320U : 0);
                }
            }
        }
        send_word(crc);
        crc_requested = true;
        break;
    }

    case 0x30: // BOOT: the FMU deliberately delays EOC by 200 ms
        if (receive(1000) != EOC) {
            goto invalid;
        }
        if (first_word != ERASED) {
            if (!synced || !crc_requested || address < 8) {
                goto invalid;
            }
            if (!write_word(0, first_word)) {
                goto failed;
            }
            first_word = ERASED;
        }
        reply(OK);
        return true;

    default:
        // The uploader sends zeros to drain an incomplete packet, and an
        // extra EOC after GET_DEVICE. Neither gets a response.
        return false;
    }
    stay = true;
    reply(OK);
    return false;

invalid:
    synced = false;
    reply(INVALID);
    return false;

failed:
    // A failed upload needs a new erase before it can be retried.
    address = APP_SIZE;
    first_word = ERASED;
    crc_requested = false;
    reply(FAILED);
    return false;
}

}
