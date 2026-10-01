/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Access to the pioasm-generated RP2350 PIO programs in this directory.
 * The .pio.h files are generated from the .pio sources by
 * Tools/scripts/rp2350_pioasm.py; edit the sources, not the headers.
 */
#pragma once

#include <stdint.h>

// the generated headers wrap their pico-sdk parts in this; only the
// instruction arrays and the wrap and label defines are used
#ifndef PICO_NO_HARDWARE
#define PICO_NO_HARDWARE 1
#endif

/*
  copy a program into instruction memory at offset. pioasm emits jump
  targets relative to the start of the program, so they are relocated the
  way pio_add_program() does it
 */
static inline void rp2350_pio_load(volatile uint32_t *instr_mem, uint8_t offset,
                                   const uint16_t *program, uint8_t length)
{
    for (uint8_t i = 0; i < length; i++) {
        uint16_t instr = program[i];
        if ((instr & 0xE000U) == 0U) {
            // JMP: the target address is in bits 4:0
            instr = (instr & ~0x1FU) | ((instr + offset) & 0x1FU);
        }
        instr_mem[offset + i] = instr;
    }
}
