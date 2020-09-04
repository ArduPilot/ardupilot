/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * Bits manipulations utilities
 */
#pragma once

#include <type_traits>
#include <stdint.h>

/**
 * Check if bit bitnumber is set in value, returned as a
 * bool. Bitnumber starts at 0 for the first bit
 * @tparam T the variable type. It should be an integer type
 * @param value the variable
 * @param bitnumber bit number to be checked
 * @return true if the bit is setted, false otherwise
 */
template <typename T>
bool BIT_IS_SET(T value, uint8_t bitnumber) noexcept {
    static_assert(std::is_integral<T>::value, "Integral required.");
    return (value & (1U<<(bitnumber))) != 0;
}

/**
 * Get low byte from 2 bytes integer
 * @tparam T the variable type. It will be checked to be uint16_t
 * @param i the variable
 * @return the low byte
 */
template <typename T>
uint8_t LOWBYTE (T const& i) noexcept {
    static_assert(std::is_same<T, uint16_t>::value, "type must be `uint16_t`");
    return static_cast<uint8_t>(i);
}

/**
 * Get high byte from 2 bytes integer
 * @tparam T the variable type. It will be checked to be uint16_t
 * @param i the variable
 * @return the high byte
 */
template <typename T>
uint8_t HIGHBYTE (T const& i) noexcept {
    static_assert(std::is_same<T, uint16_t>::value, "type must be `uint16_t`");
    return static_cast<uint8_t>(i>>8);
}

/*
  Bit manipulation
 */
/**
 * Set a bit in a variable
 * @tparam T the variable type. It will be checked to be integral type
 * @param value the variable
 * @param bitnumber the bit to set
 */
//#define BIT_SET(value, bitnumber) ((value) |= (((typeof(value))1U) << (bitnumber)))
template <typename T> void BIT_SET (T& value, uint8_t bitnumber) noexcept {
    static_assert(std::is_integral<T>::value, "Integral required.");
    ((value) |= (static_cast<T>(1U) << (bitnumber)));
}

/**
 * Clear a bit in a variable
 * @tparam T the variable type. It will be checked to be integral type
 * @param value the variable
 * @param bitnumber the bit to clear
 */
//#define BIT_CLEAR(value, bitnumber) ((value) &= ~(((typeof(value))1U) << (bitnumber)))
template <typename T> void BIT_CLEAR (T& value, uint8_t bitnumber) noexcept {
    static_assert(std::is_integral<T>::value, "Integral required.");
    ((value) &= ~(static_cast<T>(1U) << (bitnumber)));
}
