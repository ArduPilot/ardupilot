/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file   sgUtil.h
 * @author jimb
 *
 * @date Feb 2, 2021
 */

#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>

#ifndef swap16
#define swap16(data) \
    ((((data) >> 8) & 0x00FF) | (((data) << 8) & 0xFF00))
#endif

#ifndef swap24
#define swap24(data) \
    (((data) >> 16) | ((data)&0x00FF00) | (((data) << 16) & 0xFF0000))
#endif

#ifndef swap32
#define swap32(data) \
    (((data) >> 24) | (((data)&0x00FF0000) >> 8) | (((data)&0x0000FF00) << 8) | ((data) << 24))
#endif

#ifndef swap64
#define swap64(data) \
   (swap32((data & 0x00000000ffffffffULL))) << 32 | swap32(data >> 32))
#endif

#ifndef PI
#define PI 3.14159265359
#endif

#ifndef toRad
#define toRad(deg) \
    ((deg)*PI / 180.0)
#endif

#ifndef toDeg
#define toDeg(rad) \
    ((rad)*180 / PI)
#endif

#ifndef toMeter
#define toMeter(feet) \
    ((feet)*0.3048)
#endif

#ifndef toFeet
#define toFeet(meter) \
    ((meter)*3.2808)
#endif

/**
 * Converts an array of bytes to a 16 bit integer.
 *
 * @param bytes The array of bytes to convert.
 *
 * @return The 16 bit integer.
 */
int16_t toInt16(const uint8_t bytes[]);

/**
 * Converts an array of bytes to a 32 bit integer.
 *
 * @param bytes The array of bytes to convert.
 *
 * @return The 32 bit integer.
 */
int32_t toInt32(const uint8_t bytes[]);

/**
 * Converts an array of bytes to a 16 unsigned bit integer.
 *
 * @param bytes The array of bytes to convert.
 *
 * @return The 16 bit integer.
 */
uint16_t toUint16(const uint8_t bytes[]);

/**
 * Converts an array of bytes to a 24 bit unsigned integer with leading 0s.
 *
 * @param bytes The array of bytes to convert.
 *
 * @return The 24 bit unsigned integer with leading 0s.
 */
uint32_t toUint24(const uint8_t bytes[]);

/**
 * Converts an array of bytes to a 32 bit unsigned integer.
 *
 * @param bytes The array of bytes to convert.
 *
 * @return The 32 bit unsigned integer.
 */
uint32_t toUint32(const uint8_t bytes[]);

/**
 * Converts an array of bytes to a distance.
 *
 * @param bytes The array of bytes to convert.
 *
 * @return The distance value.
 */
double toDist(const uint8_t *bytes);

/**
 * Converts an array of bytes to a 24 bit unsigned integer with leading 0's.
 *
 * @param bytes The array of bytes to convert.
 *
 * @return The 32 bit unsigned integer.
 */
uint32_t toIcao(const uint8_t bytes[]);

/**
 * Converts an array of bytes to a lat/lon floating point number.
 *
 * @param bytes The array of bytes to convert.
 *
 * @return The lat/lon value.
 */
double toLatLon(const uint8_t bytes[]);

/**
 * Convert an array to an altitude.
 *
 * @param bytes The bytes to get the altitude from.
 *
 * @return The converted altitude.
 */
double toAlt(const uint8_t bytes[]);

/**
 * Converts an array of bytes to an airborne velocity.
 *
 * @param bytes The bytes to extract the velocity.
 *
 * @return The converted velocity.
 */
double toVel(const uint8_t bytes[]);

/**
 * Converts the array of bytes to the surface ground speed.
 *
 * @param bytes The bytes to extract the ground speed.
 *
 * @return The converted ground speed.
 */
uint8_t toGS(const uint8_t bytes[]);

/**
 * Converts the bytes into the heading value.
 *
 * @param bytes The bytes holding the heading value.
 *
 * @return The heading.
 */
double toHeading(const uint8_t bytes[]);

/**
 * Determine heading from y and x speed vectors.
 *
 * @param y The y speed vector.
 * @param x The x speed vector.
 *
 * @return The resulting heading.
 */
int16_t toHeading2(double y, double x);

/**
 * Convert the array of bytes to a time of applicability (TOA).
 *
 * @param bytes The bytes to convert to a TOA.
 *
 * @return The converted TOA.
 */
float toTOA(const uint8_t bytes[]);

/**
 * Convert an array of bytes to a float
 *
 * @param bufferPos the address of the field's first corresponding buffer byte.
 *
 * @return The converted float value.
 */

float toFloat(const uint8_t *bufferPos);

/**
 * Convert an array of bytes to a double
 *
 * @param bufferPos the address of the field's first corresponding buffer byte.
 *
 * @return The converted double value.
 */

double toDouble(const uint8_t *bufferPos);

/**
 * Converts a uint16_t into its host message buffer format
 *
 * @param bufferPos The address of the field's first corresponding buffer byte.
 * @param value     The uint16_t to be converted.
 *
 * no return value, two buffer bytes are filled by reference
 */
void uint162Buf(uint8_t *bufferPos, uint16_t value);

/**
 * Converts a int16_t into its host message buffer format
 *
 * @param bufferPos The address of the field's first corresponding buffer byte.
 * @param value     The int32_t to be converted.
 *
 * no return value, two buffer bytes are filled by reference
 */
void int242Buf(uint8_t *bufferPos, int32_t value);

/**
 * Converts a uint32_t into a 24 bit host message buffer format
 *
 * @param bufferPos The address of the field's first corresponding buffer byte.
 * @param value     The int32_t to be converted.
 *
 * no return value, three buffer bytes are filled by reference
 */
void uint242Buf(uint8_t *bufferPos, uint32_t value);

/**
 * Converts a uint32_t into its host message buffer format
 *
 * @param bufferPos The address of the field's first corresponding buffer byte.
 * @param value     The uint32_t to be converted.
 *
 * no return value, two buffer bytes are filled by reference
 */
void uint322Buf(uint8_t *bufferPos, uint32_t value);

/**
 * Converts a uint32_t containing an ICAO into its 24-bit host message buffer format
 *
 * @param bufferPos The address of the field's first corresponding buffer byte.
 * @param icao      The uint32_t ICAO to be converted.
 *
 * no return value, three buffer bytes are filled by reference
 *
 * @warning icao parameter must be between 0x000000 and 0xFFFFFF
 */
void icao2Buf(uint8_t *bufferPos, uint32_t icao);

/**
 * Converts an array of characters into its host message buffer format
 *
 * @param bufferPos The address of the field's first corresponding buffer byte.
 * @param arr[]     The array of characters.
 * @param len       The number of characters in the array.
 *
 * no return value, the specified quantity of buffer bytes are filled by reference
 */
void charArray2Buf(uint8_t *bufferPos, char arr[], uint8_t len);

/**
 * Converts a float into its host message buffer format
 *
 * @param bufferPos The address of the field's first corresponding buffer byte.
 * @param value     The float to be converted.
 *
 * no return value, four buffer bytes are filled by reference
 *
 * @warning The output of this function depends on the machine's endianness. It is designed
 *          for Little-Endian machines, only.
 */
void float2Buf(uint8_t *bufferPos, float value);

/**
 * Converts a double into its host message buffer format
 *
 * @param bufferPos The address of the field's first corresponding buffer byte
 * @param value     The double to be converted
 *
 * no return value, eight buffer bytes are filled by reference
 *
 * @warning The output of this function depends on the machine's endianness. It is designed
 *          for Little-Endian machines, only
 */
void double2Buf(uint8_t *bufferPos, double value);

/**
 * Converts a double into an encoded lat/lon buffer format.
 *
 * @param bytes address of the field's first corresponding buffer byte
 * @param value the double to be converted.
 *
 *	no return value, 3 buffer bytes are filled by reference.
 */
void latLon2Buf(uint8_t bytes[], double value);

/**
 * Calculate checksum for a host message.
 *
 * @param buffer The raw message buffer.
 * @param len    The total quantity of bytes in the buffer
 *
 * @return The resulting checksum.
 */
uint8_t calcChecksum(uint8_t *buffer, uint8_t len);

/**
 * Add the checksum to a host message.
 *
 * @param buffer The raw message buffer.
 * @param len    The total quantity of bytes in the buffer
 *
 * no return value, final buffer byte is set to the checksum value.
 */
void appendChecksum(uint8_t *buffer, uint8_t len);

#endif /* UTIL_H */
