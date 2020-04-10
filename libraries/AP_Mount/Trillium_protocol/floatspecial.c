#include "floatspecial.h"
#include <math.h>


/*!
 * Determine if a 32-bit field represents a valid 32-bit IEEE-754 floating
 * point number. If the field is infinity, NaN, or de-normalized then it is
 * not valid. This determination is made without using any floating point
 * instructions.
 * \param value is the field to evaluate
 * \return 0 if field is infinity, NaN, or de-normalized, else 1
 */
int isFloat32Valid(uint32_t value)
{
    // Five cases for floating point numbers:
    // 0) The eponent is greater than zero and less than the maximum. This is a normal non-zero number.
    // 1) The exponent and the significand are zero. This is zero.
    // 2) The exponent is zero, and the significant is non-zero. This is denormalized.
    // 3) The exponent is the maximum value, and the significand is zero. This is infinity.
    // 4) The exponent is the maximum value, and the significand is non-zero. This is NaN.
    // We check for cases 2, 3, 4 and return 0 if it happens

    if((value & 0x7F800000) == (0x7F800000))
    {
        // inifinity or NaN.
        return 0;

    }// if the exponent is the maximum value
    else if((value & 0x7F800000) == 0)
    {
        // Check for denormalized number
        if(value & 0x007FFFFF)
            return 0;

    }// else if the exponent is zero

    // If we get here then its a valid float
    return 1;

}// isFloat32Valid


/*!
 * Determine if a 64-bit field represents a valid 64-bit IEEE-754 floating
 * point number. If the field is infinity, NaN, or de-normalized then it is
 * not valid. This determination is made without using any floating point
 * instructions.
 * \param value is the field to evaluate
 * \return 0 if field is infinity, NaN, or de-normalized, else 1
 */
int isFloat64Valid(uint64_t value)
{
    // Five cases for floating point numbers:
    // 0) The eponent is greater than zero and less than the maximum. This is a normal non-zero number.
    // 1) The exponent and the significand are zero. This is zero.
    // 2) The exponent is zero, and the significant is non-zero. This is denormalized.
    // 3) The exponent is the maximum value, and the significand is zero. This is infinity.
    // 4) The exponent is the maximum value, and the significand is non-zero. This is NaN.
    // We check for cases 2, 3, 4 and return 0 if it happens

    if((value & 0x7FF0000000000000ULL) == (0x7FF0000000000000ULL))
    {
        // inifinity or NaN.
        return 0;

    }// if the exponent is the maximum value
    else if((value & 0x7FF0000000000000ULL) == 0)
    {
        // Check for denormalized number
        if(value & 0x000FFFFFFFFFFFFFULL)
            return 0;

    }// else if the exponent is zero

    // If we get here then its a valid float
    return 1;

}// isFloat64Valid


/*!
 * \deprecated
 * Convert a 32-bit floating point value (IEEE-754 binary32) to 24-bit floating
 * point representation with 15 bits significand. Underflow will be returned as
 * zero and overflow as the maximum possible value.
 * \param value is the 32-bit floating point data to convert.
 * \return The 24-bit floating point as a simple 32-bit integer with the most
 *         significant byte clear.
 */
uint32_t float32ToFloat24(float value)
{
    return float32ToFloat24ex(value, 15);
}


/*!
 * Convert a 32-bit floating point value (IEEE-754 binary32) to 24-bit floating
 * point representation with a variable number of bits for the significand.
 * Underflow will be returned as zero and overflow as the maximum possible value.
 * \param value is the 32-bit floating point data to convert.
 * \param sigbits is the number of bits to use for the significand.
 * \return The float24 as a simple 24-bit integer (most significant byte clear).
 */
uint32_t float32ToFloat24ex(float value, int sigbits)
{
    union
    {
        float Float;
        uint32_t Integer;
    }field;

    uint32_t significand;
    uint32_t unsignedExponent;
    int32_t  signedExponent;
    uint32_t output;

    // The bias is computed as 2 raised to the number of exponent bits divided
    // by two, minus 1. This can be simplified as 2^(exponent bits -1) - 1
    // The number of exponent bits is 24 - 1 - sigbits
    int bias = (1 << (22 - sigbits)) - 1;

    // Write the floating point value to our union so we can access its bits.
    // Note that C99 and C++2011 have built in goodness for this sort of
    // thing, but not all compilers support that (sigh...)
    field.Float = value;

    // The significand is the least significant 23 bits (IEEE754)
    significand = field.Integer & 0x007FFFFF;

    // Exponent occupies the next 8 bits (IEEE754)
    unsignedExponent = (field.Integer & 0x7F800000) >> 23;

    // Get rid of some bits, here is where we sacrifice resolution
    output = (uint16_t)(significand >> (23-sigbits));

    // If significand and exponent are zero means a number of zero
    if((output == 0) && (unsignedExponent == 0))
    {
        // return correctly signed result
        if(field.Integer & 0x80000000)
            return 0x00800000;
        else
            return 0;
    }

    // Get the un-biased exponent. Binary32 is biased by 127
    signedExponent = unsignedExponent - 127;

    if(signedExponent < -bias)
        output = 0;   // underflow to zero
    else
    {
        if(signedExponent > bias)
        {
            // Largest possible exponent and significand without making a NaN or Inf
            signedExponent = bias;
            output = (uint32_t)(1 << sigbits) - 1;
        }

        // re-bias with the new bias
        unsignedExponent = (uint32_t)(signedExponent + bias);

        // Put the exponent in the output
        output |= (uint32_t)(unsignedExponent << sigbits);
    }

    // Account for the sign
    if(field.Integer & 0x80000000)
        output |= 0x00800000;

    // return the 24-bit representation
    return output;

}// float32ToFloat24ex


/*!
 * \deprecated
 * Convert a 24 bit floating point representation with 15 bits significand to
 * binary32.
 * \param value is the 24-bit representation to convert.
 * \return the binary32 version as a float.
 */
float float24ToFloat32(uint32_t value)
{
    return float24ToFloat32ex(value, 15);
}


/*!
 * Convert a 24-bit floating point representation with variable number of
 * significand bits to binary32
 * \param value is the float16 representation to convert.
 * \param sigbits is the number of bits to use for the significand of the 24-bit float.
 * \return the binary32 version as a float.
 */
float float24ToFloat32ex(uint32_t value, int sigbits)
{
    union
    {
        float Float;
        uint32_t Integer;
    }field;

    // Zero is a special case
    if((value & 0x007FFFFF) == 0)
    {
        field.Integer = 0;
    }
    else
    {
        // The mask for the significand bits
        int sigmask = (1 << sigbits) - 1;

        // The unsigned exponent, mask off the leading sign bit
        uint32_t unsignedExponent = ((value & 0x007FFFFF) >> sigbits);

        // The bias is computed as 2 raised to the number of exponent bits divided
        // by two, minus 1. This can be simplified as 2^(exponent bits -1) - 1
        // The number of exponent bits is 24 - 1 - sigbits
        int bias = (1 << (22 - sigbits)) - 1;

        // We want to subtract our bias to get un-biased, and then add 127 for the new bias
        unsignedExponent += (127 - bias);

        // Reduced bits of signficand, shift it up to 23 bits
        field.Integer = (value & sigmask) << (23-sigbits);

        // Put the exponent in
        field.Integer |= (unsignedExponent << 23);
    }

    // And the sign bit
    if(value & 0x00800000)
        field.Integer |= 0x80000000;

    return field.Float;

}// float24ToFloat32ex


/*!
 * \deprecated
 * Convert a 32-bit floating point value (IEEE-754 binary32) to 16-bit floating
 * point representation with 9 bits significand. Underflow will be returned as
 * zero and overflow as the maximum possible value.
 * \param value is the 32-bit floating point data to convert.
 * \return The binary16 as a simple 16-bit integer.
 */
uint16_t float32ToFloat16(float value)
{
    return float32ToFloat16ex(value, 9);

}// float32ToFloat16


/*!
 * \deprecated
 * Convert a 16 bit floating point representation with 9 bits significand to
 * binary32.
 * \param value is the binary16 representation to convert.
 * \return the binary32 version as a float.
 */
float float16ToFloat32(uint16_t value)
{
    return float16ToFloat32ex(value, 9);

}// float16ToFloat32


/*!
 * Convert a 32-bit floating point value (IEEE-754 binary32) to 16-bit floating
 * point representation with a variable number of bits for the significand.
 * Underflow will be returned as zero and overflow as the maximum possible value.
 * \param value is the 32-bit floating point data to convert.
 * \param sigbits is the number of bits to use for the significand.
 * \return The float16 as a simple 16-bit integer.
 */
uint16_t float32ToFloat16ex(float value, int sigbits)
{
    union
    {
        float Float;
        uint32_t Integer;
    }field;

    uint32_t significand;
    uint32_t unsignedExponent;
    int32_t  signedExponent;
    uint16_t output;

    // The bias is computed as 2 raised to the number of exponent bits divided
    // by two, minus 1. This can be simplified as 2^(exponent bits -1) - 1
    // The number of exponent bits is 16 - 1 - sigbits
    int bias = (1 << (14 - sigbits)) - 1;

    // Write the floating point value to our union so we can access its bits.
    // Note that C99 and C++2011 have built in goodness for this sort of
    // thing, but not all compilers support that (sigh...)
    field.Float = value;

    // The significand is the least significant 23 bits (IEEE754)
    significand = field.Integer & 0x007FFFFF;

    // Exponent occupies the next 8 bits (IEEE754)
    unsignedExponent = (field.Integer & 0x7F800000) >> 23;

    // Get rid of some bits, here is where we sacrifice resolution
    output = (uint16_t)(significand >> (23-sigbits));

    // If significand and exponent are zero means a number of zero
    if((output == 0) && (unsignedExponent == 0))
    {
        // return correctly signed result
        if(field.Integer & 0x80000000)
            return 0x8000;
        else
            return 0;
    }

    // Get the un-biased exponent. Binary32 is biased by 127
    signedExponent = unsignedExponent - 127;

    // With a 6-bit exponent we can support exponents of
    // exponent : biased value
    // -31      : 0 (value is zero or denormalized)
    // -30      : 1
    //  -1      : 30
    //   0      : 31
    //   1      : 32
    //  31      : 62
    //  32      : NaN (all exponent bits are 1)

    // With a 5-bit exponent we get
    // exponent : biased value
    // -15      : 0 (value is zero or denormalized)
    // -14      : 1
    //  -1      : 14
    //   0      : 15
    //   1      : 16
    //  15      : 32
    //  16      : NaN (all exponent bits are 1)

    if(signedExponent < -bias)
        output = 0;   // underflow to zero
    else
    {
        if(signedExponent > bias)
        {
            // Largest possible exponent and significand without making a NaN or Inf
            signedExponent = bias;
            output = (uint32_t)(1 << sigbits) - 1;
        }

        // re-bias with the new bias
        unsignedExponent = (uint32_t)(signedExponent + bias);

        // Put the exponent in the output
        output |= (uint16_t)(unsignedExponent << sigbits);
    }

    // Account for the sign
    if(field.Integer & 0x80000000)
        output |= 0x8000;

    // return the binary16 representation
    return output;

}// float32ToFloat16ex


/*!
 * Convert a 16-bit floating point representation with variable number of
 * significand bits to binary32
 * \param value is the float16 representation to convert.
 * \param sigbits is the number of bits to use for the significand of the 16-bit float.
 * \return the binary32 version as a float.
 */
float float16ToFloat32ex(uint16_t value, int sigbits)
{
    union
    {
        float Float;
        uint32_t Integer;
    }field;

    // Zero is a special case
    if((value & 0x7FFF) == 0)
    {
        field.Integer = 0;
    }
    else
    {
        // The mask for the significand bits
        int sigmask = (1 << sigbits) - 1;

        // The unsigned exponent, mask off the leading sign bit
        uint32_t unsignedExponent = ((value & 0x7FFF) >> sigbits);

        // The bias is computed as 2 raised to the number of exponent bits divided
        // by two, minus 1. This can be simplified as 2^(exponent bits -1) - 1
        // The number of exponent bits is 16 - 1 - sigbits
        int bias = (1 << (14 - sigbits)) - 1;

        // We want to subtract our bias to get un-biased, and then add 127 for the new bias
        unsignedExponent += (127 - bias);

        // Reduced bits of signficand, shift it up to 23 bits
        field.Integer = (value & sigmask) << (23-sigbits);

        // Put the exponent in
        field.Integer |= (unsignedExponent << 23);
    }

    // And the sign bit
    if(value & 0x8000)
        field.Integer |= 0x80000000;

    return field.Float;

}// float16ToFloat32ex


/*!
 * Use this routine (and a debugger) to verify the special float functionality
 * return 1 if test passed
 */
int testSpecialFloat(void)
{
    int i;
    float dataIn[6], dataOut16[6], dataOut24[6];

    union
    {
        float Float;
        uint32_t Integer;
    }test;

    float error = 0;

    test.Float = -.123456789f;

    for(i = 0; i < 3; i++)
    {
        test.Float *= 10.0f;
        dataIn[i] = test.Float;
        dataOut16[i] = float16ToFloat32(float32ToFloat16(dataIn[i]));
        dataOut24[i] = float24ToFloat32(float32ToFloat24(dataIn[i]));
        error += (float)fabs((dataIn[i] - dataOut16[i])/dataIn[i]);
        error += (float)fabs((dataIn[i] - dataOut24[i])/dataIn[i]);
    }

    test.Float = 12.3456789f;
    for(;i < 6; i++)
    {
        test.Float /= 10.0f;
        dataIn[i] = test.Float;
        dataOut16[i] = float16ToFloat32(float32ToFloat16(dataIn[i]));
        dataOut24[i] = float24ToFloat32(float32ToFloat24(dataIn[i]));
        error += (float)fabs((dataIn[i] - dataOut16[i])/dataIn[i]);
        error += (float)fabs((dataIn[i] - dataOut24[i])/dataIn[i]);
    }

    if(error > 0.01f)
        return 0;

    // Maximum possible float without Inf or Nan
    test.Integer = 0x7F7FFFFF;

    // This loop exercises the overflow and underflow, use the debugger to verify functionality
    for(i = 0; i < 6; i++)
    {
        dataIn[i] = test.Float;
        dataOut16[i] = float16ToFloat32(float32ToFloat16(dataIn[i]));
        dataOut24[i] = float24ToFloat32(float32ToFloat24(dataIn[i]));
        test.Float /= 1000000000000.0f;
    }

    return 1;

}// testSpecialFloat
