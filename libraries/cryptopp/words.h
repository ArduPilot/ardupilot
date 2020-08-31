// words.h - originally written and placed in the public domain by Wei Dai

/// \file words.h
/// \brief Support functions for word operations

#ifndef CRYPTOPP_WORDS_H
#define CRYPTOPP_WORDS_H

#include "config.h"
#include "misc.h"

NAMESPACE_BEGIN(CryptoPP)

/// \brief Count the number of words
/// \param x word array
/// \param n size of the word array, in elements
/// \returns number of words used in the array.
/// \details CountWords counts the number of words in a word array.
///  Leading 0-words are not included in the count.
/// \since Crypto++ 1.0
inline size_t CountWords(const word *x, size_t n)
{
	while (n && x[n-1]==0)
		n--;
	return n;
}

/// \brief Set the value of words
/// \param r word array
/// \param a value
/// \param n size of the word array, in elements
/// \details SetWords sets all elements in the word array to the
///  specified value.
/// \since Crypto++ 1.0
inline void SetWords(word *r, word a, size_t n)
{
	for (size_t i=0; i<n; i++)
		r[i] = a;
}

/// \brief Copy word array
/// \param r destination word array
/// \param a source word array
/// \param n size of the word array, in elements
/// \details CopyWords copies the source word array to the destination
///  word array.
/// \since Crypto++ 1.0
inline void CopyWords(word *r, const word *a, size_t n)
{
	if (r != a)
#if CRYPTOPP_MSC_VERSION
		memcpy_s(r, n*WORD_SIZE, a, n*WORD_SIZE);
#else
		memcpy(r, a, n*WORD_SIZE);
#endif
}

/// \brief XOR word arrays
/// \param r destination word array
/// \param a first source word array
/// \param b second source word array
/// \param n size of the word array, in elements
/// \details XorWords XORs the two source word arrays and copies the
///  result to the destination word array.
/// \since Crypto++ 1.0
inline void XorWords(word *r, const word *a, const word *b, size_t n)
{
	for (size_t i=0; i<n; i++)
		r[i] = a[i] ^ b[i];
}

/// \brief XOR word arrays
/// \param r destination word array
/// \param a source word array
/// \param n size of the word array, in elements
/// \details XorWords XORs the source word array with the
///  destination word array.
/// \since Crypto++ 1.0
inline void XorWords(word *r, const word *a, size_t n)
{
	for (size_t i=0; i<n; i++)
		r[i] ^= a[i];
}

/// \brief AND word arrays
/// \param r destination word array
/// \param a first source word array
/// \param b second source word array
/// \param n size of the word array, in elements
/// \details AndWords ANDs the two source word arrays and copies the
///  result to the destination word array.
/// \since Crypto++ 1.0
inline void AndWords(word *r, const word *a, const word *b, size_t n)
{
	for (size_t i=0; i<n; i++)
		r[i] = a[i] & b[i];
}

/// \brief AND word arrays
/// \param r destination word array
/// \param a source word array
/// \param n size of the word array, in elements
/// \details AndWords ANDs the source word array with the
///  destination word array.
/// \since Crypto++ 1.0
inline void AndWords(word *r, const word *a, size_t n)
{
	for (size_t i=0; i<n; i++)
		r[i] &= a[i];
}

/// \brief OR word arrays
/// \param r destination word array
/// \param a first source word array
/// \param b second source word array
/// \param n size of the word array, in elements
/// \details OrWords ORs the two source word arrays and copies the
///  result to the destination word array.
/// \since Crypto++ 1.0
inline void OrWords(word *r, const word *a, const word *b, size_t n)
{
	for (size_t i=0; i<n; i++)
		r[i] = a[i] | b[i];
}

/// \brief OR word arrays
/// \param r destination word array
/// \param a source word array
/// \param n size of the word array, in elements
/// \details OrWords ORs the source word array with the
///  destination word array.
/// \since Crypto++ 1.0
inline void OrWords(word *r, const word *a, size_t n)
{
	for (size_t i=0; i<n; i++)
		r[i] |= a[i];
}

/// \brief Left shift word array
/// \param r word array
/// \param n size of the word array, in elements
/// \param shiftBits number of bits to shift
/// \returns word shifted out
/// \details ShiftWordsLeftByBits shifts the word array left by
///  shiftBits. ShiftWordsLeftByBits shifts bits out on the left;
///  it does not extend the array.
/// \note shiftBits must be less than WORD_BITS.
/// \since Crypto++ 1.0
inline word ShiftWordsLeftByBits(word *r, size_t n, unsigned int shiftBits)
{
	CRYPTOPP_ASSERT (shiftBits<WORD_BITS);
	word u, carry=0;
	if (shiftBits)
		for (size_t i=0; i<n; i++)
		{
			u = r[i];
			r[i] = (u << shiftBits) | carry;
			carry = u >> (WORD_BITS-shiftBits);
		}
	return carry;
}

/// \brief Right shift word array
/// \param r word array
/// \param n size of the word array, in elements
/// \param shiftBits number of bits to shift
/// \returns word shifted out
/// \details ShiftWordsRightByBits shifts the word array shight by
///  shiftBits. ShiftWordsRightByBits shifts bits out on the right.
/// \note shiftBits must be less than WORD_BITS.
/// \since Crypto++ 1.0
inline word ShiftWordsRightByBits(word *r, size_t n, unsigned int shiftBits)
{
	CRYPTOPP_ASSERT (shiftBits<WORD_BITS);
	word u, carry=0;
	if (shiftBits)
		for (size_t i=n; i>0; i--)
		{
			u = r[i-1];
			r[i-1] = (u >> shiftBits) | carry;
			carry = u << (WORD_BITS-shiftBits);
		}
	return carry;
}

/// \brief Left shift word array
/// \param r word array
/// \param n size of the word array, in elements
/// \param shiftWords number of words to shift
/// \details ShiftWordsLeftByWords shifts the word array left by
///  shiftWords. ShiftWordsLeftByWords shifts bits out on the left;
///  it does not extend the array.
/// \since Crypto++ 1.0
inline void ShiftWordsLeftByWords(word *r, size_t n, size_t shiftWords)
{
	shiftWords = STDMIN(shiftWords, n);
	if (shiftWords)
	{
		for (size_t i=n-1; i>=shiftWords; i--)
			r[i] = r[i-shiftWords];
		SetWords(r, 0, shiftWords);
	}
}

/// \brief Right shift word array
/// \param r word array
/// \param n size of the word array, in elements
/// \param shiftWords number of words to shift
/// \details ShiftWordsRightByWords shifts the word array right by
///  shiftWords. ShiftWordsRightByWords shifts bits out on the right.
/// \since Crypto++ 1.0
inline void ShiftWordsRightByWords(word *r, size_t n, size_t shiftWords)
{
	shiftWords = STDMIN(shiftWords, n);
	if (shiftWords)
	{
		for (size_t i=0; i+shiftWords<n; i++)
			r[i] = r[i+shiftWords];
		SetWords(r+n-shiftWords, 0, shiftWords);
	}
}

NAMESPACE_END

#endif
