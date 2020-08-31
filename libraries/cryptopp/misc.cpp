// misc.cpp - originally written and placed in the public domain by Wei Dai

#include "pch.h"
#include "config.h"

#if CRYPTOPP_MSC_VERSION
# pragma warning(disable: 4189)
# if (CRYPTOPP_MSC_VERSION >= 1400)
#  pragma warning(disable: 6237)
# endif
#endif

#ifndef CRYPTOPP_IMPORTS

#include "misc.h"
#include "trap.h"
#include "words.h"
#include "stdcpp.h"
#include "integer.h"
#include "secblock.h"

NAMESPACE_BEGIN(CryptoPP)

byte* BytePtr(SecByteBlock& str)
{
	// Caller wants a writeable pointer
	CRYPTOPP_ASSERT(str.empty() == false);

	if (str.empty())
		return NULLPTR;
	return reinterpret_cast<byte*>(str.data());
}

const byte* ConstBytePtr(const SecByteBlock& str)
{
	if (str.empty())
		return NULLPTR;
	return reinterpret_cast<const byte*>(str.data());
}

size_t BytePtrSize(const SecByteBlock& str)
{
	return str.size();
}

void xorbuf(byte *buf, const byte *mask, size_t count)
{
	CRYPTOPP_ASSERT(buf != NULLPTR);
	CRYPTOPP_ASSERT(mask != NULLPTR);
	CRYPTOPP_ASSERT(count > 0);

	size_t i=0;
	if (IsAligned<word32>(buf) && IsAligned<word32>(mask))
	{
		if (!CRYPTOPP_BOOL_SLOW_WORD64 && IsAligned<word64>(buf) && IsAligned<word64>(mask))
		{
			for (i=0; i<count/8; i++)
				((word64*)(void*)buf)[i] ^= ((word64*)(void*)mask)[i];
			count -= 8*i;
			if (!count)
				return;
			buf += 8*i;
			mask += 8*i;
		}

		for (i=0; i<count/4; i++)
			((word32*)(void*)buf)[i] ^= ((word32*)(void*)mask)[i];
		count -= 4*i;
		if (!count)
			return;
		buf += 4*i;
		mask += 4*i;
	}

	for (i=0; i<count; i++)
		buf[i] ^= mask[i];
}

void xorbuf(byte *output, const byte *input, const byte *mask, size_t count)
{
	CRYPTOPP_ASSERT(output != NULLPTR);
	CRYPTOPP_ASSERT(input != NULLPTR);
	CRYPTOPP_ASSERT(count > 0);

	size_t i=0;
	if (IsAligned<word32>(output) && IsAligned<word32>(input) && IsAligned<word32>(mask))
	{
		if (!CRYPTOPP_BOOL_SLOW_WORD64 && IsAligned<word64>(output) && IsAligned<word64>(input) && IsAligned<word64>(mask))
		{
			for (i=0; i<count/8; i++)
				((word64*)(void*)output)[i] = ((word64*)(void*)input)[i] ^ ((word64*)(void*)mask)[i];
			count -= 8*i;
			if (!count)
				return;
			output += 8*i;
			input += 8*i;
			mask += 8*i;
		}

		for (i=0; i<count/4; i++)
			((word32*)(void*)output)[i] = ((word32*)(void*)input)[i] ^ ((word32*)(void*)mask)[i];
		count -= 4*i;
		if (!count)
			return;
		output += 4*i;
		input += 4*i;
		mask += 4*i;
	}

	for (i=0; i<count; i++)
		output[i] = input[i] ^ mask[i];
}

bool VerifyBufsEqual(const byte *buf, const byte *mask, size_t count)
{
	CRYPTOPP_ASSERT(buf != NULLPTR);
	CRYPTOPP_ASSERT(mask != NULLPTR);
	// CRYPTOPP_ASSERT(count > 0);

	size_t i=0;
	byte acc8 = 0;

	if (IsAligned<word32>(buf) && IsAligned<word32>(mask) && count)
	{
		word32 acc32 = 0;
		if (!CRYPTOPP_BOOL_SLOW_WORD64 && IsAligned<word64>(buf) && IsAligned<word64>(mask))
		{
			word64 acc64 = 0;
			for (i=0; i<count/8; i++)
				acc64 |= ((word64*)(void*)buf)[i] ^ ((word64*)(void*)mask)[i];
			count -= 8*i;
			if (!count)
				return acc64 == 0;
			buf += 8*i;
			mask += 8*i;
			acc32 = word32(acc64) | word32(acc64>>32);
		}

		for (i=0; i<count/4; i++)
			acc32 |= ((word32*)(void*)buf)[i] ^ ((word32*)(void*)mask)[i];
		count -= 4*i;
		if (!count)
			return acc32 == 0;
		buf += 4*i;
		mask += 4*i;
		acc8 = byte(acc32) | byte(acc32>>8) | byte(acc32>>16) | byte(acc32>>24);
	}

	for (i=0; i<count; i++)
		acc8 |= buf[i] ^ mask[i];
	return acc8 == 0;
}

std::string StringNarrow(const wchar_t *str, bool throwOnError)
{
	CRYPTOPP_ASSERT(str);
	std::string result;

	// Safer functions on Windows for C&A, https://github.com/weidai11/cryptopp/issues/55
#if (CRYPTOPP_MSC_VERSION >= 1400)
	size_t len=0, size=0;
	errno_t err = 0;

	//const wchar_t* ptr = str;
	//while (*ptr++) len++;
	len = wcslen(str)+1;

	err = wcstombs_s(&size, NULLPTR, 0, str, len*sizeof(wchar_t));
	CRYPTOPP_ASSERT(err == 0);
	if (err != 0)
	{
		if (throwOnError)
			throw InvalidArgument("StringNarrow: wcstombs_s() call failed with error " + IntToString(err));
		else
			return std::string();
	}

	result.resize(size);
	err = wcstombs_s(&size, &result[0], size, str, len*sizeof(wchar_t));
	CRYPTOPP_ASSERT(err == 0);
	if (err != 0)
	{
		if (throwOnError)
			throw InvalidArgument("StringNarrow: wcstombs_s() call failed with error " + IntToString(err));
		else
			return std::string();
	}

	// The safe routine's size includes the NULL.
	if (!result.empty() && result[size - 1] == '\0')
		result.erase(size - 1);
#else
	size_t size = wcstombs(NULLPTR, str, 0);
	CRYPTOPP_ASSERT(size != (size_t)-1);
	if (size == (size_t)-1)
	{
		if (throwOnError)
			throw InvalidArgument("StringNarrow: wcstombs() call failed");
		else
			return std::string();
	}

	result.resize(size);
	size = wcstombs(&result[0], str, size);
	CRYPTOPP_ASSERT(size != (size_t)-1);
	if (size == (size_t)-1)
	{
		if (throwOnError)
			throw InvalidArgument("StringNarrow: wcstombs() call failed");
		else
			return std::string();
	}
#endif

	return result;
}

std::wstring StringWiden(const char *str, bool throwOnError)
{
	CRYPTOPP_ASSERT(str);
	std::wstring result;

	// Safer functions on Windows for C&A, https://github.com/weidai11/cryptopp/issues/55
#if (CRYPTOPP_MSC_VERSION >= 1400)
	size_t len=0, size=0;
	errno_t err = 0;

	//const char* ptr = str;
	//while (*ptr++) len++;
	len = std::strlen(str)+1;

	err = mbstowcs_s(&size, NULLPTR, 0, str, len);
	CRYPTOPP_ASSERT(err == 0);
	if (err != 0)
	{
		if (throwOnError)
			throw InvalidArgument("StringWiden: wcstombs_s() call failed with error " + IntToString(err));
		else
			return std::wstring();
	}

	result.resize(size);
	err = mbstowcs_s(&size, &result[0], size, str, len);
	CRYPTOPP_ASSERT(err == 0);
	if (err != 0)
	{
		if (throwOnError)
			throw InvalidArgument("StringWiden: wcstombs_s() call failed with error " + IntToString(err));
		else
			return std::wstring();
	}

	// The safe routine's size includes the NULL.
	if (!result.empty() && result[size - 1] == '\0')
		result.erase(size - 1);
#else
	size_t size = mbstowcs(NULLPTR, str, 0);
	CRYPTOPP_ASSERT(size != (size_t)-1);
	if (size == (size_t)-1)
	{
		if (throwOnError)
			throw InvalidArgument("StringWiden: mbstowcs() call failed");
		else
			return std::wstring();
	}

	result.resize(size);
	size = mbstowcs(&result[0], str, size);
	CRYPTOPP_ASSERT(size != (size_t)-1);
	if (size == (size_t)-1)
	{
		if (throwOnError)
			throw InvalidArgument("StringWiden: mbstowcs() call failed");
		else
			return std::wstring();
	}
#endif

	return result;
}

NAMESPACE_END

#endif
