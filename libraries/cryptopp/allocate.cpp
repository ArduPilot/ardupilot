// allocate.cpp - written and placed in the public domain by Jeffrey Walton

// The functions in allocate.h and allocate.cpp were originally in misc.h
// and misc.cpp. They were extracted in September 2019 to sidestep a circular
// dependency with misc.h and secblock.h.

#include "pch.h"
#include "config.h"

#ifndef CRYPTOPP_IMPORTS

#include "allocate.h"
#include "stdcpp.h"
#include "misc.h"
#include "trap.h"

// for memalign
#if defined(CRYPTOPP_MEMALIGN_AVAILABLE) || defined(CRYPTOPP_MM_MALLOC_AVAILABLE) || defined(QNX)
# include <malloc.h>
#endif
// for posix_memalign
#if defined(CRYPTOPP_POSIX_MEMALIGN_AVAILABLE)
# include <stdlib.h>
#endif

NAMESPACE_BEGIN(CryptoPP)

void CallNewHandler()
{
	using std::new_handler;
	using std::set_new_handler;

	new_handler newHandler = set_new_handler(NULLPTR);
	if (newHandler)
		set_new_handler(newHandler);

	if (newHandler)
		newHandler();
	else
		throw std::bad_alloc();
}

void * AlignedAllocate(size_t size)
{
	byte *p;
#if defined(CRYPTOPP_MM_MALLOC_AVAILABLE)
	while ((p = (byte *)_mm_malloc(size, 16)) == NULLPTR)
#elif defined(CRYPTOPP_MEMALIGN_AVAILABLE)
	while ((p = (byte *)memalign(16, size)) == NULLPTR)
#elif defined(CRYPTOPP_MALLOC_ALIGNMENT_IS_16)
	while ((p = (byte *)malloc(size)) == NULLPTR)
#elif defined(CRYPTOPP_POSIX_MEMALIGN_AVAILABLE)
	while (posix_memalign(reinterpret_cast<void**>(&p), 16, size) != 0)
#else
	while ((p = (byte *)malloc(size + 16)) == NULLPTR)
#endif
		CallNewHandler();

#ifdef CRYPTOPP_NO_ALIGNED_ALLOC
	size_t adjustment = 16-((size_t)p%16);
	CRYPTOPP_ASSERT(adjustment > 0);
	p += adjustment;
	p[-1] = (byte)adjustment;
#endif

	// If this assert fires then there are problems that need
	// to be fixed. Please open a bug report.
	CRYPTOPP_ASSERT(IsAlignedOn(p, 16));
	return p;
}

void AlignedDeallocate(void *p)
{
	// Guard pointer due to crash on AIX when CRYPTOPP_NO_ALIGNED_ALLOC
	// is in effect. The guard was previously in place in SecBlock,
	// but it was removed at f4d68353ca7c as part of GH #875.
	CRYPTOPP_ASSERT(p);

	if (p != NULLPTR)
	{
#ifdef CRYPTOPP_MM_MALLOC_AVAILABLE
		_mm_free(p);
#elif defined(CRYPTOPP_NO_ALIGNED_ALLOC)
		p = (byte *)p - ((byte *)p)[-1];
		free(p);
#else
		free(p);
#endif
	}
}

void * UnalignedAllocate(size_t size)
{
	void *p;
	while ((p = malloc(size)) == NULLPTR)
		CallNewHandler();
	return p;
}

void UnalignedDeallocate(void *p)
{
	free(p);
}

NAMESPACE_END

#endif  // CRYPTOPP_IMPORTS
