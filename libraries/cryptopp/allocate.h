// allocate.h - written and placed in the public domain by Jeffrey Walton

// The functions in allocate.h and allocate.cpp were originally in misc.h
// and misc.cpp. They were extracted in September 2019 to sidestep a circular
// dependency with misc.h and secblock.h.

/// \file allocate.h
/// \brief Functions for allocating aligned buffers

#ifndef CRYPTOPP_ALLOCATE_H
#define CRYPTOPP_ALLOCATE_H

#include "config.h"
#include "cryptlib.h"

NAMESPACE_BEGIN(CryptoPP)

/// \brief Attempts to reclaim unused memory
/// \throws bad_alloc
/// \details In the normal course of running a program, a request for memory
///  normally succeeds. If a call to AlignedAllocate or UnalignedAllocate fails,
///  then CallNewHandler is called in n effort to recover. Internally,
///  CallNewHandler calls set_new_handler(nullptr) in an effort to free memory.
///  There is no guarantee CallNewHandler will be able to obtain more memory so
///  an allocation succeeds. If the call to set_new_handler fails, then CallNewHandler
///  throws a bad_alloc exception.
/// \throws bad_alloc on failure
/// \since Crypto++ 5.0
/// \sa AlignedAllocate, AlignedDeallocate, UnalignedAllocate, UnalignedDeallocate
CRYPTOPP_DLL void CRYPTOPP_API CallNewHandler();

/// \brief Allocates a buffer on 16-byte boundary
/// \param size the size of the buffer
/// \details AlignedAllocate is primarily used when the data will be
///  proccessed by SSE, NEON, ARMv8 or PowerPC instructions. The assembly
///  language routines rely on the alignment. If the alignment is not
///  respected, then a SIGBUS could be generated on Unix and Linux, and an
///  EXCEPTION_DATATYPE_MISALIGNMENT could be generated on Windows.
/// \details Formerly, AlignedAllocate and AlignedDeallocate were only
///  available on certain platforms when CRYTPOPP_DISABLE_ASM was not in
///  effect. However, Android and iOS debug simulator builds got into a
///  state where the aligned allocator was not available and caused link
///  failures.
/// \since AlignedAllocate for SIMD since Crypto++ 1.0, AlignedAllocate
///  for all builds since Crypto++ 8.1
/// \sa AlignedDeallocate, UnalignedAllocate, UnalignedDeallocate, CallNewHandler,
///  <A HREF="http://github.com/weidai11/cryptopp/issues/779">Issue 779</A>
CRYPTOPP_DLL void* CRYPTOPP_API AlignedAllocate(size_t size);

/// \brief Frees a buffer allocated with AlignedAllocate
/// \param ptr the buffer to free
/// \since AlignedDeallocate for SIMD since Crypto++ 1.0, AlignedAllocate
///  for all builds since Crypto++ 8.1
/// \sa AlignedAllocate, UnalignedAllocate, UnalignedDeallocate, CallNewHandler,
///  <A HREF="http://github.com/weidai11/cryptopp/issues/779">Issue 779</A>
CRYPTOPP_DLL void CRYPTOPP_API AlignedDeallocate(void *ptr);

/// \brief Allocates a buffer
/// \param size the size of the buffer
/// \since Crypto++ 1.0
/// \sa AlignedAllocate, AlignedDeallocate, UnalignedDeallocate, CallNewHandler,
///  <A HREF="http://github.com/weidai11/cryptopp/issues/779">Issue 779</A>
CRYPTOPP_DLL void * CRYPTOPP_API UnalignedAllocate(size_t size);

/// \brief Frees a buffer allocated with UnalignedAllocate
/// \param ptr the buffer to free
/// \since Crypto++ 1.0
/// \sa AlignedAllocate, AlignedDeallocate, UnalignedAllocate, CallNewHandler,
///  <A HREF="http://github.com/weidai11/cryptopp/issues/779">Issue 779</A>
CRYPTOPP_DLL void CRYPTOPP_API UnalignedDeallocate(void *ptr);

NAMESPACE_END

#endif  // CRYPTOPP_ALLOCATE_H
