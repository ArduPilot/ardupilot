// secblock.h - originally written and placed in the public domain by Wei Dai

/// \file secblock.h
/// \brief Classes and functions for secure memory allocations.

#ifndef CRYPTOPP_SECBLOCK_H
#define CRYPTOPP_SECBLOCK_H

#include "config.h"
#include "allocate.h"
#include "misc.h"
#include "stdcpp.h"

#if CRYPTOPP_MSC_VERSION
# pragma warning(push)
# pragma warning(disable: 4231 4275 4700)
# if (CRYPTOPP_MSC_VERSION >= 1400)
#  pragma warning(disable: 6011 6386 28193)
# endif
#endif

NAMESPACE_BEGIN(CryptoPP)

// ************** secure memory allocation ***************

/// \brief Base class for all allocators used by SecBlock
/// \tparam T the class or type
template<class T>
class AllocatorBase
{
public:
	typedef T value_type;
	typedef size_t size_type;
	typedef std::ptrdiff_t difference_type;
	typedef T * pointer;
	typedef const T * const_pointer;
	typedef T & reference;
	typedef const T & const_reference;

	pointer address(reference r) const {return (&r);}
	const_pointer address(const_reference r) const {return (&r); }
	void construct(pointer p, const T& val) {new (p) T(val);}
	void destroy(pointer p) {CRYPTOPP_UNUSED(p); p->~T();}

	/// \brief Returns the maximum number of elements the allocator can provide
	/// \details <tt>ELEMS_MAX</tt> is the maximum number of elements the
	///  <tt>Allocator</tt> can provide. The value of <tt>ELEMS_MAX</tt> is
	///  <tt>SIZE_MAX/sizeof(T)</tt>. <tt>std::numeric_limits</tt> was avoided
	///  due to lack of <tt>constexpr</tt>-ness in C++03 and below.
	/// \note In C++03 and below <tt>ELEMS_MAX</tt> is a static data member of type
	///  <tt>size_type</tt>. In C++11 and above <tt>ELEMS_MAX</tt> is an <tt>enum</tt>
	///  inheriting from <tt>size_type</tt>. In both cases <tt>ELEMS_MAX</tt> can be
	///  used before objects are fully constructed, and it does not suffer the
	///  limitations of class methods like <tt>max_size</tt>.
	/// \sa <A HREF="http://github.com/weidai11/cryptopp/issues/346">Issue 346/CVE-2016-9939</A>
	/// \since Crypto++ 6.0
#if defined(CRYPTOPP_DOXYGEN_PROCESSING)
	static const size_type ELEMS_MAX = ...;
#elif defined(_MSC_VER) && (_MSC_VER <= 1400)
	static const size_type ELEMS_MAX = (~(size_type)0)/sizeof(T);
#elif defined(CRYPTOPP_CXX11_STRONG_ENUM)
	enum : size_type {ELEMS_MAX = SIZE_MAX/sizeof(T)};
#else
	static const size_type ELEMS_MAX = SIZE_MAX/sizeof(T);
#endif

	/// \brief Returns the maximum number of elements the allocator can provide
	/// \returns the maximum number of elements the allocator can provide
	/// \details Internally, preprocessor macros are used rather than std::numeric_limits
	///  because the latter is not a constexpr. Some compilers, like Clang, do not
	///  optimize it well under all circumstances. Compilers like GCC, ICC and MSVC appear
	///  to optimize it well in either form.
	CRYPTOPP_CONSTEXPR size_type max_size() const {return ELEMS_MAX;}

#if defined(__SUNPRO_CC)
	// https://github.com/weidai11/cryptopp/issues/770
	// and https://stackoverflow.com/q/53999461/608639
	CRYPTOPP_CONSTEXPR size_type max_size(size_type n) const {return SIZE_MAX/n;}
#endif

#if defined(CRYPTOPP_CXX11_VARIADIC_TEMPLATES) || defined(CRYPTOPP_DOXYGEN_PROCESSING)

	/// \brief Constructs a new V using variadic arguments
	/// \tparam V the type to be forwarded
	/// \tparam Args the arguments to be forwarded
	/// \param ptr pointer to type V
	/// \param args variadic arguments
	/// \details This is a C++11 feature. It is available when CRYPTOPP_CXX11_VARIADIC_TEMPLATES
	///  is defined. The define is controlled by compiler versions detected in config.h.
    template<typename V, typename... Args>
    void construct(V* ptr, Args&&... args) {::new ((void*)ptr) V(std::forward<Args>(args)...);}

	/// \brief Destroys an V constructed with variadic arguments
	/// \tparam V the type to be forwarded
	/// \details This is a C++11 feature. It is available when CRYPTOPP_CXX11_VARIADIC_TEMPLATES
	///  is defined. The define is controlled by compiler versions detected in config.h.
    template<typename V>
    void destroy(V* ptr) {if (ptr) ptr->~V();}

#endif

protected:

	/// \brief Verifies the allocator can satisfy a request based on size
	/// \param size the size of the allocation, in elements
	/// \throws InvalidArgument
	/// \details CheckSize verifies the number of elements requested is valid.
	/// \details If size is greater than max_size(), then InvalidArgument is thrown.
	///  The library throws InvalidArgument if the size is too large to satisfy.
	/// \details Internally, preprocessor macros are used rather than std::numeric_limits
	///  because the latter is not a constexpr. Some compilers, like Clang, do not
	///  optimize it well under all circumstances. Compilers like GCC, ICC and MSVC appear
	///  to optimize it well in either form.
	/// \details The <tt>sizeof(T) != 1</tt> in the condition attempts to help the
	///  compiler optimize the check for byte types. Coverity findings for
	///  CONSTANT_EXPRESSION_RESULT were generated without it. For byte types,
	///  size never exceeded ELEMS_MAX but the code was not removed.
	/// \note size is the count of elements, and not the number of bytes
	static void CheckSize(size_t size)
	{
		// Squash MSC C4100 warning for size. Also see commit 42b7c4ea5673.
		CRYPTOPP_UNUSED(size);
		// C++ throws std::bad_alloc (C++03) or std::bad_array_new_length (C++11) here.
		if (sizeof(T) != 1 && size > ELEMS_MAX)
			throw InvalidArgument("AllocatorBase: requested size would cause integer overflow");
	}
};

#define CRYPTOPP_INHERIT_ALLOCATOR_TYPES	\
typedef typename AllocatorBase<T>::value_type value_type;\
typedef typename AllocatorBase<T>::size_type size_type;\
typedef typename AllocatorBase<T>::difference_type difference_type;\
typedef typename AllocatorBase<T>::pointer pointer;\
typedef typename AllocatorBase<T>::const_pointer const_pointer;\
typedef typename AllocatorBase<T>::reference reference;\
typedef typename AllocatorBase<T>::const_reference const_reference;

/// \brief Reallocation function
/// \tparam T the class or type
/// \tparam A the class or type's allocator
/// \param alloc the allocator
/// \param oldPtr the previous allocation
/// \param oldSize the size of the previous allocation
/// \param newSize the new, requested size
/// \param preserve flag that indicates if the old allocation should be preserved
/// \note oldSize and newSize are the count of elements, and not the
///  number of bytes.
template <class T, class A>
typename A::pointer StandardReallocate(A& alloc, T *oldPtr, typename A::size_type oldSize, typename A::size_type newSize, bool preserve)
{
	// Avoid assert on pointer in reallocate. SecBlock regularly uses NULL
	// pointers rather returning non-NULL 0-sized pointers.
	if (oldSize == newSize)
		return oldPtr;

	if (preserve)
	{
		typename A::pointer newPointer = alloc.allocate(newSize, NULLPTR);
		const typename A::size_type copySize = STDMIN(oldSize, newSize) * sizeof(T);

		if (oldPtr && newPointer)
			memcpy_s(newPointer, copySize, oldPtr, copySize);

		if (oldPtr)
			alloc.deallocate(oldPtr, oldSize);

		return newPointer;
	}
	else
	{
		if (oldPtr)
			alloc.deallocate(oldPtr, oldSize);

		return alloc.allocate(newSize, NULLPTR);
	}
}

/// \brief Allocates a block of memory with cleanup
/// \tparam T class or type
/// \tparam T_Align16 boolean that determines whether allocations should be aligned on a 16-byte boundary
/// \details If T_Align16 is true, then AllocatorWithCleanup calls AlignedAllocate()
///  for memory allocations. If T_Align16 is false, then AllocatorWithCleanup() calls
///  UnalignedAllocate() for memory allocations.
/// \details Template parameter T_Align16 is effectively controlled by cryptlib.h and mirrors
///  CRYPTOPP_BOOL_ALIGN16. CRYPTOPP_BOOL_ALIGN16 is often used as the template parameter.
template <class T, bool T_Align16 = false>
class AllocatorWithCleanup : public AllocatorBase<T>
{
public:
	CRYPTOPP_INHERIT_ALLOCATOR_TYPES

	/// \brief Allocates a block of memory
	/// \param ptr the size of the allocation
	/// \param size the size of the allocation, in elements
	/// \returns a memory block
	/// \throws InvalidArgument
	/// \details allocate() first checks the size of the request. If it is non-0
	///  and less than max_size(), then an attempt is made to fulfill the request
	///  using either AlignedAllocate() or UnalignedAllocate(). AlignedAllocate() is
	///  used if T_Align16 is true. UnalignedAllocate() used if T_Align16 is false.
	/// \details This is the C++ *Placement New* operator. ptr is not used, and the
	///  function asserts in Debug builds if ptr is non-NULL.
	/// \sa CallNewHandler() for the methods used to recover from a failed
	///  allocation attempt.
	/// \note size is the count of elements, and not the number of bytes
	pointer allocate(size_type size, const void *ptr = NULLPTR)
	{
		CRYPTOPP_UNUSED(ptr); CRYPTOPP_ASSERT(ptr == NULLPTR);
		this->CheckSize(size);
		if (size == 0)
			return NULLPTR;

#if CRYPTOPP_BOOL_ALIGN16
		if (T_Align16)
			return reinterpret_cast<pointer>(AlignedAllocate(size*sizeof(T)));
#endif

		return reinterpret_cast<pointer>(UnalignedAllocate(size*sizeof(T)));
	}

	/// \brief Deallocates a block of memory
	/// \param ptr the pointer for the allocation
	/// \param size the size of the allocation, in elements
	/// \details Internally, SecureWipeArray() is called before deallocating the
	///  memory. Once the memory block is wiped or zeroized, AlignedDeallocate()
	///  or UnalignedDeallocate() is called.
	/// \details AlignedDeallocate() is used if T_Align16 is true.
	///  UnalignedDeallocate() used if T_Align16 is false.
	void deallocate(void *ptr, size_type size)
	{
		// Avoid assert on pointer in deallocate. SecBlock regularly uses NULL
		// pointers rather returning non-NULL 0-sized pointers.
		if (ptr)
		{
			SecureWipeArray(reinterpret_cast<pointer>(ptr), size);

#if CRYPTOPP_BOOL_ALIGN16
			if (T_Align16)
				return AlignedDeallocate(ptr);
#endif

			UnalignedDeallocate(ptr);
		}
	}

	/// \brief Reallocates a block of memory
	/// \param oldPtr the previous allocation
	/// \param oldSize the size of the previous allocation
	/// \param newSize the new, requested size
	/// \param preserve flag that indicates if the old allocation should be preserved
	/// \returns pointer to the new memory block
	/// \details Internally, reallocate() calls StandardReallocate().
	/// \details If preserve is true, then index 0 is used to begin copying the
	///  old memory block to the new one. If the block grows, then the old array
	///  is copied in its entirety. If the block shrinks, then only newSize
	///  elements are copied from the old block to the new one.
	/// \note oldSize and newSize are the count of elements, and not the
	///  number of bytes.
	pointer reallocate(T *oldPtr, size_type oldSize, size_type newSize, bool preserve)
	{
		CRYPTOPP_ASSERT((oldPtr && oldSize) || !(oldPtr || oldSize));
		return StandardReallocate(*this, oldPtr, oldSize, newSize, preserve);
	}

	/// \brief Template class member Rebind
	/// \tparam V bound class or type
	/// \details Rebind allows a container class to allocate a different type of object
	///  to store elements. For example, a std::list will allocate std::list_node to
	///  store elements in the list.
	/// \details VS.NET STL enforces the policy of "All STL-compliant allocators
	///  have to provide a template class member called rebind".
    template <class V> struct rebind { typedef AllocatorWithCleanup<V, T_Align16> other; };
#if _MSC_VER >= 1500
	AllocatorWithCleanup() {}
	template <class V, bool A> AllocatorWithCleanup(const AllocatorWithCleanup<V, A> &) {}
#endif
};

CRYPTOPP_DLL_TEMPLATE_CLASS AllocatorWithCleanup<byte>;
CRYPTOPP_DLL_TEMPLATE_CLASS AllocatorWithCleanup<word16>;
CRYPTOPP_DLL_TEMPLATE_CLASS AllocatorWithCleanup<word32>;
CRYPTOPP_DLL_TEMPLATE_CLASS AllocatorWithCleanup<word64>;
#if defined(CRYPTOPP_WORD128_AVAILABLE)
CRYPTOPP_DLL_TEMPLATE_CLASS AllocatorWithCleanup<word128, true>; // for Integer
#endif
#if CRYPTOPP_BOOL_X86
CRYPTOPP_DLL_TEMPLATE_CLASS AllocatorWithCleanup<word, true>;	 // for Integer
#endif

/// \brief NULL allocator
/// \tparam T class or type
/// \details A NullAllocator is useful for fixed-size, stack based allocations
///  (i.e., static arrays used by FixedSizeAllocatorWithCleanup).
/// \details A NullAllocator always returns 0 for max_size(), and always returns
///  NULL for allocation requests. Though the allocator does not allocate at
///  runtime, it does perform a secure wipe or zeroization during cleanup.
template <class T>
class NullAllocator : public AllocatorBase<T>
{
public:
	//LCOV_EXCL_START
	CRYPTOPP_INHERIT_ALLOCATOR_TYPES

	// TODO: should this return NULL or throw bad_alloc? Non-Windows C++ standard
	// libraries always throw. And late mode Windows throws. Early model Windows
	// (circa VC++ 6.0) returned NULL.
	pointer allocate(size_type n, const void* unused = NULLPTR)
	{
		CRYPTOPP_UNUSED(n); CRYPTOPP_UNUSED(unused);
		CRYPTOPP_ASSERT(false); return NULLPTR;
	}

	void deallocate(void *p, size_type n)
	{
		CRYPTOPP_UNUSED(p); CRYPTOPP_UNUSED(n);
		CRYPTOPP_ASSERT(false);
	}

	CRYPTOPP_CONSTEXPR size_type max_size() const {return 0;}
	//LCOV_EXCL_STOP
};

/// \brief Static secure memory block with cleanup
/// \tparam T class or type
/// \tparam S fixed-size of the stack-based memory block, in elements
/// \tparam T_Align16 boolean that determines whether allocations should
///  be aligned on a 16-byte boundary
/// \details FixedSizeAllocatorWithCleanup provides a fixed-size, stack-
///  based allocation at compile time. The class can grow its memory
///  block at runtime if a suitable allocator is available. If size
///  grows beyond S and a suitable allocator is available, then the
///  statically allocated array is obsoleted.
/// \note This allocator can't be used with standard collections because
///  they require that all objects of the same allocator type are equivalent.
template <class T, size_t S, class A = NullAllocator<T>, bool T_Align16 = false>
class FixedSizeAllocatorWithCleanup : public AllocatorBase<T>
{
	// The body of FixedSizeAllocatorWithCleanup is provided in the two
	// partial specializations that follow. The two specialiations
	// pivot on the boolean template parameter T_Align16. AIX, Solaris,
	// IBM XLC and SunCC receive a little extra help. We managed to
	// clear most of the warnings.
};

/// \brief Static secure memory block with cleanup
/// \tparam T class or type
/// \tparam S fixed-size of the stack-based memory block, in elements
/// \details FixedSizeAllocatorWithCleanup provides a fixed-size, stack-
///  based allocation at compile time. The class can grow its memory
///  block at runtime if a suitable allocator is available. If size
///  grows beyond S and a suitable allocator is available, then the
///  statically allocated array is obsoleted.
/// \note This allocator can't be used with standard collections because
///  they require that all objects of the same allocator type are equivalent.
template <class T, size_t S, class A>
class FixedSizeAllocatorWithCleanup<T, S, A, true> : public AllocatorBase<T>
{
public:
	CRYPTOPP_INHERIT_ALLOCATOR_TYPES

	/// \brief Constructs a FixedSizeAllocatorWithCleanup
	FixedSizeAllocatorWithCleanup() : m_allocated(false) {}

	/// \brief Allocates a block of memory
	/// \param size the count elements in the memory block
	/// \details FixedSizeAllocatorWithCleanup provides a fixed-size, stack-based
	///  allocation at compile time. If size is less than or equal to
	///  <tt>S</tt>, then a pointer to the static array is returned.
	/// \details The class can grow its memory block at runtime if a suitable
	///  allocator is available. If size grows beyond S and a suitable
	///  allocator is available, then the statically allocated array is
	///  obsoleted. If a suitable allocator is not available, as with a
	///  NullAllocator, then the function returns NULL and a runtime error
	///  eventually occurs.
	/// \sa reallocate(), SecBlockWithHint
	pointer allocate(size_type size)
	{
		CRYPTOPP_ASSERT(IsAlignedOn(m_array, 8));

		if (size <= S && !m_allocated)
		{
			m_allocated = true;
			return GetAlignedArray();
		}
		else
			return m_fallbackAllocator.allocate(size);
	}

	/// \brief Allocates a block of memory
	/// \param size the count elements in the memory block
	/// \param hint an unused hint
	/// \details FixedSizeAllocatorWithCleanup provides a fixed-size, stack-
	///  based allocation at compile time. If size is less than or equal to
	///  S, then a pointer to the static array is returned.
	/// \details The class can grow its memory block at runtime if a suitable
	///  allocator is available. If size grows beyond S and a suitable
	///  allocator is available, then the statically allocated array is
	///  obsoleted. If a suitable allocator is not available, as with a
	///  NullAllocator, then the function returns NULL and a runtime error
	///  eventually occurs.
	/// \sa reallocate(), SecBlockWithHint
	pointer allocate(size_type size, const void *hint)
	{
		if (size <= S && !m_allocated)
		{
			m_allocated = true;
			return GetAlignedArray();
		}
		else
			return m_fallbackAllocator.allocate(size, hint);
	}

	/// \brief Deallocates a block of memory
	/// \param ptr a pointer to the memory block to deallocate
	/// \param size the count elements in the memory block
	/// \details The memory block is wiped or zeroized before deallocation.
	///  If the statically allocated memory block is active, then no
	///  additional actions are taken after the wipe.
	/// \details If a dynamic memory block is active, then the pointer and
	///  size are passed to the allocator for deallocation.
	void deallocate(void *ptr, size_type size)
	{
		// Avoid assert on pointer in deallocate. SecBlock regularly uses NULL
		// pointers rather returning non-NULL 0-sized pointers.
		if (ptr == GetAlignedArray())
		{
			// If the m_allocated assert fires then the bit twiddling for
			// GetAlignedArray() is probably incorrect for the platform.
			// Be sure to check CRYPTOPP_ALIGN_DATA(8). The platform may
			// not have a way to declaritively align data to 8.
			CRYPTOPP_ASSERT(size <= S);
			CRYPTOPP_ASSERT(m_allocated);
			m_allocated = false;
			SecureWipeArray(reinterpret_cast<pointer>(ptr), size);
		}
		else
		{
			if (ptr)
				m_fallbackAllocator.deallocate(ptr, size);
		}
	}

	/// \brief Reallocates a block of memory
	/// \param oldPtr the previous allocation
	/// \param oldSize the size of the previous allocation
	/// \param newSize the new, requested size
	/// \param preserve flag that indicates if the old allocation should
	///  be preserved
	/// \returns pointer to the new memory block
	/// \details FixedSizeAllocatorWithCleanup provides a fixed-size, stack-
	///  based allocation at compile time. If size is less than or equal to
	///  S, then a pointer to the static array is returned.
	/// \details The class can grow its memory block at runtime if a suitable
	///  allocator is available. If size grows beyond S and a suitable
	///  allocator is available, then the statically allocated array is
	///  obsoleted. If a suitable allocator is not available, as with a
	///  NullAllocator, then the function returns NULL and a runtime error
	///  eventually occurs.
	/// \note size is the count of elements, and not the number of bytes.
	/// \sa reallocate(), SecBlockWithHint
	pointer reallocate(pointer oldPtr, size_type oldSize, size_type newSize, bool preserve)
	{
		if (oldPtr == GetAlignedArray() && newSize <= S)
		{
			CRYPTOPP_ASSERT(oldSize <= S);
			if (oldSize > newSize)
				SecureWipeArray(oldPtr+newSize, oldSize-newSize);
			return oldPtr;
		}

		pointer newPointer = allocate(newSize, NULLPTR);
		if (preserve && newSize)
		{
			const size_type copySize = STDMIN(oldSize, newSize);
			if (newPointer && oldPtr)  // GCC analyzer warning
				memcpy_s(newPointer, sizeof(T)*newSize, oldPtr, sizeof(T)*copySize);
		}
		deallocate(oldPtr, oldSize);
		return newPointer;
	}

	CRYPTOPP_CONSTEXPR size_type max_size() const
	{
		return STDMAX(m_fallbackAllocator.max_size(), S);
	}

private:

#if defined(CRYPTOPP_BOOL_ALIGN16) && (defined(_M_X64) || defined(__x86_64__))
	// Before we can add additional platforms we need to check the
	// linker documentation for alignment behavior for stack variables.
	// CRYPTOPP_ALIGN_DATA(16) is known OK on Linux, OS X, Solaris.
	// Also see http://stackoverflow.com/a/1468656/608639.
	T* GetAlignedArray() {
		CRYPTOPP_ASSERT(IsAlignedOn(m_array, 16));
		return m_array;
	}
	CRYPTOPP_ALIGN_DATA(16) T m_array[S];

#elif defined(CRYPTOPP_BOOL_ALIGN16)

	// There be demons here... We cannot use CRYPTOPP_ALIGN_DATA(16)
	// because linkers on 32-bit machines (and some 64-bit machines)
	// align the stack to 8-bytes or less by default, not 16-bytes as
	// requested. Additionally, the AIX linker seems to use 4-bytes
	// by default. However, all linkers tested appear to honor
	// CRYPTOPP_ALIGN_DATA(8). Also see
	// http://stackoverflow.com/a/1468656/608639.
	//
	// The 16-byte alignment is achieved by padding the requested
	// size with extra elements so we have at least 16-bytes of slack
	// to work with. Then the pointer is moved down to achieve a
	// 16-byte alignment (stacks grow down).
	//
	// The additional 16-bytes introduces a small secondary issue.
	// The secondary issue is, a large T results in 0 = 8/sizeof(T).
	// The library is OK but users may hit it. So we need to guard
	// for a large T, and that is what PAD achieves.
	T* GetAlignedArray() {
		T* p_array = reinterpret_cast<T*>(static_cast<void*>((reinterpret_cast<byte*>(m_array)) + (0-reinterpret_cast<size_t>(m_array))%16));
		// Verify the 16-byte alignment
		CRYPTOPP_ASSERT(IsAlignedOn(p_array, 16));
		// Verify allocated array with pad is large enough.
		CRYPTOPP_ASSERT(p_array+S <= m_array+(S+PAD));
		return p_array;
	}

#   if defined(_AIX)
	// PAD is elements, not bytes, and rounded up to ensure no overflow.
	enum { Q = sizeof(T), PAD = (Q >= 16) ? 1 : (Q >= 8) ? 2 : (Q >= 4) ? 4 : (Q >= 2) ? 8 : 16 };
	CRYPTOPP_ALIGN_DATA(8) T m_array[S+PAD];
#   else
	// PAD is elements, not bytes, and rounded up to ensure no overflow.
	enum { Q = sizeof(T), PAD = (Q >= 8) ? 1 : (Q >= 4) ? 2 : (Q >= 2) ? 4 : 8 };
	CRYPTOPP_ALIGN_DATA(8) T m_array[S+PAD];
#   endif

#else

	// CRYPTOPP_BOOL_ALIGN16 is 0. Use natural alignment of T.
	T* GetAlignedArray() {return m_array;}
	CRYPTOPP_ALIGN_DATA(4) T m_array[S];

#endif

	A m_fallbackAllocator;
	bool m_allocated;
};

/// \brief Static secure memory block with cleanup
/// \tparam T class or type
/// \tparam S fixed-size of the stack-based memory block, in elements
/// \details FixedSizeAllocatorWithCleanup provides a fixed-size, stack-
///  based allocation at compile time. The class can grow its memory
///  block at runtime if a suitable allocator is available. If size
///  grows beyond S and a suitable allocator is available, then the
///  statically allocated array is obsoleted.
/// \note This allocator can't be used with standard collections because
///  they require that all objects of the same allocator type are equivalent.
template <class T, size_t S, class A>
class FixedSizeAllocatorWithCleanup<T, S, A, false> : public AllocatorBase<T>
{
public:
	CRYPTOPP_INHERIT_ALLOCATOR_TYPES

	/// \brief Constructs a FixedSizeAllocatorWithCleanup
	FixedSizeAllocatorWithCleanup() : m_allocated(false) {}

	/// \brief Allocates a block of memory
	/// \param size the count elements in the memory block
	/// \details FixedSizeAllocatorWithCleanup provides a fixed-size, stack-based
	///  allocation at compile time. If size is less than or equal to
	///  <tt>S</tt>, then a pointer to the static array is returned.
	/// \details The class can grow its memory block at runtime if a suitable
	///  allocator is available. If size grows beyond S and a suitable
	///  allocator is available, then the statically allocated array is
	///  obsoleted. If a suitable allocator is not available, as with a
	///  NullAllocator, then the function returns NULL and a runtime error
	///  eventually occurs.
	/// \sa reallocate(), SecBlockWithHint
	pointer allocate(size_type size)
	{
		CRYPTOPP_ASSERT(IsAlignedOn(m_array, 8));

		if (size <= S && !m_allocated)
		{
			m_allocated = true;
			return GetAlignedArray();
		}
		else
			return m_fallbackAllocator.allocate(size);
	}

	/// \brief Allocates a block of memory
	/// \param size the count elements in the memory block
	/// \param hint an unused hint
	/// \details FixedSizeAllocatorWithCleanup provides a fixed-size, stack-
	///  based allocation at compile time. If size is less than or equal to
	///  S, then a pointer to the static array is returned.
	/// \details The class can grow its memory block at runtime if a suitable
	///  allocator is available. If size grows beyond S and a suitable
	///  allocator is available, then the statically allocated array is
	///  obsoleted. If a suitable allocator is not available, as with a
	///  NullAllocator, then the function returns NULL and a runtime error
	///  eventually occurs.
	/// \sa reallocate(), SecBlockWithHint
	pointer allocate(size_type size, const void *hint)
	{
		if (size <= S && !m_allocated)
		{
			m_allocated = true;
			return GetAlignedArray();
		}
		else
			return m_fallbackAllocator.allocate(size, hint);
	}

	/// \brief Deallocates a block of memory
	/// \param ptr a pointer to the memory block to deallocate
	/// \param size the count elements in the memory block
	/// \details The memory block is wiped or zeroized before deallocation.
	///  If the statically allocated memory block is active, then no
	///  additional actions are taken after the wipe.
	/// \details If a dynamic memory block is active, then the pointer and
	///   size are passed to the allocator for deallocation.
	void deallocate(void *ptr, size_type size)
	{
		// Avoid assert on pointer in deallocate. SecBlock regularly uses NULL
		// pointers rather returning non-NULL 0-sized pointers.
		if (ptr == GetAlignedArray())
		{
			// If the m_allocated assert fires then
			// something overwrote the flag.
			CRYPTOPP_ASSERT(size <= S);
			CRYPTOPP_ASSERT(m_allocated);
			m_allocated = false;
			SecureWipeArray((pointer)ptr, size);
		}
		else
		{
			if (ptr)
				m_fallbackAllocator.deallocate(ptr, size);
			m_allocated = false;
		}
	}

	/// \brief Reallocates a block of memory
	/// \param oldPtr the previous allocation
	/// \param oldSize the size of the previous allocation
	/// \param newSize the new, requested size
	/// \param preserve flag that indicates if the old allocation should
	///  be preserved
	/// \returns pointer to the new memory block
	/// \details FixedSizeAllocatorWithCleanup provides a fixed-size, stack-
	///  based allocation at compile time. If size is less than or equal to
	///  S, then a pointer to the static array is returned.
	/// \details The class can grow its memory block at runtime if a suitable
	///  allocator is available. If size grows beyond S and a suitable
	///  allocator is available, then the statically allocated array is
	///  obsoleted. If a suitable allocator is not available, as with a
	///  NullAllocator, then the function returns NULL and a runtime error
	///  eventually occurs.
	/// \note size is the count of elements, and not the number of bytes.
	/// \sa reallocate(), SecBlockWithHint
	pointer reallocate(pointer oldPtr, size_type oldSize, size_type newSize, bool preserve)
	{
		if (oldPtr == GetAlignedArray() && newSize <= S)
		{
			CRYPTOPP_ASSERT(oldSize <= S);
			if (oldSize > newSize)
				SecureWipeArray(oldPtr+newSize, oldSize-newSize);
			return oldPtr;
		}

		pointer newPointer = allocate(newSize, NULLPTR);
		if (preserve && newSize)
		{
			const size_type copySize = STDMIN(oldSize, newSize);
			if (newPointer && oldPtr)  // GCC analyzer warning
				memcpy_s(newPointer, sizeof(T)*newSize, oldPtr, sizeof(T)*copySize);
		}
		deallocate(oldPtr, oldSize);
		return newPointer;
	}

	CRYPTOPP_CONSTEXPR size_type max_size() const
	{
		return STDMAX(m_fallbackAllocator.max_size(), S);
	}

private:

	// The 8-byte alignments follows convention of Linux and Windows.
	// Linux and Windows receives most testing. Duplicate it here for
	// other platforms like AIX and Solaris. AIX and Solaris often use
	// alignments smaller than expected. In fact AIX caught us by
	// surprise with word16 and word32.
	T* GetAlignedArray() {return m_array;}
	CRYPTOPP_ALIGN_DATA(8) T m_array[S];

	A m_fallbackAllocator;
	bool m_allocated;
};

/// \brief Secure memory block with allocator and cleanup
/// \tparam T a class or type
/// \tparam A AllocatorWithCleanup derived class for allocation and cleanup
template <class T, class A = AllocatorWithCleanup<T> >
class SecBlock
{
public:
	typedef typename A::value_type value_type;
	typedef typename A::pointer iterator;
	typedef typename A::const_pointer const_iterator;
	typedef typename A::size_type size_type;

	/// \brief Returns the maximum number of elements the block can hold
	/// \details <tt>ELEMS_MAX</tt> is the maximum number of elements the
	///  <tt>SecBlock</tt> can hold. The value of <tt>ELEMS_MAX</tt> is
	///  <tt>SIZE_MAX/sizeof(T)</tt>. <tt>std::numeric_limits</tt> was avoided
	///  due to lack of <tt>constexpr</tt>-ness in C++03 and below.
	/// \note In C++03 and below <tt>ELEMS_MAX</tt> is a static data member of type
	///  <tt>size_type</tt>. In C++11 and above <tt>ELEMS_MAX</tt> is an <tt>enum</tt>
	///  inheriting from <tt>size_type</tt>. In both cases <tt>ELEMS_MAX</tt> can be
	///  used before objects are fully constructed, and it does not suffer the
	///  limitations of class methods like <tt>max_size</tt>.
	/// \sa <A HREF="http://github.com/weidai11/cryptopp/issues/346">Issue 346/CVE-2016-9939</A>
	/// \since Crypto++ 6.0
#if defined(CRYPTOPP_DOXYGEN_PROCESSING)
	static const size_type ELEMS_MAX = ...;
#elif defined(_MSC_VER) && (_MSC_VER <= 1400)
	static const size_type ELEMS_MAX = (~(size_type)0)/sizeof(T);
#elif defined(CRYPTOPP_CXX11_STRONG_ENUM)
	enum : size_type {ELEMS_MAX = A::ELEMS_MAX};
#else
	static const size_type ELEMS_MAX = SIZE_MAX/sizeof(T);
#endif

	/// \brief Construct a SecBlock with space for size elements.
	/// \param size the size of the allocation, in elements
	/// \throws std::bad_alloc
	/// \details The elements are not initialized.
	/// \note size is the count of elements, and not the number of bytes
	explicit SecBlock(size_type size=0)
		: m_mark(ELEMS_MAX), m_size(size), m_ptr(m_alloc.allocate(size, NULLPTR)) { }

	/// \brief Copy construct a SecBlock from another SecBlock
	/// \param t the other SecBlock
	/// \throws std::bad_alloc
	SecBlock(const SecBlock<T, A> &t)
		: m_mark(t.m_mark), m_size(t.m_size), m_ptr(m_alloc.allocate(t.m_size, NULLPTR)) {
			CRYPTOPP_ASSERT((!t.m_ptr && !m_size) || (t.m_ptr && m_size));
			if (m_ptr && t.m_ptr)
				memcpy_s(m_ptr, m_size*sizeof(T), t.m_ptr, t.m_size*sizeof(T));
		}

	/// \brief Construct a SecBlock from an array of elements.
	/// \param ptr a pointer to an array of T
	/// \param len the number of elements in the memory block
	/// \throws std::bad_alloc
	/// \details If <tt>ptr!=NULL</tt> and <tt>len!=0</tt>, then the block is initialized from the pointer
	///  <tt>ptr</tt>. If <tt>ptr==NULL</tt> and <tt>len!=0</tt>, then the block is initialized to 0.
	///  Otherwise, the block is empty and not initialized.
	/// \note size is the count of elements, and not the number of bytes
	SecBlock(const T *ptr, size_type len)
		: m_mark(ELEMS_MAX), m_size(len), m_ptr(m_alloc.allocate(len, NULLPTR)) {
			CRYPTOPP_ASSERT((!m_ptr && !m_size) || (m_ptr && m_size));
			if (m_ptr && ptr)
				memcpy_s(m_ptr, m_size*sizeof(T), ptr, len*sizeof(T));
			else if (m_ptr && m_size)
				memset(m_ptr, 0, m_size*sizeof(T));
		}

	~SecBlock()
		{m_alloc.deallocate(m_ptr, STDMIN(m_size, m_mark));}

#ifdef __BORLANDC__
	/// \brief Cast operator
        /// \returns block pointer cast to non-const <tt>T *</tt>
	operator T *() const
		{return (T*)m_ptr;}
#else
	/// \brief Cast operator
        /// \returns block pointer cast to <tt>const void *</tt>
	operator const void *() const
		{return m_ptr;}

	/// \brief Cast operator
        /// \returns block pointer cast to non-const <tt>void *</tt>
	operator void *()
		{return m_ptr;}

	/// \brief Cast operator
        /// \returns block pointer cast to <tt>const T *</tt>
	operator const T *() const
		{return m_ptr;}

	/// \brief Cast operator
        /// \returns block pointer cast to non-const <tt>T *</tt>
	operator T *()
		{return m_ptr;}
#endif

	/// \brief Provides an iterator pointing to the first element in the memory block
	/// \returns iterator pointing to the first element in the memory block
	iterator begin()
		{return m_ptr;}
	/// \brief Provides a constant iterator pointing to the first element in the memory block
	/// \returns constant iterator pointing to the first element in the memory block
	const_iterator begin() const
		{return m_ptr;}
	/// \brief Provides an iterator pointing beyond the last element in the memory block
	/// \returns iterator pointing beyond the last element in the memory block
	iterator end()
		{return m_ptr+m_size;}
	/// \brief Provides a constant iterator pointing beyond the last element in the memory block
	/// \returns constant iterator pointing beyond the last element in the memory block
	const_iterator end() const
		{return m_ptr+m_size;}

	/// \brief Provides a pointer to the first element in the memory block
	/// \returns pointer to the first element in the memory block
	typename A::pointer data() {return m_ptr;}
	/// \brief Provides a pointer to the first element in the memory block
	/// \returns constant pointer to the first element in the memory block
	typename A::const_pointer data() const {return m_ptr;}

	/// \brief Provides the count of elements in the SecBlock
	/// \returns number of elements in the memory block
	/// \note the return value is the count of elements, and not the number of bytes
	size_type size() const {return m_size;}
	/// \brief Determines if the SecBlock is empty
	/// \returns true if number of elements in the memory block is 0, false otherwise
	bool empty() const {return m_size == 0;}

	/// \brief Provides a byte pointer to the first element in the memory block
	/// \returns byte pointer to the first element in the memory block
	byte * BytePtr() {return (byte *)m_ptr;}
	/// \brief Return a byte pointer to the first element in the memory block
	/// \returns constant byte pointer to the first element in the memory block
	const byte * BytePtr() const {return (const byte *)m_ptr;}
	/// \brief Provides the number of bytes in the SecBlock
	/// \return the number of bytes in the memory block
	/// \note the return value is the number of bytes, and not count of elements.
	size_type SizeInBytes() const {return m_size*sizeof(T);}

	/// \brief Sets the number of elements to zeroize
	/// \param count the number of elements
	/// \details SetMark is a remediation for Issue 346/CVE-2016-9939 while
	///  preserving the streaming interface. The <tt>count</tt> controls the number of
	///  elements zeroized, which can be less than <tt>size</tt> or 0.
	/// \details An internal variable, <tt>m_mark</tt>, is initialized to the maximum number
	///  of elements. The maximum number of elements is <tt>ELEMS_MAX</tt>. Deallocation
	///  triggers a zeroization, and the number of elements zeroized is
	///  <tt>STDMIN(m_size, m_mark)</tt>. After zeroization, the memory is returned to the
	///  system.
	/// \details The ASN.1 decoder uses SetMark() to set the element count to 0
	///  before throwing an exception. In this case, the attacker provides a large
	///  BER encoded length (say 64MB) but only a small number of content octets
	///  (say 16). If the allocator zeroized all 64MB, then a transient DoS could
	///  occur as CPU cycles are spent zeroizing unintialized memory.
	/// \details Generally speaking, any operation which changes the size of the SecBlock
	///  results in the mark being reset to <tt>ELEMS_MAX</tt>. In particular, if Assign(),
	///  New(), Grow(), CleanNew(), CleanGrow() are called, then the count is reset to
	///  <tt>ELEMS_MAX</tt>. The list is not exhaustive.
	/// \since Crypto++ 6.0
	/// \sa <A HREF="http://github.com/weidai11/cryptopp/issues/346">Issue 346/CVE-2016-9939</A>
	void SetMark(size_t count) {m_mark = count;}

	/// \brief Set contents and size from an array
	/// \param ptr a pointer to an array of T
	/// \param len the number of elements in the memory block
	/// \details If the memory block is reduced in size, then the reclaimed memory is set to 0.
	///  Assign() resets the element count after the previous block is zeroized.
	void Assign(const T *ptr, size_type len)
	{
		New(len);
		if (m_ptr && ptr)  // GCC analyzer warning
			memcpy_s(m_ptr, m_size*sizeof(T), ptr, len*sizeof(T));
		m_mark = ELEMS_MAX;
	}

	/// \brief Set contents from a value
	/// \param count the number of values to copy
	/// \param value the value, repeated count times
	/// \details If the memory block is reduced in size, then the reclaimed memory is set to 0.
	///  Assign() resets the element count after the previous block is zeroized.
	void Assign(size_type count, T value)
	{
		New(count);
		for (size_t i=0; i<count; ++i)
			m_ptr[i] = value;

		m_mark = ELEMS_MAX;
	}

	/// \brief Copy contents from another SecBlock
	/// \param t the other SecBlock
	/// \details Assign checks for self assignment.
	/// \details If the memory block is reduced in size, then the reclaimed memory is set to 0.
	///  If an assignment occurs, then Assign() resets the element count after the previous block
	///  is zeroized.
	void Assign(const SecBlock<T, A> &t)
	{
		if (this != &t)
		{
			New(t.m_size);
			if (m_ptr && t.m_ptr)  // GCC analyzer warning
				memcpy_s(m_ptr, m_size*sizeof(T), t, t.m_size*sizeof(T));
		}
		m_mark = ELEMS_MAX;
	}

	/// \brief Assign contents from another SecBlock
	/// \param t the other SecBlock
	/// \details Internally, operator=() calls Assign().
	/// \details If the memory block is reduced in size, then the reclaimed memory is set to 0.
	///  If an assignment occurs, then Assign() resets the element count after the previous block
	///  is zeroized.
	SecBlock<T, A>& operator=(const SecBlock<T, A> &t)
	{
		// Assign guards for self-assignment
		Assign(t);
		return *this;
	}

	/// \brief Append contents from another SecBlock
	/// \param t the other SecBlock
	/// \details Internally, this SecBlock calls Grow and then appends t.
	SecBlock<T, A>& operator+=(const SecBlock<T, A> &t)
	{
		CRYPTOPP_ASSERT((!t.m_ptr && !t.m_size) || (t.m_ptr && t.m_size));
		if (t.m_size)
		{
			const size_type oldSize = m_size;
			if (this != &t)  // s += t
			{
				Grow(m_size+t.m_size);
				if (m_ptr && t.m_ptr)  // GCC analyzer warning
					memcpy_s(m_ptr+oldSize, (m_size-oldSize)*sizeof(T), t.m_ptr, t.m_size*sizeof(T));
			}
			else            // t += t
			{
				Grow(m_size*2);
				if (m_ptr && t.m_ptr)  // GCC analyzer warning
					memcpy_s(m_ptr+oldSize, (m_size-oldSize)*sizeof(T), m_ptr, oldSize*sizeof(T));
			}
		}
		m_mark = ELEMS_MAX;
		return *this;
	}

	/// \brief Construct a SecBlock from this and another SecBlock
	/// \param t the other SecBlock
	/// \returns a newly constructed SecBlock that is a conacentation of this and t
	/// \details Internally, a new SecBlock is created from this and a concatenation of t.
	SecBlock<T, A> operator+(const SecBlock<T, A> &t)
	{
		CRYPTOPP_ASSERT((!m_ptr && !m_size) || (m_ptr && m_size));
		CRYPTOPP_ASSERT((!t.m_ptr && !t.m_size) || (t.m_ptr && t.m_size));
		if(!t.m_size) return SecBlock(*this);

		SecBlock<T, A> result(m_size+t.m_size);
		if (m_size)
			memcpy_s(result.m_ptr, result.m_size*sizeof(T), m_ptr, m_size*sizeof(T));
		if (result.m_ptr && t.m_ptr)  // GCC analyzer warning
			memcpy_s(result.m_ptr+m_size, (result.m_size-m_size)*sizeof(T), t.m_ptr, t.m_size*sizeof(T));
		return result;
	}

	/// \brief Bitwise compare two SecBlocks
	/// \param t the other SecBlock
	/// \returns true if the size and bits are equal, false otherwise
	/// \details Uses a constant time compare if the arrays are equal size. The constant time
	///  compare is VerifyBufsEqual() found in misc.h.
	/// \sa operator!=()
	bool operator==(const SecBlock<T, A> &t) const
	{
		return m_size == t.m_size && VerifyBufsEqual(
			reinterpret_cast<const byte*>(m_ptr),
			reinterpret_cast<const byte*>(t.m_ptr), m_size*sizeof(T));
	}

	/// \brief Bitwise compare two SecBlocks
	/// \param t the other SecBlock
	/// \returns true if the size and bits are equal, false otherwise
	/// \details Uses a constant time compare if the arrays are equal size. The constant time
	///  compare is VerifyBufsEqual() found in misc.h.
	/// \details Internally, operator!=() returns the inverse of operator==().
	/// \sa operator==()
	bool operator!=(const SecBlock<T, A> &t) const
	{
		return !operator==(t);
	}

	/// \brief Change size without preserving contents
	/// \param newSize the new size of the memory block
	/// \details Old content is not preserved. If the memory block is reduced in size,
	///  then the reclaimed memory is set to 0. If the memory block grows in size, then
	///  the new memory is not initialized. New() resets the element count after the
	///  previous block is zeroized.
	/// \details Internally, this SecBlock calls reallocate().
	/// \sa New(), CleanNew(), Grow(), CleanGrow(), resize()
	void New(size_type newSize)
	{
		m_ptr = m_alloc.reallocate(m_ptr, m_size, newSize, false);
		m_size = newSize;
		m_mark = ELEMS_MAX;
	}

	/// \brief Change size without preserving contents
	/// \param newSize the new size of the memory block
	/// \details Old content is not preserved. If the memory block is reduced in size,
	///  then the reclaimed content is set to 0. If the memory block grows in size, then
	///  the new memory is initialized to 0. CleanNew() resets the element count after the
	///  previous block is zeroized.
	/// \details Internally, this SecBlock calls New().
	/// \sa New(), CleanNew(), Grow(), CleanGrow(), resize()
	void CleanNew(size_type newSize)
	{
		New(newSize);
		if (m_ptr) {memset_z(m_ptr, 0, m_size*sizeof(T));}
		m_mark = ELEMS_MAX;
	}

	/// \brief Change size and preserve contents
	/// \param newSize the new size of the memory block
	/// \details Old content is preserved. New content is not initialized.
	/// \details Internally, this SecBlock calls reallocate() when size must increase. If the
	///  size does not increase, then Grow() does not take action. If the size must
	///  change, then use resize(). Grow() resets the element count after the
	///  previous block is zeroized.
	/// \sa New(), CleanNew(), Grow(), CleanGrow(), resize()
	void Grow(size_type newSize)
	{
		if (newSize > m_size)
		{
			m_ptr = m_alloc.reallocate(m_ptr, m_size, newSize, true);
			m_size = newSize;
		}
		m_mark = ELEMS_MAX;
	}

	/// \brief Change size and preserve contents
	/// \param newSize the new size of the memory block
	/// \details Old content is preserved. New content is initialized to 0.
	/// \details Internally, this SecBlock calls reallocate() when size must increase. If the
	///  size does not increase, then CleanGrow() does not take action. If the size must
	///  change, then use resize(). CleanGrow() resets the element count after the
	///  previous block is zeroized.
	/// \sa New(), CleanNew(), Grow(), CleanGrow(), resize()
	void CleanGrow(size_type newSize)
	{
		if (newSize > m_size)
		{
			m_ptr = m_alloc.reallocate(m_ptr, m_size, newSize, true);
			memset_z(m_ptr+m_size, 0, (newSize-m_size)*sizeof(T));
			m_size = newSize;
		}
		m_mark = ELEMS_MAX;
	}

	/// \brief Change size and preserve contents
	/// \param newSize the new size of the memory block
	/// \details Old content is preserved. If the memory block grows in size, then
	///  new memory is not initialized. resize() resets the element count after
	///  the previous block is zeroized.
	/// \details Internally, this SecBlock calls reallocate().
	/// \sa New(), CleanNew(), Grow(), CleanGrow(), resize()
	void resize(size_type newSize)
	{
		m_ptr = m_alloc.reallocate(m_ptr, m_size, newSize, true);
		m_size = newSize;
		m_mark = ELEMS_MAX;
	}

	/// \brief Swap contents with another SecBlock
	/// \param b the other SecBlock
	/// \details Internally, std::swap() is called on m_alloc, m_size and m_ptr.
	void swap(SecBlock<T, A> &b)
	{
		// Swap must occur on the allocator in case its FixedSize that spilled into the heap.
		std::swap(m_alloc, b.m_alloc);
		std::swap(m_mark, b.m_mark);
		std::swap(m_size, b.m_size);
		std::swap(m_ptr, b.m_ptr);
	}

protected:
	A m_alloc;
	size_type m_mark, m_size;
	T *m_ptr;
};

#ifdef CRYPTOPP_DOXYGEN_PROCESSING
/// \brief \ref SecBlock "SecBlock<byte>" typedef.
class SecByteBlock : public SecBlock<byte> {};
/// \brief \ref SecBlock "SecBlock<word>" typedef.
class SecWordBlock : public SecBlock<word> {};
/// \brief SecBlock using \ref AllocatorWithCleanup "AllocatorWithCleanup<byte, true>" typedef
class AlignedSecByteBlock : public SecBlock<byte, AllocatorWithCleanup<byte, true> > {};
#else
typedef SecBlock<byte> SecByteBlock;
typedef SecBlock<word> SecWordBlock;
typedef SecBlock<byte, AllocatorWithCleanup<byte, true> > AlignedSecByteBlock;
#endif

// No need for move semantics on derived class *if* the class does not add any
// data members; see http://stackoverflow.com/q/31755703, and Rule of {0|3|5}.

/// \brief Fixed size stack-based SecBlock
/// \tparam T class or type
/// \tparam S fixed-size of the stack-based memory block, in elements
/// \tparam A AllocatorBase derived class for allocation and cleanup
template <class T, unsigned int S, class A = FixedSizeAllocatorWithCleanup<T, S> >
class FixedSizeSecBlock : public SecBlock<T, A>
{
public:
	/// \brief Construct a FixedSizeSecBlock
	explicit FixedSizeSecBlock() : SecBlock<T, A>(S) {}
};

/// \brief Fixed size stack-based SecBlock with 16-byte alignment
/// \tparam T class or type
/// \tparam S fixed-size of the stack-based memory block, in elements
/// \tparam T_Align16 boolean that determines whether allocations should be aligned on a 16-byte boundary
template <class T, unsigned int S, bool T_Align16 = true>
class FixedSizeAlignedSecBlock : public FixedSizeSecBlock<T, S, FixedSizeAllocatorWithCleanup<T, S, NullAllocator<T>, T_Align16> >
{
};

/// \brief Stack-based SecBlock that grows into the heap
/// \tparam T class or type
/// \tparam S fixed-size of the stack-based memory block, in elements
/// \tparam A AllocatorBase derived class for allocation and cleanup
template <class T, unsigned int S, class A = FixedSizeAllocatorWithCleanup<T, S, AllocatorWithCleanup<T> > >
class SecBlockWithHint : public SecBlock<T, A>
{
public:
	/// construct a SecBlockWithHint with a count of elements
	explicit SecBlockWithHint(size_t size) : SecBlock<T, A>(size) {}
};

template<class T, bool A, class V, bool B>
inline bool operator==(const CryptoPP::AllocatorWithCleanup<T, A>&, const CryptoPP::AllocatorWithCleanup<V, B>&) {return (true);}
template<class T, bool A, class V, bool B>
inline bool operator!=(const CryptoPP::AllocatorWithCleanup<T, A>&, const CryptoPP::AllocatorWithCleanup<V, B>&) {return (false);}

NAMESPACE_END

NAMESPACE_BEGIN(std)

/// \brief Swap two SecBlocks
/// \tparam T class or type
/// \tparam A AllocatorBase derived class for allocation and cleanup
/// \param a the first SecBlock
/// \param b the second SecBlock
template <class T, class A>
inline void swap(CryptoPP::SecBlock<T, A> &a, CryptoPP::SecBlock<T, A> &b)
{
	a.swap(b);
}

#if defined(_STLP_DONT_SUPPORT_REBIND_MEMBER_TEMPLATE) || (defined(_STLPORT_VERSION) && !defined(_STLP_MEMBER_TEMPLATE_CLASSES))
// working for STLport 5.1.3 and MSVC 6 SP5
template <class _Tp1, class _Tp2>
inline CryptoPP::AllocatorWithCleanup<_Tp2>&
__stl_alloc_rebind(CryptoPP::AllocatorWithCleanup<_Tp1>& __a, const _Tp2*)
{
	return (CryptoPP::AllocatorWithCleanup<_Tp2>&)(__a);
}
#endif

NAMESPACE_END

#if CRYPTOPP_MSC_VERSION
# pragma warning(pop)
#endif

#endif
