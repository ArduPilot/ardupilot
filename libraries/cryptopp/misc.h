// misc.h - originally written and placed in the public domain by Wei Dai

/// \file misc.h
/// \brief Utility functions for the Crypto++ library.

#ifndef CRYPTOPP_MISC_H
#define CRYPTOPP_MISC_H

#include "cryptlib.h"
#include "secblockfwd.h"
#include "smartptr.h"
#include "stdcpp.h"
#include "trap.h"

#if !defined(CRYPTOPP_DOXYGEN_PROCESSING)

#if (CRYPTOPP_MSC_VERSION)
# pragma warning(push)
# pragma warning(disable: 4146 4514)
# if (CRYPTOPP_MSC_VERSION >= 1400)
#  pragma warning(disable: 6326)
# endif
#endif

// Issue 340 and Issue 793
#if CRYPTOPP_GCC_DIAGNOSTIC_AVAILABLE
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wconversion"
# pragma GCC diagnostic ignored "-Wsign-conversion"
# pragma GCC diagnostic ignored "-Wunused-function"
#endif

#ifdef _MSC_VER
	#if _MSC_VER >= 1400
		// VC2005 workaround: disable declarations that conflict with winnt.h
		#define _interlockedbittestandset CRYPTOPP_DISABLED_INTRINSIC_1
		#define _interlockedbittestandreset CRYPTOPP_DISABLED_INTRINSIC_2
		#define _interlockedbittestandset64 CRYPTOPP_DISABLED_INTRINSIC_3
		#define _interlockedbittestandreset64 CRYPTOPP_DISABLED_INTRINSIC_4
		#include <intrin.h>
		#undef _interlockedbittestandset
		#undef _interlockedbittestandreset
		#undef _interlockedbittestandset64
		#undef _interlockedbittestandreset64
		#define CRYPTOPP_FAST_ROTATE(x) 1
	#elif _MSC_VER >= 1300
		#define CRYPTOPP_FAST_ROTATE(x) ((x) == 32 | (x) == 64)
	#else
		#define CRYPTOPP_FAST_ROTATE(x) ((x) == 32)
	#endif
#elif (defined(__MWERKS__) && TARGET_CPU_PPC) || \
	(defined(__GNUC__) && (defined(_ARCH_PWR2) || defined(_ARCH_PWR) || defined(_ARCH_PPC) || defined(_ARCH_PPC64) || defined(_ARCH_COM)))
	#define CRYPTOPP_FAST_ROTATE(x) ((x) == 32)
#elif defined(__GNUC__) && (CRYPTOPP_BOOL_X64 || CRYPTOPP_BOOL_X32 || CRYPTOPP_BOOL_X86)	// depend on GCC's peephole optimization to generate rotate instructions
	#define CRYPTOPP_FAST_ROTATE(x) 1
#else
	#define CRYPTOPP_FAST_ROTATE(x) 0
#endif

#ifdef __BORLANDC__
#include <mem.h>
#include <stdlib.h>
#endif

#if (defined(__GNUC__) || defined(__clang__)) && defined(__linux__)
#define CRYPTOPP_BYTESWAP_AVAILABLE 1
#include <byteswap.h>
#endif

// Limit to ARM A-32. Aarch64 is failing self tests.
#if defined(__arm__) && (defined(__GNUC__) || defined(__clang__)) && (__ARM_ARCH >= 6)
#define CRYPTOPP_ARM_BYTEREV_AVAILABLE 1
#endif

// Limit to ARM A-32. Aarch64 is failing self tests.
#if defined(__arm__) && (defined(__GNUC__) || defined(__clang__)) && (__ARM_ARCH >= 7)
#define CRYPTOPP_ARM_BITREV_AVAILABLE 1
#endif

#if defined(__BMI__)
# include <x86intrin.h>
# include <immintrin.h>
#endif  // GCC and BMI

// More LLVM bullshit. Apple Clang 6.0 does not define them.
// Later version of Clang defines them and results in warnings.
#if defined(__clang__)
# ifndef _blsr_u32
#  define _blsr_u32 __blsr_u32
# endif
# ifndef _blsr_u64
#  define _blsr_u64 __blsr_u64
# endif
# ifndef _tzcnt_u32
#  define _tzcnt_u32 __tzcnt_u32
# endif
# ifndef _tzcnt_u64
#  define _tzcnt_u64 __tzcnt_u64
# endif
#endif

#endif  // CRYPTOPP_DOXYGEN_PROCESSING

#if CRYPTOPP_DOXYGEN_PROCESSING
/// \brief The maximum value of a machine word
/// \details <tt>SIZE_MAX</tt> provides the maximum value of a machine word. The value
///  is <tt>0xffffffff</tt> on 32-bit targets, and <tt>0xffffffffffffffff</tt> on 64-bit
///  targets.
/// \details If <tt>SIZE_MAX</tt> is not defined, then <tt>__SIZE_MAX__</tt> is used if
///  defined. If not defined, then <tt>SIZE_T_MAX</tt> is used if defined. If not defined,
///  then the library uses <tt>std::numeric_limits<size_t>::max()</tt>.
/// \details The library prefers <tt>__SIZE_MAX__</tt> or <tt>__SIZE_T_MAX__</tt> because
///  they are effectively <tt>constexpr</tt> that is optimized well by all compilers.
///  <tt>std::numeric_limits<size_t>::max()</tt> is not always a <tt>constexpr</tt>, and
///  it is not always optimized well.
#  define SIZE_MAX ...
#else
// Its amazing portability problems still plague this simple concept in 2015.
// http://stackoverflow.com/questions/30472731/which-c-standard-header-defines-size-max
// Avoid NOMINMAX macro on Windows. http://support.microsoft.com/en-us/kb/143208
#ifndef SIZE_MAX
# if defined(__SIZE_MAX__)
#  define SIZE_MAX __SIZE_MAX__
# elif defined(SIZE_T_MAX)
#  define SIZE_MAX SIZE_T_MAX
# elif defined(__SIZE_TYPE__)
#  define SIZE_MAX (~(__SIZE_TYPE__)0)
# else
#  define SIZE_MAX ((std::numeric_limits<size_t>::max)())
# endif
#endif

#endif // CRYPTOPP_DOXYGEN_PROCESSING

NAMESPACE_BEGIN(CryptoPP)

// Forward declaration for IntToString specialization
class Integer;

// ************** compile-time assertion ***************

#if CRYPTOPP_DOXYGEN_PROCESSING
/// \brief Compile time assertion
/// \param expr the expression to evaluate
/// \details Asserts the expression <tt>expr</tt> during compile. If C++14 and
///  N3928 are available, then C++14 <tt>static_assert</tt> is used. Otherwise,
///  a <tt>CompileAssert</tt> structure is used. When the structure is used
///  a negative-sized array triggers the assert at compile time.
# define CRYPTOPP_COMPILE_ASSERT(expr) { ... }
#elif defined(CRYPTOPP_CXX17_STATIC_ASSERT)
# define CRYPTOPP_COMPILE_ASSERT(expr) static_assert(expr)
#else // CRYPTOPP_DOXYGEN_PROCESSING
template <bool b>
struct CompileAssert
{
	static char dummy[2*b-1];
};

#define CRYPTOPP_COMPILE_ASSERT(assertion) CRYPTOPP_COMPILE_ASSERT_INSTANCE(assertion, __LINE__)
#define CRYPTOPP_ASSERT_JOIN(X, Y) CRYPTOPP_DO_ASSERT_JOIN(X, Y)
#define CRYPTOPP_DO_ASSERT_JOIN(X, Y) X##Y

#if defined(CRYPTOPP_EXPORTS) || defined(CRYPTOPP_IMPORTS)
# define CRYPTOPP_COMPILE_ASSERT_INSTANCE(assertion, instance)
#else
# if defined(__GNUC__) || defined(__clang__)
#  define CRYPTOPP_COMPILE_ASSERT_INSTANCE(assertion, instance) \
       static CompileAssert<(assertion)> \
       CRYPTOPP_ASSERT_JOIN(cryptopp_CRYPTOPP_ASSERT_, instance) __attribute__ ((unused))
# else
#  define CRYPTOPP_COMPILE_ASSERT_INSTANCE(assertion, instance) \
       static CompileAssert<(assertion)> \
       CRYPTOPP_ASSERT_JOIN(cryptopp_CRYPTOPP_ASSERT_, instance)
# endif // GCC or Clang
#endif

#endif // CRYPTOPP_DOXYGEN_PROCESSING

// ************** count elements in an array ***************

#if CRYPTOPP_DOXYGEN_PROCESSING
/// \brief Counts elements in an array
/// \param arr an array of elements
/// \details COUNTOF counts elements in an array. On Windows COUNTOF(x) is defined
///  to <tt>_countof(x)</tt> to ensure correct results for pointers.
/// \note COUNTOF does not produce correct results with pointers, and an array must be used.
///  <tt>sizeof(x)/sizeof(x[0])</tt> suffers the same problem. The risk is eliminated by using
///  <tt>_countof(x)</tt> on Windows. Windows will provide the immunity for other platforms.
# define COUNTOF(arr)
#else
// VS2005 added _countof
#ifndef COUNTOF
# if defined(_MSC_VER) && (_MSC_VER >= 1400)
#  define COUNTOF(x) _countof(x)
# else
#  define COUNTOF(x) (sizeof(x)/sizeof(x[0]))
# endif
#endif // COUNTOF
#endif // CRYPTOPP_DOXYGEN_PROCESSING

// ************** misc classes ***************

/// \brief An Empty class
/// \details The Empty class can be used as a template parameter <tt>BASE</tt> when no base class exists.
class CRYPTOPP_DLL Empty
{
};

#if !defined(CRYPTOPP_DOXYGEN_PROCESSING)
template <class BASE1, class BASE2>
class CRYPTOPP_NO_VTABLE TwoBases : public BASE1, public BASE2
{
};

template <class BASE1, class BASE2, class BASE3>
class CRYPTOPP_NO_VTABLE ThreeBases : public BASE1, public BASE2, public BASE3
{
};
#endif // CRYPTOPP_DOXYGEN_PROCESSING

/// \tparam T class or type
/// \brief Uses encapsulation to hide an object in derived classes
/// \details The object T is declared as protected.
template <class T>
class ObjectHolder
{
protected:
	T m_object;
};

/// \brief Ensures an object is not copyable
/// \details NotCopyable ensures an object is not copyable by making the
///  copy constructor and assignment operator private. Deleters are used
///  under C++11.
/// \sa Clonable class
class NotCopyable
{
public:
	NotCopyable() {}
#if CRYPTOPP_CXX11_DELETED_FUNCTIONS
	NotCopyable(const NotCopyable &) = delete;
	void operator=(const NotCopyable &) = delete;
#else
private:
	NotCopyable(const NotCopyable &);
	void operator=(const NotCopyable &);
#endif
};

/// \brief An object factory function
/// \tparam T class or type
/// \details NewObject overloads operator()().
template <class T>
struct NewObject
{
	T* operator()() const {return new T;}
};

#if CRYPTOPP_DOXYGEN_PROCESSING
/// \brief A memory barrier
/// \details MEMORY_BARRIER attempts to ensure reads and writes are completed
///  in the absence of a language synchronization point. It is used by the
///  Singleton class if the compiler supports it. The barrier is provided at the
///  customary places in a double-checked initialization.
/// \details Internally, MEMORY_BARRIER uses <tt>std::atomic_thread_fence</tt> if
///  C++11 atomics are available. Otherwise, <tt>intrinsic(_ReadWriteBarrier)</tt>,
///  <tt>_ReadWriteBarrier()</tt> or <tt>__asm__("" ::: "memory")</tt> is used.
#define MEMORY_BARRIER ...
#else
#if defined(CRYPTOPP_CXX11_ATOMIC)
# define MEMORY_BARRIER() std::atomic_thread_fence(std::memory_order_acq_rel)
#elif (_MSC_VER >= 1400)
# pragma intrinsic(_ReadWriteBarrier)
# define MEMORY_BARRIER() _ReadWriteBarrier()
#elif defined(__INTEL_COMPILER)
# define MEMORY_BARRIER() __memory_barrier()
#elif defined(__GNUC__) || defined(__clang__)
# define MEMORY_BARRIER() __asm__ __volatile__ ("" ::: "memory")
#else
# define MEMORY_BARRIER()
#endif
#endif // CRYPTOPP_DOXYGEN_PROCESSING

/// \brief Restricts the instantiation of a class to one static object without locks
/// \tparam T the class or type
/// \tparam F the object factory for T
/// \tparam instance an instance counter for the class object
/// \details This class safely initializes a static object in a multithreaded environment. For C++03
///  and below it will do so without using locks for portability. If two threads call Ref() at the same
///  time, they may get back different references, and one object may end up being memory leaked. This
///  is by design and it avoids a subltle initialization problem ina multithreaded environment with thread
///  local storage on early Windows platforms, like Windows XP and Windows 2003.
/// \details For C++11 and above, a standard double-checked locking pattern with thread fences
///  are used. The locks and fences are standard and do not hinder portability.
/// \details Microsoft's C++11 implementation provides the necessary primitive support on Windows Vista and
///  above when using Visual Studio 2015 (<tt>cl.exe</tt> version 19.00). If C++11 is desired, you should
///  set <tt>WINVER</tt> or <tt>_WIN32_WINNT</tt> to 0x600 (or above), and compile with Visual Studio 2015.
/// \sa <A HREF="http://preshing.com/20130930/double-checked-locking-is-fixed-in-cpp11/">Double-Checked Locking
///  is Fixed In C++11</A>, <A HREF="http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2008/n2660.htm">Dynamic
///  Initialization and Destruction with Concurrency</A> and
///  <A HREF="http://msdn.microsoft.com/en-us/library/6yh4a9k1.aspx">Thread Local Storage (TLS)</A> on MSDN.
/// \since Crypto++ 5.2
template <class T, class F = NewObject<T>, int instance=0>
class Singleton
{
public:
	Singleton(F objectFactory = F()) : m_objectFactory(objectFactory) {}

	// prevent this function from being inlined
	CRYPTOPP_NOINLINE const T & Ref(CRYPTOPP_NOINLINE_DOTDOTDOT) const;

private:
	F m_objectFactory;
};

/// \brief Return a reference to the inner Singleton object
/// \tparam T the class or type
/// \tparam F the object factory for T
/// \tparam instance an instance counter for the class object
/// \details Ref() is used to create the object using the object factory. The
///  object is only created once with the limitations discussed in the class documentation.
/// \sa <A HREF="http://preshing.com/20130930/double-checked-locking-is-fixed-in-cpp11/">Double-Checked Locking is Fixed In C++11</A>
/// \since Crypto++ 5.2
template <class T, class F, int instance>
  const T & Singleton<T, F, instance>::Ref(CRYPTOPP_NOINLINE_DOTDOTDOT) const
{
#if defined(CRYPTOPP_CXX11_ATOMIC) && defined(CRYPTOPP_CXX11_SYNCHRONIZATION) && defined(CRYPTOPP_CXX11_STATIC_INIT)
	static std::mutex s_mutex;
	static std::atomic<T*> s_pObject;

	T *p = s_pObject.load(std::memory_order_relaxed);
	std::atomic_thread_fence(std::memory_order_acquire);

	if (p)
		return *p;

	std::lock_guard<std::mutex> lock(s_mutex);
	p = s_pObject.load(std::memory_order_relaxed);
	std::atomic_thread_fence(std::memory_order_acquire);

	if (p)
		return *p;

	T *newObject = m_objectFactory();
	s_pObject.store(newObject, std::memory_order_relaxed);
	std::atomic_thread_fence(std::memory_order_release);

	return *newObject;
#else
	static volatile simple_ptr<T> s_pObject;
	T *p = s_pObject.m_p;
	MEMORY_BARRIER();

	if (p)
		return *p;

	T *newObject = m_objectFactory();
	p = s_pObject.m_p;
	MEMORY_BARRIER();

	if (p)
	{
		delete newObject;
		return *p;
	}

	s_pObject.m_p = newObject;
	MEMORY_BARRIER();

	return *newObject;
#endif
}

// ************** misc functions ***************

/// \brief Create a pointer with an offset
/// \tparam PTR a pointer type
/// \tparam OFF a size type
/// \param pointer a pointer
/// \param offset a offset into the pointer
/// \details PtrAdd can be used to squash Clang and GCC
///  UBsan findings for pointer addition and subtraction.
template <typename PTR, typename OFF>
inline PTR PtrAdd(PTR pointer, OFF offset)
{
	return pointer+static_cast<ptrdiff_t>(offset);
}

/// \brief Create a pointer with an offset
/// \tparam PTR a pointer type
/// \tparam OFF a size type
/// \param pointer a pointer
/// \param offset a offset into the pointer
/// \details PtrSub can be used to squash Clang and GCC
///  UBsan findings for pointer addition and subtraction.
template <typename PTR, typename OFF>
inline PTR PtrSub(PTR pointer, OFF offset)
{
	return pointer-static_cast<ptrdiff_t>(offset);
}

/// \brief Determine pointer difference
/// \tparam PTR a pointer type
/// \param pointer1 the first pointer
/// \param pointer2 the second pointer
/// \details PtrDiff can be used to squash Clang and GCC
///  UBsan findings for pointer addition and subtraction.
///  pointer1 and pointer2 must point to the same object or
///  array (or one past the end), and yields the number of
///  elements (not bytes) difference.
template <typename PTR>
inline ptrdiff_t PtrDiff(const PTR pointer1, const PTR pointer2)
{
	return pointer1 - pointer2;
}

/// \brief Determine pointer difference
/// \tparam PTR a pointer type
/// \param pointer1 the first pointer
/// \param pointer2 the second pointer
/// \details PtrByteDiff can be used to squash Clang and GCC
///  UBsan findings for pointer addition and subtraction.
///  pointer1 and pointer2 must point to the same object or
///  array (or one past the end), and yields the number of
///  bytes (not elements) difference.
template <typename PTR>
inline size_t PtrByteDiff(const PTR pointer1, const PTR pointer2)
{
	return (size_t)(reinterpret_cast<uintptr_t>(pointer1) - reinterpret_cast<uintptr_t>(pointer2));
}

/// \brief Pointer to the first element of a string
/// \param str std::string
/// \details BytePtr returns NULL pointer for an empty string.
/// \return Pointer to the first element of a string
/// \since Crypto++ 8.0
inline byte* BytePtr(std::string& str)
{
	// Caller wants a writeable pointer
	CRYPTOPP_ASSERT(str.empty() == false);

	if (str.empty())
		return NULLPTR;
	return reinterpret_cast<byte*>(&str[0]);
}

/// \brief Pointer to the first element of a string
/// \param str SecByteBlock
/// \details BytePtr returns NULL pointer for an empty string.
/// \return Pointer to the first element of a string
/// \since Crypto++ 8.3
byte* BytePtr(SecByteBlock& str);

/// \brief Const pointer to the first element of a string
/// \param str std::string
/// \details ConstBytePtr returns non-NULL pointer for an empty string.
/// \return Pointer to the first element of a string
/// \since Crypto++ 8.0
inline const byte* ConstBytePtr(const std::string& str)
{
	if (str.empty())
		return NULLPTR;
	return reinterpret_cast<const byte*>(&str[0]);
}

/// \brief Const pointer to the first element of a string
/// \param str SecByteBlock
/// \details ConstBytePtr returns non-NULL pointer for an empty string.
/// \return Pointer to the first element of a string
/// \since Crypto++ 8.3
const byte* ConstBytePtr(const SecByteBlock& str);

/// \brief Size of a string
/// \param str std::string
/// \return size of a string
inline size_t BytePtrSize(const std::string& str)
{
	return str.size();
}

/// \brief Size of a string
/// \param str SecByteBlock
/// \return size of a string
size_t BytePtrSize(const SecByteBlock& str);

#if (!__STDC_WANT_SECURE_LIB__ && !defined(_MEMORY_S_DEFINED)) || defined(CRYPTOPP_WANT_SECURE_LIB)

/// \brief Bounds checking replacement for memcpy()
/// \param dest pointer to the desination memory block
/// \param sizeInBytes size of the desination memory block, in bytes
/// \param src pointer to the source memory block
/// \param count the number of bytes to copy
/// \throws InvalidArgument
/// \details ISO/IEC TR-24772 provides bounds checking interfaces for potentially
///  unsafe functions like memcpy(), strcpy() and memmove(). However,
///  not all standard libraries provides them, like Glibc. The library's
///  memcpy_s() is a near-drop in replacement. Its only a near-replacement
///  because the library's version throws an InvalidArgument on a bounds violation.
/// \details memcpy_s() and memmove_s() are guarded by __STDC_WANT_SECURE_LIB__.
///  If __STDC_WANT_SECURE_LIB__ is not defined or defined to 0, then the library
///  makes memcpy_s() and memmove_s() available. The library will also optionally
///  make the symbols available if <tt>CRYPTOPP_WANT_SECURE_LIB</tt> is defined.
///  <tt>CRYPTOPP_WANT_SECURE_LIB</tt> is in config.h, but it is disabled by default.
/// \details memcpy_s() will assert the pointers src and dest are not NULL
///  in debug builds. Passing NULL for either pointer is undefined behavior.
inline void memcpy_s(void *dest, size_t sizeInBytes, const void *src, size_t count)
{
	// Safer functions on Windows for C&A, http://github.com/weidai11/cryptopp/issues/55

	// Pointers must be valid; otherwise undefined behavior
	CRYPTOPP_ASSERT(dest != NULLPTR); CRYPTOPP_ASSERT(src != NULLPTR);
	// Restricted pointers. We want to check ranges, but it is not clear how to do it.
	CRYPTOPP_ASSERT(src != dest);
	// Destination buffer must be large enough to satsify request
	CRYPTOPP_ASSERT(sizeInBytes >= count);

	if (count > sizeInBytes)
		throw InvalidArgument("memcpy_s: buffer overflow");

#if CRYPTOPP_MSC_VERSION
# pragma warning(push)
# pragma warning(disable: 4996)
# if (CRYPTOPP_MSC_VERSION >= 1400)
#  pragma warning(disable: 6386)
# endif
#endif
	if (src != NULLPTR && dest != NULLPTR)
		std::memcpy(dest, src, count);
#if CRYPTOPP_MSC_VERSION
# pragma warning(pop)
#endif
}

/// \brief Bounds checking replacement for memmove()
/// \param dest pointer to the desination memory block
/// \param sizeInBytes size of the desination memory block, in bytes
/// \param src pointer to the source memory block
/// \param count the number of bytes to copy
/// \throws InvalidArgument
/// \details ISO/IEC TR-24772 provides bounds checking interfaces for potentially
///  unsafe functions like memcpy(), strcpy() and memmove(). However,
///  not all standard libraries provides them, like Glibc. The library's
///  memmove_s() is a near-drop in replacement. Its only a near-replacement
///  because the library's version throws an InvalidArgument on a bounds violation.
/// \details memcpy_s() and memmove_s() are guarded by __STDC_WANT_SECURE_LIB__.
///  If __STDC_WANT_SECURE_LIB__ is not defined or defined to 0, then the library
///  makes memcpy_s() and memmove_s() available. The library will also optionally
///  make the symbols available if <tt>CRYPTOPP_WANT_SECURE_LIB</tt> is defined.
///  <tt>CRYPTOPP_WANT_SECURE_LIB</tt> is in config.h, but it is disabled by default.
/// \details memmove_s() will assert the pointers src and dest are not NULL
///  in debug builds. Passing NULL for either pointer is undefined behavior.
inline void memmove_s(void *dest, size_t sizeInBytes, const void *src, size_t count)
{
	// Safer functions on Windows for C&A, http://github.com/weidai11/cryptopp/issues/55

	// Pointers must be valid; otherwise undefined behavior
	CRYPTOPP_ASSERT(dest != NULLPTR); CRYPTOPP_ASSERT(src != NULLPTR);
	// Destination buffer must be large enough to satsify request
	CRYPTOPP_ASSERT(sizeInBytes >= count);

	if (count > sizeInBytes)
		throw InvalidArgument("memmove_s: buffer overflow");

#if CRYPTOPP_MSC_VERSION
# pragma warning(push)
# pragma warning(disable: 4996)
# if (CRYPTOPP_MSC_VERSION >= 1400)
#  pragma warning(disable: 6386)
# endif
#endif
	if (src != NULLPTR && dest != NULLPTR)
		std::memmove(dest, src, count);
#if CRYPTOPP_MSC_VERSION
# pragma warning(pop)
#endif
}

#if __BORLANDC__ >= 0x620
// C++Builder 2010 workaround: can't use std::memcpy_s because it doesn't allow 0 lengths
# define memcpy_s CryptoPP::memcpy_s
# define memmove_s CryptoPP::memmove_s
#endif

#endif // __STDC_WANT_SECURE_LIB__

/// \brief Swaps two variables which are arrays
/// \tparam T class or type
/// \param a the first value
/// \param b the second value
/// \details C++03 does not provide support for <tt>std::swap(__m128i a, __m128i b)</tt>
///  because <tt>__m128i</tt> is an <tt>unsigned long long[2]</tt>. Most compilers
///  support it out of the box, but Sun Studio C++ compilers 12.2 and 12.3 do not.
/// \sa <A HREF="http://stackoverflow.com/q/38417413">How to swap two __m128i variables
///  in C++03 given its an opaque type and an array?</A> on Stack Overflow.
template <class T>
inline void vec_swap(T& a, T& b)
{
	// __m128i is an unsigned long long[2], and support for swapping it was
	// not added until C++11. SunCC 12.1 - 12.3 fail to consume the swap; while
	// SunCC 12.4 consumes it without -std=c++11.
#if defined(__SUNPRO_CC) && (__SUNPRO_CC <= 0x5120)
	T t;
	t=a, a=b, b=t;
#else
	std::swap(a, b);
#endif
}

/// \brief Memory block initializer and eraser that attempts to survive optimizations
/// \param ptr pointer to the memory block being written
/// \param value the integer value to write for each byte
/// \param num the size of the source memory block, in bytes
/// \details Internally the function calls memset with the value value, and receives the
///  return value from memset as a <tt>volatile</tt> pointer.
inline void * memset_z(void *ptr, int value, size_t num)
{
// avoid extranous warning on GCC 4.3.2 Ubuntu 8.10
#if CRYPTOPP_GCC_VERSION >= 30001
	if (__builtin_constant_p(num) && num==0)
		return ptr;
#endif
	volatile void* x = memset(ptr, value, num);
	return const_cast<void*>(x);
}

/// \brief Replacement function for std::min
/// \tparam T class or type
/// \param a the first value
/// \param b the second value
/// \returns the minimum value based on a comparison of <tt>b \< a</tt> using <tt>operator\<</tt>
/// \details STDMIN was provided because the library could not easily use std::min or std::max in Windows or Cygwin 1.1.0
template <class T> inline const T& STDMIN(const T& a, const T& b)
{
	return b < a ? b : a;
}

/// \brief Replacement function for std::max
/// \tparam T class or type
/// \param a the first value
/// \param b the second value
/// \returns the minimum value based on a comparison of <tt>a \< b</tt> using <tt>operator\<</tt>
/// \details STDMAX was provided because the library could not easily use std::min or std::max in Windows or Cygwin 1.1.0
template <class T> inline const T& STDMAX(const T& a, const T& b)
{
	return a < b ? b : a;
}

#if CRYPTOPP_MSC_VERSION
# pragma warning(push)
# pragma warning(disable: 4389)
#endif

#if CRYPTOPP_GCC_DIAGNOSTIC_AVAILABLE
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wsign-compare"
# pragma GCC diagnostic ignored "-Wstrict-overflow"
# if (CRYPTOPP_LLVM_CLANG_VERSION >= 20800) || (CRYPTOPP_APPLE_CLANG_VERSION >= 30000)
#  pragma GCC diagnostic ignored "-Wtautological-compare"
# elif (CRYPTOPP_GCC_VERSION >= 40300)
#  pragma GCC diagnostic ignored "-Wtype-limits"
# endif
#endif

/// \brief Safe comparison of values that could be neagtive and incorrectly promoted
/// \tparam T1 class or type
/// \tparam T2 class or type
/// \param a the first value
/// \param b the second value
/// \returns the minimum value based on a comparison a and b using <tt>operator&lt;</tt>.
/// \details The comparison <tt>b \< a</tt> is performed and the value returned is a's type T1.
template <class T1, class T2> inline const T1 UnsignedMin(const T1& a, const T2& b)
{
	CRYPTOPP_COMPILE_ASSERT((sizeof(T1)<=sizeof(T2) && T2(-1)>0) || (sizeof(T1)>sizeof(T2) && T1(-1)>0));
	if (sizeof(T1)<=sizeof(T2))
		return b < (T2)a ? (T1)b : a;
	else
		return (T1)b < a ? (T1)b : a;
}

/// \brief Tests whether a conversion from -> to is safe to perform
/// \tparam T1 class or type
/// \tparam T2 class or type
/// \param from the first value
/// \param to the second value
/// \returns true if its safe to convert from into to, false otherwise.
template <class T1, class T2>
inline bool SafeConvert(T1 from, T2 &to)
{
	to = static_cast<T2>(from);
	if (from != to || (from > 0) != (to > 0))
		return false;
	return true;
}

/// \brief Converts a value to a string
/// \tparam T class or type
/// \param value the value to convert
/// \param base the base to use during the conversion
/// \returns the string representation of value in base.
template <class T>
std::string IntToString(T value, unsigned int base = 10)
{
	// Hack... set the high bit for uppercase.
	const unsigned int HIGH_BIT = (1U << 31);
	const char CH = !!(base & HIGH_BIT) ? 'A' : 'a';
	base &= ~HIGH_BIT;

	CRYPTOPP_ASSERT(base >= 2);
	if (value == 0)
		return "0";

	bool negate = false;
	if (value < 0)
	{
		negate = true;
		value = 0-value;	// VC .NET does not like -a
	}
	std::string result;
	while (value > 0)
	{
		T digit = value % base;
		result = char((digit < 10 ? '0' : (CH - 10)) + digit) + result;
		value /= base;
	}
	if (negate)
		result = "-" + result;
	return result;
}

/// \brief Converts an unsigned value to a string
/// \param value the value to convert
/// \param base the base to use during the conversion
/// \returns the string representation of value in base.
/// \details this template function specialization was added to suppress
///   Coverity findings on IntToString() with unsigned types.
template <> CRYPTOPP_DLL
std::string IntToString<word64>(word64 value, unsigned int base);

/// \brief Converts an Integer to a string
/// \param value the Integer to convert
/// \param base the base to use during the conversion
/// \returns the string representation of value in base.
/// \details This is a template specialization of IntToString(). Use it
///  like IntToString():
/// <pre>
///  // Print integer in base 10
///  Integer n...
///  std::string s = IntToString(n, 10);
/// </pre>
/// \details The string is presented with lowercase letters by default. A
///  hack is available to switch to uppercase letters without modifying
///  the function signature.
/// <pre>
///  // Print integer in base 16, uppercase letters
///  Integer n...
///  const unsigned int UPPER = (1 << 31);
///  std::string s = IntToString(n, (UPPER | 16));</pre>
template <> CRYPTOPP_DLL
std::string IntToString<Integer>(Integer value, unsigned int base);

#if CRYPTOPP_MSC_VERSION
# pragma warning(pop)
#endif

#if CRYPTOPP_GCC_DIAGNOSTIC_AVAILABLE
# pragma GCC diagnostic pop
#endif

#define RETURN_IF_NONZERO(x) size_t returnedValue = x; if (returnedValue) return returnedValue

// this version of the macro is fastest on Pentium 3 and Pentium 4 with MSVC 6 SP5 w/ Processor Pack
#define GETBYTE(x, y) (unsigned int)byte((x)>>(8*(y)))
// these may be faster on other CPUs/compilers
// #define GETBYTE(x, y) (unsigned int)(((x)>>(8*(y)))&255)
// #define GETBYTE(x, y) (((byte *)&(x))[y])

#define CRYPTOPP_GET_BYTE_AS_BYTE(x, y) byte((x)>>(8*(y)))

/// \brief Returns the parity of a value
/// \tparam T class or type
/// \param value the value to provide the parity
/// \returns 1 if the number 1-bits in the value is odd, 0 otherwise
template <class T>
unsigned int Parity(T value)
{
	for (unsigned int i=8*sizeof(value)/2; i>0; i/=2)
		value ^= value >> i;
	return (unsigned int)value&1;
}

/// \brief Returns the number of 8-bit bytes or octets required for a value
/// \tparam T class or type
/// \param value the value to test
/// \returns the minimum number of 8-bit bytes or octets required to represent a value
template <class T>
unsigned int BytePrecision(const T &value)
{
	if (!value)
		return 0;

	unsigned int l=0, h=8*sizeof(value);
	while (h-l > 8)
	{
		unsigned int t = (l+h)/2;
		if (value >> t)
			l = t;
		else
			h = t;
	}

	return h/8;
}

/// \brief Returns the number of bits required for a value
/// \tparam T class or type
/// \param value the value to test
/// \returns the maximum number of bits required to represent a value.
template <class T>
unsigned int BitPrecision(const T &value)
{
	if (!value)
		return 0;

	unsigned int l=0, h=8*sizeof(value);

	while (h-l > 1)
	{
		unsigned int t = (l+h)/2;
		if (value >> t)
			l = t;
		else
			h = t;
	}

	return h;
}

/// Determines the number of trailing 0-bits in a value
/// \param v the 32-bit value to test
/// \returns the number of trailing 0-bits in v, starting at the least significant bit position
/// \details TrailingZeros returns the number of trailing 0-bits in v, starting at the least
///  significant bit position. The return value is undefined if there are no 1-bits set in the value v.
/// \note The function does not return 0 if no 1-bits are set because 0 collides with a 1-bit at the 0-th position.
inline unsigned int TrailingZeros(word32 v)
{
	// GCC 4.7 and VS2012 provides tzcnt on AVX2/BMI enabled processors
	// We don't enable for Microsoft because it requires a runtime check.
	// http://msdn.microsoft.com/en-us/library/hh977023%28v=vs.110%29.aspx
	CRYPTOPP_ASSERT(v != 0);
#if defined(__BMI__)
	return (unsigned int)_tzcnt_u32(v);
#elif defined(__GNUC__) && (CRYPTOPP_GCC_VERSION >= 30400)
	return (unsigned int)__builtin_ctz(v);
#elif defined(_MSC_VER) && (_MSC_VER >= 1400)
	unsigned long result;
	_BitScanForward(&result, v);
	return static_cast<unsigned int>(result);
#else
	// from http://graphics.stanford.edu/~seander/bithacks.html#ZerosOnRightMultLookup
	static const int MultiplyDeBruijnBitPosition[32] =
	{
	  0, 1, 28, 2, 29, 14, 24, 3, 30, 22, 20, 15, 25, 17, 4, 8,
	  31, 27, 13, 23, 21, 19, 16, 7, 26, 12, 18, 6, 11, 5, 10, 9
	};
	return MultiplyDeBruijnBitPosition[((word32)((v & -v) * 0x077CB531U)) >> 27];
#endif
}

/// Determines the number of trailing 0-bits in a value
/// \param v the 64-bit value to test
/// \returns the number of trailing 0-bits in v, starting at the least significant bit position
/// \details TrailingZeros returns the number of trailing 0-bits in v, starting at the least
///  significant bit position. The return value is undefined if there are no 1-bits set in the value v.
/// \note The function does not return 0 if no 1-bits are set because 0 collides with a 1-bit at the 0-th position.
inline unsigned int TrailingZeros(word64 v)
{
	// GCC 4.7 and VS2012 provides tzcnt on AVX2/BMI enabled processors
	// We don't enable for Microsoft because it requires a runtime check.
	// http://msdn.microsoft.com/en-us/library/hh977023%28v=vs.110%29.aspx
	CRYPTOPP_ASSERT(v != 0);
#if defined(__BMI__) && defined(__x86_64__)
	return (unsigned int)_tzcnt_u64(v);
#elif defined(__GNUC__) && (CRYPTOPP_GCC_VERSION >= 30400)
	return (unsigned int)__builtin_ctzll(v);
#elif defined(_MSC_VER) && (_MSC_VER >= 1400) && (defined(_M_X64) || defined(_M_IA64))
	unsigned long result;
	_BitScanForward64(&result, v);
	return static_cast<unsigned int>(result);
#else
	return word32(v) ? TrailingZeros(word32(v)) : 32 + TrailingZeros(word32(v>>32));
#endif
}

/// \brief Truncates the value to the specified number of bits.
/// \tparam T class or type
/// \param value the value to truncate or mask
/// \param bits the number of bits to truncate or mask
/// \returns the value truncated to the specified number of bits, starting at the least
///  significant bit position
/// \details This function masks the low-order bits of value and returns the result. The
///  mask is created with <tt>(1 << bits) - 1</tt>.
template <class T>
inline T Crop(T value, size_t bits)
{
	if (bits < 8*sizeof(value))
    	return T(value & ((T(1) << bits) - 1));
	else
		return value;
}

/// \brief Returns the number of 8-bit bytes or octets required for the specified number of bits
/// \param bitCount the number of bits
/// \returns the minimum number of 8-bit bytes or octets required by bitCount
/// \details BitsToBytes is effectively a ceiling function based on 8-bit bytes.
inline size_t BitsToBytes(size_t bitCount)
{
	return ((bitCount+7)/(8));
}

/// \brief Returns the number of words required for the specified number of bytes
/// \param byteCount the number of bytes
/// \returns the minimum number of words required by byteCount
/// \details BytesToWords is effectively a ceiling function based on <tt>WORD_SIZE</tt>.
///  <tt>WORD_SIZE</tt> is defined in config.h
inline size_t BytesToWords(size_t byteCount)
{
	return ((byteCount+WORD_SIZE-1)/WORD_SIZE);
}

/// \brief Returns the number of words required for the specified number of bits
/// \param bitCount the number of bits
/// \returns the minimum number of words required by bitCount
/// \details BitsToWords is effectively a ceiling function based on <tt>WORD_BITS</tt>.
///  <tt>WORD_BITS</tt> is defined in config.h
inline size_t BitsToWords(size_t bitCount)
{
	return ((bitCount+WORD_BITS-1)/(WORD_BITS));
}

/// \brief Returns the number of double words required for the specified number of bits
/// \param bitCount the number of bits
/// \returns the minimum number of double words required by bitCount
/// \details BitsToDwords is effectively a ceiling function based on <tt>2*WORD_BITS</tt>.
///  <tt>WORD_BITS</tt> is defined in config.h
inline size_t BitsToDwords(size_t bitCount)
{
	return ((bitCount+2*WORD_BITS-1)/(2*WORD_BITS));
}

/// Performs an XOR of a buffer with a mask
/// \param buf the buffer to XOR with the mask
/// \param mask the mask to XOR with the buffer
/// \param count the size of the buffers, in bytes
/// \details The function effectively visits each element in the buffers and performs
///  <tt>buf[i] ^= mask[i]</tt>. buf and mask must be of equal size.
CRYPTOPP_DLL void CRYPTOPP_API xorbuf(byte *buf, const byte *mask, size_t count);

/// Performs an XOR of an input buffer with a mask and stores the result in an output buffer
/// \param output the destination buffer
/// \param input the source buffer to XOR with the mask
/// \param mask the mask buffer to XOR with the input buffer
/// \param count the size of the buffers, in bytes
/// \details The function effectively visits each element in the buffers and performs
///  <tt>output[i] = input[i] ^ mask[i]</tt>. output, input and mask must be of equal size.
CRYPTOPP_DLL void CRYPTOPP_API xorbuf(byte *output, const byte *input, const byte *mask, size_t count);

/// \brief Performs a near constant-time comparison of two equally sized buffers
/// \param buf1 the first buffer
/// \param buf2 the second buffer
/// \param count the size of the buffers, in bytes
/// \details VerifyBufsEqual performs an XOR of the elements in two equally sized
///  buffers and retruns a result based on the XOR operation. A count of 0 returns
///  true because two empty buffers are considered equal.
/// \details The function is near constant-time because CPU micro-code timings could
///  affect the "constant-ness". Calling code is responsible for mitigating timing
///  attacks if the buffers are not equally sized.
/// \sa ModPowerOf2
CRYPTOPP_DLL bool CRYPTOPP_API VerifyBufsEqual(const byte *buf1, const byte *buf2, size_t count);

/// \brief Tests whether a value is a power of 2
/// \param value the value to test
/// \returns true if value is a power of 2, false otherwise
/// \details The function creates a mask of <tt>value - 1</tt> and returns the result
///  of an AND operation compared to 0. If value is 0 or less than 0, then the function
///  returns false.
template <class T>
inline bool IsPowerOf2(const T &value)
{
	return value > 0 && (value & (value-1)) == 0;
}

#if defined(__BMI__)
template <>
inline bool IsPowerOf2<word32>(const word32 &value)
{
	return value > 0 && _blsr_u32(value) == 0;
}

# if defined(__x86_64__)
template <>
inline bool IsPowerOf2<word64>(const word64 &value)
{
	return value > 0 && _blsr_u64(value) == 0;
}
# endif  // __x86_64__
#endif   // __BMI__

/// \brief Provide the minimum value for a type
/// \tparam T type of class
/// \returns the minimum value of the type or class
/// \details NumericLimitsMin() was introduced for Clang at <A
///  HREF="http://github.com/weidai11/cryptopp/issues/364">Issue 364,
///  Apple Clang 6.0 and numeric_limits<word128>::max() returns 0</A>.
/// \details NumericLimitsMin() requires a specialization for <tt>T</tt>,
///  meaning <tt>std::numeric_limits<T>::is_specialized</tt> must return
///  <tt>true</tt>. In the case of <tt>word128</tt> Clang did not specialize
///  <tt>numeric_limits</tt> for the type.
/// \since Crypto++ 8.1
template<class T>
inline T NumericLimitsMin()
{
	CRYPTOPP_ASSERT(std::numeric_limits<T>::is_specialized);
	return (std::numeric_limits<T>::min)();
}

/// \brief Provide the maximum value for a type
/// \tparam T type of class
/// \returns the maximum value of the type or class
/// \details NumericLimitsMax() was introduced for Clang at <A
///  HREF="http://github.com/weidai11/cryptopp/issues/364">Issue 364,
///  Apple Clang 6.0 and numeric_limits<word128>::max() returns 0</A>.
/// \details NumericLimitsMax() requires a specialization for <tt>T</tt>,
///  meaning <tt>std::numeric_limits<T>::is_specialized</tt> must return
///  <tt>true</tt>. In the case of <tt>word128</tt> Clang did not specialize
///  <tt>numeric_limits</tt> for the type.
/// \since Crypto++ 8.1
template<class T>
inline T NumericLimitsMax()
{
	CRYPTOPP_ASSERT(std::numeric_limits<T>::is_specialized);
	return (std::numeric_limits<T>::max)();
}

// NumericLimitsMin and NumericLimitsMax added for word128 types,
//   see http://github.com/weidai11/cryptopp/issues/364
#if defined(CRYPTOPP_WORD128_AVAILABLE)
template<>
inline word128 NumericLimitsMin()
{
	return 0;
}
template<>
inline word128 NumericLimitsMax()
{
	return (static_cast<word128>(LWORD_MAX) << 64U) | LWORD_MAX;
}
#endif

/// \brief Performs a saturating subtract clamped at 0
/// \tparam T1 class or type
/// \tparam T2 class or type
/// \param a the minuend
/// \param b the subtrahend
/// \returns the difference produced by the saturating subtract
/// \details Saturating arithmetic restricts results to a fixed range. Results that are
///  less than 0 are clamped at 0.
/// \details Use of saturating arithmetic in places can be advantageous because it can
///  avoid a branch by using an instruction like a conditional move (<tt>CMOVE</tt>).
template <class T1, class T2>
inline T1 SaturatingSubtract(const T1 &a, const T2 &b)
{
	// Generated ASM of a typical clamp, http://gcc.gnu.org/ml/gcc-help/2014-10/msg00112.html
	return T1((a > b) ? (a - b) : 0);
}

/// \brief Performs a saturating subtract clamped at 1
/// \tparam T1 class or type
/// \tparam T2 class or type
/// \param a the minuend
/// \param b the subtrahend
/// \returns the difference produced by the saturating subtract
/// \details Saturating arithmetic restricts results to a fixed range. Results that are
///  less than 1 are clamped at 1.
/// \details Use of saturating arithmetic in places can be advantageous because it can
///  avoid a branch by using an instruction like a conditional move (<tt>CMOVE</tt>).
template <class T1, class T2>
inline T1 SaturatingSubtract1(const T1 &a, const T2 &b)
{
	// Generated ASM of a typical clamp, http://gcc.gnu.org/ml/gcc-help/2014-10/msg00112.html
	return T1((a > b) ? (a - b) : 1);
}

/// \brief Reduces a value to a power of 2
/// \tparam T1 class or type
/// \tparam T2 class or type
/// \param a the first value
/// \param b the second value
/// \returns ModPowerOf2() returns <tt>a & (b-1)</tt>. <tt>b</tt> must be a power of 2.
///  Use IsPowerOf2() to determine if <tt>b</tt> is a suitable candidate.
/// \sa IsPowerOf2
template <class T1, class T2>
inline T2 ModPowerOf2(const T1 &a, const T2 &b)
{
	CRYPTOPP_ASSERT(IsPowerOf2(b));
	// Coverity finding CID 170383 Overflowed return value (INTEGER_OVERFLOW)
	return T2(a) & SaturatingSubtract(b,1U);
}

/// \brief Rounds a value down to a multiple of a second value
/// \tparam T1 class or type
/// \tparam T2 class or type
/// \param n the value to reduce
/// \param m the value to reduce \n to to a multiple
/// \returns the possibly unmodified value \n
/// \details RoundDownToMultipleOf is effectively a floor function based on m. The function returns
///  the value <tt>n - n\%m</tt>. If n is a multiple of m, then the original value is returned.
/// \note <tt>T1</tt> and <tt>T2</tt> should be usigned arithmetic types. If <tt>T1</tt> or
///  <tt>T2</tt> is signed, then the value should be non-negative. The library asserts in
///  debug builds when practical, but allows you to perform the operation in release builds.
template <class T1, class T2>
inline T1 RoundDownToMultipleOf(const T1 &n, const T2 &m)
{
	// http://github.com/weidai11/cryptopp/issues/364
#if !defined(CRYPTOPP_APPLE_CLANG_VERSION) || (CRYPTOPP_APPLE_CLANG_VERSION >= 80000)
	CRYPTOPP_ASSERT(std::numeric_limits<T1>::is_integer);
	CRYPTOPP_ASSERT(std::numeric_limits<T2>::is_integer);
#endif

	CRYPTOPP_ASSERT(!std::numeric_limits<T1>::is_signed || n > 0);
	CRYPTOPP_ASSERT(!std::numeric_limits<T2>::is_signed || m > 0);

	if (IsPowerOf2(m))
		return n - ModPowerOf2(n, m);
	else
		return n - n%m;
}

/// \brief Rounds a value up to a multiple of a second value
/// \tparam T1 class or type
/// \tparam T2 class or type
/// \param n the value to reduce
/// \param m the value to reduce \n to to a multiple
/// \returns the possibly unmodified value \n
/// \details RoundUpToMultipleOf is effectively a ceiling function based on m. The function
///  returns the value <tt>n + n\%m</tt>. If n is a multiple of m, then the original value is
///  returned. If the value n would overflow, then an InvalidArgument exception is thrown.
/// \note <tt>T1</tt> and <tt>T2</tt> should be usigned arithmetic types. If <tt>T1</tt> or
///  <tt>T2</tt> is signed, then the value should be non-negative. The library asserts in
///  debug builds when practical, but allows you to perform the operation in release builds.
template <class T1, class T2>
inline T1 RoundUpToMultipleOf(const T1 &n, const T2 &m)
{
	// http://github.com/weidai11/cryptopp/issues/364
#if !defined(CRYPTOPP_APPLE_CLANG_VERSION) || (CRYPTOPP_APPLE_CLANG_VERSION >= 80000)
	CRYPTOPP_ASSERT(std::numeric_limits<T1>::is_integer);
	CRYPTOPP_ASSERT(std::numeric_limits<T2>::is_integer);
#endif

	CRYPTOPP_ASSERT(!std::numeric_limits<T1>::is_signed || n > 0);
	CRYPTOPP_ASSERT(!std::numeric_limits<T2>::is_signed || m > 0);

	if (NumericLimitsMax<T1>() - m + 1 < n)
		throw InvalidArgument("RoundUpToMultipleOf: integer overflow");
	return RoundDownToMultipleOf(T1(n+m-1), m);
}

/// \brief Returns the minimum alignment requirements of a type
/// \tparam T class or type
/// \returns the minimum alignment requirements of <tt>T</tt>, in bytes
/// \details Internally the function calls C++11's <tt>alignof</tt> if available. If not
///  available, then the function uses compiler specific extensions such as
///  <tt>__alignof</tt> and <tt>_alignof_</tt>. If an extension is not available, then
///  the function uses <tt>__BIGGEST_ALIGNMENT__</tt> if <tt>__BIGGEST_ALIGNMENT__</tt>
///  is smaller than <tt>sizeof(T)</tt>. <tt>sizeof(T)</tt> is used if all others are
///  not available.
template <class T>
inline unsigned int GetAlignmentOf()
{
#if defined(CRYPTOPP_CXX11_ALIGNOF)
	return alignof(T);
#elif (_MSC_VER >= 1300)
	return __alignof(T);
#elif defined(__GNUC__)
	return __alignof__(T);
#elif defined(__SUNPRO_CC)
	return __alignof__(T);
#elif defined(__IBM_ALIGNOF__)
	return __alignof__(T);
#elif CRYPTOPP_BOOL_SLOW_WORD64
	return UnsignedMin(4U, sizeof(T));
#else
	return sizeof(T);
#endif
}

/// \brief Determines whether ptr is aligned to a minimum value
/// \param ptr the pointer being checked for alignment
/// \param alignment the alignment value to test the pointer against
/// \returns true if <tt>ptr</tt> is aligned on at least <tt>alignment</tt>
///  boundary, false otherwise
/// \details Internally the function tests whether alignment is 1. If so,
///  the function returns true. If not, then the function effectively
///  performs a modular reduction and returns true if the residue is 0.
inline bool IsAlignedOn(const void *ptr, unsigned int alignment)
{
	const uintptr_t x = reinterpret_cast<uintptr_t>(ptr);
	return alignment==1 || (IsPowerOf2(alignment) ? ModPowerOf2(x, alignment) == 0 : x % alignment == 0);
}

/// \brief Determines whether ptr is minimally aligned
/// \tparam T class or type
/// \param ptr the pointer to check for alignment
/// \returns true if <tt>ptr</tt> is aligned to at least <tt>T</tt>
///  boundary, false otherwise
/// \details Internally the function calls IsAlignedOn with a second
///  parameter of GetAlignmentOf<T>.
template <class T>
inline bool IsAligned(const void *ptr)
{
	return IsAlignedOn(ptr, GetAlignmentOf<T>());
}

#if (CRYPTOPP_LITTLE_ENDIAN)
typedef LittleEndian NativeByteOrder;
#elif (CRYPTOPP_BIG_ENDIAN)
typedef BigEndian NativeByteOrder;
#else
# error "Unable to determine endianness"
#endif

/// \brief Returns NativeByteOrder as an enumerated ByteOrder value
/// \returns LittleEndian if the native byte order is little-endian,
///  and BigEndian if the native byte order is big-endian
/// \details NativeByteOrder is a typedef depending on the platform.
///  If CRYPTOPP_LITTLE_ENDIAN is set in config.h, then
///  GetNativeByteOrder returns LittleEndian. If CRYPTOPP_BIG_ENDIAN
///  is set, then GetNativeByteOrder returns BigEndian.
/// \note There are other byte orders besides little- and big-endian,
///  and they include bi-endian and PDP-endian. If a system is neither
///  little-endian nor big-endian, then a compile time error occurs.
inline ByteOrder GetNativeByteOrder()
{
	return NativeByteOrder::ToEnum();
}

/// \brief Determines whether order follows native byte ordering
/// \param order the ordering being tested against native byte ordering
/// \returns true if order follows native byte ordering, false otherwise
inline bool NativeByteOrderIs(ByteOrder order)
{
	return order == GetNativeByteOrder();
}

/// \brief Returns the direction the cipher is being operated
/// \tparam T class or type
/// \param obj the cipher object being queried
/// \returns ENCRYPTION if the cipher obj is being operated in its forward direction,
///  DECRYPTION otherwise
/// \details A cipher can be operated in a "forward" direction (encryption) or a "reverse"
///  direction (decryption). The operations do not have to be symmetric, meaning a second
///  application of the transformation does not necessariy return the original message.
///  That is, <tt>E(D(m))</tt> may not equal <tt>E(E(m))</tt>; and <tt>D(E(m))</tt> may not
///  equal <tt>D(D(m))</tt>.
template <class T>
inline CipherDir GetCipherDir(const T &obj)
{
	return obj.IsForwardTransformation() ? ENCRYPTION : DECRYPTION;
}

/// \brief Performs an addition with carry on a block of bytes
/// \param inout the byte block
/// \param size the size of the block, in bytes
/// \details Performs an addition with carry by adding 1 on a block of bytes starting at the least
///  significant byte. Once carry is 0, the function terminates and returns to the caller.
/// \note The function is not constant time because it stops processing when the carry is 0.
inline void IncrementCounterByOne(byte *inout, unsigned int size)
{
	CRYPTOPP_ASSERT(inout != NULLPTR);

	unsigned int carry=1;
	while (carry && size != 0)
	{
		// On carry inout[n] equals 0
		carry = ! ++inout[size-1];
		size--;
	}
}

/// \brief Performs an addition with carry on a block of bytes
/// \param output the destination block of bytes
/// \param input the source block of bytes
/// \param size the size of the block
/// \details Performs an addition with carry on a block of bytes starting at the least significant
///  byte. Once carry is 0, the remaining bytes from input are copied to output using memcpy.
/// \details The function is close to near-constant time because it operates on all the bytes in the blocks.
inline void IncrementCounterByOne(byte *output, const byte *input, unsigned int size)
{
	CRYPTOPP_ASSERT(output != NULLPTR);
	CRYPTOPP_ASSERT(input != NULLPTR);

	unsigned int carry=1;
	while (carry && size != 0)
	{
		// On carry output[n] equals 0
		carry = ! (output[size-1] = input[size-1] + 1);
		size--;
	}

	while (size != 0)
	{
		output[size-1] = input[size-1];
		size--;
	}
}

/// \brief Performs a branchless swap of values a and b if condition c is true
/// \tparam T class or type
/// \param c the condition to perform the swap
/// \param a the first value
/// \param b the second value
template <class T>
inline void ConditionalSwap(bool c, T &a, T &b)
{
	T t = c * (a ^ b);
	a ^= t;
	b ^= t;
}

/// \brief Performs a branchless swap of pointers a and b if condition c is true
/// \tparam T class or type
/// \param c the condition to perform the swap
/// \param a the first pointer
/// \param b the second pointer
template <class T>
inline void ConditionalSwapPointers(bool c, T &a, T &b)
{
	ptrdiff_t t = size_t(c) * (a - b);
	a -= t;
	b += t;
}

// see http://www.dwheeler.com/secure-programs/Secure-Programs-HOWTO/protect-secrets.html
// and http://www.securecoding.cert.org/confluence/display/cplusplus/MSC06-CPP.+Be+aware+of+compiler+optimization+when+dealing+with+sensitive+data

/// \brief Sets each element of an array to 0
/// \tparam T class or type
/// \param buf an array of elements
/// \param n the number of elements in the array
/// \details The operation performs a wipe or zeroization. The function
///  attempts to survive optimizations and dead code removal.
template <class T>
void SecureWipeBuffer(T *buf, size_t n)
{
	// GCC 4.3.2 on Cygwin optimizes away the first store if this
	// loop is done in the forward direction
	volatile T *p = buf+n;
	while (n--)
		*(--p) = 0;
}

#if !defined(CRYPTOPP_DISABLE_ASM) && \
    (_MSC_VER >= 1400 || defined(__GNUC__)) && \
    (CRYPTOPP_BOOL_X64 || CRYPTOPP_BOOL_X86)

/// \brief Sets each byte of an array to 0
/// \param buf an array of bytes
/// \param n the number of elements in the array
/// \details The operation performs a wipe or zeroization. The function
///  attempts to survive optimizations and dead code removal.
template<> inline void SecureWipeBuffer(byte *buf, size_t n)
{
	volatile byte *p = buf;
#ifdef __GNUC__
	asm volatile("rep stosb" : "+c"(n), "+D"(p) : "a"(0) : "memory");
#else
	__stosb(reinterpret_cast<byte *>(reinterpret_cast<size_t>(p)), 0, n);
#endif
}

/// \brief Sets each 16-bit element of an array to 0
/// \param buf an array of 16-bit words
/// \param n the number of elements in the array
/// \details The operation performs a wipe or zeroization. The function
///  attempts to survive optimizations and dead code removal.
template<> inline void SecureWipeBuffer(word16 *buf, size_t n)
{
	volatile word16 *p = buf;
#ifdef __GNUC__
	asm volatile("rep stosw" : "+c"(n), "+D"(p) : "a"(0) : "memory");
#else
	__stosw(reinterpret_cast<word16 *>(reinterpret_cast<size_t>(p)), 0, n);
#endif
}

/// \brief Sets each 32-bit element of an array to 0
/// \param buf an array of 32-bit words
/// \param n the number of elements in the array
/// \details The operation performs a wipe or zeroization. The function
///  attempts to survive optimizations and dead code removal.
template<> inline void SecureWipeBuffer(word32 *buf, size_t n)
{
	volatile word32 *p = buf;
#ifdef __GNUC__
	asm volatile("rep stosl" : "+c"(n), "+D"(p) : "a"(0) : "memory");
#else
	__stosd(reinterpret_cast<unsigned long *>(reinterpret_cast<size_t>(p)), 0, n);
#endif
}

/// \brief Sets each 64-bit element of an array to 0
/// \param buf an array of 64-bit words
/// \param n the number of elements in the array
/// \details The operation performs a wipe or zeroization. The function
///  attempts to survive optimizations and dead code removal.
template<> inline void SecureWipeBuffer(word64 *buf, size_t n)
{
#if CRYPTOPP_BOOL_X64
	volatile word64 *p = buf;
# ifdef __GNUC__
	asm volatile("rep stosq" : "+c"(n), "+D"(p) : "a"(0) : "memory");
# else
	__stosq(const_cast<word64 *>(p), 0, n);
# endif
#else
	SecureWipeBuffer(reinterpret_cast<word32 *>(buf), 2*n);
#endif
}

#endif	// CRYPTOPP_BOOL_X64 || CRYPTOPP_BOOL_X86

#if !defined(CRYPTOPP_DISABLE_ASM) && (_MSC_VER >= 1700) && defined(_M_ARM)
template<> inline void SecureWipeBuffer(byte *buf, size_t n)
{
	char *p = reinterpret_cast<char*>(buf+n);
	while (n--)
		__iso_volatile_store8(--p, 0);
}

template<> inline void SecureWipeBuffer(word16 *buf, size_t n)
{
	short *p = reinterpret_cast<short*>(buf+n);
	while (n--)
		__iso_volatile_store16(--p, 0);
}

template<> inline void SecureWipeBuffer(word32 *buf, size_t n)
{
	int *p = reinterpret_cast<int*>(buf+n);
	while (n--)
		__iso_volatile_store32(--p, 0);
}

template<> inline void SecureWipeBuffer(word64 *buf, size_t n)
{
	__int64 *p = reinterpret_cast<__int64*>(buf+n);
	while (n--)
		__iso_volatile_store64(--p, 0);
}
#endif

/// \brief Sets each element of an array to 0
/// \tparam T class or type
/// \param buf an array of elements
/// \param n the number of elements in the array
/// \details The operation performs a wipe or zeroization. The function
///  attempts to survive optimizations and dead code removal.
template <class T>
inline void SecureWipeArray(T *buf, size_t n)
{
	if (sizeof(T) % 8 == 0 && GetAlignmentOf<T>() % GetAlignmentOf<word64>() == 0)
		SecureWipeBuffer(reinterpret_cast<word64 *>(static_cast<void *>(buf)), n * (sizeof(T)/8));
	else if (sizeof(T) % 4 == 0 && GetAlignmentOf<T>() % GetAlignmentOf<word32>() == 0)
		SecureWipeBuffer(reinterpret_cast<word32 *>(static_cast<void *>(buf)), n * (sizeof(T)/4));
	else if (sizeof(T) % 2 == 0 && GetAlignmentOf<T>() % GetAlignmentOf<word16>() == 0)
		SecureWipeBuffer(reinterpret_cast<word16 *>(static_cast<void *>(buf)), n * (sizeof(T)/2));
	else
		SecureWipeBuffer(reinterpret_cast<byte *>(static_cast<void *>(buf)), n * sizeof(T));
}

/// \brief Converts a wide character C-string to a multibyte string
/// \param str C-string consisting of wide characters
/// \param throwOnError flag indicating the function should throw on error
/// \returns str converted to a multibyte string or an empty string.
/// \details StringNarrow() converts a wide string to a narrow string using C++ std::wcstombs() under
///  the executing thread's locale. A locale must be set before using this function, and it can be
///  set with std::setlocale() if needed. Upon success, the converted string is returned.
/// \details Upon failure with throwOnError as false, the function returns an empty string. If
///  throwOnError as true, the function throws an InvalidArgument() exception.
/// \note If you try to convert, say, the Chinese character for "bone" from UTF-16 (0x9AA8) to UTF-8
///  (0xE9 0xAA 0xA8), then you must ensure the locale is available. If the locale is not available,
///  then a 0x21 error is returned on Windows which eventually results in an InvalidArgument() exception.
std::string StringNarrow(const wchar_t *str, bool throwOnError = true);

/// \brief Converts a multibyte C-string to a wide character string
/// \param str C-string consisting of wide characters
/// \param throwOnError flag indicating the function should throw on error
/// \returns str converted to a multibyte string or an empty string.
/// \details StringWiden() converts a narrow string to a wide string using C++ std::mbstowcs() under
///  the executing thread's locale. A locale must be set before using this function, and it can be
///  set with std::setlocale() if needed. Upon success, the converted string is returned.
/// \details Upon failure with throwOnError as false, the function returns an empty string. If
///  throwOnError as true, the function throws an InvalidArgument() exception.
/// \note If you try to convert, say, the Chinese character for "bone" from UTF-8 (0xE9 0xAA 0xA8)
///  to UTF-16 (0x9AA8), then you must ensure the locale is available. If the locale is not available,
///  then a 0x21 error is returned on Windows which eventually results in an InvalidArgument() exception.
std::wstring StringWiden(const char *str, bool throwOnError = true);

// ************** rotate functions ***************

/// \brief Performs a left rotate
/// \tparam R the number of bit positions to rotate the value
/// \tparam T the word type
/// \param x the value to rotate
/// \details This is a portable C/C++ implementation. The value x to be rotated can be 8 to 64-bits wide.
/// \details R must be in the range <tt>[0, sizeof(T)*8 - 1]</tt> to avoid undefined behavior.
///  Use rotlMod if the rotate amount R is outside the range.
/// \details Use rotlConstant when the rotate amount is constant. The template function was added
///  because Clang did not propagate the constant when passed as a function parameter. Clang's
///  need for a constexpr meant rotlFixed failed to compile on occasion.
/// \note rotlConstant attempts to enlist a <tt>rotate IMM</tt> instruction because its often faster
///  than a <tt>rotate REG</tt>. Immediate rotates can be up to three times faster than their register
///  counterparts.
/// \sa rotlConstant, rotrConstant, rotlFixed, rotrFixed, rotlVariable, rotrVariable
/// \since Crypto++ 6.0
template <unsigned int R, class T> inline T rotlConstant(T x)
{
	// Portable rotate that reduces to single instruction...
	// http://gcc.gnu.org/bugzilla/show_bug.cgi?id=57157,
	// http://software.intel.com/en-us/forums/topic/580884
	// and http://llvm.org/bugs/show_bug.cgi?id=24226
	CRYPTOPP_CONSTANT(THIS_SIZE = sizeof(T)*8);
	CRYPTOPP_CONSTANT(MASK = THIS_SIZE-1);
	CRYPTOPP_ASSERT(static_cast<int>(R) < THIS_SIZE);
	return T((x<<R)|(x>>(-R&MASK)));
}

/// \brief Performs a right rotate
/// \tparam R the number of bit positions to rotate the value
/// \tparam T the word type
/// \param x the value to rotate
/// \details This is a portable C/C++ implementation. The value x to be rotated can be 8 to 64-bits wide.
/// \details R must be in the range <tt>[0, sizeof(T)*8 - 1]</tt> to avoid undefined behavior.
///  Use rotrMod if the rotate amount R is outside the range.
/// \details Use rotrConstant when the rotate amount is constant. The template function was added
///  because Clang did not propagate the constant when passed as a function parameter. Clang's
///  need for a constexpr meant rotrFixed failed to compile on occasion.
/// \note rotrConstant attempts to enlist a <tt>rotate IMM</tt> instruction because its often faster
///  than a <tt>rotate REG</tt>. Immediate rotates can be up to three times faster than their register
///  counterparts.
/// \sa rotlConstant, rotrConstant, rotlFixed, rotrFixed, rotlVariable, rotrVariable
template <unsigned int R, class T> inline T rotrConstant(T x)
{
	// Portable rotate that reduces to single instruction...
	// http://gcc.gnu.org/bugzilla/show_bug.cgi?id=57157,
	// http://software.intel.com/en-us/forums/topic/580884
	// and http://llvm.org/bugs/show_bug.cgi?id=24226
	CRYPTOPP_CONSTANT(THIS_SIZE = sizeof(T)*8);
	CRYPTOPP_CONSTANT(MASK = THIS_SIZE-1);
	CRYPTOPP_ASSERT(static_cast<int>(R) < THIS_SIZE);
	return T((x >> R)|(x<<(-R&MASK)));
}

/// \brief Performs a left rotate
/// \tparam T the word type
/// \param x the value to rotate
/// \param y the number of bit positions to rotate the value
/// \details This is a portable C/C++ implementation. The value x to be rotated can be 8 to 64-bits wide.
/// \details y must be in the range <tt>[0, sizeof(T)*8 - 1]</tt> to avoid undefined behavior.
///  Use rotlMod if the rotate amount y is outside the range.
/// \note rotlFixed attempts to enlist a <tt>rotate IMM</tt> instruction because its often faster
///  than a <tt>rotate REG</tt>. Immediate rotates can be up to three times faster than their register
///  counterparts. New code should use <tt>rotlConstant</tt>, which accepts the rotate amount as a
///  template parameter.
/// \sa rotlConstant, rotrConstant, rotlFixed, rotrFixed, rotlVariable, rotrVariable
/// \since Crypto++ 6.0
template <class T> inline T rotlFixed(T x, unsigned int y)
{
	// Portable rotate that reduces to single instruction...
	// http://gcc.gnu.org/bugzilla/show_bug.cgi?id=57157,
	// http://software.intel.com/en-us/forums/topic/580884
	// and http://llvm.org/bugs/show_bug.cgi?id=24226
	CRYPTOPP_CONSTANT(THIS_SIZE = sizeof(T)*8);
	CRYPTOPP_CONSTANT(MASK = THIS_SIZE-1);
	CRYPTOPP_ASSERT(static_cast<int>(y) < THIS_SIZE);
	return T((x<<y)|(x>>(-y&MASK)));
}

/// \brief Performs a right rotate
/// \tparam T the word type
/// \param x the value to rotate
/// \param y the number of bit positions to rotate the value
/// \details This is a portable C/C++ implementation. The value x to be rotated can be 8 to 64-bits wide.
/// \details y must be in the range <tt>[0, sizeof(T)*8 - 1]</tt> to avoid undefined behavior.
///  Use rotrMod if the rotate amount y is outside the range.
/// \note rotrFixed attempts to enlist a <tt>rotate IMM</tt> instruction because its often faster
///  than a <tt>rotate REG</tt>. Immediate rotates can be up to three times faster than their register
///  counterparts. New code should use <tt>rotrConstant</tt>, which accepts the rotate amount as a
///  template parameter.
/// \sa rotlConstant, rotrConstant, rotlFixed, rotrFixed, rotlVariable, rotrVariable
/// \since Crypto++ 3.0
template <class T> inline T rotrFixed(T x, unsigned int y)
{
	// Portable rotate that reduces to single instruction...
	// http://gcc.gnu.org/bugzilla/show_bug.cgi?id=57157,
	// http://software.intel.com/en-us/forums/topic/580884
	// and http://llvm.org/bugs/show_bug.cgi?id=24226
	CRYPTOPP_CONSTANT(THIS_SIZE = sizeof(T)*8);
	CRYPTOPP_CONSTANT(MASK = THIS_SIZE-1);
	CRYPTOPP_ASSERT(static_cast<int>(y) < THIS_SIZE);
	return T((x >> y)|(x<<(-y&MASK)));
}

/// \brief Performs a left rotate
/// \tparam T the word type
/// \param x the value to rotate
/// \param y the number of bit positions to rotate the value
/// \details This is a portable C/C++ implementation. The value x to be rotated can be 8 to 64-bits wide.
/// \details y must be in the range <tt>[0, sizeof(T)*8 - 1]</tt> to avoid undefined behavior.
///  Use rotlMod if the rotate amount y is outside the range.
/// \note rotlVariable attempts to enlist a <tt>rotate IMM</tt> instruction because its often faster
///  than a <tt>rotate REG</tt>. Immediate rotates can be up to three times faster than their register
///  counterparts.
/// \sa rotlConstant, rotrConstant, rotlFixed, rotrFixed, rotlVariable, rotrVariable
/// \since Crypto++ 3.0
template <class T> inline T rotlVariable(T x, unsigned int y)
{
	CRYPTOPP_CONSTANT(THIS_SIZE = sizeof(T)*8);
	CRYPTOPP_CONSTANT(MASK = THIS_SIZE-1);
	CRYPTOPP_ASSERT(static_cast<int>(y) < THIS_SIZE);
	return T((x<<y)|(x>>(-y&MASK)));
}

/// \brief Performs a right rotate
/// \tparam T the word type
/// \param x the value to rotate
/// \param y the number of bit positions to rotate the value
/// \details This is a portable C/C++ implementation. The value x to be rotated can be 8 to 64-bits wide.
/// \details y must be in the range <tt>[0, sizeof(T)*8 - 1]</tt> to avoid undefined behavior.
///  Use rotrMod if the rotate amount y is outside the range.
/// \note rotrVariable attempts to enlist a <tt>rotate IMM</tt> instruction because its often faster
///  than a <tt>rotate REG</tt>. Immediate rotates can be up to three times faster than their register
///  counterparts.
/// \sa rotlConstant, rotrConstant, rotlFixed, rotrFixed, rotlVariable, rotrVariable
/// \since Crypto++ 3.0
template <class T> inline T rotrVariable(T x, unsigned int y)
{
	CRYPTOPP_CONSTANT(THIS_SIZE = sizeof(T)*8);
	CRYPTOPP_CONSTANT(MASK = THIS_SIZE-1);
	CRYPTOPP_ASSERT(static_cast<int>(y) < THIS_SIZE);
	return T((x>>y)|(x<<(-y&MASK)));
}

/// \brief Performs a left rotate
/// \tparam T the word type
/// \param x the value to rotate
/// \param y the number of bit positions to rotate the value
/// \details This is a portable C/C++ implementation. The value x to be rotated can be 8 to 64-bits wide.
/// \details y is reduced to the range <tt>[0, sizeof(T)*8 - 1]</tt> to avoid undefined behavior.
/// \note rotrVariable will use either <tt>rotate IMM</tt> or <tt>rotate REG</tt>.
/// \sa rotlConstant, rotrConstant, rotlFixed, rotrFixed, rotlVariable, rotrVariable
/// \since Crypto++ 3.0
template <class T> inline T rotlMod(T x, unsigned int y)
{
	CRYPTOPP_CONSTANT(THIS_SIZE = sizeof(T)*8);
	CRYPTOPP_CONSTANT(MASK = THIS_SIZE-1);
	return T((x<<(y&MASK))|(x>>(-y&MASK)));
}

/// \brief Performs a right rotate
/// \tparam T the word type
/// \param x the value to rotate
/// \param y the number of bit positions to rotate the value
/// \details This is a portable C/C++ implementation. The value x to be rotated can be 8 to 64-bits wide.
/// \details y is reduced to the range <tt>[0, sizeof(T)*8 - 1]</tt> to avoid undefined behavior.
/// \note rotrVariable will use either <tt>rotate IMM</tt> or <tt>rotate REG</tt>.
/// \sa rotlConstant, rotrConstant, rotlFixed, rotrFixed, rotlVariable, rotrVariable
/// \since Crypto++ 3.0
template <class T> inline T rotrMod(T x, unsigned int y)
{
	CRYPTOPP_CONSTANT(THIS_SIZE = sizeof(T)*8);
	CRYPTOPP_CONSTANT(MASK = THIS_SIZE-1);
	return T((x>>(y&MASK))|(x<<(-y&MASK)));
}

#ifdef _MSC_VER

/// \brief Performs a left rotate
/// \tparam T the word type
/// \param x the 32-bit value to rotate
/// \param y the number of bit positions to rotate the value
/// \details This is a Microsoft specific implementation using <tt>_lrotl</tt> provided by
///  <stdlib.h>. The value x to be rotated is 32-bits. y must be in the range
///  <tt>[0, sizeof(T)*8 - 1]</tt> to avoid undefined behavior.
/// \note rotlFixed will assert in Debug builds if is outside the allowed range.
/// \since Crypto++ 3.0
template<> inline word32 rotlFixed<word32>(word32 x, unsigned int y)
{
	// Uses Microsoft <stdlib.h> call, bound to C/C++ language rules.
	CRYPTOPP_ASSERT(y < 8*sizeof(x));
	return y ? _lrotl(x, static_cast<byte>(y)) : x;
}

/// \brief Performs a right rotate
/// \tparam T the word type
/// \param x the 32-bit value to rotate
/// \param y the number of bit positions to rotate the value
/// \details This is a Microsoft specific implementation using <tt>_lrotr</tt> provided by
///  <stdlib.h>. The value x to be rotated is 32-bits. y must be in the range
///  <tt>[0, sizeof(T)*8 - 1]</tt> to avoid undefined behavior.
/// \note rotrFixed will assert in Debug builds if is outside the allowed range.
/// \since Crypto++ 3.0
template<> inline word32 rotrFixed<word32>(word32 x, unsigned int y)
{
	// Uses Microsoft <stdlib.h> call, bound to C/C++ language rules.
	CRYPTOPP_ASSERT(y < 8*sizeof(x));
	return y ? _lrotr(x, static_cast<byte>(y)) : x;
}

/// \brief Performs a left rotate
/// \tparam T the word type
/// \param x the 32-bit value to rotate
/// \param y the number of bit positions to rotate the value
/// \details This is a Microsoft specific implementation using <tt>_lrotl</tt> provided by
///  <stdlib.h>. The value x to be rotated is 32-bits. y must be in the range
///  <tt>[0, sizeof(T)*8 - 1]</tt> to avoid undefined behavior.
/// \note rotlVariable will assert in Debug builds if is outside the allowed range.
/// \since Crypto++ 3.0
template<> inline word32 rotlVariable<word32>(word32 x, unsigned int y)
{
	CRYPTOPP_ASSERT(y < 8*sizeof(x));
	return _lrotl(x, static_cast<byte>(y));
}

/// \brief Performs a right rotate
/// \tparam T the word type
/// \param x the 32-bit value to rotate
/// \param y the number of bit positions to rotate the value
/// \details This is a Microsoft specific implementation using <tt>_lrotr</tt> provided by
///  <stdlib.h>. The value x to be rotated is 32-bits. y must be in the range
///  <tt>[0, sizeof(T)*8 - 1]</tt> to avoid undefined behavior.
/// \note rotrVariable will assert in Debug builds if is outside the allowed range.
/// \since Crypto++ 3.0
template<> inline word32 rotrVariable<word32>(word32 x, unsigned int y)
{
	CRYPTOPP_ASSERT(y < 8*sizeof(x));
	return _lrotr(x, static_cast<byte>(y));
}

/// \brief Performs a left rotate
/// \tparam T the word type
/// \param x the 32-bit value to rotate
/// \param y the number of bit positions to rotate the value
/// \details This is a Microsoft specific implementation using <tt>_lrotl</tt> provided by
///  <stdlib.h>. The value x to be rotated is 32-bits. y must be in the range
///  <tt>[0, sizeof(T)*8 - 1]</tt> to avoid undefined behavior.
/// \since Crypto++ 3.0
template<> inline word32 rotlMod<word32>(word32 x, unsigned int y)
{
	y %= 8*sizeof(x);
	return _lrotl(x, static_cast<byte>(y));
}

/// \brief Performs a right rotate
/// \tparam T the word type
/// \param x the 32-bit value to rotate
/// \param y the number of bit positions to rotate the value
/// \details This is a Microsoft specific implementation using <tt>_lrotr</tt> provided by
///  <stdlib.h>. The value x to be rotated is 32-bits. y must be in the range
///  <tt>[0, sizeof(T)*8 - 1]</tt> to avoid undefined behavior.
/// \since Crypto++ 3.0
template<> inline word32 rotrMod<word32>(word32 x, unsigned int y)
{
	y %= 8*sizeof(x);
	return _lrotr(x, static_cast<byte>(y));
}

#endif // #ifdef _MSC_VER

#if (_MSC_VER >= 1400) || (defined(_MSC_VER) && !defined(_DLL))
// Intel C++ Compiler 10.0 calls a function instead of using the rotate instruction when using these instructions

/// \brief Performs a left rotate
/// \tparam T the word type
/// \param x the 64-bit value to rotate
/// \param y the number of bit positions to rotate the value
/// \details This is a Microsoft specific implementation using <tt>_lrotl</tt> provided by
///  <stdlib.h>. The value x to be rotated is 64-bits. y must be in the range
///  <tt>[0, sizeof(T)*8 - 1]</tt> to avoid undefined behavior.
/// \note rotrFixed will assert in Debug builds if is outside the allowed range.
/// \since Crypto++ 3.0
template<> inline word64 rotlFixed<word64>(word64 x, unsigned int y)
{
	// Uses Microsoft <stdlib.h> call, bound to C/C++ language rules.
	CRYPTOPP_ASSERT(y < 8*sizeof(x));
	return y ? _rotl64(x, static_cast<byte>(y)) : x;
}

/// \brief Performs a right rotate
/// \tparam T the word type
/// \param x the 64-bit value to rotate
/// \param y the number of bit positions to rotate the value
/// \details This is a Microsoft specific implementation using <tt>_lrotr</tt> provided by
///  <stdlib.h>. The value x to be rotated is 64-bits. y must be in the range
///  <tt>[0, sizeof(T)*8 - 1]</tt> to avoid undefined behavior.
/// \note rotrFixed will assert in Debug builds if is outside the allowed range.
/// \since Crypto++ 3.0
template<> inline word64 rotrFixed<word64>(word64 x, unsigned int y)
{
	// Uses Microsoft <stdlib.h> call, bound to C/C++ language rules.
	CRYPTOPP_ASSERT(y < 8*sizeof(x));
	return y ? _rotr64(x, static_cast<byte>(y)) : x;
}

/// \brief Performs a left rotate
/// \tparam T the word type
/// \param x the 64-bit value to rotate
/// \param y the number of bit positions to rotate the value
/// \details This is a Microsoft specific implementation using <tt>_lrotl</tt> provided by
///  <stdlib.h>. The value x to be rotated is 64-bits. y must be in the range
///  <tt>[0, sizeof(T)*8 - 1]</tt> to avoid undefined behavior.
/// \note rotlVariable will assert in Debug builds if is outside the allowed range.
/// \since Crypto++ 3.0
template<> inline word64 rotlVariable<word64>(word64 x, unsigned int y)
{
	CRYPTOPP_ASSERT(y < 8*sizeof(x));
	return _rotl64(x, static_cast<byte>(y));
}

/// \brief Performs a right rotate
/// \tparam T the word type
/// \param x the 64-bit value to rotate
/// \param y the number of bit positions to rotate the value
/// \details This is a Microsoft specific implementation using <tt>_lrotr</tt> provided by
///  <stdlib.h>. The value x to be rotated is 64-bits. y must be in the range
///  <tt>[0, sizeof(T)*8 - 1]</tt> to avoid undefined behavior.
/// \note rotrVariable will assert in Debug builds if is outside the allowed range.
/// \since Crypto++ 3.0
template<> inline word64 rotrVariable<word64>(word64 x, unsigned int y)
{
	CRYPTOPP_ASSERT(y < 8*sizeof(x));
	return y ? _rotr64(x, static_cast<byte>(y)) : x;
}

/// \brief Performs a left rotate
/// \tparam T the word type
/// \param x the 64-bit value to rotate
/// \param y the number of bit positions to rotate the value
/// \details This is a Microsoft specific implementation using <tt>_lrotl</tt> provided by
///  <stdlib.h>. The value x to be rotated is 64-bits. y must be in the range
///  <tt>[0, sizeof(T)*8 - 1]</tt> to avoid undefined behavior.
/// \since Crypto++ 3.0
template<> inline word64 rotlMod<word64>(word64 x, unsigned int y)
{
	CRYPTOPP_ASSERT(y < 8*sizeof(x));
	return y ? _rotl64(x, static_cast<byte>(y)) : x;
}

/// \brief Performs a right rotate
/// \tparam T the word type
/// \param x the 64-bit value to rotate
/// \param y the number of bit positions to rotate the value
/// \details This is a Microsoft specific implementation using <tt>_lrotr</tt> provided by
///  <stdlib.h>. The value x to be rotated is 64-bits. y must be in the range
///  <tt>[0, sizeof(T)*8 - 1]</tt> to avoid undefined behavior.
/// \since Crypto++ 3.0
template<> inline word64 rotrMod<word64>(word64 x, unsigned int y)
{
	CRYPTOPP_ASSERT(y < 8*sizeof(x));
	return y ? _rotr64(x, static_cast<byte>(y)) : x;
}

#endif // #if _MSC_VER >= 1310

#if _MSC_VER >= 1400 && !defined(__INTEL_COMPILER)
// Intel C++ Compiler 10.0 gives undefined externals with these
template<> inline word16 rotlFixed<word16>(word16 x, unsigned int y)
{
	// Intrinsic, not bound to C/C++ language rules.
	return _rotl16(x, static_cast<byte>(y));
}

template<> inline word16 rotrFixed<word16>(word16 x, unsigned int y)
{
	// Intrinsic, not bound to C/C++ language rules.
	return _rotr16(x, static_cast<byte>(y));
}

template<> inline word16 rotlVariable<word16>(word16 x, unsigned int y)
{
	return _rotl16(x, static_cast<byte>(y));
}

template<> inline word16 rotrVariable<word16>(word16 x, unsigned int y)
{
	return _rotr16(x, static_cast<byte>(y));
}

template<> inline word16 rotlMod<word16>(word16 x, unsigned int y)
{
	return _rotl16(x, static_cast<byte>(y));
}

template<> inline word16 rotrMod<word16>(word16 x, unsigned int y)
{
	return _rotr16(x, static_cast<byte>(y));
}

template<> inline byte rotlFixed<byte>(byte x, unsigned int y)
{
	// Intrinsic, not bound to C/C++ language rules.
	return _rotl8(x, static_cast<byte>(y));
}

template<> inline byte rotrFixed<byte>(byte x, unsigned int y)
{
	// Intrinsic, not bound to C/C++ language rules.
	return _rotr8(x, static_cast<byte>(y));
}

template<> inline byte rotlVariable<byte>(byte x, unsigned int y)
{
	return _rotl8(x, static_cast<byte>(y));
}

template<> inline byte rotrVariable<byte>(byte x, unsigned int y)
{
	return _rotr8(x, static_cast<byte>(y));
}

template<> inline byte rotlMod<byte>(byte x, unsigned int y)
{
	return _rotl8(x, static_cast<byte>(y));
}

template<> inline byte rotrMod<byte>(byte x, unsigned int y)
{
	return _rotr8(x, static_cast<byte>(y));
}

#endif // #if _MSC_VER >= 1400

#if (defined(__MWERKS__) && TARGET_CPU_PPC)

template<> inline word32 rotlFixed<word32>(word32 x, unsigned int y)
{
	CRYPTOPP_ASSERT(y < 32);
	return y ? __rlwinm(x,y,0,31) : x;
}

template<> inline word32 rotrFixed<word32>(word32 x, unsigned int y)
{
	CRYPTOPP_ASSERT(y < 32);
	return y ? __rlwinm(x,32-y,0,31) : x;
}

template<> inline word32 rotlVariable<word32>(word32 x, unsigned int y)
{
	CRYPTOPP_ASSERT(y < 32);
	return (__rlwnm(x,y,0,31));
}

template<> inline word32 rotrVariable<word32>(word32 x, unsigned int y)
{
	CRYPTOPP_ASSERT(y < 32);
	return (__rlwnm(x,32-y,0,31));
}

template<> inline word32 rotlMod<word32>(word32 x, unsigned int y)
{
	return (__rlwnm(x,y,0,31));
}

template<> inline word32 rotrMod<word32>(word32 x, unsigned int y)
{
	return (__rlwnm(x,32-y,0,31));
}

#endif // __MWERKS__ && TARGET_CPU_PPC

// ************** endian reversal ***************

/// \brief Gets a byte from a value
/// \param order the ByteOrder of the value
/// \param value the value to retrieve the byte
/// \param index the location of the byte to retrieve
template <class T>
inline unsigned int GetByte(ByteOrder order, T value, unsigned int index)
{
	if (order == LITTLE_ENDIAN_ORDER)
		return GETBYTE(value, index);
	else
		return GETBYTE(value, sizeof(T)-index-1);
}

/// \brief Reverses bytes in a 8-bit value
/// \param value the 8-bit value to reverse
/// \note ByteReverse returns the value passed to it since there is nothing to
///  reverse.
inline byte ByteReverse(byte value)
{
	return value;
}

/// \brief Reverses bytes in a 16-bit value
/// \param value the 16-bit value to reverse
/// \details ByteReverse calls bswap if available. Otherwise the function
///  performs a 8-bit rotate on the word16.
inline word16 ByteReverse(word16 value)
{
#if defined(CRYPTOPP_BYTESWAP_AVAILABLE)
	return bswap_16(value);
#elif (_MSC_VER >= 1400) || (defined(_MSC_VER) && !defined(_DLL))
	return _byteswap_ushort(value);
#else
	return rotlFixed(value, 8U);
#endif
}

/// \brief Reverses bytes in a 32-bit value
/// \param value the 32-bit value to reverse
/// \details ByteReverse calls bswap if available. Otherwise the function uses
///  a combination of rotates on the word32.
inline word32 ByteReverse(word32 value)
{
#if defined(CRYPTOPP_BYTESWAP_AVAILABLE)
	return bswap_32(value);
#elif defined(CRYPTOPP_ARM_BYTEREV_AVAILABLE)
	word32 rvalue;
	__asm__ ("rev %0, %1" : "=r" (rvalue) : "r" (value));
	return rvalue;
#elif defined(__GNUC__) && defined(CRYPTOPP_X86_ASM_AVAILABLE)
	__asm__ ("bswap %0" : "=r" (value) : "0" (value));
	return value;
#elif defined(__MWERKS__) && TARGET_CPU_PPC
	return (word32)__lwbrx(&value,0);
#elif (_MSC_VER >= 1400) || (defined(_MSC_VER) && !defined(_DLL))
	return _byteswap_ulong(value);
#elif CRYPTOPP_FAST_ROTATE(32) && !defined(__xlC__)
	// 5 instructions with rotate instruction, 9 without
	return (rotrFixed(value, 8U) & 0xff00ff00) | (rotlFixed(value, 8U) & 0x00ff00ff);
#else
	// 6 instructions with rotate instruction, 8 without
	value = ((value & 0xFF00FF00) >> 8) | ((value & 0x00FF00FF) << 8);
	return rotlFixed(value, 16U);
#endif
}

/// \brief Reverses bytes in a 64-bit value
/// \param value the 64-bit value to reverse
/// \details ByteReverse calls bswap if available. Otherwise the function uses
///  a combination of rotates on the word64.
inline word64 ByteReverse(word64 value)
{
#if defined(CRYPTOPP_BYTESWAP_AVAILABLE)
	return bswap_64(value);
#elif defined(__GNUC__) && defined(CRYPTOPP_X86_ASM_AVAILABLE) && defined(__x86_64__)
	__asm__ ("bswap %0" : "=r" (value) : "0" (value));
	return value;
#elif (_MSC_VER >= 1400) || (defined(_MSC_VER) && !defined(_DLL))
	return _byteswap_uint64(value);
#elif CRYPTOPP_BOOL_SLOW_WORD64
	return (word64(ByteReverse(word32(value))) << 32) | ByteReverse(word32(value>>32));
#else
	value = ((value & W64LIT(0xFF00FF00FF00FF00)) >> 8) | ((value & W64LIT(0x00FF00FF00FF00FF)) << 8);
	value = ((value & W64LIT(0xFFFF0000FFFF0000)) >> 16) | ((value & W64LIT(0x0000FFFF0000FFFF)) << 16);
	return rotlFixed(value, 32U);
#endif
}

/// \brief Reverses bits in a 8-bit value
/// \param value the 8-bit value to reverse
/// \details BitReverse performs a combination of shifts on the byte.
inline byte BitReverse(byte value)
{
	value = byte((value & 0xAA) >> 1) | byte((value & 0x55) << 1);
	value = byte((value & 0xCC) >> 2) | byte((value & 0x33) << 2);
	return rotlFixed(value, 4U);
}

/// \brief Reverses bits in a 16-bit value
/// \param value the 16-bit value to reverse
/// \details BitReverse performs a combination of shifts on the word16.
inline word16 BitReverse(word16 value)
{
#if defined(CRYPTOPP_ARM_BITREV_AVAILABLE)
	// 4 instructions on ARM.
	word32 rvalue;
	__asm__ ("rbit %0, %1" : "=r" (rvalue) : "r" (value));
	return word16(rvalue >> 16);
#else
	// 15 instructions on ARM.
	value = word16((value & 0xAAAA) >> 1) | word16((value & 0x5555) << 1);
	value = word16((value & 0xCCCC) >> 2) | word16((value & 0x3333) << 2);
	value = word16((value & 0xF0F0) >> 4) | word16((value & 0x0F0F) << 4);
	return ByteReverse(value);
#endif
}

/// \brief Reverses bits in a 32-bit value
/// \param value the 32-bit value to reverse
/// \details BitReverse performs a combination of shifts on the word32.
inline word32 BitReverse(word32 value)
{
#if defined(CRYPTOPP_ARM_BITREV_AVAILABLE)
	// 2 instructions on ARM.
	word32 rvalue;
	__asm__ ("rbit %0, %1" : "=r" (rvalue) : "r" (value));
	return rvalue;
#else
	// 19 instructions on ARM.
	value = word32((value & 0xAAAAAAAA) >> 1) | word32((value & 0x55555555) << 1);
	value = word32((value & 0xCCCCCCCC) >> 2) | word32((value & 0x33333333) << 2);
	value = word32((value & 0xF0F0F0F0) >> 4) | word32((value & 0x0F0F0F0F) << 4);
	return ByteReverse(value);
#endif
}

/// \brief Reverses bits in a 64-bit value
/// \param value the 64-bit value to reverse
/// \details BitReverse performs a combination of shifts on the word64.
inline word64 BitReverse(word64 value)
{
#if CRYPTOPP_BOOL_SLOW_WORD64
	return (word64(BitReverse(word32(value))) << 32) | BitReverse(word32(value>>32));
#else
	value = word64((value & W64LIT(0xAAAAAAAAAAAAAAAA)) >> 1) | word64((value & W64LIT(0x5555555555555555)) << 1);
	value = word64((value & W64LIT(0xCCCCCCCCCCCCCCCC)) >> 2) | word64((value & W64LIT(0x3333333333333333)) << 2);
	value = word64((value & W64LIT(0xF0F0F0F0F0F0F0F0)) >> 4) | word64((value & W64LIT(0x0F0F0F0F0F0F0F0F)) << 4);
	return ByteReverse(value);
#endif
}

/// \brief Reverses bits in a value
/// \param value the value to reverse
/// \details The template overload of BitReverse operates on signed and unsigned values.
///  Internally the size of T is checked, and then value is cast to a byte,
///  word16, word32 or word64. After the cast, the appropriate BitReverse
///  overload is called.
template <class T>
inline T BitReverse(T value)
{
	if (sizeof(T) == 1)
		return (T)BitReverse((byte)value);
	else if (sizeof(T) == 2)
		return (T)BitReverse((word16)value);
	else if (sizeof(T) == 4)
		return (T)BitReverse((word32)value);
	else if (sizeof(T) == 8)
		return (T)BitReverse((word64)value);
	else
	{
		CRYPTOPP_ASSERT(0);
		return (T)BitReverse((word64)value);
	}
}

/// \brief Reverses bytes in a value depending upon endianness
/// \tparam T the class or type
/// \param order the ByteOrder of the data
/// \param value the value to conditionally reverse
/// \details Internally, the ConditionalByteReverse calls NativeByteOrderIs.
///  If order matches native byte order, then the original value is returned.
///  If not, then ByteReverse is called on the value before returning to the caller.
template <class T>
inline T ConditionalByteReverse(ByteOrder order, T value)
{
	return NativeByteOrderIs(order) ? value : ByteReverse(value);
}

/// \brief Reverses bytes in an element from an array of elements
/// \tparam T the class or type
/// \param out the output array of elements
/// \param in the input array of elements
/// \param byteCount the total number of bytes in the array
/// \details Internally, ByteReverse visits each element in the in array
///  calls ByteReverse on it, and writes the result to out.
/// \details ByteReverse does not process tail byes, or bytes that are
///  not part of a full element. If T is int (and int is 4 bytes), then
///  <tt>byteCount = 10</tt> means only the first 2 elements or 8 bytes are
///  reversed.
/// \details The following program should help illustrate the behavior.
/// <pre>vector<word32> v1, v2;
///
/// v1.push_back(1);
/// v1.push_back(2);
/// v1.push_back(3);
/// v1.push_back(4);
///
/// v2.resize(v1.size());
/// ByteReverse<word32>(&v2[0], &v1[0], 16);
///
/// cout << "V1: ";
/// for(unsigned int i = 0; i < v1.size(); i++)
///  cout << std::hex << v1[i] << " ";
/// cout << endl;
///
/// cout << "V2: ";
/// for(unsigned int i = 0; i < v2.size(); i++)
///  cout << std::hex << v2[i] << " ";
/// cout << endl;</pre>
/// The program above results in the following output.
/// <pre>V1: 00000001 00000002 00000003 00000004
/// V2: 01000000 02000000 03000000 04000000</pre>
/// \sa ConditionalByteReverse
template <class T>
void ByteReverse(T *out, const T *in, size_t byteCount)
{
	// Alignment check due to Issues 690
	CRYPTOPP_ASSERT(byteCount % sizeof(T) == 0);
	CRYPTOPP_ASSERT(IsAligned<T>(in));
	CRYPTOPP_ASSERT(IsAligned<T>(out));

	size_t count = byteCount/sizeof(T);
	for (size_t i=0; i<count; i++)
		out[i] = ByteReverse(in[i]);
}

/// \brief Conditionally reverses bytes in an element from an array of elements
/// \tparam T the class or type
/// \param order the ByteOrder of the data
/// \param out the output array of elements
/// \param in the input array of elements
/// \param byteCount the byte count of the arrays
/// \details Internally, ByteReverse visits each element in the in array
///  calls ByteReverse on it depending on the desired endianness, and writes the result to out.
/// \details ByteReverse does not process tail byes, or bytes that are
///  not part of a full element. If T is int (and int is 4 bytes), then
///  <tt>byteCount = 10</tt> means only the first 2 elements or 8 bytes are
///  reversed.
/// \sa ByteReverse
template <class T>
inline void ConditionalByteReverse(ByteOrder order, T *out, const T *in, size_t byteCount)
{
	if (!NativeByteOrderIs(order))
		ByteReverse(out, in, byteCount);
	else if (in != out)
		memcpy_s(out, byteCount, in, byteCount);
}

template <class T>
inline void GetUserKey(ByteOrder order, T *out, size_t outlen, const byte *in, size_t inlen)
{
	const size_t U = sizeof(T);
	CRYPTOPP_ASSERT(inlen <= outlen*U);
	memcpy_s(out, outlen*U, in, inlen);
	memset_z((byte *)out+inlen, 0, outlen*U-inlen);
	ConditionalByteReverse(order, out, out, RoundUpToMultipleOf(inlen, U));
}

inline byte UnalignedGetWordNonTemplate(ByteOrder order, const byte *block, const byte *)
{
	CRYPTOPP_UNUSED(order);
	return block[0];
}

inline word16 UnalignedGetWordNonTemplate(ByteOrder order, const byte *block, const word16 *)
{
	return (order == BIG_ENDIAN_ORDER)
		? block[1] | (block[0] << 8)
		: block[0] | (block[1] << 8);
}

inline word32 UnalignedGetWordNonTemplate(ByteOrder order, const byte *block, const word32 *)
{
	return (order == BIG_ENDIAN_ORDER)
		? word32(block[3]) | (word32(block[2]) << 8) | (word32(block[1]) << 16) | (word32(block[0]) << 24)
		: word32(block[0]) | (word32(block[1]) << 8) | (word32(block[2]) << 16) | (word32(block[3]) << 24);
}

inline word64 UnalignedGetWordNonTemplate(ByteOrder order, const byte *block, const word64 *)
{
	return (order == BIG_ENDIAN_ORDER)
		?
		(word64(block[7]) |
		(word64(block[6]) <<  8) |
		(word64(block[5]) << 16) |
		(word64(block[4]) << 24) |
		(word64(block[3]) << 32) |
		(word64(block[2]) << 40) |
		(word64(block[1]) << 48) |
		(word64(block[0]) << 56))
		:
		(word64(block[0]) |
		(word64(block[1]) <<  8) |
		(word64(block[2]) << 16) |
		(word64(block[3]) << 24) |
		(word64(block[4]) << 32) |
		(word64(block[5]) << 40) |
		(word64(block[6]) << 48) |
		(word64(block[7]) << 56));
}

inline void UnalignedbyteNonTemplate(ByteOrder order, byte *block, byte value, const byte *xorBlock)
{
	CRYPTOPP_UNUSED(order);
	block[0] = static_cast<byte>(xorBlock ? (value ^ xorBlock[0]) : value);
}

inline void UnalignedbyteNonTemplate(ByteOrder order, byte *block, word16 value, const byte *xorBlock)
{
	if (order == BIG_ENDIAN_ORDER)
	{
		if (xorBlock)
		{
			block[0] = xorBlock[0] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 1);
			block[1] = xorBlock[1] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 0);
		}
		else
		{
			block[0] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 1);
			block[1] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 0);
		}
	}
	else
	{
		if (xorBlock)
		{
			block[0] = xorBlock[0] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 0);
			block[1] = xorBlock[1] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 1);
		}
		else
		{
			block[0] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 0);
			block[1] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 1);
		}
	}
}

inline void UnalignedbyteNonTemplate(ByteOrder order, byte *block, word32 value, const byte *xorBlock)
{
	if (order == BIG_ENDIAN_ORDER)
	{
		if (xorBlock)
		{
			block[0] = xorBlock[0] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 3);
			block[1] = xorBlock[1] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 2);
			block[2] = xorBlock[2] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 1);
			block[3] = xorBlock[3] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 0);
		}
		else
		{
			block[0] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 3);
			block[1] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 2);
			block[2] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 1);
			block[3] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 0);
		}
	}
	else
	{
		if (xorBlock)
		{
			block[0] = xorBlock[0] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 0);
			block[1] = xorBlock[1] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 1);
			block[2] = xorBlock[2] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 2);
			block[3] = xorBlock[3] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 3);
		}
		else
		{
			block[0] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 0);
			block[1] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 1);
			block[2] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 2);
			block[3] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 3);
		}
	}
}

inline void UnalignedbyteNonTemplate(ByteOrder order, byte *block, word64 value, const byte *xorBlock)
{
	if (order == BIG_ENDIAN_ORDER)
	{
		if (xorBlock)
		{
			block[0] = xorBlock[0] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 7);
			block[1] = xorBlock[1] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 6);
			block[2] = xorBlock[2] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 5);
			block[3] = xorBlock[3] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 4);
			block[4] = xorBlock[4] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 3);
			block[5] = xorBlock[5] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 2);
			block[6] = xorBlock[6] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 1);
			block[7] = xorBlock[7] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 0);
		}
		else
		{
			block[0] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 7);
			block[1] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 6);
			block[2] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 5);
			block[3] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 4);
			block[4] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 3);
			block[5] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 2);
			block[6] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 1);
			block[7] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 0);
		}
	}
	else
	{
		if (xorBlock)
		{
			block[0] = xorBlock[0] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 0);
			block[1] = xorBlock[1] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 1);
			block[2] = xorBlock[2] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 2);
			block[3] = xorBlock[3] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 3);
			block[4] = xorBlock[4] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 4);
			block[5] = xorBlock[5] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 5);
			block[6] = xorBlock[6] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 6);
			block[7] = xorBlock[7] ^ CRYPTOPP_GET_BYTE_AS_BYTE(value, 7);
		}
		else
		{
			block[0] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 0);
			block[1] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 1);
			block[2] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 2);
			block[3] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 3);
			block[4] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 4);
			block[5] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 5);
			block[6] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 6);
			block[7] = CRYPTOPP_GET_BYTE_AS_BYTE(value, 7);
		}
	}
}

/// \brief Access a block of memory
/// \tparam T class or type
/// \param assumeAligned flag indicating alignment
/// \param order the ByteOrder of the data
/// \param block the byte buffer to be processed
/// \returns the word in the specified byte order
/// \details GetWord() provides alternate read access to a block of memory. The flag assumeAligned indicates
///  if the memory block is aligned for class or type T. The enumeration ByteOrder is BIG_ENDIAN_ORDER or
///  LITTLE_ENDIAN_ORDER.
/// \details An example of reading two word32 values from a block of memory is shown below. <tt>w</tt>
///  will be <tt>0x03020100</tt>.
/// <pre>
///   word32 w;
///   byte buffer[4] = {0,1,2,3};
///   w = GetWord<word32>(false, LITTLE_ENDIAN_ORDER, buffer);
/// </pre>
template <class T>
inline T GetWord(bool assumeAligned, ByteOrder order, const byte *block)
{
	CRYPTOPP_UNUSED(assumeAligned);

	T temp = 0;
	if (block != NULLPTR) {std::memcpy(&temp, block, sizeof(T));}
	return ConditionalByteReverse(order, temp);
}

/// \brief Access a block of memory
/// \tparam T class or type
/// \param assumeAligned flag indicating alignment
/// \param order the ByteOrder of the data
/// \param result the word in the specified byte order
/// \param block the byte buffer to be processed
/// \details GetWord() provides alternate read access to a block of memory. The flag assumeAligned indicates
///  if the memory block is aligned for class or type T. The enumeration ByteOrder is BIG_ENDIAN_ORDER or
///  LITTLE_ENDIAN_ORDER.
/// \details An example of reading two word32 values from a block of memory is shown below. <tt>w</tt>
///  will be <tt>0x03020100</tt>.
/// <pre>
///   word32 w;
///   byte buffer[4] = {0,1,2,3};
///   w = GetWord<word32>(false, LITTLE_ENDIAN_ORDER, buffer);
/// </pre>
template <class T>
inline void GetWord(bool assumeAligned, ByteOrder order, T &result, const byte *block)
{
	result = GetWord<T>(assumeAligned, order, block);
}

/// \brief Access a block of memory
/// \tparam T class or type
/// \param assumeAligned flag indicating alignment
/// \param order the ByteOrder of the data
/// \param block the destination byte buffer
/// \param value the word in the specified byte order
/// \param xorBlock an optional byte buffer to xor
/// \details PutWord() provides alternate write access to a block of memory. The flag assumeAligned indicates
///  if the memory block is aligned for class or type T. The enumeration ByteOrder is BIG_ENDIAN_ORDER or
///  LITTLE_ENDIAN_ORDER.
template <class T>
inline void PutWord(bool assumeAligned, ByteOrder order, byte *block, T value, const byte *xorBlock = NULLPTR)
{
	CRYPTOPP_UNUSED(assumeAligned);

	T t1, t2;
	t1 = ConditionalByteReverse(order, value);
	if (xorBlock != NULLPTR) {std::memcpy(&t2, xorBlock, sizeof(T)); t1 ^= t2;}
	if (block != NULLPTR) {std::memcpy(block, &t1, sizeof(T));}
}

/// \brief Access a block of memory
/// \tparam T class or type
/// \tparam B enumeration indicating endianness
/// \tparam A flag indicating alignment
/// \details GetBlock() provides alternate read access to a block of memory. The enumeration B is
///  BigEndian or LittleEndian. The flag A indicates if the memory block is aligned for class or type T.
///  Repeatedly applying operator() results in advancing in the block of memory.
/// \details An example of reading two word32 values from a block of memory is shown below. <tt>w1</tt>
///  will be <tt>0x03020100</tt> and <tt>w1</tt> will be <tt>0x07060504</tt>.
/// <pre>
///   word32 w1, w2;
///   byte buffer[8] = {0,1,2,3,4,5,6,7};
///   GetBlock<word32, LittleEndian> block(buffer);
///   block(w1)(w2);
/// </pre>
template <class T, class B, bool A=false>
class GetBlock
{
public:
	/// \brief Construct a GetBlock
	/// \param block the memory block
	GetBlock(const void *block)
		: m_block((const byte *)block) {}

	/// \brief Access a block of memory
	/// \tparam U class or type
	/// \param x the value to read
	/// \returns pointer to the remainder of the block after reading x
	template <class U>
	inline GetBlock<T, B, A> & operator()(U &x)
	{
		CRYPTOPP_COMPILE_ASSERT(sizeof(U) >= sizeof(T));
		x = GetWord<T>(A, B::ToEnum(), m_block);
		m_block += sizeof(T);
		return *this;
	}

private:
	const byte *m_block;
};

/// \brief Access a block of memory
/// \tparam T class or type
/// \tparam B enumeration indicating endianness
/// \tparam A flag indicating alignment
/// \details PutBlock() provides alternate write access to a block of memory. The enumeration B is
///  BigEndian or LittleEndian. The flag A indicates if the memory block is aligned for class or type T.
///  Repeatedly applying operator() results in advancing in the block of memory.
/// \details An example of writing two word32 values from a block of memory is shown below. After the code
///  executes, the byte buffer will be <tt>{0,1,2,3,4,5,6,7}</tt>.
/// <pre>
///   word32 w1=0x03020100, w2=0x07060504;
///   byte buffer[8];
///   PutBlock<word32, LittleEndian> block(NULLPTR, buffer);
///   block(w1)(w2);
/// </pre>
template <class T, class B, bool A=false>
class PutBlock
{
public:
	/// \brief Construct a PutBlock
	/// \param block the memory block
	/// \param xorBlock optional mask
	PutBlock(const void *xorBlock, void *block)
		: m_xorBlock((const byte *)xorBlock), m_block((byte *)block) {}

	/// \brief Access a block of memory
	/// \tparam U class or type
	/// \param x the value to write
	/// \returns pointer to the remainder of the block after writing x
	template <class U>
	inline PutBlock<T, B, A> & operator()(U x)
	{
		PutWord(A, B::ToEnum(), m_block, (T)x, m_xorBlock);
		m_block += sizeof(T);
		if (m_xorBlock)
			m_xorBlock += sizeof(T);
		return *this;
	}

private:
	const byte *m_xorBlock;
	byte *m_block;
};

/// \brief Access a block of memory
/// \tparam T class or type
/// \tparam B enumeration indicating endianness
/// \tparam GA flag indicating alignment for the Get operation
/// \tparam PA flag indicating alignment for the Put operation
/// \details GetBlock() provides alternate write access to a block of memory. The enumeration B is
///  BigEndian or LittleEndian. The flag A indicates if the memory block is aligned for class or type T.
/// \sa GetBlock() and PutBlock().
template <class T, class B, bool GA=false, bool PA=false>
struct BlockGetAndPut
{
	// function needed because of C++ grammatical ambiguity between expression-statements and declarations
	static inline GetBlock<T, B, GA> Get(const void *block) {return GetBlock<T, B, GA>(block);}
	typedef PutBlock<T, B, PA> Put;
};

/// \brief Convert a word to a string
/// \tparam T class or type
/// \param value the word to convert
/// \param order byte order
/// \returns a string representing the value of the word
template <class T>
std::string WordToString(T value, ByteOrder order = BIG_ENDIAN_ORDER)
{
	if (!NativeByteOrderIs(order))
		value = ByteReverse(value);

	return std::string((char *)&value, sizeof(value));
}

/// \brief Convert a string to a word
/// \tparam T class or type
/// \param str the string to convert
/// \param order byte order
/// \returns a word representing the value of the string
template <class T>
T StringToWord(const std::string &str, ByteOrder order = BIG_ENDIAN_ORDER)
{
	T value = 0;
	memcpy_s(&value, sizeof(value), str.data(), UnsignedMin(str.size(), sizeof(value)));
	return NativeByteOrderIs(order) ? value : ByteReverse(value);
}

// ************** help remove warning on g++ ***************

/// \brief Safely shift values when undefined behavior could occur
/// \tparam overflow boolean flag indicating if overflow is present
/// \details SafeShifter safely shifts values when undefined behavior could occur under C/C++ rules.
///  The class behaves much like a saturating arithmetic class, clamping values rather than allowing
///  the compiler to remove undefined behavior.
/// \sa SafeShifter<true>, SafeShifter<false>
template <bool overflow> struct SafeShifter;

/// \brief Shifts a value in the presence of overflow
/// \details the true template parameter indicates overflow would occur.
///  In this case, SafeShifter clamps the value and returns 0.
template<> struct SafeShifter<true>
{
	/// \brief Right shifts a value that overflows
	/// \tparam T class or type
	/// \return 0
	/// \details Since <tt>overflow == true</tt>, the value 0 is always returned.
	/// \sa SafeLeftShift
	template <class T>
	static inline T RightShift(T value, unsigned int bits)
	{
		CRYPTOPP_UNUSED(value); CRYPTOPP_UNUSED(bits);
		return 0;
	}

	/// \brief Left shifts a value that overflows
	/// \tparam T class or type
	/// \return 0
	/// \details Since <tt>overflow == true</tt>, the value 0 is always returned.
	/// \sa SafeRightShift
	template <class T>
	static inline T LeftShift(T value, unsigned int bits)
	{
		CRYPTOPP_UNUSED(value); CRYPTOPP_UNUSED(bits);
		return 0;
	}
};

/// \brief Shifts a value in the absence of overflow
/// \details the false template parameter indicates overflow would not occur.
///  In this case, SafeShifter returns the shfted value.
template<> struct SafeShifter<false>
{
	/// \brief Right shifts a value that does not overflow
	/// \tparam T class or type
	/// \return the shifted value
	/// \details Since <tt>overflow == false</tt>, the shifted value is returned.
	/// \sa SafeLeftShift
	template <class T>
	static inline T RightShift(T value, unsigned int bits)
	{
		return value >> bits;
	}

	/// \brief Left shifts a value that does not overflow
	/// \tparam T class or type
	/// \return the shifted value
	/// \details Since <tt>overflow == false</tt>, the shifted value is returned.
	/// \sa SafeRightShift
	template <class T>
	static inline T LeftShift(T value, unsigned int bits)
	{
		return value << bits;
	}
};

/// \brief Safely right shift values when undefined behavior could occur
/// \tparam bits the number of bit positions to shift the value
/// \tparam T class or type
/// \param value the value to right shift
/// \result the shifted value or 0
/// \details SafeRightShift safely shifts the value to the right when undefined behavior
///  could occur under C/C++ rules. SafeRightShift will return the shifted value or 0
///  if undefined behavior would occur.
template <unsigned int bits, class T>
inline T SafeRightShift(T value)
{
	return SafeShifter<(bits>=(8*sizeof(T)))>::RightShift(value, bits);
}

/// \brief Safely left shift values when undefined behavior could occur
/// \tparam bits the number of bit positions to shift the value
/// \tparam T class or type
/// \param value the value to left shift
/// \result the shifted value or 0
/// \details SafeLeftShift safely shifts the value to the left when undefined behavior
///  could occur under C/C++ rules. SafeLeftShift will return the shifted value or 0
///  if undefined behavior would occur.
template <unsigned int bits, class T>
inline T SafeLeftShift(T value)
{
	return SafeShifter<(bits>=(8*sizeof(T)))>::LeftShift(value, bits);
}

/// \brief Finds first element not in a range
/// \tparam InputIt Input iterator type
/// \tparam T class or type
/// \param first iterator to first element
/// \param last iterator to last element
/// \param value the value used as a predicate
/// \returns iterator to the first element in the range that is not value
template<typename InputIt, typename T>
inline InputIt FindIfNot(InputIt first, InputIt last, const T &value) {
#ifdef CRYPTOPP_CXX11_LAMBDA
    return std::find_if(first, last, [&value](const T &o) {
        return value!=o;
    });
#else
    return std::find_if(first, last, std::bind2nd(std::not_equal_to<T>(), value));
#endif
}

// ************** use one buffer for multiple data members ***************

#define CRYPTOPP_BLOCK_1(n, t, s) t* m_##n() {return (t *)(void *)(m_aggregate+0);}     size_t SS1() {return       sizeof(t)*(s);} size_t m_##n##Size() {return (s);}
#define CRYPTOPP_BLOCK_2(n, t, s) t* m_##n() {return (t *)(void *)(m_aggregate+SS1());} size_t SS2() {return SS1()+sizeof(t)*(s);} size_t m_##n##Size() {return (s);}
#define CRYPTOPP_BLOCK_3(n, t, s) t* m_##n() {return (t *)(void *)(m_aggregate+SS2());} size_t SS3() {return SS2()+sizeof(t)*(s);} size_t m_##n##Size() {return (s);}
#define CRYPTOPP_BLOCK_4(n, t, s) t* m_##n() {return (t *)(void *)(m_aggregate+SS3());} size_t SS4() {return SS3()+sizeof(t)*(s);} size_t m_##n##Size() {return (s);}
#define CRYPTOPP_BLOCK_5(n, t, s) t* m_##n() {return (t *)(void *)(m_aggregate+SS4());} size_t SS5() {return SS4()+sizeof(t)*(s);} size_t m_##n##Size() {return (s);}
#define CRYPTOPP_BLOCK_6(n, t, s) t* m_##n() {return (t *)(void *)(m_aggregate+SS5());} size_t SS6() {return SS5()+sizeof(t)*(s);} size_t m_##n##Size() {return (s);}
#define CRYPTOPP_BLOCK_7(n, t, s) t* m_##n() {return (t *)(void *)(m_aggregate+SS6());} size_t SS7() {return SS6()+sizeof(t)*(s);} size_t m_##n##Size() {return (s);}
#define CRYPTOPP_BLOCK_8(n, t, s) t* m_##n() {return (t *)(void *)(m_aggregate+SS7());} size_t SS8() {return SS7()+sizeof(t)*(s);} size_t m_##n##Size() {return (s);}
#define CRYPTOPP_BLOCKS_END(i) size_t SST() {return SS##i();} void AllocateBlocks() {m_aggregate.New(SST());} AlignedSecByteBlock m_aggregate;

NAMESPACE_END

#if (CRYPTOPP_MSC_VERSION)
# pragma warning(pop)
#endif

#if CRYPTOPP_GCC_DIAGNOSTIC_AVAILABLE
# pragma GCC diagnostic pop
#endif

#endif
