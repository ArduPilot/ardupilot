// cpu.cpp - originally written and placed in the public domain by Wei Dai
//           modified by Jeffrey Walton and the community over the years.

#include "pch.h"
#include "config.h"

#ifndef EXCEPTION_EXECUTE_HANDLER
# define EXCEPTION_EXECUTE_HANDLER 1
#endif

#ifndef CRYPTOPP_IMPORTS

#include "cpu.h"
#include "misc.h"
#include "stdcpp.h"

#ifdef _AIX
# include <sys/systemcfg.h>
#endif

#ifdef __linux__
# include <unistd.h>
#endif

// Capability queries, requires Glibc 2.16, http://lwn.net/Articles/519085/
// CRYPTOPP_GLIBC_VERSION not used because config.h is missing <feature.h>
#if (((__GLIBC__ * 100) + __GLIBC_MINOR__) >= 216)
# define CRYPTOPP_GETAUXV_AVAILABLE 1
#endif

#if CRYPTOPP_GETAUXV_AVAILABLE
# include <sys/auxv.h>
#else
#ifndef AT_HWCAP
# define AT_HWCAP 16
#endif
#ifndef AT_HWCAP2
# define AT_HWCAP2 26
#endif
unsigned long int getauxval(unsigned long int) { return 0; }
#endif

#if defined(__APPLE__)
# include <sys/utsname.h>
#endif

// The cpu-features header and source file are located in
// "$ANDROID_NDK_ROOT/sources/android/cpufeatures".
// setenv-android.sh will copy the header and source file
// into PWD and the makefile will build it in place.
#if defined(__ANDROID__)
# include "cpu-features.h"
#endif

#ifdef CRYPTOPP_GNU_STYLE_INLINE_ASSEMBLY
# include <signal.h>
# include <setjmp.h>
#endif

// Visual Studio 2008 and below are missing _xgetbv and _cpuidex.
// The 32-bit versions use inline ASM below. The 64-bit versions are in x64dll.asm.
#if defined(_MSC_VER) && defined(_M_X64)
extern "C" unsigned long long __fastcall XGETBV64(unsigned int);
extern "C" unsigned long long __fastcall CPUID64(unsigned int, unsigned int, unsigned int*);
#endif

#ifdef CRYPTOPP_GNU_STYLE_INLINE_ASSEMBLY
extern "C" {
    typedef void (*SigHandler)(int);
}

extern "C"
{
	static jmp_buf s_jmpNoCPUID;
	static void SigIllHandler(int)
	{
		longjmp(s_jmpNoCPUID, 1);
	}
}
#endif  // CRYPTOPP_GNU_STYLE_INLINE_ASSEMBLY

ANONYMOUS_NAMESPACE_BEGIN

#if (CRYPTOPP_BOOL_X86 || CRYPTOPP_BOOL_X32 || CRYPTOPP_BOOL_X64)

using CryptoPP::word32;

inline bool IsIntel(const word32 output[4])
{
	// This is the "GenuineIntel" string
	return (output[1] /*EBX*/ == 0x756e6547) &&
		(output[2] /*ECX*/ == 0x6c65746e) &&
		(output[3] /*EDX*/ == 0x49656e69);
}

inline bool IsAMD(const word32 output[4])
{
	// This is the "AuthenticAMD" string.
	return ((output[1] /*EBX*/ == 0x68747541) &&
		(output[2] /*ECX*/ == 0x444D4163) &&
		(output[3] /*EDX*/ == 0x69746E65)) ||
		// Early K5's can return "AMDisbetter!"
		((output[1] /*EBX*/ == 0x69444d41) &&
		(output[2] /*ECX*/ == 0x74656273) &&
		(output[3] /*EDX*/ == 0x21726574));
}

inline bool IsHygon(const word32 output[4])
{
	// This is the "HygonGenuine" string.
	return (output[1] /*EBX*/ == 0x6f677948) &&
		(output[2] /*ECX*/ == 0x656e6975) &&
		(output[3] /*EDX*/ == 0x6e65476e);
}

inline bool IsVIA(const word32 output[4])
{
	// This is the "CentaurHauls" string.
	return ((output[1] /*EBX*/ == 0x746e6543) &&
		(output[2] /*ECX*/ == 0x736c7561) &&
		(output[3] /*EDX*/ == 0x48727561)) ||
		// Some non-PadLock's return "VIA VIA VIA "
		((output[1] /*EBX*/ == 0x32414956) &&
		(output[2] /*ECX*/ == 0x32414956) &&
		(output[3] /*EDX*/ == 0x32414956));
}

#endif  // X86, X32 and X64

#if defined(__APPLE__)

// http://stackoverflow.com/questions/45637888/how-to-determine-armv8-features-at-runtime-on-ios
class AppleMachineInfo
{
public:
	enum { PowerMac=1, Mac, iPhone, iPod, iPad, AppleTV, AppleWatch };
	enum { PowerPC=1, I386, I686, X86_64, ARM32, ARMV8, ARMV84 };

	AppleMachineInfo() : m_device(0), m_version(0), m_arch(0)
	{
		struct utsname systemInfo;
		systemInfo.machine[0] = '\0';
		uname(&systemInfo);

		std::string machine(systemInfo.machine);

		std::string::size_type pos = machine.find_first_of("0123456789");
		if (pos != std::string::npos)
			m_version = std::atoi(machine.substr(pos).c_str());

		if (machine.find("iPhone") != std::string::npos)
		{
			m_device = iPhone;
			if (m_version >= 6) { m_arch = ARMV8; }
			else { m_arch = ARM32; }
		}
		else if (machine.find("iPod") != std::string::npos)
		{
			m_device = iPod;
			if (m_version >= 6) { m_arch = ARMV8; }
			else { m_arch = ARM32; }
		}
		else if (machine.find("iPad") != std::string::npos)
		{
			m_device = iPad;
			if (m_version >= 5) { m_arch = ARMV8; }
			else { m_arch = ARM32; }
		}
		else if (machine.find("PowerMac") != std::string::npos ||
			 machine.find("Power Macintosh") != std::string::npos)
		{
			m_device = PowerMac;
			m_arch = PowerPC;
		}
		else if (machine.find("Mac") != std::string::npos ||
			 machine.find("Macintosh") != std::string::npos)
		{
#if defined(__x86_64) || defined(__amd64)
			m_device = Mac;
			m_arch = X86_64;
#elif defined(__i386)
			m_device = Mac;
			m_arch = I386;
#elif defined(__i686)
			m_device = Mac;
			m_arch = I686;
#else
			// Should never get here
			m_device = Mac;
			m_arch = 0;
#endif
		}
		else if (machine.find("AppleTV") != std::string::npos)
		{
			m_device = AppleTV;
			if (m_version >= 4) { m_arch = ARMV8; }
			else { m_arch = ARM32; }
		}
		else if (machine.find("AppleWatch") != std::string::npos)
		{
			m_device = AppleWatch;
			if (m_version >= 4) { m_arch = ARMV8; }
			else { m_arch = ARM32; }
		}
	}

	unsigned int Device() const {
		return m_device;
	}

	unsigned int Version() const {
		return m_version;
	}

	unsigned int Arch() const {
		return m_arch;
	}

	bool IsARM32() const {
		return m_arch == ARM32;
	}

	bool IsARMv8() const {
		return m_arch == ARMV8;
	}

	bool IsARMv84() const {
		return m_arch == ARMV84;
	}

private:
	unsigned int m_device, m_version, m_arch;
};

void GetAppleMachineInfo(unsigned int& device, unsigned int& version, unsigned int& arch)
{
#if CRYPTOPP_CXX11_STATIC_INIT
	static const AppleMachineInfo info;
#else
	using CryptoPP::Singleton;
	const AppleMachineInfo& info = Singleton<AppleMachineInfo>().Ref();
#endif

	device = info.Device();
	version = info.Version();
	arch = info.Arch();
}

inline bool IsAppleMachineARM32()
{
	static unsigned int arch;
	if (arch == 0)
	{
		unsigned int unused;
		GetAppleMachineInfo(unused, unused, arch);
	}
	return arch == AppleMachineInfo::ARM32;
}

inline bool IsAppleMachineARMv8()
{
	static unsigned int arch;
	if (arch == 0)
	{
		unsigned int unused;
		GetAppleMachineInfo(unused, unused, arch);
	}
	return arch == AppleMachineInfo::ARMV8;
}

#endif  // __APPLE__

ANONYMOUS_NAMESPACE_END

NAMESPACE_BEGIN(CryptoPP)

// *************************** IA-32 CPUs ***************************

#if (CRYPTOPP_BOOL_X86 || CRYPTOPP_BOOL_X32 || CRYPTOPP_BOOL_X64)

bool CRYPTOPP_SECTION_INIT g_x86DetectionDone = false;
bool CRYPTOPP_SECTION_INIT g_hasSSE2 = false;
bool CRYPTOPP_SECTION_INIT g_hasSSSE3 = false;
bool CRYPTOPP_SECTION_INIT g_hasSSE41 = false;
bool CRYPTOPP_SECTION_INIT g_hasSSE42 = false;
bool CRYPTOPP_SECTION_INIT g_hasAESNI = false;
bool CRYPTOPP_SECTION_INIT g_hasCLMUL = false;
bool CRYPTOPP_SECTION_INIT g_hasMOVBE = false;
bool CRYPTOPP_SECTION_INIT g_hasAVX = false;
bool CRYPTOPP_SECTION_INIT g_hasAVX2 = false;
bool CRYPTOPP_SECTION_INIT g_hasADX = false;
bool CRYPTOPP_SECTION_INIT g_hasSHA = false;
bool CRYPTOPP_SECTION_INIT g_hasRDRAND = false;
bool CRYPTOPP_SECTION_INIT g_hasRDSEED = false;
bool CRYPTOPP_SECTION_INIT g_isP4 = false;
bool CRYPTOPP_SECTION_INIT g_hasPadlockRNG = false;
bool CRYPTOPP_SECTION_INIT g_hasPadlockACE = false;
bool CRYPTOPP_SECTION_INIT g_hasPadlockACE2 = false;
bool CRYPTOPP_SECTION_INIT g_hasPadlockPHE = false;
bool CRYPTOPP_SECTION_INIT g_hasPadlockPMM = false;
word32 CRYPTOPP_SECTION_INIT g_cacheLineSize = CRYPTOPP_L1_CACHE_LINE_SIZE;

// For Solaris 11
extern bool CPU_ProbeSSE2();

// xcr0 is available when xgetbv is present.
// The intrinsic is broke on GCC 8.1 and earlier. Also see
// https://gcc.gnu.org/bugzilla/show_bug.cgi?id=85684.
word64 XGetBV(word32 num)
{
// Visual Studio 2010 and above, 32 and 64-bit
#if defined(_MSC_VER) && (_MSC_VER >= 1600)

	return _xgetbv(num);

// Visual Studio 2008 and below, 64-bit
#elif defined(_MSC_VER) && defined(_M_X64)

	return XGETBV64(num);

// Visual Studio 2008 and below, 32-bit
#elif defined(_MSC_VER) && defined(_M_IX86)

	word32 a=0, d=0;
	__asm {
		push eax
		push edx
		push ecx
		mov ecx, num
		_emit 0x0f
		_emit 0x01
		_emit 0xd0
		mov a, eax
		mov d, edx
		pop ecx
		pop edx
		pop eax
	}
	return (static_cast<word64>(d) << 32) | a;

// GCC 4.4 and above
#elif (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 4))

	word32 a=0, d=0;
	__asm__
	(
		"xgetbv" : "=a"(a), "=d"(d) : "c"(num) : "cc"
	);
	return (static_cast<word64>(d) << 32) | a;

// Remainder of GCC and compatibles.
#else

	// https://gcc.gnu.org/bugzilla/show_bug.cgi?id=71659 and
	// http://www.agner.org/optimize/vectorclass/read.php?i=65
	word32 a=0, d=0;
	__asm__
	(
		".byte 0x0f, 0x01, 0xd0"      "\n\t"
		: "=a"(a), "=d"(d) : "c"(num) : "cc"
	);
	return (static_cast<word64>(d) << 32) | a;
#endif
}

// No inline due to Borland/Embarcadero and Issue 498
// cpu.cpp (131): E2211 Inline assembly not allowed in inline and template functions
bool CpuId(word32 func, word32 subfunc, word32 output[4])
{
// Visual Studio 2010 and above, 32 and 64-bit
#if defined(_MSC_VER) && (_MSC_VER >= 1600)

	__cpuidex((int *)output, func, subfunc);
	return true;

// Visual Studio 2008 and below, 64-bit
#elif defined(_MSC_VER) && defined(_M_X64)

	CPUID64(func, subfunc, output);
	return true;

// Visual Studio 2008 and below, 32-bit
#elif (defined(_MSC_VER) && defined(_M_IX86)) || defined(__BORLANDC__)

	__try
	{
		// Borland/Embarcadero and Issue 500
		// Local variables for cpuid output
		word32 a, b, c, d;
		__asm
		{
			push ebx
			mov eax, func
			mov ecx, subfunc
			cpuid
			mov [a], eax
			mov [b], ebx
			mov [c], ecx
			mov [d], edx
			pop ebx
		}
		output[0] = a;
		output[1] = b;
		output[2] = c;
		output[3] = d;
	}
	__except (EXCEPTION_EXECUTE_HANDLER)
	{
		return false;
	}

	return true;

// Linux, Unix, OS X, Solaris, Cygwin, MinGW
#else

	// longjmp and clobber warnings. Volatile is required.
	// http://github.com/weidai11/cryptopp/issues/24 and http://stackoverflow.com/q/7721854
	volatile bool result = true;

	volatile SigHandler oldHandler = signal(SIGILL, SigIllHandler);
	if (oldHandler == SIG_ERR)
		return false;

# ifndef __MINGW32__
	volatile sigset_t oldMask;
	if (sigprocmask(0, NULLPTR, (sigset_t*)&oldMask) != 0)
	{
		signal(SIGILL, oldHandler);
		return false;
	}
# endif

	if (setjmp(s_jmpNoCPUID))
		result = false;
	else
	{
		asm volatile
		(
			// save ebx in case -fPIC is being used
# if CRYPTOPP_BOOL_X32 || CRYPTOPP_BOOL_X64
			"pushq %%rbx; cpuid; mov %%ebx, %%edi; popq %%rbx"
# else
			"push %%ebx; cpuid; mov %%ebx, %%edi; pop %%ebx"
# endif
			: "=a" (output[0]), "=D" (output[1]), "=c" (output[2]), "=d" (output[3])
			: "a" (func), "c" (subfunc)
			: "cc"
		);
	}

# ifndef __MINGW32__
	sigprocmask(SIG_SETMASK, (sigset_t*)&oldMask, NULLPTR);
# endif

	signal(SIGILL, oldHandler);
	return result;
#endif
}

void DetectX86Features()
{
	// Coverity finding CID 171239. Initialize arrays.
	// Indexes: EAX=0, EBX=1, ECX=2, EDX=3
	word32 cpuid0[4]={0}, cpuid1[4]={0}, cpuid2[4]={0};

#if defined(CRYPTOPP_DISABLE_ASM)
	// Not available
	goto done;
#else
	if (!CpuId(0, 0, cpuid0))
		goto done;
	if (!CpuId(1, 0, cpuid1))
		goto done;
#endif

	CRYPTOPP_CONSTANT(EAX_REG = 0);
	CRYPTOPP_CONSTANT(EBX_REG = 1);
	CRYPTOPP_CONSTANT(ECX_REG = 2);
	CRYPTOPP_CONSTANT(EDX_REG = 3);

	CRYPTOPP_CONSTANT(MMX_FLAG   = (1 << 24));   // EDX
	CRYPTOPP_CONSTANT(SSE_FLAG   = (1 << 25));   // EDX
	CRYPTOPP_CONSTANT(SSE2_FLAG  = (1 << 26));   // EDX

	CRYPTOPP_CONSTANT(SSE3_FLAG  = (1 <<  0));   // ECX
	CRYPTOPP_CONSTANT(SSSE3_FLAG = (1 <<  9));   // ECX
	CRYPTOPP_CONSTANT(SSE41_FLAG = (1 << 19));   // ECX
	CRYPTOPP_CONSTANT(SSE42_FLAG = (1 << 20));   // ECX
	CRYPTOPP_CONSTANT(MOVBE_FLAG = (1 << 22));   // ECX
	CRYPTOPP_CONSTANT(AESNI_FLAG = (1 << 25));   // ECX
	CRYPTOPP_CONSTANT(CLMUL_FLAG = (1 <<  1));   // ECX

	CRYPTOPP_CONSTANT(XSAVE_FLAG   = (1 << 26)); // ECX
	CRYPTOPP_CONSTANT(OSXSAVE_FLAG = (1 << 27)); // ECX

	CRYPTOPP_CONSTANT(AVX_FLAG = (3 << 27));     // ECX
	CRYPTOPP_CONSTANT(YMM_FLAG = (3 <<  1));     // CR0

    // x86_64 machines don't check some flags because SSE2
    // is part of the core instruction set architecture
    CRYPTOPP_UNUSED(MMX_FLAG); CRYPTOPP_UNUSED(SSE_FLAG);
    CRYPTOPP_UNUSED(SSE3_FLAG); CRYPTOPP_UNUSED(XSAVE_FLAG);

#if (CRYPTOPP_BOOL_X32 || CRYPTOPP_BOOL_X64)
	// 64-bit core instruction set includes SSE2. Just check
	// the OS enabled SSE2 support using OSXSAVE.
	g_hasSSE2 = (cpuid1[ECX_REG] & OSXSAVE_FLAG) != 0;
#else
	// Check the processor supports SSE2. Then use OSXSAVE to
	// signal OS support for SSE2 to avoid probes.
	// Also see http://stackoverflow.com/a/22521619/608639
	// and http://github.com/weidai11/cryptopp/issues/511.
	if ((cpuid1[EDX_REG] & SSE2_FLAG) == SSE2_FLAG)
		g_hasSSE2 = (cpuid1[ECX_REG] & XSAVE_FLAG) != 0 &&
		            (cpuid1[ECX_REG] & OSXSAVE_FLAG) != 0;
#endif

	// Solaris 11 i86pc does not signal SSE support using
	// OSXSAVE. We need to probe for SSE support.
	if (g_hasSSE2 == false)
		g_hasSSE2 = CPU_ProbeSSE2();

	if (g_hasSSE2 == false)
		goto done;

	g_hasSSSE3 = (cpuid1[ECX_REG] & SSSE3_FLAG) != 0;
	g_hasSSE41 = (cpuid1[ECX_REG] & SSE41_FLAG) != 0;
	g_hasSSE42 = (cpuid1[ECX_REG] & SSE42_FLAG) != 0;
	g_hasMOVBE = (cpuid1[ECX_REG] & MOVBE_FLAG) != 0;
	g_hasAESNI = (cpuid1[ECX_REG] & AESNI_FLAG) != 0;
	g_hasCLMUL = (cpuid1[ECX_REG] & CLMUL_FLAG) != 0;

	// AVX is similar to SSE. Check if AVX is available on the cpu, then
	// check if the OS enabled XSAVE/XRESTORE for the extended registers.
	// https://software.intel.com/en-us/blogs/2011/04/14/is-avx-enabled
	if ((cpuid1[ECX_REG] & AVX_FLAG) == AVX_FLAG)
	{
		word64 xcr0 = XGetBV(0);
		g_hasAVX = (xcr0 & YMM_FLAG) == YMM_FLAG;
	}

	if (IsIntel(cpuid0))
	{
		CRYPTOPP_CONSTANT(RDRAND_FLAG = (1 << 30));
		CRYPTOPP_CONSTANT(RDSEED_FLAG = (1 << 18));
		CRYPTOPP_CONSTANT(   ADX_FLAG = (1 << 19));
		CRYPTOPP_CONSTANT(   SHA_FLAG = (1 << 29));
		CRYPTOPP_CONSTANT(  AVX2_FLAG = (1 <<  5));

		g_isP4 = ((cpuid1[0] >> 8) & 0xf) == 0xf;
		g_cacheLineSize = 8 * GETBYTE(cpuid1[1], 1);
		g_hasRDRAND = (cpuid1[ECX_REG] & RDRAND_FLAG) != 0;

		if (cpuid0[EAX_REG] >= 7)
		{
			if (CpuId(7, 0, cpuid2))
			{
				g_hasRDSEED = (cpuid2[EBX_REG] & RDSEED_FLAG) != 0;
				g_hasADX    = (cpuid2[EBX_REG] & ADX_FLAG) != 0;
				g_hasSHA    = (cpuid2[EBX_REG] & SHA_FLAG) != 0;
				g_hasAVX2   = (cpuid2[EBX_REG] & AVX2_FLAG) != 0;
			}
		}
	}
	else if (IsAMD(cpuid0) || IsHygon(cpuid0))
	{
		CRYPTOPP_CONSTANT(RDRAND_FLAG = (1 << 30));
		CRYPTOPP_CONSTANT(RDSEED_FLAG = (1 << 18));
		CRYPTOPP_CONSTANT(   ADX_FLAG = (1 << 19));
		CRYPTOPP_CONSTANT(   SHA_FLAG = (1 << 29));
		CRYPTOPP_CONSTANT(  AVX2_FLAG = (1 <<  5));

		CpuId(0x80000005, 0, cpuid2);
		g_cacheLineSize = GETBYTE(cpuid2[ECX_REG], 0);
		g_hasRDRAND = (cpuid1[ECX_REG] & RDRAND_FLAG) != 0;

		if (cpuid0[EAX_REG] >= 7)
		{
			if (CpuId(7, 0, cpuid2))
			{
				g_hasRDSEED = (cpuid2[EBX_REG] & RDSEED_FLAG) != 0;
				g_hasADX    = (cpuid2[EBX_REG] & ADX_FLAG) != 0;
				g_hasSHA    = (cpuid2[EBX_REG] & SHA_FLAG) != 0;
				g_hasAVX2   = (cpuid2[EBX_REG] & AVX2_FLAG) != 0;
			}
		}

		// Unconditionally disable RDRAND and RDSEED on AMD cpu's with family 15h or 16h.
		// See Crypto++ Issue 924, https://github.com/weidai11/cryptopp/issues/924,
		// Clear RDRAND CPUID bit on AMD family 15h/16h, https://lore.kernel.org/patchwork/patch/1115413/,
		// and AMD CPUID Specification, https://www.amd.com/system/files/TechDocs/25481.pdf
		{
			CRYPTOPP_CONSTANT(FAMILY_BASE_FLAG = (0x0f << 8));
			CRYPTOPP_CONSTANT(FAMILY_EXT_FLAG = (0xff << 20));

			word32 family = (cpuid1[0] & FAMILY_BASE_FLAG) >> 8;
			if (family == 0xf)
				family += (cpuid1[0] & FAMILY_EXT_FLAG) >> 20;
			if (family == 0x15 || family == 0x16)
			{
				g_hasRDRAND = false;
				g_hasRDSEED = false;
			}
		}
	}
	else if (IsVIA(cpuid0))
	{
		// Two bits: available and enabled
		CRYPTOPP_CONSTANT( RNG_FLAGS = (0x3 << 2));
		CRYPTOPP_CONSTANT( ACE_FLAGS = (0x3 << 6));
		CRYPTOPP_CONSTANT(ACE2_FLAGS = (0x3 << 8));
		CRYPTOPP_CONSTANT( PHE_FLAGS = (0x3 << 10));
		CRYPTOPP_CONSTANT( PMM_FLAGS = (0x3 << 12));

		CpuId(0xC0000000, 0, cpuid2);
		word32 extendedFeatures = cpuid2[0];

		if (extendedFeatures >= 0xC0000001)
		{
			CpuId(0xC0000001, 0, cpuid2);
			g_hasPadlockRNG  = (cpuid2[EDX_REG] & RNG_FLAGS) != 0;
			g_hasPadlockACE  = (cpuid2[EDX_REG] & ACE_FLAGS) != 0;
			g_hasPadlockACE2 = (cpuid2[EDX_REG] & ACE2_FLAGS) != 0;
			g_hasPadlockPHE  = (cpuid2[EDX_REG] & PHE_FLAGS) != 0;
			g_hasPadlockPMM  = (cpuid2[EDX_REG] & PMM_FLAGS) != 0;
		}

		if (extendedFeatures >= 0xC0000005)
		{
			CpuId(0xC0000005, 0, cpuid2);
			g_cacheLineSize = GETBYTE(cpuid2[ECX_REG], 0);
		}
	}

	// Keep AVX2 in sync with OS support for AVX. AVX tests both
	// cpu support and OS support, while AVX2 only tests cpu support.
	g_hasAVX2 &= g_hasAVX;

done:

#if defined(_SC_LEVEL1_DCACHE_LINESIZE)
	// Glibc does not implement on some platforms. The runtime returns 0 instead of error.
	// https://sourceware.org/git/?p=glibc.git;a=blob;f=sysdeps/posix/sysconf.c
	int cacheLineSize = (int)sysconf(_SC_LEVEL1_DCACHE_LINESIZE);
	if (g_cacheLineSize == 0 && cacheLineSize > 0)
		g_cacheLineSize = cacheLineSize;
#endif

	if (g_cacheLineSize == 0)
		g_cacheLineSize = CRYPTOPP_L1_CACHE_LINE_SIZE;

	*const_cast<volatile bool*>(&g_x86DetectionDone) = true;
}

// *************************** ARM-32, Aarch32 and Aarch64 ***************************

#elif (CRYPTOPP_BOOL_ARM32 || CRYPTOPP_BOOL_ARMV8)

bool CRYPTOPP_SECTION_INIT g_ArmDetectionDone = false;
bool CRYPTOPP_SECTION_INIT g_hasARMv7 = false;
bool CRYPTOPP_SECTION_INIT g_hasNEON = false;
bool CRYPTOPP_SECTION_INIT g_hasPMULL = false;
bool CRYPTOPP_SECTION_INIT g_hasCRC32 = false;
bool CRYPTOPP_SECTION_INIT g_hasAES = false;
bool CRYPTOPP_SECTION_INIT g_hasSHA1 = false;
bool CRYPTOPP_SECTION_INIT g_hasSHA2 = false;
bool CRYPTOPP_SECTION_INIT g_hasSHA512 = false;
bool CRYPTOPP_SECTION_INIT g_hasSHA3 = false;
bool CRYPTOPP_SECTION_INIT g_hasSM3 = false;
bool CRYPTOPP_SECTION_INIT g_hasSM4 = false;
word32 CRYPTOPP_SECTION_INIT g_cacheLineSize = CRYPTOPP_L1_CACHE_LINE_SIZE;

// ARM does not have an unprivliged equivalent to CPUID on IA-32. We have to
// jump through some hoops to detect features on a wide array of platforms.
// Our strategy is two part. First, attempt to *Query* the OS for a feature,
// like using getauxval on Linux. If that fails, then *Probe* the cpu
// executing an instruction and an observe a SIGILL if unsupported. The probes
// are in source files where compilation options like -march=armv8-a+crc make
// intrinsics available. They are expensive when compared to a standard OS
// feature query. Always perform the feature query first. For Linux see
// http://sourceware.org/ml/libc-help/2017-08/msg00012.html
// Avoid probes on Apple platforms because Apple's signal handling for SIGILLs
// appears broken. We are trying to figure out a way to feature test without
// probes. Also see http://stackoverflow.com/a/11197770/608639 and
// http://gist.github.com/erkanyildiz/390a480f27e86f8cd6ba.

extern bool CPU_ProbeARMv7();
extern bool CPU_ProbeNEON();
extern bool CPU_ProbeCRC32();
extern bool CPU_ProbeAES();
extern bool CPU_ProbeSHA1();
extern bool CPU_ProbeSHA256();
extern bool CPU_ProbeSHA512();
extern bool CPU_ProbeSHA3();
extern bool CPU_ProbeSM3();
extern bool CPU_ProbeSM4();
extern bool CPU_ProbePMULL();

// https://github.com/torvalds/linux/blob/master/arch/arm/include/uapi/asm/hwcap.h
// https://github.com/torvalds/linux/blob/master/arch/arm64/include/uapi/asm/hwcap.h
#ifndef HWCAP_ARMv7
# define HWCAP_ARMv7 (1 << 29)
#endif
#ifndef HWCAP_ASIMD
# define HWCAP_ASIMD (1 << 1)
#endif
#ifndef HWCAP_NEON
# define HWCAP_NEON (1 << 12)
#endif
#ifndef HWCAP_CRC32
# define HWCAP_CRC32 (1 << 7)
#endif
#ifndef HWCAP2_CRC32
# define HWCAP2_CRC32 (1 << 4)
#endif
#ifndef HWCAP_PMULL
# define HWCAP_PMULL (1 << 4)
#endif
#ifndef HWCAP2_PMULL
# define HWCAP2_PMULL (1 << 1)
#endif
#ifndef HWCAP_AES
# define HWCAP_AES (1 << 3)
#endif
#ifndef HWCAP2_AES
# define HWCAP2_AES (1 << 0)
#endif
#ifndef HWCAP_SHA1
# define HWCAP_SHA1 (1 << 5)
#endif
#ifndef HWCAP_SHA2
# define HWCAP_SHA2 (1 << 6)
#endif
#ifndef HWCAP2_SHA1
# define HWCAP2_SHA1 (1 << 2)
#endif
#ifndef HWCAP2_SHA2
# define HWCAP2_SHA2 (1 << 3)
#endif
#ifndef HWCAP_SHA3
# define HWCAP_SHA3 (1 << 17)
#endif
#ifndef HWCAP_SM3
# define HWCAP_SM3 (1 << 18)
#endif
#ifndef HWCAP_SM4
# define HWCAP_SM4 (1 << 19)
#endif
#ifndef HWCAP_SHA512
# define HWCAP_SHA512 (1 << 21)
#endif

inline bool CPU_QueryARMv7()
{
#if defined(__ANDROID__) && defined(__arm__)
	if (((android_getCpuFamily() & ANDROID_CPU_FAMILY_ARM) != 0) &&
		((android_getCpuFeatures() & ANDROID_CPU_ARM_FEATURE_ARMv7) != 0))
		return true;
#elif defined(__linux__) && defined(__arm__)
	if ((getauxval(AT_HWCAP) & HWCAP_ARMv7) != 0 ||
	    (getauxval(AT_HWCAP) & HWCAP_NEON) != 0)
		return true;
#elif defined(__APPLE__) && defined(__arm__)
	// Apple hardware is ARMv7 or above.
	return true;
#endif
	return false;
}

inline bool CPU_QueryNEON()
{
#if defined(__ANDROID__) && defined(__aarch64__)
	if (((android_getCpuFamily() & ANDROID_CPU_FAMILY_ARM64) != 0) &&
		((android_getCpuFeatures() & ANDROID_CPU_ARM64_FEATURE_ASIMD) != 0))
		return true;
#elif defined(__ANDROID__) && defined(__arm__)
	if (((android_getCpuFamily() & ANDROID_CPU_FAMILY_ARM) != 0) &&
		((android_getCpuFeatures() & ANDROID_CPU_ARM_FEATURE_NEON) != 0))
		return true;
#elif defined(__linux__) && defined(__aarch64__)
	if ((getauxval(AT_HWCAP) & HWCAP_ASIMD) != 0)
		return true;
#elif defined(__linux__) && defined(__aarch32__)
	if ((getauxval(AT_HWCAP2) & HWCAP2_ASIMD) != 0)
		return true;
#elif defined(__linux__) && defined(__arm__)
	if ((getauxval(AT_HWCAP) & HWCAP_NEON) != 0)
		return true;
#elif defined(__APPLE__) && defined(__aarch64__)
	// Core feature set for Aarch32 and Aarch64.
	return true;
#endif
	return false;
}

inline bool CPU_QueryCRC32()
{
#if defined(__ANDROID__) && defined(__aarch64__)
	if (((android_getCpuFamily() & ANDROID_CPU_FAMILY_ARM64) != 0) &&
		((android_getCpuFeatures() & ANDROID_CPU_ARM64_FEATURE_CRC32) != 0))
		return true;
#elif defined(__ANDROID__) && defined(__aarch32__)
	if (((android_getCpuFamily() & ANDROID_CPU_FAMILY_ARM) != 0) &&
		((android_getCpuFeatures() & ANDROID_CPU_ARM_FEATURE_CRC32) != 0))
		return true;
#elif defined(__linux__) && defined(__aarch64__)
	if ((getauxval(AT_HWCAP) & HWCAP_CRC32) != 0)
		return true;
#elif defined(__linux__) && defined(__aarch32__)
	if ((getauxval(AT_HWCAP2) & HWCAP2_CRC32) != 0)
		return true;
#elif defined(__APPLE__) && defined(__aarch64__)
	// No compiler support. CRC intrinsics result in a failed compiled.
	return false;
#endif
	return false;
}

inline bool CPU_QueryPMULL()
{
#if defined(__ANDROID__) && defined(__aarch64__)
	if (((android_getCpuFamily() & ANDROID_CPU_FAMILY_ARM64) != 0) &&
		((android_getCpuFeatures() & ANDROID_CPU_ARM64_FEATURE_PMULL) != 0))
		return true;
#elif defined(__ANDROID__) && defined(__aarch32__)
	if (((android_getCpuFamily() & ANDROID_CPU_FAMILY_ARM) != 0) &&
		((android_getCpuFeatures() & ANDROID_CPU_ARM_FEATURE_PMULL) != 0))
		return true;
#elif defined(__linux__) && defined(__aarch64__)
	if ((getauxval(AT_HWCAP) & HWCAP_PMULL) != 0)
		return true;
#elif defined(__linux__) && defined(__aarch32__)
	if ((getauxval(AT_HWCAP2) & HWCAP2_PMULL) != 0)
		return true;
#elif defined(__APPLE__) && defined(__aarch64__)
	// No compiler support. PMULL intrinsics result in a failed compiled.
	return false;
#endif
	return false;
}

inline bool CPU_QueryAES()
{
#if defined(__ANDROID__) && defined(__aarch64__)
	if (((android_getCpuFamily() & ANDROID_CPU_FAMILY_ARM64) != 0) &&
		((android_getCpuFeatures() & ANDROID_CPU_ARM64_FEATURE_AES) != 0))
		return true;
#elif defined(__ANDROID__) && defined(__aarch32__)
	if (((android_getCpuFamily() & ANDROID_CPU_FAMILY_ARM) != 0) &&
		((android_getCpuFeatures() & ANDROID_CPU_ARM_FEATURE_AES) != 0))
		return true;
#elif defined(__linux__) && defined(__aarch64__)
	if ((getauxval(AT_HWCAP) & HWCAP_AES) != 0)
		return true;
#elif defined(__linux__) && defined(__aarch32__)
	if ((getauxval(AT_HWCAP2) & HWCAP2_AES) != 0)
		return true;
#elif defined(__APPLE__) && defined(__aarch64__)
	return IsAppleMachineARMv8();
#endif
	return false;
}

inline bool CPU_QuerySHA1()
{
#if defined(__ANDROID__) && defined(__aarch64__)
	if (((android_getCpuFamily() & ANDROID_CPU_FAMILY_ARM64) != 0) &&
		((android_getCpuFeatures() & ANDROID_CPU_ARM64_FEATURE_SHA1) != 0))
		return true;
#elif defined(__ANDROID__) && defined(__aarch32__)
	if (((android_getCpuFamily() & ANDROID_CPU_FAMILY_ARM) != 0) &&
		((android_getCpuFeatures() & ANDROID_CPU_ARM_FEATURE_SHA1) != 0))
		return true;
#elif defined(__linux__) && defined(__aarch64__)
	if ((getauxval(AT_HWCAP) & HWCAP_SHA1) != 0)
		return true;
#elif defined(__linux__) && defined(__aarch32__)
	if ((getauxval(AT_HWCAP2) & HWCAP2_SHA1) != 0)
		return true;
#elif defined(__APPLE__) && defined(__aarch64__)
	return IsAppleMachineARMv8();
#endif
	return false;
}

inline bool CPU_QuerySHA256()
{
#if defined(__ANDROID__) && defined(__aarch64__)
	if (((android_getCpuFamily() & ANDROID_CPU_FAMILY_ARM64) != 0) &&
		((android_getCpuFeatures() & ANDROID_CPU_ARM64_FEATURE_SHA2) != 0))
		return true;
#elif defined(__ANDROID__) && defined(__aarch32__)
	if (((android_getCpuFamily() & ANDROID_CPU_FAMILY_ARM) != 0) &&
		((android_getCpuFeatures() & ANDROID_CPU_ARM_FEATURE_SHA2) != 0))
		return true;
#elif defined(__linux__) && defined(__aarch64__)
	if ((getauxval(AT_HWCAP) & HWCAP_SHA2) != 0)
		return true;
#elif defined(__linux__) && defined(__aarch32__)
	if ((getauxval(AT_HWCAP2) & HWCAP2_SHA2) != 0)
		return true;
#elif defined(__APPLE__) && defined(__aarch64__)
	return IsAppleMachineARMv8();
#endif
	return false;
}

inline bool CPU_QuerySHA512()
{
// Some ARMv8.4 features are disabled at the moment
#if defined(__ANDROID__) && defined(__aarch64__) && 0
	if (((android_getCpuFamily() & ANDROID_CPU_FAMILY_ARM64) != 0) &&
		((android_getCpuFeatures() & ANDROID_CPU_ARM64_FEATURE_SHA512) != 0))
		return true;
#elif defined(__ANDROID__) && defined(__aarch32__) && 0
	if (((android_getCpuFamily() & ANDROID_CPU_FAMILY_ARM) != 0) &&
		((android_getCpuFeatures() & ANDROID_CPU_ARM_FEATURE_SHA512) != 0))
		return true;
#elif defined(__linux__) && defined(__aarch64__)
	if ((getauxval(AT_HWCAP) & HWCAP_SHA512) != 0)
		return true;
#elif defined(__linux__) && defined(__aarch32__)
	if ((getauxval(AT_HWCAP2) & HWCAP2_SHA512) != 0)
		return true;
#elif defined(__APPLE__) && defined(__aarch64__) && 0
	return false;
#endif
	return false;
}

inline bool CPU_QuerySHA3()
{
// Some ARMv8.4 features are disabled at the moment
#if defined(__ANDROID__) && defined(__aarch64__) && 0
	if (((android_getCpuFamily() & ANDROID_CPU_FAMILY_ARM64) != 0) &&
		((android_getCpuFeatures() & ANDROID_CPU_ARM64_FEATURE_SHA3) != 0))
		return true;
#elif defined(__ANDROID__) && defined(__aarch32__) && 0
	if (((android_getCpuFamily() & ANDROID_CPU_FAMILY_ARM) != 0) &&
		((android_getCpuFeatures() & ANDROID_CPU_ARM_FEATURE_SHA3) != 0))
		return true;
#elif defined(__linux__) && defined(__aarch64__)
	if ((getauxval(AT_HWCAP) & HWCAP_SHA3) != 0)
		return true;
#elif defined(__linux__) && defined(__aarch32__)
	if ((getauxval(AT_HWCAP2) & HWCAP2_SHA3) != 0)
		return true;
#elif defined(__APPLE__) && defined(__aarch64__) && 0
	return false;
#endif
	return false;
}

inline bool CPU_QuerySM3()
{
// Some ARMv8.4 features are disabled at the moment
#if defined(__ANDROID__) && defined(__aarch64__) && 0
	if (((android_getCpuFamily() & ANDROID_CPU_FAMILY_ARM64) != 0) &&
		((android_getCpuFeatures() & ANDROID_CPU_ARM64_FEATURE_SM3) != 0))
		return true;
#elif defined(__ANDROID__) && defined(__aarch32__) && 0
	if (((android_getCpuFamily() & ANDROID_CPU_FAMILY_ARM) != 0) &&
		((android_getCpuFeatures() & ANDROID_CPU_ARM_FEATURE_SM3) != 0))
		return true;
#elif defined(__linux__) && defined(__aarch64__)
	if ((getauxval(AT_HWCAP) & HWCAP_SM3) != 0)
		return true;
#elif defined(__linux__) && defined(__aarch32__)
	if ((getauxval(AT_HWCAP2) & HWCAP2_SM3) != 0)
		return true;
#elif defined(__APPLE__) && defined(__aarch64__) && 0
	return false;
#endif
	return false;
}

inline bool CPU_QuerySM4()
{
// Some ARMv8.4 features are disabled at the moment
#if defined(__ANDROID__) && defined(__aarch64__) && 0
	if (((android_getCpuFamily() & ANDROID_CPU_FAMILY_ARM64) != 0) &&
		((android_getCpuFeatures() & ANDROID_CPU_ARM64_FEATURE_SM4) != 0))
		return true;
#elif defined(__ANDROID__) && defined(__aarch32__) && 0
	if (((android_getCpuFamily() & ANDROID_CPU_FAMILY_ARM) != 0) &&
		((android_getCpuFeatures() & ANDROID_CPU_ARM_FEATURE_SM4) != 0))
		return true;
#elif defined(__linux__) && defined(__aarch64__)
	if ((getauxval(AT_HWCAP) & HWCAP_SM4) != 0)
		return true;
#elif defined(__linux__) && defined(__aarch32__)
	if ((getauxval(AT_HWCAP2) & HWCAP2_SM4) != 0)
		return true;
#elif defined(__APPLE__) && defined(__aarch64__) && 0
	return false;
#endif
	return false;
}

void DetectArmFeatures()
{
	// The CPU_ProbeXXX's return false for OSes which
	// can't tolerate SIGILL-based probes
	g_hasARMv7 = CPU_QueryARMv7() || CPU_ProbeARMv7();
	g_hasNEON = CPU_QueryNEON() || CPU_ProbeNEON();
	g_hasCRC32 = CPU_QueryCRC32() || CPU_ProbeCRC32();
	g_hasPMULL = CPU_QueryPMULL() || CPU_ProbePMULL();
	g_hasAES  = CPU_QueryAES() || CPU_ProbeAES();
	g_hasSHA1 = CPU_QuerySHA1() || CPU_ProbeSHA1();
	g_hasSHA2 = CPU_QuerySHA256() || CPU_ProbeSHA256();
	g_hasSHA512 = CPU_QuerySHA512(); // || CPU_ProbeSHA512();
	g_hasSHA3 = CPU_QuerySHA3(); // || CPU_ProbeSHA3();
	g_hasSM3 = CPU_QuerySM3(); // || CPU_ProbeSM3();
	g_hasSM4 = CPU_QuerySM4(); // || CPU_ProbeSM4();

#if defined(_SC_LEVEL1_DCACHE_LINESIZE)
	// Glibc does not implement on some platforms. The runtime returns 0 instead of error.
	// https://sourceware.org/git/?p=glibc.git;a=blob;f=sysdeps/posix/sysconf.c
	int cacheLineSize = (int)sysconf(_SC_LEVEL1_DCACHE_LINESIZE);
	if (cacheLineSize > 0)
		g_cacheLineSize = cacheLineSize;
#endif

	if (g_cacheLineSize == 0)
		g_cacheLineSize = CRYPTOPP_L1_CACHE_LINE_SIZE;

	*const_cast<volatile bool*>(&g_ArmDetectionDone) = true;
}

// *************************** PowerPC and PowerPC64 ***************************

#elif (CRYPTOPP_BOOL_PPC32 || CRYPTOPP_BOOL_PPC64)

bool CRYPTOPP_SECTION_INIT g_PowerpcDetectionDone = false;
bool CRYPTOPP_SECTION_INIT g_hasAltivec = false;
bool CRYPTOPP_SECTION_INIT g_hasPower7 = false;
bool CRYPTOPP_SECTION_INIT g_hasPower8 = false;
bool CRYPTOPP_SECTION_INIT g_hasPower9 = false;
bool CRYPTOPP_SECTION_INIT g_hasAES = false;
bool CRYPTOPP_SECTION_INIT g_hasPMULL = false;
bool CRYPTOPP_SECTION_INIT g_hasSHA256 = false;
bool CRYPTOPP_SECTION_INIT g_hasSHA512 = false;
bool CRYPTOPP_SECTION_INIT g_hasDARN = false;
word32 CRYPTOPP_SECTION_INIT g_cacheLineSize = CRYPTOPP_L1_CACHE_LINE_SIZE;

extern bool CPU_ProbeAltivec();
extern bool CPU_ProbePower7();
extern bool CPU_ProbePower8();
extern bool CPU_ProbePower9();
extern bool CPU_ProbeAES();
extern bool CPU_ProbePMULL();
extern bool CPU_ProbeSHA256();
extern bool CPU_ProbeSHA512();
extern bool CPU_ProbeDARN();

// AIX defines. We used to just call __power_7_andup()
// and friends but at Power9, too many compilers were
// missing __power_9_andup(). Instead we switched to
// a pattern similar to OpenSSL caps testing.
#ifndef __power_6_andup
# define __power_6_andup() __power_set(0xffffffffU<<14)
#endif
#ifndef __power_7_andup
# define __power_7_andup() __power_set(0xffffffffU<<15)
#endif
#ifndef __power_8_andup
# define __power_8_andup() __power_set(0xffffffffU<<16)
#endif
#ifndef __power_9_andup
# define __power_9_andup() __power_set(0xffffffffU<<17)
#endif

// AIX first supported Altivec at Power6, though it
// was available much earlier for other vendors.
inline bool CPU_QueryAltivec()
{
#if defined(__linux__) && defined(PPC_FEATURE_HAS_ALTIVEC)
	if ((getauxval(AT_HWCAP) & PPC_FEATURE_HAS_ALTIVEC) != 0)
		return true;
#elif defined(_AIX)
	if (__power_6_andup() != 0)
		return true;
#elif defined(__APPLE__) && defined(__POWERPC__)
	unsigned int unused, arch;
	GetAppleMachineInfo(unused, unused, arch);
	return arch == AppleMachineInfo::PowerMac;
#endif
	return false;
}

inline bool CPU_QueryPower7()
{
	// Power7 and ISA 2.06
#if defined(__linux__) && defined(PPC_FEATURE_ARCH_2_06)
	if ((getauxval(AT_HWCAP) & PPC_FEATURE_ARCH_2_06) != 0)
		return true;
#elif defined(_AIX)
	if (__power_7_andup() != 0)
		return true;
#endif
	return false;
}

inline bool CPU_QueryPower8()
{
	// Power8 and ISA 2.07 provide in-core crypto.
#if defined(__linux__) && defined(PPC_FEATURE2_ARCH_2_07)
	if ((getauxval(AT_HWCAP2) & PPC_FEATURE2_ARCH_2_07) != 0)
		return true;
#elif defined(_AIX)
	if (__power_8_andup() != 0)
		return true;
#endif
	return false;
}

inline bool CPU_QueryPower9()
{
	// Power9 and ISA 3.0.
#if defined(__linux__) && defined(PPC_FEATURE2_ARCH_3_00)
	if ((getauxval(AT_HWCAP2) & PPC_FEATURE2_ARCH_3_00) != 0)
		return true;
#elif defined(_AIX)
	if (__power_9_andup() != 0)
		return true;
#endif
	return false;
}

inline bool CPU_QueryAES()
{
	// Power8 and ISA 2.07 provide in-core crypto. Glibc
	// 2.24 or higher is required for PPC_FEATURE2_VEC_CRYPTO.
#if defined(__linux__) && defined(PPC_FEATURE2_VEC_CRYPTO)
	if ((getauxval(AT_HWCAP2) & PPC_FEATURE2_VEC_CRYPTO) != 0)
		return true;
#elif defined(_AIX)
	if (__power_8_andup() != 0)
		return true;
#endif
	return false;
}

inline bool CPU_QueryPMULL()
{
	// Power8 and ISA 2.07 provide in-core crypto. Glibc
	// 2.24 or higher is required for PPC_FEATURE2_VEC_CRYPTO.
#if defined(__linux__) && defined(PPC_FEATURE2_VEC_CRYPTO)
	if ((getauxval(AT_HWCAP2) & PPC_FEATURE2_VEC_CRYPTO) != 0)
		return true;
#elif defined(_AIX)
	if (__power_8_andup() != 0)
		return true;
#endif
	return false;
}

inline bool CPU_QuerySHA256()
{
	// Power8 and ISA 2.07 provide in-core crypto. Glibc
	// 2.24 or higher is required for PPC_FEATURE2_VEC_CRYPTO.
#if defined(__linux__) && defined(PPC_FEATURE2_VEC_CRYPTO)
	if ((getauxval(AT_HWCAP2) & PPC_FEATURE2_VEC_CRYPTO) != 0)
		return true;
#elif defined(_AIX)
	if (__power_8_andup() != 0)
		return true;
#endif
	return false;
}
inline bool CPU_QuerySHA512()
{
	// Power8 and ISA 2.07 provide in-core crypto. Glibc
	// 2.24 or higher is required for PPC_FEATURE2_VEC_CRYPTO.
#if defined(__linux__) && defined(PPC_FEATURE2_VEC_CRYPTO)
	if ((getauxval(AT_HWCAP2) & PPC_FEATURE2_VEC_CRYPTO) != 0)
		return true;
#elif defined(_AIX)
	if (__power_8_andup() != 0)
		return true;
#endif
	return false;
}

// Power9 random number generator
inline bool CPU_QueryDARN()
{
	// Power9 and ISA 3.0 provide DARN.
#if defined(__linux__) && defined(PPC_FEATURE2_ARCH_3_00)
	if ((getauxval(AT_HWCAP2) & PPC_FEATURE2_ARCH_3_00) != 0)
		return true;
#elif defined(_AIX)
	if (__power_9_andup() != 0)
		return true;
#endif
	return false;
}

void DetectPowerpcFeatures()
{
	// The CPU_ProbeXXX's return false for OSes which
	// can't tolerate SIGILL-based probes, like Apple
	g_hasAltivec  = CPU_QueryAltivec() || CPU_ProbeAltivec();
	g_hasPower7 = CPU_QueryPower7() || CPU_ProbePower7();
	g_hasPower8 = CPU_QueryPower8() || CPU_ProbePower8();
	g_hasPower9 = CPU_QueryPower9() || CPU_ProbePower9();
	g_hasPMULL = CPU_QueryPMULL() || CPU_ProbePMULL();
	g_hasAES  = CPU_QueryAES() || CPU_ProbeAES();
	g_hasSHA256 = CPU_QuerySHA256() || CPU_ProbeSHA256();
	g_hasSHA512 = CPU_QuerySHA512() || CPU_ProbeSHA512();
	g_hasDARN = CPU_QueryDARN() || CPU_ProbeDARN();

#if defined(_AIX) && defined(SC_L1C_DLS)
	// /usr/include/sys/systemcfg.h
	int cacheLineSize = getsystemcfg(SC_L1C_DLS);
	if (cacheLineSize > 0)
		g_cacheLineSize = cacheLineSize;
#elif defined(_SC_LEVEL1_DCACHE_LINESIZE)
	// Glibc does not implement on some platforms. The runtime returns 0 instead of error.
	// https://sourceware.org/git/?p=glibc.git;a=blob;f=sysdeps/posix/sysconf.c
	int cacheLineSize = (int)sysconf(_SC_LEVEL1_DCACHE_LINESIZE);
	if (cacheLineSize > 0)
		g_cacheLineSize = cacheLineSize;
#endif

	if (g_cacheLineSize == 0)
		g_cacheLineSize = CRYPTOPP_L1_CACHE_LINE_SIZE;

	*const_cast<volatile bool*>(&g_PowerpcDetectionDone) = true;
}

#endif
NAMESPACE_END

// *************************** C++ Static Initialization ***************************

ANONYMOUS_NAMESPACE_BEGIN

class InitCpu
{
public:
	InitCpu()
	{
#if CRYPTOPP_BOOL_X86 || CRYPTOPP_BOOL_X32 || CRYPTOPP_BOOL_X64
		CryptoPP::DetectX86Features();
#elif CRYPTOPP_BOOL_ARM32 || CRYPTOPP_BOOL_ARMV8
		CryptoPP::DetectArmFeatures();
#elif CRYPTOPP_BOOL_PPC32 || CRYPTOPP_BOOL_PPC64
		CryptoPP::DetectPowerpcFeatures();
#endif
	}
};

// This is not really needed because HasSSE() and friends can dynamically initialize.
// Everything depends on CPU features so we initialize it once at load time.
// Dynamic initialization will be used if init priorities are not available.

#if HAVE_GCC_INIT_PRIORITY
	const InitCpu s_init __attribute__ ((init_priority (CRYPTOPP_INIT_PRIORITY + 10))) = InitCpu();
#elif HAVE_MSC_INIT_PRIORITY
	#pragma warning(disable: 4075)
	#pragma init_seg(".CRT$XCU")
	const InitCpu s_init;
	#pragma warning(default: 4075)
#elif HAVE_XLC_INIT_PRIORITY
	// XLC needs constant, not a define
	#pragma priority(270)
	const InitCpu s_init;
#else
	const InitCpu s_init;
#endif

ANONYMOUS_NAMESPACE_END

#endif  // CRYPTOPP_IMPORTS
