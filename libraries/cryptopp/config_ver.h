// config_ver.h - written and placed in public domain by Jeffrey Walton
//                the bits that make up this source file are from the
//                library's monolithic config.h.

/// \file config_ver.h
/// \brief Library configuration file
/// \details <tt>config.h</tt> was split into components in May 2019 to better
///  integrate with Autoconf and its feature tests. The splitting occured so
///  users could continue to include <tt>config.h</tt> while allowing Autoconf
///  to write new <tt>config_asm.h</tt> and new <tt>config_cxx.h</tt> using
///  its feature tests.
/// \sa <A HREF="https://github.com/weidai11/cryptopp/issues/835">Issue 835</A>
/// \since Crypto++ 8.3

#ifndef CRYPTOPP_CONFIG_VERSION_H
#define CRYPTOPP_CONFIG_VERSION_H

// Library version macro. Since this macro is in a header, it reflects
// the version of the library the headers came from. It is not
// necessarily the version of the library built as a shared object if
// versions are inadvertently mixed and matched.
#define CRYPTOPP_MAJOR 8
#define CRYPTOPP_MINOR 3
#define CRYPTOPP_REVISION 0
#define CRYPTOPP_VERSION 830

// Compiler version macros

#if defined(__GNUC__)
# define CRYPTOPP_GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

#if defined(__xlc__) || defined(__xlC__)
# define CRYPTOPP_XLC_VERSION ((__xlC__ / 256) * 10000 + (__xlC__ % 256) * 100)
#endif

// Apple and LLVM's Clang. Apple Clang version 7.0 roughly equals LLVM Clang version 3.7
// Also see https://gist.github.com/yamaya/2924292
#if defined(__clang__) && defined(__apple_build_version__)
# undef CRYPTOPP_GCC_VERSION
# define CRYPTOPP_APPLE_CLANG_VERSION (__clang_major__ * 10000 + __clang_minor__ * 100 + __clang_patchlevel__)
#elif defined(__clang__)
# undef CRYPTOPP_GCC_VERSION
# define CRYPTOPP_LLVM_CLANG_VERSION  (__clang_major__ * 10000 + __clang_minor__ * 100 + __clang_patchlevel__)
#endif

#ifdef _MSC_VER
# define CRYPTOPP_MSC_VERSION (_MSC_VER)
#endif

#endif  // CRYPTOPP_CONFIG_VERSION_H
