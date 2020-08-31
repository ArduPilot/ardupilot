// config_dll.h - written and placed in public domain by Jeffrey Walton
//                the bits that make up this source file are from the
//                library's monolithic config.h.

/// \file config_dll.h
/// \brief Library configuration file
/// \details <tt>config.h</tt> was split into components in May 2019 to better
///  integrate with Autoconf and its feature tests. The splitting occured so
///  users could continue to include <tt>config.h</tt> while allowing Autoconf
///  to write new <tt>config_asm.h</tt> and new <tt>config_cxx.h</tt> using
///  its feature tests.
/// \sa <A HREF="https://github.com/weidai11/cryptopp/issues/835">Issue 835</A>
/// \since Crypto++ 8.3

#ifndef CRYPTOPP_CONFIG_DLL_H
#define CRYPTOPP_CONFIG_DLL_H

#include "config_os.h"

#if !defined(CRYPTOPP_DOXYGEN_PROCESSING)

#if defined(CRYPTOPP_WIN32_AVAILABLE)

#if defined(CRYPTOPP_EXPORTS)
#  define CRYPTOPP_IS_DLL
#  define CRYPTOPP_DLL __declspec(dllexport)
#elif defined(CRYPTOPP_IMPORTS)
#  define CRYPTOPP_IS_DLL
#  define CRYPTOPP_DLL __declspec(dllimport)
#else
#  define CRYPTOPP_DLL
#endif

// C++ makes const internal linkage
#define CRYPTOPP_TABLE extern
#define CRYPTOPP_API __cdecl

#else	// not CRYPTOPP_WIN32_AVAILABLE

// C++ makes const internal linkage
#define CRYPTOPP_TABLE extern
#define CRYPTOPP_DLL
#define CRYPTOPP_API

#endif	// CRYPTOPP_WIN32_AVAILABLE

#if defined(__MWERKS__)
#  define CRYPTOPP_EXTERN_DLL_TEMPLATE_CLASS extern class CRYPTOPP_DLL
#elif defined(__BORLANDC__) || defined(__SUNPRO_CC)
#  define CRYPTOPP_EXTERN_DLL_TEMPLATE_CLASS template class CRYPTOPP_DLL
#else
#  define CRYPTOPP_EXTERN_DLL_TEMPLATE_CLASS extern template class CRYPTOPP_DLL
#endif

#if defined(CRYPTOPP_MANUALLY_INSTANTIATE_TEMPLATES) && !defined(CRYPTOPP_IMPORTS)
#  define CRYPTOPP_DLL_TEMPLATE_CLASS template class CRYPTOPP_DLL
#else
#  define CRYPTOPP_DLL_TEMPLATE_CLASS CRYPTOPP_EXTERN_DLL_TEMPLATE_CLASS
#endif

#if defined(__MWERKS__)
#  define CRYPTOPP_EXTERN_STATIC_TEMPLATE_CLASS extern class
#elif defined(__BORLANDC__) || defined(__SUNPRO_CC)
#  define CRYPTOPP_EXTERN_STATIC_TEMPLATE_CLASS template class
#else
#  define CRYPTOPP_EXTERN_STATIC_TEMPLATE_CLASS extern template class
#endif

#if defined(CRYPTOPP_MANUALLY_INSTANTIATE_TEMPLATES) && !defined(CRYPTOPP_EXPORTS)
#  define CRYPTOPP_STATIC_TEMPLATE_CLASS template class
#else
#  define CRYPTOPP_STATIC_TEMPLATE_CLASS CRYPTOPP_EXTERN_STATIC_TEMPLATE_CLASS
#endif

#endif  // CRYPTOPP_DOXYGEN_PROCESSING

#endif  // CRYPTOPP_CONFIG_DLL_H
