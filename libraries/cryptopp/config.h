// config.h - originally written and placed in the public domain by Wei Dai

/// \file config.h
/// \brief Library configuration file
/// \details <tt>config.h</tt> was split into components in May 2019 to better
///  integrate with Autoconf and its feature tests. The splitting occured so
///  users could continue to include <tt>config.h</tt> while allowing Autoconf
///  to write new <tt>config_asm.h</tt> and new <tt>config_cxx.h</tt> using
///  its feature tests.
/// \sa <A HREF="https://github.com/weidai11/cryptopp/issues/835">Issue 835</A>
/// \since Crypto++ 8.3

/// \file config.h
/// \brief Library configuration file

#ifndef CRYPTOPP_CONFIG_H
#define CRYPTOPP_CONFIG_H

#include "config_align.h"
#include "config_asm.h"
#include "config_cpu.h"
#include "config_cxx.h"
#include "config_dll.h"
#include "config_int.h"
#include "config_misc.h"
#include "config_ns.h"
#include "config_os.h"
#include "config_ver.h"

#endif // CRYPTOPP_CONFIG_H
