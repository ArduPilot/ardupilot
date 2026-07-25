# Process this file with autoconf to produce a configure script.
AC_INIT(gsoap, 2.8)
AM_INIT_AUTOMAKE([foreign])
AC_CONFIG_SRCDIR([gsoap/stdsoap2.cpp])
AC_CANONICAL_HOST

# AC_CONFIG_HEADERS([config.h])
AM_CONFIG_HEADER(config.h)

# we use subdirs.
AC_PROG_MAKE_SET 
# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AM_PROG_CC_C_O
AM_PROG_LEX
AC_PROG_YACC
AC_PROG_CPP
AC_PROG_RANLIB
#AM_PROG_LIBTOOL
AC_PROG_LN_S
AC_PROG_AWK
AC_PROG_INSTALL

AC_CHECK_TOOL([AR], [ar], [:])

# AC_DECL_YYTEXT

SAMPLE_EXTRA_LIBS="-lm"

# Platform-specific Makefile setup
case "${host}" in
        *-*-solaris*)   platform=SUN_OS 
			SAMPLE_EXTRA_LIBS="-lxnet -lsocket -lnsl -lm"
			WSDL2H_EXTRA_LIBS="-lxnet -lsocket -lnsl -lrt"
			;;
        *-*-sysv5*)     platform=UNIXWARE ;;
        *-*-UnixWare*)  platform=UNIXWARE ;;
        *-*-unixware*)  platform=UNIXWARE ;;
	*-*-sco3.2v5*)  platform=OPENSERVER
	                SAMPLE_EXTRA_LIBS="-lsocket -lm"
			;;
        *-*-linux*)     platform=LINUX
#			if test x$debug = xtrue; then 
#			    CFLAGS="-g" #don't want the -O2 for debugging.
#			else
#			    CFLAGS="-O2" #don't want the -g for release compile.
#			fi			
			;;
	*-*-cygwin*)	platform=CYGWIN ;;
	*-*-mingw*)	platform=MINGW
			SAMPLE_EXTRA_LIBS="-lws2_32 -lkernel32 -luser32 -lgdi32 -lm"
			WSDL2H_EXTRA_LIBS="-lws2_32 -lkernel32 -luser32 -lgdi32 -lm"
			;;
        *-*-freebsd*)   platform=FREEBSD ;;
        *-*-openbsd*)   platform=OPENBSD ;;
        *-*-irix*)      platform=IRIX ;;
        *-*-aix*)       platform=AIX
                        case "${host}" in
                            *-*-aix4.1*)   osver=AIX41;;
                            *-*-aix4.2*)   osver=AIX42;;
                            *-*-aix4.*)    osver=AIX43;;
                            *-*-aix5.1*)   osver=AIX51;;
                            *-*-aix5.2*)   osver=AIX52;;
                            *-*-aix5.3*)   osver=AIX53;;
                            *-*-aix6.*)    osver=AIX61;;
                            *-*-aix7.*)    osver=AIX71;;
                        esac
			SAMPLE_EXTRA_LIBS="-lm"
			;;
        *-sequent-*)    platform=PTX ;;
        *-*-hp*)        platform=HP_UX ;
                        case "${host}" in
                            *-*-hpux11*)  osver=HPUX11;;
                            *-*-hpux10*)  osver=HPUX10;;
                            *)            osver=HPUX11;;
                        esac
			;;
        *-*-mvs*)       platform=OS390 ;;
        *-*-os400*)     platform=OS400 ;;
        *-*-OS400*)     platform=OS400 ;;
        *-*-osf*)	platform=TRU64 ;;
        *-apple-*)	platform=MACOSX
			# Appease 10.7 Lion (OpenSSL deprecated)
			CXXFLAGS="-Wno-deprecated-declarations"
			CFLAGS="-Wno-deprecated-declarations"
			;;
	*-nto-qnx)	platform=QNX
			SAMPLE_EXTRA_LIBS="-lsocket -lm"
			;;
        *)              platform=UNKNOWN
			SAMPLE_EXTRA_LIBS="-lm"
                        ;;
esac

AC_SUBST(SAMPLE_EXTRA_LIBS)
AC_SUBST(platform)

# Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS([arpa/inet.h locale.h xlocale.h errno.h fcntl.h ctype.h limits.h float.h math.h netdb.h netinet/in.h stdlib.h string.h strings.h stdint.h inttypes.h time.h sys/inttypes.h sys/socket.h sys/types.h sys/time.h sys/timeb.h unistd.h poll.h openssl/ssl.h gnutls/gnutls.h zlib.h])


AC_CHECK_TYPES([socklen_t],,,
[
#if HAVE_SYS_SOCKET_H
# include <sys/socket.h>
#endif
])

# Checks for typedefs, structures, and compiler characteristics.
AC_C_CONST
AC_TYPE_SIZE_T
AC_HEADER_TIME
AC_STRUCT_TM
AC_CHECK_MEMBERS([struct tm.tm_gmtoff, struct tm.__tm_gmtoff],,,
[
#include <sys/types.h>
#include <time.h>
])

# Checks for library functions.
AC_FUNC_MKTIME
AC_TYPE_SIGNAL
AC_FUNC_STRFTIME
AC_CHECK_FUNCS([random gettimeofday ftime localtime_r timegm gmtime gmtime_r asctime asctime_r memset select socket snprintf strchr strerror strerror_r strlcpy strlcat strrchr strstr strtol strtoul strtoll strtoull strtold strtod strtof strtold_l strtod_l strtof_l sscanf sscanf_l snprintf sprintf_l poll newlocale uselocale freelocale])

# Checks for isnan and isinf
AC_CHECK_FUNC(isnan, [has_isnan=1; AC_DEFINE(HAVE_ISNAN, 1, isnan)], has_isnan=0)
AC_CHECK_FUNC(isinf, [has_isinf=1; AC_DEFINE(HAVE_ISINF, 1, isinf)], has_isinf=0)

# check for macro isnan in math
if test $has_isnan = 0; then
  AC_MSG_CHECKING(for macro isnan)
  AC_LINK_IFELSE([AC_LANG_PROGRAM([[#include <cmath>]], [[{ int foo; foo = isnan(0); }]])],[has_isnan=1],[has_isnan=0 ])
  if test $has_isnan = 1; then
    AC_MSG_RESULT(yes)
    AC_DEFINE(HAVE_ISNAN, 1, isnan)
  else
    AC_MSG_RESULT(no)
  fi
fi

# check for macro isinf in math
if test $has_isinf = 0; then
  AC_MSG_CHECKING(for macro isinf)
  AC_LINK_IFELSE([AC_LANG_PROGRAM([[#include <cmath>]], [[{ int foo; foo = isinf(0); }]])],[has_isinf=1],[has_isinf=0 ])
  if test $has_isinf = 1; then
    AC_MSG_RESULT(yes)
    AC_DEFINE(HAVE_ISINF, 1, isinf)
  else
    AC_MSG_RESULT(no)
  fi
fi

# Function test depending on platform
# AIX Has incompatible gethostbyname_r
if test $platform != AIX -a $platform != TRU64; then
  AC_CHECK_FUNCS([gethostbyname_r])
  AC_FUNC_MALLOC
fi

# the debug build options adds symbols to compiler output (-g for g++)
AC_ARG_ENABLE(debug,
[  --enable-debug	  add debug symbols for debugging],
[case "${enableval}" in
  yes) debug=true ;;
  no)  debug=false ;;
  *) AC_MSG_ERROR(bad value ${enableval} for --enable-debug) ;;
esac],[debug=false])
AM_CONDITIONAL(DEBUG_COMPILE, test x$debug = xtrue)
if test x$debug = xtrue; then 
    SOAPCPP2_DEBUG="-DDEBUG" ##define DEBUG for debugging
else
    SOAPCPP2_DEBUG=
fi			
AC_SUBST(SOAPCPP2_DEBUG)

# set the default soapcpp2 import path
SOAPCPP2_IMPORTPATH="-DSOAPCPP2_IMPORT_PATH=\"\\\"${datadir}/gsoap/import\\\"\""
#SOAPCPP2_IMPORTPATH="-DSOAPCPP2_IMPORT_PATH=\"\\\"`pwd`/gsoap/import\\\"\""
AC_SUBST(SOAPCPP2_IMPORTPATH)

# set the default wsdl2h import path
WSDL2H_IMPORTPATH="-DWSDL2H_IMPORT_PATH=\"\\\"${datadir}/gsoap/WS\\\"\""
#WSDL2H_IMPORTPATH="-DWSDL2H_IMPORT_PATH=\"\\\"`pwd`/gsoap/WS\\\"\""
AC_SUBST(WSDL2H_IMPORTPATH)

# the disable-namespaces option adds the -DWITH_NONAMESPACES to the
# library compilation thus disabling namespaces
AC_ARG_ENABLE(namespaces,
              [AC_HELP_STRING([--disable-namespaces],
                              [compile library without namespaces])],
              [with_namespaces="$enable_namespaces"],
              [with_namespaces="yes"])

AC_MSG_CHECKING(for disable namespaces in library)
if test "x$with_namespaces" = "xyes"; then
  AC_MSG_RESULT(no)
else
  AC_MSG_RESULT(yes)
  SOAPCPP2_NONAMESPACES="-DWITH_NONAMESPACES" ##define to remove nsmap link dependence
fi
AC_SUBST(SOAPCPP2_NONAMESPACES)

# the enable-xlocale option adds the -DWITH_INCLUDE_XLOCALE_H to the
# library compilation thus forcing the inclusion of xlocale.h
AC_ARG_ENABLE(xlocale,
              [AC_HELP_STRING([--enable-xlocale],
                              [compile library with forced inclusion of xlocale.h])],
              [with_xlocale="$enable_xlocale"],
              [with_xlocale="no"])

AC_MSG_CHECKING(for enable xlocale usage in library)
if test "x$with_xlocale" = "xyes"; then
  AC_MSG_RESULT(yes)
  SOAPCPP2_INCLUDE_XLOCALE="-DWITH_INCLUDE_XLOCALE_H"
else
  AC_MSG_RESULT(no)
fi
AC_SUBST(SOAPCPP2_INCLUDE_XLOCALE)

# the disable-c-locale option adds the -DWITH_NO_C_LOCALE to the
# library compilation thus disabling the use of the C locale usage
AC_ARG_ENABLE(c-locale,
              [AC_HELP_STRING([--disable-c-locale],
                              [compile library without c locale usage])],
              [with_no_c_locale="$enable_c_locale"],
              [with_no_c_locale="yes"])

AC_MSG_CHECKING(for disable c locale usage in library)
if test "x$with_no_c_locale" = "xyes"; then
  AC_MSG_RESULT(no)
else
  AC_MSG_RESULT(yes)
  SOAPCPP2_NO_C_LOCALE="-DWITH_NO_C_LOCALE"
fi
AC_SUBST(SOAPCPP2_NO_C_LOCALE)

# the enable-ipv6 option adds the -DWITH_IPV6 to the
# library compilation thus enabling IPv6
AC_ARG_ENABLE(ipv6,
              [AC_HELP_STRING([--enable-ipv6],
                              [compile library with IPv6 support])],
              [with_ipv6="$enable_ipv6"],
              [with_ipv6="no"])

AC_MSG_CHECKING(for enable ipv6 in library)
if test "x$with_ipv6" = "xyes"; then
  AC_MSG_RESULT(yes)
  SOAPCPP2_IPV6="-DWITH_IPV6" ##define to add IPv6 support
else
  AC_MSG_RESULT(no)
fi
AC_SUBST(SOAPCPP2_IPV6)

# the enable-ipv6-v6only option adds the -DWITH_IPV6_V6ONLY to the
# library compilation thus enabling IPv6 without remapping IPv4 to IPv6
AC_ARG_ENABLE(ipv6-v6only,
              [AC_HELP_STRING([--enable-ipv6-v6only],
                              [compile library with IPv6-v6only support])],
              [with_ipv6_v6only="$enable_ipv6_v6only"],
              [with_ipv6_v6only="no"])

AC_MSG_CHECKING(for enable ipv6-v6only in library)
if test "x$with_ipv6_v6only" = "xyes"; then
  AC_MSG_RESULT(yes)
  SOAPCPP2_IPV6_V6ONLY="-DWITH_IPV6_V6ONLY"
else
  AC_MSG_RESULT(no)
fi
AC_SUBST(SOAPCPP2_IPV6_V6ONLY)

# the zlib installation prefix path
AC_ARG_WITH([zlib],
            AS_HELP_STRING([--with-zlib=DIR],
                           [zlib installation prefix]),
            [ZLIB=$withval],)

# the OpenSSL installation prefix path
AC_ARG_WITH([openssl],
            AS_HELP_STRING([--with-openssl=DIR],
                           [openssl installation prefix]),
            [OPENSSL=$withval],)

# the disable-ssl option removes the dependence on OpenSSL
AC_ARG_ENABLE(ssl,
              [AC_HELP_STRING([--disable-ssl],
                              [build without SSL/TLS -- note: the wsse and wst examples will fail to build])],
              [with_openssl="$enable_ssl"],
              [with_openssl="yes"])

# the enable-gnutls option enables GNUTLS in favor of OpenSSL
AC_ARG_ENABLE(gnutls,
              [AC_HELP_STRING([--enable-gnutls],
                              [build with GNUTLS SSL/TLS])],
              [with_gnutls="$enable_gnutls"],
              [with_gnutls="no"])

AC_MSG_CHECKING(for disable openssl in library)
if test "x$with_openssl" = "xyes"; then
  AC_MSG_RESULT(no)
  AC_MSG_CHECKING(for enable gnutls in library)
  if test "x$with_gnutls" = "xyes"; then
    AC_MSG_RESULT(yes)
    WSDL2H_EXTRA_FLAGS="-DWITH_GNUTLS -DWITH_GZIP"
    WSDL2H_EXTRA_LIBS="${WSDL2H_EXTRA_LIBS} -lgnutls -lgcrypt -lgpg-error -lz"
    SAMPLE_INCLUDES=
    SAMPLE_SSL_LIBS="-lgnutls -lgcrypt -lgpg-error -lz"
    WSDL2H_SOAP_CPP_LIB="libgsoapssl++.a"
  else
    AC_MSG_RESULT(no)
    WSDL2H_EXTRA_FLAGS="-DWITH_OPENSSL -DWITH_GZIP"
    # compile with wsdl2h when OPENSSL is available
    WSDL2H_EXTRA_LIBS="${WSDL2H_EXTRA_LIBS} -lssl -lcrypto -lz"
    SAMPLE_INCLUDES=
    SAMPLE_SSL_LIBS="-lssl -lcrypto -lz"
    WSDL2H_SOAP_CPP_LIB="libgsoapssl++.a"
  fi
  if test -n "$ZLIB"; then
    WSDL2H_EXTRA_FLAGS="-I${ZLIB}/include ${WSDL2H_EXTRA_FLAGS}"
    WSDL2H_EXTRA_LIBS="-L${ZLIB}/lib ${WSDL2H_EXTRA_LIBS}"
    SAMPLE_INCLUDES="-I${ZLIB}/include ${SAMPLE_SSL_LIBS}"
    SAMPLE_SSL_LIBS="-L${ZLIB}/lib ${SAMPLE_SSL_LIBS}"
  fi
  if test -n "$OPENSSL"; then
    WSDL2H_EXTRA_FLAGS="-I${OPENSSL}/include ${WSDL2H_EXTRA_FLAGS}"
    WSDL2H_EXTRA_LIBS="-L${OPENSSL}/lib ${WSDL2H_EXTRA_LIBS}"
    SAMPLE_INCLUDES="-I${OPENSSL}/include"
    SAMPLE_SSL_LIBS="-L${OPENSSL}/lib ${SAMPLE_SSL_LIBS}"
  fi
else
  AC_MSG_RESULT(yes)
  WSDL2H_EXTRA_FLAGS=
  SAMPLE_SSL_LIBS=
  SAMPLE_INCLUDES=
  WSDL2H_SOAP_CPP_LIB="libgsoap++.a"
fi
AM_CONDITIONAL([WITH_OPENSSL], [test "x$with_openssl" = "xyes" -a "x$with_gnutls" != "xyes"])
AC_SUBST(WSDL2H_EXTRA_FLAGS)
AC_SUBST(WSDL2H_EXTRA_LIBS)
AC_SUBST(SAMPLE_INCLUDES)
AC_SUBST(SAMPLE_SSL_LIBS)
AC_SUBST(WSDL2H_SOAP_CPP_LIB)

# enable the compile of the samples
AC_ARG_ENABLE(samples,
[  --enable-samples	  enable compile for the gsoap samples],
[case "${enableval}" in
  yes) samples=true ;;
  no)  samples=false ;;
  *) AC_MSG_ERROR(bad value ${enableval} for --enable-samples) ;;
esac],[samples=false])
AM_CONDITIONAL(ENABLE_SAMPLES, test x$samples = xtrue)
AC_SUBST(ENABLE_SAMPLES)

if test x$samples = xtrue; then
   SAMPLE_DIRS=". samples"
else
   SAMPLE_DIRS=
fi
AC_SUBST(SAMPLE_DIRS)

if test x$LEX = xflex; then
   LEX_DEFINE=-DWITH_FLEX
   LEX_FLAGS="-l"
else
   AC_MSG_WARN(No 'flex' command detected on current path: trying existing scanner, but this may fail to build wsdl2h)
   LEX_DEFINE=-DWITH_LEX
   LEX_FLAGS=
fi
AC_SUBST(LEX_DEFINE)
AC_SUBST(LEX_FLAGS)
AC_SUBST(LEXLIB)

# check if we must define -DWITH_BISON
if test "$YACC" = "bison -y"; then
   BISON_DEFINE=-DWITH_BISON
   YACC_LIB=
else
   BISON_DEFINE=-DWITH_YACC
   YACC_LIB="-ly"
fi
AC_SUBST(BISON_DEFINE)
AC_SUBST(YACC_LIB)

AC_CONFIG_FILES([ Makefile \
    gsoap.pc gsoap++.pc gsoapck.pc gsoapck++.pc gsoapssl.pc gsoapssl++.pc \
    gsoap/Makefile \
    gsoap/src/Makefile \
    gsoap/wsdl/Makefile \
    gsoap/samples/Makefile \
    gsoap/samples/autotest/Makefile \
    gsoap/samples/aws/Makefile \
    gsoap/samples/calc/Makefile \
    gsoap/samples/calc++/Makefile \
    gsoap/samples/chaining/Makefile \
    gsoap/samples/chaining++/Makefile \
    gsoap/samples/databinding/Makefile \
    gsoap/samples/dime/Makefile \
    gsoap/samples/dom/Makefile \
    gsoap/samples/oneway/Makefile \
    gsoap/samples/oneway++/Makefile \
    gsoap/samples/factory/Makefile \
    gsoap/samples/factorytest/Makefile \
    gsoap/samples/gmt/Makefile \
    gsoap/samples/googleapi/Makefile \
    gsoap/samples/hello/Makefile \
    gsoap/samples/httpcookies/Makefile \
    gsoap/samples/lu/Makefile \
    gsoap/samples/magic/Makefile \
    gsoap/samples/mashup/Makefile \
    gsoap/samples/mashup++/Makefile \
    gsoap/samples/mtom/Makefile \
    gsoap/samples/mtom-stream/Makefile \
    gsoap/samples/polytest/Makefile \
    gsoap/samples/primes/Makefile \
    gsoap/samples/roll/Makefile \
    gsoap/samples/router/Makefile \
    gsoap/samples/atom/Makefile \
    gsoap/samples/rss/Makefile \
    gsoap/samples/ssl/Makefile \
    gsoap/samples/template/Makefile \
    gsoap/samples/udp/Makefile \
    gsoap/samples/varparam/Makefile \
    gsoap/samples/wsa/Makefile \
    gsoap/samples/wsrm/Makefile \
    gsoap/samples/wsse/Makefile \
    gsoap/samples/wst/Makefile \
    gsoap/samples/xml-rpc-json/Makefile \
    gsoap/samples/rest/Makefile \
    gsoap/samples/testmsgr/Makefile \
    gsoap/samples/async/Makefile
])
AC_OUTPUT
