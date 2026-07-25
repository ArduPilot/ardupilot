## This is the input file for automake, which will generate Makefile.in ##
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

SUBDIRS = . src wsdl @SAMPLE_DIRS@
CLEANFILES = soapcpp2 wsdl2h stdsoap2_cpp.cpp stdsoap2_ck.c stdsoap2_ck_cpp.cpp stdsoap2_ssl.c stdsoap2_ssl_cpp.cpp dom_cpp.cpp

##	*~ *.req.xml *.res.xml *.nsmap *.wsdl *.xsd *Proxy.h \
##	*.o soapH.h soapStub.h soapC.cpp soapC.c soapClient.cpp soapClient.c soapServer.cpp soapServer.c

AM_CXXFLAGS = $(SOAPCPP2_DEBUG) -D$(platform)
AM_CFLAGS = $(SOAPCPP2_DEBUG) -D$(platform)

# Install all soapcpp2 and wsdl2h files into ${prefix}/share/gsoap
nobase_pkgdata_DATA = $(top_srcdir)/gsoap/import/* $(top_srcdir)/gsoap/plugin/* $(top_srcdir)/gsoap/WS/* $(top_srcdir)/gsoap/custom/* $(top_srcdir)/gsoap/extras/*

## we cannot build stdsoap2.o from 2 different sources (stdsoap2.cpp and stdsoap2.c), so we need an intermediate target:
stdsoap2_ck.c: stdsoap2.cpp
	$(LN_S) -f $(top_srcdir)/gsoap/stdsoap2.cpp stdsoap2_ck.c

stdsoap2_cpp.cpp: stdsoap2.cpp
	$(LN_S) -f $(top_srcdir)/gsoap/stdsoap2.cpp stdsoap2_cpp.cpp

stdsoap2_ck_cpp.cpp: stdsoap2.cpp
	$(LN_S) -f $(top_srcdir)/gsoap/stdsoap2.cpp stdsoap2_ck_cpp.cpp

stdsoap2_ssl.c: stdsoap2.cpp
	$(LN_S) -f $(top_srcdir)/gsoap/stdsoap2.cpp stdsoap2_ssl.c

stdsoap2_ssl_cpp.cpp: stdsoap2.cpp
	$(LN_S) -f $(top_srcdir)/gsoap/stdsoap2.cpp stdsoap2_ssl_cpp.cpp
## do the same fo dom.cpp since we want to link it into the cpp libs and dom.c into the c libs
dom_cpp.cpp: dom.cpp
	$(LN_S) -f $(top_srcdir)/gsoap/dom.cpp dom_cpp.cpp

lib_LIBRARIES = libgsoap.a libgsoap++.a libgsoapck.a libgsoapck++.a libgsoapssl.a libgsoapssl++.a

libgsoap_a_SOURCES = stdsoap2.c dom.c
libgsoap_a_CFLAGS = $(SOAPCPP2_DEBUG) $(SOAPCPP2_NONAMESPACES) $(SOAPCPP2_NO_C_LOCALE) $(SOAPCPP2_IPV6) $(SOAPCPP2_IPV6_V6ONLY) -D$(platform)
libgsoap___a_SOURCES = stdsoap2_cpp.cpp dom_cpp.cpp
libgsoap___a_CXXFLAGS = $(SOAPCPP2_DEBUG) $(SOAPCPP2_NONAMESPACES) $(SOAPCPP2_NO_C_LOCALE) $(SOAPCPP2_IPV6) $(SOAPCPP2_IPV6_V6ONLY) -D$(platform)
libgsoapck_a_SOURCES = stdsoap2_ck.c dom.c
libgsoapck_a_CFLAGS = $(SOAPCPP2_DEBUG) $(SOAPCPP2_NONAMESPACES) $(SOAPCPP2_NO_C_LOCALE) $(SOAPCPP2_IPV6) $(SOAPCPP2_IPV6_V6ONLY) -D$(platform) -DWITH_COOKIES
libgsoapck___a_SOURCES = stdsoap2_ck_cpp.cpp dom_cpp.cpp
libgsoapck___a_CXXFLAGS = $(SOAPCPP2_DEBUG) $(SOAPCPP2_NONAMESPACES) $(SOAPCPP2_NO_C_LOCALE) $(SOAPCPP2_IPV6) $(SOAPCPP2_IPV6_V6ONLY) -D$(platform) -DWITH_COOKIES
libgsoapssl_a_SOURCES = stdsoap2_ssl.c dom.c
libgsoapssl_a_CFLAGS = $(SOAPCPP2_DEBUG) $(SOAPCPP2_NONAMESPACES) $(SOAPCPP2_NO_C_LOCALE) $(SOAPCPP2_IPV6) $(SOAPCPP2_IPV6_V6ONLY) -D$(platform) $(WSDL2H_EXTRA_FLAGS) -DWITH_DOM -DWITH_COOKIES
libgsoapssl___a_SOURCES = stdsoap2_ssl_cpp.cpp dom_cpp.cpp
libgsoapssl___a_CXXFLAGS = $(SOAPCPP2_DEBUG) $(SOAPCPP2_NONAMESPACES) $(SOAPCPP2_NO_C_LOCALE) $(SOAPCPP2_IPV6) $(SOAPCPP2_IPV6_V6ONLY) -D$(platform) $(WSDL2H_EXTRA_FLAGS) -DWITH_DOM -DWITH_COOKIES

BUILT_SOURCES = stdsoap2_cpp.cpp dom_cpp.cpp stdsoap2_ck.c stdsoap2_ck_cpp.cpp stdsoap2_ssl.c stdsoap2_ssl_cpp.cpp

include_HEADERS = stdsoap2.h

install-data-hook:
	$(LN_S) -f $(top_builddir)/gsoap/src/soapcpp2 $(top_builddir)/gsoap/soapcpp2 || echo "ok, link already exists".
	$(LN_S) -f $(top_builddir)/gsoap/wsdl/wsdl2h $(top_builddir)/gsoap/wsdl2h || echo "ok, link already exists".

