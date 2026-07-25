####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = mashupclient mashupserver
SOAPHEADER = mashup.hpp

include ../Makefile.defines
include ../Makefile.cpp_proxy_rules
SOAP_NS = mashup.nsmap

mashupclient_SOURCES = mashupclient.cpp $(SOAPHEADER) $(SOAP_CPP_CORE) soapmashupProxy.cpp
mashupclient_LDADD = $(SOAP_CPP_LIB) $(SAMPLE_EXTRA_LIBS)

mashupserver_SOURCES = mashupserver.cpp $(SOAPHEADER) $(SOAP_CPP_CORE) soapcalcProxy.cpp soapgmtProxy.cpp soapmashupService.cpp
mashupserver_LDADD = $(SOAP_CPP_LIB) $(SAMPLE_EXTRA_LIBS)
