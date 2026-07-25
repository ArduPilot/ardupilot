####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = event handler
SOAPHEADER = event.h

include ../Makefile.defines
include ../Makefile.cpp_proxy_rules
SOAP_NS = Event.nsmap Event.wsdl

event_SOURCES = event.cpp $(SOAPHEADER) $(SOAP_CPP_CORE) soapEventProxy.cpp
event_LDADD = $(SOAP_CPP_LIB) $(SAMPLE_EXTRA_LIBS)

handler_SOURCES = handler.cpp $(SOAPHEADER) $(SOAP_CPP_CORE) soapEventService.cpp soapEventProxy.cpp
handler_LDADD = $(SOAP_CPP_LIB) $(SAMPLE_EXTRA_LIBS) -lpthread


