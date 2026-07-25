####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = rollclient rollserver
SOAPHEADER = roll.h

include ../Makefile.defines
include ../Makefile.cpp_rules
SOAP_NS = r.nsmap r.wsdl

rollclient_SOURCES = rollclient.cpp $(SOAPHEADER) $(SOAP_CPP_CLIENT)
rollclient_LDADD = $(SOAP_CPP_LIB) $(SAMPLE_EXTRA_LIBS)

rollserver_SOURCES = rollserver.cpp $(SOAPHEADER) $(SOAP_CPP_SERVER)
rollserver_LDADD = $(SOAP_CPP_LIB) $(SAMPLE_EXTRA_LIBS)
