####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = dimeclient dimeserver
SOAPHEADER = dime.h

include ../Makefile.defines
include ../Makefile.cpp_rules
SOAP_NS = dime.nsmap dime.wsdl

dimeclient_SOURCES = dimeclient.cpp $(SOAPHEADER) $(SOAP_CPP_CLIENT)
dimeclient_LDADD = $(SOAP_CPP_LIB) $(SAMPLE_EXTRA_LIBS)

dimeserver_SOURCES = dimeserver.cpp $(SOAPHEADER) $(SOAP_CPP_SERVER)
dimeserver_LDADD = $(SOAP_CPP_LIB) $(SAMPLE_EXTRA_LIBS) -lpthread

