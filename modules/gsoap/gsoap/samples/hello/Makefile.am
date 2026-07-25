####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = helloclient helloserver
SOAPHEADER = hello.h

include ../Makefile.defines
include ../Makefile.cpp_rules
SOAP_NS = h.nsmap h.wsdl

helloclient_SOURCES = helloclient.cpp $(SOAPHEADER) $(SOAP_CPP_CLIENT)
helloclient_LDADD = $(SOAP_CPP_LIB) $(SAMPLE_EXTRA_LIBS)

helloserver_SOURCES = helloserver.cpp $(SOAPHEADER) $(SOAP_CPP_SERVER)
helloserver_LDADD = $(SOAP_CPP_LIB) $(SAMPLE_EXTRA_LIBS)
