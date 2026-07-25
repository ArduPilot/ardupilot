####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = magic magicserver
SOAPHEADER = magic.h

include ../Makefile.defines
include ../Makefile.cpp_rules
SOAP_NS = magic.nsmap magic.wsdl

magic_SOURCES = magic.cpp $(SOAPHEADER) $(SOAP_CPP_CLIENT)

magicserver_SOURCES = magicserver.cpp $(SOAPHEADER) $(SOAP_CPP_SERVER)
magicserver_LDADD = $(SOAP_CPP_LIB) $(SAMPLE_EXTRA_LIBS) -lpthread
