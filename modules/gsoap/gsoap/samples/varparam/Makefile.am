####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = varparam
SOAPHEADER = varparam.h

include ../Makefile.defines
include ../Makefile.cpp_rules
SOAP_NS = varparam.nsmap varparam.wsdl

varparam_SOURCES = varparam.cpp $(SOAPHEADER) $(SOAP_CPP_CLIENT) soapServer.cpp

