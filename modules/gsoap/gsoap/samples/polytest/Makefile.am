####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = polytest.cgi
SOAPHEADER = polymorph.h

include ../Makefile.defines
include ../Makefile.cpp_rules
SOAP_NS = polymorph.nsmap polymorph.wsdl

polytest_cgi_SOURCES = polymorph.cpp $(SOAPHEADER) $(SOAP_CPP_SRC)

