####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = domcpp dom2calc
SOAPHEADER = dom2calc.h

include ../Makefile.defines
include ../Makefile.cpp_rules
SOAP_NS = $(SOAP_NS_DEF)

domcpp_SOURCES = domcpp.cpp $(SOAPHEADER) $(SOAP_CPP_CORE)
domcpp_LDADD = $(SOAP_CPP_LIB) $(SAMPLE_EXTRA_LIBS)

dom2calc_SOURCES = $(SOAPHEADER) dom2calc.cpp $(SOAP_CPP_CORE)
dom2calc_LDADD = $(SOAP_CPP_LIB) $(SAMPLE_EXTRA_LIBS)
