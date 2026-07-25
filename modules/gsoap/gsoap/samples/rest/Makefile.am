####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = foreign 1.4

noinst_PROGRAMS = calcrest httpgettest httpposttest
SOAPHEADER = calcrest.h

include ../Makefile.defines
include ../Makefile.c_rules
SOAP_NS = $(SOAP_NS_DEF)

calcrest_SOURCES = calcrest.c calcrest.h $(SOAP_C_SRC)
httpgettest_SOURCES = httpgettest.c httpgettest.h $(SOAP_C_CORE) ../../plugin/httpget.c
httpposttest_SOURCES = httpposttest.c httpposttest.h $(SOAP_C_SERVER) ../../plugin/httppost.c

