####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = mtom
SOAPHEADER = mtom-test.h

include ../Makefile.defines
include ../Makefile.c_rules
SOAP_NS = mtom_test.nsmap mtom_test.wsdl

mtom_SOURCES = mtom-test.c $(SOAPHEADER) $(SOAP_C_SRC)
mtom_LDADD = $(SOAP_C_LIB) $(SAMPLE_EXTRA_LIBS)


