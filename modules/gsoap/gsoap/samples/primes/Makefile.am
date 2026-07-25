####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = primes
SOAPHEADER = primes.h

include ../Makefile.defines
include ../Makefile.cpp_rules
SOAP_NS =
SOAP_FLAGS=-0 -CS

primes_SOURCES = $(SOAPHEADER) primes.cpp simple_vector.h $(SOAP_CPP_CORE)
primes_LDADD = $(SOAP_CPP_LIB) $(SAMPLE_EXTRA_LIBS)
