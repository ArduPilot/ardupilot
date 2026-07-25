####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = atom
SOAPHEADER = atom.h

include ../Makefile.defines
include ../Makefile.cpp_rules
SOAP_NS = $(SOAP_NS_DEF)

atom_SOURCES = atom.cpp $(SOAPHEADER) $(SOAP_CPP_CORE)

