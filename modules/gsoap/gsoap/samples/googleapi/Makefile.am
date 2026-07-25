####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = gdm gdx
SOAPHEADER = gdm.hpp

include ../Makefile.defines
include ../Makefile.cpp_rules
SOAP_NS = gdm.nsmap

CLEANFILES = gdmStub.h gdmH.h gdmC.cpp soapStub.h soapH.h soapC.cpp *.nsmap *~

$(SOAP_CPP_CORE):	env.h
			$(SOAP) -I$(top_srcdir)/gsoap/import:$(top_srcdir)/gsoap $(SOAP_FLAGS) $<

gdm_CFLAGS = $(WSDL2H_EXTRA_FLAGS)
gdm_SOURCES = gdm.cpp $(SOAPHEADER) gdmC.cpp $(SOAP_CPP_CORE)
gdm_LDADD = $(SOAP_CPP_LIB_SSL) $(SAMPLE_EXTRA_LIBS) $(SAMPLE_SSL_LIBS)

gdx_CFLAGS = $(WSDL2H_EXTRA_FLAGS)
gdx_SOURCES = gdx.cpp $(SOAPHEADER) gdmC.cpp $(SOAP_CPP_CORE)
gdx_LDADD = $(SOAP_CPP_LIB_SSL) $(SAMPLE_EXTRA_LIBS) $(SAMPLE_SSL_LIBS)
