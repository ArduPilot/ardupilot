####### This is the input file for automake, which will generate Makefile.in ##########


## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

CPPFLAGS=$(SAMPLE_INCLUDES) -I$(top_srcdir)/gsoap -I$(top_srcdir)/gsoap/plugin
AM_LDFLAGS=$(AM_CPPFLAGS) -I$(top_srcdir)/gsoap/samples/databinding -L$(top_srcdir)/gsoap/samples/databinding
SOAP=$(top_srcdir)/gsoap/src/soapcpp2$(EXEEXT)
WSDL=$(top_srcdir)/gsoap/wsdl/wsdl2h$(EXEEXT)
WSDL_FLAGS=-g -t $(top_srcdir)/gsoap/samples/databinding/addresstypemap.dat
SOAP_FLAGS=-0 -CS -p address -I$(top_srcdir)/gsoap/import
WSDLINPUT=$(top_srcdir)/gsoap/samples/databinding/address.xsd
SOAPHEADER=$(top_srcdir)/gsoap/samples/databinding/address.h
SOAP_CPP_SRC=addressC.cpp
SOAP_CPP_LIB=$(top_builddir)/gsoap/libgsoap++.a

$(SOAP_CPP_SRC) : $(WSDLINPUT)
	$(WSDL) $(WSDL_FLAGS) $(WSDLINPUT)
	$(SOAP) $(SOAP_FLAGS) $(SOAPHEADER)

#LIBS=

noinst_PROGRAMS=address

address_CFLAGS=$(C_DEBUG_FLAGS) -D$(platform)
address_CPPFLAGS=$(C_DEBUG_FLAGS) -D$(platform)
address_CXXFLAGS=$(WSDL2H_EXTRA_FLAGS)
address_SOURCES=$(SOAP_CPP_SRC) address.cpp
address_LDADD=$(SOAP_CPP_LIB)

CLEANFILES= *~ address.h *C.cpp *H.h *Stub.h *.nsmap
