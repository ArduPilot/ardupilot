####### This is the input file for automake, which will generate Makefile.in ##########


## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

CPPFLAGS=-I$(top_srcdir)/gsoap -I$(top_srcdir)/gsoap/plugin
AM_LDFLAGS=$(CPPFLAGS) -I$(top_srcdir)/gsoap/samples/autotest -L$(top_srcdir)/gsoap/samples/autotest
SOAP=$(top_srcdir)/gsoap/src/soapcpp2$(EXEEXT)
WSDL=$(top_srcdir)/gsoap/wsdl/wsdl2h$(EXEEXT)
WSDL_FLAGS=-d -P -t $(top_srcdir)/gsoap/samples/autotest/typemap.dat
SOAP_FLAGS=-SL -T -I$(top_srcdir)/gsoap/import -I$(top_srcdir)/gsoap/custom
WSDLINPUT=$(top_srcdir)/gsoap/samples/autotest/examples.wsdl
SOAPHEADER=$(top_srcdir)/gsoap/samples/autotest/examples.h
SOAP_CPP_SRC=soapC.cpp soapServer.cpp
SOAP_CPP_LIB=$(top_builddir)/gsoap/libgsoap++.a

$(SOAP_CPP_SRC) : $(WSDLINPUT)
	$(WSDL) $(WSDL_FLAGS) $(WSDLINPUT)
	$(SOAP) $(SOAP_FLAGS) $(SOAPHEADER)

duration.cpp : ../../custom/duration.c
	ln -s ../../custom/duration.c duration.cpp

long_time.cpp : ../../custom/long_time.c
	ln -s ../../custom/long_time.c long_time.cpp

struct_tm.cpp : ../../custom/struct_tm.c
	ln -s ../../custom/struct_tm.c struct_tm.cpp

struct_tm_date.cpp : ../../custom/struct_tm_date.c
	ln -s ../../custom/struct_tm_date.c struct_tm_date.cpp

#LIBS=

noinst_PROGRAMS=autotest

autotest_CFLAGS=$(C_DEBUG_FLAGS) -D$(platform)
autotest_CPPFLAGS=$(C_DEBUG_FLAGS) -D$(platform)
autotest_CXXFLAGS=$(WSDL2H_EXTRA_FLAGS)
autotest_SOURCES=$(SOAP_CPP_SRC) soapTester.cpp duration.cpp long_time.cpp struct_tm.cpp struct_tm_date.cpp
autotest_LDADD=$(SOAP_CPP_LIB) $(SAMPLE_EXTRA_LIBS)

CLEANFILES= *~ *.h *.cpp *.xml *.log *.nsmap
