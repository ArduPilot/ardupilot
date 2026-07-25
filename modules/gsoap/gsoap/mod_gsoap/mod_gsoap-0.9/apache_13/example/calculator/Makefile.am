## this file is input for automake and will generate Makefile.in

AM_CPPFLAGS=-D_REENTRANT $(CXXDEBUGDEFINES) -DDEBUG

CLEANFILES=*~ *.log *.xml calc.nsmap soapC.cpp soapH.h soapServer.cpp ns.xsd calc.wsdl soapClient.cpp soapStub.h client


BUILT_SOURCES = soapServer.cpp soapC.cpp calc.nsmap soapH.h ns.xsd calc.wsdl calc.nsmap

lib_LTLIBRARIES = libCalculator.la

noinst_PROGRAMS = client

client_SOURCES=calcclnt.cpp soapC.cpp soapClient.cpp
client_CXXFLAGS=$(AM_CFLAGS)
calculator.cpp: calc.nsmap soapH.h

libCalculator_la_SOURCES = \
    calculator.cpp \
	soapServer.cpp \
	soapC.cpp

libCalculator_la_LIBADD = 

client_LDADD=-lgsoap++
libCalculator_la_LDFLAGS = -version-info 0:0:0 -module -lgsoap++

INCLUDES=-I$(top_builddir)

soapC.cpp soapServer.cpp ns.nsmap ns.xsd calc.wsdl soapH.h calc.nsmap: calculator.h
	soapcpp2 calculator.h

