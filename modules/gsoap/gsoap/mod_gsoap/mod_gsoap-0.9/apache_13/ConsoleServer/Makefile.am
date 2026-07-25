## this file is input for automake and will generate Makefile.in

AM_CPPFLAGS=-D_REENTRANT $(CXXDEBUGDEFINES)

INCLUDES=-I$(GSOAP_PREFIX) -I$(top_builddir)

apache_dir=$(top_builddir)

SUBDIRS = foo

bin_PROGRAMS = soaptest
soaptest_SOURCES = ConsoleMain.cpp 

soaptest_LDADD = \
	-lltdl \
	-lpthread

CLEANFILES=*~ .kdbgrc.soaptest
