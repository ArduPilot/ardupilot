####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = chaining

include ../Makefile.defines

envC.cpp:		env.h
	$(SOAP) -CS -penv -I$(top_srcdir)/gsoap/import:$(top_srcdir)/gsoap $(SOAP_FLAGS) $<
QuotequoteService.cpp:	quote.h
	$(SOAP) -iS -qQuote -I$(top_srcdir)/gsoap/import:$(top_srcdir)/gsoap $(SOAP_FLAGS) $<
CalccalcService.cpp:	calc.h
	$(SOAP) -iS -qCalc -I$(top_srcdir)/gsoap/import:$(top_srcdir)/gsoap $(SOAP_FLAGS) $<

BUILT_SOURCES = QuotequoteService.cpp CalccalcService.cpp envC.cpp

CLEANFILES = *.req.xml *.res.xml *.nsmap *.xsd *.wsdl envStub.h envH.h envC.cpp CalcStub.h CalcH.h CalcC.cpp Calc*Proxy.h Calc*Proxy.cpp Calc*Object.h Calc*Service.h Calc*Service.cpp CalcClient.cpp CalcServer.cpp CalcClientLib.cpp CalcServerLib.cpp QuoteStub.h QuoteH.h QuoteC.cpp Quote*Proxy.h Quote*Proxy.cpp Quote*Object.h Quote*Service.h Quote*Service.cpp QuoteClient.cpp QuoteServer.cpp QuoteClientLib.cpp QuoteServerLib.cpp *~

SOAP_NS = Calc.nsmap

chaining_SOURCES = chaining.cpp quote.h calc.h env.h QuotequoteService.cpp CalccalcService.cpp QuoteC.cpp CalcC.cpp envC.cpp
chaining_LDADD = $(SOAP_CPP_LIB) $(SAMPLE_EXTRA_LIBS) -lm
