####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = router
SOAPHEADER = router.h

include ../Makefile.defines
include ../Makefile.c_rules
SOAP_NS = $(SOAP_NS_DEF)

router_SOURCES = router.c $(SOAPHEADER) $(SOAP_C_CORE)
router_LDADD = $(SOAP_C_LIB) $(SAMPLE_EXTRA_LIBS) -lpthread

