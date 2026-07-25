####### This is the input file for automake, which will generate Makefile.in ##########


## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = foreign 1.4


#LIBS=
AM_YFLAGS=-d -v
AM_LFLAGS=$(LEX_FLAGS)

BUILT_SOURCES=soapcpp2_yacc.h

bin_PROGRAMS=soapcpp2

soapcpp2_CFLAGS=$(BISON_DEFINE) $(LEX_DEFINE) $(C_DEBUG_FLAGS) $(SOAPCPP2_IMPORTPATH) -D$(platform)
soapcpp2_LDADD=$(YACC_LIB) $(LEXLIB)
soapcpp2_SOURCES=soapcpp2_SOURCES= soapcpp2_yacc.y soapcpp2_lex.l symbol2.c error2.c init2.c soapcpp2.c

CLEANFILES= *~ soapcpp2_lex.c soapcpp2_yacc.c soapcpp2_yacc.h soapcpp2_yacc.output
