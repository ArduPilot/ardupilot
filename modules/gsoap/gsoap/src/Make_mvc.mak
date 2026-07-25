CC=cl
CPP=cl
LIBS=

COFLAGS=-O2
# COFLAGS=-ZI -DDEBUG

CFLAGS= -EHsc -nologo $(COFLAGS)

soapcpp2.exe:	soapcpp2.h soapcpp2_yacc.obj symbol2.obj error2.obj lex.yy.obj init2.obj soapcpp2.obj
		$(CC) $(CFLAGS) symbol2.obj error2.obj soapcpp2_yacc.obj lex.yy.obj \
			init2.obj soapcpp2.obj $(LIBS) -Fe$@
.c.o:		# soapcpp2.h soapcpp2_yacc.h error2.h
		$(CC) $(CFLAGS) -DWITH_BISON -DWITH_FLEX -c $<
clean:
		del /f *.obj
distclean:
		del /f soapcpp2.exe *.obj *.idb *.pdb *.ilk
real-distclean:
		del /f soapcpp2.exe *.obj *.idb *.pdb *.ilk lex.yy.* soapcpp2_yacc.h \
			soapcpp2_yacc.c y.tab.* *.output

soapcpp2_yacc.c:
		soapcpp2_yacc.y soapcpp2.h error2.h
		bison.exe /d /v soapcpp2_yacc.y
		ren soapcpp2_yacc.tab.h soapcpp2_yacc.h
		ren soapcpp2_yacc.tab.c soapcpp2_yacc.c
lex.yy.c:	soapcpp2_yacc.h soapcpp2_yacc.c soapcpp2_lex.l
		flex.exe /l soapcpp2_lex.l
