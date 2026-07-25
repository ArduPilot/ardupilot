%{
#define YYSTYPE double
#include "calc.tab.h"
extern YYSTYPE yylval;
%}

%%
[0-9]+\.?[0-9]* { yylval = atof( yytext ); return NUMBER; };
[ \t] ;  /* ignore whitespace */
\n      { return yytext[0]; };
. 	{   return yytext[0]; }
<<EOF>> { printf("eof\n"); return 0; };
%%
