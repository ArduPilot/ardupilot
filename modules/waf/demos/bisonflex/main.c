#include <stdio.h>
/* this file should work in both c and c++ */
extern int yyparse();

int yyerror (char const *a)
{
	printf("yyerror: (%s)\n", a);
	return 1;
}

int main(int argc, char *argv[])
{
	int yy;
	yy = yyparse();
	if (yy != 0)
	{
		printf("Syntax or parse error %i. Aborting.\n", yy);
		return 1;
	}
	else{
		printf("Success.\n");
	}
	return 0;
}
