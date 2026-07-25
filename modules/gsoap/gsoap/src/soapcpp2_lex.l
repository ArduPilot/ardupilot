/*
        soapcpp2_lex.l

        Flex/Lex tokenizer for C/C++

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2000-2015, Robert van Engelen, Genivia Inc. All Rights Reserved.
This part of the software is released under ONE of the following licenses:
GPL.
--------------------------------------------------------------------------------
GPL license.

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place, Suite 330, Boston, MA 02111-1307 USA

Author contact information:
engelen@genivia.com / engelen@acm.org

This program is released under the GPL with the additional exemption that
compiling, linking, and/or using OpenSSL is allowed.
--------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

%{
#include "soapcpp2.h"

#ifndef YY_NO_UNISTD_H
#define YY_NO_UNISTD_H
#endif

#ifdef HAVE_CONFIG_H
#include "soapcpp2_yacc.h"
#else
#include "soapcpp2_yacc.tab.h"
#endif

extern int isatty(int); /* silence warnings */

#ifdef WITH_BISON
extern YYSTYPE yylval;
#undef YY_NEED_STRLEN
#define YY_INPUT(buf, result, max_size) \
        { \
        int c = getc(yyin); \
        result = (c == EOF) ? YY_NULL : (buf[0] = c, 1); \
        }
#endif

#ifndef WITH_LEX
#define MAX_IMPORT_DEPTH 16
static struct importlist { struct importlist *next; char name[1]; } *importlist = NULL;
static const char *fnstk[MAX_IMPORT_DEPTH];
static int lnstk[MAX_IMPORT_DEPTH];
static const char *imstk[MAX_IMPORT_DEPTH];
static YY_BUFFER_STATE instk[MAX_IMPORT_DEPTH];
#endif

int imports = 0;
const char *imported = NULL;

static void check_id(const char*);
static Token install_id(void);
static Token install_tag(void);
static Token install_int(void);
static Token install_hex(void);
static Token install_num(void);
static Token install_chr(void);
static Token install_str(void);
static Token install_pragma(void);
static void directive(void), option(void), xpath(void);
static Token error_chr(void);
static Token error_str(void);
static int convchar(int*);
static int hexchar(int*);
static int octchar(int*);
static void module(const char *name, const char *fullname);
static void import(const char *file);
static long magic(const char *name);
%}
BOM             \xEF\xBB\xBF
ws              [ \t\v\r\f\n\x1A\xA0]
digit           [0-9]
alpha           ([a-zA-Z_\xA8\xAA\xAD\xAF\xB2\xB5\xB7\xB8\xB9\xBA\xBC\xBD\xBE]|[\xC0-\xFF][\x80-\xBF]*|\\u([0-9a-fA-F]{4}))
id              (:?{alpha})(:?{alpha}|{digit}|::)*
tag             `[^`\t\v\r\f\n>]+`
int             {digit}+[lLuU]*
hex             0[xX][0-9a-fA-F]+[lLuU]*
num             {digit}+(\.{digit}+([Ee][+-]?{digit}+)?|(\.{digit}+)?[Ee][+-]?{digit}+)[fFlL]?|{digit}+[fF]
chr             '(\\'|[^'\n])*'
str             L?\"(\\\"|\\\n|[^"])*\"
module          #{ws}*module{ws}+.*\n
import          #{ws}*import{ws}+.*\n
%option noyywrap
%option noinput
%option nounput
%x MLCOMMENT
%%
^{BOM}                  { /* skip UTF8 BOM */ }
{ws}                    { /* skip white space */ }
"/*"                    { BEGIN(MLCOMMENT); }
<MLCOMMENT>.|\n         { }
<MLCOMMENT>"*/"         { BEGIN(INITIAL); }
<MLCOMMENT><<EOF>>      { execerror("Unclosed multiline comment at end of file"); }
"//"\/*"gsoapopt".*\n   { option(); }
"//"\/*"gsoap"(.|\\\n)*\n { directive(); }
"//"\/*"xpath".*\n      { xpath(); }
"//".*\n                { /* skip single line comment */ }
"+="                    { return PA; }
"-="                    { return NA; }
"*="                    { return TA; }
"/="                    { return DA; }
"%="                    { return MA; }
"&="                    { return AA; }
"^="                    { return XA; }
"|="                    { return OA; }
"<<="                   { return LA; }
">>="                   { return RA; }
"||"                    { return OR; }
"&&"                    { return AN; }
"=="                    { return EQ; }
"!="                    { return NE; }
"<="                    { return LE; }
">="                    { return GE; }
"<<"                    { return LS; }
">>"                    { return RS; }
"++"                    { return PP; }
"--"                    { return NN; }
"->"                    { return AR; }
[;,:=|^&<>+\-*/%!?~(){}\[\].@$] { return yytext[0]; }
{id}                    { return install_id(); }
{tag}                   { return install_tag(); }
{int}                   { return install_int(); }
{hex}                   { return install_hex(); }
{num}                   { return install_num(); }
{chr}                   { return install_chr(); }
{str}                   { return install_str(); }
{module}                { char *s, *t, buf[1024];
                          s = strchr(yytext, '"');
                          if (!s)
                            t = yytext+7;
                          else
                          { strcpy(buf, s+1);
                            s = strchr(buf, '"');
                            *s = '\0';
                            t = strchr(s+1, '"');
                            if (t)
                            { t++;
                              s = strchr(t+1, '"');
                              if (s)
                                *s = '\0';
                            }
                          }
                          module(buf, t);
                        }
{import}                { char *s, buf[1024];
                          s = strchr(yytext, '"');
                          if (s)
                          { strncpy(buf, s + 1, sizeof(buf));
                            buf[sizeof(buf) - 1] = '\0';
                            s = strchr(buf, '"');
                            if (!s)
                              lexerror("Invalid import directive: missing ending \" or path too long");
                            else
                            { *s = '\0';
                              import(buf);
                            }
                          }
                          else
                            lexerror("Invalid import directive: \" expected");
                        }
#.*\n                   { return install_pragma(); }
'[^'\n]*/\n             { return error_chr(); }
\"[^"\n]*/\n            { return error_str(); }
.                       { lexerror("Skipping unknown symbol"); }
<<EOF>>                 { /* when Lex complains: remove this line and below */
#ifndef WITH_LEX
                          if (--imports < 0)
                          {
                            yyterminate();
                          }
                          else
                          { if (vflag)
                              fprintf(stderr, "End of %s\n\n", filename);
                            fclose(yyin);
                            yy_delete_buffer(YY_CURRENT_BUFFER);
                            yy_switch_to_buffer(instk[imports]);
                            filename = fnstk[imports];
                            yylineno = lnstk[imports];
                            imported = imstk[imports];
                          }
#endif
                        }
%%

/*
        install_id - lookup identifier in symbol table. If found, return token
        and symbol table entry. If not found, create entry in symbol table and
        return ID token.
*/
static Token
install_id()
{       Symbol *p = lookup(yytext);
        if (!p)
        {       int i, flag = 0;
                p = install(yytext, ID);
                /* [_](x|X)(m|M)(l|L)__ is OK */
                /* [_](x|X)(m|M)(l|L).* is not OK */
                /* .*__(x|X)(m|M)(l|L) is not OK */
                /* .*::(x|X)(m|M)(l|L) is not OK */
                for (i = 0; i < (int)yyleng; i++)
                {       if ((yytext[i] == '_' && yytext[i+1] == '_')
                         || (yytext[i] == ':' && yytext[i+1] == ':'))
                        {       flag = 1;
                                check_id(yytext + i + 2);
                        }
                }
                if (!flag && strcmp(yytext, "XML"))
                        check_id(yytext);
        }
        yylval.sym = p;
        return p->token;
}

/*
        install_tag -
        return TAG token.
*/
static Token
install_tag()
{       yylval.s = (char*)emalloc(yyleng);    /* yyleng = length(yytext) */
        strcpy(yylval.s, yytext+1);
        yylval.s[yyleng-2] = '\0';
        return TAG;
}

/*
        check_id - check for (x|X)(m|M)(l|L)
*/
static void
check_id(const char *s)
{       while (*s == '_')
                s++;
        if ((s[0] == 'x' || s[0] == 'X')
         && (s[1] == 'm' || s[1] == 'M')
         && (s[2] == 'l' || s[2] == 'L'))
        {       sprintf(errbuf, "identifier '%s' starts with or embeds '%3.3s' character sequence, which is exclusively reserved for and by the W3C XML standards (for enum constants: please ignore this warning)", yytext, s);
                semwarn(errbuf);
        }
}

/*
        install_int - convert digits to integer and return LNG token.
*/
static Token
install_int(void)
{
        sscanf(yytext, SOAP_ULONG_FORMAT, &yylval.i);
        return LNG;
}

/*
        install_hex - convert hexadecimal digits to integer and return LNG
*/
static Token
install_hex(void)
{
        sscanf(yytext, SOAP_XLONG_FORMAT, &yylval.i);
        return LNG;
}

/*
        install_num - convert digits to floating point number and return DBL
*/
static Token
install_num(void)
{       sscanf(yytext, "%lf", &yylval.r);
        return DBL;
}

/*
        install_chr - convert character constant and return CHR.
*/
static Token
install_chr(void)
{       int i = 2;
        if (yytext[1] == '\\')
                yylval.c = convchar(&i);
        else    yylval.c = yytext[i-1];
        if (yytext[i] != '\'')
                lexerror("Illegal character constant");
        return CHR;
}

/*
        install_str - convert and store string in memory. Return STR.
*/
static Token
install_str(void)
{       int i, j = 0;
        yylval.s = (char*)emalloc(yyleng - 1);  /* yyleng = length(yytext) */
        for (i = 1 + (yytext[0] == 'L'); i < (int)yyleng - 1; i++)
                if (yytext[i] == '\\')
                {       if (yytext[++i] != '\n')
                        {       yylval.s[j++] = convchar(&i);
                                i--;
                        }
                }
                else
                        yylval.s[j++] = yytext[i];
        yylval.s[j] = '\0';
        return STR;
}

/*
        install_pragma - store pragma in string. Return PRAGMA.
*/
static Token
install_pragma(void)
{       yylval.s = (char*)emalloc(yyleng);      /* yyleng = length(yytext) */
        strncpy(yylval.s, yytext, strlen(yytext)-1);
        yylval.s[strlen(yytext)-1] = '\0';
        return PRAGMA;
}

static void directive(void)
{       int i, j, k;
        char *s;
        Service *sp;
        Method *m;
        Data *d;
        int service;
        for (i = 0; yytext[i]; i++)
          if (yytext[i] <= 32)
            break;
        for (; yytext[i]; i++)
          if (yytext[i] > 32 && yytext[i] != '\\')
              break;
        if (!strncmp(yytext+i, "schema", 6))
        { sprintf(errbuf, "gsoap schema directive without namespace: %s", yytext+i);
          semwarn(errbuf);
          s = (char*)emalloc(1);
          *s = '\0';
        }
        else if (!strncmp(yytext+i, "service", 7))
        {
          s = (char*)emalloc(1);
          *s = '\0';
        }
        else
        {
          for (j = i; yytext[j]; j++)
            if (yytext[j] <= 32)
              break;
          if (i == j)
            return;
          s = (char*)emalloc(j-i+1);
          for (k = 0; k < j-i; k++)
          { s[k] = yytext[k+i];
            if (s[k] == '_')
              s[k] = '-';
          }
          s[k] = '\0';
          for (i = j; yytext[i]; i++)
            if (yytext[i] > 32 && yytext[i] != '\\')
              break;
        }
        for (sp = services; sp; sp = sp->next)
                if (!strcmp(sp->ns, s))
                        break;
        if (!sp)
        {       sp = (Service*)emalloc(sizeof(Service));
                sp->next = services;
                sp->ns = s;
                sp->name = NULL;
                sp->porttype = NULL;
                sp->portname = NULL;
                sp->binding = NULL;
                sp->definitions = NULL;
                sp->transport = NULL;
                sp->URL = NULL;
                sp->URI = NULL;
                sp->URI2 = NULL;
                sp->WSDL = NULL;
                sp->style = NULL;
                sp->encoding = NULL;
                sp->protocol = NULL;
                sp->xsi_type = 0;
                sp->elementForm = NULL;
                sp->attributeForm = NULL;
                sp->executable = NULL;
                sp->import = NULL;
                sp->documentation = NULL;
                sp->list = NULL;
                sp->data = NULL;
                services = sp;
        }
        if (!strncmp(yytext+i, "service", 7) || !strncmp(yytext+i, "schema", 6))
        {       service = strncmp(yytext+i, "schema", 6);
                for (i += 7; yytext[i]; i++)
                        if (yytext[i] > 32 && yytext[i] != '\\')
                                break;
                for (j = i; yytext[j]; j++)
                        if (yytext[j] <= 32)
                                break;
                for (; yytext[j]; j++)
                        if (yytext[j] > 32 && yytext[j] != '\\')
                                break;
                for (k = j; yytext[k]; k++)
                        if (yytext[k] <= 32)
                                break;
                /*
                if (j == k)
                        return;
                */
                s = (char*)emalloc(k-j+1);
                strncpy(s, yytext+j, k-j);
                s[k-j] = '\0';
                m = NULL;
                if (!strncmp(yytext+i, "name:", 5))
                {       sp->name = s;
                        for (j = k; yytext[j]; j++)
                                if (yytext[j] > 32 && yytext[j] != '\\')
                                        break;
                        for (k = j; yytext[k]; k++)
                                if (yytext[k] == '\\')
					k++;
				else if (yytext[k] == 10 || yytext[k] == 13)
                                        break;
                        if (j == k)
                                return;
                        s = (char*)emalloc(k-j+1);
                        strncpy(s, yytext+j, k-j);
                        s[k-j] = '\0';
                        sp->documentation = s;
                }
                else if (service && (!strncmp(yytext+i, "type:", 5) || !strncmp(yytext+i, "portType:", 9) || !strncmp(yytext+i, "interface:", 10)))
                        sp->porttype = s;
                else if (!strncmp(yytext+i, "portName:", 9))
                        sp->portname = s;
                else if (!strncmp(yytext+i, "binding:", 8))
                        sp->binding = s;
                else if (!strncmp(yytext+i, "definitions:", 12))
                        sp->definitions = s;
                else if (!strncmp(yytext+i, "documentation:", 14) || !strncmp(yytext+i, "doc:", 4))
                {       for (k = j; yytext[k]; k++)
                                if (yytext[k] == '\\')
					k++;
				else if (yytext[k] == 10 || yytext[k] == 13)
                                        break;
                        if (j == k)
                                return;
                        s = (char*)emalloc(k-j+1);
                        strncpy(s, yytext+j, k-j);
                        s[k-j] = '\0';
                        sp->documentation = s;
                }
                else if (!strncmp(yytext+i, "transport:", 10))
                        sp->transport = s;
                else if (!strncmp(yytext+i, "location:", 9) || !strncmp(yytext+i, "endpoint:", 9) || !strncmp(yytext+i, "port:", 5))
                {       if (sp->URL)
                        {       char *t = (char*)emalloc(strlen(sp->URL) + strlen(s) + 2);
                                strcpy(t, sp->URL);
                                strcat(t, " ");
                                strcat(t, s);
                                sp->URL = t;
                        }
                        else
                                sp->URL = s;
                        if (!service && !sp->import)
                                sp->import = s;
                }
                else if (!strncmp(yytext+i, "executable:", 11))
                        sp->executable = s;
                else if (!strncmp(yytext+i, "namespace:", 10))
                {       if (service)
                        {       if (!sp->URI)
                                        sp->URI = s;
                                sp->WSDL = s;
                        }
                        else if (!strcmp(sp->ns, "SOAP-ENV"))
                        {       if (soap_version > 0)
                                        semwarn("option -1 or -2 overrides SOAP-ENV namespace");
                                else
                                        envURI = s;
                                sp->URI = (char*)envURI;
                        }
                        else if (!strcmp(sp->ns, "SOAP-ENC"))
                        {       if (soap_version > 0)
                                        semwarn("option -1 or -2 overrides SOAP-ENC namespace");
                                else
                                        encURI = s;
                                sp->URI = (char*)encURI;
                        }
                        else
                                sp->URI = s;
                }
                else if (!strncmp(yytext+i, "namespace2:", 11))
                {       sp->URI2 = s;
                }
                else if (!strncmp(yytext+i, "typed:", 6))
                {       sp->xsi_type = (*s == 'y');
                }
                else if (!strncmp(yytext+i, "form:", 5))
                {       sp->elementForm = s;
                        sp->attributeForm = s;
                }
                else if (!strncmp(yytext+i, "elementForm:", 12))
                        sp->elementForm = s;
                else if (!strncmp(yytext+i, "attributeForm:", 14))
                        sp->attributeForm = s;
                else if (!strncmp(yytext+i, "import:", 7))
                {       if (!sp->URI)
                                sp->URI = s;
                        sp->import = s;
                }
                else if (!strncmp(yytext+i, "encoding:", 9))
                {       if (!strcmp(s, "encoded"))
                                sp->encoding = "";
                        else
                                sp->encoding = s;
                }
                else if (!strncmp(yytext+i, "style:", 6))
                        sp->style = s;
                else if (!strncmp(yytext+i, "protocol:", 9))
                        sp->protocol = s;
                else if (!strncmp(yytext+i, "method-protocol:", 16))
                {       m = (Method*)emalloc(sizeof(Method));
                        m->name = s;
                        m->mess = PROTOCOL;
                        m->part = NULL;
                        for (j = k; yytext[j]; j++)
                                if (yytext[j] > 32 && yytext[j] != '\\')
                                        break;
                        for (k = j; yytext[k]; k++)
                                if (yytext[k] == '\\')
					k++;
				else if (yytext[k] == 10 || yytext[k] == 13)
                                        break;
                        if (j == k)
                                return;
                        s = (char*)emalloc(k-j+1);
                        strncpy(s, yytext+j, k-j);
                        s[k-j] = '\0';
                        m->part = s;
                }
                else if (!strncmp(yytext+i, "method-style:", 13))
                {       m = (Method*)emalloc(sizeof(Method));
                        m->name = s;
                        m->mess = STYLE;
                        m->part = NULL;
                        for (j = k; yytext[j]; j++)
                                if (yytext[j] > 32 && yytext[j] != '\\')
                                        break;
                        for (k = j; yytext[k]; k++)
                                if (yytext[k] == '\\')
					k++;
				else if (yytext[k] == 10 || yytext[k] == 13)
                                        break;
                        if (j == k)
                                return;
                        s = (char*)emalloc(k-j+1);
                        strncpy(s, yytext+j, k-j);
                        s[k-j] = '\0';
                        m->part = s;
                }
                else if (!strncmp(yytext+i, "method-encoding:", 16))
                {       m = (Method*)emalloc(sizeof(Method));
                        m->name = s;
                        m->mess = ENCODING;
                        m->part = NULL;
                        for (j = k; yytext[j]; j++)
                                if (yytext[j] > 32 && yytext[j] != '\\')
                                        break;
                        for (k = j; yytext[k]; k++)
                                if (yytext[k] == '\\')
					k++;
				else if (yytext[k] == 10 || yytext[k] == 13)
                                        break;
                        if (j == k)
                                return;
                        s = (char*)emalloc(k-j+1);
                        strncpy(s, yytext+j, k-j);
                        s[k-j] = '\0';
                        if (strcmp(s, "encoded"))
                                m->part = s;
                        else
                                m->part = "";
                }
                else if (!strncmp(yytext+i, "method-response-encoding:", 25))
                {       m = (Method*)emalloc(sizeof(Method));
                        m->name = s;
                        m->mess = RESPONSE_ENCODING;
                        m->part = NULL;
                        for (j = k; yytext[j]; j++)
                                if (yytext[j] > 32 && yytext[j] != '\\')
                                        break;
                        for (k = j; yytext[k]; k++)
                                if (yytext[k] == '\\')
					k++;
				else if (yytext[k] == 10 || yytext[k] == 13)
                                        break;
                        if (j == k)
                                return;
                        s = (char*)emalloc(k-j+1);
                        strncpy(s, yytext+j, k-j);
                        s[k-j] = '\0';
                        if (strcmp(s, "encoded"))
                                m->part = s;
                        else
                                m->part = "";
                }
                else if (!strncmp(yytext+i, "method-documentation:", 21) || !strncmp(yytext+i, "method:", 7))
                {       d = (Data*)emalloc(sizeof(Data));
                        d->name = s;
                        d->text = NULL;
                        d->next = sp->data;
                        sp->data = d;
                        for (j = k; yytext[j]; j++)
                                if (yytext[j] > 32 && yytext[j] != '\\')
                                        break;
                        for (k = j; yytext[k]; k++)
                                if (yytext[k] == '\\')
					k++;
				else if (yytext[k] == 10 || yytext[k] == 13)
                                        break;
                        if (j == k)
                                return;
                        s = (char*)emalloc(k-j+1);
                        strncpy(s, yytext+j, k-j);
                        s[k-j] = '\0';
                        d->text = s;
                }
                else if (!strncmp(yytext+i, "method-action:", 14))
                {       m = (Method*)emalloc(sizeof(Method));
                        m->name = s;
                        m->mess = ACTION;
                        m->part = NULL;
                        for (j = k; yytext[j]; j++)
                                if (yytext[j] > 32 && yytext[j] != '\\')
                                        break;
                        for (k = j; yytext[k]; k++)
                                if (yytext[k] == '\\')
					k++;
				else if (yytext[k] == 10 || yytext[k] == 13)
                                        break;
                        if (j == k)
                                return;
                        s = (char*)emalloc(k-j+1);
                        strncpy(s, yytext+j, k-j);
                        s[k-j] = '\0';
                        m->part = s;
                }
                else if (!strncmp(yytext+i, "method-input-action:", 20))
                {       m = (Method*)emalloc(sizeof(Method));
                        m->name = s;
                        m->mess = REQUEST_ACTION;
                        m->part = NULL;
                        for (j = k; yytext[j]; j++)
                                if (yytext[j] > 32 && yytext[j] != '\\')
                                        break;
                        for (k = j; yytext[k]; k++)
                                if (yytext[k] == '\\')
					k++;
				else if (yytext[k] == 10 || yytext[k] == 13)
                                        break;
                        if (j == k)
                                return;
                        s = (char*)emalloc(k-j+1);
                        strncpy(s, yytext+j, k-j);
                        s[k-j] = '\0';
                        m->part = s;
                }
                else if (!strncmp(yytext+i, "method-output-action:", 21))
                {       m = (Method*)emalloc(sizeof(Method));
                        m->name = s;
                        m->mess = RESPONSE_ACTION;
                        m->part = NULL;
                        for (j = k; yytext[j]; j++)
                                if (yytext[j] > 32 && yytext[j] != '\\')
                                        break;
                        for (k = j; yytext[k]; k++)
                                if (yytext[k] == '\\')
					k++;
				else if (yytext[k] == 10 || yytext[k] == 13)
                                        break;
                        if (j == k)
                                return;
                        s = (char*)emalloc(k-j+1);
                        strncpy(s, yytext+j, k-j);
                        s[k-j] = '\0';
                        m->part = s;
                }
                else if (!strncmp(yytext+i, "method-fault-action:", 20))
                {       m = (Method*)emalloc(sizeof(Method));
                        m->name = s;
                        m->mess = FAULT_ACTION;
                        m->part = NULL;
                        for (j = k; yytext[j]; j++)
                                if (yytext[j] > 32 && yytext[j] != '\\')
                                        break;
                        for (k = j; yytext[k]; k++)
                                if (yytext[k] == '\\')
					k++;
				else if (yytext[k] == 10 || yytext[k] == 13)
                                        break;
                        if (j == k)
                                return;
                        s = (char*)emalloc(k-j+1);
                        strncpy(s, yytext+j, k-j);
                        s[k-j] = '\0';
                        m->part = s;
                }
                else if (!strncmp(yytext+i, "method-mime-type:", 17))
                {       m = (Method*)emalloc(sizeof(Method));
                        m->name = s;
                        m->mess = MIMEIN | MIMEOUT;
                        m->part = NULL;
                        for (j = k; yytext[j]; j++)
                                if (yytext[j] > 32 && yytext[j] != '\\')
                                        break;
                        for (k = j; yytext[k]; k++)
                                if (yytext[k] <= 32)
                                        break;
                        if (j == k)
                                return;
                        s = (char*)emalloc(k-j+1);
                        strncpy(s, yytext+j, k-j);
                        s[k-j] = '\0';
                        m->part = s;
                }
                else if (!strncmp(yytext+i, "method-input-mime-type:", 23))
                {       m = (Method*)emalloc(sizeof(Method));
                        m->name = s;
                        m->mess = MIMEIN;
                        m->part = NULL;
                        for (j = k; yytext[j]; j++)
                                if (yytext[j] > 32 && yytext[j] != '\\')
                                        break;
                        for (k = j; yytext[k]; k++)
                                if (yytext[k] <= 32)
                                        break;
                        if (j == k)
                                return;
                        s = (char*)emalloc(k-j+1);
                        strncpy(s, yytext+j, k-j);
                        s[k-j] = '\0';
                        m->part = s;
                }
                else if (!strncmp(yytext+i, "method-output-mime-type:", 24))
                {       m = (Method*)emalloc(sizeof(Method));
                        m->name = s;
                        m->mess = MIMEOUT;
                        m->part = NULL;
                        for (j = k; yytext[j]; j++)
                                if (yytext[j] > 32 && yytext[j] != '\\')
                                        break;
                        for (k = j; yytext[k]; k++)
                                if (yytext[k] <= 32)
                                        break;
                        if (j == k)
                                return;
                        s = (char*)emalloc(k-j+1);
                        strncpy(s, yytext+j, k-j);
                        s[k-j] = '\0';
                        m->part = s;
                }
                else if (!strncmp(yytext+i, "method-header-part:", 19))
                {       m = (Method*)emalloc(sizeof(Method));
                        m->name = s;
                        m->mess = HDRIN | HDROUT;
                        m->part = NULL;
                        for (j = k; yytext[j]; j++)
                                if (yytext[j] > 32 && yytext[j] != '\\')
                                        break;
                        for (k = j; yytext[k]; k++)
                                if (yytext[k] <= 32)
                                        break;
                        if (j == k)
                                return;
                        s = (char*)emalloc(k-j+1);
                        strncpy(s, yytext+j, k-j);
                        s[k-j] = '\0';
                        m->part = s;
                }
                else if (!strncmp(yytext+i, "method-input-header-part:", 25))
                {       m = (Method*)emalloc(sizeof(Method));
                        m->name = s;
                        m->mess = HDRIN;
                        m->part = NULL;
                        for (j = k; yytext[j]; j++)
                                if (yytext[j] > 32 && yytext[j] != '\\')
                                        break;
                        for (k = j; yytext[k]; k++)
                                if (yytext[k] <= 32)
                                        break;
                        if (j == k)
                                return;
                        s = (char*)emalloc(k-j+1);
                        strncpy(s, yytext+j, k-j);
                        s[k-j] = '\0';
                        m->part = s;
                }
                else if (!strncmp(yytext+i, "method-output-header-part:", 26))
                {       m = (Method*)emalloc(sizeof(Method));
                        m->name = s;
                        m->mess = HDROUT;
                        m->part = NULL;
                        for (j = k; yytext[j]; j++)
                                if (yytext[j] > 32 && yytext[j] != '\\')
                                        break;
                        for (k = j; yytext[k]; k++)
                                if (yytext[k] <= 32)
                                        break;
                        if (j == k)
                                return;
                        s = (char*)emalloc(k-j+1);
                        strncpy(s, yytext+j, k-j);
                        s[k-j] = '\0';
                        m->part = s;
                }
                else if (!strncmp(yytext+i, "method-fault:", 13))
                {       m = (Method*)emalloc(sizeof(Method));
                        m->name = s;
                        m->mess = FAULT;
                        m->part = NULL;
                        for (j = k; yytext[j]; j++)
                                if (yytext[j] > 32 && yytext[j] != '\\')
                                        break;
                        for (k = j; yytext[k]; k++)
                                if (yytext[k] <= 32)
                                        break;
                        if (j == k)
                                return;
                        s = (char*)emalloc(k-j+1);
                        strncpy(s, yytext+j, k-j);
                        s[k-j] = '\0';
                        m->part = s;
                }
                else if (!service && (!strncmp(yytext+i, "type-documentation:", 19) || !strncmp(yytext+i, "type:", 5)))
                {       d = (Data*)emalloc(sizeof(Data));
                        d->name = s;
                        d->text = NULL;
                        d->next = sp->data;
                        sp->data = d;
                        for (j = k; yytext[j]; j++)
                                if (yytext[j] > 32 && yytext[j] != '\\')
                                        break;
                        for (k = j; yytext[k]; k++)
                                if (yytext[k] == '\\')
					k++;
				else if (yytext[k] == 10 || yytext[k] == 13)
                                        break;
                        if (j == k)
                                return;
                        s = (char*)emalloc(k-j+1);
                        strncpy(s, yytext+j, k-j);
                        s[k-j] = '\0';
                        d->text = s;
                }
                else
                {       sprintf(errbuf, "unrecognized gsoap directive: %s", yytext+i);
                        semwarn(errbuf);
                }
                if (m)
                {
                  m->next = NULL;
                  if (!sp->list)
                    sp->list = m;
                  else
                  {
                    Method *p;
                    for (p = sp->list; p->next; p = p->next)
                      continue;
                    p->next = m;
                  }
                }
        }
        else
        {       sprintf(errbuf, "unrecognized gsoap directive: %s", yytext);
                semwarn(errbuf);
        }
}

static void option(void)
{       int i;
        if (imports)
        {       sprintf(errbuf, "options directive: %s ignored in imported file(s)", yytext);
                semwarn(errbuf);
                return;
        }
        for (i = 0; yytext[i]; i++)
                if (yytext[i] <= 32)
                        break;
        for (; yytext[i]; i++)
                if (yytext[i] > 32 && yytext[i] != '\\')
                        break;
        for (; yytext[i]; i++)
                switch (yytext[i])
                {
                        case 'a':
                                aflag = 1;
                                break;
                        case 'A':
                                Aflag = 1;
                                break;
                        case 'c':
                                if (yytext[i+1] == '+' && yytext[i+2] == '+')
                                {
                                        i += 2;
                                        if (yytext[i+1] == '1' && yytext[i+2] == '1')
                                        {
                                                i += 2;
                                                c11flag = 1;
                                        }
                                        cflag = 0;
                                }
                                else
                                {
                                        cflag = 1;
                                }
                                break;
                        case 'e':
                                eflag = 1;
                                break;
                        case 'i':
                                iflag = 1;
                                break;
                        case 'j':
                                jflag = 1;
                                break;
                        case 'n':
                                nflag = 1;
                                break;
                        case 's':
                                sflag = 1;
                                break;
                        case 't':
                                tflag = 1;
                                break;
                        case 'w':
                                wflag = 1;
                                break;
                        case 'x':
                                xflag = 1;
                                break;
                        default:
                                if (yytext[i] < 32 || yytext[i] == '/')
                                        return;
                }
}

static void xpath(void)
{       int i, j, k;
        char *s;
        XPath *xp;
        for (i = 0; yytext[i]; i++)
                if (yytext[i] <= 32)
                        break;
        for (; yytext[i]; i++)
                if (yytext[i] > 32 && yytext[i] != '\\')
                        break;
        for (j = i; yytext[j]; j++)
                if (yytext[j] <= 32)
                        break;
        if (i == j)
                return;
        for (k = j; yytext[k]; k++)
                if (yytext[k] > 32 && yytext[k] != '\\')
                        break;
        s = (char*)emalloc(yyleng - i);
        strncpy(s, yytext + i, j - i);
        s[j - i] = '\0';
        for (xp = xpaths; xp; xp = xp->next)
                if (!strcmp(xp->name, s))
                        break;
        if (!xp)
        {       xp = (XPath*)emalloc(sizeof(XPath));
                xp->next = xpaths;
                xp->name = s;
        }
        s += j - i + 1;
        strcpy(s, yytext + k);
        xp->path = s;
        while (*s > 32)
          s++;
        *s = '\0';
}

/*
        error_chr - lexical error in character constant. Return character 0 to
        allow parsing to continue
*/
static Token
error_chr(void)
{       lexerror("Ending-' missing in character constant");
        yylval.c = '\0';
        return CHR;
}

/*
        error_str - lexical error in string. Return empty string to allow
        parsing to continue
*/
static Token
error_str(void)
{       lexerror("Ending-\" missing in string");
        yylval.s = "";
        return STR;
}

/*
        Character conversion functions
*/
static int
convchar(int *p)
{       switch (yytext[(*p)++])
        {       case 'a':       return '\a';
                case 'b':       return '\b';
                case 'f':       return '\f';
                case 'n':       return '\n';
                case 'r':       return '\r';
                case 't':       return '\t';
                case 'v':       return '\v';
                case 'x':       return hexchar(p);
                case '0':
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':       (*p)--;
                                return octchar(p);
                default:        return yytext[*p-1];
        }
}

static int
hexchar(int *p)
{       int i, d, c = 0;
        for (i = 0; isxdigit(d = yytext[*p]) && i < 2; i++)
        {       c = (c << 4) + (d <= '9' ? d - '0' : toupper(d) - '7');
                (*p)++;
        }
        return c;
}

static int
octchar(int *p)
{       int i, d, c = 0;
        for (i = 0; (d = yytext[*p]) >= '0' && d <= '7' && i < 3; i++)
        {       c = (c << 3) + d - '0';
                (*p)++;
        }
        return c;
}

static void module(const char *name, const char *fullname)
{ char *s;
  if (!fullname)
    fullname = name;
  if (imports)
  { Pragma **pp;
    s = (char*)emalloc(strlen(fullname)+15);
    sprintf(s, "#include \"%sH.h\"", fullname);
    for (pp = &pragmas; *pp; pp = &(*pp)->next)
      if (!strcmp((*pp)->pragma, s))
        break;
    if (!*pp)
    { *pp = (Pragma*)emalloc(sizeof(Pragma));
      (*pp)->pragma = s;
      (*pp)->next = NULL;
    }
    s = (char*)emalloc(strlen(fullname)+1);
    imported = strcpy(s, fullname);
    fprintf(stderr, "Importing module '%s'\n\n", fullname);
  }
  else
  { lflag = 1;
    typeNO = magic(name);
    s = (char*)emalloc(strlen(fullname)+1);
    prefix = strcpy(s, fullname);
    fprintf(stderr, "Compiling module '%s' (magic number = %d)\n\n", fullname, typeNO);
  }
}

static long magic(const char *name)
{ size_t i;
  long n;
  if (strlen(name) > 4)
    semerror("#module name length must not exceed four characters");
  n = 0;
  for (i = 0; i < strlen(name); i++)
    if (name[i] >= 'a' && name[i] <= 'z')
      n = 26*n + name[i] - 'a';
    else if (name[i] >= 'A' && name[i] <= 'Z')
      n = 26*n + name[i] - 'A';
    else
      semerror("#module name must be alphabetic and the length must not exceed four characters.\nUse '#module name longname' for longer names.");
  return 4699*n + 153424;
}

#ifdef WITH_LEX
static void import(const char *file)
{ execerror("Cannot #import: soapcpp2 not compiled with flex (replace lex with flex)");
}
#else
static void import(const char *file)
{ char buf[1024];
  struct importlist *p;
  if (vflag)
    fprintf(stderr, "Importing '%s': ", file);
  for (p = importlist; p; p = p->next)
  { if (!strcmp(p->name, file))
    { if (vflag)
        fprintf(stderr, "file already read or currently being read\n\n");
      return;
    }
  }
  if (imports >= MAX_IMPORT_DEPTH)
    execerror("Imports nested too deep");
  instk[imports] = YY_CURRENT_BUFFER;
  fnstk[imports] = filename;
  lnstk[imports] = yylineno;
  imstk[imports] = imported;
  yylineno = 1;
  /* imported = NULL; this is useful to change the semantics of #import to NOT consider non-module imports to be part of the current module */
  imports++;
  buf[0] = '\0';
  if (!(yyin = fopen(file, "r")))
  { if (importpath)
    { const char *s, *t;
      s = importpath;
      do
      { size_t n;
        t = strstr(s, SOAP_PATHSEP);
        if (t)
        { if ((size_t)(t - s) >= sizeof(buf))
            t = s + sizeof(buf) - 1;
          strncpy(buf, s, t - s);
          buf[t - s] = '\0';
          s = t + sizeof(SOAP_PATHSEP) - 1;
        }
        else
        { strcpy(buf, s);
          s = NULL;
        }
        n = strlen(buf) - 1;
#ifdef __VMS
        if (buf[n] != ']' && buf[n] != ':')
          strcat(buf, ":");
#else
        if (buf[n] != SOAP_PATHCAT[0])
          strcat(buf, SOAP_PATHCAT);
#endif
        strcat(buf, file);
        yyin = fopen(buf, "r");
      }
      while (s && !yyin);
    }
    if (!yyin)
    { sprintf(errbuf, "Cannot open file \"%s\" to import: %s\nHint: use option -I<path> (for example -Igsoap" SOAP_PATHSEP "gsoap/import" SOAP_PATHSEP "gsoap/custom:.)", file, strerror(errno));
      execerror(errbuf);
    }
  }
  if (vflag)
  { if (buf[0])
      fprintf(stderr, "OK, found at %s\n\n", buf);
    else
      fprintf(stderr, "OK\n\n");
  }
  p = (struct importlist*)emalloc(sizeof(struct importlist) + strlen(file)); /* has already + 1 byte size */
  strcpy(p->name, file);
  filename = p->name;
  p->next = importlist;
  importlist = p;
  yy_switch_to_buffer(yy_create_buffer(yyin, YY_BUF_SIZE));
}
#endif
