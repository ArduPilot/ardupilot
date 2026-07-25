/*
        soapcpp2.h

        Common declarations.

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2000-2021, Robert van Engelen, Genivia Inc. All Rights Reserved.
This part of the software is released under one of the following licenses:
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

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <float.h>
#include <time.h>
#include "error2.h"

#ifndef VERSION
# define VERSION "2.8.114" /* Current version */
# define GSOAP_VERSION 208114
#endif

#ifdef _WIN32
# ifndef WIN32
#  define WIN32
# endif
#endif

/* for legacy purposes we use WIN32 macro, even when WIN64 is supported */
#ifdef _WIN64
# ifndef WIN32
#  define WIN32
# endif
#endif

#ifdef WIN32
# pragma warning(disable : 4996)
# ifndef WITH_BISON
#  define WITH_BISON
# endif
#endif

/* #define DEBUG */ /* uncomment to debug */

#ifdef DEBUG
# define        check(expr, msg) (void)((expr) ? 0 : (progerror(msg, __FILE__, __LINE__), 0))
# define DBGLOG(DBGCMD) { DBGCMD; }
#else
# define check(expr, msg) (void)(expr)
# define DBGLOG(DBGCMD)
#endif

#ifdef WIN32
# ifdef WITH_BISON
#  ifdef WIN32_WITHOUT_SOLARIS_FLEX
#   define yyparse soapcpp2parse
#   define yylex soapcpp2lex
#   define yyerror soapcpp2error
#   define yylval soapcpp2lval
#   define yychar soapcpp2char
#   define yydebug soapcpp2debug
#   define yynerrs soapcpp2nerrs
#   define yylineno soapcpp2lineno
#   define yytext soapcpp2text
#   define yyin soapcpp2in
#   define yywrap soapcpp2wrap
#  endif
# endif
#endif

#ifdef WIN32
# define SOAP_PATHCAT "\\"
# define SOAP_PATHSEP ";"
# define LONG64 __int64
#else
# define SOAP_PATHCAT "/"
# define SOAP_PATHSEP ":"
# define LONG64 long long
#endif

#ifndef ULONG64
# define ULONG64 unsigned LONG64
#endif

#if defined(WIN32)
# define SOAP_LONG_FORMAT "%I64d"
# define SOAP_ULONG_FORMAT "%I64u"
# define SOAP_XLONG_FORMAT "%I64x"
#elif defined(TRU64)
# define SOAP_LONG_FORMAT "%ld"
# define SOAP_ULONG_FORMAT "%lu"
# define SOAP_XLONG_FORMAT "%lx"
#endif

#ifndef SOAP_LONG_FORMAT
# define SOAP_LONG_FORMAT "%lld"        /* printf format for 64 bit ints */
#endif
#ifndef SOAP_ULONG_FORMAT
# define SOAP_ULONG_FORMAT "%llu"       /* printf format for unsigned 64 bit ints */
#endif
#ifndef SOAP_XLONG_FORMAT
# define SOAP_XLONG_FORMAT "%llx"       /* printf format for unsigned 64 bit hex ints */
#endif

extern int yylineno;

typedef enum Bool {False, True} Bool;

typedef int Token;

typedef enum Type
{
  Tnone,
  Tvoid,
  Tchar,          /* primitive types from here*/
  Twchar,
  Tshort,
  Tint,
  Tlong,
  Tllong,
  Tfloat,
  Tdouble,
  Tldouble,
  Tuchar,
  Tushort,
  Tuint,
  Tulong,
  Tullong,
  Tsize,
  Ttime,
  Tenum,
  Tenumsc,
  Tclass,         /* compound types from here */
  Tstruct,
  Tunion,
  Tpointer,       /* pointer T* */
  Treference,     /* reference T& */
  Trvalueref,     /* C+11 rvalue reference T&& */
  Tarray,         /* fixed size array T[N] */
  Ttemplate,      /* template class<T> */
  Tfun
} Type;

#define TYPES (Tfun+1)  /* number of type (operators) enumerated above */

typedef enum Storage
{
  Snone           = 0,
  Sauto           = 0x000001,
  Sregister       = 0x000002,
  Sstatic         = 0x000004,
  Sextern         = 0x000008,
  Stypedef        = 0x000010,
  Svirtual        = 0x000020,
  Sconst          = 0x000040,
  Sconstobj       = 0x000080,
  Sfinal          = 0x000100,
  Soverride       = 0x000200,
  Sconstptr       = 0x000400,
  Sfriend         = 0x000800,
  Sinline         = 0x001000,
  Sabstract       = 0x002000,
  SmustUnderstand = 0x004000,
  Sreturn         = 0x008000,
  Sattribute      = 0x010000,
  Sspecial        = 0x020000,
  Sexplicit       = 0x040000,
  Sprivate        = 0x080000,
  Sprotected      = 0x100000,
  Smutable        = 0x200000
} Storage;

typedef enum Level { INTERNAL, GLOBAL, PARAM, LOCAL } Level;

#define mknone()        mktype(Tnone,     NULL, 0)
#define mkvoid()        mktype(Tvoid,     NULL, 0)
#define mkbool()        mktype(Tenum,     booltable, 4)
#define mkchar()        mktype(Tchar,     NULL, 1)      /* int8_t */
#define mkwchart()      mktype(Twchar,    NULL, 4)      /* wchar_t */
#define mkshort()       mktype(Tshort,    NULL, 2)      /* int32_t */
#define mkint()         mktype(Tint,      NULL, 4)      /* int32_t */
#define mklong()        mktype(Tlong,     NULL, 8)      /* int32_t */
#define mkllong()       mktype(Tllong,    NULL, 8)      /* int64_t */
#define mkfloat()       mktype(Tfloat,    NULL, 4)
#define mkdouble()      mktype(Tdouble,   NULL, 8)
#define mkldouble()     mktype(Tldouble,  NULL, 16)     /* long double */
#define mkuchar()       mktype(Tuchar,    NULL, 1)      /* uint8_t unsigned char */
#define mkushort()      mktype(Tushort,   NULL, 2)      /* utin16_t unsigned short */
#define mkuint()        mktype(Tuint,     NULL, 4)      /* uint32_t unsigned int */
#define mkulong()       mktype(Tulong,    NULL, 8)      /* uint64_t unsigned long */
#define mkullong()      mktype(Tullong,   NULL, 8)      /* uint64_t unsigned long long */
#define mksize()        mktype(Tsize,     NULL, 8)      /* transient size_t */
#define mktimet()       mktype(Ttime,     NULL, 4)
#define mkenum(t)       mktype(Tenum,     t,    4)
#define mkenumsc(t)     mktype(Tenumsc,   t,    4)
#define mkmask(t)       mktype(Tenum,     t,    9)
#define mkmasksc(t)     mktype(Tenumsc,   t,    9)
#define mkpointer(t)    mktype(Tpointer,  t,    4)
#define mkreference(t)  mktype(Treference,t,    4)
#define mkrvalueref(t)  mktype(Trvalueref,t,    4)
#define mkclass(t, w)   mktype(Tclass,    t,    w)
#define mkstruct(t, w)  mktype(Tstruct,   t,    w)
#define mkunion(t, w)   mktype(Tunion,    t,    w)
#define mkarray(t, w)   mktype(Tarray,    t,    w)
#define mkfun(t)        mktype(Tfun,      t,    0)
#define mkstring()      mkpointer(mkchar())

typedef struct Symbol
{
  Token           token;
  struct Symbol   *next;
  struct Symbol   *left;
  struct Symbol   *right;
  char            name[1]; /* the actual string value flows into the allocated region id[0...] below this struct */
} Symbol;

Symbol  *install(const char*, Token), *lookup(const char*), *gensym(const char*), *gensymidx(const char*, int);

typedef enum Visited { Unexplored, Cold, Hot } Visited;

typedef struct Tnode
{
  Type            type;
  void            *ref;
  Symbol          *id;     /* struct/class/union/enum name */
  Symbol          *baseid; /* base class name */
  Symbol          *sym;    /* typedef name */
  Symbol          *restriction;   /* restriction via typedef base id */
  Symbol          *synonym;       /* synonymous typedef base name for typedef base id */
  Symbol          *extsym;        /* typedef sym of external type w/ custom serializer */
  struct Entry    *response; /* funcs only: points to response struct */
  int             width;
  int             transient;
  const char      *imported;
  struct Tnode    *next;
  struct Tnode    *base;
  Visited         visited;
  Bool            recursive;      /* recursive data type */
  Bool            generated;
  Bool            wsdl;
  int             num;
  Bool            hasmin;
  Bool            hasmax;
  Bool            incmin;
  Bool            incmax;
  LONG64          imin;
  LONG64          imax;
  double          rmin;
  double          rmax;
  int             property;
  const char      *pattern;
} Tnode;

typedef union Value
{
  LONG64          i;
  double          r;
  const char      *s;
} Value;

typedef struct IDinf
{
  Tnode           *typ;
  Storage         sto;
  Bool            hasval;         /* if true, identifier has a default value */
  Bool            ptrval;         /* if true, identifier is a pointer to a default value */
  Bool            fixed;          /* if true and hasval, identifier has a fixed value */
  Value           val;            /* ... with this value */
  int             offset;
  LONG64          minOccurs;
  LONG64          maxOccurs;
  Bool            nillable;
} IDinfo;

typedef struct Entry
{
  Symbol          *sym;
  const char      *tag;
  IDinfo          info;
  Level           level;
  const char      *filename;
  int             lineno;
  struct Entry    *next;
} Entry;

typedef struct Table
{
  Symbol          *sym;
  Level           level;
  Entry           *list;
  struct Table    *prev;
} Table;

typedef struct FNinfo
{
  Tnode   *ret;
  Table   *args;
} FNinfo;

typedef struct IR
{
  LONG64 i;
  double r;
} IR;

typedef struct Node
{
  Tnode           *typ;
  Storage         sto;
  Bool            hasval;         /* if true, this node has a default value */
  Bool            fixed;          /* if true and hasval, this node has a fixed value */
  Value           val;            /* ... this is the value */
  Bool            hasmin;
  Bool            hasmax;
  Bool            incmin;
  Bool            incmax;
  LONG64          minOccurs;
  LONG64          maxOccurs;
  LONG64          imin;
  LONG64          imax;
  double          rmin;
  double          rmax;
  Bool            nillable;
  int             property;
  const char      *pattern;
} Node;

#define ACTION                  0x0000
#define REQUEST_ACTION          0x0001
#define RESPONSE_ACTION         0x0002
#define FAULT_ACTION            0x0004
#define HDRIN                   0x0010  
#define HDROUT                  0x0020
#define MIMEIN                  0x0040
#define MIMEOUT                 0x0080
#define COMMENT                 0x0100
#define ENCODING                0x0200
#define RESPONSE_ENCODING       0x0400
#define STYLE                   0x0800
#define FAULT                   0x1000
#define PROTOCOL                0x2000

typedef struct Data
{
  struct Data     *next;
  const char      *name;
  const char      *text;
} Data;

typedef struct Method
{
  struct Method   *next;
  const char      *name;
  short           mess; /* see #defines above */
  const char      *part;
} Method;

typedef struct Service
{
  struct Service  *next;
  const char      *ns;
  const char      *name;
  const char      *porttype;
  const char      *portname;
  const char      *binding;
  const char      *definitions;
  const char      *transport;
  const char      *URL;
  const char      *executable;
  const char      *import;
  const char      *URI;
  const char      *URI2;
  const char      *WSDL;
  const char      *style;
  const char      *encoding;
  const char      *protocol;
  int             xsi_type;
  const char      *elementForm;
  const char      *attributeForm;
  const char      *documentation;
  struct Method   *list;
  struct Data     *data;
} Service;

typedef struct XPath
{
  struct XPath    *next;
  const char      *name;
  const char      *path;
} XPath;

typedef struct Pragma
{
  struct Pragma   *next;
  const char      *pragma;
} Pragma;

extern Entry *enter(Table*, Symbol*), *entry(Table*, Symbol*), *reenter(Table*, Symbol*), *enumentry(Symbol*);

extern int merge(Table*, Table*);

extern void base_of_derived(Entry*);

extern Table *mktable(Table*);

extern Tnode *mkmethod(Tnode*, Table*);

extern char *emalloc(size_t);

extern Tnode *mktype(Type, void*, int);
extern Tnode *mksymtype(Tnode*, Symbol*);
extern Tnode *mktemplate(Tnode*, Symbol*);

extern int is_transient(Tnode*);
extern int is_response(Tnode*);

extern void set_namespace(const char*);

extern Table *typetable, *enumtable, *classtable, *booltable, *templatetable;

extern void compile(Table*);
extern void freetable(Table*);
extern Entry *unlinklast(Table*); 

extern FILE *fmsg;

extern int soap_version;

extern const char *copt;

extern int aflag;
extern int Aflag;
extern int bflag;
extern int cflag;
extern int c11flag;
extern int Cflag;
extern int eflag;
extern int Ecflag;
extern int Edflag;
extern int Etflag;
extern unsigned long fflag;
extern int gflag;
extern int iflag;
extern int jflag;
extern int mflag;
extern int nflag;
extern int lflag;
extern int Lflag;
extern int Qflag;
extern int sflag;
extern int rflag;
extern int Sflag;
extern int Tflag;
extern int tflag;
extern int uflag;
extern int vflag;
extern int wflag;
extern int xflag;
extern int yflag;
extern int zflag;
extern char dirpath[1024];
extern const char *filename;
extern const char *prefix;
extern const char *fprefix;
extern const char *importpath;
extern int custom_header;
extern int custom_fault;
extern Pragma *pragmas;
extern Service *services;
extern XPath *xpaths;
extern const char *namespaceid;
extern int transient;
extern int imports;
extern const char *imported;
extern int typeNO;

extern const char *envURI;
extern const char *encURI;
extern const char *rpcURI;
extern const char *xsiURI;
extern const char *xsdURI;
