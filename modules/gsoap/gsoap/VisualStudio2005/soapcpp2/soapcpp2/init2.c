/*
        init2.c

        Symbol table initialization.

gSOAP XML Web services tools
Copyright (C) 2000-2015, Robert van Engelen, Genivia Inc. All Rights Reserved.
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

#include "soapcpp2.h"

#ifdef HAVE_CONFIG_H
#include "soapcpp2_yacc.h"
#else
#include "soapcpp2_yacc.tab.h"
#endif

typedef struct Keyword
{
  const char    *s;     /* name */
  Token         t;      /* token */
} Keyword;

static Keyword keywords[] =
{
  { "alignas",          NONE },
  { "alignof",          NONE },
  { "asm",              NONE },
  { "auto",             AUTO },
  { "bool",             BOOL },
  { "break",            BREAK },
  { "case",             CASE },
  { "catch",            NONE },
  { "char",             CHAR },
  { "class",            CLASS },
  { "const",            CONST },
  { "const_cast",       NONE },
  { "continue",         CONTINUE },
  { "default",          DEFAULT },
  { "delete",           NONE },
  { "do",               DO },
  { "double",           DOUBLE },
  { "dynamic_cast",     NONE },
  { "else",             ELSE },
  { "enum",             ENUM },
  { "errno",            NONE },
  { "explicit",         EXPLICIT },
  { "export",           NONE },
  { "extern",           EXTERN },
  { "false",            CFALSE },
  { "final",            FINAL },
  { "float",            FLOAT },
  { "for",              FOR },
  { "friend",           FRIEND },
  { "goto",             GOTO },
  { "if",               IF },
  { "inline",           INLINE },
  { "int",              INT },
  { "int8_t",           CHAR },
  { "int16_t",          SHORT },
  { "int32_t",          INT },
  { "int64_t",          LLONG },
  { "long",             LONG },
  { "LONG64",           LLONG },
  { "mutable",          MUTABLE },
  { "namespace",        NAMESPACE },
  { "new",              NONE },
  { "nullptr",          null },
  { "NULL",             null },
  { "operator",         OPERATOR },
  { "override",         OVERRIDE },
  { "private",          PRIVATE },
  { "protected",        PROTECTED },
  { "public",           PUBLIC },
  { "register",         REGISTER },
  { "reinterpret_cast", NONE },
  { "restrict",         NONE },
  { "return",           RETURN },
  { "short",            SHORT },
  { "signed",           SIGNED },
  { "size_t",           SIZE },
  { "sizeof",           SIZEOF },
  { "static",           STATIC },
  { "static_cast",      NONE },
  { "struct",           STRUCT },
  { "switch",           SWITCH },
  { "template",         TEMPLATE },
  { "this",             NONE },
  { "throw",            NONE },
  { "time_t",           TIME },
  { "true",             CTRUE },
  { "typedef",          TYPEDEF },
  { "typeid",           NONE },
  { "typename",         TYPENAME },
  { "uint8_t",          UCHAR },
  { "uint16_t",         USHORT },
  { "uint32_t",         UINT },
  { "uint64_t",         ULLONG },
  { "ULONG64",          ULLONG },
  { "union",            UNION },
  { "unsigned",         UNSIGNED },
  { "using",            USING },
  { "virtual",          VIRTUAL },
  { "void",             VOID },
  { "volatile",         VOLATILE },
  { "wchar_t",          WCHAR },
  { "while",            WHILE },

  { "operator!",        NONE },
  { "operator~",        NONE },
  { "operator=",        NONE },
  { "operator+=",       NONE },
  { "operator-=",       NONE },
  { "operator*=",       NONE },
  { "operator/=",       NONE },
  { "operator%=",       NONE },
  { "operator&=",       NONE },
  { "operator^=",       NONE },
  { "operator|=",       NONE },
  { "operator<<=",      NONE },
  { "operator>>=",      NONE },
  { "operator||",       NONE },
  { "operator&&",       NONE },
  { "operator|",        NONE },
  { "operator^",        NONE },
  { "operator&",        NONE },
  { "operator==",       NONE },
  { "operator!=",       NONE },
  { "operator<",        NONE },
  { "operator<=",       NONE },
  { "operator>",        NONE },
  { "operator>=",       NONE },
  { "operator<<",       NONE },
  { "operator>>",       NONE },
  { "operator+",        NONE },
  { "operator-",        NONE },
  { "operator*",        NONE },
  { "operator/",        NONE },
  { "operator%",        NONE },
  { "operator++",       NONE },
  { "operator--",       NONE },
  { "operator->",       NONE },
  { "operator[]",       NONE },
  { "operator()",       NONE },

  { "mustUnderstand",   MUSTUNDERSTAND },

  { "soap",             ID },
  { "SOAP_ENV__Header", ID },
  { "dummy",            ID },
  { "soap_header",      ID },

  { "SOAP_ENV__Fault",  ID },
  { "SOAP_ENV__Code",   ID },
  { "SOAP_ENV__Subcode",ID },
  { "SOAP_ENV__Reason", ID },
  { "SOAP_ENV__Text",   ID },
  { "SOAP_ENV__Detail", ID },
  { "SOAP_ENV__Value",  ID },
  { "SOAP_ENV__Node",   ID },
  { "SOAP_ENV__Role",   ID },
  { "faultcode",        ID },
  { "faultstring",      ID },
  { "faultactor",       ID },
  { "detail",           ID },
  { "__type",           ID },
  { "fault",            ID },
  { "__any",            ID },

  { "_QName",           ID },
  { "_XML",             ID },
  { "std::string",      TYPE },
  { "std::wstring",     TYPE },
  { "std::u16string",   TYPE },
  { "std::u32string",   TYPE },
  { "std::deque",       TYPE },
  { "std::queue",       TYPE },
  { "std::list",        TYPE },
  { "std::vector",      TYPE },
  { "std::set",         TYPE },
  { "std::shared_ptr",  TYPE },
  { "std::unique_ptr",  TYPE },
  { "std::auto_ptr",    TYPE },

  { "std::weak_ptr",    TYPE },
  { "std::function",    TYPE },

  { "/*?*/",            NONE },

  { 0,                  0 }
};

/*
init - initialize symbol table with predefined keywords
*/
void init(void)
{
  struct Keyword *k;
  for (k = keywords; k->s; k++)
    install(k->s, k->t);
}
