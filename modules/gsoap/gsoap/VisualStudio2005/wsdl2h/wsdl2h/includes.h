/*
	includes.h

	Common project definitions

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2000-2021, Robert van Engelen, Genivia Inc. All Rights Reserved.
This software is released under one of the following licenses:
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
--------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------

*/

#ifndef INCLUDES_H
#define INCLUDES_H

#include "stdsoap2.h"

#ifdef WITH_OPENSSL
#include "httpda.h"
#endif

#define WSDL2H_VERSION "2.8.114"

#ifdef WIN32
# pragma warning(disable : 4996)
#endif

#include <utility>
#include <iterator>
#include <vector>
#include <set>
#include <map>

struct ltstr
{ bool operator()(const char *s1, const char *s2) const
  { return strcmp(s1, s2) < 0;
  }
}; 

struct eqstr
{ const char *s;
  eqstr(const char *s) : s(s) { }
  bool operator()(const char *t) const
  { return strcmp(s, t) == 0;
  }
}; 

typedef std::set<const char*, ltstr> SetOfString;

typedef std::pair<const char*, const char*> Pair;

struct ltpair
{ bool operator()(Pair s1, Pair s2) const
  { int cmp = strcmp(s1.first, s2.first);
    if (cmp == 0)
      cmp = strcmp(s1.second, s2.second);
    return cmp < 0;
  }
};

typedef std::map<const char*, const char*, ltstr> MapOfStringToString;

typedef std::map<Pair, const char*, ltpair> MapOfPairToString;

typedef std::map<const char*, size_t, ltstr> MapOfStringToNum;

typedef std::vector<const char*> VectorOfString;

extern int _flag,
           aflag,
           bflag,
	   cflag,
	   c11flag,
	   Dflag,
	   dflag,
	   eflag,
	   Fflag,
	   fflag,
	   gflag,
	   iflag,
	   jflag,
	   kflag,
	   Lflag,
	   Mflag,
	   mflag,
	   Oflag,
	   Owflag,
	   Pflag,
	   pflag,
           Qflag,
	   Rflag,
	   sflag,
	   sflag,
	   Uflag,
	   uflag,
	   vflag,
	   Wflag,
	   wflag,
	   Xflag,
	   xflag,
	   yflag,
	   zflag;

extern FILE *stream;

extern SetOfString exturis;

#define MAXINFILES (1000)

extern int openfiles;
extern int infiles;
extern char *infile[MAXINFILES], *outfile, *proxy_host, *proxy_userid, *proxy_passwd, *auth_userid, *auth_passwd;
extern const char *mapfile, *import_path, *cwd_path, *cppnamespace;

extern int proxy_port;

extern const char *service_prefix;
extern const char *schema_prefix;
extern const char *soap_context;

extern const char elementformat[];
extern const char pointerformat[];
extern const char attributeformat[];
extern const char templateformat_open[];
extern const char attrtemplateformat_open[];
extern const char templateformat[];
extern const char pointertemplateformat[];
extern const char arrayformat[];
extern const char arraysizeformat[];
extern const char arrayoffsetformat[];
extern const char sizeformat[];
extern const char choiceformat[];
extern const char schemaformat[];
extern const char serviceformat[];
extern const char paraformat[];
extern const char anonformat[];
extern const char sizeparaformat[];
extern const char pointertemplateparaformat[];
extern const char derivedformat[];

extern const char copyrightnotice[];
extern const char licensenotice[];

extern void *emalloc(size_t size);
extern char *estrdup(const char *s);
extern char *estrdupf(const char *s);

extern void text(const char*);

class Types;
class Message;
class Operation;
class Service;
class Definitions;

#endif
