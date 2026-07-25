/*
	types.h

	WSDL parser and converter to gSOAP header file format

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2000-2012, Robert van Engelen, Genivia Inc. All Rights Reserved.
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

#ifndef TYPES_H
#define TYPES_H

#include "includes.h"
#include "wsdlH.h"

enum Type { NONE, CLASS, ENUM, STRUCT, TYPEDEF };

enum CType { CTNONE, CTBOOL, CTINT, CTUINT, CTLONG, CTULONG, CTFLOAT, CTDOUBLE, CTLONGDOUBLE, CTENUM, CTSTRING, CTWSTRING, CTQNAME, CTWQNAME };

typedef std::map<const char*, CType, ltstr> MapOfStringToCType;

enum Lookup { NOLOOKUP, LOOKUP };

class Types
{
  public:
    SetOfString		knames;	// keywords, reserved words, class names, and typedefs
    MapOfStringToString modtypemap;
    MapOfStringToString deftypemap;
    MapOfStringToString usetypemap;
    MapOfStringToString ptrtypemap;
    MapOfStringToString smptypemap;
    MapOfStringToCType	ctypemap;
    MapOfStringToString wnames;	// name -> wrapper name
    MapOfPairToString	qnames;	// (URI,name) -> name
    MapOfStringToString	uris;	// URI -> prefix
    MapOfStringToNum	syms;	// prefix -> count (ns1, ns2, ...)
    SetOfString		rnames;	// reserved symbolic names to avoid clashes
    SetOfString		onames;	// service operator names
    MapOfPairToString	enames;	// enum symbolic names
    VectorOfString	scope;	// de-anonymizer stack
    int                 snum;   // struct name index, TODO: consider map of URI to count per URI
    int                 unum;   // union name index, TODO: consider map of URI to count per URI
    int                 gnum;   // enum name index, TODO: consider map of URI to count per URI
    bool                with_union;
    bool                fake_union;
    size_t              total; // total number of types
    size_t              omitted; // number of -O removed types
    Types();
    void init();
    int read(const char *file);
  private:
    const char *fname(const char *prefix, const char *URI, const char *qname, SetOfString *reserved, enum Lookup lookup, bool isqname);
  public:
    const char *aname(const char *prefix, const char *URI, const char *qname, SetOfString *reserved = NULL);
    const char *wname(const char *prefix, const char *URI, const char *qname, enum Lookup lookup);
    const char *cname(const char *prefix, const char *URI, const char *qname);
    const char *tname(const char *prefix, const char *URI, const char *qname);
    const char *tnameptr(bool, const char *prefix, const char *URI, const char *qname);
    const char *tnamenoptr(const char *prefix, const char *URI, const char *qname);
    const char *pname(bool flag, bool smart, const char *prefix, const char *URI, const char *qname);
    const char *oname(const char *prefix, const char *URI, const char *qname);
    const char *ename(const char *type, const char *value, bool isqname, bool isbitmask);
    const char *sname(const char *URI, const char *name);
    const char *gname(const char *URI, const char *name);
    const char *uname(const char *URI);
    const char *vname(const char *var);
    const char *nsprefix(const char *prefix, const char *URI);
    const char *prefix(const char *name);
    const char *uri(const char *name);
    const char *deftname(enum Type type, bool mk_pointer, bool is_pointer, const char *prefix, const char *URI, const char *qname, const char *base);
    const char *defename(const char *type, const char *value, bool isqname);
    bool is_defined(const char *prefix, const char *URI, const char *qname);
    bool is_nillable(const xs__element& element);
    bool is_choicetype(const char *prefix, const char *URI, const char *type);
    bool is_ptr(const char *prefix, const char *URI, const char *type);
    void dump(FILE*);
    void define(const char *URI, const char *name, const xs__complexType&);
    void gen(const char *URI, const std::vector<xs__attribute>&, SetOfString&);
    void gen(const char *URI, const std::vector<xs__attributeGroup>&, SetOfString&);
    void gen(const char *URI, const std::vector<xs__all>&, SetOfString&);
    void gen(const char *URI, const std::vector<xs__element>&, const char *minOccurs, const char *maxOccurs, SetOfString&);
    void gen(const char *URI, const std::vector<xs__group>&, SetOfString&);
    void gen(const char *URI, const std::vector<xs__any>&);
    void gen(const char *URI, const std::vector<xs__contents>&, SetOfString&);
    void gen(const char *URI, const char *name, const xs__simpleType&, bool anonymous, bool nested_restriction);
    void gen(const char *URI, const char *name, const xs__complexType&, bool anonymous);
    void gen(const char *URI, const xs__attribute&, SetOfString&);
    void gen(const char *URI, const xs__all&, const char *minOccurs, const char *maxOccurs, SetOfString&);
    void gen(const char *URI, const xs__seqchoice&, const char *minOccurs, const char *maxOccurs, SetOfString&);
    void gen(const char *URI, const char *name, const xs__seqchoice&, const char *minOccurs, const char *maxOccurs, SetOfString&);
    void gen(const char *URI, const xs__element&, bool substok, const char *minOccurs, const char *maxOccurs, SetOfString&);
    void gen(const char *URI, const xs__group&, const char *minOccurs, const char *maxOccurs, SetOfString&);
    void gen(const char *URI, const xs__any&, const char *minOccurs, const char *maxOccurs);
    void gen(const char *URI, const xs__anyAttribute&);
    void gen_inh(const char *URI, const xs__complexType *complexType, bool anonymous);
    void gen_soap_array(const char *t, const char *item, const char *type);
    void gen_substitutions(const char *URI, const xs__element &element, SetOfString&);
    void document(const xs__annotation*);
    void modify(const char *name);
    const char *format(const char *text);
    void gendefault(const char *URI, const char *type, const char *name, xs__simpleType *p, const char *s, const char *q, const char *a);
    CType ctype(const char *s);
};

#endif
