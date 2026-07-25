/*
        symbol2.c

        Symbolic processing: type analysis and code generation.

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

#include "soapcpp2.h"

#ifdef HAVE_CONFIG_H
#include "soapcpp2_yacc.h"
#else
#include "soapcpp2_yacc.tab.h"
#endif

const char *envURI = "http://schemas.xmlsoap.org/soap/envelope/";
const char *encURI = "http://schemas.xmlsoap.org/soap/encoding/";
const char *rpcURI = "http://www.w3.org/2003/05/soap-rpc";
const char *xsiURI = "http://www.w3.org/2001/XMLSchema-instance";
const char *xsdURI = "http://www.w3.org/2001/XMLSchema";
const char *tmpURI = "http://tempuri.org";

static  Symbol *symroot = NULL;  /* pointer to binary tree of symbols to speed up lookup */
static  Symbol *symlist = NULL;  /* pointer to linked list of symbols */
static  Symbol *nslist = NULL;   /* pointer to linked list of namespace prefix symbols */

static Tnode *Tptr[TYPES];

const char *namespaceid = NULL;

static unsigned long idnum = 0;

Service *services = NULL;

XPath *xpaths = NULL; /* TODO? */

FILE *fout = NULL,
     *fhead = NULL,
     *fclient = NULL,
     *fserver = NULL,
     *fheader = NULL,
     *freport = NULL,
     *flib = NULL,
     *fmatlab = NULL,
     *fmheader = NULL;

int partnum = 0;

int typeNO = 1; /* unique nonzero type no. assigned to all types */

static int is_anytype_flag = 0; /* anytype is used */
static int is_anyType_flag = 0; /* anyType XML DOM is used */
static int has_nsmap = 0;

static char soapStub[1024];
static char soapH[1024];
static char soapC[1024];
static char soapClient[1024];
static char soapServer[1024];
static char soapClientLib[1024];
static char soapServerLib[1024];
static char soapReadme[1024];
static char soapProxyH[1024];
static char soapProxyC[1024];
static char soapServiceH[1024];
static char soapServiceC[1024];
static char soapTester[1024];
static char soapMatlab[1024];
static char soapMatlabHdr[1024];

static char pathsoapTester[4096];
static char pathsoapStub[4096];
static char pathsoapH[4096];
static char pathsoapC[4096];
static char pathsoapClient[4096];
static char pathsoapServer[4096];
static char pathsoapClientLib[4096];
static char pathsoapServerLib[4096];
static char pathsoapReadme[4096];
static char pathsoapProxyH[4096];
static char pathsoapProxyC[4096];
static char pathsoapServiceH[4096];
static char pathsoapServiceC[4096];
static char pathsoapMatlab[4096];
static char pathsoapMatlabHdr[4096];

int tagcmp(const char *s, const char *t);
int tagncmp(const char *s, const char *t, size_t n);
int property(Tnode*);
void property_pattern(Tnode*, const char*);
int is_qname(Tnode*);
int is_stdqname(Tnode*);

long minlen(Tnode *typ);
long maxlen(Tnode *typ);
const char* pattern(Tnode *typ);

int is_soap12(const char*);
int has_detail_string(void);
int has_Detail_string(void);

void needs_lang(Entry *e);

int is_mutable(Entry *e);
int is_header_or_fault(Tnode *typ);
int is_body(Tnode *typ);
int is_volatile(Tnode* typ);
int is_untyped(Tnode* typ);
int is_primclass(Tnode* typ);
int is_imported(Tnode* typ);
int is_smart(Tnode*);
int is_smart_shared(Tnode*);
const char * make_shared(Tnode*);
int is_container(Tnode* typ);
int is_template(Tnode* typ);
int is_mask(Tnode* typ);
int is_attachment(Tnode* typ);
int is_void(Tnode* typ);
int has_external(Tnode *typ);
int has_volatile(Tnode *typ);

int is_unmatched(Symbol *sym);
int is_special(const char *s);
int is_invisible(const char *name);
int is_invisible_empty(Tnode *p);
int is_element(Tnode *typ);

int is_eq_nons(const char *s, const char *t);
int is_eq(const char *s, const char *t);

int is_item(Entry *p);
int is_self(Entry *p);

const char *cstring(const char*, int);
const char *xstring(const char*);

const char *get_mxClassID(Tnode*);
const char *t_ident(Tnode*);
const char *c_ident(Tnode*);
const char *ident(const char*);
const char *soap_type(Tnode*);
const char *soap_union_member(Tnode*, Entry*);
const char *c_storage(Storage);
const char *c_const(Storage);
const char *c_init(Entry*);
const char *c_init_a(Entry*, const char*);
const char *c_type(Tnode*);
const char *c_type_constptr_id(Tnode*, const char*);
const char *c_type_id(Tnode*, const char*);
const char *c_type_sym(Tnode*);
const char *c_type_synonym_id(Tnode*, const char*);
const char *xsi_type(Tnode*);
const char *xsi_type_u(Tnode*);
const char *the_type(Tnode*);
const char *wsdl_type(Tnode*, const char*);
const char *base_type(Tnode*, const char*);
const char *xml_tag(Tnode*);
const char *ns_qualifiedElement(Tnode*);
const char *ns_qualifiedElementName(const char*);
const char *ns_qualifiedAttribute(Tnode*);
const char *ns_qualifiedAttributeName(const char*);
const char *ns_tag_convert(Entry*);
const char *ns_convert(const char*);
const char *field(Entry *p, const char *ns);
const char *field_overridden(Table *t, Entry *p, const char *ns);
const char *ns_add(Entry *p, const char *ns);
const char *ns_addx(const char *tag, const char *ns);
const char *ns_add_overridden(Table *t, Entry *p, const char *ns);
const char *ns_tag_remove(Entry*);
const char *ns_remove(const char*);
const char *ns_remove1(const char*);
const char *ns_remove2(const char*, const char*);
const char *ns_remove3(const char*, const char*);
const char *res_remove(const char*);
const char *ns_name(const char*);
const char *ns_cname(const char*, const char*);
const char *ns_fname(const char*);

int has_class(Tnode*);
const char *union_member(Tnode*);
int has_union(Tnode*);
void gen_constructor(FILE *fd, Tnode*);
int has_constructor(Tnode*);
int has_destructor(Tnode*);
int has_getter(Tnode*);
int has_setter(Tnode*);
int has_ns(Tnode*);
int has_ns_t(Tnode*);
int has_ns_eq(const char*, const char*);
const char *strict_check(void);
void fixed_check(FILE*, Entry*, Table*, const char*);
const char *ns_of(const char*);
int eq_ns(const char*, const char*);
const char *prefix_of(const char*);
int has_offset(Tnode*);
int is_bool(Tnode*);
int is_boolean(Tnode*);
int reflevel(Tnode *typ);
Tnode* reftype(Tnode *typ);
int is_response(Tnode*);
int is_XML(Tnode*);
int is_stdXML(Tnode *p);
Entry *get_response(Tnode*);
const char *kind_of(Tnode*);
int is_primitive_or_string(Tnode*);
int is_primitive(Tnode*);
Entry *is_discriminant(Tnode*);
Entry *is_dynamic_array(Tnode*);
int is_pointer_to_derived(Entry*);
void gen_match_derived(FILE *, Tnode*);
int is_transient(Tnode*);
int is_external(Tnode*);
int is_anyType(Tnode*);
int is_anyAttribute(Tnode*);
int is_binary(Tnode*);
int is_hexBinary(Tnode*);
int is_fixedstring(Tnode*);
int is_string(Tnode*);
int is_wstring(Tnode*);
int is_stdstring(Tnode*);
int is_stdwstring(Tnode*);
int is_stdstr(Tnode*);
int is_typedef(Tnode*);
int is_restriction(Tnode*);
int has_restriction_base(Tnode *typ, const char *base);
int is_synonym(Tnode*);
int get_dimension(Tnode*);
int get_dimension_product(Tnode*);
Tnode * get_item_type(Tnode*, int*);
const char *has_soapref(Tnode*);
int is_soapref(Tnode*);
int is_document(const char*);
int is_literal(const char*);
int is_keyword(const char *);

int is_repetition(Entry*);
int is_choice(Entry*);
int required_choice(Tnode*);
int is_sequence(Entry*);
int is_anytype(Entry*);

const char *xsi_type_Tarray(Tnode*);
const char *xsi_type_Darray(Tnode*);

void matlab_def_table(Table*);
void generate_defs(void);
void generate_type(Tnode *);
int no_of_var(Tnode*);
const char *pointer_stuff(Tnode*);
void in_defs(void);
void in_defs2(void);
void in_defs3(void);
void out_defs(void);
void mark_defs(void);
void dup_defs(void);
void del_defs(void);
void in_attach(void);
void soap_serialize(Tnode*);
void soap_traverse(Tnode*);
void soap_default(Tnode*);
void soap_put(Tnode*);
void soap_out(Tnode*);
void soap_out_Darray(Tnode *);
void soap_get(Tnode*);
void soap_in(Tnode*);
void soap_in_Darray(Tnode *);
void soap_instantiate(Tnode *);
void soap_dup(Tnode *);
void soap_del(Tnode *);
int get_Darraydims(Tnode *typ);
const char* get_Darraysize(const char *a, int d);
const char *nillable(Entry*);
const char *nillable_ref(Entry*);

void soap_serve(Table*);
void generate_proto(FILE*, Table*, Entry*);
/*
void generate_call(Table*, Entry*);
void generate_server(Table*, Entry*);
*/
void detect_cycles(void);
void detect_recursive_type(Tnode*);
void generate_header(Table*);
void get_namespace_prefixes(void);
void generate_schema(Table*);
void gen_schema(FILE*, Table*, const char*, const char*, int, const char*, const char*);
void gen_schema_type(FILE *, Table*, Entry *, const char*, const char*, int, const char*, const char*);
void gen_report_hr();
void gen_report_operation(const char *name, Entry *method, int service);
void gen_report_type(Tnode*, const char*);
void gen_report_type_doc(Entry *);
void gen_report_members(Entry *type, const char*, const char*);
void gen_report_member(Entry*, Entry*);
void gen_method_documentation(FILE *fd, Entry *p, const char *ns);
void gen_type_documentation(FILE *fd, Entry *type, const char *ns);
int gen_member_documentation(FILE *fd, Symbol *type, Entry *member, const char *ns, int scope);
void gen_text(FILE *fd, const char *s);
void gen_schema_elements_attributes(FILE *fd, Table *t, const char *ns, const char *ns1, const char *encoding, const char *style);
void gen_schema_elements(FILE *fd, Tnode *p, Tnode *b, const char *ns, const char *ns1);
int gen_schema_element(FILE *fd, Tnode *p, Entry *q, const char *ns, const char *ns1);
void gen_schema_attributes(FILE *fd, Tnode *p, Tnode *b, const char *ns, const char *ns1);
void gen_wsdl(FILE*, Table*, const char*, const char*, const char*, const char*, const char*, const char*, const char*, const char*);
const char *default_value(Entry*);
const char *set_default_value(Entry*);
void gen_nsmap(FILE*);

void gen_proxy(FILE*, Table*, Symbol*, const char*, const char*);
void gen_object(FILE*, Table*, const char*);
void gen_proxy_header(FILE*, Table*, Symbol*, const char*);
void gen_proxy_code(FILE*, Table*, Symbol*, const char*);
void gen_object_header(FILE*, Table*, Symbol*, const char*);
void gen_object_code(FILE*, Table*, Symbol*, const char*);
void gen_method(FILE *fd, Entry *method, int server);
void gen_report_params(Entry *p, Entry *result, int server);
void gen_report_req_params(Tnode *typ);
void gen_report_set_params(Tnode *typ);
void gen_params_ref(FILE *fd, Table *params, Entry *result, int flag);
void gen_params(FILE *fd, Table *params, Entry *result, int flag);
void gen_args(FILE *fd, Table *params, Entry *result, int flag);
void gen_query_url(FILE *fd, Table *params, int soap);
void gen_query_url_type2s(FILE *fd, Tnode *typ, const char *name, const char *ptr, const char *idx);
void gen_query_send_form_init(FILE *fd, Table *params);
void gen_query_send_form(FILE *fd, Table *params);
void gen_query_recv_form_init(FILE *fd, Entry *result);
void gen_query_recv_form(FILE *fd, Entry *result);
void gen_query_form_type2s(FILE *fd, Tnode *typ, const char *name, const char *ptr, const char *idx);
void gen_query_form_s2type(FILE *fd, Tnode *typ, const char *name, const char *ptr, const char *ref, const char *idx);
void gen_call_proto(FILE *fd, Entry *method);
void gen_call_method(FILE *fd, Entry *method, const char *name);
void gen_serve_method(FILE *fd, Table *table, Entry *param, const char *name);

void gen_data(const char*, Table*, const char*, const char*);
FILE *gen_env(const char*, const char*, int, const char*, int);
void gen_xmlns(FILE*, int soap);
void gen_field(FILE*, int, Entry*, const char*, const char*, const char*, int, int);
void gen_val(FILE*, int, Tnode*, const char*, const char*, const char*, int);
void gen_atts(FILE*, Table*, const char*, const char*, const char*);

struct pair
{
  const char *action;
  Entry *method;
};

int
mapcomp(const void *a, const void *b)
{
  const struct pair *p = a, *q = b;
  return strcmp(p->action, q->action);
}

/*
install - add new symbol
*/
Symbol *
install(const char *name, Token token)
{
  Symbol **q = &symroot;
  Symbol *p = (Symbol*)emalloc(sizeof(Symbol)+strlen(name));
  strcpy(p->name, name);
  p->token = token;
  p->next = symlist;
  p->left = NULL;
  p->right = NULL;
  symlist = p;
  while (*q)
  {
    int cmp = strcmp((*q)->name, name);
    if (cmp < 0)
      q = &(*q)->right;
    else
      q = &(*q)->left;
  }
  *q = p;
  return p;
}

/*
lookup - search for an identifier's name. If found, return pointer to symbol table entry. Return pointer 0 if not found.
*/
Symbol *
lookup(const char *name)
{
  Symbol *p = symroot;
  while (p)
  {
    int cmp = strcmp(p->name, name);
    if (cmp == 0)
      return p;
    if (cmp < 0)
      p = p->right;
    else
      p = p->left;
  }
  return NULL;
}

/*
gensymidx - generate new symbol from base name and index
*/
Symbol *
gensymidx(const char *base, int idx)
{
  char buf[1024];
  Symbol *s;
  sprintf(buf, "%s_%d", base, idx);
  s = lookup(buf);
  if (s)
    return s;
  return install(buf, ID);
}

/*
gensym - generate new symbol from base name
*/
Symbol *
gensym(const char *base)
{
  static int num = 1;
  return gensymidx(base, num++);
}

/*
mktable - make a new symbol table with a pointer to a previous table
*/
Table *
mktable(Table *table)
{
  Table *p;
  p = (Table*)emalloc(sizeof(Table));
  p->sym = lookup("/*?*/");
  p->list = (Entry*) 0;
  if (table == (Table*) 0)
    p->level = INTERNAL;
  else
    p->level = (enum Level)(((int)table->level) + 1);
  p->prev = table;
  return p;
}

/*
mkmethod - make a new method by calling mktype
*/
Tnode *
mkmethod(Tnode *ret, Table *args)
{
  FNinfo *fn = (FNinfo*)emalloc(sizeof(FNinfo));
  fn->ret = ret;
  fn->args = args;
  return mktype(Tfun, fn, 0);
}

/*
freetable - free space by removing a table
*/
void
freetable(Table *table)
{
  Entry *p, *q;
  if (table == (Table*) 0)
    return;
  for (p = table->list; p != (Entry*) 0; p = q)
  {
    q = p->next;
    free(p);
  }
  free(table);
}

/*
unlinklast - unlink last entry added to table
*/
Entry *
unlinklast(Table *table)
{
  Entry **p, *q;
  if (table == (Table*)0)
    return (Entry*)0;
  for (p = &table->list; *p != (Entry*)0 && (*p)->next != (Entry*)0; p = &(*p)->next)
    ;
  q = *p;
  *p = (Entry*)0;
  return q;
}

/*
enter - enter a symbol in a table. Error if already in the table
*/
Entry *
enter(Table *table, Symbol *sym)
{
  Entry *p, *q = NULL;
  Storage sto = Snone;
again:
  for (p = table->list; p; q = p, p = p->next)
  {
    if (p->sym == sym)
    {
      if (p->info.typ->type != Tfun)
      {
        char *s;
        sprintf(errbuf, "Duplicate declaration of '%s' (already declared at line %d), changing conflicting identifier name to '%s_'", sym->name, p->lineno, sym->name);
        semwarn(errbuf);
        s = (char*)emalloc(strlen(sym->name) + 2);
        strcpy(s, sym->name);
        strcat(s, "_");
        sym = lookup(s);
        if (!sym)
          sym = install(s, ID);
        free(s);
        goto again;
      }
      if (p->level == GLOBAL)
      {
        if (((int)p->info.sto & (int)Sextern))
        {
          sto = p->info.sto;
          break;
        }
        else
        {
          sprintf(errbuf, "Duplicate declaration of '%s' (already declared at line %d)", sym->name, p->lineno);
          semwarn(errbuf);
          return p;
        }
      }
    }
  }
  p = (Entry*)emalloc(sizeof(Entry));
  p->sym = sym;
  p->tag = NULL;
  p->info.typ = NULL;
  p->info.sto = sto;
  p->info.hasval = False;
  p->info.ptrval = False;
  p->info.fixed = False;
  p->info.minOccurs = 1;
  p->info.maxOccurs = 1;
  p->info.nillable = zflag >= 1 && zflag <= 3; /* False, unless version 2.8.30 or earlier */
  p->info.offset = 0;
  p->level = table->level;
  p->filename = filename;
  p->lineno = yylineno;
  p->next = NULL;
  if (!q)
    table->list = p;
  else
    q->next = p;
  return p;
}

/*
entry - return pointer to table entry of a symbol
*/
Entry *
entry(Table *table, Symbol *sym)
{
  Table *t;
  Entry *p;
  for (t = table; t; t = t->prev)
    for (p = t->list; p; p = p->next)
      if (p->sym == sym)
        return p;
  return (Entry*)0;
}

/*
reenter - re-enter a symbol in a table.
*/
Entry *
reenter(Table *table, Symbol *sym)
{
  Entry *p, *q = NULL;
  for (p = table->list; p; q = p, p = p->next)
    if (p->sym == sym)
      break;
  if (p && p->next)
  {
    if (q)
      q->next = p->next;
    else
      table->list = p->next;
    for (q = p->next; q->next; q = q->next)
      ;
    q->next = p;
    p->next = NULL;
  }
  return p;
}

/*
merge - append two tables if members are not duplicated
*/
int
merge(Table *dest, Table *src)
{
  Entry *p, *q;
  for (p = src->list; p; p = p->next)
  {
    q = entry(dest, p->sym);
    if (!q)
    {
      q = enter(dest, p->sym);
      q->info = p->info;
    }
    else if (q->info.typ != p->info.typ)
      return 1;
  }
  return 0;
}

Entry *
enumentry(Symbol *sym)
{
  const char *s = strstr(sym->name, "::");
  if (s && s[2])
  {
    Entry *e;
    char *t = (char*)emalloc(s - sym->name + 1);
    strncpy(t, sym->name, s - sym->name);
    t[s - sym->name] = '\0';
    e = entry(enumtable, lookup(t));
    free(t);
    if (!e)
      return NULL;
    return entry((Table*)e->info.typ->ref, lookup(s + 2));
  }
  else
  {
    Table *t;
    for (t = enumtable; t; t = t->prev)
    {
      Entry *p;
      for (p = t->list; p; p = p->next)
      {
        Entry *e = entry((Table*)p->info.typ->ref, sym);
        if (e)
          return e;
      }
    }
  }
  return NULL;
}

/*
mktype - make a (new) type with a reference to additional information and the
width in bytes required to store objects of that type. A pointer to the
type is returned which can be compared to check if types are identical.
*/
Tnode *
mktype(Type type, void *ref, int width)
{
  Tnode *p;
  if (type != Tstruct && type != Tclass && type != Tunion && ((type != Tenum && type != Tenumsc) || ref))
  {
    for (p = Tptr[type]; p; p = p->next)
    {
      if (p->ref == ref && p->sym == (Symbol*)0 && p->width == width && ((p->transient == 1 && transient == 1) || (p->transient != 1 && transient != 1)))
      {
        if (imported && !p->imported)
          p->imported = imported;
        return p;       /* type already exists in table */
      }
    }
  }
  p = (Tnode*)emalloc(sizeof(Tnode));   /* install new type */
  p->type = type;
  p->ref = ref;
  p->id = lookup("/*?*/");
  p->baseid = NULL;
  p->sym = NULL;
  p->restriction = NULL;
  p->synonym = NULL;
  p->extsym = NULL;
  p->response = NULL;
  p->width = width;
  p->visited = Unexplored;
  p->recursive = False;
  p->generated = False;
  p->wsdl = False;
  p->next = Tptr[type];
  p->base = NULL;
  p->transient = transient;
  p->imported = imported;
  p->hasmin = False;
  p->hasmax = False;
  p->incmin = True;
  p->incmax = True;
  p->imin = 0;
  p->imax = 0;
  p->rmin = 0.0;
  p->rmax = 0.0;
  p->property = 1;
  p->pattern = NULL;
  p->num = typeNO++;
  Tptr[type] = p;
  DBGLOG(fprintf(stderr, "New type %s %s\n", c_type(p), p->imported));
  /* deprecated change, to revert back to 2.7 behavior that is preferable with #module
  if (type == Tpointer && ((Tnode*)ref)->imported && (((Tnode*)ref)->type == Tenum || ((Tnode*)ref)->type == Tenumsc || ((Tnode*)ref)->type == Tstruct || ((Tnode*)ref)->type == Tclass))
    p->imported = ((Tnode*)ref)->imported;
  else */
  if (lflag && !is_transient(p) && (type == Tenum || type == Tenumsc || type == Tstruct || type == Tclass))
    mkpointer(p);
  return p;
}

Tnode *
mksymtype(Tnode *typ, Symbol *sym)
{
  Tnode *p;
  p = (Tnode*)emalloc(sizeof(Tnode));   /* install new type */
  p->type = typ->type;
  p->ref = typ->ref;
  if (typ->id == lookup("/*?*/"))
    p->id = sym;
  else
    p->id = typ->id;
  p->sym = sym;
  p->restriction = NULL;
  p->synonym = NULL;
  p->extsym = typ->extsym;
  p->response = (Entry*)0;
  p->width = typ->width;
  p->visited = Unexplored;
  p->recursive = False;
  p->generated = False;
  p->wsdl = False;
  p->next = Tptr[typ->type];
  p->transient = transient;
  p->imported = imported;
  p->hasmin = False;
  p->hasmax = False;
  p->incmin = True;
  p->incmax = True;
  p->imin = 0;
  p->imax = 0;
  p->rmin = 0.0;
  p->rmax = 0.0;
  property_pattern(p, sym->name);
  p->num = typeNO++;
  Tptr[typ->type] = p;
  DBGLOG(fprintf(stderr, "New typedef %s %s\n", c_type(p), p->imported));
  return p;
}

Tnode *
mktemplate(Tnode *typ, Symbol *id)
{
  Tnode *p;
  for (p = Tptr[Ttemplate]; p; p = p->next)
  {
    if (p->ref == typ && p->id == id && ((p->transient == 1 && transient == 1) || (p->transient != 1 && transient != 1)))
    {
      if (imported && !p->imported)
        p->imported = imported;
      return p; /* type alrady exists in table */
    }
  }
  p = (Tnode*)emalloc(sizeof(Tnode));   /* install new type */
  p->type = Ttemplate;
  p->ref = typ;
  p->id = id;
  p->sym = NULL;
  p->restriction = NULL;
  p->synonym = NULL;
  p->extsym = NULL;
  p->response = (Entry*)0;
  p->width = 0;
  p->generated = False;
  p->wsdl = False;
  p->next = Tptr[Ttemplate];
  p->transient = transient;
  p->imported = imported;
  p->hasmin = False;
  p->hasmax = False;
  p->incmin = True;
  p->incmax = True;
  p->imin = 0;
  p->imax = 0;
  p->rmin = 0.0;
  p->rmax = 0.0;
  p->property = 1;
  p->pattern = NULL;
  p->num = typeNO++;
  Tptr[Ttemplate] = p;
  return p;
}

/*      DO NOT REMOVE OR ALTER (SEE LICENCE AGREEMENT AND COPYING.txt)  */
void
copyrightnote(FILE *fd, const char *fn)
{
  fprintf(fd,
"/* %s\n   Generated by gSOAP " VERSION " for %s\n\
\n\
gSOAP XML Web services tools\n\
Copyright (C) 2000-2021, Robert van Engelen, Genivia Inc. All Rights Reserved.\n\
The soapcpp2 tool and its generated software are released under the GPL.\n\
This program is released under the GPL with the additional exemption that\n\
compiling, linking, and/or using OpenSSL is allowed.\n\
--------------------------------------------------------------------------------\n\
A commercial use license is available from Genivia Inc., contact@genivia.com\n\
--------------------------------------------------------------------------------\n\
*/", fn, filename);
}

void
banner(FILE *fd, const char *text)
{
  int i;
  fprintf(fd, "\n\n/");
  for (i = 0; i < 78; i++)
    fputc('*', fd);
  fprintf(fd, "\\\n *%76s*\n * %-75s*\n *%76s*\n\\", "", text, "");
  for (i = 0; i < 78; i++)
    fputc('*', fd);
  fprintf(fd, "/\n");
}

void
identify(FILE *fd, const char *fn)
{
  time_t t = time(NULL), *p = &t;
  char tmp[256];
  char *source_date_epoch;

  source_date_epoch = getenv("SOURCE_DATE_EPOCH");
  if (source_date_epoch)
  {
    ULONG64 epoch;
    if (sscanf(source_date_epoch, SOAP_ULONG_FORMAT, &epoch) == 1 && epoch != 0)
      t = epoch;
  }

  strftime(tmp, 256, "%Y-%m-%d %H:%M:%S GMT", gmtime(p));
  fprintf(fd, "\n\nSOAP_SOURCE_STAMP(\"@(#) %s ver " VERSION " %s\")\n", fn, tmp);
}

void
compile(Table *table)
{
  Entry *p;
  Tnode *typ;
  Pragma *pragma;
  int classflag = 0;
  int found;
  int filenum;
  const char *s;
  char base[1024];

  found = 0;
  for (p = table->list; p; p = p->next)
    if (p->info.typ->type == Tfun && !(p->info.sto & Sextern))
      found = 1;
  if (!found)
    Sflag = Cflag = Lflag = 1; /* no service operations were found */

  if (*dirpath)
    fprintf(fmsg, "Using project directory path: %s\n", dirpath);

  fprefix = "soap";
  if (namespaceid)
  {
    prefix = namespaceid;
    fprintf(fmsg, "Using C++ namespace: %s\n", namespaceid);
  }
  else if (!lflag)
  {
    fprefix = prefix;
  }
  strcpy(base, prefix);
  if (cflag)
    s = ".c";
  else
    s = ".cpp";

  strcpy(soapMatlab, base);
  strcat(soapMatlab, "Matlab.c");
  strcpy(pathsoapMatlab, dirpath);
  strcat(pathsoapMatlab, soapMatlab );

  strcpy(soapMatlabHdr, base);
  strcat(soapMatlabHdr, "Matlab.h");
  strcpy(pathsoapMatlabHdr, dirpath);
  strcat(pathsoapMatlabHdr, soapMatlabHdr);

  strcpy(soapStub, base);
  strcat(soapStub, "Stub.h");
  strcpy(pathsoapStub, dirpath);
  strcat(pathsoapStub, soapStub);
  strcpy(soapH, base);
  strcat(soapH, "H.h");
  strcpy(pathsoapH, dirpath);
  strcat(pathsoapH, soapH);
  strcpy(soapC, base);
  if (fflag)
    strcat(soapC, "C_nnn");
  else
    strcat(soapC, "C");
  strcat(soapC, s);
  strcpy(pathsoapC, dirpath);
  strcat(pathsoapC, soapC);
  strcpy(soapClient, base);
  strcat(soapClient, "Client");
  strcat(soapClient, s);
  strcpy(pathsoapClient, dirpath);
  strcat(pathsoapClient, soapClient);
  strcpy(soapServer, base);
  strcat(soapServer, "Server");
  strcat(soapServer, s);
  strcpy(pathsoapServer, dirpath);
  strcat(pathsoapServer, soapServer);
  strcpy(soapClientLib, base);
  strcat(soapClientLib, "ClientLib");
  strcat(soapClientLib, s);
  strcpy(pathsoapClientLib, dirpath);
  strcat(pathsoapClientLib, soapClientLib);
  strcpy(soapServerLib, base);
  strcat(soapServerLib, "ServerLib");
  strcat(soapServerLib, s);
  strcpy(pathsoapServerLib, dirpath);
  strcat(pathsoapServerLib, soapServerLib);
  strcpy(soapReadme, base);
  strcat(soapReadme, "Readme.md");
  strcpy(pathsoapReadme, dirpath);
  strcat(pathsoapReadme, soapReadme);

  if (mflag)
  {
    fprintf(fmsg, "Saving %s Matlab definitions\n", pathsoapMatlab);
    fmatlab = fopen(pathsoapMatlab, "w");
    if (!fmatlab)
      execerror("Cannot write to file");
    copyrightnote(fmatlab, soapMatlab);
    fprintf(fmatlab, "\n#include \"%s\"\n", soapMatlabHdr);
    fprintf(fmsg, "Saving %s Matlab definitions\n", pathsoapMatlabHdr);
    fmheader = fopen(pathsoapMatlabHdr, "w");
    if (!fmheader)
      execerror("Cannot write to file");
    copyrightnote(fmheader, soapMatlabHdr);
    fprintf(fmheader, "\n#include \"mex.h\"\n#include \"%s\"\n", soapStub);
  }

  if (rflag)
  {
    fprintf(fmsg, "Saving %s report\n", pathsoapReadme);
    freport = fopen(pathsoapReadme, "w");
    if (!freport)
      execerror("Cannot write to file");
  }

  fprintf(fmsg, "Saving %s annotated copy of the source interface header file\n", pathsoapStub);
  fheader = fopen(pathsoapStub, "w");
  if (!fheader)
    execerror("Cannot write to file");
  copyrightnote(fheader, soapStub);
  fprintf(fheader, "\n");
  for (pragma = pragmas; pragma; pragma = pragma->next)
    fprintf(fheader, "\n%s", pragma->pragma);
  fprintf(fheader, "\n\n#ifndef %sStub_H\n#define %sStub_H", prefix, prefix);
  if (nflag)
    fprintf(fheader, "\n#ifndef WITH_NONAMESPACES\n#define WITH_NONAMESPACES\n#endif");
  if (namespaceid)
    fprintf(fheader, "\n#ifndef WITH_NOGLOBAL\n#define WITH_NOGLOBAL\n#endif");
  fprintf(fheader, "\n#include \"stdsoap2.h\"");
  fprintf(fheader, "\n#if GSOAP_VERSION != %d\n# error \"GSOAP VERSION %d MISMATCH IN GENERATED CODE VERSUS LIBRARY CODE: PLEASE REINSTALL PACKAGE\"\n#endif\n", GSOAP_VERSION, GSOAP_VERSION);
  if (namespaceid)
    fprintf(fheader, "\n\nnamespace %s {", namespaceid);

  fprintf(fmsg, "Saving %s serialization functions to #include in projects\n", pathsoapH);
  fhead = fopen(pathsoapH, "w");
  if (!fhead)
    execerror("Cannot write to file");
  copyrightnote(fhead, soapH);
  fprintf(fhead, "\n\n#ifndef %sH_H\n#define %sH_H", prefix, prefix);
  fprintf(fhead, "\n#include \"%s\"", soapStub);
  if (namespaceid)
    fprintf(fhead, "\n\nnamespace %s {", namespaceid);
  fprintf(fhead, "\n#ifndef WITH_NOIDREF");
  if (!cflag && !namespaceid)
    fprintf(fhead, "\n\n#ifdef __cplusplus\nextern \"C\" {\n#endif");
  fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_markelement(struct soap*, const void*, int);");
  if (!cflag && !namespaceid)
    fprintf(fhead, "\n\n#ifdef __cplusplus\n}\n#endif");
  fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_putindependent(struct soap*);");
  fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_getindependent(struct soap*);");
  fprintf(fhead, "\n#endif");
  if (!cflag && !namespaceid)
    fprintf(fhead, "\n\n#ifdef __cplusplus\nextern \"C\" {\n#endif");
  fprintf(fhead, "\nSOAP_FMAC3 void * SOAP_FMAC4 soap_getelement(struct soap*, const char*, int*);");
  fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_putelement(struct soap*, const void*, const char*, int, int);");
  fprintf(fhead, "\nSOAP_FMAC3 void * SOAP_FMAC4 soap_dupelement(struct soap*, const void*, int);");
  fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_delelement(const void*, int);");
  if (!cflag && !namespaceid)
    fprintf(fhead, "\n\n#ifdef __cplusplus\n}\n#endif");
  fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_ignore_element(struct soap*);");

  detect_cycles();

  generate_header(table);

  generate_schema(table);

  if (!Sflag && !iflag && !jflag)
  {
    fprintf(fmsg, "Saving %s client call stub functions\n", pathsoapClient);
    fclient = fopen(pathsoapClient, "w");
    if (!fclient)
      execerror("Cannot write to file");
    copyrightnote(fclient, soapClient);
    fprintf(fclient, "\n\n#if defined(__BORLANDC__)");
    fprintf(fclient, "\n#pragma option push -w-8060");
    fprintf(fclient, "\n#pragma option push -w-8004");
    fprintf(fclient, "\n#endif");
    fprintf(fclient, "\n#include \"%sH.h\"", prefix);
    if (namespaceid)
      fprintf(fclient, "\n\nnamespace %s {", namespaceid);
    identify(fclient, soapClient);

    if (!Lflag)
    {
      flib = fopen(pathsoapClientLib, "w");
      if (!flib)
        execerror("Cannot write to file");
      copyrightnote(flib, soapClientLib);
      fprintf(fmsg, "Saving %s client stubs with serializers (use only for libs)\n", pathsoapClientLib);
      fprintf(flib, "\n\n/** Use this file in your project build instead of the two files %s and %s. This hides the serializer functions by making them static, avoiding linking problems when linking multiple clients and servers. */\n", soapC, soapClient);
      fprintf(flib, "\n/* disable warnings for unused static functions defined in %s */", soapC);
      fprintf(flib, "\n#if defined(WIN32)");
      fprintf(flib, "\n#pragma warning(disable:4505)");
      fprintf(flib, "\n#elif defined(__GNUC__)");
      fprintf(flib, "\n#pragma GCC diagnostic ignored \"-Wunused-function\"");
      fprintf(flib, "\n#elif defined(__clang__)");
      fprintf(flib, "\n#pragma clang diagnostic ignored \"-Wunused-function\"");
      fprintf(flib, "\n#endif");
      fprintf(flib, "\n#define WITH_STATIC");
      fprintf(flib, "\n#define SOAP_FMAC3 static");
      fprintf(flib, "\n#include \"%s\"", soapC);
      fprintf(flib, "\n#include \"%s\"", soapClient);
      fprintf(flib, "\n\n/* End of %s */\n", soapClientLib);
      if (fclose(flib))
        execerror("Cannot write to file");
    }
  }
  if (!Cflag && !iflag && !jflag)
  {
    fprintf(fmsg, "Saving %s server request dispatcher\n", pathsoapServer);
    fserver = fopen(pathsoapServer, "w");
    if (!fserver)
      execerror("Cannot write to file");
    copyrightnote(fserver, soapServer);
    fprintf(fserver, "\n\n#if defined(__BORLANDC__)");
    fprintf(fserver, "\n#pragma option push -w-8060");
    fprintf(fserver, "\n#pragma option push -w-8004");
    fprintf(fserver, "\n#endif");
    fprintf(fserver, "\n#include \"%sH.h\"", prefix);
    if (namespaceid)
      fprintf(fserver, "\n\nnamespace %s {", namespaceid);
    identify(fserver, soapServer);

    if (!Lflag)
    {
      flib = fopen(pathsoapServerLib, "w");
      if (!flib)
        execerror("Cannot write to file");
      copyrightnote(flib, soapServerLib);
      fprintf(fmsg, "Saving %s server request dispatcher with serializers (use only for libs)\n", pathsoapServerLib);
      fprintf(flib, "\n\n/** Use this file in your project build instead of the two files %s and %s. This hides the serializer functions by making them static, avoiding linking problems when linking multiple clients and servers. */\n", soapC, soapServer);
      fprintf(flib, "\n/* disable warnings for unused static functions defined in %s */", soapC);
      fprintf(flib, "\n#if defined(WIN32)");
      fprintf(flib, "\n#pragma warning(disable:4505)");
      fprintf(flib, "\n#elif defined(__GNUC__)");
      fprintf(flib, "\n#pragma GCC diagnostic ignored \"-Wunused-function\"");
      fprintf(flib, "\n#elif defined(__clang__)");
      fprintf(flib, "\n#pragma clang diagnostic ignored \"-Wunused-function\"");
      fprintf(flib, "\n#endif");
      fprintf(flib, "\n#define WITH_STATIC");
      fprintf(flib, "\n#define SOAP_FMAC3 static");
      fprintf(flib, "\n#include \"%s\"", soapC);
      fprintf(flib, "\n#include \"%s\"", soapServer);
      fprintf(flib, "\n\n/* End of %s */\n", soapServerLib);
      if (fclose(flib))
        execerror("Cannot write to file");
    }
  }

  if (!iflag && !jflag)
    soap_serve(table);

  classflag = 0;
  for (p = classtable->list; p; p = p->next)
  {
    if (p->info.typ->type == Tclass && p->info.typ->transient <= 0)
    {
      classflag = 1;
      break;
    }
  }
  for (p = enumtable->list; p; p = p->next)
  {
    if (p->info.typ->type == Tenumsc && p->info.typ->transient <= 0)
    {
      classflag = 1;
      break;
    }
  }
  if (classflag || Tptr[Ttemplate])
  {
    if (cflag)
      semwarn("Option -c conflicts with the use of class definitions in the specified input");
  }

  for (filenum = 1; partnum == 0; filenum++)
  {
    if (fflag)
    {
      char *t = strrchr(pathsoapC, '.');
      sprintf(t-3, "%03d", filenum);
      *t = '.';
      fprintf(fmsg, "Saving %s serialization functions (part %d)\n", pathsoapC, filenum);
      partnum = fflag; /* number of defs per file */
    }
    else
    {
      fprintf(fmsg, "Saving %s serialization functions\n", pathsoapC);
      partnum = 1;
    }
    fout = fopen(pathsoapC, "w");
    if (!fout)
      execerror("Cannot write to file");
    copyrightnote(fout, soapC);
    fprintf(fout, "\n\n#if defined(__BORLANDC__)");
    fprintf(fout, "\n#pragma option push -w-8060");
    fprintf(fout, "\n#pragma option push -w-8004");
    fprintf(fout, "\n#endif");

    fprintf(fout, "\n\n#include \"%sH.h\"", prefix);
    if (namespaceid)
      fprintf(fout, "\n\nnamespace %s {", namespaceid);
    identify(fout, soapC);

    fflush(fout);

    if (filenum == 1)
    {
      if (!lflag)
      {
        fprintf(fout, "\n\n#ifndef WITH_NOGLOBAL");
        fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_getheader(struct soap *soap)\n{\n\tsoap->part = SOAP_IN_HEADER;\n\tsoap->header = soap_in_SOAP_ENV__Header(soap, \"SOAP-ENV:Header\", soap->header, NULL);\n\tsoap->part = SOAP_END_HEADER;\n\treturn soap->header == NULL;\n}");
        fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_putheader(struct soap *soap)\n{\n\tif (soap->version && soap->header)\n\t{\tsoap->part = SOAP_IN_HEADER;\n\t\tif (soap_out_SOAP_ENV__Header(soap, \"SOAP-ENV:Header\", 0, soap->header, \"\"))\n\t\t\treturn soap->error;\n\t\tsoap->part = SOAP_END_HEADER;\n\t}\n\treturn SOAP_OK;\n}");
        if (cflag)
        {
          fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_serializeheader(struct soap *soap)\n{\n\tif (soap->version && soap->header)\n\t\tsoap_serialize_SOAP_ENV__Header(soap, soap->header);\n}");
          fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_header(struct soap *soap)\n{\n\tif (soap->header == NULL)\n\t{\tif ((soap->header = (struct SOAP_ENV__Header*)soap_malloc(soap, sizeof(struct SOAP_ENV__Header))))\n\t\t\tsoap_default_SOAP_ENV__Header(soap, soap->header);\n\t}\n}");
        }
        else if ((p = entry(classtable, lookup("SOAP_ENV__Header"))) && p->info.typ->type == Tstruct)
        {
          fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_serializeheader(struct soap *soap)\n{\n\tif (soap->version && soap->header)\n\t\tsoap_serialize_SOAP_ENV__Header(soap, soap->header);\n}");
          fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_header(struct soap *soap)\n{\n\tif (soap->header == NULL)\n\t{\tif ((soap->header = soap_new_SOAP_ENV__Header(soap)))\n\t\t\tsoap_default_SOAP_ENV__Header(soap, soap->header);\n\t}\n}");
        }
        else
        {
          fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_serializeheader(struct soap *soap)\n{\n\tif (soap->version && soap->header)\n\t\tsoap->header->soap_serialize(soap);\n}");
          fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_header(struct soap *soap)\n{\n\tif (soap->header == NULL)\n\t{\tif ((soap->header = soap_new_SOAP_ENV__Header(soap)))\n\t\t\tsoap->header->soap_default(soap);\n\t}\n}");
        }
        fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_fault(struct soap *soap)\n{\n\tif (soap->fault == NULL)\n\t{\tsoap->fault = soap_new_SOAP_ENV__Fault(soap, -1);\n\t\tif (soap->fault == NULL)\n\t\t\treturn;\n\t}\n\tif (soap->version == 2 && soap->fault->SOAP_ENV__Code == NULL)\n\t\tsoap->fault->SOAP_ENV__Code = soap_new_SOAP_ENV__Code(soap, -1);\n\tif (soap->version == 2 && soap->fault->SOAP_ENV__Reason == NULL)\n\t\tsoap->fault->SOAP_ENV__Reason = soap_new_SOAP_ENV__Reason(soap, -1);\n}");
        if (cflag || ((p = entry(classtable, lookup("SOAP_ENV__Fault"))) && p->info.typ->type == Tstruct))
        {
          fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_serializefault(struct soap *soap)\n{\n\tif (soap->fault)\n\t\tsoap_serialize_SOAP_ENV__Fault(soap, soap->fault);\n}");
          fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_putfault(struct soap *soap)\n{\n\tif (soap->fault)\n\t\treturn soap_put_SOAP_ENV__Fault(soap, soap->fault, \"SOAP-ENV:Fault\", \"\");\n\treturn SOAP_OK;\n}");
          fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_getfault(struct soap *soap)\n{\n\treturn (soap->fault = soap_get_SOAP_ENV__Fault(soap, NULL, \"SOAP-ENV:Fault\", NULL)) == NULL;\n}");
        }
        else
        {
          fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_serializefault(struct soap *soap)\n{\n\tif (soap->fault)\n\t\tsoap->fault->soap_serialize(soap);\n}");
          fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_putfault(struct soap *soap)\n{\n\tsoap_fault(soap);\n\tif (soap->fault)\n\t\treturn soap->fault->soap_put(soap, \"SOAP-ENV:Fault\", \"\");\n\treturn SOAP_EOM;\n}");
          fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_getfault(struct soap *soap)\n{\n\tsoap_fault(soap);\n\tif (soap->fault)\n\t\treturn soap->fault->soap_get(soap, \"SOAP-ENV:Fault\", NULL) == NULL;\n\treturn SOAP_EOM;\n}");
        }
        fprintf(fout, "\n\nSOAP_FMAC3 const char ** SOAP_FMAC4 soap_faultcode(struct soap *soap)\n{\n\tsoap_fault(soap);\n\tif (soap->fault == NULL)\n\t\treturn NULL;\n\tif (soap->version == 2 && soap->fault->SOAP_ENV__Code)\n\t\treturn (const char**)(void*)&soap->fault->SOAP_ENV__Code->SOAP_ENV__Value;\n\treturn (const char**)(void*)&soap->fault->faultcode;\n}");
        fprintf(fout, "\n\nSOAP_FMAC3 const char ** SOAP_FMAC4 soap_faultsubcode(struct soap *soap)\n{\n\tsoap_fault(soap);\n\tif (soap->fault == NULL)\n\t\treturn NULL;\n\tif (soap->version == 2 && soap->fault->SOAP_ENV__Code)\n\t{\tif (soap->fault->SOAP_ENV__Code->SOAP_ENV__Subcode == NULL)\n\t\t{\tsoap->fault->SOAP_ENV__Code->SOAP_ENV__Subcode = soap_new_SOAP_ENV__Code(soap, -1);\n\t\t\tif (soap->fault->SOAP_ENV__Code->SOAP_ENV__Subcode == NULL)\n\t\t\t\treturn NULL;\n\t\t}\n\t\treturn (const char**)(void*)&soap->fault->SOAP_ENV__Code->SOAP_ENV__Subcode->SOAP_ENV__Value;\n\t}\n\treturn (const char**)(void*)&soap->fault->faultcode;\n}");
        fprintf(fout, "\n\nSOAP_FMAC3 const char * SOAP_FMAC4 soap_fault_subcode(struct soap *soap)\n{\n\tconst char **s = soap_faultsubcode(soap);\n\treturn s ? *s : NULL;\n}");
        fprintf(fout, "\n\nSOAP_FMAC3 const char ** SOAP_FMAC4 soap_faultstring(struct soap *soap)\n{\n\tsoap_fault(soap);\n\tif (soap->fault == NULL)\n\t\treturn NULL;\n\tif (soap->version == 2 && soap->fault->SOAP_ENV__Reason)\n\t\treturn (const char**)(void*)&soap->fault->SOAP_ENV__Reason->SOAP_ENV__Text;\n\treturn (const char**)(void*)&soap->fault->faultstring;\n}");
        fprintf(fout, "\n\nSOAP_FMAC3 const char * SOAP_FMAC4 soap_fault_string(struct soap *soap)\n{\n\tconst char **s = soap_faultstring(soap);\n\treturn s ? *s : NULL;\n}");
        fprintf(fout, "\n\nSOAP_FMAC3 const char ** SOAP_FMAC4 soap_faultdetail(struct soap *soap)\n{\n\tsoap_fault(soap);\n\tif (soap->fault == NULL)\n\t\treturn NULL;");
        if (has_Detail_string())
          fprintf(fout, "\n\tif (soap->version == 2)\n\t{\tif (soap->fault->SOAP_ENV__Detail == NULL)\n\t\t\tsoap->fault->SOAP_ENV__Detail = soap_new_SOAP_ENV__Detail(soap, -1);\n\t\treturn (const char**)(void*)&soap->fault->SOAP_ENV__Detail->__any;\n\t}");
        if (has_detail_string())
          fprintf(fout, "\n\tif (soap->fault->detail == NULL)\n\t\tsoap->fault->detail = soap_new_SOAP_ENV__Detail(soap, -1);\n\treturn (const char**)(void*)&soap->fault->detail->__any;\n}");
        if (!has_detail_string() && !has_Detail_string())
          fprintf(fout, "\n\treturn NULL;\n}");
        fprintf(fout, "\n\nSOAP_FMAC3 const char * SOAP_FMAC4 soap_fault_detail(struct soap *soap)\n{\n\tconst char **s = soap_faultdetail(soap);\n\treturn s ? *s : NULL;\n}");
        fprintf(fout, "\n\n#endif");
        fprintf(fout, "\n\n#ifndef WITH_NOIDREF");
        fprintf(fout, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_getindependent(struct soap *soap)\n{");
        fprintf(fout, "\n\tint t;\n\tif (soap->version == 1)\n\t{\tfor (;;)\n\t\t{\tif (!soap_getelement(soap, NULL, &t))\n\t\t\t\tif ((soap->error && soap->error != SOAP_TAG_MISMATCH) || soap_ignore_element(soap))\n\t\t\t\t\tbreak;\n\t\t}\n\t}");
        fprintf(fout, "\n\tif (soap->error == SOAP_NO_TAG || soap->error == SOAP_EOF)");
        fprintf(fout, "\n\t\tsoap->error = SOAP_OK;");
        fprintf(fout, "\n\treturn soap->error;");
        fprintf(fout, "\n}\n#endif");

        if (!cflag && !namespaceid)
          fprintf(fout, "\n\n#ifdef __cplusplus\nextern \"C\" {\n#endif");
        fprintf(fout, "\nSOAP_FMAC3 void * SOAP_FMAC4 soap_getelement(struct soap *soap, const char *tag, int *type)\n{\t(void)type;");
        fprintf(fout, "\n\tif (soap_peek_element(soap))\n\t\treturn NULL;");
        fprintf(fout, "\n#ifndef WITH_NOIDREF\n\tif (!*soap->id || !(*type = soap_lookup_type(soap, soap->id)))\n\t\t*type = soap_lookup_type(soap, soap->href);");
        fprintf(fout, "\n\tswitch (*type)\n\t{");
        DBGLOG(fprintf(stderr, "\n Calling in_defs( )."));
        fflush(fout);
        in_defs();
        DBGLOG(fprintf(stderr, "\n Completed in_defs( )."));
        fprintf(fout, "\n\tdefault:\n#else\n\t*type = 0;\n#endif");
        fprintf(fout, "\n\t{\tconst char *t = soap->type;\n\t\tif (!*t)\n\t\t\tt = soap->tag;");
        fflush(fout);
        in_defs2();
        fprintf(fout, "\n\t\tt = soap->tag;");
        in_defs3();
        fprintf(fout, "\n#ifndef WITH_NOIDREF\n\t}\n#endif\n\t}\n\tsoap->error = SOAP_TAG_MISMATCH;\n\treturn NULL;\n}");
        if (!cflag && !namespaceid)
          fprintf(fout, "\n\n#ifdef __cplusplus\n}\n#endif");

        fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_ignore_element(struct soap *soap)\n{");
        fprintf(fout, "\n\tif (!soap_peek_element(soap))");
        fprintf(fout, "\n\t{\tint t;");
        fprintf(fout, "\n\t\tDBGLOG(TEST, SOAP_MESSAGE(fdebug, \"Unexpected element '%%s' in input at level = %%u body = %%d)\\n\", soap->tag, soap->level, soap->body));");
        fprintf(fout, "\n\t\tif (soap->mustUnderstand && !soap->other && !soap->fignore)");
        fprintf(fout, "\n\t\t\treturn soap->error = SOAP_MUSTUNDERSTAND;");
        /* old code: without option -s required SOAP_XML_STRICT to detect and flag extra elements;
           fprintf(fout, "\n\t\tif (((soap->mode & SOAP_XML_STRICT) && soap->part != SOAP_IN_HEADER) || !soap_match_tag(soap, soap->tag, \"SOAP-ENV:\"))\n\t\t{\tDBGLOG(TEST, SOAP_MESSAGE(fdebug, \"REJECTING element '%%s'\\n\", soap->tag));\n\t\t\treturn soap->error = SOAP_TAG_MISMATCH;\n\t\t}"); */
        fprintf(fout, "\n\t\tif ((%s!soap->fignore && soap->part != SOAP_IN_HEADER) || !soap_match_tag(soap, soap->tag, \"SOAP-ENV:\"))\n\t\t{\tDBGLOG(TEST, SOAP_MESSAGE(fdebug, \"REJECTING element '%%s'\\n\", soap->tag));\n\t\t\treturn soap->error = SOAP_TAG_MISMATCH;\n\t\t}", strict_check());
        fprintf(fout, "\n\t\tif (!*soap->id || !soap_getelement(soap, NULL, &t))");
        fprintf(fout, "\n\t\t{\tsoap->peeked = 0;");
        fprintf(fout, "\n\t\t\tif (soap->fignore)\n\t\t\t\tsoap->error = soap->fignore(soap, soap->tag);\n\t\t\telse\n\t\t\t\tsoap->error = SOAP_OK;");
        fprintf(fout, "\n\t\t\tDBGLOG(TEST, if (!soap->error) SOAP_MESSAGE(fdebug, \"IGNORING element '%%s'\\n\", soap->tag));");
        fprintf(fout, "\n\t\t\tif (!soap->error && soap->body && soap_ignore(soap))");
        fprintf(fout, "\n\t\t\t\treturn soap->error;");
        fprintf(fout, "\n\t\t}");
        fprintf(fout, "\n\t}");
        fprintf(fout, "\n\treturn soap->error;");
        fprintf(fout, "\n}");

        fprintf(fout, "\n\n#ifndef WITH_NOIDREF");
        fprintf(fout, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_putindependent(struct soap *soap)\n{\n\tint i;\n\tstruct soap_plist *pp;");
        fprintf(fout, "\n\tif (soap->version == 1 && soap->encodingStyle && !(soap->mode & (SOAP_XML_TREE | SOAP_XML_GRAPH)))");
        fprintf(fout, "\n\t\tfor (i = 0; i < SOAP_PTRHASH; i++)");
        fprintf(fout, "\n\t\t\tfor (pp = soap->pht[i]; pp; pp = pp->next)");
        fprintf(fout, "\n\t\t\t\tif (pp->mark1 == 2 || pp->mark2 == 2)");
        fprintf(fout, "\n\t\t\t\t\tif (soap_putelement(soap, pp->ptr, SOAP_MULTIREFTAG, pp->id, pp->type))\n\t\t\t\t\t\treturn soap->error;");
        fprintf(fout, "\n\treturn SOAP_OK;\n}\n#endif");

        if (!cflag && !namespaceid)
          fprintf(fout, "\n\n#ifdef __cplusplus\nextern \"C\" {\n#endif");
        fprintf(fout, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_putelement(struct soap *soap, const void *ptr, const char *tag, int id, int type)\n{\t(void)tag;");
        fprintf(fout, "\n\tswitch (type)\n\t{");
        fflush(fout);
        out_defs();
        fprintf(fout, "\n\tcase 0:\n\t\treturn SOAP_OK;\n\t}\n\tDBGLOG(TEST, SOAP_MESSAGE(fdebug, \"soap_putelement '%%s' failed for type %%d in %s\\n\", tag ? tag : \"\", type));", cstring(pathsoapC, 0));
        fprintf(fout, "\n\treturn soap_element_empty(soap, tag, 0, NULL); /* unknown type to serialize */\n}");
        if (!cflag && !namespaceid)
          fprintf(fout, "\n#ifdef __cplusplus\n}\n#endif");

        fprintf(fout, "\n\n#ifndef WITH_NOIDREF");
        if (!cflag && !namespaceid)
          fprintf(fout, "\n\n#ifdef __cplusplus\nextern \"C\" {\n#endif");
        if (is_anytype_flag)
        {
          fprintf(fout, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_markelement(struct soap *soap, const void *ptr, int type)\n{");
          fprintf(fout, "\n\t(void)soap; (void)ptr; (void)type; /* appease -Wall -Werror */");
          fprintf(fout, "\n\tswitch (type)\n\t{");
          fflush(fout);
          mark_defs();
          fprintf(fout, "\n\t}\n}");
        }
        else
        {
          fprintf(fout, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_markelement(struct soap *soap, const void *ptr, int type)\n{");
          fprintf(fout, "\n\t(void)soap; (void)ptr; (void)type; /* appease -Wall -Werror */");
          fprintf(fout, "\n}");
        }
        if (!cflag && !namespaceid)
          fprintf(fout, "\n#ifdef __cplusplus\n}\n#endif");
        fprintf(fout, "\n#endif");
        if (!cflag && !namespaceid)
          fprintf(fout, "\n\n#ifdef __cplusplus\nextern \"C\" {\n#endif");
        fprintf(fout, "\n\nSOAP_FMAC3 void * SOAP_FMAC4 soap_dupelement(struct soap *soap, const void *ptr, int type)\n{");
        if (Ecflag)
        {
          fprintf(fout, "\n\tif (!ptr)\n\t\treturn NULL;");
          fprintf(fout, "\n\tswitch (type)\n\t{");
          fflush(fout);
          dup_defs();
          fprintf(fout, "\n\t}");
        }
        else
        {
          fprintf(fout, "(void)soap; (void)ptr; (void)type; /* appease -Wall -Werror */");
        }
        fprintf(fout, "\n\treturn NULL;\n}");
        if (!cflag && !namespaceid)
          fprintf(fout, "\n#ifdef __cplusplus\n}\n#endif");
        if (!cflag && !namespaceid)
          fprintf(fout, "\n\n#ifdef __cplusplus\nextern \"C\" {\n#endif");
        fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_delelement(const void *ptr, int type)\n{");
        if (Edflag)
        {
          fprintf(fout, "\n\tif (!ptr)\n\t\treturn;");
          fprintf(fout, "\n\tswitch (type)\n\t{");
          fflush(fout);
          del_defs();
          fprintf(fout, "\n\t}");
        }
        else
        {
          fprintf(fout, "(void)ptr; (void)type; /* appease -Wall -Werror */");
        }
        fprintf(fout, "\n}");
        if (!cflag && !namespaceid)
          fprintf(fout, "\n#ifdef __cplusplus\n}\n#endif");
      }

      if (!cflag)
      {
        fprintf(fhead, "\nSOAP_FMAC3 void * SOAP_FMAC4 %s_instantiate(struct soap*, int, const char*, const char*, size_t*);", prefix);
        fprintf(fout, "\n\nSOAP_FMAC3 void * SOAP_FMAC4 %s_instantiate(struct soap *soap, int t, const char *type, const char *arrayType, size_t *n)\n{\t(void)type;\n\tswitch (t)\n\t{", prefix);
        if (classtable)
        {
          for (p = classtable->list; p; p = p->next)
          {
            if ((p->info.typ->type == Tclass || p->info.typ->type == Tstruct) && !is_transient(p->info.typ))
            {
              if (is_header_or_fault(p->info.typ) || is_body(p->info.typ))
                fprintf(fout, "\n#ifndef WITH_NOGLOBAL");
              fprintf(fout, "\n\tcase %s:\n\t\treturn (void*)%s_instantiate_%s(soap, -1, type, arrayType, n);", soap_type(p->info.typ), fprefix, c_ident(p->info.typ));
              if (is_header_or_fault(p->info.typ) || is_body(p->info.typ))
                fprintf(fout, "\n#endif");
            }
          }
        }
        if (typetable)
        {
          for (p = typetable->list; p; p = p->next)
          {
            if ((p->info.typ->type == Tclass || p->info.typ->type == Tstruct) && !is_transient(p->info.typ))
            {
              if (is_header_or_fault(p->info.typ) || is_body(p->info.typ))
                fprintf(fout, "\n#ifndef WITH_NOGLOBAL");
              fprintf(fout, "\n\tcase %s:\n\t\treturn (void*)%s_instantiate_%s(soap, -1, type, arrayType, n);", soap_type(p->info.typ), fprefix, c_ident(p->info.typ));
              if (is_header_or_fault(p->info.typ) || is_body(p->info.typ))
                fprintf(fout, "\n#endif");
            }
          }
        }
        for (typ = Tptr[Ttemplate]; typ; typ = typ->next)
          if (typ->ref && !is_transient(typ))
            fprintf(fout, "\n\tcase %s:\n\t\treturn (void*)%s_instantiate_%s(soap, -1, type, arrayType, n);", soap_type(typ), fprefix, c_ident(typ));

        fprintf(fout, "\n\t}\n\treturn NULL;\n}");

        fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 %s_fdelete(struct soap *soap, struct soap_clist*);", prefix);
        fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 %s_fdelete(struct soap *soap, struct soap_clist *p)", prefix);
        fprintf(fout, "\n{\n\t(void)soap; /* appease -Wall -Werror */\n\tif (!p->ptr)\n\t\treturn SOAP_OK;\n\tswitch (p->type)\n\t{");
        if (classtable)
        {
          for (p = classtable->list; p; p = p->next)
          {
            if ((p->info.typ->type == Tclass || p->info.typ->type == Tstruct) && !is_transient(p->info.typ))
            {
              if (is_header_or_fault(p->info.typ) || is_body(p->info.typ))
                fprintf(fout, "\n#ifndef WITH_NOGLOBAL");
              fprintf(fout, "\n\tcase %s:", soap_type(p->info.typ));
              fprintf(fout, "\n\t\tif (p->size < 0)\n\t\t\tSOAP_DELETE(soap, static_cast<%s*>(p->ptr), %s);\n\t\telse\n\t\t\tSOAP_DELETE_ARRAY(soap, static_cast<%s*>(p->ptr), %s);\n\t\tbreak;", c_type(p->info.typ), c_type(p->info.typ), c_type(p->info.typ), c_type(p->info.typ));
              if (is_header_or_fault(p->info.typ) || is_body(p->info.typ))
                fprintf(fout, "\n#endif");
            }
          }
        }

        if (typetable)
        {
          for (p = typetable->list; p; p = p->next)
            if (p->info.typ->type == Tclass || p->info.typ->type == Tstruct) /* && is_external(p->info.typ)) */
            {
              if (is_header_or_fault(p->info.typ) || is_body(p->info.typ))
                fprintf(fout, "\n#ifndef WITH_NOGLOBAL");
              fprintf(fout, "\n\tcase %s:", soap_type(p->info.typ));
              fprintf(fout, "\n\t\tif (p->size < 0)\n\t\t\tSOAP_DELETE(soap, static_cast<%s*>(p->ptr), %s);\n\t\telse\n\t\t\tSOAP_DELETE_ARRAY(soap, static_cast<%s*>(p->ptr), %s);\n\t\tbreak;", c_type(p->info.typ), c_type(p->info.typ), c_type(p->info.typ), c_type(p->info.typ));
              if (is_header_or_fault(p->info.typ) || is_body(p->info.typ))
                fprintf(fout, "\n#endif");
            }
        }
        for (typ = Tptr[Ttemplate]; typ; typ = typ->next)
        {
          if (typ->ref && !is_transient(typ))
          {
            fprintf(fout, "\n\tcase %s:", soap_type(typ));
            fprintf(fout, "\n\t\tif (p->size < 0)\n\t\t\tSOAP_DELETE(soap, static_cast<%s*>(p->ptr), %s);\n\t\telse\n\t\t\tSOAP_DELETE_ARRAY(soap, static_cast<%s*>(p->ptr), %s);\n\t\tbreak;", c_type(typ), c_type(typ), c_type(typ), c_type(typ));
          }
        }
        fprintf(fout, "\n\tdefault:\n\t\treturn SOAP_ERR;");
        fprintf(fout, "\n\t}\n\treturn SOAP_OK;");
        fprintf(fout, "\n}");

        fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 %s_fbase(int, int);", prefix);
        fprintf(fout, "\n\n#ifdef WIN32\n#pragma warning(push)\n// do not warn on switch w/o cases\n#pragma warning(disable:4065)\n#endif");
        fprintf(fout, "\nSOAP_FMAC3 int SOAP_FMAC4 %s_fbase(int t, int b)", prefix);
        found = 0;
        if (classtable)
        {
          for (p = classtable->list; p; p = p->next)
          {
            if ((p->info.typ->type == Tclass || p->info.typ->type == Tstruct) && p->info.typ->baseid && !is_transient(p->info.typ))
            {
              Entry *e = entry(classtable, p->info.typ->baseid);
              if (e && !is_transient(e->info.typ))
              {
                found = 1;
                break;
              }
            }
          }
        }
        if (found)
        {
          fprintf(fout, "\n{\n\tdo\n\t{\tswitch (t)\n\t\t{\n");
          for (p = classtable->list; p; p = p->next)
          {
            if ((p->info.typ->type == Tclass || p->info.typ->type == Tstruct) && p->info.typ->baseid && !is_transient(p->info.typ))
            {
              Entry *e = entry(classtable, p->info.typ->baseid);
              if (e && !is_transient(e->info.typ))
                fprintf(fout, "\n\t\tcase %s: t = %s; break;", soap_type(p->info.typ), soap_type(e->info.typ));
            }
          }
          fprintf(fout, "\n\t\tdefault: return 0;\n\t\t}\n\t}\n\twhile (t != b);\n\treturn 1;\n}");
        }
        else
          fprintf(fout, "\n{\n\t(void)t; (void)b; /* appease -Wall -Werror */\n\treturn 0;\n}");
        fprintf(fout, "\n#ifdef WIN32\n#pragma warning(pop)\n#endif");

        if (Tptr[Ttemplate] || classtable)
        {
          fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 %s_finsert(struct soap*, int, int, void*, size_t, const void*, void**);", prefix);
          fprintf(fout, "\n\n#ifndef WITH_NOIDREF");
          fprintf(fout, "\n#ifdef WIN32\n#pragma warning(push)\n// do not warn on switch w/o cases\n#pragma warning(disable:4065)\n#endif");
          fprintf(fout, "\nSOAP_FMAC3 void SOAP_FMAC4 %s_finsert(struct soap *soap, int t, int tt, void *p, size_t index, const void *q, void **x)", prefix);
          fprintf(fout, "\n{\n\t(void)soap; (void)t; (void)p; (void)index; (void)q; (void)x; /* appease -Wall -Werror */");
          fprintf(fout, "\n\tswitch (tt)\n\t{");
          for (typ = Tptr[Ttemplate]; typ; typ = typ->next)
          {
            if (typ->ref && !is_transient(typ))
            {
              fprintf(fout, "\n\tcase %s:", soap_type(typ));
              if (is_smart(typ))
              {
                Tnode *ref = (Tnode*)typ->ref;
                if (ref->type == Tclass)
                  fprintf(fout, "\n\t\tif (t == %s || %s_fbase(t, %s))\n\t\t{\tDBGLOG(TEST, SOAP_MESSAGE(fdebug, \"Smart pointer %s type=%%d location=%%p object=%%p\\n\", t, p, q));", soap_type(reftype(typ->ref)), prefix, soap_type(reftype(typ->ref)), c_type(typ));
                else
                  fprintf(fout, "\n\t\tif (t == %s)\n\t\t{\tDBGLOG(TEST, SOAP_MESSAGE(fdebug, \"Smart pointer %s type=%%d location=%%p object=%%p\\n\", t, p, q));", soap_type(reftype(typ->ref)), c_type(typ));
                if (is_smart_shared(typ))
                {
                  fprintf(fout, "\n\t\t\tif (!*x)\n\t\t\t\t*(%s)(*x = (void*)p) = %s<%s>(**(%s)q);", c_type_id(typ, "*"), make_shared(typ), c_type(typ->ref), c_type_id(typ->ref, "**"));
                  fprintf(fout, "\n\t\t\telse\n\t\t\t\t*(%s)p = *(%s)(*x);", c_type_id(typ, "*"), c_type_id(typ, "*"));
                }
                else
                {
                  fprintf(fout, "\n\t\t\t*(*(%s)p = %s(SOAP_NEW(soap, %s))) = **(%s)q;", c_type_id(typ, "*"), c_type(typ), c_type(typ->ref), c_type_id(typ->ref, "**"));
                }
              }
              else
              {
                Tnode *ref = (Tnode*)typ->ref;
                if (ref->type == Tpointer && ((Tnode*)ref->ref)->type == Tclass)
                  fprintf(fout, "\n\t\tif (t == %s || %s_fbase(t, %s))\n\t\t{\tDBGLOG(TEST, SOAP_MESSAGE(fdebug, \"Container %s insert type=%%d in %%d location=%%p object=%%p at index=%%lu\\n\", t, tt, p, q, (unsigned long)index));", soap_type(reftype(ref)), prefix, soap_type(reftype(ref)), c_type(typ));
                else
                  fprintf(fout, "\n\t\tif (t == %s)\n\t\t{\tDBGLOG(TEST, SOAP_MESSAGE(fdebug, \"Container %s insert type=%%d in %%d location=%%p object=%%p at index=%%lu\\n\", t, tt, p, q, (unsigned long)index));", soap_type(reftype(ref)), c_type(typ));
                if (is_smart(ref) && strcmp(typ->id->name, "std::set"))
                {
                  if (!strcmp(ref->id->name, "std::vector") || !strcmp(ref->id->name, "std::deque"))
                    fprintf(fout, "\n\t\t\t%s::iterator i = ((%s)p)->begin() + index;", c_type(typ), c_type_id(typ, "*"));
                  else
                    fprintf(fout, "\n\t\t\t%s::iterator i = ((%s)p)->begin();\n\t\t\twhile (index-- > 0)\n\t\t\t\t++i;", c_type(typ), c_type_id(typ, "*"));
                  if (is_smart_shared(ref))
                    fprintf(fout, "\n\t\t\tif (!*x)\n\t\t\t\t*(%s)(*x = &*i) = %s<%s>(**(%s)q);\n\t\t\telse\n\t\t\t\t*i = *(%s)*x;", c_type_id(ref, "*"), make_shared(ref), c_type(ref->ref), c_type_id(ref->ref, "**"), c_type_id(ref, "*"));
                  else
                    fprintf(fout, "\n\t\t\t*(*i = %s(SOAP_NEW(soap, %s))) = **(%s)q;", c_type(ref), c_type(ref->ref), c_type_id(ref->ref, "**"));
                }
                else
                {
                  if (!strcmp(typ->id->name, "std::vector") || !strcmp(typ->id->name, "std::deque"))
                    fprintf(fout, "\n\t\t\t(*(%s)p)[index] = *(%s)q;", c_type_id(typ, "*"), c_type_id(ref, "*"));
                  else if (!strcmp(typ->id->name, "std::set"))
                    fprintf(fout, "\n\t\t\t((%s)p)->insert(*(%s)q);", c_type_id(typ, "*"), c_type_id(ref, "*"));
                  else
                    fprintf(fout, "\n\t\t\t%s::iterator i = ((%s)p)->begin();\n\t\t\twhile (index-- > 0)\n\t\t\t\t++i;\n\t\t\t*i = *(%s)q;", c_type(typ), c_type_id(typ, "*"), c_type_id(ref, "*"));
                }
              }
              fprintf(fout, "\n\t\t}\n\t\tbreak;");
            }
          }
          if (classtable)
          {
            for (p = classtable->list; p; p = p->next)
            {
              if (!is_transient(p->info.typ))
              {
                if (is_header_or_fault(p->info.typ) || is_body(p->info.typ))
                  fprintf(fout, "\n#ifndef WITH_NOGLOBAL");
                fprintf(fout, "\n\tcase %s:", soap_type(p->info.typ));
                fprintf(fout, "\n\t\tDBGLOG(TEST, SOAP_MESSAGE(fdebug, \"Copy %s type=%%d location=%%p object=%%p\\n\", t, p, q));", c_type(p->info.typ));
                fprintf(fout, "\n\t\t*(%s*)p = *(%s*)q;\n\t\tbreak;", c_type(p->info.typ), c_type(p->info.typ));
                if (is_header_or_fault(p->info.typ) || is_body(p->info.typ))
                  fprintf(fout, "\n#endif");
              }
            }
          }
          if (typetable)
          {
            for (p = typetable->list; p; p = p->next)
            {
              if ((p->info.typ->type == Tclass || p->info.typ->type == Tstruct) && !is_transient(p->info.typ))
              {
                if (is_header_or_fault(p->info.typ) || is_body(p->info.typ))
                  fprintf(fout, "\n#ifndef WITH_NOGLOBAL");
                fprintf(fout, "\n\tcase %s:", soap_type(p->info.typ));
                fprintf(fout, "\n\t\tDBGLOG(TEST, SOAP_MESSAGE(fdebug, \"Copy %s type=%%d location=%%p object=%%p\\n\", t, p, q));", c_type(p->info.typ));
                fprintf(fout, "\n\t\t*(%s*)p = *(%s*)q;\n\t\tbreak;", c_type(p->info.typ), c_type(p->info.typ));
                if (is_header_or_fault(p->info.typ) || is_body(p->info.typ))
                  fprintf(fout, "\n#endif");
              }
            }
          }
          fprintf(fout, "\n\tdefault:\n\t\tDBGLOG(TEST, SOAP_MESSAGE(fdebug, \"Could not insert type=%%d in %%d\\n\", t, tt));");
          fprintf(fout, "\n\t}");
          fprintf(fout, "\n}");
          fprintf(fout, "\n#ifdef WIN32\n#pragma warning(pop)\n#endif");
          fprintf(fout, "\n#endif");
        }
      }
    }

    generate_defs();

    if (namespaceid)
      fprintf(fout, "\n\n} // namespace %s\n", namespaceid);
    fprintf(fout, "\n\n#if defined(__BORLANDC__)");
    fprintf(fout, "\n#pragma option pop");
    fprintf(fout, "\n#pragma option pop");
    fprintf(fout, "\n#endif");
    fprintf(fout, "\n\n/* End of %s */\n", soapC);
    if (fclose(fout))
      execerror("Cannot write to file");
  }

  if (namespaceid)
    fprintf(fhead, "\n\n} // namespace %s\n", namespaceid);
  fprintf(fhead, "\n\n#endif");
  fprintf(fhead, "\n\n/* End of %s */\n", soapH);
  if (fclose(fhead))
    execerror("Cannot write to file");

  if (namespaceid)
    fprintf(fheader, "\n\n} // namespace %s\n", namespaceid);
  fprintf(fheader, "\n\n#endif");
  fprintf(fheader, "\n\n/* End of %s */\n", soapStub);
  if (fclose(fheader))
    execerror("Cannot write to file");

  if (mflag)
  {
    DBGLOG(fprintf(stderr, "\n Calling matlab_def_table( )."));
    matlab_def_table(table);
    DBGLOG(fprintf(stderr, "\n Completed matlab_def_table( )."));
    if (fclose(fmatlab) || fclose(fmheader))
      execerror("Cannot write to file");
  }

  if (!Sflag && !iflag && !jflag)
  {
    if (namespaceid)
      fprintf(fclient, "\n\n} // namespace %s\n", namespaceid);
    fprintf(fclient, "\n\n#if defined(__BORLANDC__)");
    fprintf(fclient, "\n#pragma option pop");
    fprintf(fclient, "\n#pragma option pop");
    fprintf(fclient, "\n#endif");
    fprintf(fclient, "\n\n/* End of %s */\n", soapClient);
    if (fclose(fclient))
      execerror("Cannot write to file");
  }

  if (!Cflag && !iflag && !jflag)
  {
    if (namespaceid)
      fprintf(fserver, "\n\n} // namespace %s\n", namespaceid);
    fprintf(fserver, "\n\n#if defined(__BORLANDC__)");
    fprintf(fserver, "\n#pragma option pop");
    fprintf(fserver, "\n#pragma option pop");
    fprintf(fserver, "\n#endif");
    fprintf(fserver, "\n\n/* End of %s */\n", soapServer);
    if (fclose(fserver))
      execerror("Cannot write to file");
  }

  if (rflag)
  {
    char tmp[256];
    time_t T = time(NULL);
    strftime(tmp, 256, "%a %b %d %Y %H:%M:%S UTC", gmtime(&T));
    fprintf(freport, "\n  [1]: https://www.genivia.com/images/go-up.png\n\n");
    fprintf(freport, "--------------------------------------------------------------------------------\n\n_Generated on %s by soapcpp2 v" VERSION " for %s._\n_The gSOAP XML Web services tools are Copyright (C) Robert van Engelen, Genivia Inc. All Rights Reserved._\n", tmp, filename);
    if (fclose(freport))
      execerror("Cannot write to file");
  }
}

void
gen_class(FILE *fd, Entry *p)
{
  Entry *q;
  Tnode *typ = p->info.typ;
  const char *x, *s;
  const char *nse = ns_qualifiedElement(typ);
  const char *nsa = ns_qualifiedAttribute(typ);
  if (!typ->ref)
  {
    if (!is_transient(typ) && !is_external(typ) && !is_volatile(typ) && !is_imported(typ))
    {
      sprintf(errbuf, "%s declared at %s:%d has no content, requires members", c_type(typ), p->filename, p->lineno);
      semwarn(errbuf);
    }
    return;
  }
  if (fd == freport)
  {
    fprintf(freport, "<a name=\"%s\"></a>\n\n", c_ident(typ));
    fprintf(freport, "### `%s`\n\n", c_type(typ));
    if (soap_version < 0 && (is_header_or_fault(typ) || is_body(typ)))
    {
      fprintf(freport, "This %s is for internal use only\n\n", typ->type == Tstruct ? "struct" : typ->type == Tclass ? "class" : "union");
      return;
    }
    else
    {
      fprintf(freport, "This %s is declared in [%s](%s) at line %d, ", typ->type == Tstruct ? "struct" : typ->type == Tclass ? "class" : "union", p->filename, p->filename, p->lineno);
    }
  }
  if (is_imported(typ))
    return;
  x = xsi_type(typ);
  if (!x || !*x)
    x = wsdl_type(typ, "");
  if (fd != freport)
  {
    fprintf(fd, "\n\n/* %s:%d */", p->filename, p->lineno);
    if (is_header_or_fault(typ) || is_body(typ))
      fprintf(fd, "\n#ifndef WITH_NOGLOBAL");
    fprintf(fd, "\n#ifndef %s\n", soap_type(typ));
    if (namespaceid)
      fprintf(fd, "#define %s (-%d)\n", soap_type(typ), typ->num); /* namespaced SOAP_TYPE is negative to avoid clashes */
    else
      fprintf(fd, "#define %s (%d)\n", soap_type(typ), typ->num);
  }
  if (typ->recursive)
  {
    if (fd == freport)
      fprintf(freport, "is recursive, meaning it may (in)directly reference itself through its (base or derived class) members, and ");
    else
      fprintf(fd, "/* Type %s is a recursive data type, (in)directly referencing itself through its (base or derived class) members */\n", typ->id->name);
  }
  if (is_volatile(typ))
  {
    if (fd == freport)
      fprintf(freport, "is volatile, meaning it is declared external of the data binding interface and not redeclared here");
    else
      fprintf(fd, "#if 0 /* Volatile: declared external of the data binding interface and not redeclared here */\n");
  }
  else if (is_transient(typ))
  {
    if (fd == freport)
      fprintf(freport, "is transient, meaning not serializable");
    else
      fprintf(fd, "/* Transient type: */\n");
  }
  else if (is_invisible(typ->id->name))
  {
    if (fd == freport)
      fprintf(freport, "is a wrapper, meaning that it wraps data and is not visible in XML");
    else
      fprintf(fd, "/* Wrapper: */\n");
    x = "";
  }
  else if (is_attachment(typ))
  {
    if (fd == freport)
      fprintf(freport, "is an attachment, meaning binary data attached as a MTOM/MIME/DIME attachment or included as *`xsd:base64Binary`* base64 with `__ptr` pointing to the data of length `__size` and `id`, `type`, and `options` identifying and describing the attachment data (use the `SOAP_ENC_MTOM` or the `SOAP_ENC_MIME` flag and set at least `type` to create an attachment )");
    else
      fprintf(fd, "/* binary data attached as MTOM/MIME/DIME attachment or included as *`xsd:base64Binary`* base64: */\n");
  }
  else if (is_hexBinary(typ))
  {
    if (fd == freport)
      fprintf(freport, "is binary, meaning serialized as *`xsd:hexBinary`* hex binary data with `__ptr` pointing to the data of length `__size`");
    else
      fprintf(fd, "/* hexBinary XML schema type: */\n");
  }
  else if (is_binary(typ))
  {
    if (fd == freport)
      fprintf(freport, "is binary, meaning serialized as *`xsd:base64Binary`* base64 binary data with `__ptr` pointing to the data of length `__size`");
    else
      fprintf(fd, "/* base64Binary XML schema type: */\n");
  }
  else if (is_discriminant(typ))
  {
    if (fd == freport)
      fprintf(freport, "is a choice, meaning a union with a union variant selector");
    else
      fprintf(fd, "/* Choice: */\n");
  }
  else if ((q = is_dynamic_array(typ)))
  {
    if (has_ns(typ) || is_untyped(typ))
    {
      if (fd == freport)
        fprintf(freport, "is a sequence of elements, meaning that it has a pointer member `%s` pointing to an array of `%s` values and has an array size `%s`", ident(q->sym->name), c_type(q->info.typ->ref), ident(q->next->sym->name));
      else
        fprintf(fd, "/* Sequence of %s schema type: */\n", x);
    }
    else
    {
      if (fd == freport)
      {
        int d = get_Darraydims(typ);
        if (!d)
          fprintf(freport, "is a SOAP-encoded array, meaning that the type name lacks a prefix and it has a member `%s` pointing to an array of `%s` values and has an array size member `%s`", ident(q->sym->name), c_type(q->info.typ->ref), ident(q->next->sym->name));
        else
          fprintf(freport, "is a %dD SOAP-encoded array, meaning that the type name lacks a prefix and it has a member `%s` pointing to an array of `%s` values and has an array `%s[%d]` of sizes for each dimension", d, ident(q->sym->name), c_type(q->info.typ->ref), ident(q->next->sym->name), d);
      }
      else
      {
        if (!eflag && soap_version >= 0)
        {
          sprintf(errbuf, "SOAP-encoded array '%s' is specific to SOAP encoding only and not compliant with WS-I Basic Profile 1.0a", c_type(typ));
          compliancewarn(errbuf);
        }
        fprintf(fd, "/* SOAP encoded array of %s schema type: */\n", x);
      }
    }
  }
  else if (is_primclass(typ))
  {
    if (fd == freport)
      fprintf(freport, "is a simple content wrapper *`%s`*, meaning it wraps a primitive type", x);
    else
      fprintf(fd, "/* simple XML schema type '%s': */\n", x);
  }
  else if (is_header_or_fault(typ))
  {
    if (fd == freport)
    {
      s = ident(typ->id->name);
      fprintf(freport, "is the SOAP protocol *`<%s>`* element", ns_convert(p->sym->name));
      if (!strcmp(s, "SOAP_ENV__Header"))
      {
        int flag = 0;
        fprintf(freport, " with message-specific child elements that are mandatory to process when attributed with *`mustUnderstand=\"true\"`*.  Headers are usually added and processed by plugins.  To remove the SOAP Header when sending or returning a message, set `soap->header = NULL`.  Use `soap_header(struct soap *soap)` to allocate a `%s` which will be pointed to by `soap->header`, then initialize it with `soap_default_SOAP_ENV__Header(soap, soap->header)` and set one or more of its data members (if any):\n\n", c_type(typ));
        if (typ->ref && ((Table*)typ->ref)->list)
        {
          for (q = ((Table*)typ->ref)->list; q; q = q->next)
          {
            if (q->info.typ->type != Tfun && !is_transient(q->info.typ))
            {
              fprintf(fd, "- `%s%s` ", c_storage(q->info.sto), c_type_id(q->info.typ, q->sym->name));
              if (q->info.sto & SmustUnderstand)
                fprintf(fd, "element %s with mustUnderstand=\"true\">", ns_add(q, nse));
              else
                fprintf(fd, "element %s", ns_add(q, nse));
              if ((q->info.typ->type == Tclass || q->info.typ->type == Tstruct || q->info.typ->type == Tenum || q->info.typ->type == Tenumsc || is_typedef(q->info.typ)) && !is_stdstr(q->info.typ))
                fprintf(freport, ", see also <code><a href=\"#%s\"> %s </a></code>", ident(q->info.typ->id->name), ident(q->info.typ->id->name));
              fprintf(freport, "\n");
            }
          }
          flag = 1;
        }
        if (!flag)
          fprintf(freport, "*No SOAP headers are applicable*");
        fprintf(freport, "\n\nThis struct will be auto-generated when it is not explicitly declared in an interface header file,");
      }
      else if (!strcmp(s, "SOAP_ENV__Fault"))
      {
        fprintf(freport, " with fault information and details returned by a service that triggered the error.  At the server side, a fault can be explicitly set within a service operation by calling and returning:\n\n- `int soap_sender_fault(struct soap *soap, const char *faultstring, const char *XML)` return this error code when the sender is at fault (irrecoverable)\n- `int soap_receiver_fault(struct soap *soap, const char *faultstring, const char *XML)` return this error code when the receiver is at fault (recoverable, sender may retry)\n\nA service operation may also return an HTTP status or error code (200 to 599).\n\nAt the client side the (proxy) call returns the error code which is also stored in `soap->error`.  The fault structure is pointed to by `soap->fault`.  The fault can be displayed with:\n\n- `void soap_print_fault(struct soap *soap, FILE *fd)` display fault\n- `void soap_print_fault_location(struct soap *soap, FILE *fd)` display the location of the fault in the XML message that caused it\n- `void soap_sprint_fault(struct soap *soap, char *buf, size_t len)` write fault to buffer\n- `void soap_stream_fault(struct soap *soap, std::ostream&)` write fault to stream\n- `const char *soap_fault_subcode(struct soap *soap)` returns the SOAP Fault subcode QName string or NULL when absent\n- `const char *soap_fault_string(struct soap *soap)` returns the SOAP Fault string/reason or NULL when absent\n- `const char *soap_fault_detail(struct soap *soap)` returns the SOAP Fault detail XML string or NULL when absent\n- `const char **soap_faultsubcode(struct soap *soap)` returns a pointer to the SOAP Fault to set this QName string\n- `const char **soap_faultstring(struct soap *soap)` returns a pointer to the SOAP Fault string/reason to set this string\n- `const char **soap_faultdetail(struct soap *soap)` returns a pointer to the SOAP Fault detail XML string to set this string or returns NULL when not accessible");
        fprintf(freport, "\n\nThis struct will be auto-generated when it is not explicitly declared in an interface header file");
      }
      else if (!strcmp(s, "SOAP_ENV__Detail"))
      {
        int flag = 0;
        fprintf(freport, " with details returned by a service that triggered the error.  Fault details are added and processed by plugins by setting the `detail` (for SOAP 1.1) or `SOAP_ENV__Detail` (for SOAP 1.2) member of `struct SOAP_ENV__Fault` and then setting one ore more of the detail members:\n\n");
        if (typ->ref && ((Table*)typ->ref)->list)
        {
          for (q = ((Table*)typ->ref)->list; q; q = q->next)
          {
            if (q->info.typ->type != Tfun && !is_transient(q->info.typ))
            {
              fprintf(fd, "- `%s%s` ", c_storage(q->info.sto), c_type_id(q->info.typ, q->sym->name));
              if (is_XML(q->info.typ) && is_invisible(q->sym->name))
                fprintf(fd, "catch-all XML in literal XML string");
              else if (is_XML(q->info.typ))
                fprintf(fd, "element *`<%s>`* with XML in literal XML string", ns_add(q, nse));
              else if (is_anyType(q->info.typ))
                fprintf(fd, "catch-all XML in DOM");
              else if (is_anytype(q))
              {
                if (namespaceid)
                  fprintf(fd, "element *`<%s>`* serialized with C/C++ type `%s` = `SOAP_TYPE_%s_<Type>`", ns_add(q->next, nse), ident(q->sym->name), namespaceid);
                else
                  fprintf(fd, "element *`<%s>`* serialized with C/C++ type `%s` = `SOAP_TYPE_<Type>`", ns_add(q->next, nse), ident(q->sym->name));
                q = q->next;
              }
              else
                fprintf(fd, "element *`<%s>`*", ns_add(q, nse));
              if ((q->info.typ->type == Tclass || q->info.typ->type == Tstruct || q->info.typ->type == Tenum || q->info.typ->type == Tenumsc || is_typedef(q->info.typ)) && !is_stdstr(q->info.typ))
                fprintf(freport, ", see also <code><a href=\"#%s\"> %s </a></code>", ident(q->info.typ->id->name), ident(q->info.typ->id->name));
              fprintf(fd, "\n");
            }
          }
          flag = 1;
        }
        if (!flag)
          fprintf(freport, "No SOAP Fault details are applicable");
        fprintf(freport, "\n\nThis struct will be auto-generated when it is not explicitly declared in an interface header file,");
      }
      else
      {
        fprintf(freport, ".  This struct is for internal use and will be auto-generated when not explicitly declared.\n\n");
        gen_report_hr();
        return;
      }
    }
    else
      fprintf(fd, "/* %s: */\n", ident(typ->id->name));
  }
  else if (typ->type == Tunion)
  {
    if (fd == freport)
      fprintf(freport, "is serializable, but only when used as a member of a struct or class with a union variant selector");
    else
      fprintf(fd, "/* union serializable only when used as a member of a struct or class with a union variant selector */\n");
  }
  else if (x && *x)
  {
    if (fd == freport)
      fprintf(freport, "is serialized as XML schema type *`%s`*", x);
    else
      fprintf(fd, "/* complex XML schema type '%s': */\n", x);
  }
  else if (fd == freport)
    fprintf(freport, "is serializable");
  if (fd == freport && is_mutable(p))
    fprintf(freport, " and is declared mutable, meaning that multiple declarations in interface header files are collected into one structure");
  if (fd == freport && typ->type == Tstruct)
  {
    int flag = 0;
    Tnode *r;
    for (r = Tptr[Tfun]; r; r = r->next)
    {
      if (!strcmp(r->id->name, p->sym->name))
      {
        if (*x)
          fprintf(freport, " and is internally used as the operation request element *`<%s>`* with the request parameters of service operation `%s()`", x, ident(r->id->name));
        else
          fprintf(freport, " and is internally used to wrap the request element(s) of the service operation `%s()`", ident(r->id->name));
        flag = 1;
      }
      else if (r->response == p)
      {
        fprintf(freport, " and is internally used as the operation response element *`<%s>`* with the response parameters of service operation `%s()`", x, ident(r->id->name));
        flag = 1;
      }
    }
    if (flag)
    {
      fprintf(freport, ".\n\n");
      gen_report_hr();
      return;
    }
  }
  fflush(fd);
  if (fd == freport)
    fprintf(freport, ", and has the following auto-completed declaration in %s:\n", soapStub);
  if (typ->type == Tstruct)
  {
    int permission = -1;
    int flag = 0;
    DBGLOG(fprintf(stderr, "\nstruct %s\n", typ->id->name));
    if (cflag)
    {
      if (fd == freport)
        fprintf(freport, "\n    struct %s {", ident(typ->id->name));
      else
        fprintf(fd, "struct %s {", ident(typ->id->name));
    }
    else
    {
      if (fd == freport)
        fprintf(freport, "\n    struct SOAP_CMAC %s {", ident(typ->id->name));
      else
        fprintf(fd, "struct SOAP_CMAC %s {", ident(typ->id->name));
    }
    for (q = ((Table*)typ->ref)->list; q; q = q->next)
    {
      if (!cflag && permission != (q->info.sto & (Sprivate | Sprotected)))
      {
        if (q->info.sto & Sprivate)
          fprintf(fd, "\n      private:");
        else if (q->info.sto & Sprotected)
          fprintf(fd, "\n      protected:");
        else
          fprintf(fd, "\n      public:");
        permission = (q->info.sto & (Sprivate | Sprotected));
      }
      if (cflag && q->info.typ->type == Tfun)
        continue;
      if (cflag && (q->info.sto & Stypedef))
        continue;
      if (flag)
        flag = 0;
      else if (q->info.typ->type == Tfun || (q->info.sto & Stypedef))
        ;
      else if (is_anyAttribute(q->info.typ))
        fprintf(fd, "\n        /** XML DOM attribute list */");
      else if (q->info.sto & Sattribute)
      {
        if (q->info.minOccurs >= 1)
          fprintf(fd, "\n        /** Required attribute '%s' of XML schema type '%s' */", ns_add(q, nsa), wsdl_type(q->info.typ, ""));
        else
          fprintf(fd, "\n        /** Optional attribute '%s' of XML schema type '%s' */", ns_add(q, nsa), wsdl_type(q->info.typ, ""));
      }
      else if (is_soapref(q->info.typ))
        fprintf(fd, "\n        /** Context that manages this object */");
      else if (q->info.sto & (Sconst | Sprivate | Sprotected))
        fprintf(fd, "\n        /** Not serialized */");
      else if (q->info.sto & SmustUnderstand)
        fprintf(fd, "\n        /** MustUnderstand */");
      else if (!is_dynamic_array(typ) && is_repetition(q))
      {
        if (q->info.maxOccurs > 1)
          fprintf(fd, "\n        /** Sequence of " SOAP_LONG_FORMAT " to " SOAP_LONG_FORMAT " elements '%s' of XML schema type '%s'", q->info.minOccurs, q->info.maxOccurs, ns_add(q->next, nse), wsdl_type(q->next->info.typ, ""));
        else if (q->info.minOccurs >= 1)
          fprintf(fd, "\n        /** Sequence of at least " SOAP_LONG_FORMAT " elements '%s' of XML schema type '%s'", q->info.minOccurs, ns_add(q->next, nse), wsdl_type(q->next->info.typ, ""));
        else
          fprintf(fd, "\n        /** Sequence of elements '%s' of XML schema type '%s'", ns_add(q->next, nse), wsdl_type(q->next->info.typ, ""));
        fprintf(fd, " stored in dynamic array %s of length %s */", ident(q->next->sym->name), ident(q->sym->name));
        flag = 1;
      }
      else if (is_anytype(q))
      {
        fprintf(fd, "\n        /** Any type of element '%s' assigned to %s with its SOAP_TYPE_<typename> assigned to %s */\n        /** Do not create a cyclic data structure through this member unless SOAP encoding or SOAP_XML_GRAPH are used for id-ref serialization */", ns_add(q->next, nse), ident(q->next->sym->name), ident(q->sym->name));
        flag = 1;
      }
      else if (is_choice(q))
      {
        Entry *r;
        fprintf(fd, "\n        /** Union with %s variant selector %s set to one of:", c_type(q->next->info.typ), ident(q->sym->name));
        if ((Table*)q->next->info.typ->ref)
          for (r = ((Table*)q->next->info.typ->ref)->list; r; r = r->next)
            fprintf(fd, " %s", soap_union_member(q->next->info.typ, r));
        fprintf(fd, " */");
        flag = 1;
      }
      else if (is_anyType(q->info.typ))
        fprintf(fd, "\n        /** XML DOM element node graph */");
      else if (is_item(q))
        fprintf(fd, "\n        /** Simple content of XML schema type '%s' wrapped by this struct */", wsdl_type(q->info.typ, ""));
      else if (q->info.typ->type != Tfun && q->info.typ->type != Tunion && !(q->info.sto & (Sconst | Sprivate | Sprotected)) && !(q->info.sto & Sattribute) && !is_transient(q->info.typ) && !is_external(q->info.typ) && strncmp(q->sym->name, "__", 2))
      {
        if (q->info.maxOccurs > 1)
          fprintf(fd, "\n        /** Sequence of " SOAP_LONG_FORMAT " to " SOAP_LONG_FORMAT " elements '%s' of XML schema type '%s' */", q->info.minOccurs, q->info.maxOccurs, ns_add(q, nse), wsdl_type(q->info.typ, ""));
        else if (q->info.minOccurs >= 1)
        {
          if (q->info.nillable)
            fprintf(fd, "\n        /** Required nillable (xsi:nil when NULL) element '%s' of XML schema type '%s' */", ns_add(q, nse), wsdl_type(q->info.typ, ""));
          else
            fprintf(fd, "\n        /** Required element '%s' of XML schema type '%s' */", ns_add(q, nse), wsdl_type(q->info.typ, ""));
        }
        else
          fprintf(fd, "\n        /** Optional element '%s' of XML schema type '%s' */", ns_add(q, nse), wsdl_type(q->info.typ, ""));
      }
      else if (is_external(q->info.typ))
        fprintf(fd, "\n        /** Typedef %s with custom serializer for %s */", c_type_sym(q->info.typ), c_type(q->info.typ));
      else if (is_pointer_to_derived(q))
        fprintf(fd, "\n        /** Transient pointer to a derived type value that replaces the value of this base type %s when non-NULL */", c_type(typ));
      else if (is_transient(q->info.typ))
        fprintf(fd, "\n        /** Transient (not serialized) */");
      else if (is_imported(q->info.typ))
        fprintf(fd, "\n        /** Type imported from %s */", q->info.typ->imported);
      if (fd != freport)
      {
        if (!is_dynamic_array(typ) && !is_primclass(typ))
        {
          if (!strncmp(q->sym->name, "__size", 6))
          {
            if (q->info.typ->type != Tint && q->info.typ->type != Tsize)
            {
              sprintf(errbuf, "Member field '%s' is not an int or size_t type in struct '%s'", q->sym->name, typ->id->name);
              semwarn(errbuf);
            }
            else if (!q->next || (q->next->info.typ->type != Tpointer && !is_smart(q->next->info.typ)))
            {
              sprintf(errbuf, "Member field '%s' is not followed by a pointer member field in struct '%s'", q->sym->name, typ->id->name);
              semwarn(errbuf);
            }
          }
          else if (q->info.typ->type == Tint && !strncmp(q->sym->name, "__type", 6))
          {
            if (!q->next || ((q->next->info.typ->type != Tpointer || ((Tnode*)q->next->info.typ->ref)->type != Tvoid)))
            {
              sprintf(errbuf, "Member field '%s' is not followed by a void pointer member field in struct '%s'", q->sym->name, typ->id->name);
              semwarn(errbuf);
            }
          }
          else if (q->info.typ->type == Tint && !strncmp(q->sym->name, "__union", 7))
          {
            if (!q->next || ((q->next->info.typ->type != Tunion)))
            {
              sprintf(errbuf, "Member field '%s' is not followed by a union member field in struct '%s'", q->sym->name, typ->id->name);
              semwarn(errbuf);
            }
          }
          else if ((q->info.typ->type == Tint || q->info.typ->type == Tsize) && (q->info.sto & Sspecial))
          {
            if (!q->next || (q->next->info.typ->type != Tpointer && !is_smart(q->next->info.typ) && (q->next->info.typ->type != Tunion)))
            {
              sprintf(errbuf, "Member field '%s' is not followed by a pointer or union member field in struct '%s'", q->sym->name, typ->id->name);
              semwarn(errbuf);
            }
          }
        }
      }
      fprintf(fd, "\n        %s", c_storage(q->info.sto));
      if (q->sym == typ->id && q->info.typ->type == Tfun) /* to emit constructor in a struct: constructor has no return value */
        ((FNinfo*)q->info.typ->ref)->ret = mknone();
      fprintf(fd, "%s", c_type_id(q->info.typ, q->sym->name));
      if (q->info.sto & Sconstobj)
        fprintf(fd, " const");
      if (q->info.sto & Sfinal)
        fprintf(fd, " final");
      if (q->info.sto & Soverride)
        fprintf(fd, " override");
      if (q->info.sto & Sconst)
        fprintf(fd, "%s;", c_init(q));
      else
      {
        fprintf(fd, ";");
        if (q->info.hasval)
        {
          if (q->info.fixed)
            fprintf(fd, "\t/**< initialized with fixed value%s */", c_init(q));
          else
            fprintf(fd, "\t/**< initialized with default value%s */", c_init(q));
        }
        else if (q->info.ptrval)
        {
          if (q->info.fixed)
            fprintf(fd, "\t/**< optional with fixed value%s */", c_init(q));
          else
            fprintf(fd, "\t/**< optional with default value%s */", c_init(q));
        }
      }
    }
    if (!cflag && !is_transient(typ) && !is_volatile(typ))
    {
      fprintf(fd, "\n      public:\n        /** Return unique type id %s */\n        long soap_type() const { return %s; }", soap_type(typ), soap_type(typ));
      if ((s = union_member(typ)))
      {
        if (fd != freport)
        {
          sprintf(errbuf, "struct '%s' cannot be assigned a default constructor because it is directly or indirectly used as a member of union '%s'", typ->id->name, ident(s));
          semwarn(errbuf);
        }
      }
      else
      {
        if (!has_constructor(typ))
        {
          Table *t;
          Entry *p;
          fprintf(fd, "\n        /** Constructor with member initializations */");
          gen_constructor(fd, typ);
          for (t = (Table*)typ->ref; t; t = t->prev)
          {
            for (p = t->list; p; p = p->next)
            {
              if (p->info.typ->type == Treference || p->info.typ->type == Trvalueref)
              {
                sprintf(errbuf, "no constructor for '%s' to explicitly initialize the reference member '%s'", typ->id->name, p->sym->name);
                semwarn(errbuf);
              }
            }
          }
        }
        /* not necessary to add a default destructor, keep the C -> C++ transparency of structs
        if (!has_destructor(typ))
          fprintf(fd, "\n\t        ~%s() { }", ident(typ->id->name));
        */
      }
      if (Ecflag)
        fprintf(fd, "\n        /** Friend duplicator */\n        friend SOAP_FMAC1 %s * SOAP_FMAC2 %s_dup_%s(struct soap*, %s*, %s);", c_type(typ), fprefix, c_ident(typ), c_type(typ), c_type_constptr_id(typ, "const*"));
      if (Edflag)
        fprintf(fd, "\n        /** Friend deleter */\n        friend SOAP_FMAC1 void SOAP_FMAC2 %s_del_%s(%s);", fprefix, c_ident(typ), c_type_constptr_id(typ, "const*"));
      fprintf(fd, "\n        /** Friend allocator */\n        friend SOAP_FMAC1 %s * SOAP_FMAC2 %s_instantiate_%s(struct soap*, int, const char*, const char*, size_t*);", c_ident(typ), fprefix, c_ident(typ));
    }
    if (!((Table*)typ->ref)->list)
    {
      if (cflag && fd != freport)
        fprintf(fd, "\n#ifdef WITH_NOEMPTYSTRUCT\n\tchar dummy;\t/* empty struct is a GNU extension */\n#endif");
    }
    if (fd == freport)
    {
      fprintf(freport, "\n    };\n\n");
      if (!is_transient(p->info.typ) && !is_header_or_fault(p->info.typ))
      {
        const char *ns = prefix_of(p->sym->name);
        int uf = uflag;
        uflag = 1;
        gen_report_members(p, nsa, nse);
        fprintf(freport, "The following operations on `%s` are available:\n\n", c_type(p->info.typ));
        if (cflag)
        {
          fprintf(freport, "- `(%s *)soap_malloc(struct soap*, sizeof(%s))` raw managed allocation\n", c_type(p->info.typ), c_type(p->info.typ));
          fprintf(freport, "- `%s *soap_new_%s(struct soap*, int n)` managed allocation with default initialization of one `%s` when `n` = 1 or array `%s[n]` when `n` > 1\n", c_type(p->info.typ), c_ident(p->info.typ), c_type(p->info.typ), c_type(p->info.typ));
        }
        else
        {
          fprintf(freport, "- `%s *soap_new_%s(struct soap*)` managed allocation with default initialization\n", c_type(p->info.typ), c_ident(p->info.typ));
          fprintf(freport, "- `%s *soap_new_%s(struct soap*, int n)` managed allocation with default initialization of array `%s[n]`\n", c_type(p->info.typ), c_ident(p->info.typ), c_type(p->info.typ));
          fprintf(freport, "- `%s *soap_new_req_%s(struct soap*", c_type(p->info.typ), c_ident(p->info.typ));
          gen_report_req_params(p->info.typ);
          fprintf(freport, ")` managed allocation with required members assigned the values of these parameters, with all other members default initialized\n");
          fprintf(freport, "- `%s *soap_new_set_%s(struct soap*", c_type(p->info.typ), c_ident(p->info.typ));
          gen_report_set_params(p->info.typ);
          fprintf(freport, ")` managed allocation with the public members assigned the values of these parameters\n");
        }
        fprintf(freport, "- `void soap_default_%s(struct soap*, %s*)` (re)set to default initialization values\n", c_ident(p->info.typ), c_type(p->info.typ));
        fprintf(freport, "- `int soap_write_%s(struct soap*, const %s*)` serialize to XML, returns `SOAP_OK` or error code\n", c_ident(p->info.typ), c_type(p->info.typ));
        fprintf(freport, "- `int soap_read_%s(struct soap*, %s*)` deserialize from XML, returns `SOAP_OK` or error code\n", c_ident(p->info.typ), c_type(p->info.typ));
        fprintf(freport, "- `int soap_PUT_%s(struct soap*, const char *URL, const %s*)` REST PUT XML, returns `SOAP_OK` or error code\n", c_ident(p->info.typ), c_type(p->info.typ));
        fprintf(freport, "- `int soap_PATCH_%s(struct soap*, const char *URL, const %s*)` REST PATCH XML, returns `SOAP_OK` or error code\n", c_ident(p->info.typ), c_type(p->info.typ));
        fprintf(freport, "- `int soap_POST_send_%s(struct soap*, const char *URL, const %s*)` REST POST send XML (should be followed by a `soap_POST_recv_Type`), returns `SOAP_OK` or error code\n", c_ident(p->info.typ), c_type(p->info.typ));
        fprintf(freport, "- `int soap_POST_recv_%s(struct soap*, %s*)` REST POST receive XML (after a `soap_POST_send_Type`), returns `SOAP_OK` or error code\n", c_ident(p->info.typ), c_type(p->info.typ));
        fprintf(freport, "- `int soap_GET_%s(struct soap*, const char *URL, %s*)` REST GET XML, returns `SOAP_OK` or error code\n", c_ident(p->info.typ), c_type(p->info.typ));
        if (!wflag)
        {
          if (!is_invisible(p->sym->name) && !has_ns_eq("xsd", p->sym->name) && !is_header_or_fault(typ))
          {
            s = ns_of(p->sym->name);
            if (s && *s && has_ns_eq(NULL, p->sym->name))
              fprintf(freport, "\nThe component of XML schema type *`%s`* in schema \"[%s](#doc-namespaces)\" is:\n\n", x, s);
            else
              fprintf(freport, "\nThe component of XML schema type *`%s`* is:\n\n", x);
            gen_schema_type(freport, NULL, p, ns, ns, 1, NULL, NULL);
          }
        }
        fprintf(freport, "\n");
        uflag = uf;
      }
    }
    else
      fprintf(fd, "\n};");
  }
  else if (typ->type == Tclass)
  {
    int permission = -1;
    int flag = 0;
    DBGLOG(fprintf(stderr, "\nclass %s\n", typ->id->name));
    if (fd == freport)
      fprintf(fd, "\n    class SOAP_CMAC %s", ident(typ->id->name));
    else
      fprintf(fd, "class SOAP_CMAC %s", ident(typ->id->name));
    if (typ->baseid)
      fprintf(fd, " : public %s", ident(typ->baseid->name));
    fprintf(fd, " {");
    for (q = ((Table*)typ->ref)->list; q; q = q->next)
    {
      if (permission != (q->info.sto & (Sprivate | Sprotected)))
      {
        if (q->info.sto & Sprivate)
          fprintf(fd, "\n      private:");
        else if (q->info.sto & Sprotected)
          fprintf(fd, "\n      protected:");
        else
          fprintf(fd, "\n      public:");
        permission = (q->info.sto & (Sprivate | Sprotected));
      }
      if (is_anyAttribute(q->info.typ))
        fprintf(fd, "\n        /// XML DOM attribute list");
      else if (q->info.sto & Sattribute)
      {
        if (q->info.minOccurs >= 1)
          fprintf(fd, "\n        /// Required attribute '%s' of XML schema type '%s'", ns_add(q, nsa), wsdl_type(q->info.typ, ""));
        else
          fprintf(fd, "\n        /// Optional attribute '%s' of XML schema type '%s'", ns_add(q, nsa), wsdl_type(q->info.typ, ""));
      }
      if (flag)
        flag = 0;
      else if (q->info.typ->type == Tfun || (q->info.sto & Stypedef))
        ;
      else if (is_soapref(q->info.typ))
        fprintf(fd, "\n        /// Context that manages this object");
      else if (q->info.sto & (Sconst | Sprivate | Sprotected))
        fprintf(fd, "\n        /// Not serialized");
      else if (q->info.sto & SmustUnderstand)
        fprintf(fd, "\n        /// MustUnderstand");
      else if (!is_dynamic_array(typ) && is_repetition(q))
      {
        if (q->info.maxOccurs > 1)
          fprintf(fd, "\n        /// Sequence of " SOAP_LONG_FORMAT " to " SOAP_LONG_FORMAT " elements '%s' of XML schema type '%s'", q->info.minOccurs, q->info.maxOccurs, ns_add(q->next, nse), wsdl_type(q->next->info.typ, ""));
        else if (q->info.minOccurs >= 1)
          fprintf(fd, "\n        /// Sequence of at least " SOAP_LONG_FORMAT " elements '%s' of XML schema type '%s'", q->info.minOccurs, ns_add(q->next, nse), wsdl_type(q->next->info.typ, ""));
        else
          fprintf(fd, "\n        /// Sequence of elements '%s' of XML schema type '%s'", ns_add(q->next, nse), wsdl_type(q->next->info.typ, ""));
        fprintf(fd, " stored in dynamic array %s of length %s", ident(q->next->sym->name), ident(q->sym->name));
        flag = 1;
      }
      else if (is_anytype(q))
      {
        fprintf(fd, "\n        /// Any type of element '%s' assigned to %s with its SOAP_TYPE_<typename> assigned to %s\n        /// Do not create a cyclic data structure through this member unless SOAP encoding or SOAP_XML_GRAPH are used for id-ref serialization", ns_add(q->next, nse), ident(q->next->sym->name), ident(q->sym->name));
        flag = 1;
      }
      else if (is_choice(q))
      {
        Entry *r;
        fprintf(fd, "\n        /// Union with %s variant selector %s set to one of:", c_type(q->next->info.typ), ident(q->sym->name));
        for (r = ((Table*)q->next->info.typ->ref)->list; r; r = r->next)
          fprintf(fd, " %s", soap_union_member(q->next->info.typ, r));
        flag = 1;
      }
      else if (is_anyType(q->info.typ))
        fprintf(fd, "\n        /// XML DOM element node graph");
      else if (is_item(q))
        fprintf(fd, "\n        /// Simple content of XML schema type '%s' wrapped by this struct", wsdl_type(q->info.typ, ""));
      else if (q->info.typ->type != Tfun && q->info.typ->type != Tunion && !(q->info.sto & (Sconst | Sprivate | Sprotected)) && !(q->info.sto & Sattribute) && !is_transient(q->info.typ) && !is_external(q->info.typ) && strncmp(q->sym->name, "__", 2))
      {
        if (q->info.maxOccurs > 1)
          fprintf(fd, "\n        /// Sequence of " SOAP_LONG_FORMAT " to " SOAP_LONG_FORMAT " elements '%s' of XML schema type '%s'", q->info.minOccurs, q->info.maxOccurs, ns_add(q, nse), wsdl_type(q->info.typ, ""));
        else if (q->info.minOccurs >= 1)
        {
          if (q->info.nillable)
            fprintf(fd, "\n        /// Required nillable (xsi:nil when NULL) element '%s' of XML schema type '%s'", ns_add(q, nse), wsdl_type(q->info.typ, ""));
          else
            fprintf(fd, "\n        /// Required element '%s' of XML schema type '%s'", ns_add(q, nse), wsdl_type(q->info.typ, ""));
        }
        else
          fprintf(fd, "\n        /// Optional element '%s' of XML schema type '%s'", ns_add(q, nse), wsdl_type(q->info.typ, ""));
      }
      else if (is_external(q->info.typ))
        fprintf(fd, "\n        /// Typedef %s with custom serializer for %s", c_type_sym(q->info.typ), c_type(q->info.typ));
      else if (is_pointer_to_derived(q))
        fprintf(fd, "\n        /// Transient pointer to a derived type value that replaces the value of this base type %s when non-NULL", c_type(typ));
      else if (is_transient(q->info.typ))
        fprintf(fd, "\n        /// Transient (not serialized)");
      else if (is_imported(q->info.typ))
        fprintf(fd, "\n        /// Type imported from %s", q->info.typ->imported);
      if (fd != freport)
      {
        if (!is_dynamic_array(typ) && !is_primclass(typ))
        {
          if (!strncmp(q->sym->name, "__size", 6))
          {
            if (q->info.typ->type != Tint && q->info.typ->type != Tsize)
            {
              sprintf(errbuf, "Member field '%s' is not an int or size_t type in class '%s'", q->sym->name, typ->id->name);
              semwarn(errbuf);
            }
            else if (!q->next || (q->next->info.typ->type != Tpointer && !is_smart(q->next->info.typ)))
            {
              sprintf(errbuf, "Member field '%s' is not followed by a pointer member field in class '%s'", q->sym->name, typ->id->name);
              semwarn(errbuf);
            }
          }
          else if (q->info.typ->type == Tint && !strncmp(q->sym->name, "__type", 6))
          {
            if (!q->next || ((q->next->info.typ->type != Tpointer || ((Tnode*)q->next->info.typ->ref)->type != Tvoid)))
            {
              sprintf(errbuf, "Member field '%s' is not followed by a void pointer member field in class '%s'", q->sym->name, typ->id->name);
              semwarn(errbuf);
            }
          }
          else if (q->info.typ->type == Tint && !strncmp(q->sym->name, "__union", 7))
          {
            if (!q->next || ((q->next->info.typ->type != Tunion)))
            {
              sprintf(errbuf, "Member field '%s' is not followed by a union member field in class '%s'", q->sym->name, typ->id->name);
              semwarn(errbuf);
            }
          }
          else if ((q->info.typ->type == Tint || q->info.typ->type == Tsize) && (q->info.sto & Sspecial))
          {
            if (!q->next || (q->next->info.typ->type != Tpointer && !is_smart(q->next->info.typ) && (q->next->info.typ->type != Tunion)))
            {
              sprintf(errbuf, "Member field '%s' is not followed by a pointer or union member field in class '%s'", q->sym->name, typ->id->name);
              semwarn(errbuf);
            }
          }
        }
      }
      fprintf(fd, "\n        %s", c_storage(q->info.sto));
      fprintf(fd, "%s", c_type_id(q->info.typ, q->sym->name));
      if (q->info.sto & Sconstobj)
        fprintf(fd, " const");
      if (q->info.sto & Sfinal)
        fprintf(fd, " final");
      if (q->info.sto & Soverride)
        fprintf(fd, " override");
      if (q->info.sto & Sabstract)
        fprintf(fd, " = 0;");
      else if (q->info.sto & Sconst)
        fprintf(fd, "%s;", c_init(q));
      else
      {
        fprintf(fd, ";");
        if (q->info.hasval)
        {
          if (q->info.fixed)
            fprintf(fd, "\t///< initialized with fixed value%s", c_init(q));
          else
            fprintf(fd, "\t///< initialized with default value%s", c_init(q));
        }
        else if (q->info.ptrval)
        {
          if (q->info.fixed)
            fprintf(fd, "\t///< optional with fixed value%s", c_init(q));
          else
            fprintf(fd, "\t///< optional with default value%s", c_init(q));
        }
      }
    }
    if (!is_transient(typ) && !is_volatile(typ))
    {
      fprintf(fd, "\n      public:");
      fprintf(fd, "\n        /// Return unique type id %s", soap_type(typ));
      fprintf(fd, "\n        virtual long soap_type(void) const { return %s; }", soap_type(typ));
      fprintf(fd, "\n        /// (Re)set members to default values");
      fprintf(fd, "\n        virtual void soap_default(struct soap*);");
      fprintf(fd, "\n        /// Serialize object to prepare for SOAP 1.1/1.2 encoded output (or with SOAP_XML_GRAPH) by analyzing its (cyclic) structures");
      fprintf(fd, "\n        virtual void soap_serialize(struct soap*) const;");
      if (Etflag)
        fprintf(fd, "\n        virtual void soap_traverse(struct soap*, const char *s, soap_walker, soap_walker);");
      fprintf(fd, "\n        /// Output object in XML, compliant with SOAP 1.1 encoding style, return error code or SOAP_OK");
      fprintf(fd, "\n        virtual int soap_put(struct soap*, const char *tag, const char *type) const;");
      fprintf(fd, "\n        /// Output object in XML, with tag and optional id attribute and xsi:type, return error code or SOAP_OK");
      fprintf(fd, "\n        virtual int soap_out(struct soap*, const char *tag, int id, const char *type) const;");
      fprintf(fd, "\n        /// Get object from XML, compliant with SOAP 1.1 encoding style, return pointer to object or NULL on error");
      fprintf(fd, "\n        virtual void *soap_get(struct soap*, const char *tag, const char *type);");
      fprintf(fd, "\n        /// Get object from XML, with matching tag and type (NULL matches any tag and type), return pointer to object or NULL on error");
      fprintf(fd, "\n        virtual void *soap_in(struct soap*, const char *tag, const char *type);");
      fprintf(fd, "\n        /// Return a new object of type %s, default initialized and not managed by a soap context", c_type(typ));
      fprintf(fd, "\n        virtual %s *soap_alloc(void) const { return SOAP_NEW_UNMANAGED(%s); }", c_type(typ), c_type(typ));
      if (Ecflag)
      {
        fprintf(fd, "\n        /// Return a duplicate of this object by deep copying, replicating all deep cycles and shared pointers when a managing soap context is provided as argument.\n        /// Deep copy is a tree when argument is NULL, but the presence of deep cycles will lead to non-termination.\n        /// Use flag SOAP_XML_TREE with a managing context to copy into a tree without cycles and pointers to shared objects");
        fprintf(fd, "\n        virtual %s *soap_dup(struct soap *soap = NULL, void *dest = NULL) const { return %s_dup_%s(soap, (%s*)dest, this); }", c_type(typ), fprefix, c_ident(typ), c_type(typ));
      }
      if (Edflag)
      {
        fprintf(fd, "\n        /// Delete heap-allocated members of this object by deep deletion ONLY IF this object and all of its (deep) members are not managed by a soap context AND the deep structure is a tree (no cycles and co-referenced objects by way of multiple (non-smart) pointers pointing to the same data)\n        /// Can be safely used after soap_dup(NULL) to delete the deep copy");
        fprintf(fd, "\n        virtual void soap_del(void) const { %s_del_%s(this); }", fprefix, c_ident(typ));
      }
      if ((s = union_member(typ)))
      {
        if (fd != freport)
        {
          sprintf(errbuf, "class '%s' cannot be assigned a default constructor because it is directly or indirectly used as a member of union '%s'", typ->id->name, ident(s));
          semwarn(errbuf);
        }
      }
      else
      {
        if (!has_constructor(typ))
        {
          Table *t;
          Entry *p;
          fprintf(fd, "\n      public:\n        /// Constructor with default initializations");
          gen_constructor(fd, typ);
          for (t = (Table*)typ->ref; t; t = t->prev)
          {
            for (p = t->list; p; p = p->next)
            {
              if (p->info.typ->type == Treference || p->info.typ->type == Trvalueref)
              {
                sprintf(errbuf, "no constructor for '%s' to explicitly initialize the reference member '%s'", typ->id->name, p->sym->name);
                semwarn(errbuf);
              }
            }
          }
        }
        if (!has_destructor(typ))
          fprintf(fd, "\n        virtual ~%s() { }", c_ident(typ));
      }
      if (Ecflag)
        fprintf(fd, "\n        /// Friend duplicator\n        friend SOAP_FMAC1 %s * SOAP_FMAC2 %s_dup_%s(struct soap*, %s*, %s);", c_type(typ), fprefix, c_ident(typ), c_type(typ), c_type_constptr_id(typ, "const*"));
      if (Edflag)
        fprintf(fd, "\n        /// Friend deleter\n        friend SOAP_FMAC1 void SOAP_FMAC2 %s_del_%s(%s);", fprefix, c_ident(typ), c_type_constptr_id(typ, "const*"));
      fprintf(fd, "\n        /// Friend allocator used by soap_new_%s(struct soap*, int)\n        friend SOAP_FMAC1 %s * SOAP_FMAC2 %s_instantiate_%s(struct soap*, int, const char*, const char*, size_t*);", c_ident(typ), c_ident(typ), fprefix, c_ident(typ));
      /* the use of 'friend' causes problems linking static functions. Adding these friends could enable serializing protected/private members (which is not implemented)
         fprintf(fd, "\n        friend %s *soap_in_%s(struct soap*, const char*, %s*, const char*);", typ->id->name, typ->id->name, typ->id->name);
         fprintf(fd, "\n        friend int soap_out_%s(struct soap*, const char*, int, const %s*, const char*);", typ->id->name, typ->id->name);
       */
    }
    if (fd == freport)
    {
      fprintf(freport, "\n    };\n\n");
      if (!is_transient(p->info.typ) && !is_header_or_fault(p->info.typ))
      {
        const char *ns = prefix_of(p->sym->name);
        int uf = uflag;
        uflag = 1;
        gen_report_members(p, nsa, nse);
        fprintf(freport, "The following operations on `%s` are available:\n\n", c_type(p->info.typ));
        fprintf(freport, "- `%s *soap_new_%s(struct soap*)` managed allocation with default initialization\n", c_type(p->info.typ), c_ident(p->info.typ));
        fprintf(freport, "- `%s *soap_new_%s(struct soap*, int n)` managed allocation of array `%s[n]`\n", c_type(p->info.typ), c_ident(p->info.typ), c_type(p->info.typ));
        fprintf(freport, "- `%s *soap_new_req_%s(struct soap*", c_type(p->info.typ), c_ident(p->info.typ));
        gen_report_req_params(p->info.typ);
        fprintf(freport, ")` managed allocation with required members assigned the values of these parameters, with all other members default initialized\n");
        fprintf(freport, "- `%s *soap_new_set_%s(struct soap*", c_type(p->info.typ), c_ident(p->info.typ));
        gen_report_set_params(p->info.typ);
        fprintf(freport, ")` managed allocation with public members assigned the values of these parameters\n");
        if (is_external(p->info.typ) || is_volatile(p->info.typ))
          fprintf(freport, "- `void soap_default_%s(struct soap*, %s*)` (re)set members to default initialization values\n", c_ident(p->info.typ), c_type(p->info.typ));
        else
          fprintf(freport, "- `void %s::soap_default(struct soap*)` (re)set members to default values\n", c_type(p->info.typ));
        fprintf(freport, "- `int soap_write_%s(struct soap*, const %s*)` serialize to XML, returns `SOAP_OK` or error code\n", c_ident(p->info.typ), c_type(p->info.typ));
        fprintf(freport, "- `int soap_PUT_%s(struct soap*, const char *URL, const %s*)` REST PUT XML, returns `SOAP_OK` or error code\n", c_ident(p->info.typ), c_type(p->info.typ));
        fprintf(freport, "- `int soap_PATCH_%s(struct soap*, const char *URL, const %s*)` REST PATCH XML, returns `SOAP_OK` or error code\n", c_ident(p->info.typ), c_type(p->info.typ));
        fprintf(freport, "- `int soap_POST_send_%s(struct soap*, const char *URL, const %s*)` REST POST send XML (MUST be followed by a `soap_POST_recv_Type`), returns `SOAP_OK` or error code\n", c_ident(p->info.typ), c_type(p->info.typ));
        fprintf(freport, "- `int soap_read_%s(struct soap*, %s*)` deserialize from XML, returns `SOAP_OK` or error code\n", c_ident(p->info.typ), c_type(p->info.typ));
        fprintf(freport, "- `int soap_GET_%s(struct soap*, const char *URL, %s*)` REST GET XML, returns `SOAP_OK` or error code\n", c_ident(p->info.typ), c_type(p->info.typ));
        fprintf(freport, "- `int soap_POST_recv_%s(struct soap*, %s*)` REST POST receive XML (after a `soap_POST_send_Type`), returns `SOAP_OK` or error code\n", c_ident(p->info.typ), c_type(p->info.typ));
        if (!wflag)
        {
          if (!is_invisible(p->sym->name) && !has_ns_eq("xsd", p->sym->name) && !is_header_or_fault(typ))
          {
            s = ns_of(p->sym->name);
            if (s && *s && has_ns_eq(NULL, p->sym->name))
              fprintf(freport, "\nThe component of XML schema type *`%s`* in schema \"[%s](#doc-namespaces)\" is:\n\n", x, s);
            else
              fprintf(freport, "\nThe component of XML schema type *`%s`* is:\n\n", x);
            gen_schema_type(freport, NULL, p, ns, ns, 1, NULL, NULL);
          }
        }
        fprintf(freport, "\n");
        uflag = uf;
      }
    }
    else
      fprintf(fd, "\n};");
  }
  else if (typ->type == Tunion)
  {
    int i = 1;
    if (fd == freport)
      fprintf(fd, "\n    union %s\n    {", ident(typ->id->name));
    else
      fprintf(fd, "union %s\n{", ident(typ->id->name));
    for (q = ((Table*)typ->ref)->list; q; q = q->next)
    {
      if (q->info.typ->type == Tfun)
      {
        if (q->sym == typ->id) /* to emit constructor in a union: constructor has no return value */
          ((FNinfo*)q->info.typ->ref)->ret = mknone();
      }
      else
      {
        fprintf(fd, "\n        #define %s\t(%d)\t/**< union variant selector value for member %s */", soap_union_member(typ, q), i, ident(q->sym->name));
        i++;
        if (q->info.sto & (Sconst | Sprivate | Sprotected))
          fprintf(fd, "\n        /** Not serialized */");
        else if (is_transient(q->info.typ))
          fprintf(fd, "\n        /** Transient (not serialized) */");
        else if (is_external(q->info.typ))
          fprintf(fd, "\n        /** Typedef %s with custom serializer for %s */", c_type_sym(q->info.typ), c_type(q->info.typ));
        if (q->info.sto & Sattribute)
        {
          sprintf(errbuf, "union '%s' contains attribute declarations", typ->id->name);
          semwarn(errbuf);
        }
      }
      fprintf(fd, "\n        %s", c_storage(q->info.sto));
      fprintf(fd, "%s;", c_type_id(q->info.typ, q->sym->name));
    }
    if (!((Table*)typ->ref)->list)
    {
      if (cflag && fd != freport)
        fprintf(fd, "\n#ifdef WITH_NOEMPTYSTRUCT\n\tchar dummy;\t/* empty union is a GNU extension */\n#endif");
    }
    if (fd == freport)
      fprintf(fd, "\n    };\n\n");
    else
      fprintf(fd, "\n};");
  }
  if (fd == freport)
  {
    gen_report_hr();
  }
  else
  {
    if (is_volatile(typ))
      fprintf(fd, "\n#endif");
    fprintf(fd, "\n#endif");
    if (is_header_or_fault(typ) || is_body(typ))
      fprintf(fd, "\n#endif");
  }
  fflush(fd);
}

void
gen_constructor(FILE *fd, Tnode *typ)
{
  Table *t = typ->ref;
  Entry *p;
  const char *sep = " :";
  if (!t)
    return;
  fprintf(fd, "\n        %s()", c_ident(typ));
  for (p = t->list; p; p = p->next)
  {
    if (p->info.typ->type == Tfun || p->info.typ->type == Tunion)
      continue;
    if ((p->info.sto & Sstatic))
      continue;
    if (is_fixedstring(p->info.typ))
      continue;
    if (p->info.hasval)
      fprintf(fd, "%s %s(%s)", sep, ident(p->sym->name), c_init_a(p, ""));
    else
      fprintf(fd, "%s %s()", sep, ident(p->sym->name));
    sep = ",";
  }
  fprintf(fd, " {");
  for (p = t->list; p; p = p->next)
  {
    if (is_fixedstring(p->info.typ))
    {
      if (p->info.hasval)
        fprintf(fd, " soap_strcpy(%s, %d, \"%s\");", ident(p->sym->name), get_dimension(p->info.typ), cstring(p->info.val.s, 0));
      else
        fprintf(fd, " %s[0] = '\\0';", ident(p->sym->name));
    }
  }
  fprintf(fd, " }");
  fflush(fd);
}

void
generate_header(Table *t)
{
  Service *sp;
  Entry *p, *q;
  int i;
  if (rflag)
  {
    fprintf(freport, "<div style='display: none'>\n@tableofcontents @section README\n\nTo view this file in the Firefox web browser, download readmeviewer.html from https://www.genivia.com/files/readmeviewer.html.zip, unzip and copy it to the same directory where this soapReadme.md file is located, then open it in Firefox to view the contents of soapReadme.md.\n\nThis markdown file is compatible with Doxygen.\n</div>\n\n");
    fprintf(freport, "## Overview {#doc-overview}\n\nThis report was generated by soapcpp2 v" VERSION " for interface header file [%s](%s) with options -r %s%s%s%s%s%s%s%s%s\n\n", filename, filename, copt ? copt : "", soap_version < 0 ? " -0 " : " ", Cflag ? "-C " : "", Sflag ? "-S " : "", Lflag ? "-L " : "", iflag ? "-i " : jflag ? "-j " : "", wflag ? "-w " : "", namespaceid ? "-q" : strcmp(prefix, "soap") ? "-p" : "", strcmp(prefix, "soap") ? prefix : "");
    for (sp = services; sp; sp = sp->next)
    {
      if (sp->documentation)
      {
        gen_text(freport, sp->documentation);
        fprintf(freport, "\n\n");
      }
    }
    fprintf(freport, "### Tools\n\nThe **wsdl2h** command line tool takes a set of WSDL and XSD files to generate a data binding interface header file.  This interface header file is similar to a C/C++ header file and contains declarations of C/C++ types and functions with explanatory comments, directives, and annotations.  Types are declared in this interface header file as serializable.  Functions are declared as Web service operations for the client and server sides.\n\nThe **soapcpp2** command line tool takes an interface header file (i.e. a header file) such as [%s](%s) to generate the data binding implementation.  This implementation includes XML serializers and source code for the client and server side.  An interface header file for soapcpp2 can be a regular C/C++ header file with type and function declarations (without code), and include annotations to declare XML schema-related properties.\n\n", filename, filename);
    fprintf(freport, "### Files\n\nThe following %s source code files were generated by soapcpp2 for interface header file [%s](%s):\n\n", cflag ? "C" : copt ? copt + 1 : "c++", filename, filename);
    fprintf(freport, "- [%s](%s) contains an annotated copy of [%s](%s) and of the imported files (if any), where most of the information in this report [%s](%s) is sourced from (this header file also #includes \"stdsoap2.h\")\n", soapStub, pathsoapStub, filename, filename, soapReadme, pathsoapReadme);
    fprintf(freport, "- [%s](%s) declares allocation and (de)serialization functions for each C/C++ type, to #include in projects (this header file also #includes \"%s\")\n", soapH, pathsoapH, soapStub);
    fprintf(freport, "- [%s](%s) defines allocation and (de)serialization functions for each C/C++ type, to compile with a project\n", soapC, pathsoapC);
    if (!Sflag)
    {
      if (!iflag && !jflag)
      {
        fprintf(freport, "- [%s](%s) defines client call stub functions for service invocation, to compile with a project\n", soapClient, pathsoapClient);
        if (!Lflag)
          fprintf(freport, "- [%s](%s) combines %s with %s into one \"library\" file, intended for combining multiple clients and servers that are separately generated with soapcpp2 option -p, thereby preventing serializer naming conflicts (this makes serialization functions locally visible to the client/server as static functions)\n", soapClientLib, pathsoapClientLib, soapC, soapClient);
      }
      else
      {
        fprintf(freport, "- See [Web client proxy class](#doc-client) for the client-side source code files to use\n");
      }
    }
    if (!Cflag)
    {
      if (!iflag && !jflag)
      {
        fprintf(freport, "- [%s](%s) defines server functions, including the service dispatcher that calls back-end service operations, to compile with a project\n", soapServer, pathsoapServer);
        if (!Lflag)
          fprintf(freport, "- [%s](%s) combines %s with %s into one \"library\" file, intended for combining multiple clients and servers that are separately generated with soapcpp2 option -p, thereby preventing serializer function naming conflicts (this makes serialization functions locally visible to the client/server as static functions)\n", soapServerLib, pathsoapServerLib, soapC, soapServer);
      }
      else
      {
        fprintf(freport, "- See [Web service class](#doc-server) for the server-side source code files to use\n");
      }
    }
    fprintf(freport, "\nAlso compile stdsoap2.%s (and dom.%s %s XML DOM is used) with a project (or link libgsoapssl%s.a) and use the following compile-time options:\n\n- `-DWITH_OPENSSL` to enable HTTPS with OpenSSL\n- `-DWITH_GNUTLS` to enable HTTPS with GNUTLS\n- `-DWITH_DOM` is required when using the WS-Security plugin\n- `-DWITH_GZIP` to enable message compression\n\nSee the gSOAP documentation for additional options.\n\n### Contents\n\nThis report has the following contents describing the data binding interface types, and the client- and the server-side operations (if any):\n\n", cflag ? "c" : "cpp", cflag ? "c" : "cpp", is_anyType_flag ? "because" : "if", cflag ? "" : "++");
    if (enumtable && enumtable->list)
      fprintf(freport, "- [enumeration types](#doc-enums)\n");
    if (classtable && classtable->list)
      fprintf(freport, "- [%sstruct and union types](#doc-classes)\n", cflag ? "" : "class, ");
    if (typetable && typetable->list)
      fprintf(freport, "- [typedefs](#doc-typedefs)\n");
    fprintf(freport, "- [summary of serializable types](#doc-types)\n");
    fprintf(freport, "- [schemas and namespaces](#doc-namespaces)\n");
    if (!Sflag)
    {
      if (!iflag && !jflag)
        fprintf(freport, "- [Web client operations](#doc-client) lists the service operations to call\n");
      else
        fprintf(freport, "- [Web client proxy class](#doc-client) lists the methods to invoke\n");
    }
    if (!Cflag)
    {
      if (!iflag && !jflag)
        fprintf(freport, "- [Web server operations](#doc-server) lists the service operations to implement when developing a service\n");
      else
        fprintf(freport, "- [Web service class](#doc-server) lists the methods to implement when developing a service\n");
    }
    fprintf(freport, "\n");
    gen_report_hr();
  }
  if (enumtable && enumtable->list)
  {
    Table *r = mktable(NULL);
    banner(fheader, "Enumeration Types");
    fflush(fheader);
    if (rflag)
      fprintf(freport, "## Enumeration Types {#doc-enums}\n\n<table class=\"doxtable\">\n<tr><th> Type </th><th> Declared </th><th> Serializable </th><th> Bitmask </th><th> Values </th></tr>\n");
    for (p = enumtable->list; p; p = p->next)
    {
      if (rflag && p->info.typ->ref)
        fprintf(freport, "<tr><td><code><a href=\"#%s\"> %s </a></code></td><td> %s:%d </td><td> %s </td><td> %s </td><td>", c_ident(p->info.typ), c_type(p->info.typ), p->filename, p->lineno, !is_transient(p->info.typ) ? "yes" : "", is_mask(p->info.typ) ? "yes" : "");
      if (!is_imported(p->info.typ) && (!is_transient(p->info.typ) || p->info.typ->ref))
      {
        int enum64 = False;
        const char *x;
        x = xsi_type(p->info.typ);
        if (!x || !*x)
          x = wsdl_type(p->info.typ, "");
        fprintf(fheader, "\n\n/* %s:%d */", p->filename, p->lineno);
        fprintf(fheader, "\n#ifndef %s", soap_type(p->info.typ));
        if (namespaceid)
          fprintf(fheader, "\n#define %s (-%d)\n", soap_type(p->info.typ), p->info.typ->num); /* namespaced SOAP_TYPE is negative to avoid clashes */
        else
          fprintf(fheader, "\n#define %s (%d)", soap_type(p->info.typ), p->info.typ->num);
        if (is_volatile(p->info.typ))
          fprintf(fheader, "\n#if 0 /* Volatile: not declared here */");
        if (is_mask(p->info.typ))
          fprintf(fheader, "\n/* Bitmask %s */", x);
        else
          fprintf(fheader, "\n/* %s */", x);
        if ((Table*)p->info.typ->ref)
        {
          for (q = ((Table*)p->info.typ->ref)->list; q; q = q->next)
          {
            if (q->info.val.i > 0x7FFFLL || q->info.val.i < -0x8000LL)
            {
              enum64 = True;
              break;
            }
          }
        }
        if (cflag && p->info.typ->type == Tenum && p->sym->token == TYPE)
          fprintf(fheader, "\ntypedef ");
        else
          fprintf(fheader, "\n");
        if (p->info.typ->width == 4 || is_mask(p->info.typ))
        {
          if (p->info.typ->type == Tenumsc)
          {
            if (enum64)
              fprintf(fheader, "enum class %s : int64_t {", ident(p->info.typ->id->name));
            else
              fprintf(fheader, "enum class %s {", ident(p->info.typ->id->name));
          }
          else
            fprintf(fheader, "enum %s {", ident(p->info.typ->id->name));
        }
        else
        {
          const char *t;
          switch (p->info.typ->width)
          {
            case 1: t = "int8_t"; break;
            case 2: t = "int16_t"; break;
            case 4: t = "int32_t"; break;
            default: t = "int64_t"; break;
          }
          if (p->info.typ->type == Tenumsc)
            fprintf(fheader, "enum class %s : %s {", ident(p->info.typ->id->name), t);
          else
            fprintf(fheader, "enum %s : %s {", ident(p->info.typ->id->name), t);
        }
        if ((Table*)p->info.typ->ref)
        {
          LONG64 delta = 0;
          const char *c = "";
          if (p->info.typ->type == Tenum && !is_mask(p->info.typ))
          {
            for (q = ((Table*)p->info.typ->ref)->list; q; q = q->next)
            {
              Entry *e = entry(r, q->sym);
              if (e && delta <= e->info.val.i)
                delta = e->info.val.i + 1;
            }
          }
          for (q = ((Table*)p->info.typ->ref)->list; q; q = q->next)
          {
            Entry *e = NULL;
            if (rflag)
              fprintf(freport, "%s <code> %s </code>", c, ident(q->sym->name));
            if (p->info.typ->type == Tenum && (e = entry(r, q->sym)))
            {
              fprintf(fheader, " /* %s\n\t%s = " SOAP_LONG_FORMAT " */", c, ident(q->sym->name), e->info.val.i);
              if (delta > 0)
                delta--;
            }
            else
            {
              LONG64 i = q->info.val.i + delta;
              if (delta == 0 && i > 0 && i < 128 && isalpha((int)i))
              {
                fprintf(fheader, "%s\n\t%s = '%c'", c, ident(q->sym->name), (int)q->info.val.i);
              }
              else
              {
                if (i <= 0x7FFFLL && i >= -0x8000LL)
                  fprintf(fheader, "%s\n\t%s = " SOAP_LONG_FORMAT, c, ident(q->sym->name), i);
                else
                  fprintf(fheader, "%s\n\t%s = " SOAP_LONG_FORMAT "LL", c, ident(q->sym->name), i);
              }
              if (p->info.typ->type == Tenum && !e)
              {
                e = enter(r, q->sym);
                e->info.val.i = i;
              }
              c = ",";
            }
          }
        }
        else
        {
          if (!is_transient(p->info.typ) && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_imported(p->info.typ))
          {
            sprintf(errbuf, "%s declared at %s:%d has no content, requires enum constants", c_type(p->info.typ), p->filename, p->lineno);
            semwarn(errbuf);
          }
        }
        if (cflag && p->info.typ->type == Tenum && p->sym->token == TYPE)
          fprintf(fheader, "\n} %s;", ident(p->sym->name));
        else
          fprintf(fheader, "\n};");
        if (is_volatile(p->info.typ))
          fprintf(fheader, "\n#endif");
        fprintf(fheader, "\n#endif");
      }
      if (rflag && p->info.typ->ref)
        fprintf(freport, " </td></tr>\n");
    }
    freetable(r);
    if (rflag)
    {
      int uf = uflag;
      uflag = 1;
      fprintf(freport, "</table>\n\n");
      for (p = enumtable->list; p; p = p->next)
      {
        const char *ns = prefix_of(p->sym->name);
        const char *x;
        x = xsi_type(p->info.typ);
        if (!x || !*x)
          x = wsdl_type(p->info.typ, "");
        fprintf(freport, "<a name=\"%s\"></a>\n\n", c_ident(p->info.typ));
        fprintf(freport, "### `%s`\n\n", c_type(p->info.typ));
        gen_report_type_doc(p);
        fprintf(freport, "This enum%s type is declared in [%s](%s) at line %d ", p->info.typ->type == Tenumsc ? " class" : "", p->filename, p->filename, p->lineno);
        if (is_mask(p->info.typ))
          fprintf(freport, "and is a bitmask type, meaning that it represents a set of values created with bit-wise `|` (bitor) operations, ");
        if ((Table*)p->info.typ->ref)
        {
          const char *c = "";
          fprintf(freport, "and has values ");
          for (q = ((Table*)p->info.typ->ref)->list; q; q = q->next)
          {
            if (!is_mask(p->info.typ) && q->info.val.i > 0 && q->info.val.i < 128 && isalpha((int)q->info.val.i))
              fprintf(freport, "%s `%s` (= " SOAP_LONG_FORMAT " = ASCII '%c')", c, ident(q->sym->name), q->info.val.i, (int)q->info.val.i);
            else
              fprintf(freport, "%s `%s` (= " SOAP_LONG_FORMAT ")", c, ident(q->sym->name), q->info.val.i);
            gen_report_member(p, q);
            c = ",";
          }
        }
        fprintf(freport, ".\n\n");
        if (!is_transient(p->info.typ))
        {
          if (cflag)
          {
            fprintf(freport, "- `(%s *)soap_malloc(struct soap*, sizeof(%s))` raw managed allocation\n", c_type(p->info.typ), c_type(p->info.typ));
            fprintf(freport, "- `%s *soap_new_%s(struct soap*, int n)` managed allocation with default initialization of one `%s` when `n` = 1 or array `%s[n]` when `n` > 1\n", c_type(p->info.typ), c_ident(p->info.typ), c_type(p->info.typ), c_type(p->info.typ));
          }
          else
          {
            fprintf(freport, "- `%s *soap_new_%s(struct soap*)` managed allocation with default initialization\n", c_type(p->info.typ), c_ident(p->info.typ));
            fprintf(freport, "- `%s *soap_new_%s(struct soap*, int n)` managed allocation with default initialization of array `%s[n]`\n", c_type(p->info.typ), c_ident(p->info.typ), c_type(p->info.typ));
          }
          fprintf(freport, "- `void soap_default_%s(struct soap*, %s)` set to default value\n", c_ident(p->info.typ), c_type_id(p->info.typ, "*value"));
          fprintf(freport, "- `const char *soap_%s2s(struct soap*, %s)` returns string-converted value in temporary string buffer\n", c_ident(p->info.typ), c_type_id(p->info.typ, "value"));
          fprintf(freport, "- `int soap_s2%s(struct soap*, const char*, %s)` convert string to value, returns `SOAP_OK` or error code\n", c_ident(p->info.typ), c_type_id(p->info.typ, "*value"));
          if (!wflag)
          {
            if (!is_invisible(p->sym->name) && !has_ns_eq("xsd", p->sym->name))
            {
              const char *s = ns_of(p->sym->name);
              if (s && *s && has_ns_eq(NULL, p->sym->name))
                fprintf(freport, "\nThe component of XML schema type *`%s`* in schema \"[%s](#doc-namespaces)\" is:\n\n", x, s);
              else
                fprintf(freport, "\nThe component of XML schema type *`%s`* is:\n\n", x);
              gen_schema_type(freport, NULL, p, ns, ns, 1, NULL, NULL);
            }
          }
          fprintf(freport, "\n");
        }
        gen_report_hr();
      }
      uflag = uf;
    }
  }
  banner(fheader, "Types with Custom Serializers");
  fflush(fheader);
  if (typetable)
  {
    for (p = typetable->list; p; p = p->next)
    {
      if (is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_imported(p->info.typ))
      {
        fprintf(fheader, "\n\n/* %s:%d */", p->filename, p->lineno);
        fprintf(fheader, "\n#ifndef %s", soap_type(p->info.typ));
        if (namespaceid)
          fprintf(fheader, "\n#define %s (-%d)\n", soap_type(p->info.typ), p->info.typ->num); /* namespaced SOAP_TYPE is negative to avoid clashes */
        else
          fprintf(fheader, "\n#define %s (%d)", soap_type(p->info.typ), p->info.typ->num);
        fprintf(fheader, "\n%s%s;", c_storage(p->info.sto), c_type_synonym_id(p->info.typ, p->sym->name));
        fprintf(fheader, "\n#endif");
      }
    }
  }
  if (typetable)
  {
    for (p = typetable->list; p; p = p->next)
    {
      if (p->info.typ->type == Tclass && is_eq(p->info.typ->sym->name, "xsd__QName") && !is_external(p->info.typ) && !is_imported(p->info.typ))
      {
        fprintf(fheader, "\n\n/* %s:%d */", p->filename, p->lineno);
        fprintf(fheader, "\n#ifndef %s", soap_type(p->info.typ));
        if (namespaceid)
          fprintf(fheader, "\n#define %s (-%d)\n", soap_type(p->info.typ), p->info.typ->num); /* namespaced SOAP_TYPE is negative to avoid clashes */
        else
          fprintf(fheader, "\n#define %s (%d)", soap_type(p->info.typ), p->info.typ->num);
        fprintf(fheader, "\n%sstd::string %s;", c_storage(p->info.sto), ident(p->sym->name));
        fprintf(fheader, "\n#endif\n");
      }
    }
  }
  if (classtable && classtable->list)
  {
    if (cflag)
      banner(fheader, "Structs and Unions");
    else
      banner(fheader, "Classes, Structs and Unions");
    fflush(fheader);
    if (rflag)
      fprintf(freport, "## %sStruct and Union Types {#doc-classes}\n\nThe table below lists the %sstructs and unions declared in [%s](%s) or that are imported:\n\n<table class=\"doxtable\">\n<tr><th> Type </th><th> Declared </th><th> Serializable </th></tr>\n", cflag ? "" : "Class, ", cflag ? "" : "classes, ", filename, filename);
    for (p = classtable->list; p; p = p->next)
    {
      if (rflag && p->info.typ->ref && (soap_version >= 0 || (!is_header_or_fault(p->info.typ) && !is_body(p->info.typ))))
        fprintf(freport, "<tr><td><code><a href=\"#%s\"> %s </a></code></td><td> %s:%d </td><td> %s </td></tr>\n", c_ident(p->info.typ), c_type(p->info.typ), p->filename, p->lineno, !is_transient(p->info.typ) ? "yes" : "");
      if (!is_imported(p->info.typ) && !is_volatile(p->info.typ) && p->info.typ->ref)
      {
        if (!is_header_or_fault(p->info.typ) && !is_body(p->info.typ))
        {
          if (cflag && (p->info.typ->type == Tstruct || p->info.typ->type == Tunion) && p->sym->token == TYPE)
            fprintf(fheader, "\ntypedef ");
          else
            fprintf(fheader, "\n");
          if (p->info.typ->type == Tstruct)
            fprintf(fheader, "struct %s", ident(p->sym->name));
          else if (!cflag && p->info.typ->type == Tclass)
            fprintf(fheader, "class %s", ident(p->sym->name));
          else if (p->info.typ->type == Tunion)
            fprintf(fheader, "union %s", ident(p->sym->name));
          if (cflag && (p->info.typ->type == Tstruct || p->info.typ->type == Tunion) && p->sym->token == TYPE)
            fprintf(fheader, " %s;", ident(p->sym->name));
          else
            fprintf(fheader, ";");
          fprintf(fheader, "\t/* %s:%d */", p->filename, p->lineno);
        }
      }
    }
    if (rflag)
      fprintf(freport, "</table>\n\n");
    for (p = classtable->list; p; p = p->next)
    {
      if (rflag)
        gen_class(freport, p);
      gen_class(fheader, p);
    }
  }
  if (typetable)
  {
    banner(fheader, "Typedefs");
    fflush(fheader);
    if (rflag)
    {
      fprintf(freport, "## Typedefs {#doc-typedefs}\n\n");
      fprintf(freport, "A typedef type is serializable if its underlying base type is serializable.  Typedefs may declare custom serializers, meaning their underlying types are custom-serialized in XML using serialization rules that differ from the serialization rules of the underlying base type:");
      fprintf(freport, "\n<table class=\"doxtable\">\n<tr><th> Typedef </th><th> Type </th><th> Declared </th><th> Serializable </th><th> Custom </th></tr>\n");
    }
    for (p = typetable->list; p; p = p->next)
    {
      if (rflag)
      {
        if (!is_transient(p->info.typ))
          fprintf(freport, "<tr><td><code><a href=\"#%s\"> %s </a></code></td><td><code> %s </code></td><td> %s:%d </td><td> yes </td><td> %s </td></tr>\n", c_ident(p->info.typ), c_ident(p->info.typ), c_type(p->info.typ), p->filename, p->lineno, is_external(p->info.typ) || is_qname(p->info.typ) || is_stdqname(p->info.typ) || is_XML(p->info.typ) || is_stdXML(p->info.typ) ? "yes" : "");
        else
          fprintf(freport, "<tr><td><code> %s </code></td><td><code> %s </code></td><td> %s:%d </td><td> </td><td> </td></tr>\n", c_ident(p->info.typ), c_type(p->info.typ), p->filename, p->lineno);
      }
      if (!wflag && !is_primitive_or_string(p->info.typ) && !is_external(p->info.typ) && !is_XML(p->info.typ) && !is_transient(p->info.typ) && !has_ns_t(p->info.typ) && !is_imported(p->info.typ) && !is_template(p->info.typ))
      {
        sprintf(errbuf, "serializable typedef '%s' is not namespace qualified: schema definition for '%s' in WSDL file output may be invalid", p->sym->name, p->sym->name);
        semwarn(errbuf);
      }
      if (p->info.typ->type == Tclass && is_eq(p->info.typ->sym->name, "xsd__QName") && !is_external(p->info.typ) && !is_imported(p->info.typ))
        continue;
      if (!(is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_imported(p->info.typ)) || is_synonym(p->info.typ))
      {
        fprintf(fheader, "\n\n/* %s:%d */", p->filename, p->lineno);
        fprintf(fheader, "\n#ifndef %s", soap_type(p->info.typ));
        if (namespaceid)
          fprintf(fheader, "\n#define %s (-%d)\n", soap_type(p->info.typ), p->info.typ->num); /* namespaced SOAP_TYPE is negative to avoid clashes */
        else
          fprintf(fheader, "\n#define %s (%d)", soap_type(p->info.typ), p->info.typ->num);
        fprintf(fheader, "\n%s%s;", c_storage(p->info.sto), c_type_synonym_id(p->info.typ, p->sym->name));
        fprintf(fheader, "\n#endif");
      }
    }
    if (rflag)
    {
      fprintf(freport, "</table>\n\n");
      for (p = typetable->list; p; p = p->next)
      {
        if (!is_transient(p->info.typ))
        {
          fprintf(freport, "<a name=\"%s\"></a>\n\n### `%s`\n\n", c_ident(p->info.typ), c_ident(p->info.typ));
          gen_report_type_doc(p);
          if (p->lineno)
            fprintf(freport, "This typedef is declared in [%s](%s) at line %d", p->filename, p->filename, p->lineno);
          else
            fprintf(freport, "This typedef is internally generated");
          if (has_ns_t(p->info.typ))
            fprintf(freport, ", is serialized as XML schema type *`%s`*", xsi_type(p->info.typ));
          if (is_XML(p->info.typ) || is_stdXML(p->info.typ))
            fprintf(freport, " and is a built-in string type to hold XML that is literally serialized to and from XML");
          else if (is_qname(p->info.typ) || is_stdqname(p->info.typ))
            fprintf(freport, " and is a built-in string type to serialize a list of space-separated qualified names (*`xsd:QName`*), such that XML namespace prefixes are normalized to the XML prefixes defined in the [namespace table](#doc-namespaces) or replaced with \"URI\": when the namespace table has no prefix entry for the URI");
          else if (is_anyType(p->info.typ))
            fprintf(freport, " and is a built-in XML DOM element node graph");
          else if (is_anyAttribute(p->info.typ))
            fprintf(freport, " and is a built-in XML DOM attribute list");
          else
            fprintf(freport, " and has underlying base type `%s`", c_type(p->info.typ));
          if (is_primitive_or_string(p->info.typ) && (p->info.typ->pattern || p->info.typ->hasmin || p->info.typ->hasmax))
          {
            fprintf(freport, ", which should be\n");
            if (p->info.typ->pattern)
            {
              if (p->info.typ->pattern[0] == '%' && p->info.typ->pattern[1])
              {
                unsigned int n = (unsigned int)strtoul(p->info.typ->pattern + 1, NULL, 10);
                unsigned int f = 0;
                const char *s = strchr(p->info.typ->pattern, '.');
                if (s)
                  f = (unsigned int)strtoul(s + 1, NULL, 10);
                if (n)
                  fprintf(freport, "\n- number of total digits: %u", n);
                if (f)
                  fprintf(freport, "\n- number of fraction digits: %u", f);
              }
              else
                fprintf(freport, "\n- matching regex pattern \"%s\"", p->info.typ->pattern);
            }
            if (p->info.typ->hasmin)
            {
              if (p->info.typ->type >= Tfloat && p->info.typ->type <= Tldouble)
              {
                if (p->info.typ->incmin)
                  fprintf(freport, "\n- greater than or equal to %.16lG\n", p->info.typ->rmin);
                else
                  fprintf(freport, "\n- greater than %.16lG\n", p->info.typ->rmin);
              }
              else if (p->info.typ->type >= Tchar && p->info.typ->type <= Tullong)
              {
                if (p->info.typ->incmin)
                  fprintf(freport, "\n- greater than or equal to " SOAP_LONG_FORMAT "\n", p->info.typ->imin);
                else
                  fprintf(freport, "\n- greater than " SOAP_LONG_FORMAT "\n", p->info.typ->imin);
              }
              else if (p->info.typ->hasmax && p->info.typ->imax >= 0 && p->info.typ->incmin && p->info.typ->incmax && p->info.typ->imin == p->info.typ->imax)
                fprintf(freport, "\n- equal to " SOAP_LONG_FORMAT " characters in length\n", p->info.typ->imin);
              else
                fprintf(freport, "\n- longer than or equal to " SOAP_LONG_FORMAT " characters in length\n", p->info.typ->imin);
            }
            if (p->info.typ->hasmax)
            {
              if (p->info.typ->type >= Tfloat && p->info.typ->type <= Tldouble)
              {
                if (p->info.typ->incmax)
                  fprintf(freport, "\n- less than or equal to %.16lG\n", p->info.typ->rmax);
                else
                  fprintf(freport, "\n- less than %.16lG\n", p->info.typ->rmax);
              }
              else if (p->info.typ->type >= Tchar && p->info.typ->type <= Tullong)
              {
                if (p->info.typ->incmax)
                  fprintf(freport, "\n- less than or equal to " SOAP_LONG_FORMAT "\n", p->info.typ->imax);
                else
                  fprintf(freport, "\n- less than " SOAP_LONG_FORMAT "\n", p->info.typ->imax);
              }
              else if (p->info.typ->hasmax && p->info.typ->imax >= 0 && p->info.typ->incmin && p->info.typ->incmax && p->info.typ->imin == p->info.typ->imax)
                ;
              else
                fprintf(freport, "\n- shorter than or equal to " SOAP_LONG_FORMAT " characters in length\n", p->info.typ->imin);
            }
          }
          fprintf(freport, "\n\n");
          gen_report_hr();
        }
      }
    }
  }
  banner(fheader, "Serializable Types");
  fflush(fheader);
  if (rflag)
  {
    fprintf(freport, "## Summary of Serializable Types {#doc-types}\n\n");
    fprintf(freport, "Each serializable %s *Type* with binding name *Name* has a set of auto-generated functions:\n\n", cflag ? "C" : "C/C++");
    if (cflag)
    {
      fprintf(freport, "- `Type *soap_new_Name(struct soap*, int n)` managed allocation with default initialization of one *Type* when `n` = 1 or array *Type*`[n]` when `n` > 1\n");
      fprintf(freport, "- `void soap_default_Name(struct soap*, Type*)` initialize or reset *Type* to default\n");
    }
    else
    {
      fprintf(freport, "- `Type *soap_new_Name(struct soap*)` managed allocation and default initialization\n");
      fprintf(freport, "- `Type *soap_new_Name(struct soap*, int n)` managed allocation and default initialization of an array `Type[n]`\n");
      fprintf(freport, "- `void soap_default_Name(struct soap*, Type*)` initialize or reset non-class *Type* to default)\n");
      fprintf(freport, "- `void Type::soap_default(struct soap*)` non-volatile class *Type* reset to default\n");
    }
    fprintf(freport, "- `Type *soap_dup_Name(struct soap*, Type *dst, const Type *src)` requires soapcpp2 option -Ec, deep copy `src` to `dst` managed by context or unmanaged when context is NULL, returning `dst` (if `dst` is NULL then allocates `dst` copy)\n");
    fprintf(freport, "- `void soap_del_Name(struct soap*, Type*)` requires soapcpp2 option -Ec, deep delete *Type* which must be unmanaged\n");
    fprintf(freport, "- `const char *soap_Name2s(struct soap*, Type)` primitive *Type* only, returns string-converted *Type* in temporary string buffer\n");
    fprintf(freport, "- `int soap_s2Name(struct soap*, const char*, Type*)` primitive *Type* only, convert string to value, returns `SOAP_OK` or error code\n");
    fprintf(freport, "- `int soap_write_Name(struct soap*, const Type*)` serialize *Type* to XML, returns `SOAP_OK` or error code\n");
    fprintf(freport, "- `int soap_PUT_Name(struct soap*, const char *URL, const Type*)` REST PUT *Type* in XML, returns `SOAP_OK` or error code\n");
    fprintf(freport, "- `int soap_PATCH_Name(struct soap*, const char *URL, const Type*)` REST PATCH *Type* in XML, returns `SOAP_OK` or error code\n");
    fprintf(freport, "- `int soap_POST_send_Name(struct soap*, const char *URL, const Type*)` REST POST send *Type* in XML (MUST be followed by a `soap_POST_recv_OtherName`), returns `SOAP_OK` or error code\n");
    fprintf(freport, "- `int soap_read_Name(struct soap*, Type*)` deserialize *Type* from XML, returns `SOAP_OK` or error code\n");
    fprintf(freport, "- `int soap_GET_Name(struct soap*, const char *URL, Type*)` REST GET *Type* from XML, returns `SOAP_OK` or error code\n");
    fprintf(freport, "- `int soap_POST_recv_Name(struct soap*, Type*)` REST GET *Type* from XML (after a `soap_POST_send_OtherName`), returns `SOAP_OK` or error code\n");
    if (cflag)
      fprintf(freport, "- `(Type *)soap_malloc(struct soap*, sizeof(Type))` raw managed allocation of *Type* without initialization\n");
    else
      fprintf(freport, "- `(Type *)soap_malloc(struct soap*, sizeof(Type))` raw managed allocation of primitive *Type* (types that are not structs or classes) without initialization\n");
    fprintf(freport, "- `const char *soap_strdup(struct soap*, const char*)` managed allocation and duplication of string\n");
    fprintf(freport, "- `const wchar_t *soap_wstrdup(struct soap*, const wchar_t*)` managed allocation and duplication of wide string\n");
    if (namespaceid)
      fprintf(freport, "\nEach *Type* also has a unique type id `SOAP_TYPE_%s_<Type>` that you can use to serialize `void*` in a struct/class by setting the `int __type` member to this type id.", namespaceid);
    else
      fprintf(freport, "\nEach *Type* also has a unique type id `SOAP_TYPE_<Type>` that you can use to serialize `void*` in a struct/class by setting the `int __type` member to this type id.");
    if (!cflag)
      fprintf(freport, " The unique type id is also used to distinguish derived class instances from base class instances by calling their `virtual soap_type()` methods that return this type id.");
    fprintf(freport, "\n\nFrom the toolkit documentation:\n\n");
    if (cflag)
      fprintf(freport, "- Set `soap->sendfd = fd` to serialize to an `int fd` file descriptor\n- Set `soap->os = &cs` to serialize to a string `const char *cs`, which will be assigned by the engine and set to point to a managed string that is automatically deleted\n- Set `soap->recvfd = fd` to deserialize from an `int fd` file descriptor\n- Set `soap->is = cs` to deserialize from a `const char *cs` string\n- All managed allocated data is freed by `soap_end(soap)` with context `soap`\n");
    else
      fprintf(freport, "- Set `soap->sendfd = fd` to serialize to an `int fd` file descriptor\n- Set `soap->os = &os` to serialize to a `std::ostream os`\n- Set `soap->recvfd = fd` to deserialize from an `int fd` file descriptor\n- Set `soap->is = &is` to deserialize from a `std::istream`\n- All managed allocated data is deleted by `soap_destroy(soap)` followed by `soap_end(soap)` with context `soap`\n");
    fprintf(freport, "\nThe table below lists the serializable types by *Type*, binding *Name*, *Kind*, and the XSD data binding type and/or element.  Pointers, arrays, and containers of these types are also serializable:\n\n<table class=\"doxtable\">\n<tr><th> %s Type </th><th> Name </th><th> Kind </th><th> XML schema name </th></tr>\n", cflag ? "C" : "C/C++");
  }
  for (i = 0; i < TYPES; i++)
  {
    if (i != Tnone && i != Tvoid && i != Tsize && i != Tunion && i != Treference && i != Trvalueref && i != Tfun)
    {
      Tnode *p;
      for (p = Tptr[i]; p; p = p->next)
      {
        if (!is_transient(p) && !is_invisible(p->id->name))
        {
          fprintf(fheader, "\n\n/* %s has binding name '%s' for type '%s' */", c_type_sym(p), c_ident(p), xsi_type(p));
          fprintf(fheader, "\n#ifndef %s", soap_type(p));
          if (namespaceid)
            fprintf(fheader, "\n#define %s (-%d)\n", soap_type(p), p->num); /* namespaced SOAP_TYPE is negative to avoid clashes */
          else
            fprintf(fheader, "\n#define %s (%d)", soap_type(p), p->num);
          fprintf(fheader, "\n#endif");
          if (rflag && (p->type != Tpointer || is_string(p) || is_wstring(p)))
          {
            if ((p->type == Tstruct || p->type == Tclass || p->type == Tenum || p->type == Tenumsc || is_typedef(p)) && !is_stdstr(p))
              fprintf(freport, "<tr><td><code><a href=\"#%s\"> %s </a></code></td><td><code> %s </code></td><td> %s </td><td> %s </td></tr>\n", c_ident(p), c_type_sym(p), c_ident(p), kind_of(p), is_XML(p) ? "(literal XML string)" : is_anyType(p) ? "(XML DOM element)" : is_anyAttribute(p) ? "(XML DOM attribute list)" : xsi_type(p));
            else
              fprintf(freport, "<tr><td><code> %s </code></td><td><code> %s </code></td><td> %s </td><td> %s </td></tr>\n", c_type_sym(p), c_ident(p), kind_of(p), is_XML(p) ? "(literal XML string)" : is_anyType(p) ? "(XML DOM element node)" : is_anyAttribute(p) ? "(XML DOM attribute list)" : xsi_type(p));
          }
        }
      }
    }
  }
  if (rflag)
  {
    fprintf(freport, "</table>\n\n");
    gen_report_hr();
  }
  banner(fheader, "Externals");
  fflush(fheader);
  if (t)
  {
    for (p = t->list; p; p = p->next)
    {
      if (p->info.typ->type != Tfun || (p->info.sto & Sextern))
      {
        fprintf(fheader, "\n\n/* %s:%d */\n", p->filename, p->lineno);
        if (!(p->info.sto & Sstatic))
          fprintf(fheader, "extern ");
        fprintf(fheader, "%s", c_storage(p->info.sto));
        fprintf(fheader, "%s", c_type_id(p->info.typ, p->sym->name));
        fprintf(fheader, "%s;", c_init(p));
      }
    }
  }
  fflush(fheader);
}

void
get_namespace_prefixes(void)
{
  Symbol *p, *q;
  Entry *e, *v;
  int i, n;
  const char *s;
  char buf[256];
  if (nslist)
    return;
  for (p = symlist; p; p = p->next)
  {
    s = p->name;
    if (!strncmp(s, "__size", 6) || !strncmp(s, "__type", 6) || !strncmp(s, "__union", 7))
      continue;
    while (*s == '_' || *s == '~')
      s++;
    n = (int)(strlen(s) - 2);
    for (i = 1; i < n; i++)
    {
      if (s[i] == ':' || (s[i-1] != '_' && s[i] == '_' && s[i+1] == '_' && s[i+2] && (s[i+2] != '_' || is_special(s+i+2))))
      {
        if (s[i+1] == ':')
        {
          i++;
          continue;
        }
        strncpy(buf, s, i);
        buf[i] = '\0';
        if (!strcmp(buf, "SOAP_ENV") || !strcmp(buf, "SOAP_ENC") || !strcmp(buf, "xsd") || !strcmp(buf, "xsi") || !strcmp(buf, "xml") || !strcmp(buf, "std") || !strncmp(buf, "soap_", 5))
          goto nsnext;
        for (q = nslist; q; q = q->next)
          if (!strcmp(q->name, buf))
            goto nsnext;
        if (enumtable)
          for (e = enumtable->list; e; e = e->next)
            if (!strcmp(buf, e->sym->name))
              if (e->info.typ->ref)
                for (v = ((Table*)e->info.typ->ref)->list; v; v = v->next)
                  if (p == v->sym)
                    goto nsnext;
        q = (Symbol*)emalloc(sizeof(Symbol)+i);
        strcpy(q->name, buf);
        q->next = nslist;
        nslist = q;
        break;
      }
    }
nsnext:
    ;
  }
  q = (Symbol*)emalloc(sizeof(Symbol)+4);
  strcpy(q->name, "xsd");
  q->next = nslist;
  nslist = q;
  q = (Symbol*)emalloc(sizeof(Symbol)+4);
  strcpy(q->name, "xsi");
  q->next = nslist;
  nslist = q;
  q = (Symbol*)emalloc(sizeof(Symbol)+9);
  strcpy(q->name, "SOAP-ENC");
  q->next = nslist;
  nslist = q;
  q = (Symbol*)emalloc(sizeof(Symbol)+9);
  strcpy(q->name, "SOAP-ENV");
  q->next = nslist;
  nslist = q;
}

void
generate_schema(Table *t)
{
  Entry *p = NULL;
  Symbol *ns;
  const char *name = NULL;
  const char *URL = NULL;
  const char *executable = NULL;
  const char *URI = NULL;
  const char *style = NULL;
  const char *encoding = NULL;
  const char *protocol = NULL;
  const char *import = NULL;
  Service *sp = NULL;
  char buf[4096];
  FILE *fd;
  get_namespace_prefixes();
  for (ns = nslist; ns; ns = ns->next)
  {
    if (!strcmp(ns->name, "SOAP-ENV") || !strcmp(ns->name, "SOAP-ENC") || !strcmp(ns->name, "xsi") || !strcmp(ns->name, "xsd"))
      continue;
    name = NULL;
    URL = NULL;
    executable = NULL;
    URI = NULL;
    style = NULL;
    encoding = NULL;
    import = NULL;
    for (sp = services; sp; sp = sp->next)
    {
      if (!tagcmp(sp->ns, ns->name))
      {
        name = ns_cname(sp->name, NULL);
        URL = sp->URL;
        executable = sp->executable;
        URI = sp->URI;
        style = sp->style;
        encoding = sp->encoding;
        protocol = sp->protocol;
        import = sp->import;
        break;
      }
    }
    if (!URI)
    {
      char *s = (char*)emalloc(strlen(tmpURI) + strlen(ns->name) + 6);
      sprintf(s, "%s/%s.xsd", tmpURI, ns_convert(ns->name));
      URI = s;
    }
    if (soap_version >= 0 && is_document(style) && encoding && !*encoding)
    {
      semwarn("Cannot use document style with SOAP encoding");
      encoding = NULL;
    }
    if (!name)
      name = "Service";
    if (t)
    {
      for (p = t->list; p; p = p->next)
      {
        if (p->info.typ->type == Tfun && !(p->info.sto & Sextern) && has_ns_eq(ns->name, p->sym->name))
        {
          if (name)
            fprintf(fmsg, "Using %s service name: %s\n", ns->name, name);
          if (protocol)
            fprintf(fmsg, "Using %s service protocol: %s\n", ns->name, protocol);
          if (style && soap_version >= 0)
            fprintf(fmsg, "Using %s service style: %s\n", ns->name, style);
          else if (!eflag && soap_version >= 0)
            fprintf(fmsg, "Using %s service style: document\n", ns->name);
          else if (eflag)
            fprintf(fmsg, "Using %s service style: rpc\n", ns->name);
          if (encoding && *encoding)
            fprintf(fmsg, "Using %s service encoding: %s\n", ns->name, encoding);
          else if (encoding && !*encoding && soap_version >= 0)
            fprintf(fmsg, "Using %s service encoding: encoded\n", ns->name);
          else if (!eflag && soap_version >= 0)
            fprintf(fmsg, "Using %s service encoding: literal\n", ns->name);
          else if (eflag)
            fprintf(fmsg, "Using %s service encoding: encoded\n", ns->name);
          if (URL)
            fprintf(fmsg, "Using %s service location: %s\n", ns->name, URL);
          if (executable)
            fprintf(fmsg, "Using %s service executable: %s\n", ns->name, executable);
          if (import)
            fprintf(fmsg, "Using %s schema import namespace: %s\n", ns->name, import);
          else if (URI)
            fprintf(fmsg, "Using %s schema namespace: %s\n", ns->name, URI);
          if (sp && sp->name)
            sprintf(buf, "%s%s.wsdl", dirpath, ns_cname(name, NULL));
          else
            sprintf(buf, "%s%s.wsdl", dirpath, ns_cname(ns->name, NULL));
          if (!wflag && !import)
          {
            fprintf(fmsg, "Saving %s Web Service description\n", buf);
            fd = fopen(buf, "w");
            if (!fd)
              execerror("Cannot write WSDL file");
            gen_wsdl(fd, t, ns->name, name, URL ? URL : "http://localhost:80", executable, URI, style, encoding, protocol);
            if (fclose(fd))
              execerror("Cannot write to file");
          }
          if (!cflag)
          {
            if (iflag || jflag)
            {
              const char *sname;
              if (sp && sp->name)
                sname = sp->name;
              else
                sname = "";
              if (!Sflag)
              {
                const char *name1 = ns_cname(sname, "Proxy");
                sprintf(soapProxyH, "%s%s.h", prefix, name1);
                sprintf(pathsoapProxyH, "%s%s", dirpath, soapProxyH);
                sprintf(soapProxyC, "%s%s.cpp", prefix, name1);
                sprintf(pathsoapProxyC, "%s%s", dirpath, soapProxyC);
                fprintf(fmsg, "Saving %s client proxy class\n", pathsoapProxyH);
                fd = fopen(pathsoapProxyH, "w");
                if (!fd)
                  execerror("Cannot write proxy class file");
                copyrightnote(fd, soapProxyH);
                if (rflag)
                  gen_proxy_header(freport, t, ns, name1);
                gen_proxy_header(fd, t, ns, name1);
                if (fclose(fd))
                  execerror("Cannot write to file");
                fprintf(fmsg, "Saving %s client proxy class\n", pathsoapProxyC);
                fd = fopen(pathsoapProxyC, "w");
                if (!fd)
                  execerror("Cannot write proxy class file");
                copyrightnote(fd, soapProxyC);
                gen_proxy_code(fd, t, ns, name1);
                if (fclose(fd))
                  execerror("Cannot write to file");
              }
              if (!Cflag)
              {
                const char *name1 = ns_cname(sname, "Service");
                sprintf(soapServiceH, "%s%s.h", prefix, name1);
                sprintf(pathsoapServiceH, "%s%s", dirpath, soapServiceH);
                sprintf(soapServiceC, "%s%s.cpp", prefix, name1);
                sprintf(pathsoapServiceC, "%s%s", dirpath, soapServiceC);
                fprintf(fmsg, "Saving %s service class\n", pathsoapServiceH);
                fd = fopen(pathsoapServiceH, "w");
                if (!fd)
                  execerror("Cannot write service class file");
                copyrightnote(fd, soapServiceH);
                if (rflag)
                  gen_object_header(freport, t, ns, name1);
                gen_object_header(fd, t, ns, name1);
                if (fclose(fd))
                  execerror("Cannot write to file");
                fprintf(fmsg, "Saving %s service class\n", pathsoapServiceC);
                fd = fopen(pathsoapServiceC, "w");
                if (!fd)
                  execerror("Cannot write service class file");
                copyrightnote(fd, soapServiceC);
                gen_object_code(fd, t, ns, name1);
                if (fclose(fd))
                  execerror("Cannot write to file");
              }
            }
            else if (zflag == 1)
            {
              if (!Sflag && sp && sp->name)
              {
                sprintf(buf, "%s%s%s.h", dirpath, prefix, ns_cname(name, "Proxy"));
                fprintf(fmsg, "Saving %s simple client proxy (deprecated)\n", buf);
                fd = fopen(buf, "w");
                if (!fd)
                  execerror("Cannot write proxy file");
                sprintf(buf, "%s%s.h", prefix, ns_cname(name, "Proxy"));
                copyrightnote(fd, buf);
                gen_proxy(fd, t, ns, name, URL ? URL : "http://localhost:80");
                if (fclose(fd))
                  execerror("Cannot write to file");
              }
              else if (!Sflag)
              {
                sprintf(buf, "%s%s.h", dirpath, ns_cname(prefix, "Proxy"));
                fprintf(fmsg, "Saving %s simple client proxy (deprecated)\n", buf);
                fd = fopen(buf, "w");
                if (!fd)
                  execerror("Cannot write proxy file");
                sprintf(buf, "%s.h", ns_cname(prefix, "Proxy"));
                copyrightnote(fd, buf);
                gen_proxy(fd, t, ns, "Service", URL ? URL : "http://localhost:80");
                if (fclose(fd))
                  execerror("Cannot write to file");
              }
              if (!Cflag && sp && sp->name)
              {
                sprintf(buf, "%s%s%s.h", dirpath, prefix, ns_cname(name, "Object"));
                fprintf(fmsg, "Saving %s simple server object (deprecated)\n", buf);
                fd = fopen(buf, "w");
                if (!fd)
                  execerror("Cannot write server object file");
                sprintf(buf, "%s%s.h", prefix, ns_cname(name, "Object"));
                copyrightnote(fd, buf);
                gen_object(fd, t, name);
                if (fclose(fd))
                  execerror("Cannot write to file");
              }
              else if (!Cflag)
              {
                sprintf(buf, "%s%s.h", dirpath, ns_cname(prefix, "Object"));
                fprintf(fmsg, "Saving %s simple server object (deprecated)\n", buf);
                fd = fopen(buf, "w");
                if (!fd)
                  execerror("Cannot write server object file");
                sprintf(buf, "%s.h", ns_cname(prefix, "Object"));
                copyrightnote(fd, buf);
                gen_object(fd, t, "Service");
                if (fclose(fd))
                  execerror("Cannot write to file");
              }
            }
          }
          if (!xflag)
          {
            strcpy(buf, dirpath);
            if (sp && sp->name)
              strcat(buf, ns_fname(name));
            else
              strcat(buf, ns_fname(ns->name));
            strcat(buf, ".");
            gen_data(buf, t, ns->name, encoding);
          }
          break;
        }
      }
      if (sp && sp->name)
      {
        has_nsmap = 1;
        if (nflag)
          sprintf(buf, "%s%s.nsmap", dirpath, prefix);
        else
          sprintf(buf, "%s%s.nsmap", dirpath, ns_cname(name, NULL));
        fprintf(fmsg, "Saving %s namespace mapping table\n", buf);
        fd = fopen(buf, "w");
        if (!fd)
          execerror("Cannot write nsmap file");
        /* fprintf(fd, "\n#include \"%sH.h\"", prefix); better to leave to users to include this, so use stdsoap2.h instead: */
        fprintf(fd, "\n#include \"stdsoap2.h\"\n/* This defines the global XML namespaces[] table to #include and compile */");
        if (nflag)
          fprintf(fd, "\nSOAP_NMAC struct Namespace %s_namespaces[] = ", prefix);
        else
          fprintf(fd, "\nSOAP_NMAC struct Namespace namespaces[] = ");
        gen_nsmap(fd);
        if (fclose(fd))
          execerror("Cannot write to file");
        if (rflag)
        {
          Symbol *ns1;
          fprintf(freport, "## Schemas and Namespaces {#doc-namespaces}\n\nThe following schemas and namespaces are used in addition to the predefined SOAP and built-in XSD namespaces:\n\n");
          for (ns1 = nslist; ns1; ns1 = ns1->next)
          {
            if (strcmp(ns1->name, "SOAP-ENV") && strcmp(ns1->name, "SOAP-ENC") && strcmp(ns1->name, "xsi") && strcmp(ns1->name, "xsd"))
            {
              Service *sp1;
              for (sp1 = services; sp1; sp1 = sp1->next)
                if (!tagcmp(sp1->ns, ns1->name) && sp1->URI)
                  break;
              if (sp1)
                fprintf(freport, "- Prefix `%s` is bound to namespace URI *`%s`*", ns_convert(ns1->name), sp1->URI);
              else
                fprintf(freport, "- Prefix `%s` is bound to namespace URI *`%s/%s.xsd`*", ns_convert(ns1->name), tmpURI, ns_convert(ns1->name));
              if (!wflag)
              {
                if (sp1 && sp1->name)
                  fprintf(freport, " of service WSDL [%s.wsdl](%s%s.wsdl) and schema [%s.xsd](%s%s.xsd)", ns_cname(sp1->name, NULL), dirpath, ns_cname(sp1->name, NULL), ns_cname(ns1->name, NULL), dirpath, ns_cname(ns1->name, NULL));
                else
                  fprintf(freport, " of schema [%s.xsd](%s%s.xsd)", ns_cname(ns1->name, NULL), dirpath, ns_cname(ns1->name, NULL));
              }
              if (sp1 && (sp1->elementForm || sp1->attributeForm))
                fprintf(freport, " with local element form default *`%s`* and attribute form default *`%s`*\n", sp->elementForm?sp->elementForm:"unqualified", sp->attributeForm?sp->attributeForm:"unqualified");
              else
                fprintf(freport, " with local element and attribute form default *`unqualified`*\n");
            }
          }
          fprintf(freport, "\n\nThe following namespace table is saved to %s%s.nsmap:\n\n", dirpath, nflag ? prefix : ns_cname(name, NULL));
          if (nflag)
            fprintf(freport, "    struct Namespace %s_namespaces[] = ", prefix);
          else
            fprintf(freport, "    struct Namespace namespaces[] = ");
          gen_nsmap(freport);
          fprintf(freport, "\nThe table binds XML namespace prefixes (first column) to namespace URIs (second column), similar to xmlns:prefix=\"URI\" in XML.  The third column is a URI pattern with `*` wildcards that is also accepted as a valid namespace URI for inbound XML messages.  The fourth column is NULL.  This table is globally defined for the gSOAP engine.  The engine context will look for it unless you compiled the gSOAP source codes with `-DWITH_NONAMESPACES`.  You must assign this or another namespace table with `soap_set_namespaces(struct soap *soap, struct Namespace *namespaces)` after initializing the context and before processing XML.\n\n");
          gen_report_hr();
        }
        if (Cflag)
          Tflag = 0;
        if (Tflag)
        {
          Entry *method;
          const char *name1 = NULL;
          Tflag = 0;
          strcpy(soapTester, prefix);
          strcat(soapTester, "Tester");
          if (cflag)
            strcat(soapTester, ".c");
          else
            strcat(soapTester, ".cpp");
          strcpy(pathsoapTester, dirpath);
          strcat(pathsoapTester, soapTester);
          fprintf(fmsg, "Saving %s auto-test echo server\n", pathsoapTester);
          fd = fopen(pathsoapTester, "w");
          if (!fd)
            execerror("Cannot write to file");
          copyrightnote(fd, soapTester);
          fprintf(fd, "\n/*\n   Stand-alone server auto-test code:\n   Takes request from standard input or over TCP/IP socket and returns\nresponse to standard output or socket\n\n   Compile:\n   cc soapTester.c soapServer.c soapC.c stdsoap2.c\n\n   Command line usage with redirect over stdin/out:\n   > ./a.out < SomeTest.req.xml\n   > ./a.out 12288 < SomeTest.req.xml\n     Note: 12288 = SOAP_XML_INDENT | SOAP_XML_STRICT (see codes in stdsoap2.h)\n   Command line usage to start server at port 8080:\n   > a.out 12288 8080\n*/");
          if (iflag || jflag)
          {
            const char *sname;
            if (sp && sp->name)
              sname = sp->name;
            else
              sname = "";
            name1 = ns_cname(sname, "Service");
            if (namespaceid)
              fprintf(fd, "\n\n#include \"%s%s%s.h\"\n#include \"%s%s.nsmap\"\n\n\n\n#ifndef SOAP_DEFMAIN\n# define SOAP_DEFMAIN main\t/* redefine to use your own main() */\n#endif\n\nint SOAP_DEFMAIN(int argc, char **argv)\n{\n\t%s::%s service(argc > 1 ? atoi(argv[1]) : 0);\n\tif (argc <= 2)\n\t\treturn service.serve();\n\twhile (service.run(atoi(argv[2])) != SOAP_OK && service.%serror != SOAP_TCP_ERROR)\n\t\tservice.soap_print_fault(stderr);\n\treturn 0;\n}\n", dirpath, prefix, name1, dirpath, nflag?prefix:ns_cname(name, NULL), namespaceid, name1, iflag ? "" : "soap->");
            else
              fprintf(fd, "\n\n#include \"%s%s%s.h\"\n#include \"%s%s.nsmap\"\n\n\n\n#ifndef SOAP_DEFMAIN\n# define SOAP_DEFMAIN main\t/* redefine to use your own main() */\n#endif\n\nint SOAP_DEFMAIN(int argc, char **argv)\n{\n\t%s service(argc > 1 ? atoi(argv[1]) : 0);\n\tif (argc <= 2)\n\t\treturn service.serve();\n\twhile (service.run(atoi(argv[2])) != SOAP_OK && service.%serror != SOAP_TCP_ERROR)\n\t\tservice.soap_print_fault(stderr);\n\treturn 0;\n}\n", dirpath, prefix, name1, dirpath, nflag?prefix:ns_cname(name, NULL), name1, iflag ? "" : "soap->");
          }
          else
          {
            if (namespaceid)
              fprintf(fd, "\n\n#include \"%s%sH.h\"\n#include \"%s%s.nsmap\"\n\n#ifndef SOAP_DEFMAIN\n# define SOAP_DEFMAIN main\t/* redefine to use your own main() */\n#endif\n\nint SOAP_DEFMAIN(int argc, char **argv)\n{\n\tstruct soap *soap = soap_new1(argc > 1 ? atoi(argv[1]) : 0);\n\tif (argc <= 2)\n\t\treturn %s::%s_serve(soap);\n\tif (soap_valid_socket(soap_bind(soap, NULL, atoi(argv[2]), 100)))\n\t{\twhile (soap_valid_socket(soap_accept(soap)))\n\t\t{\tif (%s::%s_serve(soap))\n\t\t\t\tsoap_print_fault(soap, stderr);\n\t\t\tsoap_destroy(soap);\n\t\t\tsoap_end(soap);\n\t\t}\n\t}\n\tsoap_destroy(soap);\n\tsoap_end(soap);\n\tsoap_free(soap);\n\treturn 0;\n}\n", dirpath, prefix, dirpath, nflag?prefix:ns_cname(name, NULL), namespaceid, nflag?prefix:"soap", namespaceid, nflag?prefix:"soap");
            else
              fprintf(fd, "\n\n#include \"%s%sH.h\"\n#include \"%s%s.nsmap\"\n\n#ifndef SOAP_DEFMAIN\n# define SOAP_DEFMAIN main\t/* redefine to use your own main() */\n#endif\n\nint SOAP_DEFMAIN(int argc, char **argv)\n{\n\tstruct soap *soap = soap_new1(argc > 1 ? atoi(argv[1]) : 0);\n\tif (argc <= 2)\n\t\treturn %s_serve(soap);\n\tif (soap_valid_socket(soap_bind(soap, NULL, atoi(argv[2]), 100)))\n\t{\twhile (soap_valid_socket(soap_accept(soap)))\n\t\t{\tif (%s_serve(soap))\n\t\t\t\tsoap_print_fault(soap, stderr);\n\t\t\tsoap_destroy(soap);\n\t\t\tsoap_end(soap);\n\t\t}\n\t}\n\tsoap_destroy(soap);\n\tsoap_end(soap);\n\tsoap_free(soap);\n\treturn 0;\n}\n", dirpath, prefix, dirpath, nflag?prefix:ns_cname(name, NULL), nflag?prefix:"soap", nflag?prefix:"soap");
          }
          if (namespaceid)
            fprintf(fd, "\nnamespace %s {", namespaceid);
          for (method = t->list; method; method = method->next)
          {
            if (method->info.typ->type == Tfun && !(method->info.sto & Sextern))
            {
              Entry *p = NULL, *q = entry(t, method->sym);
              Table *r;
              if (q)
                p = (Entry*)q->info.typ->ref;
              else
              {
                fprintf(stderr, "Internal error: no table entry\n");
                return;
              }
              q = entry(classtable, method->sym);
              r = (Table*)q->info.typ->ref;
              if (iflag || jflag)
                fprintf(fd, "\n\n/** Auto-test server operation %s */\nint %s::%s(", method->sym->name, name1, ns_cname(method->sym->name, NULL));
              else
                fprintf(fd, "\n\n/** Auto-test server operation %s */\nint %s(struct soap *soap", method->sym->name, ident(method->sym->name));
              gen_params(fd, r, p, !iflag && !jflag);
              /* single param to single param echo */
              if (p && r && r->list && r->list->info.typ == p->info.typ)
                fprintf(fd, "\n{\n\t/* Echo request-response parameter */\n\t*%s = *%s;\n\treturn SOAP_OK;\n}\n", ident(p->sym->name), ident(r->list->sym->name));
              else if (p && r && r->list && p->info.typ->type == Tpointer && r->list->info.typ == (Tnode*)p->info.typ->ref)
                fprintf(fd, "\n{\n\t/* Echo request-response parameter */\n\t*%s = %s;\n\treturn SOAP_OK;\n}\n", ident(p->sym->name), ident(r->list->sym->name));
              else if (p && r && r->list && p->info.typ->type == Treference && r->list->info.typ == (Tnode*)p->info.typ->ref)
                fprintf(fd, "\n{\n\t/* Echo request-response parameter */\n\t%s = %s;\n\treturn SOAP_OK;\n}\n", ident(p->sym->name), ident(r->list->sym->name));
              else if (p && r && r->list && p->info.typ->type == Treference && r->list->info.typ->type == Tpointer && r->list->info.typ->ref == (Tnode*)p->info.typ->ref)
                fprintf(fd, "\n{\n\t/* Echo request-response parameter */\n\t%s = *%s;\n\treturn SOAP_OK;\n}\n", ident(p->sym->name), ident(r->list->sym->name));
              /* params to wrapped params echo */
              else
              {
                fprintf(fd, "\n{\n\t(void)soap; /* appease -Wall -Werror */");
                if (r && p && p->info.typ->ref && ((Tnode*)p->info.typ->ref)->ref && (((Tnode*)p->info.typ->ref)->type == Tstruct || ((Tnode*)p->info.typ->ref)->type == Tclass))
                {
                  const char *s, *a;
                  int d = 1;
                  s = ident(p->sym->name);
                  if (p->info.typ->type == Treference || p->info.typ->type == Trvalueref)
                    a = ".";
                  else
                    a = "->";
                  for (p = ((Table*)((Tnode*)p->info.typ->ref)->ref)->list, q = r->list; p && q; p = p->next, q = q->next)
                  {
                    if (p->info.typ == q->info.typ)
                      fprintf(fd, "\n\t%s%s%s = %s;", s, a, ident(p->sym->name), ident(q->sym->name));
                    else if (q->info.typ->type == Tpointer && p->info.typ == (Tnode*)q->info.typ->ref)
                      fprintf(fd, "\n\t%s%s%s = *%s;", s, a, ident(p->sym->name), ident(q->sym->name));
                    else
                      d = 0;
                  }
                  if (!d)
                    fprintf(fd, "\n\t/* Return response with default data and some values copied from the request */");
                }
                fprintf(fd, "\n\treturn SOAP_OK;\n}\n");
              }
              fflush(fd);
            }
          }
          if (namespaceid)
            fprintf(fd, "\n} // namespace %s\n", namespaceid);
          if (fclose(fd))
            execerror("Cannot write to file");
        }
      }
    }
    if (!wflag && !import)
    {
      sprintf(buf, "%s%s.xsd", dirpath, ns_cname(ns->name, NULL));
      fprintf(fmsg, "Saving %s XML schema\n", buf);
      fd = fopen(buf, "w");
      if (!fd)
        execerror("Cannot write schema file");
      fprintf(fd, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
      gen_schema(fd, t, ns->name, ns->name, 1, style, encoding);
      if (fclose(fd))
        execerror("Cannot write to file");
    }
  }
  if (Tflag)
    fprintf(fmsg, "Warning: cannot save soapTester, need directive //gsoap ns service name:\n");
  if (!has_nsmap)
  {
    for (ns = nslist; ns; ns = ns->next)
      if (strcmp(ns->name, "SOAP-ENV") && strcmp(ns->name, "SOAP-ENC") && strcmp(ns->name, "xsi") && strcmp(ns->name, "xsd"))
        break;
    if (nflag)
      sprintf(buf, "%s%s.nsmap", dirpath, prefix);
    else if (ns)
      sprintf(buf, "%s%s.nsmap", dirpath, ns_cname(ns->name, NULL));
    else
      sprintf(buf, "%ssoap.nsmap", dirpath);
    fprintf(fmsg, "Saving %s namespace mapping table\n", buf);
    fd = fopen(buf, "w");
    if (!fd)
      execerror("Cannot write nsmap file");
    /* fprintf(fd, "\n#include \"%sH.h\"", prefix); better to leave to users to include this, so use stdsoap2.h instead: */
    fprintf(fd, "\n#include \"stdsoap2.h\"\n/* This defines the global XML namespaces[] table to #include and compile */");
    if (nflag)
      fprintf(fd, "\nSOAP_NMAC struct Namespace %s_namespaces[] = ", prefix);
    else
      fprintf(fd, "\nSOAP_NMAC struct Namespace namespaces[] = ");
    gen_nsmap(fd);
    if (fclose(fd))
      execerror("Cannot write to file");
    if (rflag)
    {
      Symbol *ns1;
      fprintf(freport, "## Schemas and Namespaces {#doc-namespaces}\n\nThe following schemas and namespaces are used in addition to the predefined SOAP and built-in XSD namespaces:\n\n");
      for (ns1 = nslist; ns1; ns1 = ns1->next)
      {
        if (strcmp(ns1->name, "SOAP-ENV") && strcmp(ns1->name, "SOAP-ENC") && strcmp(ns1->name, "xsi") && strcmp(ns1->name, "xsd"))
        {
          fprintf(freport, "- Prefix `%s` is bound to namespace URI *`%s/%s.xsd`*", ns_convert(ns1->name), tmpURI, ns_convert(ns1->name));
          if (!wflag)
            fprintf(freport, " of schema [%s.xsd](%s%s.xsd)", ns_cname(ns1->name, NULL), dirpath, ns_cname(ns1->name, NULL));
          else
            fprintf(freport, " with local element and attribute form default *`unqualified`*\n");
        }
      }
      fprintf(freport, "\n\nThe following namespace table is saved to %s:\n\n", buf);
      if (nflag)
        fprintf(freport, "    struct Namespace %s_namespaces[] = ", prefix);
      else
        fprintf(freport, "    struct Namespace namespaces[] = ");
      gen_nsmap(freport);
      fprintf(freport, "\nThe table binds XML namespace prefixes (first column) to namespace URIs (second column), similar to xmlns:prefix=\"URI\" in XML.  The third column is a URI pattern with `*` wildcards that is also accepted as a valid namespace URI for inbound XML messages.  The fourth column is NULL (and can be omitted).  This table is globally defined for the gSOAP engine and the engine context will look for it unless you compiled the gSOAP source codes with `-DWITH_NONAMESPACES`.  Then assign this or another namespace table with `soap_set_namespaces(struct soap *soap, struct Namespace *namespaces)` after initializing the context and before processing XML.\n\n");
      gen_report_hr();
    }
  }
}

int
chkhdr(const char *part)
{
  Entry *p;
  p = entry(classtable, lookup("SOAP_ENV__Header"));
  if (p)
    for (p = ((Table*)p->info.typ->ref)->list; p; p = p->next)
      if (has_ns_eq(NULL, p->sym->name) && (!strcmp(part, p->sym->name) || is_eq_nons(part, p->sym->name)))
        return 1;
  sprintf(errbuf, "Cannot define method-header-part in WSDL: SOAP_ENV__Header \"%s\" member is not qualified", part);
  semwarn(errbuf);
  return 0;
}

void
gen_wsdl(FILE *fd, Table *t, const char *ns, const char *name, const char *URL, const char *executable, const char *URI, const char *style, const char *encoding, const char *protocol)
{
  Entry *p, *q, *r;
  Symbol *s;
  Service *sp, *sp2;
  Method *m;
  const char *mimein = NULL, *mimeout = NULL;
  int prot, mask = 0x00; /* 0x01 = SOAP, 0x02 = GET, 0x04 = PUT, 0x08 = DELETE, 0x10 = POST */
  int hasport = 0;
  const char *action, *method_style = NULL, *method_encoding = NULL, *method_response_encoding = NULL;
  const char *portname;
  const char *binding;
  fprintf(fd, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
  for (sp = services; sp; sp = sp->next)
    if (!tagcmp(sp->ns, ns))
      break;
  if (sp && sp->definitions)
    fprintf(fd, "<definitions name=\"%s\"\n", sp->definitions);
  else
    fprintf(fd, "<definitions name=\"%s\"\n", name);
  if (sp && sp->WSDL)
    fprintf(fd, "  targetNamespace=\"%s\"\n  xmlns:tns=\"%s\"", sp->WSDL, sp->WSDL);
  else
    fprintf(fd, "  targetNamespace=\"%s/%s.wsdl\"\n  xmlns:tns=\"%s/%s.wsdl\"", URI, name, URI, name);
  if (sp && sp->binding)
    binding = ns_cname(sp->binding, NULL);
  else
    binding = name;
  if (sp && sp->portname)
    portname = ns_cname(sp->portname, NULL);
  else
    portname = name;
  for (s = nslist; s; s = s->next)
  {
    for (sp2 = services; sp2; sp2 = sp2->next)
      if (!tagcmp(sp2->ns, s->name) && sp2->URI)
        break;
    if (sp2)
      fprintf(fd, "\n  xmlns:%s=\"%s\"", ns_convert(s->name), sp2->URI);
    else if (!strcmp(s->name, "SOAP-ENV"))
      fprintf(fd, "\n  xmlns:SOAP-ENV=\"%s\"", envURI);
    else if (!strcmp(s->name, "SOAP-ENC"))
      fprintf(fd, "\n  xmlns:SOAP-ENC=\"%s\"", encURI);
    else if (!strcmp(s->name, "xsi"))
      fprintf(fd, "\n  xmlns:xsi=\"%s\"", xsiURI);
    else if (!strcmp(s->name, "xsd"))
      fprintf(fd, "\n  xmlns:xsd=\"%s\"", xsdURI);
    else
      fprintf(fd, "\n  xmlns:%s=\"%s/%s.xsd\"", ns_convert(s->name), tmpURI, ns_convert(s->name));
  }
  if (is_soap12(encoding))
    fprintf(fd, "\n  xmlns:SOAP=\"http://schemas.xmlsoap.org/wsdl/soap12/\"");
  else
    fprintf(fd, "\n  xmlns:SOAP=\"http://schemas.xmlsoap.org/wsdl/soap/\"");
  fprintf(fd, "\n  xmlns:HTTP=\"http://schemas.xmlsoap.org/wsdl/http/\"");
  fprintf(fd, "\n  xmlns:MIME=\"http://schemas.xmlsoap.org/wsdl/mime/\"");
  fprintf(fd, "\n  xmlns:DIME=\"http://schemas.xmlsoap.org/ws/2002/04/dime/wsdl/\"");
  fprintf(fd, "\n  xmlns:WSDL=\"http://schemas.xmlsoap.org/wsdl/\"");
  fprintf(fd, "\n  xmlns=\"http://schemas.xmlsoap.org/wsdl/\">\n\n");
  fprintf(fd, "<types>\n\n");
  for (s = nslist; s; s = s->next)
    gen_schema(fd, t, ns, s->name, !strcmp(s->name, ns), style, encoding);
  fprintf(fd, "</types>\n\n");
  fflush(fd);
  if (t)
  {
    for (p = t->list; p; p = p->next)
    {
      if (p->info.typ->type == Tfun && !(p->info.sto & Sextern) && has_ns_eq(ns, p->sym->name))
      {
        mimein = NULL;
        mimeout = NULL;
        method_style = style;
        method_encoding = encoding;
        method_response_encoding = NULL;
        if (sp)
        {
          for (m = sp->list; m; m = m->next)
          {
            if (is_eq_nons(m->name, p->sym->name))
            {
              if (m->mess&MIMEIN)
                mimein = m->part;
              if (m->mess&MIMEOUT)
                mimeout = m->part;
              if (m->mess == ENCODING)
                method_encoding = m->part;
              else if (m->mess == RESPONSE_ENCODING)
                method_response_encoding = m->part;
              else if (m->mess == STYLE)
                method_style = m->part;
            }
          }
        }
        if (!method_response_encoding)
          method_response_encoding = method_encoding;
        if (get_response(p->info.typ))
          fprintf(fd, "<message name=\"%sRequest\">\n", ns_remove(p->sym->name));
        else
          fprintf(fd, "<message name=\"%s\">\n", ns_remove(p->sym->name));
        fflush(fd);
        if (is_document(method_style))
        {
          if (is_invisible(p->sym->name))
          {
            q = entry(classtable, p->sym);
            if (q)
            {
              q = ((Table*)q->info.typ->ref)->list;
              if (q)
              {
                if (is_invisible(q->sym->name))
                {
                  r = entry(classtable, q->sym);
                  if (r)
                  {
                    r = ((Table*)r->info.typ->ref)->list;
                    if (r)
                    {
                      fprintf(fd, "  <part name=\"Body\" element=\"%s\"", ns_add(r, ns));
                      if (gen_member_documentation(fd, p->sym, r, ns, 1))
                        fprintf(fd,  " </part>\n");
                    }
                  }
                }
                else
                {
                  fprintf(fd, "  <part name=\"Body\" element=\"%s\"", ns_add(q, ns));
                  if (gen_member_documentation(fd, p->sym, q, ns, 1))
                    fprintf(fd, "  </part>\n");
                }
              }
            }
          }
          else
          {
            fprintf(fd, "  <part name=\"Body\" element=\"%s\"", ns_add(p, ns));
            if (gen_member_documentation(fd, p->sym, p, ns, 1))
              fprintf(fd, "  </part>\n");
          }
        }
        else
        {
          q = entry(classtable, p->sym);
          if (q)
          {
            for (q = ((Table*)q->info.typ->ref)->list; q; q = q->next)
            {
              if (!is_transient(q->info.typ) && !(q->info.sto & Sattribute) && q->info.typ->type != Tfun && !is_repetition(q) && !is_anytype(q))
              {
                if (is_XML(q->info.typ) || is_stdXML(q->info.typ))
                  fprintf(fd, "  <part name=\"Body\" type=\"xsd:anyType\"/>\n");
                else
                {
                  fprintf(fd, "  <part name=\"%s\" type=\"%s\"", ns_remove(q->sym->name), wsdl_type(q->info.typ, ns));
                  if (gen_member_documentation(fd, p->sym, q, ns, 1))
                    fprintf(fd, "  </part>\n");
                }
              }
            }
          }
        }
        if (mimein)
          fprintf(fd, "  <part name=\"Attachments\" type=\"xsd:base64Binary\"/>\n");
        fprintf(fd, "</message>\n\n");
        fflush(fd);
        q = (Entry*)p->info.typ->ref;
        for (r = t->list; r; r = r->next)
          if (r != p && r->info.typ->type == Tfun && !(r->info.sto & Sextern) && q == (Entry*)r->info.typ->ref)
            q = NULL;
        if (q && is_transient(q->info.typ))
          ;
        else if (q && !is_response(q->info.typ))
        {
          fprintf(fd, "<message name=\"%sResponse\">\n", ns_remove(p->sym->name));
          if (is_document(method_style))
            fprintf(fd, "  <part name=\"Body\" element=\"%sResponse\"/>\n", ns_add(p, ns));
          else if (is_literal(method_response_encoding))
          {
            fprintf(fd, "  <part name=\"%s\" element=\"%s\"", ns_remove(q->sym->name), ns_add(q, ns));
            if (gen_member_documentation(fd, p->sym, q, ns, 1))
              fprintf(fd, "  </part>\n");
          }
          else if (is_XML((Tnode*)q->info.typ->ref) || is_stdXML((Tnode*)q->info.typ->ref))
            fprintf(fd, "  <part name=\"Body\" type=\"xsd:anyType\"/>\n");
          else
          {
            fprintf(fd, "  <part name=\"%s\" type=\"%s\"", ns_remove(q->sym->name), wsdl_type(q->info.typ, ns));
            if (gen_member_documentation(fd, p->sym, q, ns, 1))
              fprintf(fd, "  </part>\n");
          }
          if (mimeout)
            fprintf(fd, "  <part name=\"Attachments\" type=\"xsd:base64Binary\"/>\n");
          fprintf(fd, "</message>\n\n");
        }
        else if (q && q->info.typ->wsdl == False)
        {
          q->info.typ->wsdl = True;
          fprintf(fd, "<message name=\"%s\">\n", ns_remove(((Tnode*)q->info.typ->ref)->id->name));
          if (is_document(method_style))
          {
            if (has_ns_eq(NULL, ((Entry*)p->info.typ->ref)->sym->name))
              fprintf(fd, "  <part name=\"Body\" element=\"%s\"/>\n", ns_convert(((Entry*)p->info.typ->ref)->sym->name));
            else if (is_invisible(((Tnode*)q->info.typ->ref)->id->name))
            {
              r = ((Table*)((Tnode*)q->info.typ->ref)->ref)->list;
              if (r)
              {
                fprintf(fd, "  <part name=\"Body\" element=\"%s\"", ns_add(r, ns));
                if (gen_member_documentation(fd, p->sym, r, ns, 1))
                  fprintf(fd, "  </part>\n");
              }
            }
            else
            {
              fprintf(fd, "  <part name=\"Body\" element=\"%s\"", ns_convert(((Tnode*)q->info.typ->ref)->id->name));
              if (gen_member_documentation(fd, p->sym, q, ns, 1))
                fprintf(fd, "  </part>\n");
            }
          }
          else
          {
            if (((Tnode*)q->info.typ->ref)->ref)
            {
              for (q = ((Table*)((Tnode*)q->info.typ->ref)->ref)->list; q; q = q->next)
              {
                if (!is_transient(q->info.typ) && !(q->info.sto & Sattribute) && q->info.typ->type != Tfun && !is_repetition(q) && !is_anytype(q))
                {
                  if (is_XML(q->info.typ) || is_stdXML(q->info.typ))
                    fprintf(fd, "  <part name=\"Body\" type=\"xsd:anyType\"/>\n");
                  else
                  {
                    fprintf(fd, "  <part name=\"%s\" type=\"%s\"", ns_remove(q->sym->name), wsdl_type(q->info.typ, ns));
                    if (gen_member_documentation(fd, p->sym, q, ns, 1))
                      fprintf(fd, "  </part>\n");
                  }
                }
              }
            }
          }
          if (mimeout)
            fprintf(fd, "  <part name=\"Attachments\" type=\"xsd:base64Binary\"/>\n");
          fprintf(fd, "</message>\n\n");
        }
        fflush(fd);
      }
    }
    if (custom_header)
    {
      Table *r;
      fprintf(fd, "<message name=\"%sHeader\">\n", name);
      r = (Table*)entry(classtable, lookup("SOAP_ENV__Header"))->info.typ->ref;
      if (r)
      {
        for (q = r->list; q; q = q->next)
        {
          if (!is_transient(q->info.typ) && !(q->info.sto & Sattribute) && q->info.typ->type != Tfun && !is_repetition(q) && !is_anytype(q))
            fprintf(fd, "  <part name=\"%s\" element=\"%s\"/>\n", ns_remove(q->sym->name), ns_add(q, ns));
        }
      }
      fprintf(fd, "</message>\n\n");
    }
    if (custom_fault)
    {
      Table *r;
      fprintf(fd, "<message name=\"%sFault\">\n", name);
      r = (Table*)entry(classtable, lookup("SOAP_ENV__Detail"))->info.typ->ref;
      if (r)
        for (q = r->list; q; q = q->next)
          if (!is_transient(q->info.typ) && !is_repetition(q) && !is_anytype(q) && !(q->info.sto & Sattribute) && q->info.typ->type != Tfun && has_ns_eq(NULL, q->sym->name))
          {
            fprintf(fd, "  <part name=\"%s\" element=\"%s\"", ns_remove(q->sym->name), ns_add(q, ns));
            if (gen_member_documentation(fd, q->sym, q, ns, 1))
              fprintf(fd, "  </part>\n");
          }
      fprintf(fd, "</message>\n\n");
    }
    if (sp)
    {
      for (m = sp->list; m; m = m->next)
      {
        if (m->mess&FAULT && m->part)
        {
          Method *m2;
          int flag = 0;
          for (m2 = sp->list; m2 && m2 != m; m2 = m2->next)
            if (m2->mess&FAULT && !strcmp(m2->part, m->part))
              flag = 1;
          if (!flag)
          {
            if (typetable)
              for (p = typetable->list; p; p = p->next)
                if ((m->mess&FAULT) && is_eq(m->part, p->info.typ->sym->name))
                  break;
            if (!p && classtable)
              for (p = classtable->list; p; p = p->next)
                if ((m->mess&FAULT) && is_eq(m->part, p->info.typ->id->name))
                  break;
            if (p)
            {
              fprintf(fd, "<message name=\"%sFault\">\n", ns_remove(m->part));
              fprintf(fd, "  <part name=\"Fault\" element=\"%s\"/>\n", ns_convert(m->part));
              fprintf(fd, "</message>\n\n");
              flag = 0;
              if (custom_fault)
              {
                Table *r;
                r = (Table*)entry(classtable, lookup("SOAP_ENV__Detail"))->info.typ->ref;
                if (r)
                  for (q = r->list; q; q = q->next)
                    if (!is_transient(q->info.typ) && !is_repetition(q) && !is_anytype(q) && !(q->info.sto & Sattribute) && q->info.typ->type != Tfun && (!strcmp(q->sym->name, m->part) || !strcmp(q->sym->name + 1, m->part) || ((q->info.typ->type == Tpointer || is_smart(q->info.typ)) && ((((Tnode*)q->info.typ->ref)->id && !strcmp(((Tnode*)q->info.typ->ref)->id->name, m->part)) || (((Tnode*)q->info.typ->ref)->sym && !strcmp(((Tnode*)q->info.typ->ref)->sym->name, m->part))))))
                    {
                      flag = 1;
                      break;
                    }
              }
              if (!flag)
              {
                sprintf(errbuf, "//gsoap %s method-fault %s %s directive does not refer to a member of struct SOAP_ENV__Detail: suggest to define struct SOAP_ENV__Detail with member %s", sp->ns, m->name, m->part, m->part);
                semwarn(errbuf);
              }
            }
            else
            {
              sprintf(errbuf, "//gsoap %s method-fault %s %s directive does not refer to struct/class or typedef: should globablly define fault %s as type (typedef or struct/class)", sp->ns, m->name, m->part, m->part);
              semwarn(errbuf);
            }
          }
        }
      }
    }
    fflush(fd);
    if (sp && sp->porttype)
      fprintf(fd, "<portType name=\"%s\">\n", sp->porttype);
    else
      fprintf(fd, "<portType name=\"%s\">\n", ns_cname(name, "PortType"));
    if (protocol)
    {
      if (strncmp(protocol, "SOAP", 4))
      {
        if (strstr(protocol, "GET"))
          mask = 0x02;
        else if (strstr(protocol, "PUT"))
          mask = 0x04;
        else if (strstr(protocol, "DELETE"))
          mask = 0x08;
        else /* assume POST */
          mask = 0x10;
      }
      else
        mask = 0x01;
    }
    for (p = t->list; p; p = p->next)
    {
      if (p->info.typ->type == Tfun && !(p->info.sto & Sextern) && has_ns_eq(ns, p->sym->name))
      {
        if (sp)
        {
          for (m = sp->list; m; m = m->next)
          {
            if (m->mess == PROTOCOL)
            {
              if (strncmp(m->part, "SOAP", 4))
              {
                if (strstr(m->part, "GET"))
                  mask |= 0x02;
                else if (strstr(m->part, "PUT"))
                  mask |= 0x04;
                else if (strstr(m->part, "DELETE"))
                  mask |= 0x08;
                else /* assume POST */
                  mask |= 0x10;
              }
              else
                mask |= 0x01;
            }
          }
        }
        if (!mask)
        {
          if (soap_version < 0)
            mask = 0x10; /* -0 option: use POST */
          else
            mask = 0x01;
        }
        fprintf(fd, "  <operation name=\"%s\">\n", ns_remove(p->sym->name));
        gen_method_documentation(fd, p, ns);
        if (get_response(p->info.typ))
          fprintf(fd, "    <input message=\"tns:%sRequest\"/>\n", ns_remove(p->sym->name));
        else
          fprintf(fd, "    <input message=\"tns:%s\"/>\n", ns_remove(p->sym->name));
        q = (Entry*)p->info.typ->ref;
        if (q && is_transient(q->info.typ))
          ;
        else if (q && !is_response(q->info.typ))
          fprintf(fd, "    <output message=\"tns:%sResponse\"/>\n", ns_remove(p->sym->name));
        else if (q)
          fprintf(fd, "    <output message=\"tns:%s\"/>\n", ns_remove(((Tnode*)q->info.typ->ref)->id->name));
        if (sp)
          for (m = sp->list; m; m = m->next)
            if ((m->mess&FAULT) && is_eq_nons(m->name, p->sym->name))
              fprintf(fd, "    <fault name=\"%s\" message=\"tns:%sFault\"/>\n", ns_remove(m->part), ns_remove(m->part));
        fprintf(fd, "  </operation>\n");
      }
    }
    fprintf(fd, "</portType>\n\n");
    for (prot = 0x01; prot <= 0x10; prot <<= 1)
    {
      if ((prot & mask))
      {
        const char *v = "";
        switch (prot)
        {
          case 0x01: v = "";       break;
          case 0x02: v = "GET";    break;
          case 0x04: v = "PUT";    break;
          case 0x08: v = "DELETE"; break;
          case 0x10: v = "POST";   break;
        }
        fprintf(fd, "<binding name=\"%s%s\" ", binding, v);
        if (prot == 0x01)
        {
          if (is_document(style))
          {
            if (sp && sp->porttype)
              fprintf(fd, "type=\"tns:%s\">\n  <SOAP:binding style=\"document\"", sp->porttype);
            else
              fprintf(fd, "type=\"tns:%s\">\n  <SOAP:binding style=\"document\"", ns_cname(name, "PortType"));
          }
          else
          {
            if (sp && sp->porttype)
              fprintf(fd, "type=\"tns:%s\">\n  <SOAP:binding style=\"rpc\"", sp->porttype);
            else
              fprintf(fd, "type=\"tns:%s\">\n  <SOAP:binding style=\"rpc\"", ns_cname(name, "PortType"));
          }
          if (sp && sp->transport)
            fprintf(fd, " transport=\"%s\"/>\n", sp->transport);
          else
            fprintf(fd, " transport=\"http://schemas.xmlsoap.org/soap/http\"/>\n");
        }
        else
        {
          if (sp && sp->porttype)
            fprintf(fd, "type=\"tns:%s\">\n  <HTTP:binding verb=\"%s\"/>\n", sp->porttype, v);
          else
            fprintf(fd, "type=\"tns:%s\">\n  <HTTP:binding verb=\"%s\"/>\n", ns_cname(name, "PortType"), v);
        }
        fflush(fd);
        for (p = t->list; p; p = p->next)
        {
          if (p->info.typ->type == Tfun && !(p->info.sto & Sextern) && has_ns_eq(ns, p->sym->name))
          {
            action = "";
            mimein = NULL;
            mimeout = NULL;
            method_style = style;
            method_encoding = encoding;
            method_response_encoding = NULL;
            if (sp)
            {
              int v = 0x01;
              if (sp->protocol)
              {
                if (strncmp(sp->protocol, "SOAP", 4))
                {
                  if (strstr(sp->protocol, "GET"))
                    v = 0x02;
                  else if (strstr(sp->protocol, "PUT"))
                    v = 0x04;
                  else if (strstr(sp->protocol, "DELETE"))
                    v = 0x08;
                  else /* assume POST */
                    v = 0x10;
                }
              }
              for (m = sp->list; m; m = m->next)
              {
                if (is_eq_nons(m->name, p->sym->name))
                {
                  if (m->mess&MIMEIN)
                    mimein = m->part;
                  if (m->mess&MIMEOUT)
                    mimeout = m->part;
                  if (m->mess == ENCODING)
                    method_encoding = m->part;
                  else if (m->mess == RESPONSE_ENCODING)
                    method_response_encoding = m->part;
                  else if (m->mess == STYLE)
                    method_style = m->part;
                  else if (m->mess == ACTION || m->mess == REQUEST_ACTION)
                    action = m->part;
                  else if (m->mess == RESPONSE_ACTION)
                    action = m->part;
                  else if (m->mess == PROTOCOL)
                  {
                    if (strncmp(m->part, "SOAP", 4))
                    {
                      if (strstr(m->part, "GET"))
                        v = 0x02;
                      else if (strstr(m->part, "PUT"))
                        v = 0x04;
                      else if (strstr(m->part, "DELETE"))
                        v = 0x08;
                      else /* assume POST */
                        v = 0x10;
                    }
                    else
                      v = 0x1;
                  }
                }
              }
              if (soap_version < 0)
                v = 0x10; /* -0 option: POST */
              if (prot != v)
                continue;
            }
            if (!method_response_encoding)
              method_response_encoding = method_encoding;
            fprintf(fd, "  <operation name=\"%s\">\n", ns_remove(p->sym->name));
            if (prot == 0x01)
            {
              if (is_document(style))
              {
                if (is_document(method_style))
                {
                  if (is_soap12(encoding) && !*action)
                    fprintf(fd, "    <SOAP:operation/>\n");
                  else if (*action == '"')
                    fprintf(fd, "    <SOAP:operation soapAction=%s/>\n", action);
                  else
                    fprintf(fd, "    <SOAP:operation soapAction=\"%s\"/>\n", action);
                }
                else if (is_soap12(encoding) && !*action)
                  fprintf(fd, "    <SOAP:operation style=\"rpc\"/>\n");
                else if (*action == '"')
                  fprintf(fd, "    <SOAP:operation style=\"rpc\" soapAction=%s/>\n", action);
                else
                  fprintf(fd, "    <SOAP:operation style=\"rpc\" soapAction=\"%s\"/>\n", action);
              }
              else
              {
                if (is_document(method_style))
                {
                  if (is_soap12(encoding) && !*action)
                    fprintf(fd, "    <SOAP:operation style=\"document\"/>\n");
                  else if (*action == '"')
                    fprintf(fd, "    <SOAP:operation style=\"document\" soapAction=%s/>\n", action);
                  else
                    fprintf(fd, "    <SOAP:operation style=\"document\" soapAction=\"%s\"/>\n", action);
                }
                else if (is_soap12(encoding) && !*action)
                  fprintf(fd, "    <SOAP:operation style=\"rpc\"/>\n");
                else if (*action == '"')
                  fprintf(fd, "    <SOAP:operation style=\"rpc\" soapAction=%s/>\n", action);
                else
                  fprintf(fd, "    <SOAP:operation style=\"rpc\" soapAction=\"%s\"/>\n", action);
              }
            }
            else
            {
              if (!*action)
                fprintf(fd, "    <HTTP:operation/>\n");
              else if (*action == '"')
                fprintf(fd, "    <HTTP:operation location=%s/>\n", action);
              else
                fprintf(fd, "    <HTTP:operation location=\"%s\"/>\n", action);
            }
            fprintf(fd, "    <input>\n");
            q = entry(classtable, p->sym);
            if (prot == 0x01)
            {
              if (mimein)
                fprintf(fd, "      <MIME:multipartRelated>\n        <MIME:part>\n");
              if (is_literal(method_encoding) || (q && (q = (((Table*)q->info.typ->ref)->list)) && q && is_XML(q->info.typ)))
              {
                if (is_document(method_style))
                  fprintf(fd, "          <SOAP:body use=\"literal\" parts=\"Body\"/>\n");
                else
                  fprintf(fd, "          <SOAP:body use=\"literal\" parts=\"Body\" namespace=\"%s\"/>\n", URI);
              }
              else
              {
                if (encoding && *encoding)
                  fprintf(fd, "          <SOAP:body use=\"encoded\" encodingStyle=\"%s\" namespace=\"%s\"/>\n", encoding, URI);
                else if (method_encoding && *method_encoding)
                  fprintf(fd, "          <SOAP:body use=\"encoded\" encodingStyle=\"%s\" namespace=\"%s\"/>\n", method_encoding, URI);
                else
                  fprintf(fd, "          <SOAP:body use=\"encoded\" encodingStyle=\"%s\" namespace=\"%s\"/>\n", encURI, URI);
                if (!eflag)
                {
                  sprintf(errbuf, "operation '%s' is specific to SOAP encoding only and not compliant with WS-I Basic Profile 1.0a", p->sym->name);
                  compliancewarn(errbuf);
                }
              }
              if (custom_header)
              {
                m = NULL;
                if (sp)
                {
                  for (m = sp->list; m; m = m->next)
                    if (is_eq_nons(m->name, p->sym->name) && (m->mess&HDRIN))
                    {
                      if (chkhdr(m->part))
                        fprintf(fd, "          <SOAP:header use=\"literal\" message=\"tns:%sHeader\" part=\"%s\"/>\n", name, ns_remove(m->part));
                    }
                }
              }
              if (mimein)
              {
                if (sp)
                {
                  for (m = sp->list; m; m = m->next)
                  {
                    if (is_eq_nons(m->name, p->sym->name) && (m->mess&MIMEIN))
                      fprintf(fd, "        </MIME:part>\n        <MIME:part>\n          <MIME:content part=\"Attachments\" type=\"%s\"/>\n", m->part);
                  }
                }
                fprintf(fd, "        </MIME:part>\n      </MIME:multipartRelated>\n");
              }
            }
            else if (prot == 0x02)
              fprintf(fd, "      <HTTP:urlEncoded/>\n");
            else
            {
              if (mimein)
                fprintf(fd, "      <MIME:content type=\"%s\"/>\n", mimein);
              else if (!q || is_document(method_style))
                fprintf(fd, "      <MIME:mimeXml part=\"Body\"/>\n");
              else
                fprintf(fd, "      <MIME:mimeXml part=\"%s\"/>\n", ns_remove(q->sym->name));
            }
            fprintf(fd, "    </input>\n");
            q = (Entry*)p->info.typ->ref;
            if (!q || !q->info.typ->ref)
            {
              fprintf(fd, "  </operation>\n");
              continue;
            }
            if (prot != 0x04)
            {
              fprintf(fd, "    <output>\n");
              if (prot == 0x01)
              {
                if (mimeout)
                  fprintf(fd, "      <MIME:multipartRelated>\n        <MIME:part>\n");
                if (is_literal(method_response_encoding) || is_XML((Tnode*)q->info.typ->ref))
                {
                  if (is_document(method_style))
                    fprintf(fd, "          <SOAP:body use=\"literal\" parts=\"Body\"/>\n");
                  else
                    fprintf(fd, "          <SOAP:body use=\"literal\" parts=\"Body\" namespace=\"%s\"/>\n", URI);
                }
                else if (encoding && *encoding)
                  fprintf(fd, "          <SOAP:body use=\"encoded\" encodingStyle=\"%s\" namespace=\"%s\"/>\n", encoding, URI);
                else if (method_response_encoding && *method_response_encoding)
                  fprintf(fd, "          <SOAP:body use=\"encoded\" encodingStyle=\"%s\" namespace=\"%s\"/>\n", method_response_encoding, URI);
                else
                  fprintf(fd, "          <SOAP:body use=\"encoded\" encodingStyle=\"%s\" namespace=\"%s\"/>\n", encURI, URI);
                if (custom_header)
                {
                  if (sp)
                    for (m = sp->list; m; m = m->next)
                      if (is_eq_nons(m->name, p->sym->name) && (m->mess&HDROUT))
                      {
                        if (chkhdr(m->part))
                          fprintf(fd, "          <SOAP:header use=\"literal\" message=\"tns:%sHeader\" part=\"%s\"/>\n", name, ns_remove(m->part));
                      }
                }
                if (mimeout)
                {
                  if (sp)
                  {
                    for (m = sp->list; m; m = m->next)
                    {
                      if (is_eq_nons(m->name, p->sym->name) && (m->mess&MIMEOUT))
                        fprintf(fd, "        </MIME:part>\n        <MIME:part>\n          <MIME:content part=\"Attachments\" type=\"%s\"/>\n", m->part);
                    }
                  }
                  fprintf(fd, "        </MIME:part>\n      </MIME:multipartRelated>\n");
                }
              }
              else
              {
                q = (Entry*)p->info.typ->ref;
                if (mimeout)
                  fprintf(fd, "      <MIME:content type=\"%s\"/>\n", mimeout);
                else if (is_document(method_style))
                  fprintf(fd, "      <MIME:mimeXml part=\"Body\"/>\n");
                else if (q && !is_transient(q->info.typ) && !is_response(q->info.typ) && is_literal(method_response_encoding))
                  fprintf(fd, "      <MIME:mimeXml part=\"%s\"/>\n", ns_remove(q->sym->name));
                else
                  fprintf(fd, "      <MIME:mimeXml part=\"Body\"/>\n");
              }
              fprintf(fd, "    </output>\n");
            }
            if (sp)
              for (m = sp->list; m; m = m->next)
                if ((m->mess&FAULT) && is_eq_nons(m->name, p->sym->name))
                  fprintf(fd, "    <fault name=\"%s\">\n      <SOAP:fault use=\"literal\" name=\"%s\"/>\n    </fault>\n", ns_remove(m->part), ns_remove(m->part));
            fprintf(fd, "  </operation>\n");
            fflush(fd);
          }
        }
        fprintf(fd, "</binding>\n\n");
      }
    }
  }
  fprintf(fd, "<service name=\"%s\">\n", name);
  if (sp && sp->documentation)
  {
    fprintf(fd, "  <documentation>\n    ");
    gen_text(fd, sp->documentation);
    fprintf(fd, "\n  </documentation>\n");
  }
  else
  {
    fprintf(fd, "  <documentation>gSOAP " VERSION " generated service definition</documentation>\n");
  }
  for (prot = 0x01; prot <= 0x10; prot <<= 1)
  {
    if ((prot & mask))
    {
      const char *s, *t, *v = "", *b = "", *e = NULL;
      switch (prot)
      {
        case 0x01: v = "";       b = "SOAP"; break;
        case 0x02: v = "GET";    b = "HTTP"; break;
        case 0x04: v = "PUT";    b = "HTTP"; break;
        case 0x08: v = "DELETE"; b = "HTTP"; break;
        case 0x10: v = "POST";   b = "HTTP"; break;
      }
      fprintf(fd, "  <port name=\"%s%s\" binding=\"tns:%s%s\">\n", portname, v, binding, v);
      if (executable)
        e = executable;
      for (s = URL; s; s = t)
      {
        int n;
        t = strchr(s, ' ');
        if (t)
        {
          n = (int)(t - s);
          t++;
        }
        else
          n = (int)strlen(s);
        if (e)
          fprintf(fd, "    <%s:address location=\"%.*s/%s\"/>\n", b, n, s, e);
        else
          fprintf(fd, "    <%s:address location=\"%.*s\"/>\n", b, n, s);
      }
      fprintf(fd, "  </port>\n");
      hasport = 1;
    }
  }
  if (!hasport && executable)
  {
    const char *s, *t;
    fprintf(fd, "  <port name=\"%s\" binding=\"tns:%s\">\n", portname, binding);
    for (s = URL; s; s = t)
    {
      int n;
      t = strchr(s, ' ');
      if (t)
      {
        n = (int)(t - s);
        t++;
      }
      else
        n = (int)strlen(s);
      fprintf(fd, "    <SOAP:address location=\"%.*s/%s\"/>\n", n, s, executable);
    }
    fprintf(fd, "  </port>\n");
  }
  fprintf(fd, "</service>\n\n</definitions>\n");
}

const char *
default_value(Entry *e)
{
  Tnode *p = e->info.typ;
  Entry *q;
  static char buf[4096];
  buf[0] = '\0';
  if (e->info.ptrval)
    p = p->ref;
  if (e->info.hasval || e->info.ptrval)
  {
    switch (p->type)
    {
      case Tchar:
      case Twchar:
      case Tuchar:
      case Tshort:
      case Tushort:
      case Tint:
      case Tuint:
      case Tlong:
      case Tllong:
      case Tulong:
      case Tullong:
      case Tsize:
        sprintf(buf, SOAP_LONG_FORMAT, e->info.val.i);
        break;
      case Tfloat:
      case Tdouble:
      case Tldouble:
        sprintf(buf, "%.17lG", e->info.val.r);
        break;
      case Ttime:
        break; /* should get value? */
      case Tenum:
      case Tenumsc:
        if (p->ref)
        {
          for (q = ((Table*)p->ref)->list; q; q = q->next)
          {
            if (q->info.val.i == e->info.val.i)
            {
              sprintf(buf, "%s", ns_remove2(q->sym->name, c_ident(p)));
              break;
            }
          }
        }
        break;
      default:
        if (e->info.val.s && strlen(e->info.val.s) < sizeof(buf)-12)
          sprintf(buf, "%s", xstring(e->info.val.s));
        break;
    }
  }
  else
  {
    switch (p->type)
    {
      case Tchar:
      case Twchar:
      case Tuchar:
      case Tshort:
      case Tushort:
      case Tint:
      case Tuint:
      case Tlong:
      case Tllong:
      case Tulong:
      case Tullong:
      case Tsize:
        if (p->hasmin && p->imin > 0)
          sprintf(buf, SOAP_LONG_FORMAT, p->imin + (p->incmin == False));
        else if (p->hasmax && p->imax < 0)
          sprintf(buf, SOAP_LONG_FORMAT, p->imax - (p->incmax == False));
        else
          strcpy(buf, "0");
        break;
      case Tfloat:
      case Tdouble:
      case Tldouble:
        if (p->hasmin && p->rmin > 0)
          sprintf(buf, "%.17lG", p->rmin * (1 + (p->incmin == False)/1000));
        else if (p->hasmax && p->rmax > 0)
          sprintf(buf, "%.17lG", p->rmax * (1 - (p->incmax == False)/1000));
        else if (p->hasmin && p->rmin < 0)
          sprintf(buf, "%.17lG", p->rmin * (1 - (p->incmin == False)/1000));
        else if (p->hasmax && p->rmax < 0)
          sprintf(buf, "%.17lG", p->rmax * (1 + (p->incmax == False)/1000));
        else
          strcpy(buf, "0");
        break;
      case Ttime:
        break; /* should get value? */
      case Tenum:
      case Tenumsc:
        if (p->ref)
          if ((q = ((Table*)p->ref)->list))
            sprintf(buf, "%s", ns_remove2(q->sym->name, c_ident(p)));
        break;
      default:
        break;
    }
  }
  return buf;
}

const char *
set_default_value(Entry *e)
{
  const char *a;
  Entry *q;
  static char buf[4096];
  buf[0] = '\0';
  if (e->info.fixed)
    a = "fixed";
  else
    a = "default";
  if (e->info.hasval || e->info.ptrval)
  {
    Tnode *p = e->info.typ;
    if (e->info.ptrval)
      p = p->ref;
    switch (p->type)
    {
      case Tchar:
      case Twchar:
      case Tuchar:
      case Tshort:
      case Tushort:
      case Tint:
      case Tuint:
      case Tlong:
      case Tllong:
      case Tulong:
      case Tullong:
      case Tsize:
        sprintf(buf, " %s=\"" SOAP_LONG_FORMAT "\"", a, e->info.val.i);
        break;
      case Tfloat:
      case Tdouble:
      case Tldouble:
        sprintf(buf, " %s=\"%.17lG\"", a, e->info.val.r);
        break;
      case Ttime:
        break; /* should get value? */
      case Tenum:
      case Tenumsc:
        if (p->ref)
        {
          for (q = ((Table*)p->ref)->list; q; q = q->next)
          {
            if (q->info.val.i == e->info.val.i)
            {
              sprintf(buf, " %s=\"%s\"", a, ns_remove2(q->sym->name, c_ident(p)));
              break;
            }
          }
        }
        break;
      default:
        if (e->info.val.s && strlen(e->info.val.s) < sizeof(buf)-12)
          sprintf(buf, " %s=\"%s\"", a, xstring(e->info.val.s));
        break;
    }
  }
  return buf;
}

const char *
nillable(Entry *e)
{
  if (e->info.nillable && (e->info.typ->type == Tpointer || is_smart(e->info.typ)))
    return " nillable=\"true\"";
  return "";
}

const char *
nillable_ref(Entry *e)
{
  if (e->info.nillable && (((Tnode*)e->info.typ->ref)->type == Tpointer || is_smart(e->info.typ->ref)))
    return " nillable=\"true\"";
  return "";
}

void
gen_schema(FILE *fd, Table *t, const char *ns1, const char *ns, int all, const char *style, const char *encoding)
{
  Entry *p, *q, *r;
  Tnode *n;
  Symbol *s;
  Service *sp, *sp2;
  Method *m;
  int flag;
  if (!strcmp(ns, "SOAP-ENV") || !strcmp(ns, "SOAP-ENC") || !strcmp(ns, "xsi") || !strcmp(ns, "xsd"))
    return;
  for (sp = services; sp; sp = sp->next)
    if (!tagcmp(sp->ns, ns) && sp->URI)
      break;
  if (sp && sp->import)
    return;
  fprintf(fd, "  <schema ");
  if (sp)
    fprintf(fd, "targetNamespace=\"%s\"", sp->URI);
  else
    fprintf(fd, "targetNamespace=\"%s/%s.xsd\"", tmpURI, ns_convert(ns));
  for (s = nslist; s; s = s->next)
  {
    for (sp2 = services; sp2; sp2 = sp2->next)
      if (!tagcmp(sp2->ns, s->name) && sp2->URI)
        break;
    if (sp2)
      fprintf(fd, "\n    xmlns:%s=\"%s\"", ns_convert(s->name), sp2->URI);
    else if (!strcmp(s->name, "SOAP-ENV"))
    {
      if (soap_version >= 0)
        fprintf(fd, "\n    xmlns:SOAP-ENV=\"%s\"", envURI);
    }
    else if (!strcmp(s->name, "SOAP-ENC"))
    {
      if (soap_version >= 0)
        fprintf(fd, "\n    xmlns:SOAP-ENC=\"%s\"", encURI);
    }
    else if (!strcmp(s->name, "xsi"))
      fprintf(fd, "\n    xmlns:xsi=\"%s\"", xsiURI);
    else if (!strcmp(s->name, "xsd"))
      fprintf(fd, "\n    xmlns:xsd=\"%s\"", xsdURI);
    else
      fprintf(fd, "\n    xmlns:%s=\"%s/%s.xsd\"", ns_convert(s->name), tmpURI, ns_convert(s->name));
  }
  fprintf(fd, "\n    xmlns=\"%s\"\n", xsdURI);
  if (sp && (sp->elementForm || sp->attributeForm))
    fprintf(fd, "    elementFormDefault=\"%s\"\n    attributeFormDefault=\"%s\">\n", sp->elementForm?sp->elementForm:"unqualified", sp->attributeForm?sp->attributeForm:"unqualified");
  else
    fprintf(fd, "    elementFormDefault=\"unqualified\"\n    attributeFormDefault=\"unqualified\">\n");
  fflush(fd);
  flag = 0;
  for (s = nslist; s; s = s->next)
  {
    for (sp2 = services; sp2; sp2 = sp2->next)
      if (sp2 != sp && !tagcmp(sp2->ns, s->name) && sp2->URI)
        break;
    if (sp2)
    {
      fprintf(fd, "    <import namespace=\"%s\"", sp2->URI);
      if (sp2->import)
        fprintf(fd, " schemaLocation=\"%s\"", sp2->import);
      fprintf(fd, "/>\n");
      if (!strcmp(sp2->URI, encURI))
        flag = 1;
    }
  }
  if (!flag)
    fprintf(fd, "    <import namespace=\"%s\"/>", encURI);
  fprintf(fd, "\n");
  fflush(fd);
  if (typetable)
  {
    for (p = typetable->list; p; p = p->next)
    {
      if (is_transient(p->info.typ) || is_invisible(p->sym->name) || p->info.typ->type == Ttemplate)
        continue;
      if (is_element(p->info.typ) && (p->info.typ->type == Tclass || p->info.typ->type == Tstruct) && !is_stdstr(p->info.typ))
        continue;
      if ((!is_external(p->info.typ) || is_volatile(p->info.typ)) && ((has_ns_eq(ns, p->sym->name))))
      {
        /* typedefs that are used for SOAP Fault details */
        m = NULL;
        if (p->info.typ->type != Tstruct && p->info.typ->type != Tclass)
        {
          for (sp2 = services; sp2 && !m; sp2 = sp2->next)
          {
            for (m = sp2->list; m; m = m->next)
            {
              if ((m->mess&FAULT) && m->part && is_eq(m->part, p->sym->name))
                break;
            }
          }
        }
        if (m)
        {
          if (!uflag)
            fprintf(fd, "    <!-- fault element -->\n");
          fprintf(fd, "    <element name=\"%s\" type=\"%s\">\n", ns_tag_remove(p), base_type(p->info.typ, ns1));
          gen_type_documentation(fd, p, ns);
          fprintf(fd, "    </element>\n");
          continue;
        }
        if (is_primitive_or_string(p->info.typ) || is_binary(p->info.typ) || (p->info.typ->type == Tpointer && is_primitive_or_string((Tnode*)p->info.typ->ref)))
        {
          fprintf(fd, "    <simpleType name=\"%s\">", ns_remove(p->sym->name));
          gen_type_documentation(fd, p, ns);
          if (!is_synonym(p->info.typ) && p->info.typ->pattern && p->info.typ->pattern[0] == '%' && p->info.typ->pattern[1])
          {
            unsigned int n = (unsigned int)strtoul(p->info.typ->pattern + 1, NULL, 10);
            unsigned int f = 0;
            const char *s = strchr(p->info.typ->pattern, '.');
            if (s)
              f = (unsigned int)strtoul(s + 1, NULL, 10);
            if (s && f && (p->info.typ->type == Tfloat || p->info.typ->type == Tdouble || p->info.typ->type == Tldouble))
            {
              fprintf(fd, "      <restriction base=\"%s\">\n", base_type(p->info.typ, ns1));
              if (n)
                fprintf(fd, "        <totalDigits value=\"%u\"/>\n", n);
              fprintf(fd, "        <fractionDigits value=\"%u\"/>\n", f);
            }
            else if (n && (p->info.typ->type == Tfloat || p->info.typ->type == Tdouble || p->info.typ->type == Tldouble))
            {
              fprintf(fd, "      <restriction base=\"%s\">\n", base_type(p->info.typ, ns1));
              fprintf(fd, "        <totalDigits value=\"%u\"/>\n", n);
            }
            else if (n && p->info.typ->type >= Tchar && p->info.typ->type <= Tullong)
            {
              fprintf(fd, "      <restriction base=\"%s\">\n", base_type(p->info.typ, ns1));
              fprintf(fd, "        <totalDigits value=\"%u\"/>\n", n);
            }
            else
            {
              fprintf(fd, "      <restriction base=\"%s\">\n", base_type(p->info.typ, ns1));
            }
          }
          else
          {
            fprintf(fd, "      <restriction base=\"%s\">\n", base_type(p->info.typ, ns1));
            if (!is_synonym(p->info.typ) && p->info.typ->pattern)
              fprintf(fd, "        <pattern value=\"%s\"/>\n", p->info.typ->pattern);
          }
          if (is_primitive(p->info.typ) || (p->info.typ->type == Tpointer && is_primitive((Tnode*)p->info.typ->ref) && !is_string(p->info.typ) && !is_wstring(p->info.typ)))
          {
            if (!is_synonym(p->info.typ) && p->info.typ->hasmin)
            {
              if (p->info.typ->type >= Tfloat && p->info.typ->type <= Tldouble)
              {
                if (p->info.typ->incmin)
                  fprintf(fd, "        <minInclusive value=\"%.16lG\"/>\n", p->info.typ->rmin);
                else
                  fprintf(fd, "        <minExclusive value=\"%.16lG\"/>\n", p->info.typ->rmin);
              }
              else
              {
                if (p->info.typ->incmin)
                  fprintf(fd, "        <minInclusive value=\"" SOAP_LONG_FORMAT "\"/>\n", p->info.typ->imin);
                else
                  fprintf(fd, "        <minExclusive value=\"" SOAP_LONG_FORMAT "\"/>\n", p->info.typ->imin);
              }
            }
            if (!is_synonym(p->info.typ) && p->info.typ->hasmax)
            {
              if (p->info.typ->type >= Tfloat && p->info.typ->type <= Tldouble)
              {
                if (p->info.typ->incmax)
                  fprintf(fd, "        <maxInclusive value=\"%.16lG\"/>\n", p->info.typ->rmax);
                else
                  fprintf(fd, "        <maxExclusive value=\"%.16lG\"/>\n", p->info.typ->rmax);
              }
              else
              {
                if (p->info.typ->incmax)
                  fprintf(fd, "        <maxInclusive value=\"" SOAP_LONG_FORMAT "\"/>\n", p->info.typ->imax);
                else
                  fprintf(fd, "        <maxExclusive value=\"" SOAP_LONG_FORMAT "\"/>\n", p->info.typ->imax);
              }
            }
          }
          else
          {
            if (!is_synonym(p->info.typ) && p->info.typ->hasmax && p->info.typ->imax >= 0 && p->info.typ->incmin && p->info.typ->incmax && p->info.typ->imin == p->info.typ->imax)
              fprintf(fd, "        <length value=\"" SOAP_LONG_FORMAT "\"/>\n", p->info.typ->imax);
            else
            {
              if (!is_synonym(p->info.typ) && p->info.typ->hasmin && p->info.typ->imin >= 0)
              {
                if (p->info.typ->incmin)
                  fprintf(fd, "        <minLength value=\"" SOAP_LONG_FORMAT "\"/>\n", p->info.typ->imin);
                else
                  fprintf(fd, "        <minLength value=\"" SOAP_LONG_FORMAT "\"/>\n", p->info.typ->imin + 1);
              }
              if (!is_synonym(p->info.typ) && p->info.typ->hasmax && p->info.typ->imax >= 0)
              {
                if (p->info.typ->incmax)
                  fprintf(fd, "        <maxLength value=\"" SOAP_LONG_FORMAT "\"/>\n", (ULONG64)p->info.typ->imax);
                else
                  fprintf(fd, "        <maxLength value=\"" SOAP_LONG_FORMAT "\"/>\n", (ULONG64)p->info.typ->imax - 1);
              }
            }
          }
          fprintf(fd, "      </restriction>\n    </simpleType>\n");
        }
        else if (is_fixedstring(p->info.typ))
        {
          fprintf(fd, "    <simpleType name=\"%s\">", ns_remove(p->sym->name));
          gen_type_documentation(fd, p, ns);
          fprintf(fd, "      <restriction base=\"%s\">\n", base_type(p->info.typ, ns1));
          if (!is_synonym(p->info.typ) && p->info.typ->hasmin && p->info.typ->imin >= 0)
          {
            if (p->info.typ->incmin)
              fprintf(fd, "        <minLength value=\"" SOAP_LONG_FORMAT "\"/>\n", p->info.typ->imin);
            else
              fprintf(fd, "        <minLength value=\"" SOAP_LONG_FORMAT "\"/>\n", p->info.typ->imin + 1);
          }
          if (!is_synonym(p->info.typ) && p->info.typ->hasmax && p->info.typ->imax >= 0)
          {
            if (p->info.typ->incmax)
              fprintf(fd, "        <maxLength value=\"" SOAP_LONG_FORMAT "\"/>\n", p->info.typ->imax);
            else
              fprintf(fd, "        <maxLength value=\"" SOAP_LONG_FORMAT "\"/>\n", p->info.typ->imax - 1);
          }
          else
            fprintf(fd, "        <maxLength value=\"%d\"/>\n", get_dimension(p->info.typ) - 1);
          fprintf(fd, "      </restriction>\n    </simpleType>\n");
        }
        else if (is_restriction(p->info.typ) && !has_ns_eq("xsd", p->info.typ->restriction->name))
        {
          fprintf(fd, "    <complexType name=\"%s\">", ns_remove(p->sym->name));
          gen_type_documentation(fd, p, ns);
          fprintf(fd, "      <complexContent>\n        <restriction base=\"%s\">\n", base_type(p->info.typ, ns1));
          fprintf(fd, "        </restriction>\n      </complexContent>\n    </complexType>\n");
        }
        else 
        {
          fprintf(fd, "    <simpleType name=\"%s\">", ns_remove(p->sym->name));
          gen_type_documentation(fd, p, ns);
          fprintf(fd, "      <restriction base=\"%s\">\n", base_type(p->info.typ, ns1));
          fprintf(fd, "      </restriction>\n    </simpleType>\n");
        }
      }
    }
  }
  fflush(fd);
  if (enumtable)
    for (p = enumtable->list; p; p = p->next)
      gen_schema_type(fd, t, p, ns1, ns, all, style, encoding);
  fflush(fd);
  if (classtable)
  {
    for (p = classtable->list; p; p = p->next)
    {
      if (is_transient(p->info.typ) || is_invisible(p->sym->name))
        continue;
      for (q = t->list; q; q = q->next)
        if (q->info.typ->type == Tfun && !(q->info.sto & Sextern) && p == get_response(q->info.typ))
          break;
      /* omit the auto-generated and user-defined response struct/class (when necessary) */
      if (!q)
      {
        for (q = t->list; q; q = q->next)
        {
          if (q->info.typ->type == Tfun && !(q->info.sto & Sextern) && !has_ns_eq(NULL, ((Entry*)q->info.typ->ref)->sym->name))
          {
            r = entry(t, q->sym);
            if (r && r->info.typ->ref && is_response(((Entry*)r->info.typ->ref)->info.typ) && p->info.typ == (Tnode*)((Entry*)r->info.typ->ref)->info.typ->ref)
              break;
          }
        }
      }
      if (q)
        continue;
      /* classes that are used for SOAP Fault details */
      m = NULL;
      for (sp2 = services; sp2 && !m; sp2 = sp2->next)
        for (m = sp2->list; m; m = m->next)
          if ((m->mess&FAULT) && m->part && is_eq(m->part, p->sym->name))
            break;
      if (m)
      {
        if ((!has_ns(p->info.typ) && all) || has_ns_eq(ns, p->sym->name))
        {
          if (!uflag)
            fprintf(fd, "    <!-- fault element and type -->\n");
          fprintf(fd, "    <element name=\"%s\" type=\"%s\">\n", ns_tag_remove(p), base_type(p->info.typ, ns1));
          gen_type_documentation(fd, p, ns);
          fprintf(fd, "    </element>\n");
        }
      }
      gen_schema_type(fd, t, p, ns1, ns, all, style, encoding);
    }
  }
  fflush(fd);
  for (n = Tptr[Tarray]; n; n = n->next)
  {
    if (is_transient(n) || is_fixedstring(n))
      continue;
    if (is_literal(encoding))
      fprintf(fd, "    <complexType name=\"%s\">\n        <sequence>\n          <element name=\"item\" type=\"%s\" minOccurs=\"0\" maxOccurs=\"%d\"/>\n        </sequence>\n    </complexType>\n", c_ident(n), wsdl_type((Tnode*)n->ref, ns1), get_dimension(n));
    else
      fprintf(fd, "    <complexType name=\"%s\">\n      <complexContent>\n        <restriction base=\"SOAP-ENC:Array\">\n          <sequence>\n            <element name=\"item\" type=\"%s\" minOccurs=\"0\" maxOccurs=\"%d\"/>\n          </sequence>\n        </restriction>\n      </complexContent>\n    </complexType>\n", c_ident(n), wsdl_type((Tnode*)n->ref, ns1), get_dimension(n));
    fflush(fd);
  }
  gen_schema_elements_attributes(fd, t, ns, ns1, style, encoding);
  fprintf(fd, "  </schema>\n\n");
}

void
gen_schema_type(FILE *fd, Table *t, Entry *p, const char *ns1, const char *ns, int all, const char *style, const char *encoding)
{
  int i, d;
  char cbuf[4];
  Entry *q;
  Tnode *typ = p->info.typ;
  if (!typ->ref)
    return;
  if (typ->type == Tenum || typ->type == Tenumsc)
  {
    int flag;
    if (!is_transient(p->info.typ) && !is_invisible(p->sym->name) && ((!has_ns(p->info.typ) && all) || has_ns_eq(ns, p->sym->name)))
    {
      flag = 0;
      if ((Table*)p->info.typ->ref)
      {
        for (q = ((Table*)p->info.typ->ref)->list; q; q = q->next)
        {
          if (!has_ns_eq(NULL, ns_remove2(q->sym->name, c_ident(p->info.typ))))
          {
            flag = 1;
            break;
          }
        }
        if (flag == 1)
        {
          for (q = ((Table*)p->info.typ->ref)->list; q; q = q->next)
          {
            const char *s = ns_remove2(q->sym->name, c_ident(p->info.typ));
            while (*s && isdigit(*s))
              s++;
            if (*s)
            {
              flag = 2;
              break;
            }
          }
        }
      }
      fprintf(fd, "    <simpleType name=\"%s\">", wsdl_type(p->info.typ, NULL));
      gen_type_documentation(fd, p, ns);
      if (is_mask(p->info.typ))
      {
        fprintf(fd, "       <list>\n");
        if (flag == 0)
          fprintf(fd, "        <restriction base=\"xsd:QName\">\n");
        else if (flag == 1)
          fprintf(fd, "        <restriction base=\"xsd:long\">\n");
        else
          fprintf(fd, "        <restriction base=\"xsd:string\">\n");
        if ((Table*)p->info.typ->ref)
        {
          for (q = ((Table*)p->info.typ->ref)->list; q; q = q->next)
          {
            if (!uflag)
              fprintf(fd, "          <!-- = " SOAP_LONG_FORMAT " -->\n", q->info.val.i);
            fprintf(fd, "          <enumeration value=\"%s\"", ns_remove2(q->sym->name, c_ident(p->info.typ)));
            if (gen_member_documentation(fd, p->sym, q, ns, q->info.typ->type == Tenumsc))
              fprintf(fd, "          </enumeration>\n");
          }
        }
        fprintf(fd, "        </restriction>\n      </list>\n    </simpleType>\n");
      }
      else
      {
        if (flag == 0)
          fprintf(fd, "      <restriction base=\"xsd:QName\">\n");
        else if (flag == 1)
          fprintf(fd, "      <restriction base=\"xsd:long\">\n");
        else
          fprintf(fd, "      <restriction base=\"xsd:string\">\n");
        if ((Table*)p->info.typ->ref)
        {
          for (q = ((Table*)p->info.typ->ref)->list; q; q = q->next)
          {
            if (!uflag)
              fprintf(fd, "        <!-- = " SOAP_LONG_FORMAT " -->\n", q->info.val.i);
            fprintf(fd, "        <enumeration value=\"%s\"", ns_remove2(q->sym->name, c_ident(p->info.typ)));
            if (gen_member_documentation(fd, p->sym, q, ns, q->info.typ->type == Tenumsc))
              fprintf(fd, "        </enumeration>\n");
          }
        }
        fprintf(fd, "      </restriction>\n    </simpleType>\n");
      }
    }
  }
  else if (is_binary(typ))
  {
    if ((!has_ns(typ) && all) || has_ns_eq(ns, p->sym->name))
    {
      if (is_attachment(typ))
      {
        fprintf(fd, "    <complexType name=\"%s\">", ns_remove(p->sym->name));
        gen_type_documentation(fd, p, ns);
        fprintf(fd, "      <simpleContent>\n        <extension base=\"xsd:base64Binary\">\n");
        fprintf(fd, "          <attribute name=\"href\" type=\"xsd:anyURI\" use=\"optional\"/>");
        if (!uflag)
          fprintf(fd, "<!-- attachment reference -->");
        fprintf(fd, "\n");
        gen_schema_attributes(fd, typ, NULL, ns, ns1);
        fprintf(fd, "        </extension>\n      </simpleContent>\n    </complexType>\n");
      }
      else
      {
        fprintf(fd, "    <simpleType name=\"%s\">", ns_remove(p->sym->name));
        gen_type_documentation(fd, p, ns);
        if (is_hexBinary(typ))
          fprintf(fd, "      <restriction base=\"xsd:hexBinary\">\n");
        else
          fprintf(fd, "      <restriction base=\"xsd:base64Binary\">\n");
        if (typ->hasmax && typ->imax >= 0 && typ->incmin && typ->incmax && typ->imin == typ->imax)
          fprintf(fd, "        <length value=\"" SOAP_LONG_FORMAT "\"/>\n", typ->imin);
        else
        {
          if (typ->hasmin && typ->imin >= 0)
          {
            if (typ->incmin)
              fprintf(fd, "        <minLength value=\"" SOAP_LONG_FORMAT "\"/>\n", typ->imin);
            else
              fprintf(fd, "        <minLength value=\"" SOAP_LONG_FORMAT "\"/>\n", typ->imin + 1);
          }
          if (typ->hasmax && typ->imax >= 0)
          {
            if (typ->incmax)
              fprintf(fd, "        <maxLength value=\"" SOAP_LONG_FORMAT "\"/>\n", typ->imax);
            else
              fprintf(fd, "        <maxLength value=\"" SOAP_LONG_FORMAT "\"/>\n", typ->imax - 1);
          }
        }
        fprintf(fd, "      </restriction>\n    </simpleType>\n");
      }
    }
  }
  else if (!is_transient(typ) && is_primclass(typ))
  {
    if ((!has_ns(typ) && all) || has_ns_eq(ns, p->sym->name))
    {
      q = ((Table*)typ->ref)->list;
      if (q && strncmp(q->sym->name, "xsd__anyType", 12))
      {
        if (is_string(q->info.typ) || is_wstring(q->info.typ) || is_stdstring(q->info.typ) || is_stdwstring(q->info.typ))
        {
          fprintf(fd, "    <complexType name=\"%s\" mixed=\"true\">", ns_remove(p->sym->name));
          gen_type_documentation(fd, p, ns);
          fprintf(fd, "      <simpleContent>\n        <extension base=\"%s\">\n", wsdl_type(q->info.typ, ns1));
          gen_schema_attributes(fd, typ, NULL, ns, ns1);
          fprintf(fd, "        </extension>\n      </simpleContent>\n    </complexType>\n");
        }
        else
        {
          fprintf(fd, "    <complexType name=\"%s\">", ns_remove(p->sym->name));
          gen_type_documentation(fd, p, ns);
          fprintf(fd, "      <simpleContent>\n        <extension base=\"%s\">\n", wsdl_type(q->info.typ, ns1));
          gen_schema_attributes(fd, typ, NULL, ns, ns1);
          fprintf(fd, "        </extension>\n      </simpleContent>\n    </complexType>\n");
        }
      }
    }
  }
  else if (!is_transient(typ))
  {
    q = ((Table*)typ->ref)->list;
    if (t && entry(t, p->sym) && (!q || !is_XML(q->info.typ)))
      ;
    else if (is_dynamic_array(typ))
    {
      if (eflag || (!has_ns(typ) && !is_untyped(typ)))
      {
        if (all && strcmp(typ->id->name, "SOAP_ENC__Array"))
        {
          d = get_Darraydims(typ)-1;
          for (i = 0; i < d; i++)
            cbuf[i] = ',';
          cbuf[i] = '\0';
          if (q->info.maxOccurs == 1)
          {
            fprintf(fd, "    <complexType name=\"%s\">\n      <complexContent>\n", wsdl_type(typ, NULL));
            if (!is_literal(encoding))
              fprintf(fd, "        <restriction base=\"SOAP-ENC:Array\">\n");
            fprintf(fd, "          <sequence>\n            <element name=\"%s\" type=\"%s\" minOccurs=\"0\" maxOccurs=\"unbounded\"%s/>\n          </sequence>\n", q->tag?ns_tag_remove(q):q->sym->name[5]?ns_remove(q->sym->name+5):"item", wsdl_type(q->info.typ, ns1), nillable_ref(q));
            if (!is_literal(encoding))
              fprintf(fd, "          <attribute ref=\"SOAP-ENC:arrayType\" WSDL:arrayType=\"%s[%s]\"/>\n        </restriction>\n", wsdl_type(q->info.typ, ns1), cbuf);
            fprintf(fd, "      </complexContent>\n    </complexType>\n");
          }
          else
          {
            fprintf(fd, "    <complexType name=\"%s\">\n      <complexContent>\n", wsdl_type(typ, NULL));
            if (!is_literal(encoding))
              fprintf(fd, "        <restriction base=\"SOAP-ENC:Array\">\n");
            fprintf(fd, "          <sequence>\n            <element name=\"%s\" type=\"%s\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"" SOAP_LONG_FORMAT "\"%s/>\n          </sequence>\n", q->tag?ns_tag_remove(q):q->sym->name[5]?ns_remove(q->sym->name+5):"item", wsdl_type(q->info.typ, ns1), q->info.minOccurs, q->info.maxOccurs, nillable_ref(q));
            if (!is_literal(encoding))
              fprintf(fd, "          <attribute ref=\"SOAP-ENC:arrayType\" WSDL:arrayType=\"%s[%s]\"/>\n        </restriction>\n", wsdl_type(q->info.typ, ns1), cbuf);
            fprintf(fd, "      </complexContent>\n    </complexType>\n");
          }
        }
      }
      else if (typ->ref && ((Table*)typ->ref)->prev && !is_transient(entry(classtable, ((Table*)typ->ref)->prev->sym)->info.typ) && strncmp(((Table*)typ->ref)->prev->sym->name, "xsd__anyType", 12))
      {
        if (q->info.maxOccurs == 1)
        {
          fprintf(fd, "    <complexType name=\"%s\">", ns_remove(p->sym->name));
          gen_type_documentation(fd, p, ns);
          fprintf(fd, "      <complexContent>\n        <extension base=\"%s\">\n          <sequence>\n", ns_convert(((Table*)typ->ref)->prev->sym->name));
          fprintf(fd, "            <element name=\"%s\" type=\"%s\" minOccurs=\"0\" maxOccurs=\"unbounded\" nillable=\"true\"/>\n", q->tag?ns_tag_remove(q):q->sym->name[5]?ns_remove(q->sym->name+5):"item", wsdl_type(q->info.typ, ns1));
          fprintf(fd, "          </sequence>\n        </extension>\n      </complexContent>\n");
          gen_schema_attributes(fd, typ, NULL, ns, ns1);
          fprintf(fd, "    </complexType>\n");
        }
        else
        {
          fprintf(fd, "    <complexType name=\"%s\">", ns_remove(p->sym->name));
          gen_type_documentation(fd, p, ns);
          fprintf(fd, "      <complexContent>\n        <extension base=\"%s\">\n          <sequence>\n", ns_convert(((Table*)typ->ref)->prev->sym->name));
          fprintf(fd, "            <element name=\"%s\" type=\"%s\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"" SOAP_LONG_FORMAT "\"%s/>\n", q->tag?ns_tag_remove(q):q->sym->name[5]?ns_remove(q->sym->name+5):"item", wsdl_type(q->info.typ, ns1), q->info.minOccurs, q->info.maxOccurs, nillable_ref(q));
          fprintf(fd, "          </sequence>\n        </extension>\n      </complexContent>\n");
          gen_schema_attributes(fd, typ, NULL, ns, ns1);
          fprintf(fd, "    </complexType>\n");
        }
      }
      else if (strcmp(typ->id->name, "SOAP_ENC__Array"))
      {
        if (q->info.maxOccurs == 1)
        {
          fprintf(fd, "    <complexType name=\"%s\">", ns_remove(p->sym->name));
          gen_type_documentation(fd, p, ns);
          fprintf(fd, "      <sequence>\n        <element name=\"%s\" type=\"%s\" minOccurs=\"0\" maxOccurs=\"unbounded\"%s/>\n      </sequence>\n    </complexType>\n", q->tag?ns_tag_remove(q):q->sym->name[5]?ns_remove(q->sym->name+5):"item", wsdl_type(q->info.typ, ns1), nillable_ref(q));
        }
        else
        {
          fprintf(fd, "    <complexType name=\"%s\">", ns_remove(p->sym->name));
          gen_type_documentation(fd, p, ns);
          fprintf(fd, "      <sequence>\n        <element name=\"%s\" type=\"%s\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"" SOAP_LONG_FORMAT "\"%s/>\n      </sequence>\n    </complexType>\n", q->tag?ns_tag_remove(q):q->sym->name[5]?ns_remove(q->sym->name+5):"item", wsdl_type(q->info.typ, ns1), q->info.minOccurs, q->info.maxOccurs, nillable_ref(q));
        }
      }
    }
    else if (is_discriminant(typ) && ((!has_ns(typ) && all) || has_ns_eq(ns, p->sym->name)))
    {
      if (typ->ref)
      {
        fprintf(fd, "    <complexType name=\"%s\">\n", ns_remove(p->sym->name));
        gen_schema_elements(fd, typ, NULL, ns, ns1);
        fprintf(fd, "    </complexType>\n");
      }
    }
    else if (typ->type == Tstruct && ((!has_ns(typ) && all) || has_ns_eq(ns, p->sym->name)))
    {
      if (typ->ref)
      {
        if (typ->base && !is_transient(typ->base))
        {
          fprintf(fd, "    <complexType name=\"%s\">", ns_remove(p->sym->name));
          gen_type_documentation(fd, p, ns);
          fprintf(fd, "      <complexContent>\n        <extension base=\"%s\">\n          <sequence>\n", ns_convert(typ->base->id->name));
          gen_schema_elements(fd, typ, typ->base, ns, ns1);
          fprintf(fd, "          </sequence>\n        </extension>\n      </complexContent>\n");
          gen_schema_attributes(fd, typ, typ->base, ns, ns1);
          fprintf(fd, "    </complexType>\n");
        }
        else
        {
          fprintf(fd, "    <complexType name=\"%s\">", ns_remove(p->sym->name));
          gen_type_documentation(fd, p, ns);
          fprintf(fd, "          <sequence>\n");
          gen_schema_elements(fd, typ, NULL, ns, ns1);
          fprintf(fd, "          </sequence>\n");
          gen_schema_attributes(fd, typ, NULL, ns, ns1);
          fprintf(fd, "    </complexType>\n");
        }
      }
    }
    else if (typ->type == Tclass && ((!has_ns(typ) && all) || has_ns_eq(ns, p->sym->name)))
    {
      if (typ->ref)
      {
        if (((Table*)typ->ref)->prev && !is_transient(entry(classtable, ((Table*)typ->ref)->prev->sym)->info.typ) && strncmp(((Table*)typ->ref)->prev->sym->name, "xsd__anyType", 12))
        {
          fprintf(fd, "    <complexType name=\"%s\">", ns_remove(p->sym->name));
          gen_type_documentation(fd, p, ns);
          fprintf(fd, "      <complexContent>\n        <extension base=\"%s\">\n          <sequence>\n", ns_convert(((Table*)typ->ref)->prev->sym->name));
          gen_schema_elements(fd, typ, NULL, ns, ns1);
          fprintf(fd, "          </sequence>\n        </extension>\n      </complexContent>\n");
          gen_schema_attributes(fd, typ, NULL, ns, ns1);
          fprintf(fd, "    </complexType>\n");
        }
        else
        {
          if (typ->base && !is_transient(typ->base))
          {
            fprintf(fd, "    <complexType name=\"%s\">", ns_remove(p->sym->name));
            gen_type_documentation(fd, p, ns);
            fprintf(fd, "      <complexContent>\n        <extension base=\"%s\">\n          <sequence>\n", ns_convert(typ->base->id->name));
            gen_schema_elements(fd, typ, typ->base, ns, ns1);
            fprintf(fd, "          </sequence>\n        </extension>\n      </complexContent>\n");
            gen_schema_attributes(fd, typ, typ->base, ns, ns1);
            fprintf(fd, "    </complexType>\n");
          }
          else
          {
            fprintf(fd, "    <complexType name=\"%s\">", ns_remove(p->sym->name));
            gen_type_documentation(fd, p, ns);
            fprintf(fd, "          <sequence>\n");
            gen_schema_elements(fd, typ, NULL, ns, ns1);
            fprintf(fd, "          </sequence>\n");
            gen_schema_attributes(fd, typ, NULL, ns, ns1);
            fprintf(fd, "    </complexType>\n");
          }
        }
      }
    }
  }
  fflush(fd);
}

void
gen_schema_elements(FILE *fd, Tnode *p, Tnode *b, const char *ns, const char *ns1)
{
  Entry *q;
  for (q = ((Table*)p->ref)->list; q; q = q->next)
  {
    if (b)
    {
      Entry *e;
      for (e = ((Table*)b->ref)->list; e; e = e->next)
        if (q->info.typ == e->info.typ && q->sym == e->sym)
          break;
      if (e)
        continue;
    }
    if (gen_schema_element(fd, p, q, ns, ns1))
      q = q->next;
  }
}

int
gen_schema_element(FILE *fd, Tnode *p, Entry *q, const char *ns, const char *ns1)
{
  const char *s, *t;
  if (is_transient(q->info.typ) || (q->info.sto & Sattribute) || q->info.typ->type == Tfun || q->info.typ->type == Tunion)
    return 0;
  if (is_repetition(q))
  {
    if (is_sequence(q->next))
    {
      if (q->info.maxOccurs == 1)
        fprintf(fd, "            <sequence minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"unbounded\">\n", q->info.minOccurs);
      else
        fprintf(fd, "            <sequence minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"" SOAP_LONG_FORMAT "\">\n", q->info.minOccurs, q->info.maxOccurs);
      if (q->next->info.typ->ref)
        gen_schema_elements(fd, (Tnode*)q->next->info.typ->ref, NULL, ns, ns1);
      fprintf(fd, "            </sequence>\n");
      return 1;
    }
    t = ns_tag_convert(q->next);
    if (*t == '-')
    {
      fprintf(fd, "            <any processContents=\"lax\" minOccurs=\"0\" maxOccurs=\"unbounded\"/>");
      if (!uflag)
        fprintf(fd, "<!-- %s -->", q->next->sym->name);
      fprintf(fd, "\n");
    }
    else if ((s = strchr(t+1, ':')) && (!strchr(q->next->sym->name+1, ':') || !has_ns_eq(ns, q->next->sym->name)))
    {
      if (((Tnode*)q->next->info.typ->ref)->type == Tpointer)
      {
        if (q->info.maxOccurs == 1)
          fprintf(fd, "            <element ref=\"%s\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"unbounded\"", t, q->info.minOccurs);
        else
          fprintf(fd, "     <element ref=\"%s\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"" SOAP_LONG_FORMAT "\"", t, q->info.minOccurs, q->info.maxOccurs);
      }
      else if (q->info.maxOccurs == 1)
        fprintf(fd, "            <element ref=\"%s\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"unbounded\"", t, q->info.minOccurs);
      else
        fprintf(fd, "            <element ref=\"%s\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"" SOAP_LONG_FORMAT "\"", t, q->info.minOccurs, q->info.maxOccurs);
      if (gen_member_documentation(fd, p->id, q, ns, 1))
        fprintf(fd, "            </element>\n");
    }
    else
    {
      const char *form = "";
      if (!s)
      {
        s = t;
        if (*s == ':')
        {
          s++;
          form = " form=\"unqualified\"";
        }
      }
      else
      {
        s++;
        form = " form=\"qualified\"";
      }
      if (((Tnode*)q->next->info.typ->ref)->type == Tpointer)
      {
        if (q->info.maxOccurs == 1)
          fprintf(fd, "            <element name=\"%s\" type=\"%s\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"unbounded\"%s%s", s, wsdl_type((Tnode*)q->next->info.typ->ref, ns1), q->info.minOccurs, nillable(q), form);
        else
          fprintf(fd, "            <element name=\"%s\" type=\"%s\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"" SOAP_LONG_FORMAT "\"%s%s", s, wsdl_type((Tnode*)q->next->info.typ->ref, ns1), q->info.minOccurs, q->info.maxOccurs, nillable(q), form);
      }
      else if (q->info.maxOccurs == 1)
        fprintf(fd, "            <element name=\"%s\" type=\"%s\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"unbounded\"%s", s, wsdl_type((Tnode*)q->next->info.typ->ref, ns1), q->info.minOccurs, form);
      else
        fprintf(fd, "            <element name=\"%s\" type=\"%s\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"" SOAP_LONG_FORMAT "\"%s", s, wsdl_type((Tnode*)q->next->info.typ->ref, ns1), q->info.minOccurs, q->info.maxOccurs, form);
      if (gen_member_documentation(fd, p->id, q, ns, 1))
        fprintf(fd, "            </element>\n");
    }
    return 1;
  }
  else if ((q->info.typ->type == Ttemplate && !is_smart(q->info.typ)) || (q->info.typ->type == Tpointer && ((Tnode*)q->info.typ->ref)->type == Ttemplate) || (q->info.typ->type == Treference && ((Tnode*)q->info.typ->ref)->type == Ttemplate) || (q->info.typ->type == Trvalueref && ((Tnode*)q->info.typ->ref)->type == Ttemplate))
  {
    t = ns_tag_convert(q);
    if (*t == '-')
    {
      fprintf(fd, "            <any processContents=\"lax\" minOccurs=\"0\" maxOccurs=\"unbounded\"/>");
      if (!uflag)
        fprintf(fd, "<!-- %s -->", q->sym->name);
      fprintf(fd, "\n");
    }
    else if ((s = strchr(t+1, ':')) && (!strchr(q->sym->name+1, ':') || !has_ns_eq(ns, q->sym->name)))
    {
      if (((Tnode*)q->info.typ->ref)->type == Tpointer)
      {
        if (q->info.maxOccurs == 1)
          fprintf(fd, "            <element ref=\"%s\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"unbounded\"", t, q->info.minOccurs);
        else
          fprintf(fd, "            <element ref=\"%s\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"" SOAP_LONG_FORMAT "\"", t, q->info.minOccurs, q->info.maxOccurs);
      }
      else if (q->info.maxOccurs == 1)
        fprintf(fd, "            <element ref=\"%s\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"unbounded\"", t, q->info.minOccurs);
      else
        fprintf(fd, "            <element ref=\"%s\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"" SOAP_LONG_FORMAT "\"", t, q->info.minOccurs, q->info.maxOccurs);
      if (gen_member_documentation(fd, p->id, q, ns, 1))
        fprintf(fd, "            </element>\n");
    }
    else
    {
      const char *form = "";
      if (!s)
      {
        s = t;
        if (*s == ':')
        {
          s++;
          form = " form=\"unqualified\"";
        }
      }
      else
      {
        s++;
        form = " form=\"qualified\"";
      }
      if (((Tnode*)q->info.typ->ref)->type == Tpointer)
      {
        if (q->info.maxOccurs == 1)
          fprintf(fd, "            <element name=\"%s\" type=\"%s\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"unbounded\"%s%s", s, wsdl_type((Tnode*)q->info.typ->ref, ns1), q->info.minOccurs, nillable(q), form);
        else
          fprintf(fd, "            <element name=\"%s\" type=\"%s\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"" SOAP_LONG_FORMAT "\"%s%s", s, wsdl_type((Tnode*)q->info.typ->ref, ns1), q->info.minOccurs, q->info.maxOccurs, nillable(q), form);
      }
      else if (q->info.maxOccurs == 1)
        fprintf(fd, "            <element name=\"%s\" type=\"%s\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"unbounded\"%s", s, wsdl_type((Tnode*)q->info.typ->ref, ns1), q->info.minOccurs, form);
      else
        fprintf(fd, "            <element name=\"%s\" type=\"%s\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"" SOAP_LONG_FORMAT "\"%s", s, wsdl_type((Tnode*)q->info.typ->ref, ns1), q->info.minOccurs, q->info.maxOccurs, form);
      if (gen_member_documentation(fd, p->id, q, ns, 1))
        fprintf(fd, "            </element>\n");
    }
  }
  else if (is_anytype(q)) /* ... maybe need to show all possible types rather than xsd:anyType */
  {
    t = ns_tag_convert(q->next);
    if (*t == '-')
    {
      fprintf(fd, "            <any processContents=\"lax\" minOccurs=\"0\" maxOccurs=\"unbounded\"/>");
      if (!uflag)
        fprintf(fd, "<!-- %s -->", q->next->sym->name);
      fprintf(fd, "\n");
    }
    else
    {
      fprintf(fd, "            <element name=\"%s\" type=\"xsd:anyType\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"" SOAP_LONG_FORMAT "\" nillable=\"true\"/>\n", t, q->info.minOccurs, q->info.maxOccurs);
    }
    return 1;
  }
  else if (is_choice(q))
  {
    if (q->info.minOccurs == 0)
      fprintf(fd, "          <choice minOccurs=\"0\" maxOccurs=\"1\">\n");
    else
      fprintf(fd, "          <choice>\n");
    if (q->next->info.typ->ref)
      gen_schema_elements(fd, q->next->info.typ, NULL, ns, ns1);
    fprintf(fd, "          </choice>\n");
    return 1;
  }
  else if (is_sequence(q))
  {
    if (q->info.minOccurs == 0)
      fprintf(fd, "          <sequence minOccurs=\"0\" maxOccurs=\"1\">\n");
    else
      fprintf(fd, "          <sequence>\n");
    if (q->info.typ->type == Tpointer)
      gen_schema_elements(fd, (Tnode*)q->info.typ->ref, NULL, ns, ns1);
    else if (q->info.typ->ref)
      gen_schema_elements(fd, q->info.typ, NULL, ns, ns1);
    fprintf(fd, "          </sequence>\n");
    return 0;
  }
  else
  {
    t = ns_tag_convert(q);
    if (*t == '-' || !*t)
    {
      fprintf(fd, "            <any processContents=\"lax\" minOccurs=\"0\" maxOccurs=\"1\"/>");
      if (!uflag)
        fprintf(fd, "<!-- %s -->", q->sym->name);
      fprintf(fd, "\n");
    }
    else if ((s = strchr(t+1, ':')) && (!strchr(q->sym->name+1, ':') || !has_ns_eq(ns, q->sym->name)))
    {
      fprintf(fd, "            <element ref=\"%s\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"" SOAP_LONG_FORMAT "\"%s%s", t, q->info.minOccurs, q->info.maxOccurs, nillable(q), set_default_value(q));
      if (gen_member_documentation(fd, p->id, q, ns, 1))
        fprintf(fd, "            </element>\n");
    }
    else
    {
      const char *form = "";
      if (!s)
      {
        s = t;
        if (*s == ':')
        {
          s++;
          form = " form=\"unqualified\"";
        }
      }
      else
      {
        s++;
        form = " form=\"qualified\"";
      }
      fprintf(fd, "            <element name=\"%s\" type=\"%s\" minOccurs=\"" SOAP_LONG_FORMAT "\" maxOccurs=\"" SOAP_LONG_FORMAT "\"%s%s%s", s, wsdl_type(q->info.typ, ns1), q->info.minOccurs, q->info.maxOccurs, nillable(q), set_default_value(q), form);
      if (gen_member_documentation(fd, p->id, q, ns, 1))
        fprintf(fd, "            </element>\n");
    }
  }
  fflush(fd);
  return 0;
}

void
gen_schema_elements_attributes(FILE *fd, Table *t, const char *ns, const char *ns1, const char *style, const char *encoding)
{
  Entry *p, *q, *e;
  Table *elt, *att;
  Service *sp;
  Method *m;
  Symbol *sym;
  const char *s, *method_style, *method_encoding, *method_response_encoding;
  int all = !strcmp(ns, ns1);
  elt = mktable(NULL);
  att = mktable(NULL);
  if (t && all)
  {
    for (p = t->list; p; p = p->next)
    {
      if (!eflag && p->info.typ->type == Tfun && !(p->info.sto & Sextern) && has_ns_eq(ns, p->sym->name))
      {
        method_encoding = encoding;
        method_response_encoding = NULL;
        method_style = style;
        for (sp = services; sp; sp = sp->next)
        {
          if (!tagcmp(sp->ns, ns))
          {
            for (m = sp->list; m; m = m->next)
            {
              if (is_eq_nons(m->name, p->sym->name))
              {
                if (m->mess == ENCODING)
                  method_encoding = m->part;
                else if (m->mess == RESPONSE_ENCODING)
                  method_response_encoding = m->part;
                else if (m->mess == STYLE)
                  method_style = m->part;
              }
            }
          }
        }
        if (!method_response_encoding)
          method_response_encoding = method_encoding;
        q = entry(classtable, p->sym);
        if (q)
        {
          if (is_document(method_style))
          {
            if (!is_invisible(p->sym->name))
            {
              sym = p->sym;
              s = sym->name;
              if (*s == '_' && !(sym = lookup(s + 1)))
                sym = install(s + 1, ID);
              if (!entry(elt, sym))
              {
                if (!uflag)
                  fprintf(fd, "    <!-- operation request element -->\n");
                fprintf(fd, "    <element name=\"%s\">\n      <complexType>\n          <sequence>\n", ns_tag_remove(p));
                gen_schema_elements(fd, q->info.typ, NULL, ns, ns1);
                fprintf(fd, "          </sequence>\n");
                gen_schema_attributes(fd, q->info.typ, NULL, ns, ns1);
                fprintf(fd, "      </complexType>\n    </element>\n");
                e = enter(elt, sym);
                e->info = p->info;
              }
            }
            else
            {
              for (q = ((Table*)q->info.typ->ref)->list; q; q = q->next)
              {
                if (!(q->info.sto & Sextern) && !has_ns_eq(NULL, q->sym->name))
                {
                  sym = q->sym;
                  s = sym->name;
                  if (*s == '_' && !(sym = lookup(s + 1)))
                    sym = install(s + 1, ID);
                  if (!entry(elt, sym)) /* enter/entry w/o prefix is not useful to avoid clashes */
                  {
                    if (!uflag)
                      fprintf(fd, "    <!-- operation request element -->\n");
                    fprintf(fd, "    <element name=\"%s\" type=\"%s\"/>\n", ns_tag_remove(q), wsdl_type(q->info.typ, ns1));
                    e = enter(elt, sym);
                    e->info = q->info;
                  }
                }
              }
            }
          }
          q = (Entry*)p->info.typ->ref;
          for (e = t->list; e; e = e->next)
            if (e != p && e->info.typ->type == Tfun && !(e->info.sto & Sextern) && q == (Entry*)e->info.typ->ref)
              q = NULL;
          if (q && !is_transient(q->info.typ))
          {
            if (!is_response(q->info.typ))
            {
              if (is_document(method_style))
              {
                if (!uflag)
                  fprintf(fd, "    <!-- operation response element -->\n");
                fprintf(fd, "    <element name=\"%sResponse\">\n      <complexType>\n", ns_tag_remove(p));
                fprintf(fd, "          <sequence>\n");
                gen_schema_element(fd, p->info.typ, q, ns, ns1);
                fprintf(fd, "          </sequence>\n");
                fprintf(fd, "      </complexType>\n    </element>\n");
              }
            }
            else if (((Tnode*)q->info.typ->ref)->ref)
            {
              if (is_document(method_style))
              {
                if (!has_ns_eq(NULL, q->sym->name))
                {
                  sym = ((Tnode*)q->info.typ->ref)->id;
                  s = sym->name;
                  if (*s == '_' && !(sym = lookup(s + 1)))
                    sym = install(s + 1, ID);
                  if (!entry(elt, sym))
                  {
                    if (!uflag)
                      fprintf(fd, "    <!-- operation response element and type -->\n");
                    fprintf(fd, "    <element name=\"%s\" type=\"%s\"/>\n    <complexType name=\"%s\">\n", ns_remove(((Tnode*)q->info.typ->ref)->id->name), ns_convert(((Tnode*)q->info.typ->ref)->id->name), ns_remove(((Tnode*)q->info.typ->ref)->id->name));
                    fprintf(fd, "          <sequence>\n");
                    gen_schema_elements(fd, (Tnode*)q->info.typ->ref, NULL, ns, ns1);
                    fprintf(fd, "          </sequence>\n");
                    gen_schema_attributes(fd, (Tnode*)q->info.typ->ref, NULL, ns, ns1);
                    fprintf(fd, "    </complexType>\n");
                    e = enter(elt, sym);
                    e->info = q->info;
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  if (t && !eflag)
  {
    for (p = t->list; p; p = p->next)
    {
      if (p->info.typ->type == Tfun && !(p->info.sto & Sextern))
      {
        q = (Entry*)p->info.typ->ref;
        if (q && !is_transient(q->info.typ))
        {
          if (is_response(q->info.typ))
          {
            if (has_ns_eq(ns, q->sym->name))
            {
              sym = q->sym;
              s = sym->name;
              if (*s == '_' && !(sym = lookup(s + 1)))
                sym = install(s + 1, ID);
              if (!entry(elt, sym))
              {
                if (!uflag)
                  fprintf(fd, "    <!-- operation response element -->\n");
                fprintf(fd, "    <element name=\"%s\" type=\"%s\"/>\n", ns_tag_remove(q), wsdl_type(q->info.typ, ns1));
                e = enter(elt, sym);
                e->info = q->info;
              }
            }
          }
        }
      }
    }
  }
  for (p = typetable->list; p; p = p->next)
  {
    if (is_transient(p->info.typ) || is_primclass(p->info.typ) || is_dynamic_array(p->info.typ))
      continue;
    if (is_element(p->info.typ) && has_ns_eq(ns, p->sym->name))
    {
      m = NULL;
      for (sp = services; sp && !m; sp = sp->next)
        for (m = sp->list; m; m = m->next)
          if ((m->mess&FAULT) && m->part && is_eq(m->part, p->sym->name))
            break;
      if (!m)
      {
        sym = p->sym;
        s = sym->name;
        if (*s == '_' && !(sym = lookup(s + 1)))
          sym = install(s + 1, ID);
        if (!entry(elt, sym))
        {
          fprintf(fd, "    <element name=\"%s\" type=\"%s\"/>\n", ns_tag_remove(p), base_type(p->info.typ, ns1));
          e = enter(elt, sym);
          e->info = p->info;
        }
      }
    }
  }
  for (p = classtable->list; p; p = p->next)
  {
    if (!p->info.typ->ref || /* is_invisible(p->info.typ->id->name) || */ is_transient(p->info.typ) || is_primclass(p->info.typ) || is_dynamic_array(p->info.typ))
      continue;
    for (q = ((Table*)p->info.typ->ref)->list; q; q = q->next)
    {
      if (!is_repetition(q) && !is_anytype(q) && (!strchr(q->sym->name+1, ':') || !eq_ns(p->sym->name, q->sym->name)) && has_ns_eq(ns, q->sym->name) && !is_transient(q->info.typ) && q->info.typ->type != Tfun)
      {
        m = NULL;
        for (sp = services; sp && !m; sp = sp->next)
          for (m = sp->list; m; m = m->next)
            if ((m->mess&FAULT) && m->part && is_eq(m->part, q->sym->name))
              break;
        if (m)
          continue; /* already generated element for fault */
        if (!(q->info.sto & Sattribute) && (q->info.typ->type == Tclass || q->info.typ->type == Tstruct || is_typedef(q->info.typ)))
          if (is_element(q->info.typ) || is_eq(q->info.typ->id->name, q->sym->name))
            continue; /* type is already an element or type name matches member name */
        sym = q->sym;
        s = sym->name;
        if (*s == '_' && !(sym = lookup(s + 1)))
          sym = install(s + 1, ID);
        if (q->info.sto & Sattribute)
        {
          e = entry(att, sym);
          if (e)
          {
            if ((!is_primitive(q->info.typ) || e->info.typ->type != q->info.typ->type) && reftype(e->info.typ) != reftype(q->info.typ))
            {
              sprintf(errbuf, "Member '%s::%s' of type '%s' at line %d has a type that does not correspond to the required unique type '%s' defined for root-level attribute '%s'", p->sym->name, q->sym->name, c_type(q->info.typ), q->lineno, c_type(e->info.typ), ns_convert(q->sym->name));
              semwarn(errbuf);
            }
          }
          else
          {
            fprintf(fd, "          <attribute name=\"%s\" type=\"%s\"/>\n", ns_tag_remove(q), wsdl_type(q->info.typ, ns1));
            e = enter(att, sym);
            e->info = q->info;
          }
        }
        else
        {
          e = entry(elt, sym);
          if (e)
          {
            if ((!is_primitive(q->info.typ) || e->info.typ->type != q->info.typ->type) && reftype(e->info.typ) != reftype(q->info.typ))
            {
              sprintf(errbuf, "Member '%s::%s' of type '%s' at line %d has a type that does not correspond to the required unique type '%s' defined for root-level element '%s'", p->sym->name, q->sym->name, c_type(q->info.typ), q->lineno, c_type(e->info.typ), ns_convert(q->sym->name));
              semwarn(errbuf);
            }
          }
          else
          {
            fprintf(fd, "    <element name=\"%s\" type=\"%s\"/>\n", ns_tag_remove(q), wsdl_type(q->info.typ, ns1));
            e = enter(elt, sym);
            e->info = q->info;
          }
        }
      }
    }
    if (is_element(p->info.typ) && has_ns_eq(ns, p->sym->name))
    {
      m = NULL;
      for (sp = services; sp && !m; sp = sp->next)
        for (m = sp->list; m; m = m->next)
          if ((m->mess&FAULT) && m->part && is_eq(m->part, p->sym->name))
            break;
      if (!m)
      {
        sym = p->sym;
        s = sym->name;
        if (*s == '_' && !(sym = lookup(s + 1)))
          sym = install(s + 1, ID);
        if (!entry(elt, sym))
        {
          fprintf(fd, "    <element name=\"%s\" type=\"%s\"/>\n", ns_tag_remove(p), wsdl_type(p->info.typ, ns1));
          e = enter(elt, sym);
          e->info = p->info;
        }
      }
    }
  }
  freetable(elt);
  freetable(att);
}

void
gen_schema_attributes(FILE *fd, Tnode *p, Tnode *b, const char *ns, const char *ns1)
{
  Entry *q;
  const char *t = NULL, *s = NULL, *r = NULL;
  for (q = ((Table*)p->ref)->list; q; q = q->next)
  {
    if (b)
    {
      Entry *e;
      for (e = ((Table*)b->ref)->list; e; e = e->next)
        if (q->info.typ == e->info.typ && q->sym == e->sym)
          break;
      if (e)
        continue;
    }
    if (q->info.sto & Sattribute && !(q->info.sto & (Sprivate | Sprotected)))
    {
      r = set_default_value(q);
      t = ns_tag_convert(q);
      if (*t == '-' || is_anyAttribute(q->info.typ))
      {
        fprintf(fd, "          <anyAttribute processContents=\"lax\"/>");
        if (!uflag)
          fprintf(fd, "<!-- %s -->", q->sym->name);
        fprintf(fd, "\n");
      }
      else if (*t && (s = strchr(t+1, ':')) && (!strchr(q->sym->name+1, ':') || !has_ns_eq(ns, q->sym->name)))
      {
        if (q->info.minOccurs)
          fprintf(fd, "          <attribute ref=\"%s\" use=\"required\"%s/>\n", t, r);
        else if (q->info.maxOccurs == 0)
          fprintf(fd, "          <attribute ref=\"%s\" use=\"prohibited\"%s/>\n", t, r);
        else
          fprintf(fd, "          <attribute ref=\"%s\"%s/>\n", t, r);
      }
      else
      {
        const char *form = "";
        if (!s)
        {
          s = t;
          if (*s == ':')
          {
            s++;
            form = " form=\"unqualified\"";
          }
        }
        else
        {
          s++;
          form = " form=\"qualified\"";
        }
        if (q->info.minOccurs)
          fprintf(fd, "          <attribute name=\"%s\" type=\"%s\" use=\"required\"%s%s", s, wsdl_type(q->info.typ, ns1), form, r);
        else if (q->info.maxOccurs == 0)
          fprintf(fd, "          <attribute name=\"%s\" type=\"%s\" use=\"prohibited\"", s, wsdl_type(q->info.typ, ns1));
        else
          fprintf(fd, "          <attribute name=\"%s\" type=\"%s\"%s%s", s, wsdl_type(q->info.typ, ns1), form, r);
        if (gen_member_documentation(fd, p->id, q, ns, 1))
          fprintf(fd, "          </attribute>\n");
      }
      fflush(fd);
    }
  }
}

void
gen_report_hr()
{
  fprintf(freport, "[![][1] To top](#)\n\n\n");
}

void
gen_report_operation(const char *name, Entry *method, int service)
{
  Service *sp;
  Method *m;
  int soap = (soap_version >= 0);
  int version = soap_version;
  int get = 0;
  int put = 0;
  int del = 0;
  int post = 0;
  int mimein = 0;
  int mimeout = 0;
  const char *style = NULL, *encoding = NULL;
  const char *action = NULL, *response_action = NULL, *method_encoding = NULL, *method_response_encoding = NULL;
  Entry *result, *p;
  result = (Entry*)method->info.typ->ref;
  (void)post;
  (void)method_response_encoding;
  p = entry(classtable, method->sym);
  if (!p)
    execerror("no table entry");
  if (name)
  {
    if (service)
      fprintf(freport, "### Service Operation `%s::%s()`\n\n", name, ns_remove(method->sym->name));
    else
      fprintf(freport, "### Proxy Operation `%s::%s()`\n\n", name, ns_remove(method->sym->name));
  }
  else
  {
    if (service)
      fprintf(freport, "### Service Operation `%s()`\n\n", ident(method->sym->name));
    else
      fprintf(freport, "### Operation `%s()`\n\n", ident(method->sym->name));
  }
  for (sp = services; sp; sp = sp->next)
  {
    if (has_ns_eq(sp->ns, method->sym->name))
    {
      style = sp->style;
      encoding = sp->encoding;
      method_encoding = encoding;
      method_response_encoding = NULL;
      if (sp->protocol)
      {
        if (strstr(sp->protocol, "GET"))
          get = 1;
        else if (strstr(sp->protocol, "PUT"))
          put = 1;
        else if (strstr(sp->protocol, "DELETE"))
          del = 1;
        else if (strstr(sp->protocol, "POST"))
          post = 1;
        if (strncmp(sp->protocol, "SOAP", 4))
          soap = 0;
        else if (strlen(sp->protocol) > 6)
          version = sp->protocol[6] - '0';
      }
      for (m = sp->list; m; m = m->next)
      {
        if (is_eq_nons(m->name, method->sym->name))
        {
          if (m->mess == ACTION || m->mess == REQUEST_ACTION)
          action = m->part;
          else if (m->mess == RESPONSE_ACTION)
            response_action = m->part;
          else if (m->mess == ENCODING)
            method_encoding = m->part;
          else if (m->mess == RESPONSE_ENCODING)
            method_response_encoding = m->part;
          else if (m->mess == PROTOCOL)
          {
            if (strstr(m->part, "GET"))
              get = 1;
            else if (strstr(m->part, "PUT"))
              put = 1;
            else if (strstr(m->part, "DELETE"))
              del = 1;
            else if (strstr(m->part, "POST"))
              post = 1;
            if (strncmp(m->part, "SOAP", 4))
              soap = 0;
            else if (strlen(m->part) > 6)
              version = m->part[6] - '0';
          }
          else
          {
            if ((m->mess&MIMEIN) && !strcmp(m->part, "application/x-www-form-urlencoded"))
              mimein = 1;
            if ((m->mess&MIMEOUT) && !strcmp(m->part, "application/x-www-form-urlencoded"))
              mimeout = 1;
          }
        }
      }
      break;
    }
  }
  gen_report_type_doc(p);
  fprintf(freport, "This service operation is declared in [%s](%s) at line %d and has the following properties:\n\n", method->filename, method->filename, method->lineno);
  if (soap)
  {
    fprintf(freport, "- SOAP 1.%d protocol\n", version ? version : 1);
    fprintf(freport, "- SOAP %s style\n", is_document(style) ? "document" : "rpc");
    if (sp && sp->URI && method_encoding)
    {
      if (is_literal(method_encoding))
        fprintf(freport, "- SOAP literal\n");
      else if (method_encoding && *method_encoding)
        fprintf(freport, "- SOAP %s\n", method_encoding);
      else if (method_encoding)
        fprintf(freport, "- SOAP encoded\n");
    }
    else if (eflag)
      fprintf(freport, "- SOAP encoded\n");
    else
      fprintf(freport, "- SOAP literal\n");
    if (action && *action == '"')
      fprintf(freport, "- SOAP action %s\n", action);
    else if (action)
      fprintf(freport, "- SOAP action \"%s\"\n", action);
    else
      fprintf(freport, "- SOAP action \"\"\n");
    if (response_action && *response_action == '"')
      fprintf(freport, "- SOAP response action %s\n", response_action);
    else if (response_action)
      fprintf(freport, "- SOAP response action \"%s\"\n", response_action);
  }
  else
  {
    fprintf(freport, "- REST operation\n");
  }
  if (post)
    fprintf(freport, "- POST method\n");
  else if (get)
    fprintf(freport, "- GET method\n");
  else if (put)
    fprintf(freport, "- PUT method\n");
  else if (del)
    fprintf(freport, "- DELETE method\n");
  if (!get && !del && mimein)
    fprintf(freport, "- HTTP application/x-www-form-urlencoded request\n");
  if (!put && !del && mimeout)
    fprintf(freport, "- HTTP application/x-www-form-urlencoded response\n");
  if (sp && sp->URL)
    fprintf(freport, "- Default service endpoint URL \"%s\"\n", sp->URL);
  if (!soap && action)
    fprintf(freport, "- Operation endpoint location path \"%s\"\n", action);
  if (sp && sp->URI)
    fprintf(freport, "- Operation namespace prefix `%s` and URI \"[%s](#doc-namespaces)\"\n", sp->ns, sp->URI);
  if (name)
  {
    if (service)
    {
      if (!is_transient(result->info.typ))
        fprintf(freport, "\nThe following service class method must be implemented in the service back-end code and will be called by the service dispatcher `%s::serve(soap)`:\n", name);
      else
        fprintf(freport, "\nThe following service class method must be implemented in the service back-end code and will be called by the service dispatcher `%s::serve(soap)`, and should return the value of `int %s::send_empty_%s_response(struct soap *soap, int httpcode)` when using HTTP:\n", name, name, ns_remove(method->sym->name));
    }
    else
    {
      fprintf(freport, "\nTo invoke this service operation, use the following methods:\n");
    }
  }
  else
  {
    if (service)
    {
      if (!is_transient(result->info.typ))
        fprintf(freport, "\nThe following service operation function declared in [%s](%s) should be implemented in the service back-end code and is called by the service dispatcher `%s_serve(soap)` defined in [%s](%s), and should return `SOAP_OK` with the result value `%s` set or return an error code:\n", soapStub, pathsoapStub, nflag?prefix:"soap", soapServer, pathsoapServer, ident(result->sym->name));
      else
        fprintf(freport, "\nThe following service operation function declared in [%s](%s) should be implemented in the service back-end code and is called by the service dispatcher `%s_serve(soap)` defined in [%s](%s), and should return the value of `int soap_send_empty_response(struct soap *soap, int httpcode)` when using HTTP:\n", soapStub, pathsoapStub, nflag?prefix:"soap", soapServer, pathsoapServer);
    }
    else
    {
      if (!is_transient(result->info.typ))
        fprintf(freport, "\nTo invoke this service operation, use the following auto-generated function declared in [%s](%s) and defined in [%s](%s):\n", soapStub, pathsoapStub, soapClient, pathsoapClient);
      else
        fprintf(freport, "\nTo invoke this service operation, use the following auto-generated send function declared in [%s](%s) and defined in [%s](%s), where the recv function can be used by an asynchronous receiver:\n", soapStub, pathsoapStub, soapClient, pathsoapClient);
    }
  }
}

void
gen_report_type(Tnode *typ, const char *what)
{
  if (!is_transient(typ) && ((typ->type == Tstruct || typ->type == Tclass || typ->type == Tenum || typ->type == Tenumsc || is_typedef(typ)) && !is_stdstr(typ)) && !is_bool(typ))
    fprintf(freport, ", where the type of this %s is <code><a href=\"#%s\"> %s </a></code>", what, ident(typ->id->name), ident(typ->id->name));
  else if (typ->type == Tpointer && !is_string(typ) && !is_wstring(typ) && !is_soapref(typ))
  {
    Tnode *ref = typ->ref;
    if ((ref->type == Tstruct || ref->type == Tclass || ref->type == Tenum || ref->type == Tenumsc || is_typedef(ref)) && !is_stdstr(ref) && !is_bool(ref))
      fprintf(freport, ", where the type of this %s is a pointer to <code><a href=\"#%s\"> %s </a></code>", what, ident(ref->id->name), ident(ref->id->name));
  }
  else if (is_smart(typ))
  {
    Tnode *ref = typ->ref;
    if ((ref->type == Tstruct || ref->type == Tclass || ref->type == Tenum || ref->type == Tenumsc || is_typedef(ref)) && !is_stdstr(ref) && !is_bool(ref))
      fprintf(freport, ", where the type of this %s is a smart pointer to <code><a href=\"#%s\"> %s </a></code>", what, ident(ref->id->name), ident(ref->id->name));
  }
  else if (is_template(typ))
  {
    Tnode *ref = typ->ref;
    if (ref->type == Tpointer || is_smart(ref))
    {
      ref = (Tnode*)ref->ref;
      if ((ref->type == Tstruct || ref->type == Tclass || ref->type == Tenum || ref->type == Tenumsc || is_typedef(ref)) && !is_stdstr(ref) && !is_bool(ref))
        fprintf(freport, ", where the type of this %s is a container of pointers to <code><a href=\"#%s\"> %s </a></code>", what, ident(ref->id->name), ident(ref->id->name));
    }
    else if ((ref->type == Tstruct || ref->type == Tclass || ref->type == Tenum || ref->type == Tenumsc || is_typedef(ref)) && !is_stdstr(ref) && !is_bool(ref))
      fprintf(freport, ", where the type of this %s is a container of <code><a href=\"#%s\"> %s </a></code>", what, ident(ref->id->name), ident(ref->id->name));
  }
}

void
gen_report_type_doc(Entry *type)
{
  Service *sp;
  Data *d;
  if (!type->sym)
    return;
  for (sp = services; sp; sp = sp->next)
  {
    if (has_ns_eq(sp->ns, type->sym->name))
    {
      for (d = sp->data; d; d = d->next)
      {
        if (d->name && d->text && !strstr(d->name, "::") && is_eq_nons(d->name, type->sym->name))
        {
          gen_text(freport, d->text);
          fprintf(freport, "\n\n");
        }
      }
    }
  }
}

void
gen_report_members(Entry *type, const char *nsa, const char *nse)
{
  Entry *q;
  int flag = 0;
  if (!(Table*)type->info.typ->ref)
    return;
  if (is_dynamic_array(type->info.typ) || is_choice(type))
    return;
  fprintf(freport, "where:\n\n");
  if (type->info.typ->baseid)
    fprintf(freport, "- <code><a href=\"#%s\"> %s </a></code> is the base class of this derived class\n", ident(type->info.typ->baseid->name), ident(type->info.typ->baseid->name));
  for (q = classtable->list; q; q = q->next)
    if (q->info.typ->baseid == type->sym)
      fprintf(freport, "- <code><a href=\"#%s\"> %s </a></code> is a derived class of this base class\n", c_type(q->info.typ), c_type(q->info.typ));
  if (type->info.typ->base && !is_transient(type->info.typ->base))
    fprintf(freport, "- <code><a href=\"#%s\"> %s </a></code> is the base type of this derived type\n", ident(type->info.typ->base->id->name), ident(type->info.typ->base->id->name));
  for (q = ((Table*)type->info.typ->ref)->list; q; q = q->next)
    if (is_pointer_to_derived(q))
      fprintf(freport, "- <code><a href=\"#%s\"> %s </a></code> is a derived type of this base type\n", c_type(q->info.typ->ref), c_type(q->info.typ->ref));
  for (q = ((Table*)type->info.typ->ref)->list; q; q = q->next)
  {
    if (q->info.typ->type == Tfun)
      continue;
    if ((q->info.sto & Stypedef))
      continue;
    if (flag)
    {
      flag = 0;
      continue;
    }
    fprintf(freport, "- `%s`", c_type_id(q->info.typ, q->sym->name));
    gen_report_member(type, q);
    if (is_anyAttribute(q->info.typ))
      fprintf(freport, " is an XML DOM attribute list");
    else if (q->info.sto & Sattribute)
    {
      if (q->info.minOccurs >= 1)
        fprintf(freport, " is a required attribute *`%s`* of XML schema type *`%s`*", ns_add(q, nsa), wsdl_type(q->info.typ, ""));
      else
        fprintf(freport, " is an optional attribute *`%s`* of XML schema type *`%s`*", ns_add(q, nsa), wsdl_type(q->info.typ, ""));
      if (is_external(q->info.typ))
        fprintf(freport, " with a custom serializer `%s`", c_type_sym(q->info.typ));
    }
    else if (is_soapref(q->info.typ))
    {
      fprintf(freport, " the context that manages this object");
    }
    else if (q->info.sto & (Sconst | Sprivate | Sprotected))
    {
      fprintf(freport, " is not serialized\n");
      continue;
    }
    else if (!is_dynamic_array(type->info.typ) && is_repetition(q))
    {
      if (q->info.maxOccurs > 1)
        fprintf(freport, " is a sequence of " SOAP_LONG_FORMAT " to " SOAP_LONG_FORMAT " elements *`<%s>`* of XML schema type *`%s`*", q->info.minOccurs, q->info.maxOccurs, ns_add(q->next, nse), wsdl_type(q->next->info.typ, ""));
      else if (q->info.minOccurs >= 1)
        fprintf(freport, " is a sequence of at least " SOAP_LONG_FORMAT " elements *`<%s>`* of XML schema type *`%s`*", q->info.minOccurs, ns_add(q->next, nse), wsdl_type(q->next->info.typ, ""));
      else
        fprintf(freport, " is a sequence of elements *`<%s>`* of XML schema type *`%s`*", ns_add(q->next, nse), wsdl_type(q->next->info.typ, ""));
      fprintf(freport, " stored in dynamic array `%s` of length `%s`", ident(q->next->sym->name), ident(q->sym->name));
      flag = 1;
    }
    else if (is_anytype(q))
    {
      fprintf(freport, " is any type of element *`<%s>`* assigned to member `%s` as a `void*` pointer cast, with its `SOAP_TYPE_Name` assigned to member `%s`", ns_add(q->next, nse), ident(q->next->sym->name), ident(q->sym->name));
      flag = 1;
    }
    else if (is_choice(q))
    {
      Entry *r;
      fprintf(freport, " is a union `%s` with variant selector `%s` set to one of:", c_type(q->next->info.typ), ident(q->sym->name));
      for (r = ((Table*)q->next->info.typ->ref)->list; r; r = r->next)
        fprintf(freport, " `%s`", soap_union_member(q->next->info.typ, r));
      flag = 1;
    }
    else if (is_anyType(q->info.typ))
      fprintf(freport, " is an XML DOM element node graph");
    else if (is_item(q))
      fprintf(freport, " is simple content of XML schema type *`%s`* wrapped in *`%s`*", wsdl_type(q->info.typ, ""), wsdl_type(type->info.typ, ""));
    else if (is_self(q))
      fprintf(freport, " is *`%s`* wrapped in *`%s`*, where `__self` matches the element tag of the struct/class member with the `%s` type", wsdl_type(q->info.typ, ""), wsdl_type(type->info.typ, ""), c_type(type->info.typ));
    else if (q->info.typ->type != Tfun && q->info.typ->type != Tunion && !(q->info.sto & (Sconst | Sprivate | Sprotected)) && !(q->info.sto & Sattribute) && !is_transient(q->info.typ) && !is_external(q->info.typ) && strncmp(q->sym->name, "__", 2))
    {
      if (q->info.maxOccurs > 1)
        fprintf(freport, " is a sequence of " SOAP_LONG_FORMAT " to " SOAP_LONG_FORMAT " elements *`<%s>`* of XML schema type *`%s`*", q->info.minOccurs, q->info.maxOccurs, ns_add(q, nse), wsdl_type(q->info.typ, ""));
      else if (q->info.minOccurs >= 1)
      {
        if (q->info.nillable)
          fprintf(freport, " is a required nillable (xsi:nil when NULL) element *`<%s>`* of XML schema type *`%s`*", ns_add(q, nse), wsdl_type(q->info.typ, ""));
        else
          fprintf(freport, " is a required element *`<%s>`* of XML schema type *`%s`*", ns_add(q, nse), wsdl_type(q->info.typ, ""));
      }
      else
        fprintf(freport, " is an optional element *`<%s>`* of XML schema type *`%s`*", ns_add(q, nse), wsdl_type(q->info.typ, ""));
    }
    else if (is_external(q->info.typ) && q->info.minOccurs >= 1)
    {
      if (q->info.nillable)
        fprintf(freport, " is a required nillable (xsi:nil when NULL) element *`<%s>`* of XML schema type *`%s`* with a custom serializer `%s`", ns_add(q, nse), wsdl_type(q->info.typ, ""), c_type_sym(q->info.typ));
      else
        fprintf(freport, " is a required element *`<%s>`* of XML schema type *`%s`* with a custom serializer `%s`", ns_add(q, nse), wsdl_type(q->info.typ, ""), c_type_sym(q->info.typ));
    }
    else if (is_external(q->info.typ))
    {
      fprintf(freport, " is an optional element *`<%s>`* of XML schema type *`%s`* with a custom serializer `%s`", ns_add(q, nse), wsdl_type(q->info.typ, ""), c_type_sym(q->info.typ));
    }
    else if (is_pointer_to_derived(q))
    {
      fprintf(freport, " is a transient pointer to a derived type value that replaces the value of this base type when non-NULL");
    }
    else if (is_transient(q->info.typ))
    {
      fprintf(freport, " is transient and not serialized");
    }
    if (q->info.hasval)
    {
      if (q->info.fixed)
        fprintf(freport, " that must have the fixed value `%s`", c_init_a(q, ""));
      else
        fprintf(freport, " with default value `%s`", c_init_a(q, ""));
    }
    else if (q->info.ptrval)
    {
      if (q->info.fixed)
        fprintf(freport, " that has the fixed value `%s`", c_init_a(q, ""));
      else
        fprintf(freport, " with default value `%s`", c_init_a(q, ""));
    }
    gen_report_type(q->info.typ, "member");
    if (is_invisible(q->sym->name))
      fprintf(freport, " (the leading underscores of the member name make this member invisible in XML, meaning it has no element tag)");
    else if (is_unmatched(q->sym))
      fprintf(freport, " (the leading underscore of the member name makes this member a wildcard that matches any XML element tag)");
    fprintf(freport, "\n");
  }
  if (has_setter(type->info.typ))
    fprintf(freport, "- `int set(struct soap *soap)` is a setter method, which is invoked by the serializer immediately before the instance is serialized\n");
  if (has_getter(type->info.typ))
    fprintf(freport, "- `int get(struct soap *soap)` is a getter method, which is invoked by the deserializer immediately after the instance is populated by the deserializer\n");
  fprintf(freport, "\n");
}

void
gen_report_member(Entry *type, Entry *member)
{
  Service *sp;
  const char *t;
  if (!type->sym || !member->sym)
    return;
  t = ns_remove(type->sym->name);
  for (sp = services; sp; sp = sp->next)
  {
    Data *d;
    if (has_ns_eq(sp->ns, type->sym->name))
    {
      for (d = sp->data; d; d = d->next)
      {
        const char *s = strstr(d->name, "::");
        if (s && !strncmp(t, d->name, s - d->name) && !t[s - d->name] && !strcmp(s + 2, member->sym->name))
        {
          fprintf(freport, " ");
          gen_text(freport, d->text);
        }
      }
    }
  }
}

void
gen_method_documentation(FILE *fd, Entry *p, const char *ns)
{
  Service *sp;
  Data *d;
  if (!p->sym)
    return;
  for (sp = services; sp; sp = sp->next)
  {
    if (!tagcmp(sp->ns, ns))
    {
      for (d = sp->data; d; d = d->next)
      {
        if (d->name && d->text && !strstr(d->name, "::") && is_eq_nons(d->name, p->sym->name))
        {
          fprintf(fd, "    <documentation>\n      ");
          gen_text(fd, d->text);
          fprintf(fd, "\n    </documentation>\n");
          return;
        }
      }
    }
  }
  if (!uflag)
    fprintf(fd, "    <documentation>Service definition of function %s</documentation>\n", p->sym->name);
}

void
gen_type_documentation(FILE *fd, Entry *type, const char *ns)
{
  Service *sp;
  Data *d;
  if (!type->sym)
  {
    fprintf(fd, "\n");
    return;
  }
  for (sp = services; sp; sp = sp->next)
  {
    if (!tagcmp(sp->ns, ns))
    {
      for (d = sp->data; d; d = d->next)
      {
        if (d->name && d->text && !strstr(d->name, "::") && is_eq_nons(d->name, type->sym->name))
        {
          fprintf(fd, "\n      <annotation>\n        <documentation>\n          ");
          gen_text(fd, d->text);
          fprintf(fd, "\n        </documentation>\n      </annotation>\n");
          return;
        }
      }
    }
  }
  if (!uflag)
    fprintf(fd, "<!-- %s -->", type->sym->name);
  fprintf(fd, "\n");
}

int
gen_member_documentation(FILE *fd, Symbol *type, Entry *member, const char *ns, int scope)
{
  Service *sp;
  Data *d;
  const char *t;
  if (!type || !member->sym)
  {
    fprintf(fd, "/>\n");
    return 0;
  }
  t = ns_remove(type->name);
  for (sp = services; sp; sp = sp->next)
  {
    if (!tagcmp(sp->ns, ns))
    {
      for (d = sp->data; d; d = d->next)
      {
        if (d->name && d->text)
        {
          const char *s = strstr(d->name, "::");
          if (s && !strncmp(t, d->name, s - d->name) && !t[s - d->name] && !strcmp(s + 2, member->sym->name))
          {
            fprintf(fd, ">\n            <annotation>\n              <documentation>\n                ");
            gen_text(fd, d->text);
            fprintf(fd, "\n              </documentation>\n            </annotation>\n");
            return 1;
          }
        }
      }
    }
  }
  fprintf(fd, "/>");
  if (!uflag)
  {
    if (scope)
      fprintf(fd, "<!-- %s::%s -->", type->name, member->sym->name);
    else
      fprintf(fd, "<!-- %s -->", member->sym->name);
  }
  fprintf(fd, "\n");
  return 0;
}

void
gen_text(FILE *fd, const char *s)
{
  for (; *s; s++)
  {
    if (*s == 10)
      fprintf(fd, "\n");
    else if (*s < 32 || *s >= 127)
      fprintf(fd, "&#%.2x;", (unsigned char)*s);
    else if (*s == '<')
      fprintf(fd, "&lt;");
    else if (*s == '>')
      fprintf(fd, "&gt;");
    else if (*s == '&')
      fprintf(fd, "&amp;");
    else if (*s == '\\' && (s[1] == '\n' || s[1] == '\r'))
      s += (s[1] == '\r');
    else if (*s == '\\')
      fprintf(fd, "\\\\");
    else
      fprintf(fd, "%c", *s);
  }
}

void
gen_nsmap(FILE *fd)
{
  Symbol *ns1;
  Service *sp;
  fprintf(fd, "{\n");
  for (ns1 = nslist; ns1; ns1 = ns1->next)
  {
    for (sp = services; sp; sp = sp->next)
      if (!tagcmp(sp->ns, ns1->name) && sp->URI)
        break;
    if (sp)
    {
      if (!strcmp(ns1->name, "SOAP-ENV"))
      {
        if (soap_version < 0)
          fprintf(fd, "        { \"SOAP-ENV\", NULL, NULL, NULL },\n");
        else
          fprintf(fd, "        { \"%s\", \"%s\", \"%s\", NULL },\n", ns_convert(ns1->name), sp->URI, sp->URI2 ? sp->URI2 : envURI);
      }
      else if (!strcmp(ns1->name, "SOAP-ENC"))
      {
        if (soap_version < 0)
          fprintf(fd, "        { \"SOAP-ENC\", NULL, NULL, NULL },\n");
        else
          fprintf(fd, "        { \"%s\", \"%s\", \"%s\", NULL },\n", ns_convert(ns1->name), sp->URI, sp->URI2 ? sp->URI2 : encURI);
      }
      else if (sp->URI2)
        fprintf(fd, "        { \"%s\", \"%s\", \"%s\", NULL },\n", ns_convert(ns1->name), sp->URI, sp->URI2);
      else
        fprintf(fd, "        { \"%s\", \"%s\", NULL, NULL },\n", ns_convert(ns1->name), sp->URI);
    }
    else if (!strcmp(ns1->name, "SOAP-ENV"))
    {
      if (soap_version < 0)
        fprintf(fd, "        { \"SOAP-ENV\", NULL, NULL, NULL },\n");
      else if (is_soap12(NULL))
        fprintf(fd, "        { \"SOAP-ENV\", \"%s\", \"http://schemas.xmlsoap.org/soap/envelope/\", NULL },\n", envURI);
      else
        fprintf(fd, "        { \"SOAP-ENV\", \"%s\", \"http://www.w3.org/*/soap-envelope\", NULL },\n", envURI);
    }
    else if (!strcmp(ns1->name, "SOAP-ENC"))
    {
      if (soap_version < 0)
        fprintf(fd, "        { \"SOAP-ENC\", NULL, NULL, NULL },\n");
      else if (is_soap12(NULL))
        fprintf(fd, "        { \"SOAP-ENC\", \"%s\", \"http://schemas.xmlsoap.org/soap/encoding/\", NULL },\n", encURI);
      else
        fprintf(fd, "        { \"SOAP-ENC\", \"%s\", \"http://www.w3.org/*/soap-encoding\", NULL },\n", encURI);
    }
    else if (!strcmp(ns1->name, "xsi"))
      fprintf(fd, "        { \"xsi\", \"%s\", \"http://www.w3.org/*/XMLSchema-instance\", NULL },\n", xsiURI);
    else if (!strcmp(ns1->name, "xsd"))
      fprintf(fd, "        { \"xsd\", \"%s\", \"http://www.w3.org/*/XMLSchema\", NULL },\n", xsdURI);
    else
      fprintf(fd, "        { \"%s\", \"%s/%s.xsd\", NULL, NULL },\n", ns_convert(ns1->name), tmpURI, ns_convert(ns1->name));
  }
  fprintf(fd, "        { NULL, NULL, NULL, NULL}\n    };\n");
}

void
gen_proxy(FILE *fd, Table *table, Symbol *ns, const char *name, const char *URL)
{
  Entry *p, *q, *r;
  Table *t, *output;
  Service *sp;
  int flag;
  const char *name1;
  name1 = ns_cname(name, NULL);
  for (sp = services; sp; sp = sp->next)
    if (!tagcmp(sp->ns, ns->name))
      break;
  fprintf(fd, "\n\n#ifndef %s%sProxy_H\n#define %s%sProxy_H\n#include \"%sH.h\"", prefix, name1, prefix, name1, prefix);
  if (nflag)
    fprintf(fd, "\nextern SOAP_NMAC struct Namespace %s_namespaces[];", prefix);
  if (namespaceid)
    fprintf(fd, "\n\nnamespace %s {", namespaceid);
  fprintf(fd, "\nclass %s\n{   public:\n\t/// Runtime engine context allocated in constructor\n\tstruct soap *soap;\n\t/// Endpoint URL of service '%s' (change as needed)\n\tconst char *soap_endpoint_url;\n\t/// Constructor allocates soap engine context, sets default endpoint URL, and sets namespace mapping table\n", name1, name);
  if (nflag)
    fprintf(fd, "\t%s() { soap = soap_new(); if (soap) soap->namespaces = %s_namespaces; soap_endpoint_url = \"%s\"; }\n", name1, prefix, URL);
  else
  {
    fprintf(fd, "\t%s()\n\t{ soap = soap_new(); soap_endpoint_url = \"%s\"; if (soap && !soap->namespaces) { static const struct Namespace namespaces[] = ", name1, URL);
    gen_nsmap(fd);
    fprintf(fd, "\tsoap->namespaces = namespaces; } }\n");
  }
  fprintf(fd, "\t/// Destructor deletes deserialized data and engine context\n\tvirtual ~%s() { if (soap) { soap_destroy(soap); soap_end(soap); soap_free(soap); } }\n", name1);
  fflush(fd);
  for (r = table->list; r; r = r->next)
  {
    if (r->info.typ->type == Tfun && !(r->info.sto & Sextern) && has_ns_eq(ns->name, r->sym->name))
    {
      p = entry(table, r->sym);
      if (p)
        q = (Entry*)p->info.typ->ref;
      else
      {
        fprintf(stderr, "Internal error: no table entry\n");
        return;
      }
      p = entry(classtable, r->sym);
      if (!p)
      {
        fprintf(stderr, "Internal error: no parameter table entry\n");
        return;
      }
      output = (Table*)p->info.typ->ref;
      fprintf(fd, "\t/// Invoke '%s' of service '%s' and return error code (or SOAP_OK)\n", ns_remove(r->sym->name), name);
      fprintf(fd, "\tvirtual int %s(", ident(r->sym->name));
      flag = 0;
      for (t = output; t; t = t->prev)
      {
        p = t->list;
        if (p)
        {
          fprintf(fd, "%s%s", c_storage(p->info.sto), c_type_id(p->info.typ, p->sym->name));
          for (p = p->next; p; p = p->next)
            fprintf(fd, ", %s%s", c_storage(p->info.sto), c_type_id(p->info.typ, p->sym->name));
          flag = 1;
        }
      }
      if (is_transient(q->info.typ))
        fprintf(fd, ") { return soap ? soap_send_%s(soap, soap_endpoint_url, NULL", ident(r->sym->name));
      else if (flag)
        fprintf(fd, ", %s%s) { return soap ? soap_call_%s(soap, soap_endpoint_url, NULL", c_storage(q->info.sto), c_type_id(q->info.typ, q->sym->name), ident(r->sym->name));
      else
        fprintf(fd, "%s%s) { return soap ? soap_call_%s(soap, soap_endpoint_url, NULL", c_storage(q->info.sto), c_type_id(q->info.typ, q->sym->name), ident(r->sym->name));
      for (t = output; t; t = t->prev)
        for (p = t->list; p; p = p->next)
          fprintf(fd, ", %s", ident(p->sym->name));
      if (is_transient(q->info.typ))
        fprintf(fd, ") : SOAP_EOM; }\n");
      else
        fprintf(fd, ", %s) : SOAP_EOM; }\n", ident(q->sym->name));
      fflush(fd);
    }
  }
  fprintf(fd, "};");
  if (namespaceid)
    fprintf(fd, "\n\n} // namespace %s\n", namespaceid);
  fprintf(fd, "\n#endif\n");
}

void
gen_object(FILE *fd, Table *table, const char *name)
{
  const char *name1;
  Entry *method;
  name1 = ns_cname(name, NULL);
  fprintf(fd, "\n\n#ifndef %s%sObject_H\n#define %s%sObject_H\n#include \"%sH.h\"", prefix, name1, prefix, name1, prefix);
  banner(fd, "Service Object");
  if (namespaceid)
    fprintf(fd, "\n\nnamespace %s {", namespaceid);
  fprintf(fd, "\nclass %sService : public soap\n{    public:", name1);
  fprintf(fd, "\n\t%sService()\n\t{ static const struct Namespace namespaces[] = ", name1);
  gen_nsmap(fd);
  fprintf(fd, "\n\tsoap_init(this); this->namespaces = namespaces; };");
  fprintf(fd, "\n\t/// Destructor deletes deserialized data and engine context");
  fprintf(fd, "\n\tvirtual ~%sService() { soap_destroy(this); soap_end(this); };", name1);
  fprintf(fd, "\n#ifndef WITH_NOIO");
  fprintf(fd, "\n\t/// Bind service to port (returns master socket or SOAP_INVALID_SOCKET)");
  fprintf(fd, "\n\tvirtual\tSOAP_SOCKET bind(const char *host, int port, int backlog) { return soap_bind(this, host, port, backlog); };");
  fprintf(fd, "\n\t/// Accept next request (returns socket or SOAP_INVALID_SOCKET)");
  fprintf(fd, "\n\tvirtual\tSOAP_SOCKET accept() { return soap_accept(this); };");
  fprintf(fd, "\n#if defined(WITH_OPENSSL) || defined(WITH_GNUTLS)");
  fprintf(fd, "\n\t/// Then accept SSL handshake, when SSL is used");
  fprintf(fd, "\n\tvirtual\tint ssl_accept() { return soap_ssl_accept(this); };");
  fprintf(fd, "\n#endif");
  fprintf(fd, "\n#endif");
  fprintf(fd, "\n\t/// Serve the pending request (returns SOAP_OK or error code)");
  if (nflag)
    fprintf(fd, "\n\tvirtual\tint serve() { return %s_serve(this); };", prefix);
  else
    fprintf(fd, "\n\tvirtual\tint serve() { return soap_serve(this); };");
  fprintf(fd, "\n};");
  banner(fd, "Service Operations (you should define these globally)");
  for (method = table->list; method; method = method->next)
  {
    if (method->info.typ->type == Tfun && !(method->info.sto & Sextern))
    {
      Entry *p, *q = entry(table, method->sym);
      Table *output;
      if (q)
        p = (Entry*)q->info.typ->ref;
      else
      {
        fprintf(stderr, "Internal error: no table entry\n");
        return;
      }
      q = entry(classtable, method->sym);
      output = (Table*)q->info.typ->ref;
      fprintf(fd, "\n\nSOAP_FMAC5 int SOAP_FMAC6 %s(struct soap*", ident(method->sym->name));
      gen_params(fd, output, p, 1);
      fprintf(fd, ";");
    }
  }
  if (namespaceid)
    fprintf(fd, "\n\n} // namespace %s\n", namespaceid);
  fprintf(fd, "\n\n#endif\n");
}

void
gen_proxy_header(FILE *fd, Table *table, Symbol *ns, const char *name)
{
  static int first_class = 1;
  Entry *p, *method;
  Table *t;
  if (fd != freport)
    fprintf(fd, "\n\n#ifndef %s%s_H\n#define %s%s_H\n#include \"%sH.h\"", prefix, name, prefix, name, prefix);
  else
  {
    Service *sp;
    if (first_class)
      fprintf(fd, "## Web Client Proxy Class %s {#doc-client}\n\n", name);
    else
      fprintf(fd, "## Web Client Proxy Class %s {#%s}\n\n", name, name);
    first_class = 0;
    for (sp = services; sp; sp = sp->next)
    {
      if (sp->documentation)
      {
        gen_text(freport, sp->documentation);
        fprintf(freport, "\n\n");
      }
    }
    fprintf(fd, "This client proxy class is declared in [%s](%s) and defined in [%s](%s):", soapProxyH, pathsoapProxyH, soapProxyC, pathsoapProxyC);
  }
  if (namespaceid)
    fprintf(fd, "\n\n    namespace %s {", namespaceid);
  if (iflag)
    fprintf(fd, "\n\n    class SOAP_CMAC %s : public soap {\n      public:", name);
  else
  {
    fprintf(fd, "\n\n    class SOAP_CMAC %s {\n      public:", name);
    fprintf(fd, "\n        /// Context to manage proxy IO and data\n        struct soap *soap;\n        /// flag indicating that this context is owned by this proxy and should be deleted by the destructor\n        bool soap_own;");
  }
  fprintf(fd, "\n        /// Endpoint URL of service '%s' (change as needed)", name);
  fprintf(fd, "\n        const char *soap_endpoint;");
  fprintf(fd, "\n        /// Variables globally declared in %s, if any", filename);
  for (p = table->list; p; p = p->next)
    if (p->info.typ->type != Tfun && !(p->info.sto & Sstatic))
      fprintf(fd, "\n        %s%s;", c_storage(p->info.sto), c_type_id(p->info.typ, p->sym->name));
  fprintf(fd, "\n        /// Construct a proxy with new managing context");
  fprintf(fd, "\n        %s();", name);
  fprintf(fd, "\n        /// Copy constructor");
  fprintf(fd, "\n        %s(const %s& rhs);", name, name);
  if (iflag)
  {
    fprintf(fd, "\n        /// Construct proxy given a managing context");
    fprintf(fd, "\n        %s(const struct soap&);", name);
    fprintf(fd, "\n        /// Construct proxy given a managing context and endpoint URL");
    fprintf(fd, "\n        %s(const struct soap&, const char *soap_endpoint_url);", name);
  }
  else
  {
    fprintf(fd, "\n        /// Construct proxy given a shared managing context");
    fprintf(fd, "\n        %s(struct soap*);", name);
    fprintf(fd, "\n        /// Construct proxy given a shared managing context and endpoint URL");
    fprintf(fd, "\n        %s(struct soap*, const char *soap_endpoint_url);", name);
  }
  fprintf(fd, "\n        /// Constructor taking an endpoint URL");
  fprintf(fd, "\n        %s(const char *soap_endpoint_url);", name);
  fprintf(fd, "\n        /// Constructor taking input and output mode flags for the new managing context");
  fprintf(fd, "\n        %s(soap_mode iomode);", name);
  fprintf(fd, "\n        /// Constructor taking endpoint URL and input and output mode flags for the new managing context");
  fprintf(fd, "\n        %s(const char *soap_endpoint_url, soap_mode iomode);", name);
  fprintf(fd, "\n        /// Constructor taking input and output mode flags for the new managing context");
  fprintf(fd, "\n        %s(soap_mode imode, soap_mode omode);", name);
  if (iflag)
    fprintf(fd, "\n        /// Destructor deletes deserialized data and its managing context");
  else
    fprintf(fd, "\n        /// Destructor deletes deserialized data and its managing context, when the context was allocated by the constructor");
  fprintf(fd, "\n        virtual ~%s();", name);
  fprintf(fd, "\n        /// Initializer used by constructors");
  fprintf(fd, "\n        virtual void %s_init(soap_mode imode, soap_mode omode);", name);
  fprintf(fd, "\n        /// Return a copy that has a new managing context with the same engine state");
  fprintf(fd, "\n        virtual %s *copy();", name);
  fprintf(fd, "\n        /// Copy assignment");
  fprintf(fd, "\n        %s& operator=(const %s&);", name, name);
  fprintf(fd, "\n        /// Delete all deserialized data (uses soap_destroy() and soap_end())");
  fprintf(fd, "\n        virtual void destroy();");
  fprintf(fd, "\n        /// Delete all deserialized data and reset to default");
  fprintf(fd, "\n        virtual void reset();");
  fprintf(fd, "\n        /// Disables and removes SOAP Header from message by setting soap->header = NULL");
  fprintf(fd, "\n        virtual void soap_noheader();");
  if (!namespaceid)
  {
    p = entry(classtable, lookup("SOAP_ENV__Header"));
    if (p)
    {
      t = (Table*)p->info.typ->ref;
      if (t && t->list && !is_void(t->list->info.typ))
      {
        fprintf(fd, "\n        /// Add SOAP Header to message");
        fprintf(fd, "\n        virtual void soap_header(");
        gen_params_ref(fd, t, NULL, 0);
        fprintf(fd, ";");
      }
    }
  }
  else
    fprintf(fd, "\n        // virtual void soap_header(...) method removed due to option -q or -Q");
  fprintf(fd, "\n        /// Get SOAP Header structure (i.e. soap->header, which is NULL when absent)");
  fprintf(fd, "\n        virtual ::SOAP_ENV__Header *soap_header();");
  fprintf(fd, "\n        /// Get SOAP Fault structure (i.e. soap->fault, which is NULL when absent)");
  fprintf(fd, "\n        virtual ::SOAP_ENV__Fault *soap_fault();");
  fprintf(fd, "\n        /// Get SOAP Fault subcode QName string (NULL when absent)");
  fprintf(fd, "\n        virtual const char *soap_fault_subcode();");
  fprintf(fd, "\n        /// Get SOAP Fault string/reason (NULL when absent)");
  fprintf(fd, "\n        virtual const char *soap_fault_string();");
  fprintf(fd, "\n        /// Get SOAP Fault detail XML string (NULL when absent)");
  fprintf(fd, "\n        virtual const char *soap_fault_detail();");
  fprintf(fd, "\n        /// Close connection (normally automatic, except for send_X ops)");
  fprintf(fd, "\n        virtual int soap_close_socket();");
  fprintf(fd, "\n        /// Force close connection (can kill a thread blocked on IO)");
  fprintf(fd, "\n        virtual int soap_force_close_socket();");
  fprintf(fd, "\n        /// Print fault");
  fprintf(fd, "\n        virtual void soap_print_fault(FILE*);");
  fprintf(fd, "\n    #ifndef WITH_LEAN");
  fprintf(fd, "\n    #ifndef WITH_COMPAT");
  fprintf(fd, "\n        /// Print fault to stream");
  fprintf(fd, "\n        virtual void soap_stream_fault(std::ostream&);");
  fprintf(fd, "\n    #endif");
  fprintf(fd, "\n        /// Write fault to buffer");
  fprintf(fd, "\n        virtual char *soap_sprint_fault(char *buf, size_t len);");
  fprintf(fd, "\n    #endif");
  for (method = table->list; method; method = method->next)
    if (method->info.typ->type == Tfun && !(method->info.sto & Sextern) && has_ns_eq(ns->name, method->sym->name))
      gen_method(fd, method, 0);
  fprintf(fd, "\n    };");
  if (namespaceid)
    fprintf(fd, "\n\n    } // namespace %s\n", namespaceid);
  if (fd != freport)
    fprintf(fd, "\n#endif\n");
  else
  {
    fprintf(freport, "\n\n");
    for (method = table->list; method; method = method->next)
    {
      Entry *result, *p;
      result = (Entry*)method->info.typ->ref;
      p = entry(classtable, method->sym);
      if (!p)
        execerror("no table entry");
      if (method->info.typ->type == Tfun && !(method->info.sto & Sextern) && has_ns_eq(ns->name, method->sym->name))
      {
        gen_report_operation(name, method, 0);
        gen_method(freport, method, 0);
        fprintf(freport, "\n\nwhere:\n\n- `const char *soap_endpoint_url` is the endpoint URL or NULL to use the default endpoint\n- `const char *soap_action` is the SOAP action header or NULL to use the default action\n");
        gen_report_params(p, result, 0);
        if (!is_transient(result->info.typ))
          fprintf(freport, "The `%s` method sends the request message and receives the response message, assigning the last parameter `%s` the response value received. The `send_%s` method sends the request message and the `recv_%s` method receives the response message asynchronously. These methods return `SOAP_OK` or an error code.\n\n", ns_remove(method->sym->name), ident(result->sym->name), ns_remove(method->sym->name), ns_remove(method->sym->name));
        else
          fprintf(freport, "The `send_%s` method sends the one-way request message and the `recv_%s` method receives the one-way request message. The `int soap_recv_empty_response(struct soap *soap)` function should be called after `send_%s` when communicating over HTTP to receive the HTTP acknowledgment.\n\n", ns_remove(method->sym->name), ns_remove(method->sym->name), ns_remove(method->sym->name));
        gen_report_hr();
      }
    }
  }
}

void
gen_proxy_code(FILE *fd, Table *table, Symbol *ns, const char *name)
{
  Entry *p, *method, *param;
  Table *t;
  const char *soap;
  if (iflag)
    soap = "this";
  else
    soap = "this->soap";
  fprintf(fd, "\n\n#include \"%s%s.h\"", prefix, name);
  if (namespaceid)
    fprintf(fd, "\n\nnamespace %s {", namespaceid);
  if (iflag)
  {
    fprintf(fd, "\n\n%s::%s() : soap(SOAP_IO_DEFAULT)\n{\t%s_init(SOAP_IO_DEFAULT, SOAP_IO_DEFAULT);\n}", name, name, name);
    fprintf(fd, "\n\n%s::%s(const %s& rhs)\n{\tsoap_copy_context(this, &rhs);\n\tthis->soap_endpoint = rhs.soap_endpoint;\n}", name, name, name);
    fprintf(fd, "\n\n%s::%s(const struct soap &_soap) : soap(_soap)\n{ }", name, name);
    fprintf(fd, "\n\n%s::%s(const struct soap &_soap, const char *soap_endpoint_url) : soap(_soap)\n{\n\tsoap_endpoint = soap_endpoint_url;\n}", name, name);
    fprintf(fd, "\n\n%s::%s(const char *soap_endpoint_url) : soap(SOAP_IO_DEFAULT)\n{\t%s_init(SOAP_IO_DEFAULT, SOAP_IO_DEFAULT);\n\tsoap_endpoint = soap_endpoint_url;\n}", name, name, name);
    fprintf(fd, "\n\n%s::%s(soap_mode iomode) : soap(iomode)\n{\t%s_init(iomode, iomode);\n}", name, name, name);
    fprintf(fd, "\n\n%s::%s(const char *soap_endpoint_url, soap_mode iomode) : soap(iomode)\n{\t%s_init(iomode, iomode);\n\tsoap_endpoint = soap_endpoint_url;\n}", name, name, name);
    fprintf(fd, "\n\n%s::%s(soap_mode imode, soap_mode omode) : soap(imode, omode)\n{\t%s_init(imode, omode);\n}", name, name, name);
    fprintf(fd, "\n\n%s::~%s()\n{\n\tthis->destroy();\n}", name, name);
  }
  else
  {
    fprintf(fd, "\n\n%s::%s()\n{\tthis->soap = soap_new();\n\tthis->soap_own = true;\n\t%s_init(SOAP_IO_DEFAULT, SOAP_IO_DEFAULT);\n}", name, name, name);
    fprintf(fd, "\n\n%s::%s(const %s& rhs)\n{\tthis->soap = rhs.soap;\n\tthis->soap_own = false;\n\tthis->soap_endpoint = rhs.soap_endpoint;\n}", name, name, name);
    fprintf(fd, "\n\n%s::%s(struct soap *_soap)\n{\tthis->soap = _soap;\n\tthis->soap_own = false;\n\t%s_init(_soap->imode, _soap->omode);\n}", name, name, name);
    fprintf(fd, "\n\n%s::%s(struct soap *_soap, const char *soap_endpoint_url)\n{\tthis->soap = _soap;\n\tthis->soap_own = false;\n\t%s_init(_soap->imode, _soap->omode);\n\tsoap_endpoint = soap_endpoint_url;\n}", name, name, name);
    fprintf(fd, "\n\n%s::%s(const char *soap_endpoint_url)\n{\tthis->soap = soap_new();\n\tthis->soap_own = true;\n\t%s_init(SOAP_IO_DEFAULT, SOAP_IO_DEFAULT);\n\tsoap_endpoint = soap_endpoint_url;\n}", name, name, name);
    fprintf(fd, "\n\n%s::%s(soap_mode iomode)\n{\tthis->soap = soap_new();\n\tthis->soap_own = true;\n\t%s_init(iomode, iomode);\n}", name, name, name);
    fprintf(fd, "\n\n%s::%s(const char *soap_endpoint_url, soap_mode iomode)\n{\tthis->soap = soap_new();\n\tthis->soap_own = true;\n\t%s_init(iomode, iomode);\n\tsoap_endpoint = soap_endpoint_url;\n}", name, name, name);
    fprintf(fd, "\n\n%s::%s(soap_mode imode, soap_mode omode)\n{\tthis->soap = soap_new();\n\tthis->soap_own = true;\n\t%s_init(imode, omode);\n}", name, name, name);
    fprintf(fd, "\n\n%s::~%s()\n{\tif (this->soap_own)\n\t{\tthis->destroy();\n\t\tsoap_free(this->soap);\n\t}\n}", name, name);
  }
  fprintf(fd, "\n\nvoid %s::%s_init(soap_mode imode, soap_mode omode)\n{\tsoap_imode(%s, imode);\n\tsoap_omode(%s, omode);\n\tsoap_endpoint = NULL;\n\tstatic const struct Namespace namespaces[] = ", name, name, soap, soap);
  gen_nsmap(fd);
  fprintf(fd, "\tsoap_set_namespaces(%s, namespaces);\n}", soap);
  if (iflag)
  {
    fprintf(fd, "\n\n%s *%s::copy()\n{\t%s *dup = SOAP_NEW_UNMANAGED(%s(*(struct soap*)%s));\n\treturn dup;\n}", name, name, name, name, soap);
    fprintf(fd, "\n\n%s& %s::operator=(const %s& rhs)\n{\tsoap_done(this);\n\tsoap_copy_context(this, &rhs);\n\tthis->soap_endpoint = rhs.soap_endpoint;\n\treturn *this;\n}", name, name, name);
  }
  else
  {
    fprintf(fd, "\n\n%s *%s::copy()\n{\t%s *dup = SOAP_NEW_UNMANAGED(%s);\n\tif (dup)\n\t{\tsoap_done(dup->soap);\n\t\tsoap_copy_context(dup->soap, this->soap);\n\t}\n\treturn dup;\n}", name, name, name, name);
    fprintf(fd, "\n\n%s& %s::operator=(const %s& rhs)\n{\tif (this->soap != rhs.soap)\n\t{\tif (this->soap_own)\n\t\t\tsoap_free(this->soap);\n\t\tthis->soap = rhs.soap;\n\t\tthis->soap_own = false;\n\t\tthis->soap_endpoint = rhs.soap_endpoint;\n\t}\n\treturn *this;\n}", name, name, name);
  }
  fprintf(fd, "\n\nvoid %s::destroy()\n{\tsoap_destroy(%s);\n\tsoap_end(%s);\n}", name, soap, soap);
  fprintf(fd, "\n\nvoid %s::reset()\n{\tthis->destroy();\n\tsoap_done(%s);\n\tsoap_initialize(%s);\n\t%s_init(SOAP_IO_DEFAULT, SOAP_IO_DEFAULT);\n}", name, soap, soap, name);
  fprintf(fd, "\n\nvoid %s::soap_noheader()\n{\t%s->header = NULL;\n}", name, soap);
  if (!namespaceid)
  {
    p = entry(classtable, lookup("SOAP_ENV__Header"));
    if (p)
    {
      t = (Table*)p->info.typ->ref;
      if (t && t->list && !is_void(t->list->info.typ))
      {
        fprintf(fd, "\n\nvoid %s::soap_header(", name);
        gen_params_ref(fd, t, NULL, 0);
        fprintf(fd, "\n{\n\t::soap_header(%s);", soap);
        for (param = t->list; param; param = param->next)
        {
          if (namespaceid)
            fprintf(fd, "\n\t((%s::SOAP_ENV__Header*)%s->header)->%s = %s;", namespaceid, soap, ident(param->sym->name), ident(param->sym->name));
          else
            fprintf(fd, "\n\t%s->header->%s = %s;", soap, ident(param->sym->name), ident(param->sym->name));
        }
        fprintf(fd, "\n}");
      }
    }
  }
  fprintf(fd, "\n\n::SOAP_ENV__Header *%s::soap_header()\n{\treturn %s->header;\n}", name, soap);
  fprintf(fd, "\n\n::SOAP_ENV__Fault *%s::soap_fault()\n{\treturn %s->fault;\n}", name, soap);
  fprintf(fd, "\n\nconst char *%s::soap_fault_subcode()\n{\treturn ::soap_fault_subcode(%s);\n}", name, soap);
  fprintf(fd, "\n\nconst char *%s::soap_fault_string()\n{\treturn ::soap_fault_string(%s);\n}", name, soap);
  fprintf(fd, "\n\nconst char *%s::soap_fault_detail()\n{\treturn ::soap_fault_detail(%s);\n}", name, soap);
  fprintf(fd, "\n\nint %s::soap_close_socket()\n{\treturn ::soap_closesock(%s);\n}", name, soap);
  fprintf(fd, "\n\nint %s::soap_force_close_socket()\n{\treturn ::soap_force_closesock(%s);\n}", name, soap);
  fprintf(fd, "\n\nvoid %s::soap_print_fault(FILE *fd)\n{\t::soap_print_fault(%s, fd);\n}", name, soap);
  fprintf(fd, "\n\n#ifndef WITH_LEAN\n#ifndef WITH_COMPAT\nvoid %s::soap_stream_fault(std::ostream& os)\n{\t::soap_stream_fault(%s, os);\n}\n#endif", name, soap);
  fprintf(fd, "\n\nchar *%s::soap_sprint_fault(char *buf, size_t len)\n{\treturn ::soap_sprint_fault(%s, buf, len);\n}\n#endif", name, soap);
  for (method = table->list; method; method = method->next)
    if (method->info.typ->type == Tfun && !(method->info.sto & Sextern) && !is_imported(method->info.typ) && has_ns_eq(ns->name, method->sym->name))
      gen_call_method(fd, method, name);
  if (namespaceid)
    fprintf(fd, "\n\n} // namespace %s\n", namespaceid);
  fprintf(fd, "\n/* End of client proxy code */\n");
}

void
gen_object_header(FILE *fd, Table *table, Symbol *ns, const char *name)
{
  static int first_class = 1;
  Entry *p, *method;
  Table *t;
  if (fd != freport)
    fprintf(fd, "\n\n#ifndef %s%s_H\n#define %s%s_H\n#include \"%sH.h\"", prefix, name, prefix, name, prefix);
  else
  {
    Service *sp;
    if (first_class)
      fprintf(fd, "## Web Service Class %s {#doc-server}\n\n", name);
    else
      fprintf(fd, "## Web Service Class %s {#%s}\n\n", name, name);
    first_class = 0;
    for (sp = services; sp; sp = sp->next)
    {
      if (sp->documentation)
      {
        gen_text(freport, sp->documentation);
        fprintf(freport, "\n\n");
      }
    }
    fprintf(fd, "This service class is declared in [%s](%s) and defined in [%s](%s):", soapServiceH, pathsoapServiceH, soapServiceC, pathsoapServiceC);
  }
  if (namespaceid)
    fprintf(fd, "\n\n    namespace %s {", namespaceid);
  if (iflag)
    fprintf(fd, "\n\n    class SOAP_CMAC %s : public soap {\n      public:", name);
  else
  {
    fprintf(fd, "\n\n    class SOAP_CMAC %s {\n      public:", name);
    fprintf(fd, "\n        /// Context to manage service IO and data\n        struct soap *soap;\n        /// flag indicating that this context is owned by this service and should be deleted by the destructor\n        bool soap_own;");
  }
  fprintf(fd, "\n        /// Variables globally declared in %s, if any", filename);
  for (p = table->list; p; p = p->next)
    if (p->info.typ->type != Tfun && !(p->info.sto & Sstatic))
      fprintf(fd, "\n        %s%s;", c_storage(p->info.sto), c_type_id(p->info.typ, p->sym->name));
  fprintf(fd, "\n        /// Construct a service with new managing context");
  fprintf(fd, "\n        %s();", name);
  fprintf(fd, "\n        /// Copy constructor");
  fprintf(fd, "\n        %s(const %s&);", name, name);
  if (iflag)
  {
    fprintf(fd, "\n        /// Construct service given a managing context");
    fprintf(fd, "\n        %s(const struct soap&);", name);
    fprintf(fd, "\n        /// Construct service given a managing context and endpoint");
    fprintf(fd, "\n        %s(const struct soap&, const char *soap_endpoint_url);", name);
  }
  else
  {
    fprintf(fd, "\n        /// Construct service given a shared managing context");
    fprintf(fd, "\n        %s(struct soap*);", name);
  }
  fprintf(fd, "\n        /// Constructor taking input+output mode flags for the new managing context");
  fprintf(fd, "\n        %s(soap_mode iomode);", name);
  fprintf(fd, "\n        /// Constructor taking input and output mode flags for the new managing context");
  fprintf(fd, "\n        %s(soap_mode imode, soap_mode omode);", name);
  if (iflag)
    fprintf(fd, "\n        /// Destructor deletes deserialized data and its managing context");
  else
    fprintf(fd, "\n        /// Destructor deletes deserialized data and its managing context, when the context was allocated by the constructor");
  fprintf(fd, "\n        virtual ~%s();", name);
  fprintf(fd, "\n        /// Delete all deserialized data (with soap_destroy() and soap_end())");
  fprintf(fd, "\n        virtual void destroy();");
  fprintf(fd, "\n        /// Delete all deserialized data and reset to defaults");
  fprintf(fd, "\n        virtual void reset();");
  fprintf(fd, "\n        /// Initializer used by constructors");
  fprintf(fd, "\n        virtual void %s_init(soap_mode imode, soap_mode omode);", name);
  fprintf(fd, "\n        /// Return a copy that has a new managing context with the same engine state");
  fprintf(fd, "\n        virtual %s *copy() SOAP_PURE_VIRTUAL_COPY;", name);
  fprintf(fd, "\n        /// Copy assignment");
  fprintf(fd, "\n        %s& operator=(const %s&);", name, name);
  fprintf(fd, "\n        /// Close connection (normally automatic)");
  fprintf(fd, "\n        virtual int soap_close_socket();");
  fprintf(fd, "\n        /// Force close connection (can kill a thread blocked on IO)");
  fprintf(fd, "\n        virtual int soap_force_close_socket();");
  fprintf(fd, "\n        /// Return sender-related fault to sender");
  fprintf(fd, "\n        virtual int soap_senderfault(const char *string, const char *detailXML);");
  fprintf(fd, "\n        /// Return sender-related fault with SOAP 1.2 subcode to sender");
  fprintf(fd, "\n        virtual int soap_senderfault(const char *subcodeQName, const char *string, const char *detailXML);");
  fprintf(fd, "\n        /// Return receiver-related fault to sender");
  fprintf(fd, "\n        virtual int soap_receiverfault(const char *string, const char *detailXML);");
  fprintf(fd, "\n        /// Return receiver-related fault with SOAP 1.2 subcode to sender");
  fprintf(fd, "\n        virtual int soap_receiverfault(const char *subcodeQName, const char *string, const char *detailXML);");
  fprintf(fd, "\n        /// Print fault");
  fprintf(fd, "\n        virtual void soap_print_fault(FILE*);");
  fprintf(fd, "\n    #ifndef WITH_LEAN");
  fprintf(fd, "\n    #ifndef WITH_COMPAT");
  fprintf(fd, "\n        /// Print fault to stream");
  fprintf(fd, "\n        virtual void soap_stream_fault(std::ostream&);");
  fprintf(fd, "\n    #endif");
  fprintf(fd, "\n        /// Write fault to buffer");
  fprintf(fd, "\n        virtual char *soap_sprint_fault(char *buf, size_t len);");
  fprintf(fd, "\n    #endif");
  fprintf(fd, "\n        /// Disables and removes SOAP Header from message by setting soap->header = NULL");
  fprintf(fd, "\n        virtual void soap_noheader();");
  if (!namespaceid)
  {
    p = entry(classtable, lookup("SOAP_ENV__Header"));
    if (p)
    {
      t = (Table*)p->info.typ->ref;
      if (t && t->list && !is_void(t->list->info.typ))
      {
        fprintf(fd, "\n        /// Add SOAP Header to message");
        fprintf(fd, "\n        virtual void soap_header(");
        gen_params_ref(fd, t, NULL, 0);
        fprintf(fd, ";");
      }
    }
  }
  else
    fprintf(fd, "\n        // virtual void soap_header(...) method removed due to option -q or -Q");
  fprintf(fd, "\n        /// Get SOAP Header structure (i.e. soap->header, which is NULL when absent)");
  fprintf(fd, "\n        virtual ::SOAP_ENV__Header *soap_header();");
  fprintf(fd, "\n    #ifndef WITH_NOIO");
  fprintf(fd, "\n        /// Run simple single-thread (iterative, non-SSL) service on port until a connection error occurs (returns SOAP_OK or error code), use this->bind_flag = SO_REUSEADDR to rebind for immediate rerun");
  fprintf(fd, "\n        virtual int run(int port, int backlog = 1);");
  fprintf(fd, "\n    #if defined(WITH_OPENSSL) || defined(WITH_GNUTLS)");
  fprintf(fd, "\n        /// Run simple single-thread SSL service on port until a connection error occurs (returns SOAP_OK or error code), use this->bind_flag = SO_REUSEADDR to rebind for immediate rerun");
  fprintf(fd, "\n        virtual int ssl_run(int port, int backlog = 1);");
  fprintf(fd, "\n    #endif");
  fprintf(fd, "\n        /// Bind service to port (returns master socket or SOAP_INVALID_SOCKET upon error)");
  fprintf(fd, "\n        virtual SOAP_SOCKET bind(const char *host, int port, int backlog);");
  fprintf(fd, "\n        /// Accept next request (returns socket or SOAP_INVALID_SOCKET upon error)");
  fprintf(fd, "\n        virtual SOAP_SOCKET accept();");
  fprintf(fd, "\n    #if defined(WITH_OPENSSL) || defined(WITH_GNUTLS)");
  fprintf(fd, "\n        /// When SSL is used, after accept() should perform and accept SSL handshake");
  fprintf(fd, "\n        virtual int ssl_accept();");
  fprintf(fd, "\n    #endif");
  fprintf(fd, "\n    #endif");
  fprintf(fd, "\n        /// After accept() serve the pending request (returns SOAP_OK or error code)");
  fprintf(fd, "\n        virtual int serve();");
  fprintf(fd, "\n        /// Used by serve() to dispatch a pending request (returns SOAP_OK or error code)");
  fprintf(fd, "\n        virtual int dispatch();");
  if (jflag)
    fprintf(fd, "\n        virtual int dispatch(struct soap *soap);");
  fprintf(fd, "\n        //\n        // Service operations are listed below: you should define these\n        // Note: compile with -DWITH_PURE_VIRTUAL to declare pure virtual methods");
  for (method = table->list; method; method = method->next)
    if (method->info.typ->type == Tfun && !(method->info.sto & Sextern) && has_ns_eq(ns->name, method->sym->name))
      gen_method(fd, method, 1);
  fprintf(fd, "\n    };");
  if (namespaceid)
    fprintf(fd, "\n\n    } // namespace %s\n", namespaceid);
  if (fd != freport)
    fprintf(fd, "\n#endif\n");
  else
  {
    fprintf(freport, "\n\nUse the service request dispatcher to accept and process service requests:\n\n");
    fprintf(freport, "- `int serve()` serves requests by calling one of the service methods that matches the request.  Returns `SOAP_OK` or an error code.  This function supports CGI by accepting a request on stdin and sending the response to stdout, and FastCGI.  To serve over HTTP(S), use the following functions to establish a connection:\n\n");
    fprintf(freport, "- `SOAP_SOCKET bind(const char *host, int port, int backlog)` returns master socket bound to port (and restricted to host name, if not NULL) or `SOAP_INVALID_SOCKET` upon error\n");
    fprintf(freport, "- `SOAP_SOCKET accept()` accepts connection and returns socket when accepted, or `SOAP_INVALID_SOCKET` upon error\n");
    fprintf(freport, "- `int soap_ssl_accept()` performs SSL handshake and returns `SOAP_OK` when successful or an error code, invoke this method after `accept()` to accept SSL/TLS connection\n\n");
    for (method = table->list; method; method = method->next)
    {
      Entry *result, *p;
      result = (Entry*)method->info.typ->ref;
      p = entry(classtable, method->sym);
      if (!p)
        execerror("no table entry");
      if (method->info.typ->type == Tfun && !(method->info.sto & Sextern) && has_ns_eq(ns->name, method->sym->name))
      {
        gen_report_operation(name, method, 1);
        gen_method(freport, method, 1);
        fprintf(freport, "\n\nwhere:\n\n");
        gen_report_params(p, result, 1);
        if (!is_transient(result->info.typ))
          fprintf(freport, "This service method should be implemented as part of the service back-end code and return `SOAP_OK` and set the last parameter `%s` to the result, or return an error code\n\n", ident(result->sym->name));
        else
          fprintf(freport, "This service method should be implemented as part of the service back-end code and call `int %s::send_%s_empty_response(int httpcode)` with a HTTP status or error code (200 to 599) to return, when communicating over HTTP to return a HTTP header.\n\n", name, ns_remove(method->sym->name));
        gen_report_hr();
      }
    }
  }
}

void
gen_object_code(FILE *fd, Table *table, Symbol *ns, const char *name)
{
  Entry *p, *method, *catch_method, *param;
  Table *t;
  const char *soap;
  if (iflag)
    soap = "this";
  else
    soap = "this->soap";
  fprintf(fd, "\n\n#include \"%s%s.h\"", prefix, name);
  if (namespaceid)
    fprintf(fd, "\n\nnamespace %s {", namespaceid);
  if (iflag)
  {
    fprintf(fd, "\n\n%s::%s() : soap(SOAP_IO_DEFAULT)\n{\t%s_init(SOAP_IO_DEFAULT, SOAP_IO_DEFAULT);\n}", name, name, name);
    fprintf(fd, "\n\n%s::%s(const %s& rhs)\n{\tsoap_copy_context(this, &rhs);\n}", name, name, name);
    fprintf(fd, "\n\n%s::%s(const struct soap &_soap) : soap(_soap)\n{ }", name, name);
    fprintf(fd, "\n\n%s::%s(soap_mode iomode) : soap(iomode)\n{\t%s_init(iomode, iomode);\n}", name, name, name);
    fprintf(fd, "\n\n%s::%s(soap_mode imode, soap_mode omode) : soap(imode, omode)\n{\t%s_init(imode, omode);\n}", name, name, name);
    fprintf(fd, "\n\n%s::~%s()\n{\n\tthis->destroy();\n}", name, name);
  }
  else
  {
    fprintf(fd, "\n\n%s::%s()\n{\tthis->soap = soap_new();\n\tthis->soap_own = true;\n\t%s_init(SOAP_IO_DEFAULT, SOAP_IO_DEFAULT);\n}", name, name, name);
    fprintf(fd, "\n\n%s::%s(const %s& rhs)\n{\tthis->soap = rhs.soap;\n\tthis->soap_own = false;\n}", name, name, name);
    fprintf(fd, "\n\n%s::%s(struct soap *_soap)\n{\tthis->soap = _soap;\n\tthis->soap_own = false;\n\t%s_init(_soap->imode, _soap->omode);\n}", name, name, name);
    fprintf(fd, "\n\n%s::%s(soap_mode iomode)\n{\tthis->soap = soap_new();\n\tthis->soap_own = true;\n\t%s_init(iomode, iomode);\n}", name, name, name);
    fprintf(fd, "\n\n%s::%s(soap_mode imode, soap_mode omode)\n{\tthis->soap = soap_new();\n\tthis->soap_own = true;\n\t%s_init(imode, omode);\n}", name, name, name);
    fprintf(fd, "\n\n%s::~%s()\n{\tif (this->soap_own)\n\t{\tthis->destroy();\n\t\tsoap_free(this->soap);\n\t}\n}", name, name);
  }
  fprintf(fd, "\n\nvoid %s::%s_init(soap_mode imode, soap_mode omode)\n{\tsoap_imode(%s, imode);\n\tsoap_omode(%s, omode);\n\tstatic const struct Namespace namespaces[] = ", name, name, soap, soap);
  gen_nsmap(fd);
  fprintf(fd, "\tsoap_set_namespaces(%s, namespaces);\n}", soap);
  fprintf(fd, "\n\nvoid %s::destroy()\n{\tsoap_destroy(%s);\n\tsoap_end(%s);\n}", name, soap, soap);
  fprintf(fd, "\n\nvoid %s::reset()\n{\tthis->destroy();\n\tsoap_done(%s);\n\tsoap_initialize(%s);\n\t%s_init(SOAP_IO_DEFAULT, SOAP_IO_DEFAULT);\n}", name, soap, soap, name);
  if (iflag)
  {
    fprintf(fd, "\n\n#ifndef WITH_PURE_VIRTUAL\n%s *%s::copy()\n{\t%s *dup = SOAP_NEW_UNMANAGED(%s(*(struct soap*)%s));\n\treturn dup;\n}\n#endif", name, name, name, name, soap);
    fprintf(fd, "\n\n%s& %s::operator=(const %s& rhs)\n{\tsoap_done(this);\n\tsoap_copy_context(this, &rhs);\n\treturn *this;\n}", name, name, name);
  }
  else
  {
    fprintf(fd, "\n\n#ifndef WITH_PURE_VIRTUAL\n%s *%s::copy()\n{\t%s *dup = SOAP_NEW_UNMANAGED(%s);\n\tif (dup)\n\t{\tsoap_done(dup->soap);\n\t\tsoap_copy_context(dup->soap, this->soap);\n\t}\n\treturn dup;\n}\n#endif", name, name, name, name);
    fprintf(fd, "\n\n%s& %s::operator=(const %s& rhs)\n{\tif (this->soap != rhs.soap)\n\t{\tif (this->soap_own)\n\t\t\tsoap_free(this->soap);\n\t\tthis->soap = rhs.soap;\n\t\tthis->soap_own = false;\n\t}\n\treturn *this;\n}", name, name, name);
  }
  fprintf(fd, "\n\nint %s::soap_close_socket()\n{\treturn soap_closesock(%s);\n}", name, soap);
  fprintf(fd, "\n\nint %s::soap_force_close_socket()\n{\treturn soap_force_closesock(%s);\n}", name, soap);
  fprintf(fd, "\n\nint %s::soap_senderfault(const char *string, const char *detailXML)\n{\treturn ::soap_sender_fault(%s, string, detailXML);\n}", name, soap);
  fprintf(fd, "\n\nint %s::soap_senderfault(const char *subcodeQName, const char *string, const char *detailXML)\n{\treturn ::soap_sender_fault_subcode(%s, subcodeQName, string, detailXML);\n}", name, soap);
  fprintf(fd, "\n\nint %s::soap_receiverfault(const char *string, const char *detailXML)\n{\treturn ::soap_receiver_fault(%s, string, detailXML);\n}", name, soap);
  fprintf(fd, "\n\nint %s::soap_receiverfault(const char *subcodeQName, const char *string, const char *detailXML)\n{\treturn ::soap_receiver_fault_subcode(%s, subcodeQName, string, detailXML);\n}", name, soap);
  fprintf(fd, "\n\nvoid %s::soap_print_fault(FILE *fd)\n{\t::soap_print_fault(%s, fd);\n}", name, soap);
  fprintf(fd, "\n\n#ifndef WITH_LEAN\n#ifndef WITH_COMPAT\nvoid %s::soap_stream_fault(std::ostream& os)\n{\t::soap_stream_fault(%s, os);\n}\n#endif", name, soap);
  fprintf(fd, "\n\nchar *%s::soap_sprint_fault(char *buf, size_t len)\n{\treturn ::soap_sprint_fault(%s, buf, len);\n}\n#endif", name, soap);
  fprintf(fd, "\n\nvoid %s::soap_noheader()\n{\t%s->header = NULL;\n}", name, soap);
  if (!namespaceid)
  {
    p = entry(classtable, lookup("SOAP_ENV__Header"));
    if (p)
    {
      t = (Table*)p->info.typ->ref;
      if (t && t->list && !is_void(t->list->info.typ))
      {
        fprintf(fd, "\n\nvoid %s::soap_header(", name);
        gen_params_ref(fd, t, NULL, 0);
        fprintf(fd, "\n{\n\t::soap_header(%s);", soap);
        for (param = t->list; param; param = param->next)
        {
          if (namespaceid)
            fprintf(fd, "\n\t((%s::SOAP_ENV__Header*)%s->header)->%s = %s;", namespaceid, soap, ident(param->sym->name), ident(param->sym->name));
          else
            fprintf(fd, "\n\t%s->header->%s = %s;", soap, ident(param->sym->name), ident(param->sym->name));
        }
        fprintf(fd, "\n}");
      }
    }
  }
  fprintf(fd, "\n\n::SOAP_ENV__Header *%s::soap_header()\n{\treturn %s->header;\n}", name, soap);
  fprintf(fd, "\n\n#ifndef WITH_NOIO");
  fprintf(fd, "\nint %s::run(int port, int backlog)\n{\tif (!soap_valid_socket(%s->master) && !soap_valid_socket(this->bind(NULL, port, backlog)))\n\t\treturn %s->error;\n\tfor (;;)\n\t{\tif (!soap_valid_socket(this->accept()))\n\t\t{\tif (%s->errnum == 0) // timeout?\n\t\t\t\t%s->error = SOAP_OK;\n\t\t\tbreak;\n\t\t}\n\t\tif (this->serve())\n\t\t\tbreak;\n\t\tthis->destroy();\n\t}\n\treturn %s->error;\n}", name, soap, soap, soap, soap, soap);
  fprintf(fd, "\n\n#if defined(WITH_OPENSSL) || defined(WITH_GNUTLS)");
  fprintf(fd, "\nint %s::ssl_run(int port, int backlog)\n{\tif (!soap_valid_socket(%s->master) && !soap_valid_socket(this->bind(NULL, port, backlog)))\n\t\treturn %s->error;\n\tfor (;;)\n\t{\tif (!soap_valid_socket(this->accept()))\n\t\t{\tif (%s->errnum == 0) // timeout?\n\t\t\t\t%s->error = SOAP_OK;\n\t\t\tbreak;\n\t\t}\n\t\tif (this->ssl_accept() || this->serve())\n\t\t\tbreak;\n\t\tthis->destroy();\n\t}\n\treturn %s->error;\n}", name, soap, soap, soap, soap, soap);
  fprintf(fd, "\n#endif");
  fprintf(fd, "\n\nSOAP_SOCKET %s::bind(const char *host, int port, int backlog)\n{\treturn soap_bind(%s, host, port, backlog);\n}", name, soap);
  fprintf(fd, "\n\nSOAP_SOCKET %s::accept()\n{\treturn soap_accept(%s);\n}", name, soap);
  fprintf(fd, "\n\n#if defined(WITH_OPENSSL) || defined(WITH_GNUTLS)");
  fprintf(fd, "\nint %s::ssl_accept()\n{\treturn soap_ssl_accept(%s);\n}", name, soap);
  fprintf(fd, "\n#endif");
  fprintf(fd, "\n#endif");
  fprintf(fd, "\n\nint %s::serve()", name);
  fprintf(fd, "\n{\n#ifndef WITH_FASTCGI\n\t%s->keep_alive = %s->max_keep_alive + 1;\n#endif\n\tdo\n\t{", soap, soap);
  fprintf(fd, "\n#ifndef WITH_FASTCGI\n\t\tif (%s->keep_alive > 0 && %s->max_keep_alive > 0)\n\t\t\t%s->keep_alive--;\n#endif", soap, soap, soap);
  fprintf(fd, "\n\t\tif (soap_begin_serve(%s))\n\t\t{\tif (%s->error >= SOAP_STOP)\n\t\t\t\tcontinue;\n\t\t\treturn %s->error;\n\t\t}", soap, soap, soap);
  fprintf(fd, "\n\t\tif ((dispatch() || (%s->fserveloop && %s->fserveloop(%s))) && %s->error && %s->error < SOAP_STOP)\n\t\t{\n#ifdef WITH_FASTCGI\n\t\t\tsoap_send_fault(%s);\n#else\n\t\t\treturn soap_send_fault(%s);\n#endif\n\t\t}", soap, soap, soap, soap, soap, soap, soap);
  fprintf(fd, "\n#ifdef WITH_FASTCGI\n\t\tsoap_destroy(%s);\n\t\tsoap_end(%s);\n\t} while (1);\n#else\n\t} while (%s->keep_alive);\n#endif", soap, soap, soap);
  fprintf(fd, "\n\treturn SOAP_OK;");
  fprintf(fd, "\n}\n");
  for (method = table->list; method; method = method->next)
  {
    if (method->info.typ->type == Tfun && !(method->info.sto & Sextern) && has_ns_eq(ns->name, method->sym->name))
    {
      if (jflag)
        fprintf(fd, "\nstatic int serve_%s(struct soap*, %s*);", ident(method->sym->name), name);
      else
        fprintf(fd, "\nstatic int serve_%s(%s*);", ident(method->sym->name), name);
    }
  }
  fprintf(fd, "\n\nint %s::dispatch()\n{", name);
  if (!iflag)
  {
    fprintf(fd, "\treturn dispatch(this->soap);\n}");
    fprintf(fd, "\n\nint %s::dispatch(struct soap* soap)\n{", name);
    fprintf(fd, "\n\t%s_init(soap->imode, soap->omode);", name);
    soap = "soap";
  }
  if (sflag)
    fprintf(fd, "\n\t%s->mode |= SOAP_XML_STRICT;", soap);
  if (aflag)
  {
    int i, num = 0;
    struct pair *map;
    for (method = table->list; method; method = method->next)
    {
      if (method->info.typ->type == Tfun && !(method->info.sto & Sextern) && has_ns_eq(ns->name, method->sym->name))
      {
        int found = 0;
        Service *sp;
        for (sp = services; sp; sp = sp->next)
        {
          if (has_ns_eq(sp->ns, method->sym->name))
          {
            Method *m;
            for (m = sp->list; m; m = m->next)
            {
              if (is_eq_nons(m->name, method->sym->name))
              {
                if (m->mess == ACTION || m->mess == REQUEST_ACTION)
                {
                  ++num;
                  found = 1;
                }
              }
            }
          }
        }
        if (Aflag && !found)
        {
          sprintf(errbuf, "Option -A requires a SOAPAction specified for operation %s where none is defined", ident(method->sym->name));
          compliancewarn(errbuf);
        }
      }
    }
    map = (struct pair*)emalloc(num * sizeof(struct pair));
    num = 0;
    for (method = table->list; method; method = method->next)
    {
      if (method->info.typ->type == Tfun && !(method->info.sto & Sextern) && has_ns_eq(ns->name, method->sym->name))
      {
        Service *sp;
        for (sp = services; sp; sp = sp->next)
        {
          if (has_ns_eq(sp->ns, method->sym->name))
          {
            Method *m;
            for (m = sp->list; m; m = m->next)
            {
              if (is_eq_nons(m->name, method->sym->name))
              {
                if (m->mess == ACTION || m->mess == REQUEST_ACTION)
                {
                  map[num].action = m->part;
                  map[num].method = method;
                  ++num;
                }
              }
            }
          }
        }
      }
    }
    if (num > 0)
    {
      qsort(map, num, sizeof(struct pair), mapcomp);
      if (num > 4) /* binary search worthwhile when num > 4 */
      {
        fprintf(fd, "\n\tif (soap->action)\n\t{\n\t\tconst char *soap_action[] = { ");
        for (i = 0; i < num; i++)
        {
          if (*map[i].action == '"')
            fprintf(fd, "%s, ", map[i].action);
          else
            fprintf(fd, "\"%s\", ", map[i].action);
        }
        fprintf(fd, " };");
        fprintf(fd, "\n\t\tswitch (soap_binary_search_string(soap_action, %d, soap->action))\n\t\t{", num);
        for (i = 0; i < num; i++)
        {
          fprintf(fd, "\n\t\t\tcase %d:\t", i);
          if (iflag)
            fprintf(fd, "\n\t\t\t\treturn serve_%s(this);", ident(map[i].method->sym->name));
          else
            fprintf(fd, "\n\t\t\t\treturn serve_%s(%s, this);", ident(map[i].method->sym->name), soap);
        }
        fprintf(fd, "\n\t\t}\n\t}");
      }
      else
      {
        fprintf(fd, "\n\tif (soap->action)\n\t{");
        for (i = 0; i < num; i++)
        {
          if (*map[i].action == '"')
            fprintf(fd, "\n\t\tif (!strcmp(soap->action, %s))", map[i].action);
          else
            fprintf(fd, "\n\t\tif (!strcmp(soap->action, \"%s\"))", map[i].action);
          if (iflag)
            fprintf(fd, "\n\t\t\treturn serve_%s(this);", ident(map[i].method->sym->name));
          else
            fprintf(fd, "\n\t\t\treturn serve_%s(%s, this);", ident(map[i].method->sym->name), soap);
        }
        fprintf(fd, "\n\t}");
      }
    }
  }
  if (!Aflag)
  {
    fprintf(fd, "\n\t(void)soap_peek_element(%s);", soap);
    catch_method = NULL;
    for (method = table->list; method; method = method->next)
    {
      if (method->info.typ->type == Tfun && !(method->info.sto & Sextern) && has_ns_eq(ns->name, method->sym->name))
      {
        if (is_invisible(method->sym->name))
        {
          Entry *param = entry(classtable, method->sym);
          if (param)
            param = ((Table*)param->info.typ->ref)->list;
          if (param)
          {
            fprintf(fd, "\n\tif (!soap_match_tag(%s, %s->tag, \"%s\")", soap, soap, ns_convert(param->sym->name));
            if (iflag)
              fprintf(fd, ")\n\t\treturn serve_%s(this);", ident(method->sym->name));
            else
              fprintf(fd, ")\n\t\treturn serve_%s(%s, this);", ident(method->sym->name), soap);
          }
          else
          {
            catch_method = method;
          }
        }
        else
        {
          fprintf(fd, "\n\tif (!soap_match_tag(%s, %s->tag, \"%s\")", soap, soap, ns_convert(method->sym->name));
          if (iflag)
            fprintf(fd, ")\n\t\treturn serve_%s(this);", ident(method->sym->name));
          else
            fprintf(fd, ")\n\t\treturn serve_%s(%s, this);", ident(method->sym->name), soap);
        }
      }
    }
    if (catch_method)
    {
      if (iflag)
        fprintf(fd, "\n\treturn serve_%s(this);\n}", ident(catch_method->sym->name));
      else
        fprintf(fd, "\n\treturn serve_%s(soap, this);\n}", ident(catch_method->sym->name));
    }
    else
    {
      fprintf(fd, "\n\treturn %s->error = SOAP_NO_METHOD;\n}", soap);
    }
  }
  else
  {
    fprintf(fd, "\n\treturn %s->error = SOAP_NO_METHOD;\n}", soap);
  }
  for (method = table->list; method; method = method->next)
    if (method->info.typ->type == Tfun && !(method->info.sto & Sextern) && !is_imported(method->info.typ) && has_ns_eq(ns->name, method->sym->name))
      gen_serve_method(fd, table, method, name);
  if (namespaceid)
    fprintf(fd, "\n\n} // namespace %s\n", namespaceid);
  fprintf(fd, "\n/* End of server object code */\n");
}

void
gen_method(FILE *fd, Entry *method, int server)
{
  Table *params;
  Entry *result, *p;
  const char *soap;
  if (iflag)
    soap = "this";
  else
    soap = "this->soap";
  result = (Entry*)method->info.typ->ref;
  p = entry(classtable, method->sym);
  if (!p)
    execerror("no table entry");
  params = (Table*)p->info.typ->ref;
  if (server || !is_transient(result->info.typ))
  {
    if (is_transient(result->info.typ))
      fprintf(fd, "\n        //\n        /// Web service one-way operation '%s' implementation, should return value of send_%s_empty_response() to send HTTP Accept acknowledgment, or return an error code, or return SOAP_OK to immediately return without sending an HTTP response message", ns_remove(method->sym->name), ns_remove(method->sym->name));
    else if (server)
      fprintf(fd, "\n        //\n        /// Web service operation '%s' implementation, should return SOAP_OK or error code", ns_remove(method->sym->name));
    else
      fprintf(fd, "\n        //\n        /// Web service synchronous operation '%s' with default endpoint and default SOAP Action header, returns SOAP_OK or error code", ns_remove(method->sym->name));
    fprintf(fd, "\n        virtual int %s(", ns_cname(method->sym->name, NULL));
    gen_params_ref(fd, params, result, 0);
    if (!server)
    {
      fprintf(fd, " { return this->%s(NULL, NULL", ns_cname(method->sym->name, NULL));
      gen_args(fd, params, result, 1);
      fprintf(fd, "; }");
      fprintf(fd, "\n        /// Web service synchronous operation '%s' to the specified endpoint and SOAP Action header, returns SOAP_OK or error code", ns_remove(method->sym->name));
      fprintf(fd, "\n        virtual int %s(const char *soap_endpoint_url, const char *soap_action", ns_cname(method->sym->name, NULL));
      gen_params_ref(fd, params, result, 1);
      fprintf(fd, " { return this->send_%s(soap_endpoint_url, soap_action", ns_cname(method->sym->name, NULL));
      gen_args(fd, params, NULL, 1);
      fprintf(fd, " || this->recv_%s(", ns_cname(method->sym->name, NULL));
      gen_args(fd, NULL, result, 0);
      if (iflag)
        fprintf(fd, " ? this->error : SOAP_OK; }");
      else
        fprintf(fd, " ? this->soap->error : SOAP_OK; }");
      fprintf(fd, "\n        /// Web service asynchronous operation 'send_%s' to send a request message to the specified endpoint and SOAP Action header, returns SOAP_OK or error code", ns_remove(method->sym->name));
      fprintf(fd, "\n        virtual int send_%s(const char *soap_endpoint_url, const char *soap_action", ns_cname(method->sym->name, NULL));
      gen_params_ref(fd, params, NULL, 1);
      fprintf(fd, ";");
      fprintf(fd, "\n        /// Web service asynchronous operation 'recv_%s' to receive a response message from the connected endpoint, returns SOAP_OK or error code", ns_remove(method->sym->name));
      fprintf(fd, "\n        virtual int recv_%s(", ns_cname(method->sym->name, NULL));
      gen_params_ref(fd, NULL, result, 0);
      fprintf(fd, ";");
    }
    if (server)
      fprintf(fd, " SOAP_PURE_VIRTUAL;");
    if (is_transient(result->info.typ))
    {
      fprintf(fd, "\n        /// Web service asynchronous send of HTTP Accept acknowledgment to be called in '%s', returns SOAP_OK or error code", ns_remove(method->sym->name));
      fprintf(fd, "\n        virtual int send_%s_empty_response(int soap_http_status = 202) { return soap_send_empty_response(%s, soap_http_status); }", ns_cname(method->sym->name, NULL), soap);
    }
  }
  else
  {
    fprintf(fd, "\n        //\n        /// Web service one-way synchronous send operation '%s' to the default endpoint with the default SOAP Action header then wait for HTTP OK/Accept response, returns SOAP_OK or error code", ns_remove(method->sym->name));
    fprintf(fd, "\n        virtual int %s(", ns_cname(method->sym->name, NULL));
    gen_params_ref(fd, params, result, 0);
    fprintf(fd, " { return this->%s(NULL, NULL", ns_cname(method->sym->name, NULL));
    gen_args(fd, params, result, 1);
    fprintf(fd, "; }");
    fprintf(fd, "\n        /// Web service one-way synchronous send operation '%s' to the specified endpoint and SOAP Action header then wait for HTTP OK/Accept response, returns SOAP_OK or error code", ns_remove(method->sym->name));
    fprintf(fd, "\n        virtual int %s(const char *soap_endpoint_url, const char *soap_action", ns_cname(method->sym->name, NULL));
    gen_params_ref(fd, params, result, 1);
    fprintf(fd, " { return (this->send_%s(soap_endpoint_url, soap_action", ns_cname(method->sym->name, NULL));
    gen_args(fd, params, result, 1);
    fprintf(fd, " || soap_recv_empty_response(%s)) ? %s->error : SOAP_OK; }", soap, soap);
    fprintf(fd, "\n        /// Web service one-way asynchronous send operation 'send_%s' with default endpoint and default SOAP Action header, returns SOAP_OK or error code", ns_remove(method->sym->name));
    fprintf(fd, "\n        virtual int send_%s(", ns_cname(method->sym->name, NULL));
    gen_params_ref(fd, params, result, 0);
    fprintf(fd, " { return this->send_%s(NULL, NULL", ns_cname(method->sym->name, NULL));
    gen_args(fd, params, result, 1);
    fprintf(fd, "; }");
    fprintf(fd, "\n        /// Web service one-way asynchronous send operation 'send_%s' to the specified endpoint and SOAP Action header, returns SOAP_OK or error code", ns_remove(method->sym->name));
    fprintf(fd, "\n        virtual int send_%s(const char *soap_endpoint_url, const char *soap_action", ns_cname(method->sym->name, NULL));
    gen_params_ref(fd, params, result, 1);
    fprintf(fd, ";\n        /// Web service one-way asynchronous receive operation 'recv_%s', returns SOAP_OK or error code", ns_remove(method->sym->name));
    fprintf(fd, ";\n        virtual int recv_%s(", ns_cname(method->sym->name, NULL));
    fprintf(fd, "struct %s&);", ident(method->sym->name));
    fprintf(fd, "\n        /// Web service asynchronous receive of HTTP Accept acknowledgment for one-way asynchronous send operation 'send_%s', returns SOAP_OK or error code", ns_remove(method->sym->name));
    fprintf(fd, "\n        virtual int recv_%s_empty_response() { return soap_recv_empty_response(%s); }", ns_cname(method->sym->name, NULL), soap);
  }
}

void
gen_report_params(Entry *type, Entry *result, int server)
{
  Table *params = (Table*)type->info.typ->ref;
  if (params)
  {
    Entry *param;
    for (param = params->list; param; param = param->next)
    {
      fprintf(freport, "- `%s` is %s", c_type_id(param->info.typ, param->sym->name), param->info.minOccurs > 0 ? "required" : "optional");
      if (param->info.hasval || param->info.ptrval)
      {
        if (param->info.fixed)
          fprintf(freport, " with the fixed value%s", c_init(param));
        else
          fprintf(freport, " with default value%s", c_init(param));
      }
      gen_report_member(type, param);
      gen_report_type(param->info.typ, "parameter");
      if (is_unmatched(param->sym))
        fprintf(freport, " (the leading underscore makes this parameter a wildcard that matches any XML element tag)");
      fprintf(freport, "\n");
    }
  }
  if (!is_transient(result->info.typ))
  {
    Tnode *ref = (Tnode*)result->info.typ->ref;
    if (result->info.typ->type == Tpointer)
      fprintf(freport, "- `%s` is the service operation response, which is a non-NULL pointer to a `%s`%s", c_type_id(result->info.typ, result->sym->name), c_type(result->info.typ->ref), server ? " that the service operation should populate with the response data" : " that the client provides with the call and is populated by the service operation as the result of the call");
    else
      fprintf(freport, "- `%s` is the service operation response data populated by the service operation", c_type_id(result->info.typ, result->sym->name));
    gen_report_type(ref, "result parameter");
    if (is_unmatched(result->sym))
      fprintf(freport, " (the leading underscore makes this result parameter a wildcard that matches any XML element tag)");
    fprintf(freport, "\n");
  }
  fprintf(freport, "\n");
}

void
gen_report_req_params(Tnode *typ)
{
  Table *Tptr;
  Entry *Eptr;
  int derclass = 0, flag = 0;
  if (!is_dynamic_array(typ))
  {
    for (Tptr = (Table*)typ->ref, derclass = 0; Tptr; Tptr = Tptr->prev, derclass++)
    {
      for (Eptr = Tptr->list; Eptr; Eptr = Eptr->next)
      {
        if (is_repetition(Eptr) || is_anytype(Eptr))
          flag = 2;
        if ((Eptr->info.minOccurs > 0 || flag) && !(Eptr->info.sto & (Sprivate | Sprotected | Sconst | Sstatic | Stypedef)) && Eptr->info.typ->type != Tfun && !is_soapref(Eptr->info.typ))
        {
          if (flag)
            flag--;
          if (is_smart(Eptr->info.typ))
          {
            if (is_smart_shared(Eptr->info.typ))
              fprintf(freport, ", %s %s", c_type_id(Eptr->info.typ, "&"), ident(Eptr->sym->name));
            else
              fprintf(freport, ", %s %s", c_type_id(Eptr->info.typ->ref, "*"), ident(Eptr->sym->name));
          }
          else if (Eptr->info.typ->type == Ttemplate && is_smart(Eptr->info.typ->ref) && !is_smart_shared(Eptr->info.typ->ref))
            continue;
          else if (Eptr->info.typ->type == Tclass || Eptr->info.typ->type == Tstruct || Eptr->info.typ->type == Tunion || Eptr->info.typ->type == Ttemplate)
            fprintf(freport, ", const %s& %s", c_type(Eptr->info.typ), ident(Eptr->sym->name));
          else if ((Eptr->info.sto & Sconstptr))
            fprintf(freport, ", const %s", c_type_id(Eptr->info.typ, Eptr->sym->name));
          else if (Eptr->info.typ->type == Tarray)
            fprintf(freport, ", %s", c_type_id(Eptr->info.typ, Eptr->sym->name));
          else
            fprintf(freport, ", %s", c_type_id(Eptr->info.typ, Eptr->sym->name));
          if (derclass && Eptr->info.typ->type != Tarray)
            fprintf(freport, "__%d", derclass);
        }
      }
    }
  }
}

void
gen_report_set_params(Tnode *typ)
{
  Table *Tptr;
  Entry *Eptr;
  int derclass = 0;
  for (Tptr = (Table*)typ->ref, derclass = 0; Tptr; Tptr = Tptr->prev, derclass++)
  {
    for (Eptr = Tptr->list; Eptr; Eptr = Eptr->next)
    {
      if (!(Eptr->info.sto & (Sprivate | Sprotected | Sconst | Sstatic | Stypedef)) && Eptr->info.typ->type != Tfun && !is_soapref(Eptr->info.typ))
      {
        if (is_smart(Eptr->info.typ))
        {
          if (is_smart_shared(Eptr->info.typ))
            fprintf(freport, ", %s %s", c_type_id(Eptr->info.typ, "&"), ident(Eptr->sym->name));
          else
            fprintf(freport, ", %s %s", c_type_id(Eptr->info.typ->ref, "*"), ident(Eptr->sym->name));
        }
        else if (Eptr->info.typ->type == Ttemplate && is_smart(Eptr->info.typ->ref) && !is_smart_shared(Eptr->info.typ->ref))
          continue;
        else if (Eptr->info.typ->type == Tclass || Eptr->info.typ->type == Tstruct || Eptr->info.typ->type == Tunion || Eptr->info.typ->type == Ttemplate)
          fprintf(freport, ", const %s& %s", c_type(Eptr->info.typ), ident(Eptr->sym->name));
        else if ((Eptr->info.sto & Sconstptr))
          fprintf(freport, ", const %s", c_type_id(Eptr->info.typ, Eptr->sym->name));
        else if (Eptr->info.typ->type == Tarray)
          fprintf(freport, ", %s", c_type_id(Eptr->info.typ, Eptr->sym->name));
        else
          fprintf(freport, ", %s", c_type_id(Eptr->info.typ, Eptr->sym->name));
        if (derclass && Eptr->info.typ->type != Tarray)
          fprintf(freport, "__%d", derclass);
      }
    }
  }
}

void
gen_params_ref(FILE *fd, Table *params, Entry *result, int flag)
{
  if (params)
  {
    Entry *param;
    for (param = params->list; param; param = param->next)
    {
      if (!cflag && (param->info.typ->type == Tstruct || param->info.typ->type == Tclass))
        fprintf(fd, "%s%s%s& %s", (flag || param != params->list) ? ", " : "", c_storage(param->info.sto | Sconst), c_type(param->info.typ), ident(param->sym->name));
      else
        fprintf(fd, "%s%s%s", (flag || param != params->list) ? ", " : "", c_storage(param->info.sto), c_type_id(param->info.typ, param->sym->name));
    }
  }
  if (!result || is_transient(result->info.typ))
    fprintf(fd, ")");
  else
    fprintf(fd, "%s%s%s)", (flag || (params && params->list)) ? ", " : "", c_storage(result->info.sto), c_type_id(result->info.typ, result->sym->name));
}

void
gen_params(FILE *fd, Table *params, Entry *result, int flag)
{
  if (params)
  {
    Entry *param;
    for (param = params->list; param; param = param->next)
      fprintf(fd, "%s%s%s", (flag || param != params->list) ? ", " : "", c_storage(param->info.sto), c_type_id(param->info.typ, param->sym->name));
  }
  if (!result || is_transient(result->info.typ))
    fprintf(fd, ")");
  else
    fprintf(fd, "%s%s%s)", (flag || (params && params->list)) ? ", " : "", c_storage(result->info.sto), c_type_id(result->info.typ, result->sym->name));
}

void
gen_args(FILE *fd, Table *params, Entry *result, int flag)
{
  if (params)
  {
    Entry *param;
    for (param = params->list; param; param = param->next)
      fprintf(fd, "%s%s", (flag || param != params->list) ? ", " : "", ident(param->sym->name));
  }
  if (!result || is_transient(result->info.typ))
    fprintf(fd, ")");
  else
    fprintf(fd, "%s%s)", (flag || (params && params->list)) ? ", " : "", ident(result->sym->name));
}

void
gen_query_url(FILE *fd, Table *params, int soap)
{
  Entry *param;
  if (soap)
    fprintf(fd, "\n\tsoap_extend_url_query(soap, soap_endpoint, NULL);");
  else
    fprintf(fd, "\n\tsoap_extend_url_query(soap, soap_endpoint, soap_action);");
  for (param = params->list; param; param = param->next)
  {
    if (is_transient(param->info.typ))
      continue;
    if (is_repetition(param) && is_primitive_or_string(param->next->info.typ->ref))
    {
      fprintf(fd, "\n\t{\tint i;\n\t\tfor (i = 0; i < (int)%s; i++)\n\t\t\tsoap_url_query(soap, \"%s=\", ", ident(param->sym->name), ns_remove(param->next->sym->name));
      gen_query_url_type2s(fd, param->next->info.typ->ref, ident(param->next->sym->name), "", "[i]");
      fprintf(fd, ");\n\t}");
      param = param->next;
    }
    else if (param->info.typ->type == Tpointer && is_container(param->info.typ->ref) && is_primitive_or_string(((Tnode*)param->info.typ->ref)->ref))
    {
      fprintf(fd, "\n\tif (%s)\n\t\tfor (%s::const_iterator i = %s->begin(); i != %s->end(); ++i)\n\t\t\tsoap_url_query(soap, \"%s=\", ", ident(param->sym->name), c_type(param->info.typ->ref), ident(param->sym->name), ident(param->sym->name), ns_remove(param->sym->name));
      gen_query_url_type2s(fd, ((Tnode*)param->info.typ->ref)->ref, "i", "*", "");
      fprintf(fd, ");");
    }
    else if (is_primitive_or_string(param->info.typ))
    {
      fprintf(fd, "\n\tsoap_url_query(soap, \"%s=\", ", ns_remove(param->sym->name));
      gen_query_url_type2s(fd, param->info.typ, ident(param->sym->name), "", "");
      fprintf(fd, ");");
    }
    else if (param->info.typ->type == Tpointer && is_primitive_or_string(param->info.typ->ref))
    {
      fprintf(fd, "\n\tif (%s)\n\t\tsoap_url_query(soap, \"%s=\", ", ident(param->sym->name), ns_remove(param->sym->name));
      gen_query_url_type2s(fd, param->info.typ->ref, ident(param->sym->name), "*", "");
      fprintf(fd, ");");
    }
  }
}

void
gen_query_url_type2s(FILE *fd, Tnode *typ, const char *name, const char *ptr, const char *idx)
{
  if (!is_transient(typ))
  {
    if (is_stdstring(typ))
      fprintf(fd, "(%s%s%s).c_str()", ptr, name, idx);
    else if (is_stdwstring(typ))
      fprintf(fd, "soap_wchar2s(soap, (%s%s%s).c_str())", ptr, name, idx);
    else if (is_string(typ))
      fprintf(fd, "%s%s%s", ptr, name, idx);
    else if (is_wstring(typ))
      fprintf(fd, "soap_wchar2s(soap, %s%s%s)", ptr, name, idx);
    else if (is_primitive(typ))
      fprintf(fd, "soap_%s2s(soap, %s%s%s)", c_ident(typ), ptr, name, idx);
  }
}

void
gen_query_send_form_init(FILE *fd, Table *params)
{
  Entry *param;
  for (param = params->list; param; param = param->next)
  {
    if (!is_transient(param->info.typ))
    {
      if (is_repetition(param) && is_primitive_or_string(param->next->info.typ->ref))
      {
        fprintf(fd, "\n\tint soap_tmp_i;");
        break;
      }
    }
  }
}

void
gen_query_send_form(FILE *fd, Table *params)
{
  Entry *param;
  int init = 0;
  for (param = params->list; param; param = param->next)
  {
    if (!is_transient(param->info.typ))
    {
      if (is_repetition(param) && is_primitive_or_string(param->next->info.typ->ref))
      {
        if (init)
          fprintf(fd, ")\n\t\treturn soap_closesock(soap);");
        fprintf(fd, "\n\tfor (soap_tmp_i = 0; soap_tmp_i < (int)%s; soap_tmp_i++)\n\t\tif (soap_query_send_key(soap, \"%s\") || ", ident(param->sym->name), ns_remove(param->next->sym->name));
        gen_query_form_type2s(fd, param->next->info.typ->ref, ident(param->next->sym->name), "", "[soap_tmp_i]");
        fprintf(fd, ")\n\t\t\treturn soap_closesock(soap);");
        param = param->next;
        init = 0;
      }
      else if (is_container(param->info.typ) && is_primitive_or_string(param->info.typ->ref))
      {
        if (init)
          fprintf(fd, ")\n\t\treturn soap_closesock(soap);");
        fprintf(fd, "\n\tfor (%s::const_iterator i = %s.begin(); i != %s.end(); ++i)\n\t\tif (soap_query_send_key(soap, \"%s\") || ", c_type(param->info.typ), ident(param->sym->name), ident(param->sym->name), ns_remove(param->sym->name));
        gen_query_form_type2s(fd, param->info.typ->ref, "i", "*", "");
        fprintf(fd, ")\n\t\t\treturn soap_closesock(soap);");
        init = 0;
      }
      else if (param->info.typ->type == Tpointer && is_container(param->info.typ->ref) && is_primitive_or_string(((Tnode*)param->info.typ->ref)->ref))
      {
        if (init)
          fprintf(fd, ")\n\t\treturn soap_closesock(soap);");
        fprintf(fd, "\n\tif (%s)\n\t\tfor (%s::const_iterator i = %s->begin(); i != %s->end(); ++i)\n\t\t\tif (soap_query_send_key(soap, \"%s\") || ", ident(param->sym->name), c_type(param->info.typ->ref), ident(param->sym->name), ident(param->sym->name), ns_remove(param->sym->name));
        gen_query_form_type2s(fd, ((Tnode*)param->info.typ->ref)->ref, "i", "*", "");
        fprintf(fd, ")\n\t\t\t\treturn soap_closesock(soap);");
        init = 0;
      }
      else if (is_primitive_or_string(param->info.typ))
      {
        if (!init++)
          fprintf(fd, "\n\tif (");
        else
          fprintf(fd, "\n\t || ");
        fprintf(fd, "soap_query_send_key(soap, \"%s\")\n\t || ", ns_remove(param->sym->name));
        gen_query_form_type2s(fd, param->info.typ, ident(param->sym->name), "", "");
      }
      else if (param->info.typ->type == Tpointer && is_primitive_or_string(param->info.typ->ref))
      {
        if (!init++)
          fprintf(fd, "\n\tif (");
        else
          fprintf(fd, "\n\t || ");
        fprintf(fd, "(%s && (soap_query_send_key(soap, \"%s\") || ", ident(param->sym->name), ns_remove(param->sym->name));
        gen_query_form_type2s(fd, param->info.typ->ref, ident(param->sym->name), "*", "");
        fprintf(fd, "))");
      }
    }
  }
  if (!init)
    fprintf(fd, "\n\tif (");
  else
    fprintf(fd, "\n\t || ");
  fprintf(fd, "soap_end_send(soap))\n\t\treturn soap_closesock(soap);");
}

void
gen_query_recv_form_init(FILE *fd, Entry *result)
{
  if ((result->info.typ->type == Treference || result->info.typ->type == Tpointer) && !is_invisible_empty((Tnode*)result->info.typ->ref))
  {
    if ((((Tnode*)(result->info.typ->ref))->type == Tstruct || ((Tnode*)(result->info.typ->ref))->type == Tclass) && !is_stdstr(result->info.typ->ref) && !is_dynamic_array(result->info.typ->ref))
    {
      Table *table = ((Tnode*)result->info.typ->ref)->ref;
      if (table)
      {
        Entry *param;
        for (param = table->list; param; param = param->next)
        {
          if (!is_transient(param->info.typ))
          {
            if (is_repetition(param) && is_primitive_or_string(param->next->info.typ->ref))
            {
              fprintf(fd, "\n\tstruct soap_blist *soap_blist_%s = NULL;", ident(param->next->sym->name));
              param = param->next;
            }
          }
        }
      }
    }
  }
}

void
gen_query_recv_form(FILE *fd, Entry *result)
{
  if ((result->info.typ->type == Treference || result->info.typ->type == Tpointer) && !is_invisible_empty((Tnode*)result->info.typ->ref))
  {
    if (is_primitive_or_string(result->info.typ->ref))
    {
      fprintf(fd, "\n\tsoap_tmp_key = soap_query_key(soap, &soap_tmp);\n\tif (!soap_tmp_key)\n\t{\tsoap->error = SOAP_EMPTY;\n\t\treturn soap_closesock(soap);\n\t}");
      fprintf(fd, "\n\tif (!strcmp(soap_tmp_key, \"%s\"))\n\t{\tif (", ns_remove(result->sym->name));
      gen_query_form_s2type(fd, result->info.typ->ref, ident(result->sym->name), result->info.typ->type == Treference ? "&" : "", "", "");
      fprintf(fd, ")\n\t\t\treturn soap_closesock(soap);\n\t}");
    }
    else if ((((Tnode*)(result->info.typ->ref))->type == Tstruct || ((Tnode*)(result->info.typ->ref))->type == Tclass) && !is_dynamic_array(result->info.typ->ref))
    {
      Table *table = ((Tnode*)result->info.typ->ref)->ref;
      if (table)
      {
        Entry *param;
        fprintf(fd, "\n\twhile (soap_tmp && (soap_tmp_key = soap_query_key(soap, &soap_tmp)))\n\t{");
        for (param = table->list; param; param = param->next)
        {
          if (!is_transient(param->info.typ))
          {
            if (is_repetition(param) && is_stdstr(param->next->info.typ->ref))
            {
              fprintf(fd, "\n\t\tif (!strcmp(soap_tmp_key, \"%s\"))\n\t\t{\tif (soap_blist_%s == NULL)\n\t\t\t\tsoap_blist_%s = soap_alloc_block(soap);\n\t\t\t%s%s%s = soap_block<%s>::push(soap, soap_blist_%s);\n\t\t\tif (%s%s%s == NULL)\n\t\t\t\tbreak;\n\t\t\t%s%s%s++;\n\t\t\tif (", ns_remove(param->next->sym->name), ident(param->next->sym->name), ident(param->next->sym->name),  ident(result->sym->name), result->info.typ->type == Treference ? "." : "->", ident(param->next->sym->name), c_type(param->next->info.typ->ref), ident(param->next->sym->name), ident(result->sym->name), result->info.typ->type == Treference ? "." : "->", ident(param->next->sym->name), ident(result->sym->name), result->info.typ->type == Treference ? "." : "->", ident(param->sym->name));
              gen_query_form_s2type(fd, param->next->info.typ->ref, ident(result->sym->name), "", result->info.typ->type == Treference ? "." : "->", ident(param->next->sym->name));
              fprintf(fd, ")\n\t\t\t\tbreak;\n\t\t}");
              param = param->next;
            }
            else if (is_repetition(param) && is_primitive_or_string(param->next->info.typ->ref))
            {
              fprintf(fd, "\n\t\tif (!strcmp(soap_tmp_key, \"%s\"))\n\t\t{\tif (soap_blist_%s == NULL)\n\t\t\t\tsoap_blist_%s = soap_alloc_block(soap);\n\t\t\t%s%s%s = (%s)soap_push_block_max(soap, soap_blist_%s, sizeof(%s));\n\t\t\tif (%s%s%s == NULL)\n\t\t\t\tbreak;\n\t\t\t%s%s%s++;\n\t\t\tif (", ns_remove(param->next->sym->name), ident(param->next->sym->name), ident(param->next->sym->name),  ident(result->sym->name), result->info.typ->type == Treference ? "." : "->", ident(param->next->sym->name), c_type(param->next->info.typ), ident(param->next->sym->name), c_type(param->next->info.typ->ref), ident(result->sym->name), result->info.typ->type == Treference ? "." : "->", ident(param->next->sym->name), ident(result->sym->name), result->info.typ->type == Treference ? "." : "->", ident(param->sym->name));
              gen_query_form_s2type(fd, param->next->info.typ->ref, ident(result->sym->name), "", result->info.typ->type == Treference ? "." : "->", ident(param->next->sym->name));
              fprintf(fd, ")\n\t\t\t\tbreak;\n\t\t}");
              param = param->next;
            }
            else if (is_container(param->info.typ) && is_primitive_or_string(param->info.typ->ref))
            {
              fprintf(fd, "\n\t\tif (!strcmp(soap_tmp_key, \"%s\"))\n\t\t{\t%s soap_tmp_val;\n\t\t\tif (", ns_remove(param->sym->name), c_type(param->info.typ->ref));
              gen_query_form_s2type(fd, param->info.typ->ref, "soap_tmp_val", "&", "", "");
              fprintf(fd, ")\n\t\t\t\tbreak;\n\t\t\t%s%s%s.insert(%s%s%s.end(), soap_tmp_val);\n\t\t}", ident(result->sym->name), result->info.typ->type == Treference ? "." : "->", ident(param->sym->name), ident(result->sym->name), result->info.typ->type == Treference ? "." : "->", ident(param->sym->name));
            }
            else if (param->info.typ->type == Tpointer && is_container(param->info.typ->ref) && is_primitive_or_string(((Tnode*)param->info.typ->ref)->ref))
            {
              fprintf(fd, "\n\t\tif (!strcmp(soap_tmp_key, \"%s\"))\n\t\t{\t%s soap_tmp_val;\n\t\t\tif (", ns_remove(param->sym->name), c_type(((Tnode*)param->info.typ->ref)->ref));
              gen_query_form_s2type(fd, ((Tnode*)param->info.typ->ref)->ref, "soap_tmp_val", "&", "", "");
              fprintf(fd, ")\n\t\t\t\tbreak;\n\t\t\t%s%s%s->insert(%s%s%s->end(), soap_tmp_val);\n\t\t}", ident(result->sym->name), result->info.typ->type == Treference ? "." : "->", ident(param->sym->name), ident(result->sym->name), result->info.typ->type == Treference ? "." : "->", ident(param->sym->name));
            }
            else if (is_primitive_or_string(param->info.typ))
            {
              fprintf(fd, "\n\t\tif (!strcmp(soap_tmp_key, \"%s\"))\n\t\t{\tif (", ns_remove(param->sym->name));
              gen_query_form_s2type(fd, param->info.typ, ident(result->sym->name), "&", result->info.typ->type == Treference ? "." : "->", ident(param->sym->name));
              fprintf(fd, ")\n\t\t\t\tbreak;\n\t\t}");
            }
            else if (param->info.typ->type == Tpointer && is_primitive_or_string(param->info.typ->ref))
            {
              fprintf(fd, "\n\t\tif (!strcmp(soap_tmp_key, \"%s\"))\n\t\t{\tif (", ns_remove(param->sym->name));
              gen_query_form_s2type(fd, param->info.typ->ref, ident(result->sym->name), "", result->info.typ->type == Treference ? "." : "->", ident(param->sym->name));
              fprintf(fd, ")\n\t\t\t\tbreak;\n\t\t}");
            }
          }
        }
        fprintf(fd, "\n\t}");
        for (param = table->list; param; param = param->next)
        {
          if (!is_transient(param->info.typ))
          {
            if (is_repetition(param) && is_stdstr(param->next->info.typ->ref))
            {
              fprintf(fd, "\n\tif (soap_blist_%s)\n\t{\t%s%s%s = soap_new_%s(soap, %s%s%s);\n\t\tif (%s%s%s)\n\t\t\tsoap_block<%s>::save(soap, soap_blist_%s, %s%s%s);\n\t}", ident(param->next->sym->name), ident(result->sym->name), result->info.typ->type == Treference ? "." : "->", ident(param->next->sym->name), c_ident(param->next->info.typ->ref), ident(result->sym->name), result->info.typ->type == Treference ? "." : "->", ident(param->sym->name), ident(result->sym->name), result->info.typ->type == Treference ? "." : "->", ident(param->next->sym->name), c_type(param->next->info.typ->ref), ident(param->next->sym->name), ident(result->sym->name), result->info.typ->type == Treference ? "." : "->", ident(param->next->sym->name));
            }
            else if (is_repetition(param) && is_primitive_or_string(param->next->info.typ->ref))
            {
              fprintf(fd, "\n\tif (soap_blist_%s)\n\t\t%s%s%s = (%s)soap_save_block(soap, soap_blist_%s, NULL, 1);", ident(param->next->sym->name),  ident(result->sym->name), result->info.typ->type == Treference ? "." : "->", ident(param->next->sym->name), c_type(param->next->info.typ), ident(param->next->sym->name));
              param = param->next;
            }
          }
        }
      }
    }
  }
}

void
gen_query_form_type2s(FILE *fd, Tnode *typ, const char *name, const char *ptr, const char *idx)
{
  if (is_stdstring(typ))
    fprintf(fd, "soap_query_send_val(soap, (%s%s%s).c_str())", ptr, name, idx);
  else if (is_stdwstring(typ))
    fprintf(fd, "soap_query_send_val(soap, soap_wchar2s(soap, (%s%s%s)->c_str()))", ptr, name, idx);
  else if (is_string(typ))
    fprintf(fd, "soap_query_send_val(soap, %s%s%s)", ptr, name, idx);
  else if (is_wstring(typ))
    fprintf(fd, "soap_query_send_val(soap, soap_wchar2s(soap, %s%s%s))", ptr, name, idx);
  else if (typ->type == Tenum || typ->type == Tenumsc)
    fprintf(fd, "soap_query_send_val(soap, soap_%s2s(soap, %s%s%s))", c_ident(typ), ptr, name, idx);
  else if (is_primitive(typ))
    fprintf(fd, "soap_query_send_val(soap, soap_%s2s(soap, %s%s%s))", c_ident(typ), ptr, name, idx);
}

void
gen_query_form_s2type(FILE *fd, Tnode *typ, const char *name, const char *ptr, const char *ref, const char *idx)
{
  if (is_stdstring(typ))
    fprintf(fd, "soap_s2stdchar(soap, soap_query_val(soap, &soap_tmp), %s%s%s%s, %d, %ld, %ld, %s)", ptr, name, ref, idx, property(typ), minlen(typ), maxlen(typ), pattern(typ));
  else if (is_stdwstring(typ))
    fprintf(fd, "soap_s2stdwchar(soap, soap_query_val(soap, &soap_tmp), %s%s%s%s, %d, %ld, %ld, %s)", ptr, name, ref, idx, property(typ), minlen(typ), maxlen(typ), pattern(typ));
  else if (is_string(typ))
    fprintf(fd, "soap_string(soap, soap_query_val(soap, &soap_tmp), %s%s%s%s, %d, %ld, %ld, %s)", ptr, name, ref, idx, property(typ), minlen(typ), maxlen(typ), pattern(typ));
  else if (is_wstring(typ))
    fprintf(fd, "soap_wstring(soap, soap_query_val(soap, &soap_tmp), %s%s%s%s, %d, %ld, %ld, %s)", ptr, name, ref, idx, property(typ), minlen(typ), maxlen(typ), pattern(typ));
  else if (is_primitive(typ))
    fprintf(fd, "soap_s2%s(soap, soap_query_val(soap, &soap_tmp), %s%s%s%s)", c_ident(typ), ptr, name, ref, idx);
}

void
gen_call_proto(FILE *fd, Entry *method)
{
  Table *params;
  Entry *result, *p;
  result = (Entry*)method->info.typ->ref;
  p = entry(classtable, method->sym);
  if (!p)
    execerror("no table entry");
  params = (Table*)p->info.typ->ref;
  if (fd == freport)
    gen_report_operation(NULL, method, 0);
  if (!is_transient(result->info.typ))
  {
    fprintf(fd, "\n    ");
    fprintf(fd, "\n    /** Web service synchronous operation 'soap_call_%s' to the specified endpoint and SOAP Action header, returns SOAP_OK or error code */", ident(method->sym->name));
    fprintf(fd, "\n    SOAP_FMAC5 int SOAP_FMAC6 soap_call_%s(struct soap *soap, const char *soap_endpoint, const char *soap_action", ident(method->sym->name));
    gen_params_ref(fd, params, result, 1);
    fprintf(fd, ";");
    fprintf(fd, "\n    /** Web service asynchronous operation 'soap_send_%s' to send a request message to the specified endpoint and SOAP Action header, returns SOAP_OK or error code */", ident(method->sym->name));
    fprintf(fd, "\n    SOAP_FMAC5 int SOAP_FMAC6 soap_send_%s(struct soap *soap, const char *soap_endpoint, const char *soap_action", ident(method->sym->name));
    gen_params_ref(fd, params, NULL, 1);
    fprintf(fd, ";");
    fprintf(fd, "\n    /** Web service asynchronous operation 'soap_recv_%s' to receive a response message from the connected endpoint, returns SOAP_OK or error code */", ident(method->sym->name));
    fprintf(fd, "\n    SOAP_FMAC5 int SOAP_FMAC6 soap_recv_%s(struct soap *soap", ident(method->sym->name));
    gen_params_ref(fd, NULL, result, 1);
    fprintf(fd, ";");
  }
  else
  {
    fprintf(fd, "\n    /** Web service one-way asynchronous operation 'soap_send_%s' */", ident(method->sym->name));
    fprintf(fd, "\n    SOAP_FMAC5 int SOAP_FMAC6 soap_send_%s(struct soap *soap, const char *soap_endpoint, const char *soap_action", ident(method->sym->name));
    gen_params_ref(fd, params, NULL, 1);
    fprintf(fd, ";");
    fprintf(fd, "\n    /** Web service one-way asynchronous operation 'soap_recv_%s' */", ident(method->sym->name));
    fprintf(fd, "\n    SOAP_FMAC5 int SOAP_FMAC6 soap_recv_%s(struct soap *soap, struct %s *%s);", ident(method->sym->name), ident(method->sym->name), ident(result->sym->name));
  }
  if (fd == freport)
  {
    fprintf(freport, "\n\nwhere:\n\n- `struct soap *soap` is the context\n- `const char *soap_endpoint` is the endpoint URL (or list of space-separated URLs) or NULL to use the default endpoint(s)\n- `const char *soap_action` is the SOAP action header or NULL to use the default action (recommended)\n");
    gen_report_params(p, result, 0);
    if (!is_transient(result->info.typ))
      fprintf(freport, "The `soap_call_%s` function sends the request message and receives the response message, assigning the last parameter `%s` the response value received. The `soap_send_%s` function sends the request message and the `soap_recv_%s` function receives the response message asynchronously. These functions return `SOAP_OK` or an error code.\n\n", ident(method->sym->name), ident(result->sym->name), ident(method->sym->name), ident(method->sym->name));
    else
      fprintf(freport, "The `soap_send_%s` function sends the one-way request message and the `soap_recv_%s` function receives the one-way request message. The `int soap_recv_empty_response(struct soap *soap)` function should be called after the `send_%s` function when communicating over HTTP to receive the HTTP acknowledgment.\n\n", ident(method->sym->name), ident(method->sym->name), ident(method->sym->name));
    gen_report_hr();
  }
}

void
gen_call_method(FILE *fd, Entry *method, const char *name)
{
  Service *sp;
  Method *m;
  int soap = (soap_version >= 0);
  int version = soap_version;
  int get = 0;
  int put = 0;
  int del = 0;
  int post = 0;
  int mimein = 0;
  int mimeout = 0;
  const char *encoding = NULL;
  const char *xtag;
  const char *action = NULL, *method_encoding = NULL, *method_response_encoding = NULL;
  Table *params;
  Entry *param, *result, *p, *response = NULL;
  result = (Entry*)method->info.typ->ref;
  (void)post;
  p = entry(classtable, method->sym);
  if (!p)
    execerror("no table entry");
  params = (Table*)p->info.typ->ref;
  if (!is_response(result->info.typ) && !is_XML(result->info.typ))
    response = get_response(method->info.typ);
  for (sp = services; sp; sp = sp->next)
  {
    if (has_ns_eq(sp->ns, method->sym->name))
    {
      encoding = sp->encoding;
      method_encoding = encoding;
      method_response_encoding = NULL;
      if (sp->protocol)
      {
        if (strstr(sp->protocol, "GET"))
          get = 1;
        else if (strstr(sp->protocol, "PUT"))
          put = 1;
        else if (strstr(sp->protocol, "DELETE"))
          del = 1;
        else if (strstr(sp->protocol, "POST"))
          post = 1;
        if (strncmp(sp->protocol, "SOAP", 4))
          soap = 0;
        else if (strlen(sp->protocol) > 6)
          version = sp->protocol[6] - '0';
      }
      for (m = sp->list; m; m = m->next)
      {
        if (is_eq_nons(m->name, method->sym->name))
        {
          if (m->mess == ACTION || m->mess == REQUEST_ACTION)
            action = m->part;
          else if (m->mess == ENCODING)
            method_encoding = m->part;
          else if (m->mess == RESPONSE_ENCODING)
            method_response_encoding = m->part;
          else if (m->mess == PROTOCOL)
          {
            if (strstr(m->part, "GET"))
              get = 1;
            else if (strstr(m->part, "PUT"))
              put = 1;
            else if (strstr(m->part, "DELETE"))
              del = 1;
            else if (strstr(m->part, "POST"))
              post = 1;
            if (strncmp(m->part, "SOAP", 4))
              soap = 0;
            else if (strlen(m->part) > 6)
              version = m->part[6] - '0';
          }
          else
          {
            if ((m->mess&MIMEIN) && !strcmp(m->part, "application/x-www-form-urlencoded"))
              mimein = 1;
            if ((m->mess&MIMEOUT) && !strcmp(m->part, "application/x-www-form-urlencoded"))
              mimeout = 1;
          }
        }
      }
      break;
    }
  }
  if (name)
  {
    if (!is_transient(result->info.typ))
    {
      fprintf(fd, "\n\nint %s::send_%s(const char *soap_endpoint_url, const char *soap_action", name, ns_cname(method->sym->name, NULL));
      gen_params_ref(fd, params, NULL, 1);
    }
    else
    {
      fprintf(fd, "\n\nint %s::send_%s(const char *soap_endpoint_url, const char *soap_action", name, ns_cname(method->sym->name, NULL));
      gen_params_ref(fd, params, NULL, 1);
    }
  }
  else
  {
    if (!is_transient(result->info.typ))
    {
      fprintf(fd, "\n\nSOAP_FMAC5 int SOAP_FMAC6 soap_call_%s(struct soap *soap, const char *soap_endpoint, const char *soap_action", ident(method->sym->name));
      gen_params_ref(fd, params, result, 1);
      fprintf(fd, "\n{\tif (soap_send_%s(soap, soap_endpoint, soap_action", ident(method->sym->name));
      gen_args(fd, params, NULL, 1);
      if (!del)
      {
        fprintf(fd, " || soap_recv_%s(soap", ident(method->sym->name));
        gen_args(fd, NULL, result, 1);
      }
      fprintf(fd, ")\n\t\treturn soap->error;\n\treturn SOAP_OK;\n}");
      fprintf(fd, "\n\nSOAP_FMAC5 int SOAP_FMAC6 soap_send_%s(struct soap *soap, const char *soap_endpoint, const char *soap_action", ident(method->sym->name));
      gen_params_ref(fd, params, NULL, 1);
    }
    else
    {
      fprintf(fd, "\n\nSOAP_FMAC5 int SOAP_FMAC6 soap_send_%s(struct soap *soap, const char *soap_endpoint, const char *soap_action", ident(method->sym->name));
      gen_params_ref(fd, params, NULL, 1);
    }
  }
  if (name)
  {
    if (iflag)
      fprintf(fd, "\n{\n\tstruct soap *soap = this;\n");
    else
      fprintf(fd, "\n{\n");
  }
  else
  {
    fprintf(fd, "\n{");
  }
  if (!get && !del && !mimein)
    fprintf(fd, "\tstruct %s soap_tmp_%s;", ident(method->sym->name), ident(method->sym->name));
  else if (!get && !del && mimein)
    gen_query_send_form_init(fd, params);
  if (name)
    fprintf(fd, "\n\tif (soap_endpoint_url != NULL)\n\t\tsoap_endpoint = soap_endpoint_url;");
  if (sp && sp->URL)
    fprintf(fd, "\n\tif (soap_endpoint == NULL)\n\t\tsoap_endpoint = \"%s\";", sp->URL);
  if (action)
  {
    fprintf(fd, "\n\tif (soap_action == NULL)\n\t\tsoap_action = ");
    if (*action == '"')
      fprintf(fd, "%s;", action);
    else
      fprintf(fd, "\"%s\";", action);
  }
  if (!method_response_encoding)
    method_response_encoding = method_encoding;
  if (!get && !del && !mimein)
  {
    for (param = params->list; param; param = param->next)
    {
      if (param->info.typ->type == Tarray)
        fprintf(fd, "\n\tsoap_memcpy(soap_tmp_%s.%s, sizeof(%s), %s, sizeof(%s));", ident(method->sym->name), ident(param->sym->name), c_type(param->info.typ), ident(param->sym->name), c_type(param->info.typ));
      else
        fprintf(fd, "\n\tsoap_tmp_%s.%s = %s;", ident(method->sym->name), ident(param->sym->name), ident(param->sym->name));
    }
    fprintf(fd, "\n\tsoap_begin(soap);");
    if (!soap)
      fprintf(fd, "\n\tsoap_set_version(soap, 0); /* no SOAP */");
    else if (version > 0)
      fprintf(fd, "\n\tsoap_set_version(soap, %d); /* use SOAP1.%d */", version, version);
    if (soap && sp && method_encoding)
    {
      if (is_literal(method_encoding))
        fprintf(fd, "\n\tsoap->encodingStyle = NULL; /* use SOAP literal style */");
      else if (method_encoding)
        fprintf(fd, "\n\tsoap->encodingStyle = \"%s\"; /* use SOAP encoding style */", method_encoding);
    }
    else if (!eflag)
    {
      fprintf(fd, "\n\tsoap->encodingStyle = NULL; /* use SOAP literal style */");
    }
    if (soap)
      fprintf(fd, "\n\tsoap_serializeheader(soap);");
    fprintf(fd, "\n\tsoap_serialize_%s(soap, &soap_tmp_%s);", ident(method->sym->name), ident(method->sym->name));
    fprintf(fd, "\n\tif (soap_begin_count(soap))\n\t\treturn soap->error;");
    fprintf(fd, "\n\tif ((soap->mode & SOAP_IO_LENGTH))");
    fprintf(fd, "\n\t{\tif (soap_envelope_begin_out(soap)");
    if (soap)
    {
      fprintf(fd, "\n\t\t || soap_putheader(soap)");
      fprintf(fd, "\n\t\t || soap_body_begin_out(soap)");
    }
    fprintf(fd, "\n\t\t || soap_put_%s(soap, &soap_tmp_%s, \"%s\", \"\")", ident(method->sym->name), ident(method->sym->name), ns_convert(method->sym->name));
    if (soap)
      fprintf(fd, "\n\t\t || soap_body_end_out(soap)");
    fprintf(fd, "\n\t\t || soap_envelope_end_out(soap))");
    fprintf(fd, "\n\t\t\t return soap->error;");
    fprintf(fd, "\n\t}");
    fprintf(fd, "\n\tif (soap_end_count(soap))\n\t\treturn soap->error;");
    if (soap)
    {
      fprintf(fd, "\n\tif (soap_connect(soap, soap_endpoint, soap_action)");
    }
    else
    {
      if (put)
        fprintf(fd, "\n\tif (soap_PUT(soap, soap_extend_url(soap, soap_endpoint, soap_action), soap_action, soap->version == 2 ? \"application/soap+xml; charset=utf-8\" : \"text/xml; charset=utf-8\")");
      else
        fprintf(fd, "\n\tif (soap_POST(soap, soap_extend_url(soap, soap_endpoint, soap_action), soap_action, soap->version == 2 ? \"application/soap+xml; charset=utf-8\" : \"text/xml; charset=utf-8\")");
    }
    fprintf(fd, "\n\t || soap_envelope_begin_out(soap)");
    if (soap)
    {
      fprintf(fd, "\n\t || soap_putheader(soap)");
      fprintf(fd, "\n\t || soap_body_begin_out(soap)");
    }
    fprintf(fd, "\n\t || soap_put_%s(soap, &soap_tmp_%s, \"%s\", \"\")", ident(method->sym->name), ident(method->sym->name), ns_convert(method->sym->name));
    if (soap)
      fprintf(fd, "\n\t || soap_body_end_out(soap)");
    fprintf(fd, "\n\t || soap_envelope_end_out(soap)");
    fprintf(fd, "\n\t || soap_end_send(soap))");
    fprintf(fd, "\n\t\treturn soap_closesock(soap);");
  }
  else if (get)
  {
    if (params->list)
    {
      gen_query_url(fd, params, soap);
      fprintf(fd, "\n\tif (soap_GET(soap, soap->msgbuf, soap_action))");
    }
    else if (soap)
    {
      fprintf(fd, "\n\tif (soap_GET(soap, soap_endpoint, soap_action))");
    }
    else
    {
      fprintf(fd, "\n\tif (soap_GET(soap, soap_extend_url(soap, soap_endpoint, soap_action), soap_action))");
    }
    fprintf(fd, "\n\t\treturn soap_closesock(soap);");
  }
  else if (del)
  {
    if (params->list)
    {
      gen_query_url(fd, params, soap);
      fprintf(fd, "\n\tif (soap_DELETE(soap, soap->msgbuf))");
    }
    else if (soap)
    {
      fprintf(fd, "\n\tif (soap_DELETE(soap, soap_endpoint))");
    }
    else
    {
      fprintf(fd, "\n\tif (soap_DELETE(soap, soap_extend_url(soap, soap_endpoint, soap_action)))");
    }
    fprintf(fd, "\n\t\treturn soap_closesock(soap);");
  }
  else if (mimein)
  {
    if (put)
      fprintf(fd, "\n\tif (soap_PUT(soap, soap_extend_url(soap, soap_endpoint, soap_action), soap_action, \"application/x-www-form-urlencoded\"))");
    else
      fprintf(fd, "\n\tif (soap_POST(soap, soap_extend_url(soap, soap_endpoint, soap_action), soap_action, \"application/x-www-form-urlencoded\"))");
    fprintf(fd, "\n\t\treturn soap_closesock(soap);");
    gen_query_send_form(fd, params);
  }
  fprintf(fd, "\n\treturn SOAP_OK;\n}");
  if (del)
    return;
  if (is_transient(result->info.typ))
  {
    if (name)
    {
      fprintf(fd, "\n\nint %s::recv_%s(", name, ns_cname(method->sym->name, NULL));
      fprintf(fd, "struct %s& tmp)", ident(method->sym->name));
      if (iflag)
        fprintf(fd, "\n{\n\tstruct soap *soap = this;");
      else
        fprintf(fd, "\n{");
      fprintf(fd, "\n\tstruct %s *%s = &tmp;", ident(method->sym->name), ident(result->sym->name));
    }
    else
    {
      fprintf(fd, "\n\nSOAP_FMAC5 int SOAP_FMAC6 soap_recv_%s(struct soap *soap, ", ident(method->sym->name));
      fprintf(fd, "struct %s *%s)\n{", ident(method->sym->name), ident(result->sym->name));
    }
    if (mimeout)
    {
      fprintf(fd, "\n\tchar *soap_tmp = NULL, *soap_tmp_key = NULL;");
      gen_query_recv_form_init(fd, result);
    }
    fprintf(fd, "\n\tsoap_default_%s(soap, %s);", ident(method->sym->name), ident(result->sym->name));
    fprintf(fd, "\n\tsoap_begin(soap);");
  }
  else
  {
    if (name)
    {
      fprintf(fd, "\n\nint %s::recv_%s(", name, ns_cname(method->sym->name, NULL));
      gen_params_ref(fd, NULL, result, 0);
      if (iflag)
        fprintf(fd, "\n{\n\tstruct soap *soap = this;");
      else
        fprintf(fd, "\n{");
    }
    else
    {
      fprintf(fd, "\n\nSOAP_FMAC5 int SOAP_FMAC6 soap_recv_%s(struct soap *soap", ident(method->sym->name));
      gen_params_ref(fd, NULL, result, 1);
      fprintf(fd, "\n{");
    }
    if (response && !mimeout)
      fprintf(fd, "\n\tstruct %s *soap_tmp_%s;", c_ident(response->info.typ), c_ident(response->info.typ));
    if (mimeout)
    {
      fprintf(fd, "\n\tchar *soap_tmp = NULL, *soap_tmp_key = NULL;");
      gen_query_recv_form_init(fd, result);
    }
    if (result->info.typ->type == Tarray)
      fprintf(fd, "\n\tsoap_default_%s(soap, %s);", c_ident(result->info.typ), ident(result->sym->name));
    else if (result->info.typ->type == Treference && ((Tnode*)result->info.typ->ref)->type == Tclass && !is_external((Tnode*)result->info.typ->ref) && !is_volatile((Tnode*)result->info.typ->ref))
      fprintf(fd, "\n\t%s.soap_default(soap);", ident(result->sym->name));
    else if (((Tnode*)result->info.typ->ref)->type == Tclass && !is_external((Tnode*)result->info.typ->ref) && !is_volatile((Tnode*)result->info.typ->ref))
      fprintf(fd, "\n\tif (!%s)\n\t\treturn soap_closesock(soap);\n\t%s->soap_default(soap);", ident(result->sym->name), ident(result->sym->name));
    else if (result->info.typ->type == Treference && ((Tnode*)result->info.typ->ref)->type == Tpointer)
      fprintf(fd, "\n\t%s = NULL;", ident(result->sym->name));
    else if (((Tnode*)result->info.typ->ref)->type == Tpointer)
      fprintf(fd, "\n\tif (!%s)\n\t\treturn soap_closesock(soap);\n\t*%s = NULL;", ident(result->sym->name), ident(result->sym->name));
    else if (result->info.typ->type == Treference)
      fprintf(fd, "\n\tsoap_default_%s(soap, &%s);", c_ident((Tnode*)result->info.typ->ref), ident(result->sym->name));
    else if (!is_void(result->info.typ))
      fprintf(fd, "\n\tif (!%s)\n\t\treturn soap_closesock(soap);\n\tsoap_default_%s(soap, %s);", ident(result->sym->name), c_ident((Tnode*)result->info.typ->ref), ident(result->sym->name));
  }
  if (mimeout)
  {
    fprintf(fd, "\n\tif (soap_begin_recv(soap)\n\t || (soap_tmp = soap_http_get_form(soap)) == NULL\n\t || soap_end_recv(soap))");
    fprintf(fd, "\n\t\treturn soap_closesock(soap);");
    gen_query_recv_form(fd, result);
    fprintf(fd, "\n\treturn soap_closesock(soap);");
    fprintf(fd , "\n}");
    fflush(fd);
    return;
  }
  fprintf(fd, "\n\tif (soap_begin_recv(soap)");
  fprintf(fd, "\n\t || soap_envelope_begin_in(soap)");
  fprintf(fd, "\n\t || soap_recv_header(soap)");
  fprintf(fd, "\n\t || soap_body_begin_in(soap))");
  fprintf(fd, "\n\t\treturn soap_closesock(soap);");
  if (is_transient(result->info.typ))
  {
    if (sflag)
      fprintf(fd, "\n\tsoap->mode |= SOAP_XML_STRICT;");
    fprintf(fd, "\n\tsoap_get_%s(soap, %s, \"%s\", NULL);", ident(method->sym->name), ident(result->sym->name), ns_convert(method->sym->name));
    fprintf(fd, "\n\tif (soap->error == SOAP_TAG_MISMATCH && soap->level == 2)\n\t\tsoap->error = SOAP_OK;");
    fprintf(fd, "\n\tif (soap->error");
    fprintf(fd, "\n\t || soap_body_end_in(soap)");
    fprintf(fd, "\n\t || soap_envelope_end_in(soap)");
    fprintf(fd, "\n\t || soap_end_recv(soap))");
    fprintf(fd, "\n\t\treturn soap_closesock(soap);");
    fprintf(fd, "\n\treturn soap_closesock(soap);\n}");
    fflush(fd);
    return;
  }
  /* With RPC encoded responses, try to parse the fault first */
  if (soap && !is_literal(method_response_encoding))
  {
    fprintf(fd, "\n\tif (soap_recv_fault(soap, 1))\n\t\treturn soap->error;");
    xtag = "";
  }
  else if (has_ns_eq(NULL, result->sym->name))
  {
    if (response)
      xtag = xml_tag(response->info.typ);
    else
      xtag = ns_convert(result->sym->name);
  }
  else
  {
    if (response)
      xtag = xml_tag(response->info.typ);
    else
      xtag = xml_tag(result->info.typ);
  }
  if (sflag)
    fprintf(fd, "\n\tsoap->mode |= SOAP_XML_STRICT;");
  if (response)
  {
    fprintf(fd, "\n\tsoap_tmp_%s = soap_get_%s(soap, NULL, \"%s\", NULL);", c_ident(response->info.typ), c_ident(response->info.typ), xtag);
    fprintf(fd, "\n\tif (!soap_tmp_%s || soap->error)\n\t\treturn soap_recv_fault(soap, 0);", c_ident(response->info.typ));
  }
  else if ((result->info.typ->type == Treference || result->info.typ->type == Tpointer) && !is_invisible_empty((Tnode*)result->info.typ->ref))
  {
    if (result->info.typ->type == Treference && ((Tnode *) result->info.typ->ref)->type == Tclass && !is_external((Tnode*)result->info.typ->ref) && !is_volatile((Tnode*)result->info.typ->ref) && !is_dynamic_array((Tnode*)result->info.typ->ref))
      fprintf(fd, "\n\t%s.soap_get(soap, \"%s\", NULL);", ident(result->sym->name), xtag);
    else if (result->info.typ->type == Tpointer && ((Tnode *) result->info.typ->ref)->type == Tclass && !is_external((Tnode*)result->info.typ->ref) && !is_volatile((Tnode*)result->info.typ->ref) && !is_dynamic_array((Tnode*)result->info.typ->ref))
      fprintf(fd, "\n\t%s->soap_get(soap, \"%s\", NULL);", ident(result->sym->name), xtag);
    else if (result->info.typ->type == Treference && ((Tnode *) result->info.typ->ref)->type == Tstruct && !is_external((Tnode*)result->info.typ->ref) && !is_volatile((Tnode*)result->info.typ->ref) && !is_dynamic_array((Tnode*)result->info.typ->ref))
    {
      fprintf(fd, "\n\tsoap_get_%s(soap, &%s, \"%s\", NULL);", c_ident((Tnode*)result->info.typ->ref), ident(result->sym->name), xtag);
    }
    else if (result->info.typ->type == Tpointer && ((Tnode *) result->info.typ->ref)->type == Tstruct && !is_dynamic_array((Tnode*)result->info.typ->ref))
    {
      fprintf(fd, "\n\tsoap_get_%s(soap, %s, \"%s\", NULL);", c_ident((Tnode*)result->info.typ->ref), ident(result->sym->name), xtag);
    }
    else if (result->info.typ->type == Tpointer && is_XML((Tnode*)result->info.typ->ref) && is_string((Tnode*)result->info.typ->ref))
    {
      fprintf(fd, "\n\tsoap_inliteral(soap, \"%s\", (char**)%s);", xtag, ident(result->sym->name));
    }
    else if (result->info.typ->type == Treference && is_XML((Tnode*)result->info.typ->ref) && is_string((Tnode*)result->info.typ->ref))
    {
      fprintf(fd, "\n\tsoap_inliteral(soap, \"%s\", (char**)&%s);", xtag, ident(result->sym->name));
    }
    else if (result->info.typ->type == Tpointer && is_XML((Tnode*)result->info.typ->ref) && is_wstring((Tnode*)result->info.typ->ref))
    {
      fprintf(fd, "\n\tsoap_inwliteral(soap, \"%s\", (wchar_t**)%s);", xtag, ident(result->sym->name));
    }
    else if (result->info.typ->type == Treference && is_XML((Tnode*)result->info.typ->ref) && is_wstring((Tnode*)result->info.typ->ref))
    {
      fprintf(fd, "\n\tsoap_inwliteral(soap, \"%s\", (wchar_t**)&%s);", xtag, ident(result->sym->name));
    }
    else if (result->info.typ->type == Treference)
    {
      fprintf(fd, "\n\tsoap_get_%s(soap, &%s, \"%s\", NULL);", c_ident(result->info.typ), ident(result->sym->name), xtag);
    }
    else
    {
      fprintf(fd, "\n\tsoap_get_%s(soap, %s, \"%s\", NULL);", c_ident(result->info.typ), ident(result->sym->name), xtag);
    }
    fprintf(fd, "\n\tif (soap->error)\n\t\treturn soap_recv_fault(soap, 0);");
  }
  else if (soap && is_literal(method_response_encoding))
  {
    fprintf(fd, "\n\tif (soap_recv_fault(soap, 1))\n\t\treturn soap->error;");
  }
  fflush(fd);
  fprintf(fd, "\n\tif (soap_body_end_in(soap)");
  fprintf(fd, "\n\t || soap_envelope_end_in(soap)");
  fprintf(fd, "\n\t || soap_end_recv(soap))");
  fprintf(fd, "\n\t\treturn soap_closesock(soap);");
  if (response)
  {
    if (result->info.typ->type == Tarray)
      fprintf(fd, "\n\tsoap_memcpy(%s, sizeof(%s), soap_tmp_%s->%s, sizeof(%s));", ident(result->sym->name), c_ident(response->info.typ), c_type(result->info.typ), ident(result->sym->name), c_type(result->info.typ));
    else if (result->info.typ->type == Treference)
      fprintf(fd, "\n\t%s = soap_tmp_%s->%s;", ident(result->sym->name), c_ident(response->info.typ), ident(result->sym->name));
    else
    {
      fprintf(fd, "\n\tif (%s && soap_tmp_%s->%s)", ident(result->sym->name), c_ident(response->info.typ), ident(result->sym->name));
      fprintf(fd, "\n\t\t*%s = *soap_tmp_%s->%s;", ident(result->sym->name), c_ident(response->info.typ), ident(result->sym->name));
    }
  }
  fprintf(fd, "\n\treturn soap_closesock(soap);");
  fprintf(fd , "\n}");
  fflush(fd);
}

void
gen_serve_method(FILE *fd, Table *table, Entry *param, const char *name)
{
  Service *sp = NULL;
  const char *encoding;
  Entry *result, *p, *q, *pin, *pout, *response = NULL;
  Table *input;
  const char *xtag;
  Method *m;
  const char *method_encoding = NULL, *method_response_encoding = NULL;
  result = (Entry*)param->info.typ->ref;
  p = entry(classtable, param->sym);
  if (!p)
    execerror("no table entry");
  if (!is_response(result->info.typ) && !is_XML(result->info.typ))
    response = get_response(param->info.typ);
  q = entry(table, param->sym);
  if (!q)
    execerror("no table entry");
  pout = (Entry*)q->info.typ->ref;
  if (!pout)
    execerror("no table entry");
  if (name)
  {
    if (iflag)
      fprintf(fd, "\n\nstatic int serve_%s(%s *soap)\n{", ident(param->sym->name), name);
    else
      fprintf(fd, "\n\nstatic int serve_%s(struct soap *soap, %s *service)\n{", ident(param->sym->name), name);
  }
  else
  {
    fprintf(fheader, "\n\nSOAP_FMAC5 int SOAP_FMAC6 soap_serve_%s(struct soap*);", ident(param->sym->name));
    fprintf(fd, "\n\nSOAP_FMAC5 int SOAP_FMAC6 soap_serve_%s(struct soap *soap)\n{", ident(param->sym->name));
  }
  fprintf(fd, "\tstruct %s soap_tmp_%s;", ident(param->sym->name), ident(param->sym->name));
  for (sp = services; sp; sp = sp->next)
  {
    if (has_ns_eq(sp->ns, param->sym->name))
    {
      encoding = sp->encoding;
      method_encoding = encoding;
      method_response_encoding = NULL;
      for (m = sp->list; m; m = m->next)
      {
        if (is_eq_nons(m->name, param->sym->name))
        {
          if (m->mess == ENCODING)
            method_encoding = m->part;
          else if (m->mess == RESPONSE_ENCODING)
            method_response_encoding = m->part;
        }
      }
      break;
    }
  }
  if (!method_response_encoding)
    method_response_encoding = method_encoding;
  fflush(fd);
  if (!is_transient(pout->info.typ))
  {
    if (pout->info.typ->type == Tarray && response)
    {
      fprintf(fd, "\n\tstruct %s soap_tmp_%s;", c_ident(response->info.typ), c_ident(response->info.typ));
      fprintf(fd, "\n\tsoap_default_%s(soap, &soap_tmp_%s);", c_ident(response->info.typ), c_ident(response->info.typ));
    }
    else if (pout->info.typ->type == Tpointer && !is_stdstring(pout->info.typ) && !is_stdwstring(pout->info.typ) && response)
    {
      fprintf(fd, "\n\tstruct %s soap_tmp_%s;", c_ident(response->info.typ), c_ident(response->info.typ));
      fprintf(fd, "\n\t%s soap_tmp_%s;", c_type((Tnode*)pout->info.typ->ref), c_ident((Tnode*)pout->info.typ->ref));
      fprintf(fd, "\n\tsoap_default_%s(soap, &soap_tmp_%s);", c_ident(response->info.typ), c_ident(response->info.typ));
      if (((Tnode*)pout->info.typ->ref)->type == Tclass && !is_external((Tnode*)pout->info.typ->ref) && !is_volatile((Tnode*)pout->info.typ->ref) && !is_typedef((Tnode*)pout->info.typ->ref))
        fprintf(fd, "\n\tsoap_tmp_%s.soap_default(soap);", c_ident((Tnode*)pout->info.typ->ref));
      else if (((Tnode*)pout->info.typ->ref)->type == Tpointer)
        fprintf(fd, "\n\tsoap_tmp_%s = NULL;", c_ident((Tnode*)pout->info.typ->ref));
      else
        fprintf(fd, "\n\tsoap_default_%s(soap, &soap_tmp_%s);", c_ident((Tnode*)pout->info.typ->ref), c_ident((Tnode*)pout->info.typ->ref));
      fprintf(fd, "\n\tsoap_tmp_%s.%s = &soap_tmp_%s;", c_ident(response->info.typ), ident(pout->sym->name), c_ident((Tnode*)pout->info.typ->ref));
    }
    else if (response)
    {
      fprintf(fd, "\n\tstruct %s soap_tmp_%s;", c_ident(response->info.typ), c_ident(response->info.typ));
      fprintf(fd, "\n\tsoap_default_%s(soap, &soap_tmp_%s);", c_ident(response->info.typ), c_ident(response->info.typ));
    }
    else if (((Tnode *)pout->info.typ->ref)->type == Tclass && !is_stdstring((Tnode*)pout->info.typ->ref) && !is_stdwstring((Tnode*)pout->info.typ->ref) && (is_external((Tnode*)pout->info.typ->ref) || is_volatile((Tnode*)pout->info.typ->ref) || is_typedef((Tnode*)pout->info.typ->ref)) && !is_dynamic_array((Tnode*)pout->info.typ->ref))
    {
      fprintf(fd, "\n\t%s %s;", c_type((Tnode*)pout->info.typ->ref), ident(pout->sym->name));
      fprintf(fd, "\n\tsoap_default_%s(soap, &%s);", c_ident((Tnode*)pout->info.typ->ref), ident(pout->sym->name));
    }
    else if (((Tnode *)pout->info.typ->ref)->type == Tclass && !is_stdstring((Tnode*)pout->info.typ->ref) && !is_stdwstring((Tnode*)pout->info.typ->ref) && !is_dynamic_array((Tnode*)pout->info.typ->ref))
    {
      fprintf(fd, "\n\t%s %s;", c_type((Tnode*)pout->info.typ->ref), ident(pout->sym->name));
      fprintf(fd, "\n\t%s.soap_default(soap);", ident(pout->sym->name));
    }
    else if (((Tnode *)pout->info.typ->ref)->type == Tstruct && !is_dynamic_array((Tnode*)pout->info.typ->ref))
    {
      fprintf(fd, "\n\t%s %s;", c_type((Tnode*)pout->info.typ->ref), ident(pout->sym->name));
      fprintf(fd, "\n\tsoap_default_%s(soap, &%s);", c_ident((Tnode *)pout->info.typ->ref), ident(pout->sym->name));
    }
    else
    {
      fprintf(fd, "\n\t%s soap_tmp_%s;", c_type((Tnode*)pout->info.typ->ref), c_ident((Tnode*)pout->info.typ->ref));
      if (is_string((Tnode*)pout->info.typ->ref) || is_wstring((Tnode*)pout->info.typ->ref))
        fprintf(fd, "\n\tsoap_tmp_%s = NULL;", c_ident((Tnode*)pout->info.typ->ref));
      else
        fprintf(fd, "\n\tsoap_default_%s(soap, &soap_tmp_%s);", c_ident((Tnode*)pout->info.typ->ref), c_ident((Tnode*)pout->info.typ->ref));
    }
  }
  fprintf(fd, "\n\tsoap_default_%s(soap, &soap_tmp_%s);", ident(param->sym->name), ident(param->sym->name));
  fflush(fd);
  q = entry(classtable, param->sym);
  if (!is_invisible_empty(q->info.typ))
  {
    fprintf(fd, "\n\tif (!soap_get_%s(soap, &soap_tmp_%s, \"%s\", NULL))", ident(param->sym->name), ident(param->sym->name), ns_convert(param->sym->name));
    fprintf(fd, "\n\t\treturn soap->error;");
  }
  fprintf(fd, "\n\tif (soap_body_end_in(soap)");
  fprintf(fd, "\n\t || soap_envelope_end_in(soap)");
  fprintf(fd, "\n\t || soap_end_recv(soap))\n\t\treturn soap->error;");
  if (name)
  {
    if (iflag)
      fprintf(fd, "\n\tsoap->error = soap->%s(", ns_cname(param->sym->name, NULL));
    else
      fprintf(fd, "\n\tsoap->error = service->%s(", ns_cname(param->sym->name, NULL));
  }
  else
    fprintf(fd, "\n\tsoap->error = %s(soap", ident(param->sym->name));
  fflush(fd);
  input = (Table*) q->info.typ->ref;
  if (!input)
    execerror("no table entry");
  for (pin = input->list; pin; pin = pin->next)
  {
    if (pin->info.typ->type == Trvalueref)
      fprintf(fd, "%sstd::move(soap_tmp_%s.%s)", !name || pin != input->list ? ", " : "", ident(param->sym->name), ident(pin->sym->name));
    else
      fprintf(fd, "%ssoap_tmp_%s.%s", !name || pin != input->list ? ", " : "", ident(param->sym->name), ident(pin->sym->name));
  }
  if (is_transient(pout->info.typ))
    fprintf(fd, ");");
  else
  {
    if (!name || input->list)
      fprintf(fd, ", ");
    if (response)
      fprintf(fd, "soap_tmp_%s.%s);", c_ident(response->info.typ), ident(pout->sym->name));
    else if (pout->info.typ->type == Treference && (((Tnode*)pout->info.typ->ref)->type == Tstruct || ((Tnode*)pout->info.typ->ref)->type == Tclass) && !is_stdstring((Tnode*)pout->info.typ->ref) && !is_stdwstring((Tnode*)pout->info.typ->ref) && !is_dynamic_array((Tnode*)pout->info.typ->ref))
      fprintf(fd, "%s);", ident(pout->sym->name));
    else if ((((Tnode*)pout->info.typ->ref)->type == Tstruct || ((Tnode*)pout->info.typ->ref)->type == Tclass) && !is_stdstring((Tnode*)pout->info.typ->ref) && !is_stdwstring((Tnode*)pout->info.typ->ref) && !is_dynamic_array((Tnode*)pout->info.typ->ref))
      fprintf(fd, "&%s);", ident(pout->sym->name));
    else if (pout->info.typ->type == Treference)
      fprintf(fd, "soap_tmp_%s);", c_ident((Tnode*)pout->info.typ->ref));
    else
      fprintf(fd, "&soap_tmp_%s);", c_ident((Tnode*)pout->info.typ->ref));
  }
  fprintf(fd, "\n\tif (soap->error)\n\t\treturn soap->error;");
  if (!is_transient(pout->info.typ))
  {
    if (sp && method_response_encoding)
    {
      if (is_literal(method_response_encoding))
        fprintf(fd, "\n\tsoap->encodingStyle = NULL; /* use SOAP literal style */");
      else if (sp->encoding)
        fprintf(fd, "\n\tsoap->encodingStyle = \"%s\"; /* use SOAP encoding style */", sp->encoding);
      else if (method_response_encoding)
        fprintf(fd, "\n\tsoap->encodingStyle = \"%s\"; /* use SOAP encoding style */", method_response_encoding);
      else if (!eflag)
        fprintf(fd, "\n\tsoap->encodingStyle = NULL; /* use SOAP literal style */");
    }
    else if (!eflag)
    {
      fprintf(fd, "\n\tsoap->encodingStyle = NULL; /* use SOAP literal style */");
    }
    fprintf(fd, "\n\tsoap_serializeheader(soap);");
    if (pout->info.typ->type == Tarray && response)
      fprintf(fd, "\n\tsoap_serialize_%s(soap, &soap_tmp_%s);", c_ident(response->info.typ), c_ident(response->info.typ));
    else if (response)
      fprintf(fd, "\n\tsoap_serialize_%s(soap, &soap_tmp_%s);", c_ident(response->info.typ), c_ident(response->info.typ));
    else if (((Tnode *)pout->info.typ->ref)->type == Tclass && !is_stdstring((Tnode*)pout->info.typ->ref) && !is_stdwstring((Tnode*)pout->info.typ->ref) && (is_external((Tnode*)pout->info.typ->ref) || is_volatile((Tnode*)pout->info.typ->ref) || is_typedef((Tnode*)pout->info.typ->ref)) && !is_dynamic_array((Tnode*)pout->info.typ->ref))
      fprintf(fd, "\n\tsoap_serialize_%s(soap, &%s);", c_ident((Tnode*)pout->info.typ->ref), ident(pout->sym->name));
    else if (((Tnode *)pout->info.typ->ref)->type == Tclass && !is_stdstring((Tnode*)pout->info.typ->ref) && !is_stdwstring((Tnode*)pout->info.typ->ref) && !is_dynamic_array((Tnode*)pout->info.typ->ref))
      fprintf(fd, "\n\t%s.soap_serialize(soap);", ident(pout->sym->name));
    else if (((Tnode *)pout->info.typ->ref)->type == Tstruct && !is_dynamic_array((Tnode*)pout->info.typ->ref))
      fprintf(fd, "\n\tsoap_serialize_%s(soap, &%s);", c_ident((Tnode*)pout->info.typ->ref), ident(pout->sym->name));
    else if (!is_XML((Tnode*)pout->info.typ->ref))
      fprintf(fd, "\n\tsoap_serialize_%s(soap, &soap_tmp_%s);", c_ident((Tnode*)pout->info.typ->ref), c_ident((Tnode*)pout->info.typ->ref));
    if (is_literal(method_response_encoding) && has_ns_eq(NULL, pout->sym->name))
      xtag = ns_convert(pout->sym->name);
    else
      xtag = xml_tag(pout->info.typ);
    fprintf(fd, "\n\tif (soap_begin_count(soap))\n\t\treturn soap->error;");
    fprintf(fd, "\n\tif ((soap->mode & SOAP_IO_LENGTH))");
    fprintf(fd, "\n\t{\tif (soap_envelope_begin_out(soap)");
    fprintf(fd, "\n\t\t || soap_putheader(soap)");
    fprintf(fd, "\n\t\t || soap_body_begin_out(soap)");
    if (response)
      fprintf(fd, "\n\t\t || soap_put_%s(soap, &soap_tmp_%s, \"%s\", \"\")", c_ident(response->info.typ), c_ident(response->info.typ), xml_tag(response->info.typ));
    else if (((Tnode*)pout->info.typ->ref)->type == Tclass && !is_stdstring((Tnode*)pout->info.typ->ref) && !is_stdwstring((Tnode*)pout->info.typ->ref) && (is_external((Tnode*)pout->info.typ->ref) || is_volatile((Tnode*)pout->info.typ->ref) || is_typedef((Tnode*)pout->info.typ->ref)) && !is_dynamic_array((Tnode*)pout->info.typ->ref))
      fprintf(fd, "\n\t\t || soap_put_%s(soap, &%s, \"%s\", \"\")", c_ident((Tnode*)pout->info.typ->ref), ident(pout->sym->name), ns_convert(pout->sym->name));
    else if (((Tnode*)pout->info.typ->ref)->type == Tclass && !is_stdstring((Tnode*)pout->info.typ->ref) && !is_stdwstring((Tnode*)pout->info.typ->ref) && !is_dynamic_array((Tnode*)pout->info.typ->ref))
      fprintf(fd, "\n\t\t || %s.soap_put(soap, \"%s\", \"\")", ident(pout->sym->name), xtag);
    else if (((Tnode*)pout->info.typ->ref)->type == Tstruct && !is_dynamic_array((Tnode*)pout->info.typ->ref))
      fprintf(fd, "\n\t\t || soap_put_%s(soap, &%s, \"%s\", \"\")", c_ident((Tnode*)pout->info.typ->ref), ident(pout->sym->name), xtag);
    else if (is_XML((Tnode*)pout->info.typ->ref) && is_string((Tnode*)pout->info.typ->ref))
      fprintf(fd, "\n\t\t || soap_outliteral(soap, \"%s\", &soap_tmp_%s, NULL)", ns_convert(pout->sym->name), c_ident((Tnode*)pout->info.typ->ref));
    else if (is_XML((Tnode*)pout->info.typ->ref) && is_wstring((Tnode*)pout->info.typ->ref))
      fprintf(fd, "\n\t\t || soap_outwliteral(soap, \"%s\", &soap_tmp_%s, NULL)", ns_convert(pout->sym->name), c_ident((Tnode*)pout->info.typ->ref));
    else
      fprintf(fd, "\n\t\t || soap_put_%s(soap, &soap_tmp_%s, \"%s\", \"\")", c_ident(pout->info.typ), c_ident((Tnode*)pout->info.typ->ref), ns_convert(pout->sym->name));
    fprintf(fd, "\n\t\t || soap_body_end_out(soap)");
    fprintf(fd, "\n\t\t || soap_envelope_end_out(soap))");
    fprintf(fd, "\n\t\t\t return soap->error;");
    fprintf(fd, "\n\t};");
    fprintf(fd, "\n\tif (soap_end_count(soap)");
    fprintf(fd, "\n\t || soap_response(soap, SOAP_OK)");
    fprintf(fd, "\n\t || soap_envelope_begin_out(soap)");
    fprintf(fd, "\n\t || soap_putheader(soap)");
    fprintf(fd, "\n\t || soap_body_begin_out(soap)");
    if (response)
      fprintf(fd, "\n\t || soap_put_%s(soap, &soap_tmp_%s, \"%s\", \"\")", c_ident(response->info.typ), c_ident(response->info.typ), xml_tag(response->info.typ));
    else if (((Tnode *)pout->info.typ->ref)->type == Tclass && !is_stdstring((Tnode*)pout->info.typ->ref) && !is_stdwstring((Tnode*)pout->info.typ->ref) && (is_external((Tnode*)pout->info.typ->ref) || is_volatile((Tnode*)pout->info.typ->ref) || is_typedef((Tnode*)pout->info.typ->ref)) && !is_dynamic_array((Tnode*)pout->info.typ->ref))
      fprintf(fd, "\n\t || soap_put_%s(soap, &%s, \"%s\", \"\")", c_ident((Tnode*)pout->info.typ->ref), ident(pout->sym->name), ns_convert(pout->sym->name));
    else if (((Tnode *)pout->info.typ->ref)->type == Tclass && !is_stdstring((Tnode*)pout->info.typ->ref) && !is_stdwstring((Tnode*)pout->info.typ->ref) && !is_dynamic_array((Tnode*)pout->info.typ->ref))
      fprintf(fd, "\n\t || %s.soap_put(soap, \"%s\", \"\")", ident(pout->sym->name), xtag);
    else if (((Tnode *)pout->info.typ->ref)->type == Tstruct && !is_dynamic_array((Tnode*)pout->info.typ->ref))
      fprintf(fd, "\n\t || soap_put_%s(soap, &%s, \"%s\", \"\")", c_ident((Tnode*)pout->info.typ->ref), ident(pout->sym->name), xtag);
    else if (is_XML((Tnode*)pout->info.typ->ref) && is_string((Tnode*)pout->info.typ->ref))
      fprintf(fd, "\n\t || soap_outliteral(soap, \"%s\", &soap_tmp_%s, NULL)", ns_convert(pout->sym->name), c_ident((Tnode*)pout->info.typ->ref));
    else if (is_XML((Tnode*)pout->info.typ->ref) && is_wstring((Tnode*)pout->info.typ->ref))
      fprintf(fd, "\n\t || soap_outwliteral(soap, \"%s\", &soap_tmp_%s, NULL)", ns_convert(pout->sym->name), c_ident((Tnode*)pout->info.typ->ref));
    else
      fprintf(fd, "\n\t || soap_put_%s(soap, &soap_tmp_%s, \"%s\", \"\")", c_ident(pout->info.typ), c_ident((Tnode*)pout->info.typ->ref), ns_convert(pout->sym->name));
    fprintf(fd, "\n\t || soap_body_end_out(soap)");
    fprintf(fd, "\n\t || soap_envelope_end_out(soap)");
    fprintf(fd, "\n\t || soap_end_send(soap))");
    fprintf(fd, "\n\t\treturn soap->error;");
  }
  fprintf(fd, "\n\treturn soap_closesock(soap);");
  fprintf(fd, "\n}");
  fflush(fd);
}

void
gen_response_begin(FILE *fd, int n, const char *s, int soap)
{
  if (!is_invisible(s))
  {
    fprintf(fd, "%*s<%sResponse", n, "", s);
    if (!soap || soap_version < 0)
      gen_xmlns(fd, 0);
    fprintf(fd, ">\n");
  }
}

void
gen_response_end(FILE *fd, int n, const char *s)
{
  if (!is_invisible(s))
    fprintf(fd, "%*s</%sResponse>\n", n, "", s);
}

void
gen_element_begin(FILE *fd, int n, const char *s, Entry *p)
{
  if (!is_invisible(s))
  {
    fprintf(fd, "%*s<%s", n, "", s);
    if (p)
    {
      const char *x = xsi_type_u(p->info.typ);
      if (x && *x)
        fprintf(fd, " xsi:type=\"%s\"", x);
    }
    if (soap_version >= 0 && p && (p->info.sto & SmustUnderstand))
      fprintf(fd, " SOAP-ENV:mustUnderstand=\"%s\"", soap_version == 2 ? "true" : "1");
  }
}

void
gen_element_array(FILE *fd, int n, const char *s)
{
  if (!is_invisible(s))
  {
    fprintf(fd, "%*s<%s", n, "", s);
    if (tflag)
      fprintf(fd, " xsi:type=\"SOAP-ENC:Array\"");
  }
}

void
gen_element_end(FILE *fd, int n, const char *s)
{
  if (!is_invisible(s))
    fprintf(fd, "%*s</%s>\n", n, "", s);
}

void
gen_data(const char *buf, Table *t, const char *ns, const char *encoding)
{
  Entry *p, *q, *r;
  FILE *fd;
  const char *method_encoding = NULL;
  const char *method_response_encoding = NULL;
  if (t)
  {
    for (p = t->list; p; p = p->next)
    {
      if (p->info.typ->type == Tfun && !(p->info.sto & Sextern) && has_ns_eq(ns, p->sym->name))
      {
        int get = 0, put = 0, soap = 1, mimein = 0, mimeout = 0;
        Service *sp;
        Method *m;
        const char *nse = ns_qualifiedElement(p->info.typ);
        const char *nsa = ns_qualifiedAttribute(p->info.typ);
        method_encoding = encoding;
        method_response_encoding = NULL;
        for (sp = services; sp; sp = sp->next)
        {
          if (!tagcmp(sp->ns, ns))
          {
            for (m = sp->list; m; m = m->next)
            {
              if (is_eq_nons(m->name, p->sym->name))
              {
                if (m->mess == ENCODING)
                  method_encoding = m->part;
                else if (m->mess == RESPONSE_ENCODING)
                  method_response_encoding = m->part;
                else if (m->mess == PROTOCOL)
                {
                  if (strncmp(m->part, "SOAP", 4))
                    soap = 0;
                  if (strstr(m->part, "GET"))
                    get = 1;
                  else if (strstr(m->part, "PUT"))
                    put = 1;
                }
                else
                {
                  if ((m->mess&MIMEIN) && !strcmp(m->part, "application/x-www-form-urlencoded"))
                    mimein = 1;
                  if ((m->mess&MIMEOUT) && !strcmp(m->part, "application/x-www-form-urlencoded"))
                    mimeout = 1;
                }
              }
            }
          }
        }
        if (!method_response_encoding)
          method_response_encoding = method_encoding;
        /* request */
        if (!get && !mimein)
        {
          fd = gen_env(buf, ns_remove(p->sym->name), 0, method_encoding, soap);
          if (!fd)
            return;
          q = entry(classtable, p->sym);
          if (yflag)
          {
            fprintf(fd, "%*s<!-- %s(...", 2, "", ident(p->sym->name));
            if (q)
            {
              Table *r = (Table*)q->info.typ->ref;
              while (r)
              {
                Entry *e;
                for (e = r->list; e; e = e->next)
                  fprintf(fd, ", %s", c_type_id(e->info.typ, e->sym->name));
                r = r->prev;
              }
            }
            fprintf(fd, ", ...) -->\n");
          }
          if (!is_invisible(p->sym->name))
          {
            gen_element_begin(fd, 2, ns_convert(p->sym->name), NULL);
            if (!soap || soap_version < 0)
              gen_xmlns(fd, 0);
          }
          if (q)
          {
            int xmlns = 0;
            if (!is_invisible(p->sym->name))
            {
              gen_atts(fd, (Table*)q->info.typ->ref, nse, nsa, encoding);
              fprintf(fd, "\n");
            }
            else if (!soap || soap_version < 0)
              xmlns = 1;
            for (q = ((Table*)q->info.typ->ref)->list; q; q = q->next)
              gen_field(fd, 3, q, nse, nsa, method_encoding, q->info.minOccurs == 0, xmlns);
          }
          if (!is_invisible(p->sym->name))
            gen_element_end(fd, 2, ns_convert(p->sym->name));
          if (soap && soap_version >= 0)
            fprintf(fd, " </SOAP-ENV:Body>\n</SOAP-ENV:Envelope>\n");
          if (fclose(fd))
            execerror("Cannot write to file");
        }
        /* response */
        q = (Entry*)p->info.typ->ref;
        if (!put && !mimeout && q && !is_transient(q->info.typ))
        {
          fd = gen_env(buf, ns_remove(p->sym->name), 1, method_response_encoding, soap);
          if (!fd)
            return;
          if (!is_response(q->info.typ))
          {
            if (is_XML((Tnode*)q->info.typ->ref))
            {
              gen_response_begin(fd, 2, ns_convert(p->sym->name), soap);
              gen_response_end(fd, 2, ns_convert(p->sym->name));
            }
            else
            {
              gen_response_begin(fd, 2, ns_convert(p->sym->name), soap);
              gen_field(fd, 3, q, nse, nsa, method_response_encoding, q->info.minOccurs == 0, 0);
              gen_response_end(fd, 2, ns_convert(p->sym->name));
            }
          }
          else if (q->info.typ->ref && ((Tnode*)q->info.typ->ref)->ref)
          {
            const char *xtag;
            nse = ns_qualifiedElement((Tnode*)q->info.typ->ref);
            nsa = ns_qualifiedAttribute((Tnode*)q->info.typ->ref);
            if (is_literal(method_response_encoding) && has_ns_eq(NULL, q->sym->name))
              xtag = ns_convert(q->sym->name);
            else
              xtag = xml_tag(q->info.typ);
            if (yflag)
              fprintf(fd, "%*s<!-- %s(..., %s) -->\n", 2, "", ident(p->sym->name), c_type_id(q->info.typ, q->sym->name));
            if (!is_invisible(xtag))
            {
              gen_element_begin(fd, 2, xtag, NULL);
              if (!soap || soap_version < 0)
                gen_xmlns(fd, 0);
              gen_atts(fd, (Table*)((Tnode*)q->info.typ->ref)->ref, nse, nsa, encoding);
              fprintf(fd, "\n");
            }
            for (r = ((Table*)((Tnode*)q->info.typ->ref)->ref)->list; r; r = r->next)
              gen_field(fd, 3, r, nse, nsa, method_response_encoding, r->info.minOccurs == 0, 0);
            if (!is_invisible(xtag))
              gen_element_end(fd, 2, ns_addx(xtag, nse));
          }
          fflush(fd);
          if (soap && soap_version >= 0)
            fprintf(fd, " </SOAP-ENV:Body>\n</SOAP-ENV:Envelope>\n");
          if (fclose(fd))
            execerror("Cannot write to file");
        }
      }
    }
  }
}

void
gen_field(FILE *fd, int n, Entry *p, const char *nse, const char *nsa, const char *encoding, int opt, int xmlns)
{
  Entry *q;
  char tmp[32];
  LONG64 i;
  int d;
  if (!(p->info.sto & (Sattribute | Sconst | Sprivate | Sprotected | Sspecial)) && !is_transient(p->info.typ) && p->info.typ->type != Tfun && strncmp(p->sym->name, "__size", 6) && strncmp(p->sym->name, "__type", 6) && !is_choice(p))
  {
    if (n > 8 && (p->info.typ->type == Tpointer || is_smart(p->info.typ)) && p->info.nillable && !is_string(p->info.typ) && !is_wstring(p->info.typ))
    {
      gen_element_begin(fd, n, ns_add(p, nse), NULL);
      if (xmlns)
        gen_xmlns(fd, 0);
      fprintf(fd, " xsi:nil=\"true\"/>\n");
      return;
    }
    if (n > 8 && opt && !is_string(p->info.typ) && !is_wstring(p->info.typ))
      return;
    if (n >= 16)
    {
      fprintf(fd, "%*s<!-- WARNING max depth of 16 levels exceeded: schema may incorrectly define indefinitely large documents in recursion over required elements with minOccurs>0 -->\n", n, "");
      return;
    }
    if (yflag)
      fprintf(fd, "%*s<!-- %s -->\n", n, "", c_type_id(p->info.typ, p->sym->name));
    if (is_container(p->info.typ))
    {
      if (gflag)
      {
        fprintf(fd, "%*s<__REPEAT min=\"" SOAP_LONG_FORMAT "\"", n, "", p->info.minOccurs);
        if (p->info.maxOccurs > 1)
          fprintf(fd, " max=\"" SOAP_LONG_FORMAT "\"", p->info.maxOccurs);
        else
          fprintf(fd, " max=\"unbounded\"");
        fprintf(fd, ">\n");
      }
      else if (yflag)
      {
        fprintf(fd, "%*s<!-- a repetition of " SOAP_LONG_FORMAT, n, "", p->info.minOccurs);
        if (p->info.maxOccurs == p->info.minOccurs && p->info.maxOccurs > 1)
          fprintf(fd, " of the following");
        else if (p->info.maxOccurs > 1)
          fprintf(fd, " to " SOAP_LONG_FORMAT " of the following", p->info.maxOccurs);
        else
          fprintf(fd, " or more of the following");
        fprintf(fd, " -->\n");
      }
      opt = 0;
    }
    if (is_soap12(encoding) && (p->info.sto & Sreturn) && (nse || has_ns_eq(NULL, p->sym->name)) && !is_literal(encoding))
      fprintf(fd, "%*s<SOAP-RPC:result xmlns:SOAP-RPC=\"%s\">%s</SOAP-RPC:result>\n", n, "", rpcURI, ns_add(p, nse));
    if (is_XML(p->info.typ))
    {
      if (!is_invisible(p->sym->name))
      {
        gen_element_begin(fd, n, ns_add(p, nse), NULL);
        if (xmlns)
          gen_xmlns(fd, 0);
        fprintf(fd, ">");
        gen_element_end(fd, n, ns_add(p, nse));
      }
      else
      {
        fprintf(fd, "%*s<!-- extensibility element(s) -->\n", n, "");
      }
    }
    else
    {
      if (is_fixedstring(p->info.typ))
      {
        gen_element_begin(fd, n, ns_add(p, nse), p);
        if (xmlns)
          gen_xmlns(fd, 0);
        fprintf(fd, ">");
        fflush(fd);
        if ((p->info.hasval || p->info.ptrval) && p->info.val.s)
          fprintf(fd, "%s", xstring(p->info.val.s));
        else
          gen_val(fd, n, p->info.typ, nse, nsa, encoding, opt);
      }
      else if (p->info.typ->type == Tarray)
      {
        i = ((Tnode*) p->info.typ->ref)->width;
        if (i)
        {
          i = p->info.typ->width / i;
          if (i > 100000) /* SOAP_MAXOCCURS */
            i = 100000;
        }
        gen_element_array(fd, n, ns_add(p, nse));
        if (is_soap12(encoding))
          fprintf(fd, " SOAP-ENC:itemType=\"%s\" SOAP-ENC:arraySize=\"" SOAP_LONG_FORMAT "\"", xsi_type_Tarray(p->info.typ), i);
        else if (!is_literal(encoding))
          fprintf(fd, " SOAP-ENC:arrayType=\"%s[" SOAP_LONG_FORMAT "]\"", xsi_type_Tarray(p->info.typ), i);
        fprintf(fd, ">");
        fflush(fd);
        gen_val(fd, n, p->info.typ, nse, nsa, encoding, opt);
      }
      else if (is_binary(p->info.typ))
      {
        gen_element_begin(fd, n, ns_add(p, nse), p);
        if (xmlns)
          gen_xmlns(fd, 0);
        fprintf(fd, ">");
        fflush(fd);
        if ((p->info.hasval || p->info.ptrval) && p->info.val.s)
          fprintf(fd, "%s", xstring(p->info.val.s));
        else
          gen_val(fd, n, p->info.typ, nse, nsa, encoding, opt);
      }
      else if ((q = is_dynamic_array(p->info.typ)))
      {
        if (!eflag && (has_ns(p->info.typ) || is_untyped(p->info.typ)))
        {
          gen_element_begin(fd, n, ns_add(p, nse), p);
          if (xmlns)
            gen_xmlns(fd, 0);
          gen_atts(fd, (Table*)p->info.typ->ref, nse, nsa, encoding);
          fprintf(fd, "\n");
          gen_val(fd, n, p->info.typ, nse, nsa, encoding, 0);
          if (!is_invisible(p->sym->name))
            fprintf(fd, "%*s", n, "");
        }
        else
        {
          d = get_Darraydims(p->info.typ);
          if (d)
          {
            for (i = 0; i < d-1; i++)
            {
              if (is_soap12(encoding))
                tmp[2*i] = ' ';
              else
                tmp[2*i] = ',';
              tmp[2*i+1] = '1';
            }
            tmp[2*d-2] = '\0';
          }
          else
            *tmp = '\0';
          gen_element_array(fd, n, ns_add(p, nse));
          i = q->info.minOccurs < 100000 ? q->info.minOccurs : 100000;
          if (i < 1)
            i = 1;
          if (is_soap12(encoding))
            fprintf(fd, " SOAP-ENC:itemType=\"%s\" SOAP-ENC:arraySize=\"" SOAP_LONG_FORMAT "%s\"", wsdl_type(q->info.typ, ""), i, tmp);
          else if (!is_literal(encoding))
            fprintf(fd, " SOAP-ENC:arrayType=\"%s[" SOAP_LONG_FORMAT "%s]\"", wsdl_type(q->info.typ, ""), i, tmp);
          fprintf(fd, ">\n");
          gen_val(fd, n, p->info.typ, nse, nsa, encoding, q->info.minOccurs == 0);
          fprintf(fd, "%*s", n, "");
        }
        fflush(fd);
      }
      else if ((p->info.typ->type == Tpointer || is_smart(p->info.typ) || p->info.typ->type == Treference || p->info.typ->type == Trvalueref) && (q = is_dynamic_array((Tnode*)p->info.typ->ref)) && !is_binary((Tnode*)p->info.typ->ref))
      {
        if (!eflag && (has_ns((Tnode*)p->info.typ->ref) || is_untyped((Tnode*)p->info.typ->ref)))
        {
          gen_element_begin(fd, n, ns_add(p, nse), p);
          if (xmlns)
            gen_xmlns(fd, 0);
          gen_atts(fd, (Table*)((Tnode*)p->info.typ->ref)->ref, nse, nsa, encoding);
          fprintf(fd, "\n");
          gen_val(fd, n, (Tnode*)p->info.typ->ref, nse, nsa, encoding, 0);
          if (!is_invisible(p->sym->name))
            fprintf(fd, "%*s", n, "");
        }
        else
        {
          d = get_Darraydims((Tnode*)p->info.typ->ref);
          if (d)
          {
            for (i = 0; i < d-1; i++)
            {
              tmp[2*i] = ',';
              tmp[2*i+1] = '1';
            }
            tmp[2*d-2] = '\0';
          }
          else
            *tmp = '\0';
          gen_element_array(fd, n, ns_add(p, nse));
          i = q->info.minOccurs < 100000 ? q->info.minOccurs : 100000;
          if (i < 1)
            i = 1;
          if (is_soap12(encoding))
            fprintf(fd, " SOAP-ENC:itemType=\"%s\" SOAP-ENC:arraySize=\"" SOAP_LONG_FORMAT "%s\"", wsdl_type(((Table*)((Tnode*)p->info.typ->ref)->ref)->list->info.typ, ""), i, tmp);
          else if (!is_literal(encoding))
            fprintf(fd, " SOAP-ENC:arrayType=\"%s[" SOAP_LONG_FORMAT "%s]\"", wsdl_type(((Table*)((Tnode*)p->info.typ->ref)->ref)->list->info.typ, ""), i, tmp);
          fprintf(fd, ">\n");
          gen_val(fd, n, (Tnode*)p->info.typ->ref, nse, nsa, encoding, q->info.minOccurs == 0);
          fprintf(fd, "%*s", n, "");
        }
      }
      else if (p->info.typ->type == Tstruct || p->info.typ->type == Tclass)
      {
        if (!is_invisible(p->sym->name))
        {
          gen_element_begin(fd, n, ns_add(p, nse), p);
          if (xmlns)
            gen_xmlns(fd, 0);
          gen_atts(fd, (Table*)p->info.typ->ref, nse, nsa, encoding);
        }
        else if (is_anyType(p->info.typ))
          fprintf(fd, "%*s<!-- extensibility element(s) -->\n", n, "");
      }
      else if ((p->info.typ->type == Tpointer || is_smart(p->info.typ) || p->info.typ->type == Treference || p->info.typ->type == Trvalueref)
          && (((Tnode*)p->info.typ->ref)->type == Tstruct || ((Tnode*)p->info.typ->ref)->type == Tclass))
      {
        if (!is_invisible(p->sym->name))
        {
          gen_element_begin(fd, n, ns_add(p, nse), p);
          if (xmlns)
            gen_xmlns(fd, 0);
          gen_atts(fd, (Table*)((Tnode*)p->info.typ->ref)->ref, nse, nsa, encoding);
        }
        else if (is_anyType(p->info.typ))
          fprintf(fd, "%*s<!-- extensibility element(s) -->\n", n, "");
      }
      else if (p->info.typ->type != Tunion)
      {
        if (!is_invisible(p->sym->name))
        {
          Tnode *ref = p->info.typ;
          gen_element_begin(fd, n, ns_add(p, nse), p);
          if (xmlns)
            gen_xmlns(fd, 0);
          if (p->info.typ->type == Tpointer || is_smart(p->info.typ) || p->info.typ->type == Treference || is_smart(p->info.typ))
            ref = p->info.typ->ref;
          if (ref->type == Ttemplate)
          {
            if ((((Tnode*)ref->ref)->type == Tpointer || is_smart((Tnode*)ref->ref))
                && (((Tnode*)((Tnode*)ref->ref)->ref)->type == Tclass
                  || ((Tnode*)((Tnode*)ref->ref)->ref)->type == Tstruct))
              gen_atts(fd, (Table*)((Tnode*)((Tnode*)ref->ref)->ref)->ref, nse, nsa, encoding);
            else if (((Tnode*)ref->ref)->type == Tclass || ((Tnode*)ref->ref)->type == Tstruct)
              gen_atts(fd, (Table*)((Tnode*)ref->ref)->ref, nse, nsa, encoding);
            else
              fprintf(fd, ">");
          }
          else
            fprintf(fd, ">");
        }
      }
      if (p->info.typ->sym && has_ns_eq("xsd", p->info.typ->sym->name))
      {
        gen_val(fd, n+1, p->info.typ, nse, nsa, encoding, opt);
      }
      else
      {
        switch (p->info.typ->type)
        {
          case Tchar:
          case Tshort:
          case Tint:
          case Tlong:
          case Tllong:
          case Tuchar:
          case Tushort:
          case Tuint:
          case Tulong:
          case Tullong:
            if (p->info.hasval || p->info.ptrval)
              fprintf(fd, SOAP_LONG_FORMAT, p->info.val.i);
            else
              gen_val(fd, n+1, p->info.typ, nse, nsa, encoding, opt);
            break;
          case Tfloat:
          case Tdouble:
          case Tldouble:
            if (p->info.hasval || p->info.ptrval)
              fprintf(fd, "%.16lG", p->info.val.r);
            else
              gen_val(fd, n+1, p->info.typ, nse, nsa, encoding, opt);
            break;
          case Ttime:
            gen_val(fd, n+1, p->info.typ, nse, nsa, encoding, opt);
            break;
          case Tenum:
          case Tenumsc:
            if ((p->info.hasval || p->info.ptrval) && p->info.typ->ref)
            {
              for (q = ((Table*)p->info.typ->ref)->list; q; q = q->next)
              {
                if (p->info.val.i == q->info.val.i)
                {
                  fprintf(fd, "%s", ns_remove2(q->sym->name, c_ident(p->info.typ)));
                  break;
                }
              }
            }
            else
            {
              gen_val(fd, n+1, p->info.typ, nse, nsa, encoding, opt);
            }
            break;
          case Tpointer:
          case Treference:
          case Trvalueref:
            if (is_string(p->info.typ) || is_wstring(p->info.typ))
            {
              if ((p->info.hasval || p->info.ptrval) && p->info.val.s)
                fprintf(fd, "%s", xstring(p->info.val.s));
              else
                gen_val(fd, n, p->info.typ, nse, nsa, encoding, opt);
            }
            else if (!is_dynamic_array(p->info.typ->ref))
            {
              Tnode *ref= p->info.typ->ref;
              if (p->info.hasval || p->info.ptrval)
              {
                switch (ref->type)
                {
                  case Tchar:
                  case Tshort:
                  case Tint:
                  case Tlong:
                  case Tllong:
                  case Tuchar:
                  case Tushort:
                  case Tuint:
                  case Tulong:
                  case Tullong:
                    fprintf(fd, SOAP_LONG_FORMAT, p->info.val.i);
                    break;
                  case Tfloat:
                  case Tdouble:
                  case Tldouble:
                    fprintf(fd, "%.16lG", p->info.val.r);
                    break;
                  case Tenum:
                  case Tenumsc:
                    if (ref->ref)
                    {
                      for (q = ((Table*)ref->ref)->list; q; q = q->next)
                      {
                        if (p->info.val.i == q->info.val.i)
                        {
                          fprintf(fd, "%s", ns_remove2(q->sym->name, c_ident(ref)));
                          break;
                        }
                      }
                    }
                    else
                    {
                      gen_val(fd, n, ref, nse, nsa, encoding, opt);
                    }
                    break;
                  default:
                    gen_val(fd, n, ref, nse, nsa, encoding, opt);
                    break;
                }
              }
              else
              {
                gen_val(fd, n, ref, nse, nsa, encoding, opt);
              }
            }
            break;
          case Tclass:
            if (is_stdstr(p->info.typ))
            {
              if ((p->info.hasval || p->info.ptrval) && p->info.val.s)
                fprintf(fd, "%s", xstring(p->info.val.s));
              else
                gen_val(fd, n, p->info.typ, nse, nsa, encoding, opt);
            }
            else if (!is_dynamic_array(p->info.typ))
              gen_val(fd, n, p->info.typ, nse, nsa, encoding, opt);
            break;
          case Tstruct:
            if (!is_dynamic_array(p->info.typ))
              gen_val(fd, n, p->info.typ, nse, nsa, encoding, opt);
            break;
          case Tunion:
            gen_val(fd, n, p->info.typ, nse, nsa, encoding, opt);
            break;
          case Ttemplate:
            if (gflag)
            {
              gen_val(fd, n, p->info.typ, nse, nsa, encoding, opt);
            }
            else if (!is_dynamic_array(p->info.typ->ref))
            {
              if (is_smart(p->info.typ))
              {
                i = 1;
              }
              else
              {
                i = p->info.minOccurs < 100000 ? p->info.minOccurs : 100000;
                if (i == 0)
                  i = 1;
              }
              do
              {
                /* a bit of a hack, with a copy of the code above */
                {
                  gen_val(fd, n, p->info.typ, nse, nsa, encoding, opt);
                  if  (i > 1)
                  {
                    gen_element_end(fd, 0, ns_add(p, nse));
                    if (!is_invisible(p->sym->name))
                    {
                      gen_element_begin(fd, n, ns_add(p, nse), p);
                      if (p->info.typ->type == Ttemplate)
                      {
                        if ((((Tnode*)p->info.typ->ref)->type == Tpointer || is_smart((Tnode*)p->info.typ->ref))
                            && (((Tnode*)((Tnode*)p->info.typ->ref)->ref)->type == Tclass
                              || ((Tnode*)((Tnode*)p->info.typ->ref)->ref)->type == Tstruct))
                          gen_atts(fd, (Table*)((Tnode*)((Tnode*)p->info.typ->ref)->ref)->ref, nse, nsa, encoding);
                        else if (((Tnode*)p->info.typ->ref)->type == Tclass
                            || ((Tnode*)p->info.typ->ref)->type == Tstruct)
                          gen_atts(fd, (Table*)((Tnode*)p->info.typ->ref)->ref, nse, nsa, encoding);
                        else
                          fprintf(fd, ">");
                      }
                      else
                        fprintf(fd, ">");
                    }
                  }
                  fflush(fd);
                }
              } while (--i);
            }
            break;
          default:
            break;
        }
      }
      if (p->info.typ->type != Tunion)
      {
        if (!is_invisible(p->sym->name))
          gen_element_end(fd, 0, ns_add(p, nse));
        else
          fprintf(fd, "\n");
      }
      if (gflag && is_container(p->info.typ))
        fprintf(fd, "%*s</__REPEAT>\n", n, "");
      fflush(fd);
    }
  }
}

void
gen_atts(FILE *fd, Table *t, const char *nse, const char *nsa, const char *encoding)
{
  Entry *q, *r;
  Tnode *p;
  while (t)
  {
    for (q = t->list; q; q = q->next)
    {
      if (q->info.sto & Sattribute && !is_invisible(q->sym->name) && q->info.maxOccurs != 0)
      {
        fprintf(fd, " %s=\"", ns_add(q, nsa));
        if ((q->info.typ->type == Tpointer || is_smart(q->info.typ) || q->info.typ->type == Treference || q->info.typ->type == Trvalueref || q->info.typ->type == Ttemplate) && !is_string(q->info.typ) && !is_wstring(q->info.typ))
          p = (Tnode*)q->info.typ->ref;
        else
          p = q->info.typ;
        if (!gflag && is_eq(q->sym->name, "id"))
          fprintf(fd, "_%lu", ++idnum); /* id="#" should be unique */
        else if (!q->info.hasval && !q->info.ptrval && p->sym && has_ns_eq("xsd", p->sym->name))
        {
          gen_val(fd, 0, p, nse, nsa, encoding, q->info.minOccurs == 0);
        }
        else
        {
          switch (p->type)
          {
            case Tchar:
            case Tshort:
            case Tint:
            case Tuchar:
            case Tushort:
            case Tuint:
            case Tlong:
            case Tulong:
            case Tllong:
            case Tullong:
              if (q->info.hasval || q->info.ptrval)
                fprintf(fd, SOAP_LONG_FORMAT, q->info.val.i);
              else
                gen_val(fd, 0, p, nse, nsa, encoding, q->info.minOccurs == 0);
              break;
            case Tfloat:
            case Tdouble:
            case Tldouble:
              if (q->info.hasval || q->info.ptrval)
                fprintf(fd, "%.16lG", q->info.val.r);
              else
                gen_val(fd, 0, p, nse, nsa, encoding, q->info.minOccurs == 0);
              break;
            case Ttime:
              gen_val(fd, 0, p, nse, nsa, encoding, q->info.minOccurs == 0);
              break;
            case Tenum:
            case Tenumsc:
              if ((q->info.hasval || q->info.ptrval) && p->ref)
              {
                for (r = ((Table*)p->ref)->list; r; r = r->next)
                {
                  if (r->info.val.i == q->info.val.i)
                  {
                    fprintf(fd, "%s", ns_remove2(r->sym->name, c_ident(p)));
                    break;
                  }
                }
              }
              else
                gen_val(fd, 0, p, nse, nsa, encoding, q->info.minOccurs == 0);
              break;
            case Tpointer:
            case Treference:
            case Trvalueref:
            case Ttemplate:
              if (is_string(p) || is_wstring(p))
              {
                if ((q->info.hasval || q->info.ptrval) && q->info.val.s)
                  fprintf(fd, "%s", xstring(q->info.val.s));
                else
                  gen_val(fd, 0, p, nse, nsa, encoding, q->info.minOccurs == 0);
              }
              else if (is_stdstr(p->ref))
              {
                if ((q->info.hasval || q->info.ptrval) && q->info.val.s)
                  fprintf(fd, "%s", xstring(q->info.val.s));
                else
                  gen_val(fd, 0, p->ref, nse, nsa, encoding, q->info.minOccurs == 0);
              }
              break;
            case Tstruct:
            case Tclass:
              if (is_stdstr(p))
              {
                if ((q->info.hasval || q->info.ptrval) && q->info.val.s)
                  fprintf(fd, "%s", xstring(q->info.val.s));
                else
                  gen_val(fd, 0, p, nse, nsa, encoding, q->info.minOccurs == 0);
              }
              else
              {
                gen_val(fd, 0, p, nse, nsa, encoding, q->info.minOccurs == 0);
              }
              break;
            default:
              break;
          }
        }
        fprintf(fd, "\"");
      }
    }
    t = t->prev;
    if (t)
      nsa = ns_qualifiedAttributeName(t->sym->name);
  }
  fprintf(fd, ">");
  fflush(fd);
}

void
gen_val(FILE *fd, int n, Tnode *p, const char *nse, const char *nsa, const char *encoding, int opt)
{
  Entry *q;
  LONG64 i;
  if (!is_transient(p) && p->type != Tfun && !is_XML(p))
  {
    if (is_fixedstring(p))
    {
      LONG64 k = p->width / ((Tnode*)p->ref)->width - 1;
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        fprintf(fd, "%%[[TEXT[" SOAP_LONG_FORMAT ":" SOAP_LONG_FORMAT "]]]%%", k, k);
      }
      else
      {
        for (i = 0; i < k; i++)
          fprintf(fd, "X");
      }
      return;
    }
    else if (p->type == Tarray)
    {
      i = ((Tnode*) p->ref)->width;
      fprintf(fd, "\n");
      if (i)
      {
        i = p->width / i;
        if (gflag)
        {
          fprintf(fd, "%*s<__REPEAT min=\"" SOAP_LONG_FORMAT "\" max=\"" SOAP_LONG_FORMAT "\">\n", n+1, "", i, i);
          fprintf(fd, "%*s<item>", n+1, "");
          gen_val(fd, n+1, (Tnode*)p->ref, nse, nsa, encoding, 0);
          fprintf(fd, "</item>\n");
          fprintf(fd, "%*s</__REPEAT>\n", n+1, "");
        }
        else
        {
          fprintf(fd, "%*s<item>", n+1, "");
          gen_val(fd, n+1, (Tnode*)p->ref, nse, nsa, encoding, 0);
          fprintf(fd, "</item>\n");
        }
        fprintf(fd, "%*s", n, "");
      }
      return;
    }
    else if ((q = is_dynamic_array(p)))
    {
      if (is_hexBinary(p))
      {
        if (gflag)
        {
          if (opt)
            fprintf(fd, "???");
          if (p->hasmin && p->hasmax)
            fprintf(fd, "%%[[HEX[%ld:%ld]]]%%", minlen(p), maxlen(p));
          else if (p->hasmin)
            fprintf(fd, "%%[[HEX[%ld:2147483647]]]%%", minlen(p));
          else if (p->hasmax)
            fprintf(fd, "%%[[HEX[0:%ld]]]%%", maxlen(p));
          else
            fprintf(fd, "%%[[HEX]]%%");
        }
      }
      else if (is_binary(p))
      {
        if (gflag)
        {
          if (opt)
            fprintf(fd, "???");
          if (p->hasmin && p->hasmax)
            fprintf(fd, "%%[[BASE64[%ld:%ld]]]%%", minlen(p), maxlen(p));
          else if (p->hasmin)
            fprintf(fd, "%%[[BASE64[%ld:2147483647]]]%%", minlen(p));
          else if (p->hasmax)
            fprintf(fd, "%%[[BASE64[0:%ld]]]%%", maxlen(p));
          else
            fprintf(fd, "%%[[BASE64]]%%");
        }
      }
      else
      {
        int d = get_Darraydims(p);
        if (d == 0)
          d = 1;
        LONG64 k = q->info.minOccurs < 100000 ? q->info.minOccurs : 100000;
        if (k < 1 && (q->info.maxOccurs > 0 || q->info.maxOccurs < 0))
          k = 1;
        if (gflag)
        {
          for (i = 0; i < d; i++)
          {
            fprintf(fd, "%*s<__REPEAT min=\"" SOAP_LONG_FORMAT "\"", n+1, "", q->info.minOccurs);
            if (q->info.maxOccurs > 1)
              fprintf(fd, " max=\"" SOAP_LONG_FORMAT "\"", q->info.maxOccurs);
            else
              fprintf(fd, " max=\"unbounded\"");
            fprintf(fd, ">\n");
          }
          k = 1;
        }
        else if (yflag)
        {
          fprintf(fd, "%*s<!-- a repetition of " SOAP_LONG_FORMAT, n+1, "", q->info.minOccurs);
          if (q->info.maxOccurs == q->info.minOccurs && q->info.maxOccurs > 1)
            fprintf(fd, " of the following");
          else if (q->info.maxOccurs > 1)
            fprintf(fd, " to " SOAP_LONG_FORMAT " of the following", q->info.maxOccurs);
          else
            fprintf(fd, " or more of the following");
          fprintf(fd, " -->\n");
        }
        for (i = 0; i < k; ++i)
        {
          fprintf(fd, "%*s<%s>", n+1, "", q->sym->name[5]?q->sym->name+5:"item");
          gen_val(fd, n+1, q->info.typ, nse, nsa, encoding, q->info.maxOccurs == 0);
          fprintf(fd, "</%s>\n", q->sym->name[5]?q->sym->name+5:"item");
        }
        if (gflag)
          for (i = 0; i < d; i++)
            fprintf(fd, "%*s</__REPEAT>\n", n+1, "");
      }
      return;
    }
    else if (has_restriction_base(p, "xsd__anyURI"))
    {
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        fprintf(fd, "%%[[URI]]%%");
      }
      else
      {
        fprintf(fd, "http://www.example.com/schema/anyURI");
      }
      return;
    }
    else if (has_restriction_base(p, "xsd__boolean"))
    {
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        fprintf(fd, "%%[[BOOL]]%%");
      }
      else
      {
        fprintf(fd, "false");
      }
      return;
    }
    else if (has_restriction_base(p, "xsd__ENTITY") || has_restriction_base(p, "xsd__ENTITIES"))
    {
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        if (p->hasmin && p->hasmax)
          fprintf(fd, "%%[[ENTITY[%ld:%ld]]]%%", minlen(p), maxlen(p));
        else if (p->hasmin)
          fprintf(fd, "%%[[ENTITY[%ld:2147483647]]]%%", minlen(p));
        else if (p->hasmax)
          fprintf(fd, "%%[[ENTITY[0:%ld]]]%%", maxlen(p));
        else
          fprintf(fd, "%%[[ENTITY]]%%");
      }
      else
      {
        if (p->imin > 0 && p->imin < 100000)
          for (i = 0; i < (int)p->imin; i++)
            fprintf(fd, "X");
      }
      return;
    }
    else if (has_restriction_base(p, "xsd__ID"))
    {
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        if (p->hasmin && p->hasmax)
          fprintf(fd, "%%[[ID[%ld:%ld]]]%%", minlen(p), maxlen(p));
        else if (p->hasmin)
          fprintf(fd, "%%[[ID[%ld:2147483647]]]%%", minlen(p));
        else if (p->hasmax)
          fprintf(fd, "%%[[ID[0:%ld]]]%%", maxlen(p));
        else
          fprintf(fd, "%%[[ID]]%%");
      }
      else if (p->imin > 0 && p->imin < 100000)
      {
        for (i = 0; i < (int)p->imin; i++)
          fprintf(fd, "X");
      }
      else
      {
        fprintf(fd, "_%lu", ++idnum);
      }
      return;
    }
    else if (has_restriction_base(p, "xsd__IDREF") || has_restriction_base(p, "xsd__IDREFS"))
    {
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        if (p->hasmin && p->hasmax)
          fprintf(fd, "%%[[IDREF[%ld:%ld]]]%%", minlen(p), maxlen(p));
        else if (p->hasmin)
          fprintf(fd, "%%[[IDREF[%ld:2147483647]]]%%", minlen(p));
        else if (p->hasmax)
          fprintf(fd, "%%[[IDREF[0:%ld]]]%%", maxlen(p));
        else
          fprintf(fd, "%%[[IDREF]]%%");
      }
      else if (p->imin > 0 && p->imin < 100000)
      {
        for (i = 0; i < (int)p->imin; i++)
          fprintf(fd, "X");
      }
      else
      {
        fprintf(fd, "_%lu", ++idnum);
      }
      return;
    }
    else if (has_restriction_base(p, "xsd__integer"))
    {
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        fprintf(fd, "%%[[INT64]]%%");
      }
      else
        fprintf(fd, "0");
      return;
    }
    else if (has_restriction_base(p, "xsd__date"))
    {
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        fprintf(fd, "%%[[DATE]]%%");
      }
      else
      {
        fprintf(fd, "1999-12-31");
      }
      return;
    }
    else if (has_restriction_base(p, "xsd__dateTime"))
    {
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        fprintf(fd, "%%[[DATETIME]]%%");
      }
      else
      {
        char tmp[256];
        time_t T = time(NULL);
        strftime(tmp, 256, "%Y-%m-%dT%H:%M:%SZ", gmtime(&T));
        fprintf(fd, "%s", tmp);
      }
      return;
    }
    else if (has_restriction_base(p, "xsd__decimal"))
    {
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        fprintf(fd, "%%[[DOUBLE]]%%");
      }
      else
      {
        fprintf(fd, "0.0");
      }
      return;
    }
    else if (has_restriction_base(p, "xsd__duration"))
    {
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        fprintf(fd, "%%[[DURATION]]%%");
      }
      else
      {
        fprintf(fd, "PT0S");
      }
      return;
    }
    else if (has_restriction_base(p, "xsd__language"))
    {
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        if (p->hasmin && p->hasmax)
          fprintf(fd, "%%[[LANG[%ld:%ld]]]%%", minlen(p), maxlen(p));
        else if (p->hasmin)
          fprintf(fd, "%%[[LANG[%ld:2]]]%%", minlen(p));
        else if (p->hasmax)
          fprintf(fd, "%%[[LANG[0:%ld]]]%%", maxlen(p));
        else
          fprintf(fd, "%%[[LANG]]%%");
      }
      else
      {
        if (p->imin > 0 && p->imin < 100000)
          for (i = 0; i < (int)p->imin; i++)
            fprintf(fd, "X");
      }
      return;
    }
    else if (has_restriction_base(p, "xsd__Name"))
    {
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        if (p->hasmin && p->hasmax)
          fprintf(fd, "%%[[NAME[%ld:%ld]]]%%", minlen(p), maxlen(p));
        else if (p->hasmin)
          fprintf(fd, "%%[[NAME[%ld:2147483647]]]%%", minlen(p));
        else if (p->hasmax)
          fprintf(fd, "%%[[NAME[0:%ld]]]%%", maxlen(p));
        else
          fprintf(fd, "%%[[NAME]]%%");
      }
      else
      {
        if (p->imin > 0 && p->imin < 100000)
          for (i = 0; i < (int)p->imin; i++)
            fprintf(fd, "X");
      }
      return;
    }
    else if (has_restriction_base(p, "xsd__NCName"))
    {
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        if (p->hasmin && p->hasmax)
          fprintf(fd, "%%[[NCNAME[%ld:%ld]]]%%", minlen(p), maxlen(p));
        else if (p->hasmin)
          fprintf(fd, "%%[[NCNAME[%ld:2147483647]]]%%", minlen(p));
        else if (p->hasmax)
          fprintf(fd, "%%[[NCNAME[0:%ld]]]%%", maxlen(p));
        else
          fprintf(fd, "%%[[NCNAME]]%%");
      }
      else
      {
        if (p->imin > 0 && p->imin < 100000)
          for (i = 0; i < (int)p->imin; i++)
            fprintf(fd, "X");
      }
      return;
    }
    else if (has_restriction_base(p, "xsd__NMTOKEN") || has_restriction_base(p, "xsd__NMTOKENS"))
    {
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        if (p->hasmin && p->hasmax)
          fprintf(fd, "%%[[NMTOKEN[%ld:%ld]]]%%", minlen(p), maxlen(p));
        else if (p->hasmin)
          fprintf(fd, "%%[[NMTOKEN[%ld:2147483647]]]%%", minlen(p));
        else if (p->hasmax)
          fprintf(fd, "%%[[NMTOKEN[0:%ld]]]%%", maxlen(p));
        else
          fprintf(fd, "%%[[NMTOKEN]]%%");
      }
      else
      {
        if (p->imin > 0 && p->imin < 100000)
          for (i = 0; i < (int)p->imin; i++)
            fprintf(fd, "X");
      }
      return;
    }
    else if (has_restriction_base(p, "xsd__token") || has_restriction_base(p, "xsd__token"))
    {
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        if (p->hasmin && p->hasmax)
          fprintf(fd, "%%[[TOKEN[%ld:%ld]]]%%", minlen(p), maxlen(p));
        else if (p->hasmin)
          fprintf(fd, "%%[[TOKEN[%ld:2147483647]]]%%", minlen(p));
        else if (p->hasmax)
          fprintf(fd, "%%[[TOKEN[0:%ld]]]%%", maxlen(p));
        else
          fprintf(fd, "%%[[TOKEN]]%%");
      }
      else
      {
        if (p->imin > 0 && p->imin < 100000)
          for (i = 0; i < (int)p->imin; i++)
            fprintf(fd, "X");
      }
      return;
    }
    else if (has_restriction_base(p, "xsd__negativeInteger"))
    {
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        fprintf(fd, "%%[[-9223372036854775808:-1]]%%");
      }
      else
      {
        fprintf(fd, "-1");
      }
      return;
    }
    else if (has_restriction_base(p, "xsd__nonNegativeInteger"))
    {
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        fprintf(fd, "%%[[0:9223372036854775807]]%%");
      }
      else
      {
        fprintf(fd, "0");
      }
      return;
    }
    else if (has_restriction_base(p, "xsd__nonPositiveInteger"))
    {
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        fprintf(fd, "%%[[-9223372036854775808:0]]%%");
      }
      else
      {
        fprintf(fd, "0");
      }
      return;
    }
    else if (has_restriction_base(p, "xsd__positiveInteger"))
    {
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        fprintf(fd, "%%[[1:9223372036854775807]]%%");
      }
      else
      {
        fprintf(fd, "1");
      }
      return;
    }
    else if (has_restriction_base(p, "xsd__time"))
    {
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        fprintf(fd, "%%[[TIME]]%%");
      }
      else
      {
        fprintf(fd, "12:34:56.789Z");
      }
      return;
    }
    else if (is_qname(p) || is_stdqname(p))
    {
      if (gflag)
      {
        if (opt)
          fprintf(fd, "???");
        fprintf(fd, "%%[[QNAME]]%%");
      }
      else
      {
        fprintf(fd, "xsd:string");
      }
      return;
    }
    if (p->pattern && *p->pattern != '%')
    {
      if (gflag)
      {
        const char *s;
        if (opt)
          fprintf(fd, "???");
        fprintf(fd, "%%[['");
        for (s = p->pattern; *s; s++)
        {
          if (*s == '\'')
            fprintf(fd, "\\'");
          else
            fprintf(fd, "%c", *s);
        }
        if (p->hasmin && p->hasmax)
          fprintf(fd, "'[%ld:%ld]]]%%", minlen(p), maxlen(p));
        else if (p->hasmin)
          fprintf(fd, "'[%ld:2147483647]]]%%", minlen(p));
        else if (p->hasmax)
          fprintf(fd, "'[0:%ld]]]%%", maxlen(p));
        else
          fprintf(fd, "']]%%");
        return;
      }
    }
    switch (p->type)
    {
      case Tchar:
        if (gflag)
        {
          if (opt)
            fprintf(fd, "???");
          if (p->hasmin && p->hasmax)
            fprintf(fd, "%%[[%d:%d]]%%", (int)p->imin + (p->incmin == False), (int)p->imax - (p->incmax == False));
          else if (p->hasmin)
            fprintf(fd, "%%[[%d:127]]%%", (int)p->imin + (p->incmin == False));
          else if (p->hasmax)
            fprintf(fd, "%%[[-128:%d]]%%", (int)p->imax - (p->incmax == False));
          else
            fprintf(fd, "%%[[INT8]]%%");
        }
        else
        {
          if (p->hasmin && p->imin > 0)
            fprintf(fd, "%d", (int)p->imin + (p->incmin == False));
          else if (p->hasmax && p->imax < 0)
            fprintf(fd, "%d", (int)p->imax - (p->incmax == False));
          else
            fprintf(fd, "0");
        }
        break;
      case Tshort:
        if (gflag)
        {
          if (opt)
            fprintf(fd, "???");
          if (p->hasmin && p->hasmax)
            fprintf(fd, "%%[[%d:%d]]%%", (int)p->imin + (p->incmin == False), (int)p->imax - (p->incmax == False));
          else if (p->hasmin)
            fprintf(fd, "%%[[%d:32767]]%%", (int)p->imin + (p->incmin == False));
          else if (p->hasmax)
            fprintf(fd, "%%[[-32768:%d]]%%", (int)p->imax - (p->incmax == False));
          else
            fprintf(fd, "%%[[INT16]]%%");
        }
        else
        {
          if (p->hasmin && p->imin > 0)
            fprintf(fd, "%d", (int)p->imin + (p->incmin == False));
          else if (p->hasmax && p->imax < 0)
            fprintf(fd, "%d", (int)p->imax - (p->incmax == False));
          else
            fprintf(fd, "0");
        }
        break;
      case Tint:
      case Tlong:
        if (gflag)
        {
          if (opt)
            fprintf(fd, "???");
          if (p->hasmin && p->hasmax)
            fprintf(fd, "%%[[%ld:%ld]]%%", (long)p->imin + (p->incmin == False), (long)p->imax - (p->incmax == False));
          else if (p->hasmin)
            fprintf(fd, "%%[[%ld:2147483647]]%%", (long)p->imin + (p->incmin == False));
          else if (p->hasmax)
            fprintf(fd, "%%[[-2147483648:%ld]]%%", (long)p->imax - (p->incmax == False));
          else
            fprintf(fd, "%%[[INT32]]%%");
        }
        else
        {
          if (p->hasmin && p->imin > 0)
            fprintf(fd, "%ld", (long)p->imin + (p->incmin == False));
          else if (p->hasmax && p->imax < 0)
            fprintf(fd, "%ld", (long)p->imax - (p->incmax == False));
          else
            fprintf(fd, "0");
        }
        break;
      case Tllong:
        if (gflag)
        {
          if (opt)
            fprintf(fd, "???");
          if (p->hasmin && p->hasmax)
            fprintf(fd, "%%[[" SOAP_LONG_FORMAT ":" SOAP_LONG_FORMAT "]]%%", p->imin + (p->incmin == False), p->imax - (p->incmax == False));
          else if (p->hasmin)
            fprintf(fd, "%%[[" SOAP_LONG_FORMAT ":9223372036854775807]]%%", p->imin + (p->incmin == False));
          else if (p->hasmax)
            fprintf(fd, "%%[[-9223372036854775808:" SOAP_LONG_FORMAT "]]%%", p->imax - (p->incmax == False));
          else
            fprintf(fd, "%%[[INT64]]%%");
        }
        else
        {
          if (p->hasmin && p->imin > 0)
            fprintf(fd, SOAP_LONG_FORMAT, p->imin + (p->incmin == False));
          else if (p->hasmax && p->imax < 0)
            fprintf(fd, SOAP_LONG_FORMAT, p->imax - (p->incmax == False));
          else
            fprintf(fd, "0");
        }
        break;
      case Tuchar:
        if (gflag)
        {
          if (opt)
            fprintf(fd, "???");
          if (p->hasmin && p->hasmax)
            fprintf(fd, "%%[[%u:%u]]%%", (unsigned int)p->imin + (p->incmin == False), (unsigned int)p->imax - (p->incmax == False));
          else if (p->hasmin)
            fprintf(fd, "%%[[%u:255]]%%", (unsigned int)p->imin + (p->incmin == False));
          else if (p->hasmax)
            fprintf(fd, "%%[[0:%u]]%%", (unsigned int)p->imax - (p->incmax == False));
          else
            fprintf(fd, "%%[[UINT8]]%%");
        }
        else
        {
          if (p->hasmin && p->imin > 0)
            fprintf(fd, "%u", (unsigned int)p->imin + (p->incmin == False));
          else if (p->hasmax && p->imax < 0)
            fprintf(fd, "%u", (unsigned int)p->imax - (p->incmax == False));
          else
            fprintf(fd, "0");
        }
        break;
      case Tushort:
        if (gflag)
        {
          if (opt)
            fprintf(fd, "???");
          if (p->hasmin && p->hasmax)
            fprintf(fd, "%%[[%u:%u]]%%", (unsigned int)p->imin + (p->incmin == False), (unsigned int)p->imax - (p->incmax == False));
          else if (p->hasmin)
            fprintf(fd, "%%[[%u:65535]]%%", (unsigned int)p->imin + (p->incmin == False));
          else if (p->hasmax)
            fprintf(fd, "%%[[0:%u]]%%", (unsigned int)p->imax - (p->incmax == False));
          else
            fprintf(fd, "%%[[UINT16]]%%");
        }
        else
        {
          if (p->hasmin && p->imin > 0)
            fprintf(fd, "%u", (unsigned int)p->imin + (p->incmin == False));
          else if (p->hasmax && p->imax < 0)
            fprintf(fd, "%u", (unsigned int)p->imax - (p->incmax == False));
          else
            fprintf(fd, "0");
        }
        break;
      case Tuint:
      case Tulong:
        if (gflag)
        {
          if (opt)
            fprintf(fd, "???");
          if (p->hasmin && p->hasmax)
            fprintf(fd, "%%[[%lu:%lu]]%%", (unsigned long)p->imin + (p->incmin == False), (unsigned long)p->imax - (p->incmax == False));
          else if (p->hasmin)
            fprintf(fd, "%%[[%lu:4294967295]]%%", (unsigned long)p->imin + (p->incmin == False));
          else if (p->hasmax)
            fprintf(fd, "%%[[0:%lu]]%%", (unsigned long)p->imax - (p->incmax == False));
          else
            fprintf(fd, "%%[[UINT32]]%%");
        }
        else
        {
          if (p->hasmin && p->imin > 0)
            fprintf(fd, "%lu", (unsigned long)p->imin + (p->incmin == False));
          else if (p->hasmax && p->imax < 0)
            fprintf(fd, "%lu", (unsigned long)p->imax - (p->incmax == False));
          else
            fprintf(fd, "0");
        }
        break;
      case Tullong:
        if (gflag)
        {
          if (opt)
            fprintf(fd, "???");
          if (p->hasmin && p->hasmax)
            fprintf(fd, "%%[[" SOAP_ULONG_FORMAT ":" SOAP_ULONG_FORMAT "]]%%", (ULONG64)p->imin + (p->incmin == False), (ULONG64)p->imax - (p->incmax == False));
          else if (p->hasmin)
            fprintf(fd, "%%[[" SOAP_ULONG_FORMAT ":18446744073709551615]]%%", (ULONG64)p->imin + (p->incmin == False));
          else if (p->hasmax)
            fprintf(fd, "%%[[0:" SOAP_ULONG_FORMAT "]]%%", (ULONG64)p->imax - (p->incmax == False));
          else
            fprintf(fd, "%%[[UINT64]]%%");
        }
        else
        {
          if (p->hasmin && p->imin > 0)
            fprintf(fd, SOAP_ULONG_FORMAT, (ULONG64)p->imin + (p->incmin == False));
          else if (p->hasmax && p->imax < 0)
            fprintf(fd, SOAP_ULONG_FORMAT, (ULONG64)p->imax - (p->incmax == False));
          else
            fprintf(fd, "0");
        }
        break;
      case Tfloat:
        if (gflag)
        {
          const char *pattern = p->pattern && *p->pattern == '%' ? p->pattern : "";
          if (opt)
            fprintf(fd, "???");
          if (p->hasmin && p->hasmax && p->incmin && p->incmax)
            fprintf(fd, "%%[[%.8E:%.8E%s]]%%", p->rmin, p->rmax, pattern);
          else if (p->hasmin && p->hasmax && p->incmin)
            fprintf(fd, "%%[[%.8E:%.8E%s)]%%", p->rmin, p->rmax, pattern);
          else if (p->hasmin && p->hasmax && p->incmax)
            fprintf(fd, "%%[(%.8E:%.8E%s]]%%", p->rmin, p->rmax, pattern);
          else if (p->hasmin && p->hasmax)
            fprintf(fd, "%%[(%.8E:%.8E%s)]%%", p->rmin, p->rmax, pattern);
          else if (p->hasmin && p->incmin)
            fprintf(fd, "%%[[%.8E:%.8E%s]]%%", p->rmin, FLT_MAX, pattern);
          else if (p->hasmin)
            fprintf(fd, "%%[(%.8E:%.8E%s]]%%", p->rmin, FLT_MAX, pattern);
          else if (p->hasmax && p->incmax)
            fprintf(fd, "%%[[%.8E:%.8E%s]]%%", -FLT_MAX, p->rmax, pattern);
          else if (p->hasmax)
            fprintf(fd, "%%[[%.8E:%.8E%s)]%%", -FLT_MAX, p->rmax, pattern);
          else
            fprintf(fd, "%%[[FLOAT%s]]%%", pattern);
        }
        else
        {
          if (p->hasmin && p->rmin > 0)
            fprintf(fd, "%.9lG", p->rmin * (1 + (p->incmin == False)/1000));
          else if (p->hasmax && p->rmax > 0)
            fprintf(fd, "%.9lG", p->rmax * (1 - (p->incmax == False)/1000));
          else if (p->hasmin && p->rmin < 0)
            fprintf(fd, "%.9lG", p->rmin * (1 - (p->incmin == False)/1000));
          else if (p->hasmax && p->rmax < 0)
            fprintf(fd, "%.9lG", p->rmax * (1 + (p->incmax == False)/1000));
          else
            fprintf(fd, "0.0");
        }
        break;
      case Tdouble:
      case Tldouble:
        if (gflag)
        {
          const char *pattern = p->pattern && *p->pattern == '%' ? p->pattern : "";
          if (opt)
            fprintf(fd, "???");
          if (p->hasmin && p->hasmax && p->incmin && p->incmax)
            fprintf(fd, "%%[[%.16lE:%.16lE%s]]%%", p->rmin, p->rmax, pattern);
          else if (p->hasmin && p->hasmax && p->incmin)
            fprintf(fd, "%%[[%.16lE:%.16lE%s)]%%", p->rmin, p->rmax, pattern);
          else if (p->hasmin && p->hasmax && p->incmax)
            fprintf(fd, "%%[(%.16lE:%.16lE%s]]%%", p->rmin, p->rmax, pattern);
          else if (p->hasmin && p->hasmax)
            fprintf(fd, "%%[(%.16lE:%.16lE%s)]%%", p->rmin, p->rmax, pattern);
          else if (p->hasmin && p->incmin)
            fprintf(fd, "%%[[%.16lE:%.16lE%s]]%%", p->rmin, DBL_MAX, pattern);
          else if (p->hasmin)
            fprintf(fd, "%%[(%.16lE:%.16lE%s]]%%", p->rmin, DBL_MAX, pattern);
          else if (p->hasmax && p->incmax)
            fprintf(fd, "%%[[%.16lE:%.16lE%s]]%%", -DBL_MAX, p->rmax, pattern);
          else if (p->hasmax)
            fprintf(fd, "%%[[%.16lE:%.16lE%s)]%%", -DBL_MAX, p->rmax, pattern);
          else
            fprintf(fd, "%%[[DOUBLE%s]]%%", pattern);
        }
        else
        {
          if (p->hasmin && p->rmin > 0)
            fprintf(fd, "%.17lG", p->rmin * (1 + (p->incmin == False)/1000));
          else if (p->hasmax && p->rmax > 0)
            fprintf(fd, "%.17lG", p->rmax * (1 - (p->incmax == False)/1000));
          else if (p->hasmin && p->rmin < 0)
            fprintf(fd, "%.17lG", p->rmin * (1 - (p->incmin == False)/1000));
          else if (p->hasmax && p->rmax < 0)
            fprintf(fd, "%.17lG", p->rmax * (1 + (p->incmax == False)/1000));
          else
            fprintf(fd, "0.0");
        }
        break;
      case Ttime:
        if (gflag)
        {
          if (opt)
            fprintf(fd, "???");
          fprintf(fd, "%%[[DATETIME]]%%");
        }
        else
        {
          char tmp[256];
          time_t T = time(NULL);
          strftime(tmp, 256, "%Y-%m-%dT%H:%M:%SZ", gmtime(&T));
          fprintf(fd, "%s", tmp);
        }
        break;
      case Tenum:
      case Tenumsc:
        if (gflag)
        {
          if (opt)
            fprintf(fd, "???");
          if (p->ref)
          {
            q = ((Table*)p->ref)->list;
            if (q)
            {
              if (!q->next)
              {
                fprintf(fd, "%s", ns_remove2(q->sym->name, c_ident(p)));
              }
              else
              {
                fprintf(fd, "%%[[");
                while (q)
                {
                  fprintf(fd, "%s", ns_remove2(q->sym->name, c_ident(p)));
                  if (q->next)
                    fprintf(fd, "][");
                  q = q->next;
                }
                fprintf(fd, "]]%%");
              }
            }
          }
        }
        else
        {
          if (p->ref && (q = ((Table*)p->ref)->list))
            fprintf(fd, "%s", ns_remove2(q->sym->name, c_ident(p)));
          else
            fprintf(fd, "0");
        }
        break;
      case Tpointer:
      case Treference:
      case Trvalueref:
      case Ttemplate:
        if (is_string(p) || is_wstring(p))
        {
          if (gflag)
          {
            if (opt)
              fprintf(fd, "???");
            if (p->hasmin && p->hasmax)
              fprintf(fd, "%%[[TEXT[%ld:%ld]]]%%", minlen(p), maxlen(p));
            else if (p->hasmin)
              fprintf(fd, "%%[[TEXT[%ld:2147483647]]]%%", minlen(p));
            else if (p->hasmax)
              fprintf(fd, "%%[[TEXT[0:%ld]]]%%", maxlen(p));
            else
              fprintf(fd, "%%[[TEXT]]%%");
          }
          else
          {
            if (p->imin > 0 && p->imin < 100000)
              for (i = 0; i < (int)p->imin; i++)
                fprintf(fd, "X");
          }
        }
        else
        {
          gen_val(fd, n, (Tnode*)p->ref, nse, nsa, encoding, opt);
        }
        break;
      case Tclass:
      case Tstruct:
        nse = ns_qualifiedElement(p);
        nsa = ns_qualifiedAttribute(p);
        if (is_stdstr(p))
        {
          if (gflag)
          {
            if (opt)
              fprintf(fd, "???");
            if (p->hasmin && p->hasmax)
              fprintf(fd, "%%[[TEXT[%ld:%ld]]]%%", minlen(p), maxlen(p));
            else if (p->hasmin)
              fprintf(fd, "%%[[TEXT[%ld:2147483647]]]%%", minlen(p));
            else if (p->hasmax)
              fprintf(fd, "%%[[TEXT[0:%ld]]]%%", maxlen(p));
            else
              fprintf(fd, "%%[[TEXT]]%%");
          }
          else
          {
            if (p->imin > 0 && p->imin < 100000)
              for (i = 0; i < (int)p->imin; i++)
                fprintf(fd, "X");
          }
        }
        else if (is_primclass(p))
        {
          Table *t = (Table*)p->ref;
          while (t)
          {
            Entry *r = entry(classtable, t->sym);
            r = t->list;
            while (r && !is_item(r))
              r = r->next;
            if (r)
            {
              gen_val(fd, n, r->info.typ, nse, nsa, encoding, opt);
              return;
            }
            t = t->prev;
            if (t)
              nse = ns_qualifiedElementName(t->sym->name);
          }
        }
        else if (!is_dynamic_array(p) && p->ref)
        {
          int i = 0;
          Table *t;
          if (gflag && opt)
            fprintf(fd, "???");
          fprintf(fd, "\n");
          for (t = (Table*)p->ref; t; t = t->prev)
            i++;
          for (; i > 0; i--)
          {
            int j;
            const char *nse1;
            t = (Table*)p->ref;
            for (j = 0; j < i-1; j++)
              t = t->prev;
            if (t == (Table*)p->ref)
              nse1 = nse;
            else
              nse1 = ns_qualifiedElementName(t->sym->name);
            for (q = t->list; q; q = q->next)
            {
              if (is_sequence(q))
              {
                if (gflag)
                {
                  if (q->info.minOccurs == 0)
                    fprintf(fd, "%*s<__REPEAT min=\"" SOAP_LONG_FORMAT "\">\n", n+1, "", q->info.minOccurs);
                  gen_field(fd, n, q, nse1, nsa, encoding, 0, 0);
                  if (q->info.minOccurs == 0)
                    fprintf(fd, "%*s</__REPEAT>\n", n+1, "");
                }
                else
                {
                  gen_field(fd, n, q, nse1, nsa, encoding, 0, 0);
                }
              }
              else if (is_repetition(q))
              {
                if (gflag)
                {
                  fprintf(fd, "%*s<__REPEAT min=\"" SOAP_LONG_FORMAT "\"", n+1, "", q->info.minOccurs);
                  if (q->info.maxOccurs > 1)
                    fprintf(fd, " max=\"" SOAP_LONG_FORMAT "\"", q->info.maxOccurs);
                  else
                    fprintf(fd, " max=\"unbounded\"");
                  fprintf(fd, ">\n");
                  gen_field(fd, n+1, q->next, nse1, nsa, encoding, 0, 0);
                  fprintf(fd, "%*s</__REPEAT>\n", n+1, "");
                }
                else
                {
                  LONG64 j;
                  if (yflag)
                  {
                    fprintf(fd, "%*s<!-- a repetition of " SOAP_LONG_FORMAT, n+1, "", q->info.minOccurs);
                    if (q->info.maxOccurs == q->info.minOccurs && q->info.maxOccurs > 1)
                      fprintf(fd, " of the following");
                    else if (q->info.maxOccurs > 1)
                      fprintf(fd, " to " SOAP_LONG_FORMAT " of the following", q->info.maxOccurs);
                    else
                      fprintf(fd, " or more of the following");
                    fprintf(fd, " -->\n");
                  }
                  j = q->info.minOccurs < 100000 ? q->info.minOccurs : 100000;
                  if (j < 1 && (q->info.maxOccurs > 0 || q->info.maxOccurs < 0))
                    j = 1;
                  for (; j > 0; --j)
                    gen_field(fd, n+1, q->next, nse1, nsa, encoding, q->info.minOccurs == 0 && (opt || !is_invisible(p->id->name)), 0);
                }
                q = q->next;
              }
              else
              {
                gen_field(fd, n+1, q, nse1, nsa, encoding, q->info.minOccurs == 0 && (opt || !is_invisible(p->id->name)), 0);
              }
            }
          }
          fprintf(fd, "%*s", n, "");
        }
        break;
      case Tunion:
        if ((Table*)p->ref && ((Table*)p->ref)->list)
        {
          if (gflag)
          {
            fprintf(fd, "%*s<__SELECT>\n", n, "");
            for (q = ((Table*)p->ref)->list; q; q = q->next)
              gen_field(fd, n, q, nse, nsa, encoding, 0, 0);
            fprintf(fd, "%*s</__SELECT>\n", n, "");
          }
          else
          {
            if (yflag)
            {
              fprintf(fd, "%*s<!-- a selection of elements", n, "");
              for (q = ((Table*)p->ref)->list; q; q = q->next)
                fprintf(fd, " <%s>,", ns_add(q, nse));
              fprintf(fd, " of which only the first choice is used here -->\n");
            }
            gen_field(fd, n, ((Table*)p->ref)->list, nse, nsa, encoding, 0, 0);
          }
        }
        break;
      default:
        break;
    }
  }
}

void
gen_header(FILE *fd, const char *method, int response, const char *encoding)
{
  if (custom_header)
  {
    Service *sp;
    Method *m = NULL;
    Entry *q;
    Table *r;
    q = entry(classtable, lookup("SOAP_ENV__Header"));
    if (yflag)
      fprintf(fd, " <!-- %s *soap::header -->\n", c_type(q->info.typ));
    fprintf(fd, " <SOAP-ENV:Header>\n");
    if (q)
    {
      r = (Table*)q->info.typ->ref;
      if (r)
      {
        for (q = r->list; q; q = q->next)
        {
          if (!is_transient(q->info.typ) && !(q->info.sto & Sattribute) && q->info.typ->type != Tfun)
          {
            for (sp = services; sp; sp = sp->next)
              for (m = sp->list; m; m = m->next)
                if (is_eq(m->name, method) && (!strcmp(m->part, q->sym->name) || is_eq_nons(m->part, q->sym->name)) && ((!response && (m->mess&HDRIN)) || (response && (m->mess&HDROUT))))
                {
                  gen_field(fd, 2, q, NULL, NULL, encoding, q->info.minOccurs == 0, 0);
                  break;
                }
          }
        }
        fprintf(fd, " </SOAP-ENV:Header>\n");
      }
    }
  }
}

FILE *
gen_env(const char *buf, const char *method, int response, const char *encoding, int soap)
{
  char tmp[1024];
  FILE *fd;
  strcpy(tmp, buf);
  if (!soap)
    strcat(tmp, "REST.");
#ifdef __vms
  if (!response)
  {
    sprintf(strrchr(tmp, '.'), "_%s_req.xml", method);
    fprintf(fmsg, "Saving %s sample SOAP/XML request\n", tmp);
  }
  else
  {
    sprintf(strrchr(tmp, '.'), "_%s_res.xml", method);
    fprintf(fmsg, "Saving %s sample SOAP/XML response\n", tmp);
  }
#else
  strcpy(strrchr(tmp, '.')+1, method);
  if (!response)
  {
    strcat(tmp, ".req.xml");
    fprintf(fmsg, "Saving %s sample SOAP/XML request\n", tmp);
  }
  else
  {
    strcat(tmp, ".res.xml");
    fprintf(fmsg, "Saving %s sample SOAP/XML response\n", tmp);
  }
#endif
  fd = fopen(tmp, "w");
  if (!fd)
    execerror("Cannot write XML file");
  fprintf(fd, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
  if (soap && soap_version >= 0)
  {
    fprintf(fd, "<SOAP-ENV:Envelope");
    gen_xmlns(fd, 1);
    fprintf(fd, ">\n");
    gen_header(fd, method, response, encoding);
    fprintf(fd, " <SOAP-ENV:Body");
    if (eflag && !encoding)
      fprintf(fd, " SOAP-ENV:encodingStyle=\"%s\"", encURI);
    else if (encoding && !*encoding)
      fprintf(fd, " SOAP-ENV:encodingStyle=\"%s\"", encURI);
    else if (encoding && strcmp(encoding, "literal"))
      fprintf(fd, " SOAP-ENV:encodingStyle=\"%s\"", encoding);
    fprintf(fd, ">\n");
  }
  return fd;
}

void
gen_xmlns(FILE *fd, int soap)
{
  Symbol *s;
  Service *sp = NULL;
  for (s = nslist; s; s = s->next)
  {
    for (sp = services; sp; sp = sp->next)
      if (!tagcmp(sp->ns, s->name) && sp->URI)
        break;
    if (sp)
      fprintf(fd, "\n    xmlns:%s=\"%s\"", ns_convert(s->name), sp->URI);
    else if (!strcmp(s->name, "SOAP-ENV"))
    {
      if (soap && soap_version >= 0)
        fprintf(fd, "\n    xmlns:SOAP-ENV=\"%s\"", envURI);
    }
    else if (!strcmp(s->name, "SOAP-ENC"))
    {
      if (soap && soap_version >= 0)
        fprintf(fd, "\n    xmlns:SOAP-ENC=\"%s\"", encURI);
    }
    else if (!strcmp(s->name, "xsi"))
      fprintf(fd, "\n    xmlns:xsi=\"%s\"", xsiURI);
    else if (!strcmp(s->name, "xsd"))
      fprintf(fd, "\n    xmlns:xsd=\"%s\"", xsdURI);
    else
      fprintf(fd, "\n    xmlns:%s=\"%s/%s.xsd\"", ns_convert(s->name), tmpURI, ns_convert(s->name));
  }
}

char *
emalloc(size_t n)
{
  char *p;
  if ((p = (char*)malloc(n)) == NULL)
    execerror("out of memory");
  return p;
}

void
soap_serve(Table *table)
{
  if (!Sflag)
  {
    Entry *method;
    banner(fheader, "Client-Side Call Stub Functions");
    if (rflag)
    {
      Service *sp;
      fprintf(freport, "## Web Client Operations {#doc-client}\n\n");
      for (sp = services; sp; sp = sp->next)
      {
        if (sp->documentation)
        {
          gen_text(freport, sp->documentation);
          fprintf(freport, "\n\n");
        }
      }
    }
    for (method = table->list; method; method = method->next)
    {
      if (method->info.typ->type == Tfun && !(method->info.sto & Sextern) && !is_imported(method->info.typ))
      {
        if (rflag)
          gen_call_proto(freport, method);
        gen_call_proto(fheader, method);
        gen_call_method(fclient, method, NULL);
      }
    }
  }
  if (!Cflag)
  {
    Entry *method, *catch_method;
    if (rflag)
    {
      Service *sp;
      fprintf(freport, "## Web Server Operations {#doc-server}\n\n");
      for (sp = services; sp; sp = sp->next)
      {
        if (sp->documentation)
        {
          gen_text(freport, sp->documentation);
          fprintf(freport, "\n\n");
        }
      }
    }
    banner(fheader, "Server-Side Operations");
    if (!cflag && !namespaceid)
      fprintf(fserver, "extern \"C\" ");
    fprintf(fserver, "SOAP_FMAC5 int SOAP_FMAC6 %s_serve(struct soap *soap)", nflag?prefix:"soap");
    fprintf(fserver, "\n{\n#ifndef WITH_FASTCGI\n\tsoap->keep_alive = soap->max_keep_alive + 1;\n#endif\n\tdo\n\t{");
    fprintf(fserver, "\n#ifndef WITH_FASTCGI\n\t\tif (soap->keep_alive > 0 && soap->max_keep_alive > 0)\n\t\t\tsoap->keep_alive--;\n#endif");
    fprintf(fserver, "\n\t\tif (soap_begin_serve(soap))\n\t\t{\tif (soap->error >= SOAP_STOP)\n\t\t\t\tcontinue;\n\t\t\treturn soap->error;\n\t\t}");
    if (namespaceid)
      fprintf(fserver, "\n\t\tif ((%s::%s_serve_request(soap) || (soap->fserveloop && soap->fserveloop(soap))) && soap->error && soap->error < SOAP_STOP)\n\t\t{\n#ifdef WITH_FASTCGI\n\t\t\tsoap_send_fault(soap);\n#else\n\t\t\treturn soap_send_fault(soap);\n#endif\n\t\t}", namespaceid, nflag?prefix:"soap");
    else
      fprintf(fserver, "\n\t\tif ((%s_serve_request(soap) || (soap->fserveloop && soap->fserveloop(soap))) && soap->error && soap->error < SOAP_STOP)\n\t\t{\n#ifdef WITH_FASTCGI\n\t\t\tsoap_send_fault(soap);\n#else\n\t\t\treturn soap_send_fault(soap);\n#endif\n\t\t}", nflag?prefix:"soap");
    fprintf(fserver, "\n#ifdef WITH_FASTCGI\n\t\tsoap_destroy(soap);\n\t\tsoap_end(soap);\n\t} while (1);\n#else\n\t} while (soap->keep_alive);\n#endif");
    fprintf(fserver, "\n\treturn SOAP_OK;");
    fprintf(fserver, "\n}");
    fprintf(fserver, "\n\n#ifndef WITH_NOSERVEREQUEST\n");
    if (!cflag && !namespaceid)
      fprintf(fserver, "extern \"C\" ");
    fprintf(fserver, "SOAP_FMAC5 int SOAP_FMAC6 %s_serve_request(struct soap *soap)\n{", nflag?prefix:"soap");
    if (sflag)
      fprintf(fserver, "\n\tsoap->mode |= SOAP_XML_STRICT;");
    if (aflag)
    {
      int i, num = 0;
      struct pair *map;
      for (method = table->list; method; method = method->next)
      {
        if (method->info.typ->type == Tfun && !(method->info.sto & Sextern))
        {
          int found = 0;
          Service *sp;
          for (sp = services; sp; sp = sp->next)
          {
            if (has_ns_eq(sp->ns, method->sym->name))
            {
              Method *m;
              for (m = sp->list; m; m = m->next)
              {
                if (is_eq_nons(m->name, method->sym->name))
                {
                  if (m->mess == ACTION || m->mess == REQUEST_ACTION)
                  {
                    ++num;
                    found = 1;
                  }
                }
              }
            }
          }
          if (Aflag && !found)
          {
            sprintf(errbuf, "Option -A requires a SOAPAction specified for operation %s where none is defined", ident(method->sym->name));
            compliancewarn(errbuf);
          }
        }
      }
      map = (struct pair*)emalloc(num * sizeof(struct pair));
      num = 0;
      for (method = table->list; method; method = method->next)
      {
        if (method->info.typ->type == Tfun && !(method->info.sto & Sextern))
        {
          Service *sp;
          for (sp = services; sp; sp = sp->next)
          {
            if (has_ns_eq(sp->ns, method->sym->name))
            {
              Method *m;
              for (m = sp->list; m; m = m->next)
              {
                if (is_eq_nons(m->name, method->sym->name))
                {
                  if (m->mess == ACTION || m->mess == REQUEST_ACTION)
                  {
                    map[num].action = m->part;
                    map[num].method = method;
                    ++num;
                  }
                }
              }
            }
          }
        }
      }
      if (num > 0)
      {
        qsort(map, num, sizeof(struct pair), mapcomp);
        if (num > 4) /* binary search worthwhile when num > 4 */
        {
          fprintf(fserver, "\n\tif (soap->action)\n\t{\n\t\tconst char *soap_action[] = { ");
          for (i = 0; i < num; i++)
          {
            if (*map[i].action == '"')
              fprintf(fserver, "%s, ", map[i].action);
            else
              fprintf(fserver, "\"%s\", ", map[i].action);
          }
          fprintf(fserver, " };");
          fprintf(fserver, "\n\t\tswitch (soap_binary_search_string(soap_action, %d, soap->action))\n\t\t{", num);
          for (i = 0; i < num; i++)
            fprintf(fserver, "\n\t\t\tcase %d:\treturn soap_serve_%s(soap);", i, ident(map[i].method->sym->name));
          fprintf(fserver, "\n\t\t}\n\t}");
        }
        else
        {
          fprintf(fserver, "\n\tif (soap->action)\n\t{");
          for (i = 0; i < num; i++)
          {
            if (*map[i].action == '"')
              fprintf(fserver, "\n\t\tif (!strcmp(soap->action, %s))", map[i].action);
            else
              fprintf(fserver, "\n\t\tif (!strcmp(soap->action, \"%s\"))", map[i].action);
            fprintf(fserver, "\n\t\t\treturn soap_serve_%s(soap);", ident(map[i].method->sym->name));
          }
          fprintf(fserver, "\n\t}");
        }
      }
    }
    if (!Aflag)
    {
      fprintf(fserver, "\n\t(void)soap_peek_element(soap);");
      catch_method = NULL;
      for (method = table->list; method; method = method->next)
      {
        if (method->info.typ->type == Tfun && !(method->info.sto & Sextern))
        {
          if (is_invisible(method->sym->name))
          {
            Entry *param = entry(classtable, method->sym);
            if (param)
              param = ((Table*)param->info.typ->ref)->list;
            if (param)
              fprintf(fserver, "\n\tif (!soap_match_tag(soap, soap->tag, \"%s\"))\n\t\treturn soap_serve_%s(soap);", ns_convert(param->sym->name), ident(method->sym->name));
            else
              catch_method = method;
          }
          else
          {
            fprintf(fserver, "\n\tif (!soap_match_tag(soap, soap->tag, \"%s\"))\n\t\treturn soap_serve_%s(soap);", ns_convert(method->sym->name), ident(method->sym->name));
          }
        }
      }
      if (catch_method)
        fprintf(fserver, "\n\treturn soap_serve_%s(soap);", ident(catch_method->sym->name));
      else
        fprintf(fserver, "\n\treturn soap->error = SOAP_NO_METHOD;");
    }
    else
    {
      fprintf(fserver, "\n\treturn soap->error = SOAP_NO_METHOD;");
    }
    fprintf(fserver, "\n}\n#endif");
    if (rflag)
    {
      fprintf(freport, "Use the service request dispatcher that is auto-generated in [%s](%s) to accept and process service requests:\n\n", soapServer, pathsoapServer);
      fprintf(freport, "    SOAP_FMAC5 int SOAP_FMAC6 %s_serve(struct soap *soap);\n\n", nflag?prefix:"soap");
      fprintf(freport, "This function serves requests by calling one of the service operations listed further below that matches the request.  Returns `SOAP_OK` or an error code.  This function supports CGI by accepting a request on stdin and sending the response to stdout, and FastCGI.  To serve over HTTP(S), use the following functions to establish a connection:\n\n");
      fprintf(freport, "- `SOAP_SOCKET soap_bind(struct soap *soap, const char *host, int port, int backlog)` returns master socket bound to port (and restricted to host name if not NULL) or `SOAP_INVALID_SOCKET` upon error\n");
      fprintf(freport, "- `SOAP_SOCKET soap_accept(struct soap *soap)` accepts connection and returns socket when accepted, or `SOAP_INVALID_SOCKET` upon error\n");
      fprintf(freport, "- `int soap_ssl_accept(struct soap *soap)` performs SSL handshake and returns `SOAP_OK` when successful or an error code, call this function after `soap_accept()` to accept SSL/TLS connection\n\n");
    }
    for (method = table->list; method; method = method->next)
    {
      if (method->info.typ->type == Tfun && !(method->info.sto & Sextern))
      {
        if (rflag)
          generate_proto(freport, table, method);
        generate_proto(fheader, table, method);
      }
    }
    if (rflag)
      fprintf(freport, "\n\n");
    banner(fheader, "Server-Side Skeletons to Invoke Service Operations");
    fprintf(fheader, "\n");
    if (!cflag && !namespaceid)
      fprintf(fheader, "extern \"C\" ");
    fprintf(fheader, "SOAP_FMAC5 int SOAP_FMAC6 %s_serve(struct soap*);", nflag?prefix:"soap");
    fprintf(fheader, "\n\n");
    if (!cflag && !namespaceid)
      fprintf(fheader, "extern \"C\" ");
    fprintf(fheader, "SOAP_FMAC5 int SOAP_FMAC6 %s_serve_request(struct soap*);", nflag?prefix:"soap");
    for (method = table->list; method; method = method->next)
      if (method->info.typ->type == Tfun && !(method->info.sto & Sextern) && !is_imported(method->info.typ))
        gen_serve_method(fserver, table, method, NULL);
  }
}

void
generate_proto(FILE *fd, Table *table, Entry *param)
{
  Entry *q, *pout;
  Table *output;
  Entry *result;
  result = (Entry*)param->info.typ->ref;
  q = entry(table, param->sym);
  if (q)
  {
    pout = (Entry*)q->info.typ->ref;
  }
  else
  {
    fprintf(stderr, "Internal error: no table entry\n");
    return;
  }
  q = entry(classtable, param->sym);
  output = (Table*)q->info.typ->ref;
  if (fd == freport)
    gen_report_operation(NULL, param, 1);
  if (is_transient(pout->info.typ))
    fprintf(fd, "\n    /** Web service one-way operation '%s' implementation, should return value of soap_send_empty_response() to send HTTP Accept acknowledgment, or return an error code, or return SOAP_OK to immediately return without sending an HTTP response message */", ident(param->sym->name));
  else
    fprintf(fd, "\n    /** Web service operation '%s' implementation, should return SOAP_OK or error code */", ident(param->sym->name));
  fprintf(fd, "\n    SOAP_FMAC5 int SOAP_FMAC6 %s(struct soap*", ident(param->sym->name));
  gen_params(fd, output, pout, 1);
  fprintf(fd, ";");
  if (fd == freport)
  {
    fprintf(freport, "\n\nwhere:\n\n- `struct soap *soap` is the context\n");
    gen_report_params(q, result, 1);
    if (!is_transient(result->info.typ))
      fprintf(freport, "This service function should be implemented as part of the service back-end code and return `SOAP_OK` and set the last parameter `%s` to the result, or return an error code\n\n", ident(result->sym->name));
    else
      fprintf(freport, "This service function should be implemented as part of the service back-end code and call `int soap_send_empty_response(struct soap *soap, int httpcode)` with a HTTP status or error code (200 to 599) to return, when communicating over HTTP to return a HTTP header.\n\n");
    gen_report_hr();
  }
}

int
tagcmp(const char *s, const char *t)
{
  size_t i, n;
  if (!s)
    return -1;
  if (!t)
    return 1;
  n = strlen(s);
  for (i = 0; i < n; i++)
  {
    int c = t[i];
    if (c == '_' && s[i] != '_')
      c = '-';
    if (s[i] > c)
      return 1;
    if (s[i] < c)
      return -1;
  }
  return -(t[i] != 0);
}

int
tagncmp(const char *s, const char *t, size_t n)
{
  size_t i;
  if (!s)
    return -1;
  if (!t)
    return 1;
  for (i = 0; i < n; i++)
  {
    int c = t[i];
    if (c == '_' && s[i] != '_')
      c = '-';
    if (s[i] > c)
      return 1;
    if (s[i] < c)
      return -1;
  }
  return 0;
}

int
property(Tnode *p)
{
  if (is_primitive(p))
    return 5; /* collapse white space in primitive type values */
  return p->property;
}

void
property_pattern(Tnode *p, const char *name)
{
  p->property = 1;
  p->pattern = NULL;
  if (is_eq(name, "xsd__QName")
   || is_eq(name, "QName"))
  {
    p->property = 2;
  }
  else if (is_eq(name, "xsd__normalizedString")
        || is_eq(name, "xsd__anyURI"))
  {
    p->property = 4;
  }
  else if (is_eq(name, "xsd__NOTATION")
        || is_eq(name, "xsd__token"))
  {
    p->property = 5;
  }
  else if (is_eq(name, "xsd__language"))
  {
    p->property = 5;
    p->pattern = "([a-zA-Z]{2}|[iI]-[a-zA-Z]+|[xX]-[a-zA-Z]{1,8})(-[a-zA-Z]{1,8})*";
  }
  else if (is_eq(name, "xsd__Name"))
  {
    p->property = 5;
    p->pattern = "\\i\\c*";
  }
  else if (is_eq(name, "xsd__NMTOKEN"))
  {
    p->property = 5;
    p->pattern = "\\c+";
  }
  else if (is_eq(name, "xsd__NMTOKENS"))
  {
    p->property = 5;
    p->pattern = "(\\c+[ ])*\\c+";
  }
  else if (is_eq(name, "xsd__ENTITY")
        || is_eq(name, "xsd__ID")
        || is_eq(name, "xsd__IDREF")
        || is_eq(name, "xsd__NCName"))
  {
    p->property = 5;
    p->pattern = "[\\i-[:]][\\c-[:]]*";
  }
  else if (is_eq(name, "xsd__ENTITIES")
        || is_eq(name, "xsd__IDREFS"))
  {
    p->property = 5;
    p->pattern = "([\\i-[:]][\\c-[:]]*[ ])*[\\i-[:]][\\c-[:]]*";
  }
  else if (is_eq(name, "xsd__date"))
  {
    p->property = 5;
    p->pattern = "[-+]?\\d{4,}-\\d{2}-\\d{2}(Z|[-+]\\d{2}:\\d{2})?";
  }
  else if (is_eq(name, "xsd__dateTime"))
  {
    p->property = 5;
    p->pattern = "[-+]?\\d{4,}-\\d{2}-\\d{2}T\\d{2}:\\d{2}:\\d{2}(Z|[-+]\\d{2}:\\d{2})?";
  }
  else if (is_eq(name, "xsd__decimal"))
  {
    p->property = 5;
    p->pattern = "[-+]?(\\d+|\\d*\\.\\d*)";
  }
  else if (is_eq(name, "xsd__duration"))
  {
    p->property = 5;
    p->pattern = "[-+]?P(\\d+Y)?(\\d+M)?(\\d+D)?(T(\\d+H)?(\\d+M)?(\\d+(\\.\\d*)?S)?)?";
  }
  else if (is_eq(name, "xsd__gDay"))
  {
    p->property = 5;
    p->pattern = "---\\d{2}(Z|[-+]\\d{2}:\\d{2})?";
  }
  else if (is_eq(name, "xsd__gMonth"))
  {
    p->property = 5;
    p->pattern = "--\\d{2}(Z|[-+]\\d{2}:\\d{2})?";
  }
  else if (is_eq(name, "xsd__gMonthDay"))
  {
    p->property = 5;
    p->pattern = "--\\d{2}-\\d{2}(Z|[-+]\\d{2}:\\d{2})?";
  }
  else if (is_eq(name, "xsd__gYear"))
  {
    p->property = 5;
    p->pattern = "([-+]?\\d{4,})(Z|[-+]\\d{2}:\\d{2})?";
  }
  else if (is_eq(name, "xsd__gYearMonth"))
  {
    p->property = 5;
    p->pattern = "([-+]?\\d{4,})-\\d{2}(Z|[-+]\\d{2}:\\d{2})?";
  }
  else if (is_eq(name, "xsd__integer"))
  {
    p->property = 5;
    p->pattern = "[-+]?\\d+";
  }
  else if (is_eq(name, "xsd__negativeInteger"))
  {
    p->property = 5;
    p->pattern = "-\\d*[1-9]\\d*";
  }
  else if (is_eq(name, "xsd__nonNegativeInteger"))
  {
    p->property = 5;
    p->pattern = "\\+?\\d+";
  }
  else if (is_eq(name, "xsd__nonPositiveInteger"))
  {
    p->property = 5;
    p->pattern = "-\\d+|\\+?0+";
  }
  else if (is_eq(name, "xsd__positiveInteger"))
  {
    p->property = 5;
    p->pattern = "\\+?\\d*[1-9]\\d*";
  }
  else if (is_eq(name, "xsd__time"))
  {
    p->property = 5;
    p->pattern = "\\d{2}:\\d{2}:\\d{2}(Z|[-+]\\d{2}:\\d{2})?";
  }
}

int
is_qname(Tnode *p)
{
  if (!is_string(p) && !is_wstring(p))
    return 0;
  if (p->property == 2)
    return 1;
  if (p->sym && (is_eq(p->sym->name, "xsd__QName") || is_eq(p->sym->name, "QName")))
    return 1;
  return p->id && (is_eq(p->id->name, "xsd__QName") || is_eq(p->id->name, "QName"));
}

int
is_stdqname(Tnode *p)
{
  if (!is_stdstring(p) && !is_stdwstring(p))
    return 0;
  if (p->property == 2)
    return 1;
  if (p->sym && (is_eq(p->sym->name, "xsd__QName") || is_eq(p->sym->name, "QName")))
    return 1;
  return p->id && (is_eq(p->id->name, "xsd__QName") || is_eq(p->id->name, "QName"));
}

int
is_XML(Tnode *p)
{
  if (is_synonym(p))
    return (is_string(p) || is_wstring(p)) && is_eq(p->synonym->name, "XML");
  return (p->sym && (is_string(p) || is_wstring(p)) && is_eq(p->sym->name, "XML")) || ((p->type == Tpointer || p->type == Treference || p->type == Trvalueref) && is_XML((Tnode*)p->ref));
}

int
is_stdXML(Tnode *p)
{
  if (is_synonym(p))
    return (is_stdstring(p) || is_stdwstring(p)) && is_eq(p->synonym->name, "XML");
  return p->sym && (is_stdstring(p) || is_stdwstring(p)) && is_eq(p->sym->name, "XML");
}

int
is_response(Tnode *p)
{
  return (p->type == Tpointer || p->type == Treference || p->type == Trvalueref)
    && p->ref
    && has_ns((Tnode*)p->ref)
    && ((((Tnode*)p->ref)->type == Tstruct || ((Tnode*)p->ref)->type == Tclass) && !is_external((Tnode*)p->ref) && !is_primclass((Tnode*)p->ref) && !is_dynamic_array((Tnode*)p->ref) && !is_stdstring((Tnode*)p->ref) && !is_stdwstring((Tnode*)p->ref));
}

Entry*
get_response(Tnode *p)
{
  if (p->type == Tfun)
    return p->response;
  return 0;
}

int
is_unmatched(Symbol *sym)
{
  const char *s = sym->name;
  if (*s == ':')
    s++;
  return s[0] == '_' && s[1] != '_' && !is_special(s);
}

int
is_special(const char *s)
{
  return strncmp(s, "_DOT", 4) == 0
    || strncmp(s, "_USCORE", 7) == 0
    || (strncmp(s, "_x", 2) == 0 && isxdigit(s[2]) && isxdigit(s[3]) && isxdigit(s[4]) && isxdigit(s[5]));
}

int
is_invisible(const char *name)
{
  return name[0] == '-' || (name[0] == '_' && name[1] == '_' && strncmp(name, "__ptr", 5));
}

int
is_invisible_empty(Tnode *p)
{
  if (p->type == Tstruct || p->type == Tclass)
    if (is_invisible(p->id->name))
      if (!p->ref || !((Table*)p->ref)->list)
        return 1;
  return 0;
}

int
is_element(Tnode *typ)
{
  if (is_XML(typ) || is_stdXML(typ) || is_qname(typ) || is_stdqname(typ))
    return 0;
  if (typ->sym)
    return is_unmatched(typ->sym);
  if (typ->type == Tstruct || typ->type == Tclass)
    return is_unmatched(typ->id);
  return 0;
}

int
is_untyped(Tnode *typ)
{
  Tnode *p;
  if (typ->sym)
    return is_unmatched(typ->sym);
  if (typ->type == Tpointer || typ->type == Treference || typ->type == Trvalueref || typ->type == Tarray)
    return is_untyped((Tnode*)typ->ref);
  if (typ->type == Tstruct || typ->type == Tclass)
  {
    if (is_dynamic_array(typ) && !has_ns(typ) && !is_binary(typ))
    {
      p = (Tnode*)((Table*)typ->ref)->list->info.typ->ref;
      return is_untyped(p);
    }
    else
      return is_unmatched(typ->id);
  }
  return 0;
}

int
is_primclass(Tnode *typ)
{
  Table *t;
  if (typ->type == Tstruct || typ->type == Tclass)
  {
    if (!is_dynamic_array(typ))
    {
      t = (Table*)typ->ref;
      while (t)
      {
        Entry *p = t->list;
        while (p && !is_item(p))
          p = p->next;
        if (p)
          break;
        t = t->prev;
      }
      if (!t)
        return 0;
      t = (Table*)typ->ref;
      while (t)
      {
        Entry *p;
        for (p = t->list; p; p = p->next)
          if (!is_item(p) && p->info.typ->type != Tfun && !is_transient(p->info.typ) && p->info.sto != Sattribute && p->info.sto != Sprivate && p->info.sto != Sprotected)
            return 0;
        t = t->prev;
      }
      return 1;
    }
  }
  else if (typ->type == Tpointer || typ->type == Treference || typ->type == Trvalueref)
    return is_primclass((Tnode*)typ->ref);
  return 0;
}

int
is_mask(Tnode *typ)
{
  return ((typ->type == Tenum || typ->type == Tenumsc) && typ->width == 9);
}

int
is_void(Tnode *typ)
{
  if (!typ)
    return 1;
  if (typ->type == Tvoid)
    return 1;
  if (typ->type == Tpointer || typ->type == Treference || typ->type == Trvalueref || typ->type == Tarray || typ->type == Ttemplate)
    return is_void((Tnode*)typ->ref);
  return 0;
}

int
is_pointer_to_derived(Entry *e)
{
  if (is_soapref(e->info.typ))
    return 0;
  if (e->info.typ->type == Tpointer && !(e->info.sto & (Sconst | Sprivate | Sprotected)) && is_transient(e->info.typ))
  {
    Tnode *ref = e->info.typ->ref;
    if (ref && ref->id && !is_transient(ref))
    {
      if (ref->sym)
        return is_eq(ref->sym->name, e->sym->name);
      return is_eq(ref->id->name, e->sym->name);
    }
  }
  return 0;
}

void
gen_match_derived(FILE *fd, Tnode *typ)
{
  fprintf(fd, "!soap_match_tag(soap, soap->type, \"%s\")", xsi_type(typ));
  if ((typ->type == Tstruct || typ->type == Tclass) && typ->ref && !is_transient(typ))
  {
    Entry *p;
    for (p = ((Table*)typ->ref)->list; p; p = p->next)
    {
      if (is_pointer_to_derived(p))
      {
        fprintf(fout, " || ");
        gen_match_derived(fd, p->info.typ->ref);
      }
    }
  }
}

void
base_of_derived(Entry *p)
{
  if (p->info.typ->ref)
  {
    Entry *e;
    for (e = ((Table*)p->info.typ->ref)->list; e; e = e->next)
    {
      if (!is_soapref(e->info.typ) && e->info.typ->type == Tpointer && !(e->info.sto & (Sconst | Sprivate | Sprotected)) && is_transient(e->info.typ))
      {
        Tnode *q = (Tnode*)e->info.typ->ref;
        if (q && q->id && is_eq(q->sym ? q->sym->name : q->id->name, e->sym->name))
        {
          if (q->baseid || q->base)
          {
            sprintf(errbuf, "%s declared at %s:%d has multiple base types, including %s", c_type(p->info.typ), p->filename, p->lineno, c_type(q));
            semwarn(errbuf);
          }
          else
          {
            q->base = p->info.typ;
          }
        }
      }
    }
  }
}

int
is_transient(Tnode *typ)
{
  if (!typ)
    return 1;
  if (typ->type == Tstruct && typ->id == lookup("soap"))
    return 1;
  if (is_external(typ) || is_volatile(typ))
    return 0;
  if (typ->transient > 0)
    return 1;
  if (typ->type == Tpointer && ((Tnode*)typ->ref)->type == Twchar)
  {
    /* wchar_t* is not transient unless its wchar_t is transient or external */
    Tnode *ref = (Tnode*)typ->ref;
    if (is_external(ref) || is_volatile(ref))
      return 0;
    if (ref->transient > 0)
      return 1;
    return 0;
  }
  switch (typ->type)
  {
    case Tpointer:
    case Treference:
    case Trvalueref:
    case Tarray:
    case Ttemplate:
      return is_transient((Tnode*)typ->ref);
    case Tstruct:
    case Tclass:
    case Tunion:
      return typ->ref == NULL; /* declared but undefined structs, classes, and unions are transient */
    case Tnone:
    case Tvoid:
    case Twchar: /* wchar_t is transient */
    case Tsize:  /* size_t is transient */
      return 1;
    default:
      break;
  }
  return 0;
}

int
is_imported(Tnode* typ)
{
  return typ->imported != NULL;
}

int
is_external(Tnode* typ)
{
  return typ->transient == -1 || typ->transient == -3;
}

int
is_anyType(Tnode* typ)
{
  if (!typ)
    return 0;
  if (typ->type == Tpointer || typ->type == Ttemplate)
    return is_anyType((Tnode*)typ->ref);
  if (is_external(typ) && typ->type == Tstruct && !strcmp(typ->id->name, "soap_dom_element"))
    return is_anyType_flag = 1;
  return 0;
}

int
is_anyAttribute(Tnode* typ)
{
  if (!typ)
    return 0;
  if (typ->type == Tpointer || typ->type == Ttemplate)
    return is_anyAttribute((Tnode*)typ->ref);
  return is_external(typ) && typ->type == Tstruct && !strcmp(typ->id->name, "soap_dom_attribute");
}

int
is_volatile(Tnode* typ)
{
  return typ->transient == -2 || typ->transient == -3;
}

int
is_smart(Tnode *p)
{
  return p->type == Ttemplate && p->ref && is_volatile(p);
}

int
is_smart_shared(Tnode *p)
{
  if (is_smart(p))
  {
    const char *s = strstr(p->id->name, "::");
    if (s)
      return !strcmp(s, "::shared_ptr"); /* support shared_ptr STL and Boost */
  }
  return 0;
}

const char *
make_shared(Tnode *p)
{
  const char *s = "";
  /* should produce make_shared only if is_smart_shared(p) is true */
  if (is_smart_shared(p))
  {
    char *t;
    s = strstr(p->id->name, "::");
    t = emalloc(s - p->id->name + 14);
    strncpy(t, p->id->name, s - p->id->name + 2);
    strcpy(t + (s - p->id->name) + 2, "make_shared");
    s = t;
  }
  return s;
}

int
is_container(Tnode *p)
{
  if (p->type == Tpointer)
    return is_container((Tnode*)p->ref);
  return p->type == Ttemplate && !is_smart(p);
}

int
is_template(Tnode *p)
{
  if (p->type == Tpointer)
    return is_template((Tnode*)p->ref);
  return p->type == Ttemplate;
}

int
is_repetition(Entry *p)
{
  if (p)
    return p->next && p->next->info.typ->type == Tpointer && ((Tnode*)p->next->info.typ->ref)->type != Tvoid && (p->info.typ->type == Tint || p->info.typ->type == Tsize) && ((p->info.sto & Sspecial) || !strncmp(p->sym->name, "__size", 6));
  return 0;
}

int
is_item(Entry *p)
{
  if (p)
  {
    size_t n;
    const char *s = p->sym->name;
    for (n = strlen(s) - 1; n && s[n] == '_'; n--)
      ;
    return !strncmp(s, "__item", n + 1);
  }
  return 0;
}

int
is_self(Entry *p)
{
  if (p)
  {
    size_t n;
    const char *s = p->sym->name;
    for (n = strlen(s) - 1; n && s[n] == '_'; n--)
      ;
    return !strncmp(s, "__self", n + 1);
  }
  return 0;
}

int
is_choice(Entry *p)
{
  if (p)
    if (p->next && p->next->info.typ->type == Tunion && p->info.typ->type == Tint && ((p->info.sto & Sspecial) || !strncmp(p->sym->name, "__union", 7)))
      return 1;
  return 0;
}

int
required_choice(Tnode *typ)
{
  if (typ->type == Tunion && ((Table*)typ->ref))
  {
    Entry *p;
    for (p = ((Table*)typ->ref)->list; p; p = p->next)
    {
      if (!(p->info.sto & (Sconst | Sprivate | Sprotected))
          && !is_transient(p->info.typ)
          && (!(p->info.sto & Sattribute))
          && !is_repetition(p)
          && !is_anytype(p))
      {
        if (p->info.minOccurs <= 0)
          return 0;
      }
    }
  }
  return -1;
}

int
is_sequence(Entry *p)
{
  if (p)
  {
    Tnode *q = p->info.typ;
    if (q->type == Tpointer)
      q = (Tnode*)q->ref;
    if ((q->type == Tstruct || q->type == Tclass) && is_invisible(p->sym->name) && is_invisible(q->id->name) && !is_transient(q))
      return 1;
  }
  return 0;
}


int
is_anytype(Entry *p)
{
  if (p)
  {
    if (p->next && p->next->info.typ->type == Tpointer && ((Tnode*)p->next->info.typ->ref)->type == Tvoid && p->info.typ->type == Tint && ((p->info.sto & Sspecial) || !strncmp(p->sym->name, "__type", 6)))
    {
      is_anytype_flag = 1;
      return 1;
    }
  }
  return 0;
}

int
is_keyword(const char *name)
{
  Symbol *s = lookup(name);
  if (s)
    return s->token != ID;
  return 0;
}


int
has_ptr(Tnode *typ)
{
  Tnode *p;
  if (typ->type == Tpointer || typ->type == Treference || typ->type == Trvalueref)
    return 0;
  for (p = Tptr[Tpointer]; p; p = p->next)
    if ((Tnode*)p->ref == typ && p->transient != 1)
      return 1;
  return 0;
}

int
has_detail_string(void)
{
  Entry *p = entry(classtable, lookup("SOAP_ENV__Fault"));
  if (p && p->info.typ->ref && (p->info.typ->type == Tstruct || p->info.typ->type == Tclass))
  {
    Entry *e = entry((Table*)p->info.typ->ref, lookup("detail"));
    if (e && e->info.typ->ref && e->info.typ->type == Tpointer && ((Tnode*)e->info.typ->ref)->type == Tstruct)
    {
      Entry *e2 = entry((Table*)((Tnode*)e->info.typ->ref)->ref, lookup("__any"));
      return e2 && is_string(e2->info.typ);
    }
  }
  return 0;
}

int
has_Detail_string(void)
{
  Entry *p = entry(classtable, lookup("SOAP_ENV__Fault"));
  if (p && p->info.typ->ref && (p->info.typ->type == Tstruct || p->info.typ->type == Tclass))
  {
    Entry *e = entry((Table*)p->info.typ->ref, lookup("SOAP_ENV__Detail"));
    if (e && e->info.typ->ref && e->info.typ->type == Tpointer && ((Tnode*)e->info.typ->ref)->type == Tstruct)
    {
      Entry *e2 = entry((Table*)((Tnode*)e->info.typ->ref)->ref, lookup("__any"));
      return e2 && is_string(e2->info.typ);
    }
  }
  return 0;
}

int
has_class(Tnode *typ)
{
  Entry *p;
  if (!cflag && typ->type == Tstruct && typ->ref)
  {
    for (p = ((Table*)typ->ref)->list; p; p = p->next)
    {
      if (p->info.sto & Stypedef)
        continue;
      if (p->info.typ->type == Tclass || p->info.typ->type == Ttemplate)
        return 1;
      if (p->info.typ->type == Tstruct && has_class(p->info.typ))
        return 1;
    }
  }
  return 0;
}

int
has_external(Tnode *typ)
{
  Entry *p;
  if ((typ->type == Tstruct || typ->type == Tclass) && typ->ref)
  {
    for (p = ((Table*)typ->ref)->list; p; p = p->next)
    {
      if (p->info.typ->type == Tstruct || p->info.typ->type == Tclass)
      {
        if (is_external(p->info.typ) || has_external(p->info.typ))
          return 1;
      }
    }
  }
  return 0;
}

int
has_volatile(Tnode *typ)
{
  Entry *p;
  if ((typ->type == Tstruct || typ->type == Tclass) && typ->ref)
  {
    for (p = ((Table*)typ->ref)->list; p; p = p->next)
    {
      if (p->info.typ->type == Tstruct || p->info.typ->type == Tclass)
      {
        if (is_volatile(p->info.typ) || has_volatile(p->info.typ))
          if (!is_stdstr(p->info.typ))
            return 1;
      }
    }
  }
  return 0;
}

int
has_ns(Tnode *typ)
{
  if (typ->type == Tstruct || typ->type == Tclass || typ->type == Tenum || typ->type == Tenumsc)
    return has_ns_eq(NULL, typ->id->name);
  return 0;
}

int
has_ns_t(Tnode *typ)
{
  if (typ->sym)
    return has_ns_eq(NULL, typ->sym->name);
  return has_ns(typ);
}

/* needs_lang adds xml:lang attribute to matching struct/class member name
   we should use an annotation for soapcpp2's input this in the future instead
   of a hard-coded member name */
void
needs_lang(Entry *e)
{
  if (!strcmp(e->sym->name, "SOAP_ENV__Text"))
    fprintf(fout, "\n\tif (soap->lang)\n\t\tsoap_set_attr(soap, \"xml:lang\", soap->lang, 1);");
}

int
is_eq_nons(const char *s, const char *t)
{
#ifdef SOAP_OLD_DIRECTIVE_NAME_MATCHING
  size_t n, m;
#endif
  const char *r;
  while (*s == '_' || *s == ':')
    s++;
  while (*t == '_' || *t == ':')
    t++;
  if (!*s || !*t)
    return 0;
  r = strstr(t, "__");
  if (r)
    t = r + 2;
#ifdef SOAP_OLD_DIRECTIVE_NAME_MATCHING
  n = strlen(s) - 1;
  m = strlen(t) - 1;
  while (n > 0 && s[n] == '_')
    n--;
  while (m > 0 && t[m] == '_')
    m--;
  if (n != m)
    return 0;
  return !strncmp(s, t, n + 1);
#else
  return !strcmp(s, t);
#endif
}

int
is_eq(const char *s, const char *t)
{
  size_t n, m;
  while (*s == '_' || *s == ':')
    s++;
  while (*t == '_' || *t == ':')
    t++;
  if (!*s || !*t)
    return 0;
  for (n = strlen(s) - 1; n && s[n] == '_'; n--)
    ;
  for (m = strlen(t) - 1; m && t[m] == '_'; m--)
    ;
  if (n != m)
    return 0;
  return !strncmp(s, t, n + 1);
}

int
has_ns_eq(const char *ns, const char *s)
{
  size_t n;
  while (*s == '_' || *s == ':')
    s++;
  if (!ns)
  {
    const char *t = *s ? strstr(s + 1, "__") : NULL;
    if (!t || is_special(t+1))
    {
      t = strchr(s, ':');
      if (t && t[1] == ':')
        t = NULL;
    }
    return t && t[1] && t[2] && (t[2] != '_' || is_special(t+2));
  }
  if ((n = strlen(ns)) < strlen(s))
    return ((s[n] == '_' && s[n+1] == '_') || (s[n] == ':' && s[n+1] != ':')) && !tagncmp(ns, s, n);
  return 0;
}

const char *
strict_check(void)
{
  if (sflag)
    return "";
  return "(soap->mode & SOAP_XML_STRICT) && ";
}

void
fixed_check(FILE *fd, Entry *e, Table *t, const char *tabs)
{
  const char *name = NULL;
  const char *type = NULL;
  char *buf;
  Tnode *p = e->info.typ;
  if (!e || (!e->info.hasval && !e->info.ptrval) || !e->info.fixed)
    return;
  name = ident(e->sym->name);
  if (t)
    type = ident(t->sym->name);
  buf = emalloc(2*strlen(name) + 2*(t ? strlen(type) + 2 : 0) + 20);
  if (e->info.ptrval)
    strcpy(buf, "(*a->");
  else
    strcpy(buf, "a->");
  if (t)
  {
    strcat(buf, type);
    strcat(buf, "::");
  }
  strcat(buf, name);
  if (e->info.ptrval)
    strcat(buf, ")");
  fprintf(fd, "\n%sif (*soap->href != '#' && (%s", tabs, strict_check());
  if (e->info.ptrval)
  {
    fprintf(fd, "a->");
    if (t)
      fprintf(fd, "%s::", type);
    fprintf(fd, "%s", name);
    if (is_smart(e->info.typ))
      fprintf(fd, ".get()");
    fprintf(fd, " && ");
    p = p->ref;
  }
  switch (p->type)
  {
    case Tchar:
    case Twchar:
    case Tuchar:
    case Tshort:
    case Tushort:
    case Tint:
    case Tuint:
    case Ttime:
      fprintf(fd, "%s != " SOAP_LONG_FORMAT, buf, e->info.val.i);
      break;
    case Tlong:
      fprintf(fd, "%s != " SOAP_LONG_FORMAT "L", buf, e->info.val.i);
      break;
    case Tulong:
      fprintf(fd, "%s != " SOAP_LONG_FORMAT "UL", buf, e->info.val.i);
      break;
    case Tllong:
      fprintf(fd, "%s != " SOAP_LONG_FORMAT "LL", buf, e->info.val.i);
      break;
    case Tullong:
    case Tsize:
      fprintf(fd, "%s != " SOAP_LONG_FORMAT "ULL", buf, e->info.val.i);
      break;
    case Tfloat:
    case Tdouble:
      fprintf(fd, "%s != %g", buf, e->info.val.r);
      break;
    case Tldouble:
      fprintf(fd, "%s != %gL", buf, e->info.val.r);
      break;
    case Tenum:
    case Tenumsc:
      if (e->info.val.i <= 0x7FFFLL && e->info.val.i >= -0x8000LL)
        fprintf(fd, "%s != (%s)" SOAP_LONG_FORMAT, buf, c_type(p), e->info.val.i);
      else
        fprintf(fd, "%s != (%s)" SOAP_LONG_FORMAT "LL", buf, c_type(p), e->info.val.i);
      break;
    default:
      if (is_stdstring(p) && e->info.val.s)
        fprintf(fd, "%s.compare(\"%s\")", buf, cstring(e->info.val.s, 0));
      else if (is_stdwstring(p) && e->info.val.s)
        fprintf(fd, "%s.compare(L\"%s\")", buf, cstring(e->info.val.s, 0));
      else if (is_wstring(p) && e->info.val.s)
        fprintf(fd, "wcscmp(%s, L\"%s\")", buf, cstring(e->info.val.s, 0));
      else if (is_string(p) && e->info.val.s)
        fprintf(fd, "strcmp(%s, \"%s\")", buf, cstring(e->info.val.s, 0));
      else
        fprintf(fd, "0");
      break;
  }
  fprintf(fd, "))\n%s{\tsoap->error = SOAP_FIXED;\n%s\treturn NULL;\n%s}", tabs, tabs, tabs);
}

const char *
ns_of(const char *name)
{
  Service *sp;
  for (sp = services; sp; sp = sp->next)
    if (has_ns_eq(sp->ns, name))
      break;
  if (sp)
    return sp->URI;
  return NULL;
}

int
eq_ns(const char *s, const char *t)
{
  return ns_of(s) == ns_of(t);
}

const char *
prefix_of(const char *s)
{
  const char *t;
  if (*s == ':' && s[1] != ':')
    return NULL;
  while (*s == '_' || *s == ':')
    s++;
  t = *s ? strstr(s + 1, "__") : NULL;
  if (!t)
  {
    t = strchr(s, ':');
    if (t && t[1] == ':')
      t = NULL;
  }
  if (t && t[1] && t[2] && (t[2] != '_' || is_special(t+2)))
  {
    char *r = (char*)emalloc(t - s + 1);
    strncpy(r, s, t - s);
    r[t - s] = '\0';
    return r;
  }
  return NULL;
}

const char *
ns_add_overridden(Table *t, Entry *p, const char *ns)
{
  Entry *q;
  Symbol *s = t->sym;
  if (s)
  {
    do
    {
      for (q = t->list; q; q = q->next)
        if (!strcmp(q->sym->name, p->sym->name))
          return ns_add(p, ns ? prefix_of(t->sym->name) : NULL);
    } while ((t = t->prev) != NULL);
  }
  return ns_add(p, ns);
}


const char *
c_ident(Tnode *typ)
{
  if (typ->sym && strcmp(typ->sym->name, "/*?*/"))
    return res_remove(typ->sym->name);
  return t_ident(typ);
}

const char *
soap_type(Tnode *typ)
{
  const char *t = c_ident(typ);
  char *s;
  if (namespaceid && (Qflag || !is_external(typ)))
  {
    s = (char*)emalloc(strlen(t) + strlen(namespaceid) + 12);
    strcpy(s, "SOAP_TYPE_");
    strcat(s, namespaceid);
    strcat(s, "_");
  }
  else
  {
    s = (char*)emalloc(strlen(t) + 11);
    strcpy(s, "SOAP_TYPE_");
  }
  strcat(s, t);
  return s;
}

const char *
soap_union_member(Tnode *typ, Entry *p)
{
  const char *t = c_ident(typ);
  const char *n = ident(p->sym->name);
  char *s;
  if (namespaceid && (zflag == 0 || zflag > 3))
  {
    s = (char*)emalloc(strlen(t) + strlen(n) + strlen(namespaceid) + 14);
    strcpy(s, "SOAP_UNION_");
    strcat(s, namespaceid);
    strcat(s, "_");
  }
  else
  {
    s = (char*)emalloc(strlen(t) + strlen(n) + 13);
    strcpy(s, "SOAP_UNION_");
  }
  strcat(s, t);
  strcat(s, "_");
  strcat(s, n);
  return s;
}


const char *
ident(const char *name)
{
  if (name)
  {
    const char *s = strrchr(name, ':');
    if (s && *(s+1) && (s == name || *(s-1) != ':'))
      return s+1;
  }
  return name;
}

/*t_ident gives the base type name (e.g. when typedef'ed) of a type in identifier format*/
const char *
t_ident(Tnode *typ)
{
  char *p;
  const char *q;
  if (typ->extsym)
    return ident(typ->extsym->name);
  if (typ->restriction)
    return ident(typ->restriction->name);
  switch(typ->type)
  {
    case Tnone:
      return "";
    case Tvoid:
      return "void";
    case Tchar:
      return "byte";
    case Twchar:
      return "wchar";
    case Tshort:
      return "short";
    case Tint:
      return "int";
    case Tlong:
      return "long";
    case Tllong:
      return "LONG64";
    case Tfloat:
      return "float";
    case Tdouble:
      return "double";
    case Tldouble:
      return "decimal";
    case Tuchar:
      return "unsignedByte";
    case Tushort:
      return "unsignedShort";
    case Tuint:
      return "unsignedInt";
    case Tulong:
      return "unsignedLong";
    case Tullong:
      return "ULONG64";
    case Tsize:
      return "size_t";
    case Ttime:
      return "dateTime";
    case Tstruct:
    case Tclass:
    case Tunion:
    case Tenum:
      if (is_bool(typ))
        return "bool";
    case Tenumsc:
      return res_remove(typ->id->name);
    case Treference:
    case Trvalueref:
      return c_ident((Tnode*)typ->ref);
    case Tpointer:
      if (is_string(typ))
        return "string";
      if (is_wstring(typ))
        return "wstring";
      p = (char*)emalloc((10+strlen(q = c_ident((Tnode*)typ->ref))) * sizeof(char));
      strcpy(p, "PointerTo");
      strcat(p, q);
      return p;
    case Tarray:
      p = (char*)emalloc((16+strlen(c_ident((Tnode*)typ->ref))) * sizeof(char));
      if (((Tnode*)typ->ref)->width)
        sprintf(p, "Array%dOf%s", typ->width / ((Tnode*) typ->ref)->width, c_ident((Tnode*)typ->ref));
      else
        sprintf(p, "ArrayOf%s", c_ident((Tnode*)typ->ref));
      return p;
    case Ttemplate:
      if (typ->ref)
      {
        p = (char*)emalloc((11+strlen(res_remove(typ->id->name))+strlen(q = c_ident((Tnode*)typ->ref))) * sizeof(char));
        strcpy(p, res_remove(typ->id->name));
        strcat(p, "TemplateOf");
        strcat(p, q);
        return p;
      }
    case Tfun:
      return "Function";
  }
  return "anyType";
}

void
utf8(char **t, long c)
{
  if (c < 0x0080)
    *(*t)++ = (char)c;
  else
  {
    if (c < 0x0800)
      *(*t)++ = (char)(0xC0 | ((c >> 6) & 0x1F));
    else
    {
      if (c < 0x010000)
        *(*t)++ = (char)(0xE0 | ((c >> 12) & 0x0F));
      else
      {
        if (c < 0x200000)
          *(*t)++ = (char)(0xF0 | ((c >> 18) & 0x07));
        else
        {
          if (c < 0x04000000)
            *(*t)++ = (char)(0xF8 | ((c >> 24) & 0x03));
          else
          {
            *(*t)++ = (char)(0xFC | ((c >> 30) & 0x01));
            *(*t)++ = (char)(0x80 | ((c >> 24) & 0x3F));
          }
          *(*t)++ = (char)(0x80 | ((c >> 18) & 0x3F));
        }
        *(*t)++ = (char)(0x80 | ((c >> 12) & 0x3F));
      }
      *(*t)++ = (char)(0x80 | ((c >> 6) & 0x3F));
    }
    *(*t)++ = (char)(0x80 | (c & 0x3F));
  }
  *(*t) = '\0';
}

const char *
ns_tag_convert(Entry *p)
{
  if (p->tag)
    return p->tag;
  return ns_convert(p->sym->name);
}

const char *
ns_convert(const char *tag)
{
  const char *t;
  char *s;
  size_t i, n;
  if (*tag == ':')
    tag++;
  if (*tag == '_')
  {
    if (!strncmp(tag, "__ptr", 5))
    {
      if (tag[5])
        tag += 5;
      else
        tag = "item";
    }
    else if (!is_special(tag))
    {
      tag++; /* skip leading _ */
    }
  }
  for (n = strlen(tag); n > 0; n--)
  {
    if (tag[n-1] != '_')
      break;
  }
  t = s = (char*)emalloc(n+1);
  for (i = 0; i < n; i++)
  {
    if (tag[i] == '_')
    {
      if (tag[i+1] == '_' && !(tag[i+2] == 'x' && isxdigit(tag[i+3]) && isxdigit(tag[i+4]) && isxdigit(tag[i+5]) && isxdigit(tag[i+6])))
        break;
      else if (!strncmp(tag+i, "_DOT", 4))
      {
        *s++ = '.';
        i += 3;
      }
      else if (!strncmp(tag+i, "_USCORE", 7))
      {
        *s++ = '_';
        i += 6;
      }
      else if (!strncmp(tag+i, "_x", 2) && isxdigit(tag[i+2]) && isxdigit(tag[i+3]) && isxdigit(tag[i+4]) && isxdigit(tag[i+5]))
      {
        char d[5];
        strncpy(d, tag+i+2, 4);
        d[4] = '\0';
        utf8(&s, strtoul(d, NULL, 16));
        i += 5;
      }
      else
        *s++ = '-';
    }
    else if (!strncmp(tag+i, "\\u", 2) && isxdigit(tag[i+2]) && isxdigit(tag[i+3]) && isxdigit(tag[i+4]) && isxdigit(tag[i+5]))
    {
      char d[5];
      strncpy(d, tag+i+2, 4);
      d[4] = '\0';
      utf8(&s, strtoul(d, NULL, 16));
      i += 5;
    }
    else if (tag[i] == ':' && tag[i+1] == ':')
      break;
    else
      *s++ = tag[i];
  }
  if (i < n)
  {
    *s++ = ':';
    for (i += 2; i < n; i++)
    {
      if (tag[i] == '_')
      {
        if (!strncmp(tag+i, "_DOT", 4))
        {
          *s++ = '.';
          i += 3;
        }
        else if (!strncmp(tag+i, "_USCORE", 7))
        {
          *s++ = '_';
          i += 6;
        }
        else if (!strncmp(tag+i, "_x", 2) && isxdigit(tag[i+2]) && isxdigit(tag[i+3]) && isxdigit(tag[i+4]) && isxdigit(tag[i+5]))
        {
          char d[5];
          strncpy(d, tag+i+2, 4);
          d[4] = '\0';
          utf8(&s, strtoul(d, NULL, 16));
          i += 5;
        }
        else
          *s++ = '-';
      }
      else if (!strncmp(tag+i, "\\u", 2) && isxdigit(tag[i+2]) && isxdigit(tag[i+3]) && isxdigit(tag[i+4]) && isxdigit(tag[i+5]))
      {
        char d[5];
        strncpy(d, tag+i+2, 4);
        d[4] = '\0';
        utf8(&s, strtoul(d, NULL, 16));
        i += 5;
      }
      else
        *s++ = tag[i];
    }
  }
  *s = '\0';
  return t;
}

const char *
res_remove(const char *tag)
{
  char *s;
  const char *t;
  if (!(t = strchr(tag, ':')))
    return tag;
  if (t[1] != ':')
    tag = t + 1;
  if (!strchr(tag, ':'))
    return tag;
  s = (char*)emalloc(strlen(tag) + 1);
  t = strcpy(s, tag);
  while ((s = strchr(s, ':')))
    *s = '_';
  return t;
}

const char *
ns_qualifiedElement(Tnode *typ)
{
  const char *s = NULL;
  if (typ->sym)
    s = typ->sym->name;
  if (!s && typ->id)
    s = typ->id->name;
  return ns_qualifiedElementName(s);
}

const char *
ns_qualifiedElementName(const char *s)
{
  Service *sp;
  if (!s)
    return NULL;
  s = prefix_of(s);
  if (!s)
    return NULL;
  for (sp = services; sp; sp = sp->next)
  {
    if (sp->elementForm && !tagcmp(sp->ns, s))
    {
      if (!strcmp(sp->elementForm, "qualified"))
        return s;
      return NULL;
    }
  }
  return NULL;
}

const char *
ns_qualifiedAttribute(Tnode *typ)
{
  const char *s = NULL;
  if (typ->sym)
    s = typ->sym->name;
  if (!s && typ->id)
    s = typ->id->name;
  return ns_qualifiedAttributeName(s);
}

const char *
ns_qualifiedAttributeName(const char *s)
{
  Service *sp;
  if (!s)
    return NULL;
  s = prefix_of(s);
  if (!s)
    return NULL;
  for (sp = services; sp; sp = sp->next)
  {
    if (sp->attributeForm && !tagcmp(sp->ns, s))
    {
      if (!strcmp(sp->attributeForm, "qualified"))
        return s;
      return NULL;
    }
  }
  return NULL;
}

const char *
field(Entry *p, const char *ns)
{
  const char *r;
  char *s;
  if (is_self(p))
    return "tag";
  r = ns_add(p, ns);
  s = (char*)emalloc(strlen(r) + 3);
  strcpy(s, "\"");
  strcat(s, r);
  strcat(s, "\"");
  return s;
}

const char *
field_overridden(Table *t, Entry *p, const char *ns)
{
  const char *r;
  char *s;
  if (is_self(p))
    return "tag";
  r = ns_add_overridden(t, p, ns);
  s = (char*)emalloc(strlen(r) + 3);
  strcpy(s, "\"");
  strcat(s, r);
  strcat(s, "\"");
  return s;
}

const char *
ns_add(Entry *p, const char *ns)
{
  if (p->tag)
    return ns_addx(p->tag, ns);
  if (*p->sym->name == ':')
    return ns_convert(p->sym->name);
  return ns_addx(ns_convert(p->sym->name), ns);
}

const char *
ns_addx(const char *tag, const char *ns)
{
  const char *n;
  char *t;
  const char *s = tag;
  if (*s == ':')
    return s+1;
  if (!ns || *s == '-' || strchr(s, ':'))
    return s;
  n = ns_convert(ns);
  t = (char*)emalloc(strlen(n) + strlen(s) + 2);
  strcpy(t, n);
  strcat(t, ":");
  strcat(t, s);
  return t;
}

const char *
ns_name(const char *tag)
{
  const char *t, *r, *s = tag;
  if (*s)
  {
    for (r = s+strlen(s)-1; r > s; r--)
      if (*r != '_')
        break;
    for (t = s + 1; t < r; t++)
    {
      if (t[0] == '_' && t[1] == '_')
      {
        s = t + 2;
        t++;
      }
      else if (t[0] == ':' && t[1] != ':')
      {
        s = t + 1;
        t++;
      }
    }
  }
  return s;
}

const char *
ns_cname(const char *tag, const char *suffix)
{
  char *s;
  const char *t;
  size_t i, n;
  if (!tag)
    return NULL;
  t = ns_name(tag);
  n = strlen(t);
  if (suffix)
    s = (char*)emalloc(n + strlen(suffix) + 2);
  else
    s = (char*)emalloc(n + 2);
  for (i = 0; i < n; i++)
  {
    if (!isalnum(t[i]))
      s[i] = '_';
    else
      s[i] = t[i];
  }
  s[i] = '\0';
  if (suffix)
    strcat(s, suffix);
  if (is_keyword(t))
    strcat(s, "_");
  return s;
}

const char *
ns_fname(const char *tag)
{
  char *s;
  size_t i;
  s = (char*)emalloc(strlen(tag) + 1);
  strcpy(s, tag);
  for (i = 0; s[i]; i++)
    if (!isalnum(s[i]))
      s[i] = '_';
  return s;
}

const char *
ns_tag_remove(Entry *p)
{
  if (p->tag)
  {
    const char *s = strchr(p->tag, ':');
    if (s)
      return s + 1;
    return p->tag;
  }
  return ns_remove(p->sym->name);
}

const char *
ns_remove(const char *tag)
{
  return ns_convert(ns_name(tag));
}

const char *
ns_remove1(const char *tag)
{
  const char *t, *s = tag;
  int n = 2;
  if (*s)
  {
    for (t = s + 1; *t && n; t++)
    {
      if (t[0] == '_' && t[1] == '_')
      {
        s = t + 2;
        t++;
        n--;
      }
    }
    if (n || (s[0] == '_' && !is_special(s)) || !*s)
      s = tag;
  }
  return s;
}

const char *
ns_remove2(const char *tag, const char *type)
{
  return ns_convert(ns_remove3(tag, type));
}

const char *
ns_remove3(const char *tag, const char *type)
{
  size_t n;
  if (tag && type && !strncmp(tag, type, n = strlen(type)) && strlen(tag) > n + 2 && tag[n] == '_' && tag[n + 1] == '_')
    return tag + n + 2;
  return tag;
}

const char *
xsi_type_u(Tnode *typ)
{
  Service *sp;
  const char *s;
  if (tflag)
    return xsi_type(typ);
  s = xsi_type(typ);
  for (sp = services; sp; sp = sp->next)
    if (sp->xsi_type && has_ns_eq(sp->ns, s))
      return s;
  return "";
}

const char *
xsi_type(Tnode *typ)
{
  if (!typ)
    return "NULL";
  if (is_dynamic_array(typ) && !has_ns(typ) && !is_binary(typ))
    return xsi_type_Darray(typ);
  if (typ->type == Tarray)
    return xsi_type_Tarray(typ);
  if ((is_qname(typ) || is_stdqname(typ)))
    return "xsd:QName";
  if (is_untyped(typ))
    return "";
  if (typ->sym)
  {
    if (!strncmp(typ->sym->name, "SOAP_ENV__", 10))
      return "";
    if (is_XML(typ))
      return "xsd:anyType";
    if (typ->type != Ttemplate)
      return ns_convert(typ->sym->name);
  }
  if (is_string(typ) || is_wstring(typ) || is_stdstring(typ) || is_stdwstring(typ))
    return "xsd:string";
  switch(typ->type)
  {
    case Tvoid:
      return "xsd:anyType";
    case Tchar:
      return "xsd:byte";
    case Twchar:
      return "wchar";
    case Tshort:
      return "xsd:short";
    case Tint:
      return "xsd:int";
    case Tlong:
    case Tllong:
      return "xsd:long";
    case Tfloat:
      return "xsd:float";
    case Tdouble:
      return "xsd:double";
    case Tldouble:
      return "xsd:decimal";
    case Tuchar:
      return "xsd:unsignedByte";
    case Tushort:
      return "xsd:unsignedShort";
    case Tuint:
      return "xsd:unsignedInt";
    case Tulong:
    case Tullong:
      return "xsd:unsignedLong";
    case Ttime:
      return "xsd:dateTime";
    case Tpointer:
    case Treference:
    case Trvalueref:
      return xsi_type((Tnode*)typ->ref);
    case Tenum:
      if (is_bool(typ))
        return "xsd:boolean";
    case Tenumsc:
    case Tstruct:
    case Tclass:
      if (!strncmp(typ->id->name, "SOAP_ENV__", 10))
        return "";
      return ns_convert(typ->id->name);
    case Ttemplate:
      if ((Tnode*)typ->ref)
        return xsi_type((Tnode*)typ->ref);
      break;
    default:
      break;
  }
  return "";
}

const char *
xml_tag(Tnode *typ)
{
  if (!typ)
    return "NULL";
  if ((typ->type == Tpointer && !is_string(typ) && !is_wstring(typ)) || typ->type == Treference || typ->type == Trvalueref)
    return xml_tag((Tnode*)typ->ref);
  if (typ->sym)
    return ns_convert(typ->sym->name);
  return the_type(typ);
}

const char *
wsdl_type(Tnode *typ, const char *ns)
{
  if (!typ)
    return "NULL";
  if ((is_qname(typ) || is_stdqname(typ)) && ns)
    return "xsd:QName";
  if (typ->sym)
  {
    if (is_XML(typ))
      return "xsd:anyType";
    else if (ns)
      return ns_convert(typ->sym->name);
    else
      return ns_remove(typ->sym->name);
  }
  return base_type(typ, ns);
}

const char *
base_type(Tnode *typ, const char *ns)
{
  int d;
  const char *s;
  char *t;
  if (typ->restriction)
  {
    if (ns)
      return ns_convert(typ->restriction->name);
    else
      return ns_remove(typ->restriction->name);
  }
  if (is_string(typ) || is_wstring(typ) || is_stdstring(typ) || is_stdwstring(typ) || is_fixedstring(typ))
  {
    if (ns)
      return "xsd:string";
    return "string";
  }
  if (is_dynamic_array(typ) && !is_binary(typ) && !has_ns(typ) && !is_untyped(typ))
  {
    s = ns_remove(wsdl_type(((Table*)typ->ref)->list->info.typ, NULL));
    if (ns && *ns)
    {
      t = (char*)emalloc(strlen(s)+strlen(ns_convert(ns))+13);
      strcpy(t, ns_convert(ns));
      strcat(t, ":");
      strcat(t, "ArrayOf");
    }
    else
    {
      t = (char*)emalloc(strlen(s)+12);
      strcpy(t, "ArrayOf");
    }
    strcat(t, s);
    d = get_Darraydims(typ);
    if (d)
      sprintf(t+strlen(t), "%dD", d);
    return t;
  }
  switch (typ->type)
  {
    case Tvoid :
      if (ns)
        return "xsd:anyType";
      return "anyType";
    case Tchar :
      if (ns)
        return "xsd:byte";
      return "byte";
    case Tshort :
      if (ns)
        return "xsd:short";
      return "short";
    case Tint  :
      if (ns)
        return "xsd:int";
      return "int";
    case Tlong  :
    case Tllong  :
      if (ns)
        return "xsd:long";
      return "long";
    case Tfloat:
      if (ns)
        return "xsd:float";
      return "float";
    case Tdouble:
      if (ns)
        return "xsd:double";
      return "double";
    case Tldouble:
      if (ns)
        return "xsd:decimal";
      return "decimal";
    case Tuchar:
      if (ns)
        return "xsd:unsignedByte";
      return "unsignedByte";
    case Tushort:
      if (ns)
        return "xsd:unsignedShort";
      return "unsignedShort";
    case Tuint:
      if (ns)
        return "xsd:unsignedInt";
      return "unsignedInt";
    case Tulong:
    case Tullong:
      if (ns)
        return "xsd:unsignedLong";
      return "unsignedLong";
    case Ttime:
      if (ns)
        return "xsd:dateTime";
      return "dateTime";
    case Tpointer:
    case Treference:
    case Trvalueref:
      return wsdl_type((Tnode*)typ->ref, ns);
    case Tarray:
      if (is_fixedstring(typ))
      {
        if (typ->sym)
        {
          if (ns)
            return ns_convert(typ->sym->name);
          return ns_remove(typ->sym->name);
        }
        if (ns)
          return "xsd:string";
        return "string";
      }
      if (ns && *ns)
      {
        t = (char*)emalloc((strlen(ns_convert(ns))+strlen(c_ident(typ))+2) * sizeof(char));
        strcpy(t, ns_convert(ns));
        strcat(t, ":");
        strcat(t, c_ident(typ));
        return t;
      }
      else
        return c_ident(typ);
    case Tenum:
      if (is_bool(typ))
      {
        if (ns)
          return "xsd:boolean";
        return "boolean";
      }
    case Tenumsc:
    case Tstruct:
    case Tclass:
      if (!has_ns(typ) && ns && *ns)
      {
        t = (char*)emalloc((strlen(ns_convert(ns))+strlen(typ->id->name)+2) * sizeof(char));
        strcpy(t, ns_convert(ns));
        strcat(t, ":");
        strcat(t, ns_convert(typ->id->name));
        return t;
      }
      else if (ns)
        return ns_convert(typ->id->name);
      else
        return ns_remove(typ->id->name);
    case Tunion:
      if (ns)
        return "xsd:choice";
      return "choice";
    case Ttemplate:
      if ((Tnode*)typ->ref)
        return wsdl_type((Tnode*)typ->ref, ns);
      break;
    default:
      break;
  }
  return "";
}

const char *
the_type(Tnode *typ)
{
  if (!typ)
    return "NULL";
  if (typ->type == Tarray || (is_dynamic_array(typ) && !is_binary(typ) && (eflag || (!has_ns(typ) && !is_untyped(typ)))))
    return "SOAP-ENC:Array";
  if (is_string(typ) || is_wstring(typ) || is_stdstring(typ) || is_stdwstring(typ))
    return "string";
  switch (typ->type)
  {
    case Tchar:
      return "byte";
    case Twchar:
      return "wchar";
    case Tshort:
      return "short";
    case Tint :
      return "int";
    case Tlong :
    case Tllong :
      return "long";
    case Tfloat:
      return "float";
    case Tdouble:
      return "double";
    case Tldouble:
      return "decimal";
    case Tuchar:
      return "unsignedByte";
    case Tushort:
      return "unsignedShort";
    case Tuint:
      return "unsignedInt";
    case Tulong:
    case Tullong:
      return "unsignedLong";
    case Ttime:
      return "dateTime";
    case Tpointer:
    case Treference:
    case Trvalueref:
      return the_type((Tnode*)typ->ref);
    case Tarray:
      return "SOAP-ENC:Array";
    case Tenum:
      if (is_bool(typ))
        return "boolean";
    case Tenumsc:
    case Tstruct:
    case Tclass:
      return ns_convert(typ->id->name);
    default:
      break;
  }
  return "";
}

/* c_type returns the type to be used in parameter declaration*/
const char *
c_type(Tnode *typ)
{
  char *p = NULL;
  const char *q, *r;
  Entry *e;
  if (typ == NULL)
    return "NULL";
  switch (typ->type)
  {
    case Tnone:
      return "";
    case Tvoid:
      return "void";
    case Tchar:
      return "char";
    case Twchar:
      return "wchar_t";
    case Tshort:
      return "short";
    case Tint  :
      return "int";
    case Tlong  :
      return "long";
    case Tllong  :
      return "LONG64";
    case Tfloat:
      return "float";
    case Tdouble:
      return "double";
    case Tldouble:
      return "long double";
    case Tuchar:
      return "unsigned char";
    case Tushort:
      return "unsigned short";
    case Tuint:
      return "unsigned int";
    case Tulong:
      return "unsigned long";
    case Tullong:
      return "ULONG64";
    case Tsize:
      return "size_t";
    case Ttime:
      return "time_t";
    case Tstruct:
      q = ident(typ->id->name);
      p = (char*)emalloc((8+strlen(q)) * sizeof(char));
      strcpy(p, "struct ");
      strcat(p, q);
      return p;
    case Tclass:
      return ident(typ->id->name);
    case Tunion:
      q = ident(typ->id->name);
      p = (char*)emalloc((7+strlen(q)) * sizeof(char));
      strcpy(p, "union ");
      strcat(p, q);
      return p;
    case Tenum:
      if (is_bool(typ))
        return "bool";
      q = ident(typ->id->name);
      p = (char*)emalloc((6+strlen(q)) * sizeof(char));
      strcpy(p, "enum ");
      strcat(p, q);
      return p;
    case Tenumsc:
      return ident(typ->id->name);
    case Tpointer:
      return c_type_id((Tnode*)typ->ref, "*");
    case Treference:
      return c_type_id((Tnode*)typ->ref, "&");
    case Trvalueref:
      return c_type_id((Tnode*)typ->ref, "&&");
    case Tarray:
      return c_type_id((Tnode*)typ->ref, "*");
    case Tfun:
      p = (char*)emalloc(1024);
      strcpy(p, c_type(((FNinfo*)typ->ref)->ret));
      strcat(p, "(");
      if (((FNinfo*)typ->ref)->args)
      {
        for (e = ((FNinfo*)typ->ref)->args->list; e; e = e->next)
        {
          strcat(p, c_storage(e->info.sto));
          if (e->info.typ->type != Tvoid)
          {
            strcat(p, c_type_id(e->info.typ, e->sym->name));
            strcat(p, c_init(e));
          }
          else
            strcat(p, "void");
          if (e->next)
            strcat(p, ", ");
        }
      }
      strcat(p, ")");
      return p;
    case Ttemplate:
      if (typ->ref)
      {
        p = (char*)emalloc((strlen(q = c_type((Tnode*)typ->ref)) + strlen(r = ident(typ->id->name)) + 4) * sizeof(char));
        strcpy(p, r);
        strcat(p, "<");
        strcat(p, q);
        strcat(p, "> ");
        return p;
      }
    default:
      return "UnknownType";
  }
  return p;
}

const char *
c_storage(Storage sto)
{
  static char buf[256];
  buf[0] = '\0';
  if (sto & Smutable)
    strcat(buf, "mutable ");
  if (sto & Sinline)
    strcat(buf, "inline ");
  if (sto & Sfriend)
    strcat(buf, "friend ");
  if (sto & Svirtual)
    strcat(buf, "virtual ");
  if (sto & Stypedef)
    strcat(buf, "typedef ");
  if (sto & Sexplicit)
    strcat(buf, "explicit ");
  if (sto & Sstatic)
    strcat(buf, "static ");
  if (sto & Sregister)
    strcat(buf, "register ");
  if (sto & Sauto)
    strcat(buf, "auto ");
  if (sto & Sconstptr)
    strcat(buf, "const ");
  if (sto & Sconst)
    strcat(buf, "const ");
  return buf;
}

const char *
c_const(Storage sto)
{
  if (sto & Sconst)
    return "const ";
  if (sto & Sconstptr)
    return "const ";
  return "";
}

const char *
c_init(Entry *e)
{
  return c_init_a(e, " = ");
}

const char *
c_init_a(Entry *e, const char *a)
{
  static char buf[8196];
  buf[0] = '\0';
  if (e && (e->info.hasval || e->info.ptrval))
  {
    Tnode *p = e->info.typ;
    if (e->info.ptrval)
      p = p->ref;
    switch (p->type)
    {
      case Tchar:
      case Twchar:
      case Tuchar:
      case Tshort:
      case Tushort:
      case Tint:
      case Tuint:
      case Ttime:
      case Tsize:
        sprintf(buf, "%s" SOAP_LONG_FORMAT, a, e->info.val.i);
        break;
      case Tlong:
        sprintf(buf, "%s" SOAP_LONG_FORMAT "L", a, e->info.val.i);
        break;
      case Tulong:
        sprintf(buf, "%s" SOAP_LONG_FORMAT "UL", a, e->info.val.i);
        break;
      case Tllong:
        sprintf(buf, "%s" SOAP_LONG_FORMAT "LL", a, e->info.val.i);
        break;
      case Tullong:
        sprintf(buf, "%s" SOAP_LONG_FORMAT "ULL", a, e->info.val.i);
        break;
      case Tfloat:
      case Tdouble:
        sprintf(buf, "%s%g", a, e->info.val.r);
        break;
      case Tldouble:
        sprintf(buf, "%s%gL", a, e->info.val.r);
        break;
      case Tenum:
      case Tenumsc:
        if (e->info.val.i <= 0x7FFFLL && e->info.val.i >= -0x8000LL)
          sprintf(buf, "%s(%s)" SOAP_LONG_FORMAT, a, c_type(p), e->info.val.i);
        else
          sprintf(buf, "%s(%s)" SOAP_LONG_FORMAT "LL", a, c_type(p), e->info.val.i);
        break;
      default:
        if (is_stdstring(p) && e->info.val.s && strlen(e->info.val.s) < sizeof(buf)-6)
          sprintf(buf, "%s\"%s\"", a, cstring(e->info.val.s, 0));
        else if (is_stdwstring(p) && e->info.val.s && strlen(e->info.val.s) < sizeof(buf)-7)
          sprintf(buf, "%sL\"%s\"", a, cstring(e->info.val.s, 0));
        else if (is_wstring(p) && e->info.val.s && strlen(e->info.val.s) < sizeof(buf)-17)
          sprintf(buf, "%s(wchar_t*)L\"%s\"", a, cstring(e->info.val.s, 0));
        else if (e->info.val.s && strlen(e->info.val.s) < sizeof(buf)-13)
          sprintf(buf, "%s(char*)\"%s\"", a, cstring(e->info.val.s, 0));
        else if (p->type == Tpointer)
          sprintf(buf, "%sNULL", a);
        break;
    }
  }
  return buf;
}

/* c_type_constptr_id returns the const* type to be used in parameter declaration */
const char *
c_type_constptr_id(Tnode *typ, const char *name)
{
  return c_type_id(typ, name);
}

/* c_type_id returns the type to be used in parameter declaration */
const char *
c_type_id(Tnode *typ, const char *name)
{
  const char *id;
  char *p = NULL;
  const char *q;
  char tempBuf[10];
  Tnode *temp;
  Entry *e;
  if (!typ)
    return "NULL";
  id = ident(name);
  switch (typ->type)
  {
    case Tnone:
      return id;
    case Tvoid:
      p = (char*)emalloc(6+strlen(id));
      strcpy(p, "void ");
      strcat(p, id);
      break;
    case Tchar:
      p = (char*)emalloc(6+strlen(id));
      strcpy(p, "char ");
      strcat(p, id);
      break;
    case Twchar:
      p = (char*)emalloc(9+strlen(id));
      strcpy(p, "wchar_t ");
      strcat(p, id);
      break;
    case Tshort:
      p = (char*)emalloc(7+strlen(id));
      strcpy(p, "short ");
      strcat(p, id);
      break;
    case Tint  :
      p = (char*)emalloc(5+strlen(id));
      strcpy(p, "int ");
      strcat(p, id);
      break;
    case Tlong  :
      p = (char*)emalloc(6+strlen(id));
      strcpy(p, "long ");
      strcat(p, id);
      break;
    case Tllong  :
      p = (char*)emalloc(8+strlen(id));
      strcpy(p, "LONG64 ");
      strcat(p, id);
      break;
    case Tfloat:
      p = (char*)emalloc(7+strlen(id));
      strcpy(p, "float ");
      strcat(p, id);
      break;
    case Tdouble:
      p = (char*)emalloc(8+strlen(id));
      strcpy(p, "double ");
      strcat(p, id);
      break;
    case Tldouble:
      p = (char*)emalloc(13+strlen(id));
      strcpy(p, "long double ");
      strcat(p, id);
      break;
    case Tuchar:
      p = (char*)emalloc(15+strlen(id));
      strcpy(p, "unsigned char ");
      strcat(p, id);
      break;
    case Tushort:
      p = (char*)emalloc(16+strlen(id));
      strcpy(p, "unsigned short ");
      strcat(p, id);
      break;
    case Tuint:
      p = (char*)emalloc(14+strlen(id));
      strcpy(p, "unsigned int ");
      strcat(p, id);
      break;
    case Tulong:
      p = (char*)emalloc(15+strlen(id));
      strcpy(p, "unsigned long ");
      strcat(p, id);
      break;
    case Tullong:
      p = (char*)emalloc(9+strlen(id));
      strcpy(p, "ULONG64 ");
      strcat(p, id);
      break;
    case Tsize:
      p = (char*)emalloc(8+strlen(id));
      strcpy(p, "size_t ");
      strcat(p, id);
      break;
    case Ttime:
      p = (char*)emalloc(8+strlen(id));
      strcpy(p, "time_t ");
      strcat(p, id);
      break;
    case Tstruct:
      q = ident(typ->id->name);
      p = (char*)emalloc((9+strlen(q)+strlen(id)) * sizeof(char));
      strcpy(p, "struct ");
      strcat(p, q);
      strcat(p, " ");
      strcat(p, id);
      break;
    case Tclass:
      q = ident(typ->id->name);
      p = (char*)emalloc((2+strlen(q)+strlen(id)) * sizeof(char));
      strcpy(p, q);
      strcat(p, " ");
      strcat(p, id);
      break;
    case Tunion:
      q = ident(typ->id->name);
      p = (char*)emalloc((8+strlen(q)+strlen(id)) * sizeof(char));
      strcpy(p, "union ");
      strcat(p, q);
      strcat(p, " ");
      strcat(p, id);
      break;
    case Tenum:
      if (is_bool(typ))
      {
        p = (char*)emalloc((strlen(id)+6)* sizeof(char));
        strcpy(p, "bool ");
        strcat(p, id);
        return p;
      }
      q = ident(typ->id->name);
      p = (char*)emalloc((7+strlen(q)+strlen(id)) * sizeof(char));
      strcpy(p, "enum ");
      strcat(p, q);
      strcat(p, " ");
      strcat(p, id);
      break;
    case Tenumsc:
      q = ident(typ->id->name);
      p = (char*)emalloc((7+strlen(q)+strlen(id)) * sizeof(char));
      strcpy(p, q);
      strcat(p, " ");
      strcat(p, id);
      break;
    case Tpointer:
      p = (char*)emalloc(strlen(id)+2);
      p[0] = '*';
      strcpy(p+1, id);
      return c_type_id((Tnode*)typ->ref, p);
    case Treference:
      p = (char*)emalloc(strlen(id)+2);
      p[0] = '&';
      strcpy(p+1, id);
      return c_type_id((Tnode*)typ->ref, p);
    case Trvalueref:
      p = (char*)emalloc(strlen(id)+3);
      p[0] = '&';
      p[1] = '&';
      strcpy(p+2, id);
      return c_type_id((Tnode*)typ->ref, p);
    case Tarray:
      temp = typ;
      while (((Tnode*) (typ->ref))->type == Tarray)
        typ = (Tnode*)typ->ref;
      p = (char*)emalloc((12+strlen(q = c_type_id((Tnode*)typ->ref, id))) * sizeof(char));
      strcpy(p, q);
      typ = temp;
      while (typ->type == Tarray)
      {
        if (((Tnode*) typ->ref)->width)
        {
          sprintf(tempBuf, "[%d]", (typ->width / ((Tnode*) typ->ref)->width));
          strcat(p, tempBuf);
        }
        typ = (Tnode*)typ->ref;
      }
      break;
    case Tfun:
      if (id && strncmp(id, "operator ", 9))
        q = c_type_id(((FNinfo*)typ->ref)->ret, id);
      else
        q = id;
      p = (char*)emalloc(1024);
      if (q)
        strcpy(p, q);
      strcat(p, "(");
      for (e = ((FNinfo*)typ->ref)->args->list; e; e = e->next)
      {
        strcat(p, c_storage(e->info.sto));
        if (e->info.typ->type != Tvoid)
        {
          strcat(p, c_type_id(e->info.typ, e->sym->name));
          strcat(p, c_init(e));
        }
        else
          strcat(p, "void");
        if (e->next)
          strcat(p, ", ");
      }
      strcat(p, ")");
      break;
    case Ttemplate:
      if (typ->ref)
      {
        p = (char*)emalloc((strlen(q = c_type((Tnode*)typ->ref))+strlen(ident(typ->id->name))+strlen(id)+4) * sizeof(char));
        strcpy(p, ident(typ->id->name));
        strcat(p, "<");
        strcat(p, q);
        strcat(p, "> ");
        strcat(p, id);
        break;
      }
    default:
      return "UnknownType";
  }
  return p;
}

const char *
c_type_sym(Tnode *typ)
{
  if (typ->sym)
    return ident(typ->sym->name);
  return c_type(typ);
}

/* c_type_synonym_id returns the typedef synonym name as a type (if present) to be used in parameter declaration */
const char *
c_type_synonym_id(Tnode *typ, const char *name)
{
  if (typ->synonym)
  {
    const char *synonym = ident(typ->synonym->name);
    const char *id = ident(name);
    char *p = (char*)emalloc(strlen(synonym) + strlen(id) + 2);
    strcpy(p, synonym);
    strcat(p, " ");
    strcat(p, id);
    return p;
  }
  return c_type_id(typ, name);
}

const char *
xsi_type_Tarray(Tnode *typ)
{
  Tnode *t;
  int cardinality;
  char *p;
  const char *s;
  t = (Tnode*)typ->ref;
  if (is_fixedstring(typ))
  {
    if (typ->sym)
      return ns_convert(typ->sym->name);
    return "xsd:string";
  }
  cardinality = 1;
  while (t->type == Tarray || (is_dynamic_array(t) && !has_ns(t) && !is_untyped(t) && !is_binary(t)))
  {
    if (t->type == Tarray)
      t = (Tnode*)t->ref;
    else
      t = (Tnode*)((Table*)t->ref)->list->info.typ->ref;
    cardinality++;
  }
  s = xsi_type(t);
  if (!*s)
    s = wsdl_type(t, "");
  p = (char*)emalloc(strlen(s)+cardinality+3);
  strcpy(p, s);
  if (cardinality > 1)
  {
    strcat(p, "[");
    for (; cardinality > 2; cardinality--)
      strcat(p, ", ");
    strcat(p, "]");
  }
  return p;
}

const char *
xsi_type_Darray(Tnode *typ)
{
  Tnode *t;
  int cardinality;
  char *p;
  const char *s;
  Entry *q;
  if (!typ->ref)
    return "";
  q = ((Table*)typ->ref)->list;
  while (q && q->info.typ->type == Tfun)
    q = q->next;
  t = (Tnode*)q->info.typ->ref;
  cardinality = 1;
  while (t->type == Tarray || (is_dynamic_array(t) && !has_ns(t) && !is_untyped(t) && !is_binary(t)))
  {
    if (t->type == Tarray)
      t = (Tnode*)t->ref;
    else
    {
      q = ((Table*)t->ref)->list;
      while (q && q->info.typ->type == Tfun)
        q = q->next;
      t = (Tnode*)q->info.typ->ref;
    }
    cardinality++;
  }
  s = xsi_type(t);
  if (!*s)
    s = wsdl_type(t, "");
  p = (char*)emalloc(strlen(s)+cardinality*2+1);
  strcpy(p, s);
  if (cardinality > 1)
  {
    strcat(p, "[");
    for (; cardinality > 2; cardinality--)
      strcat(p, ", ");
    strcat(p, "]");
  }
  return p;
}

void
generate_type(Tnode *typ)
{
  if (is_transient(typ) || typ->type == Twchar || is_void(typ))
    return;

  if (lflag && typ->type == Tint && !typ->sym)
  {
    fprintf(fhead, "\n\n#ifndef %s_DEFINED", soap_type(typ));
    fprintf(fhead, "\n#define %s_DEFINED", soap_type(typ));
    fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_default_int(struct soap*, int*);");
    fprintf(fhead, "\nSOAP_FMAC1 int SOAP_FMAC2 soap_out_int(struct soap*, const char*, int, const int*, const char*);");
    fprintf(fhead, "\nSOAP_FMAC1 int* SOAP_FMAC2 soap_in_int(struct soap*, const char*, int*, const char*);");
    fprintf(fhead, "\n#endif");
    return; /* do not generate int serializers in libs */
  }
  else if (is_imported(typ) && (typ->type != Tint || typ->sym))
  {
    return;
  }
  if (is_typedef(typ) && (is_element(typ) || is_synonym(typ)))
    fprintf(fhead, "\n/* %s is a typedef synonym of %s */", c_ident(typ), t_ident(typ));
  else if (is_typedef(typ) && (is_element(typ) || is_restriction(typ)))
    fprintf(fhead, "\n/* %s is a typedef restriction of %s */", c_ident(typ), t_ident(typ));
  if (is_primitive(typ) || is_string(typ) || is_wstring(typ))
  {
    if (!Qflag && is_external(typ) && namespaceid)
    {
      const char *id = namespaceid;
      namespaceid = NULL;
      fprintf(fhead, "\n\n#ifndef %s_DEFINED", soap_type(typ));
      fprintf(fhead, "\n#define %s_DEFINED", soap_type(typ));
      namespaceid = id;
      fprintf(fhead, "\n\n}");
      fprintf(fout, "\n\n}");
    }
    else
    {
      fprintf(fhead, "\n\n#ifndef %s_DEFINED", soap_type(typ));
      fprintf(fhead, "\n#define %s_DEFINED", soap_type(typ));
    }
    fflush(fhead);
    soap_default(typ);
    soap_serialize(typ);
    if (Etflag)
      soap_traverse(typ);
    soap_out(typ);
    soap_in(typ);
    if (!Qflag && is_external(typ) && namespaceid)
    {
      fprintf(fhead, "\n\nnamespace %s {", namespaceid);
      fprintf(fout, "\n\nnamespace %s {", namespaceid);
    }
    if (is_string(typ) || is_wstring(typ))
    {
      soap_dup(typ);
      soap_del(typ);
    }
    soap_instantiate(typ);
    soap_put(typ);
    soap_get(typ);
    fprintf(fhead, "\n#endif");
    return;
  }
  switch(typ->type)
  {
    case Ttemplate:
    case Tpointer:
    case Tarray:
    case Tstruct:
    case Tclass:
    case Tunion:
      if (is_header_or_fault(typ) || is_body(typ))
      {
        fprintf(fhead, "\n\n#ifndef WITH_NOGLOBAL");
        fprintf(fout, "\n\n#ifndef WITH_NOGLOBAL");
      }
      if (!Qflag && is_external(typ) && namespaceid)
      {
        const char *id = namespaceid;
        namespaceid = NULL;
        fprintf(fhead, "\n\n#ifndef %s_DEFINED", soap_type(typ));
        fprintf(fhead, "\n#define %s_DEFINED", soap_type(typ));
        namespaceid = id;
        fprintf(fhead, "\n\n}");
        fprintf(fout, "\n\n}");
      }
      else
      {
        fprintf(fhead, "\n\n#ifndef %s_DEFINED", soap_type(typ));
        fprintf(fhead, "\n#define %s_DEFINED", soap_type(typ));
      }
      fflush(fhead);
      soap_default(typ);
      soap_serialize(typ);
      if (Etflag)
        soap_traverse(typ);
      soap_out(typ);
      soap_in(typ);
      if (!Qflag && is_external(typ) && namespaceid)
      {
        fprintf(fhead, "\n\nnamespace %s {", namespaceid);
        fprintf(fout, "\n\nnamespace %s {", namespaceid);
      }
      soap_dup(typ);
      soap_del(typ);
      soap_instantiate(typ);
      soap_put(typ);
      soap_get(typ);
      fprintf(fhead, "\n#endif");
      if (is_header_or_fault(typ) || is_body(typ))
      {
        fprintf(fhead, "\n\n#endif");
        fprintf(fout, "\n\n#endif");
      }
      break;
    default:
      break;
  }
}

void
matlab_gen_sparseStruct(void)
{
  fprintf(fmheader, "\nstruct soapSparseArray{\n");
  fprintf(fmheader, "  int *ir;\n");
  fprintf(fmheader, "  int *jc;\n");
  fprintf(fmheader, "  double *pr;\n");
  fprintf(fmheader, "  int num_columns;\n");
  fprintf(fmheader, "  int num_rows;\n");
  fprintf(fmheader, "  int nzmax;\n");
  fprintf(fmheader, "};\n");
}

void
matlab_c_to_mx_sparse(void)
{
  fprintf(fmheader, "\nmxArray* c_to_mx_soapSparseArray(struct soapSparseArray);\n");
  fprintf(fmatlab, "\nmxArray* c_to_mx_soapSparseArray(struct soapSparseArray a)\n");
  fprintf(fmatlab, "{\n");
  fprintf(fmatlab, "  mxArray *b;\n");
  fprintf(fmatlab, "  b = mxCreateSparse(a.num_rows, a.num_columns, a.nzmax, mxREAL);\n");
  fprintf(fmatlab, "  mxSetIr(b,a.ir);\n");
  fprintf(fmatlab, "  mxSetJc(b,a.jc);\n");
  fprintf(fmatlab, "  mxSetPr(b,a.pr);\n");
  fprintf(fmatlab, "  return b;\n");
  fprintf(fmatlab, "}\n");
}

void
matlab_mx_to_c_sparse(void)
{
  fprintf(fmheader, "\nmxArray* mx_to_c_soapSparseArray(const mxArray *, struct soapSparseArray *);\n");
  fprintf(fmatlab, "\nmxArray* mx_to_c_soapSparseArray(const mxArray *a, struct soapSparseArray *b)\n");
  fprintf(fmatlab, "{\n");
  fprintf(fmatlab, "  if (!mxIsSparse(a))\n");
  fprintf(fmatlab, "    {\n");
  fprintf(fmatlab, "      mexErrMsgTxt(\"Input should be a sparse array.\");\n");
  fprintf(fmatlab, "    }\n");

  fprintf(fmatlab, "  /* Get the starting positions of the data in the sparse array. */  \n");
  fprintf(fmatlab, "  b->pr = mxGetPr(a);\n");
  fprintf(fmatlab, "  b->ir = mxGetIr(a);\n");
  fprintf(fmatlab, "  b->jc = mxGetJc(a);\n");
  fprintf(fmatlab, "  b->num_columns = mxGetN(a);\n");
  fprintf(fmatlab, "  b->num_rows = mxGetM(a);\n");
  fprintf(fmatlab, "  b->nzmax = mxGetNzmax(a);\n");
  fprintf(fmatlab, "}\n");
}

void
matlab_mx_to_c_dynamicArray(Tnode* typ)
{
  int d;
  Entry *p;

  p = is_dynamic_array(typ);

  fprintf(fmatlab, "{\n");
  fprintf(fmatlab, "\tint i, numdims;\n");
  fprintf(fmatlab, "\tconst int *dims;\n");
  fprintf(fmatlab, "\tdouble *temp;\n");
  fprintf(fmatlab, "\tint size = 1;\n");
  fprintf(fmatlab, "\tint ret;\n");
  fprintf(fmatlab, "\tnumdims = mxGetNumberOfDimensions(a);\n");
  fprintf(fmatlab, "\tdims = mxGetDimensions(a);\n");

  d = get_Darraydims(typ);
  fprintf(fmatlab, "\tif (numdims != %d)\n", d);
  fprintf(fmatlab, "\t\tmexErrMsgTxt(\"Incompatible array specifications in C and mx.\");\n");

  /*
  fprintf(fmatlab, "\tfor (i=0;i<numdims; i++) {\n");
  fprintf(fmatlab, "\t  b->__size[i] = dims[i];\n");
  fprintf(fmatlab, "\t}\n");
  */

  if ((((Tnode *)p->info.typ->ref)->type != Tchar) && (((Tnode *)p->info.typ->ref)->type != Tuchar))
  {
    fprintf(fmatlab, "\ttemp = (double*)mxGetPr(a);\n");
    fprintf(fmatlab, "\tif (!temp)\n\t\tmexErrMsgTxt(\"mx_to_c_ArrayOfdouble: Pointer to data is NULL\");\n");
  }

  fprintf(fmatlab, "\tfor (i = 0; i < numdims; i++) {\n");
  fprintf(fmatlab, "\t\tif (b->__size[i] < dims[i])\n");
  fprintf(fmatlab, "\t\t\tmexErrMsgTxt(\"Incompatible array dimensions in C and mx.\");\n");
  fprintf(fmatlab, "\t\tsize *= dims[i];\n");
  fprintf(fmatlab, "\t}\n");

  if ((((Tnode *)p->info.typ->ref)->type != Tchar) && (((Tnode *)p->info.typ->ref)->type != Tuchar))
  {

    fprintf(fmatlab, "\tfor (i = 0; i < size; i++)\n");
    fprintf(fmatlab, "\t\tb->__ptr[i] = (%s)*temp++;\n", c_type((Tnode*)p->info.typ->ref));
  }
  else
  {
    fprintf(fmatlab, "\tret = mxGetString(a, b->__ptr, size + 1);\n");
    fprintf(fmatlab, "\tmexPrintf(\"ret = %%d, b->__ptr = %%s, size = %%d\", ret, b->__ptr, size);\n");
  }
  fprintf(fmatlab, "\n}\n");

  fflush(fmatlab);
}


void
matlab_c_to_mx_dynamicArray(Tnode* typ)
{
  int d, i;
  Entry *p;

  p = is_dynamic_array(typ);

  fprintf(fmatlab, "{\n");
  fprintf(fmatlab, "\tmxArray *out;\n");
  fprintf(fmatlab, "\t%s;\n", c_type_id((Tnode*)p->info.typ->ref, "*temp"));
  d = get_Darraydims(typ);
  fprintf(fmatlab, "\tint i;\n");

  fprintf(fmatlab, "\tint ndim = %d, dims[%d] = {", d, d);
  for (i = 0; i < d; i++)
  {
    if (i == 0)
      fprintf(fmatlab, "a.__size[%d]", i);
    else
      fprintf(fmatlab, ", a.__size[%d]", i);
  }
  fprintf(fmatlab, "};\n");

  fprintf(fmatlab, "\tint size = ");
  for (i = 0; i < d; i++)
  {
    if (i == 0)
      fprintf(fmatlab, "dims[%d]", i);
    else
      fprintf(fmatlab, "*dims[%d]", i);
  }
  fprintf(fmatlab, ";\n");
  if ((((Tnode *)p->info.typ->ref)->type != Tchar) && (((Tnode *)p->info.typ->ref)->type != Tuchar))
  {
    fprintf(fmatlab, "\tout = mxCreateNumericArray(ndim, dims, %s, mxREAL);\n", get_mxClassID((Tnode*)p->info.typ->ref));
    fprintf(fmatlab, "\tif (!out)\n\t\tmexErrMsgTxt(\"Could not create mxArray.\");\n");
    fprintf(fmatlab, "\ttemp = (%s) mxGetPr(out);\n", c_type_id((Tnode*)p->info.typ->ref, "*"));
    fprintf(fmatlab, "\tif (!temp)\n\t\tmexErrMsgTxt(\"matlab_array_c_to_mx: Pointer to data is NULL\");\n");

    fprintf(fmatlab, "\tfor (i = 0; i < size; i++)\n");
    fprintf(fmatlab, "\t\t*temp++ = a.__ptr[i];\n");
  }
  else
  {
    fprintf(fmatlab, "\tout = mxCreateString(a.__ptr);\n");
    fprintf(fmatlab, "\tif (!out)\n\t\tmexErrMsgTxt(\"Could not create mxArray.\");\n");
  }
  fprintf(fmatlab, "\treturn out;\n}\n");
  fflush(fmatlab);
}

const char*
get_mxClassID(Tnode* typ)
{
  switch(typ->type)
  {
    case Tdouble:
      return "mxDOUBLE_CLASS";
    case Tfloat:
      return "mxSINGLE_CLASS";
    case Tshort:
      return "mxINT16_CLASS";
    case Tushort:
      return "mxUINT16_CLASS";
    case Tint:
      return "mxINT32_CLASS";
    case Tuint:
      return "mxUINT32_CLASS";
    case Tlong:
      return "mxINT32_CLASS";
    case Tulong:
      return "mxUINT32_CLASS";
    case Tllong:
      return "mxINT64_CLASS";
    case Tullong:
      return "mxUINT64_CLASS";
    case Tchar:
      return "mxCHAR_CLASS";
    case Tuchar:
      return "mxCHAR_CLASS";
    default:
      return "";
  }
}

/*Function not in use.*/
void
matlab_array_c_to_mx(Tnode* typ)
{
  Tnode* temp;
  int d, i;

  fprintf(fmatlab, "{\n\tint rows, r, cols, c;\n");
  fprintf(fmatlab, "\tmxArray* out;\n");
  fprintf(fmatlab, "\tdouble* temp;\n");
  d = get_dimension(typ);
  fprintf(fmatlab, "\tint ndim = %d, dims[%d] = {", d, d);
  temp=typ;
  for (i = 0; i < d; i++)
  {
    if (i == 0)
      fprintf(fmatlab, "%d", temp->width / ((Tnode*) temp->ref)->width);
    else
      fprintf(fmatlab, ", %d", temp->width / ((Tnode*) temp->ref)->width);
    temp=(Tnode*)typ->ref;
  }
  fprintf(fmatlab, "};\n");

  fprintf(fmatlab, "\tout = mxCreateNumericArray(ndim, dims, mxDOUBLE_CLASS, mxREAL);\n");
  fprintf(fmatlab, "\ttemp = (double *) mxGetPr(out);\n");
  fprintf(fmatlab, "\tif (!out)\n\t\tmexErrMsgTxt(\"Could not create mxArray.\");\n");
  fprintf(fmatlab, "\tif (!temp)\n\t\tmexErrMsgTxt(\"matlab_array_c_to_mx: Pointer to data is NULL\");\n");
  fprintf(fmatlab, "\trows = mxGetM(out);\n");
  fprintf(fmatlab, "\tif (!rows)\n\t\tmexErrMsgTxt(\"matlab_array_c_to_mx: Data has zero rows\");\n");
  fprintf(fmatlab, "\tcols = mxGetN(out);\n");
  fprintf(fmatlab, "\tif (!cols)\n\t\tmexErrMsgTxt(\"matlab_array_c_to_mx: Data has zero columns\");\n");
  fprintf(fmatlab, "\tfor (c = 0; c < cols; c++)\n");
  fprintf(fmatlab, "\t\tfor (r = 0; r < rows; r++)\n");
  fprintf(fmatlab, "\t\t\t*temp++ = z->a[r][c];\n");
  fprintf(fmatlab, "\treturn out;\n}\n");
  fflush(fmatlab);
}


void matlab_c_to_mx_pointer(Tnode* typ)
{
  if (!typ->ref)
    return;

  fprintf(fmheader, "\nmxArray* c_to_mx_%s(%s);\n", c_ident(typ), c_type_id(typ, ""));
  fprintf(fmatlab, "\nmxArray* c_to_mx_%s(%s)\n", c_ident(typ), c_type_id(typ, "a"));
  fprintf(fmatlab, "{\n");
  fprintf(fmatlab, "\tmxArray  *fout;\n");
  fprintf(fmatlab, "\tfout = c_to_mx_%s(*a);\n", c_ident((Tnode*)typ->ref));
  fprintf(fmatlab, "\treturn fout;\n");
  fprintf(fmatlab, "}\n");
}

void matlab_mx_to_c_pointer(Tnode* typ)
{
  if (!typ->ref)
    return;
  fprintf(fmheader, "\nvoid mx_to_c_%s(const mxArray*, %s);\n", c_ident(typ), c_type_id(typ, "*"));
  fprintf(fmatlab, "\nvoid mx_to_c_%s(const mxArray* a, %s)\n", c_ident(typ), c_type_id(typ, "*b"));
  fprintf(fmatlab, "{\n\tmx_to_c_%s(a, *b);\n", c_ident((Tnode*)typ->ref));
  fprintf(fmatlab, "\n}\n");
}

void func2(Tnode* typ)
{
  Table *table, *t;
  Entry *p;

  fprintf(fmatlab, "\tif (!mxIsStruct(a))\n\t\tmexErrMsgTxt(\"Input must be a structure.\");\n");

  table=(Table*)typ->ref;
  for (t = table; t != (Table*)0; t = t->prev)
  {
    for (p = t->list; p != (Entry*)0; p = p->next)
    {
      if (p->info.typ->type != Tfun && !is_void(p->info.typ) && !is_XML(p->info.typ))
      {
        fprintf(fmatlab, "\t{mxArray *tmp = mxGetField(a, 0, \"%s\");\n", ident(p->sym->name));
        fprintf(fmatlab, "\tif (!tmp) {\n");
        fprintf(fmatlab, "\t\tmexErrMsgTxt(\"Above member field is empty!\");\n\t}\n");
        fprintf(fmatlab, "\tmx_to_c_%s(tmp, &b->%s);}\n", c_ident(p->info.typ), ident(p->sym->name));
      }
    }
  }
}

void
matlab_mx_to_c_struct(Tnode* typ)
{
  if (!typ->ref)
    return;

  if (is_dynamic_array(typ))
  {
    fprintf(fmheader, "\nvoid mx_to_c_%s(const mxArray*, %s);\n", c_ident(typ), c_type_id(typ, "*"));
    fprintf(fmatlab, "\nvoid mx_to_c_%s(const mxArray* a, %s)\n", c_ident(typ), c_type_id(typ, "*b"));
    matlab_mx_to_c_dynamicArray(typ);
    return;
  }
  else if (strstr(c_type_id(typ, ""), "soapSparseArray"))
  {
    return;
  }

  fprintf(fmheader, "\nvoid mx_to_c_%s(const mxArray*, %s);\n", c_ident(typ), c_type_id(typ, "*"));
  fprintf(fmatlab, "\nvoid mx_to_c_%s(const mxArray* a, %s)\n", c_ident(typ), c_type_id(typ, "*b"));
  fprintf(fmatlab, "{\n");

  func2(typ);
  fprintf(fmatlab, "\n}\n");

  return;
}

void
matlab_c_to_mx_struct(Tnode* typ)
{
  Table *table, *t;
  Entry *p;
  int number_of_fields=0;

  if (!typ->ref)
    return;

  if (is_dynamic_array(typ))
  {
    fprintf(fmheader, "\nmxArray* c_to_mx_%s(%s);\n", c_ident(typ), c_type_id(typ, ""));
    fprintf(fmatlab, "\nmxArray* c_to_mx_%s(%s)\n", c_ident(typ), c_type_id(typ, "a"));
    matlab_c_to_mx_dynamicArray(typ);
    return;
  }
  else if (strstr(c_type_id(typ, ""), "soapSparseArray"))
    return;

  fprintf(fmheader, "\nmxArray* c_to_mx_%s(%s);\n", c_ident(typ), c_type_id(typ, ""));
  fprintf(fmatlab, "\nmxArray* c_to_mx_%s(%s)\n", c_ident(typ), c_type_id(typ, "a"));
  table=(Table*)typ->ref;
  fprintf(fmatlab, "{\n\tconst char* fnames[] = {");
  for (t = table; t != (Table*)0; t = t->prev)
  {
    for (p = t->list; p != (Entry*)0; p = p->next)
    {
      if (p->info.typ->type != Tfun && !is_void(p->info.typ) && !is_XML(p->info.typ))
      {
        if (number_of_fields)
          fprintf(fmatlab, ", \"%s\"", ident(p->sym->name));
        else
          fprintf(fmatlab, "\"%s\"", ident(p->sym->name));
        number_of_fields++;
      }
    }
  }
  fprintf(fmatlab, "}; /* pointers to member field names */\n");

  fprintf(fmatlab, "\tint rows = 1, cols = 1;\n\tint index = 0;\n\tint number_of_fields = %d;\n\tmxArray *struct_array_ptr;\n", number_of_fields);
  fprintf(fmatlab, "\t/* Create a 1x1 struct matrix for output  */\n");
  fprintf(fmatlab, "\tstruct_array_ptr = mxCreateStructMatrix(rows, cols, number_of_fields, fnames);\n\tmexPrintf(\"6\");\n\tif (struct_array_ptr == NULL) {\n\t\tmexPrintf(\"COULDNT CREATE A MATRIX\");}\n\tmexPrintf(\"7\");\n");


  for (t = table; t != (Table*)0; t = t->prev)
  {
    for (p = t->list; p != (Entry*)0; p = p->next)
    {
      if (p->info.typ->type != Tfun && !is_void(p->info.typ) && !is_XML(p->info.typ))
      {
        fprintf(fmatlab, "\t{mxArray *fout = c_to_mx_%s(a.%s);\n", c_ident(p->info.typ), ident(p->sym->name));
        fprintf(fmatlab, "\tmxSetField(struct_array_ptr, index, \"%s\" , fout);}\n", ident(p->sym->name));
      }
    }
  }
  fprintf(fmatlab, "\treturn struct_array_ptr;\n}\n");
}

void
matlab_c_to_mx_primitive(Tnode *typ)
{
  fprintf(fmheader, "\nmxArray* c_to_mx_%s(%s);", c_ident(typ), c_type_id(typ, ""));
  fprintf(fmatlab, "\nmxArray* c_to_mx_%s(%s)\n", c_ident(typ), c_type_id(typ, "a"));

  fprintf(fmatlab, "{\n\tmxArray  *fout;\n");
  if ((typ->type == Tchar) || (typ->type == Tuchar))
  {
    fprintf(fmatlab, "\tchar buf[2];\n");
    fprintf(fmatlab, "\tbuf[0] = a;\n");
    fprintf(fmatlab, "\tbuf[1] = \'\\0\';\n");
    fprintf(fmatlab, "\tfout = mxCreateString(buf);\n");
    fprintf(fmatlab, "\tif (!fout)\n");
    fprintf(fmatlab, "\t\tmexErrMsgTxt(\"Could not create mxArray.\");\n");
  }
  else
  {
    fprintf(fmatlab, "\tint ndim = 1, dims[1] = {1};\n");
    fprintf(fmatlab, "\tfout = mxCreateNumericArray(ndim, dims, %s, mxREAL);\n", get_mxClassID(typ));
    fprintf(fmatlab, "\t%s = (%s)mxGetPr(fout);\n", c_type_id(typ, "*temp"), c_type_id(typ, "*"));
    fprintf(fmatlab, "\tif (!fout)\n");
    fprintf(fmatlab, "\t\tmexErrMsgTxt(\"Could not create mxArray.\");\n");
    fprintf(fmatlab, "\tif (!temp) \n");
    fprintf(fmatlab, "\t\tmexErrMsgTxt(\"matlab_array_c_to_mx: Pointer to data is NULL\");\n");
    fprintf(fmatlab, "\t*temp++= a;\n");
  }
  fprintf(fmatlab, "\treturn fout;\n}\n");
}

void
matlab_mx_to_c_primitive(Tnode *typ)
{
  fprintf(fmheader, "\nvoid mx_to_c_%s(const mxArray *, %s);\n", c_ident(typ), c_type_id(typ, "*"));
  fprintf(fmatlab, "\nvoid mx_to_c_%s(const mxArray *a, %s)\n", c_ident(typ), c_type_id(typ, "*b"));
  if ((typ->type == Tchar) || (typ->type == Tuchar))
  {
    fprintf(fmatlab, "{\n\tint ret;\n");
    fprintf(fmatlab, "\tchar buf[2];\n");
    fprintf(fmatlab, "\tret = mxGetString(a, buf, 2);\n");
    fprintf(fmatlab, "\tmexPrintf(\"ret = %%d, buf = %%s\", ret, buf);\n");
    fprintf(fmatlab, "\t*b = buf[0];\n");
  }
  else
  {
    fprintf(fmatlab, "{\n\tdouble* data = (double*)mxGetData(a);\n");
    fprintf(fmatlab, "\t*b = (%s)*data;\n", c_type(typ));
  }
  fprintf(fmatlab, "\n}\n");
}

void
matlab_out_generate(Tnode *typ)
{
  if (is_transient(typ) || typ->type == Twchar || is_XML(typ))
    return;

  /*
     typeNO++;
     if (typeNO>=1024)
     execerror("Too many user-defined data types");
   */

  if (is_primitive(typ))
  {
    matlab_c_to_mx_primitive(typ);
    matlab_mx_to_c_primitive(typ);
    return;
  }

  switch(typ->type)
  {
    case Tstruct:
      matlab_c_to_mx_struct(typ);
      matlab_mx_to_c_struct(typ);
      break;
    case Tpointer:
      matlab_c_to_mx_pointer(typ);
      matlab_mx_to_c_pointer(typ);
      break;
    default:
      break;
  }
}

/*his function is called first it first generates all routines
  and then in the second pass calls all routines to generate
  matlab_out for the table*/

void
func1(Table *table, Entry *param)
{
  Entry *q, *pout, *response=NULL;
  q = entry(table, param->sym);
  if (q)
  {
    pout = (Entry*)q->info.typ->ref;
  }
  else
  {
    fprintf(stderr, "Internal error: no table entry\n");
    return;
  }
  q = entry(classtable, param->sym);
  if (!is_response(pout->info.typ))
  {
    response = get_response(param->info.typ);
  }
  fprintf(fmheader, "\n\toutside loop struct %s soap_tmp_%s;", param->sym->name, param->sym->name);
  if (!is_response(pout->info.typ) && response)
  {
    fprintf(fmheader, "\n\tif..inside loop struct %s *soap_tmp_%s;", c_ident(response->info.typ), c_ident(response->info.typ));
  }
  fflush(fmheader);
}

void
matlab_def_table(Table *table)
{
  Entry *q, *pout, *e, *response=NULL;
  int i;
  Tnode *p;

  /*  for (q1 = table->list; q1 != (Entry*) 0; q1 = q1->next)
      if (q1->info.typ->type == Tfun)
      func1(table, q1);
   */

  /* Sparse matrix code will be present by default */
  matlab_gen_sparseStruct();
  matlab_c_to_mx_sparse();
  matlab_mx_to_c_sparse();

  for (i = 0; i < TYPES; i++)
  {
    for (p = Tptr[i]; p != (Tnode*)0 ; p = p->next)
    {
      /* This is generated for everything declared in the ".h" file. To make
         sure that it doesnt get generated for functions do a comparison with
         p->sym->name, so that its not generated for functions.
       */
      if (is_XML(p))
        continue;
      if (strstr(c_ident(p), "SOAP_ENV_") != NULL)
        continue;
      for (q = table->list; q != (Entry*) 0; q = q->next)
      {
        if (strcmp(c_ident(p), q->sym->name) == 0)
          break;
        e=entry(table, q->sym);
        if (e)
          pout = (Entry*)e->info.typ->ref;
        else
        {
          fprintf(stderr, "Internal error: no table entry\n");
          return;
        }
        if (!is_response(pout->info.typ))
        {
          response = get_response(q->info.typ);
        }
        if (!is_response(pout->info.typ) && response)
        {
          if (strcmp(c_ident(p), c_ident(response->info.typ)) == 0)
            break;
        }
      }
      if (q == (Entry*) 0)
        matlab_out_generate(p);
    }
  }
}

void
detect_cycles(void)
{
  Tnode *p;
  for (p = Tptr[Tclass]; p; p = p->next)
    detect_recursive_type(p);
  for (p = Tptr[Tstruct]; p; p = p->next)
    detect_recursive_type(p);
}

void
detect_recursive_type(Tnode *p)
{
  if (is_transient(p))
    return;
  if (p->type == Tclass || p->type == Tstruct || p->type == Tunion)
  {
    if (p->visited == Unexplored)
    {
      Table *t;
      Entry *e, *b = NULL;
      Tnode *q;
      p->visited = Hot;
      if ((p->type == Tclass || p->type == Tstruct) && p->baseid)
      {
        q = p;
        while (q->baseid)
        {
          b = entry(classtable, q->baseid);
          if (!b)
            break;
          q = b->info.typ;
          if (q->visited == Unexplored)
            detect_recursive_type(q);
          if (q->recursive)
          {
            p->recursive = True;
            break;
          }
          q->visited = Hot;
        }
      }
      if (!p->recursive)
        for (t = (Table*)p->ref; t; t = t->prev)
          for (e = t->list; e; e = e->next)
            detect_recursive_type(e->info.typ);
      if ((p->type == Tclass || p->type == Tstruct) && p->baseid)
      {
        q = p;
        while (q->baseid)
        {
          b = entry(classtable, q->baseid);
          if (!b)
            break;
          q = b->info.typ;
          if (q->recursive)
            p->recursive = True;
          q->visited = Cold;
        }
      }
      p->visited = Cold;
    }
    else if (p->visited == Hot)
    {
      p->recursive = True;
    }
  }
  else if (p->type == Tpointer || p->type == Treference || p->type == Trvalueref || p->type == Tarray || p->type == Ttemplate)
  {
    detect_recursive_type(p->ref);
  }
}

void
generate_defs(void)
{
  int i;
  Tnode *p;
  for (i = 0; i < TYPES; i++)
  {
    for (p = Tptr[i]; p; p = p->next)
    {
      if (!p->generated && !is_transient(p) && p->type != Twchar && !is_void(p))
      {
        p->generated = True;
        generate_type(p);
        if (fflag)
          if (--partnum == 0)
            return;
      }
    }
  }
}

int
no_of_var(Tnode *typ)
{
  Entry *p;
  Table *t;
  int i=0;
  if (typ->type == Tstruct || typ->type == Tclass)
  {
    t = (Table*)typ->ref;
    for (p = t->list; p != (Entry*) 0; p = p->next)
    {
      if (p->info.typ->type == Tpointer)
        i++;
    }
  }
  if ((((Tnode *)(typ->ref))->type == Tstruct) ||
      (((Tnode *)(typ->ref))->type == Tclass) )
  {
    t = (Table*)((Tnode*)(typ->ref))->ref;
    for (p = t->list; p != (Entry*) 0; p = p->next)
    {
      if (p->info.typ->type == Tpointer)
        i++;
    }
  }
  return i;
}

void
in_defs(void)
{
  int i;
  Tnode *p;
  for (i = 0; i < TYPES; i++)
  {
    for (p = Tptr[i]; p; p = p->next)
    {
      if (!is_element(p) && !is_transient(p) && p->type != Twchar && p->type != Tfun && p->type != Treference && p->type != Trvalueref && p->type != Tunion && !is_XML(p) && !is_header_or_fault(p) && !is_body(p) && !is_template(p))
      {
        const char *s = xsi_type(p);
        if (!*s)
          s = wsdl_type(p, "");
        if (*s == '-')
          continue;
        if (is_string(p))
          fprintf(fout, "\n\tcase %s:\n\t{\tchar **s;\n\t\ts = soap_in_%s(soap, tag, NULL, \"%s\");\n\t\treturn s ? *s : NULL;\n\t}", soap_type(p), c_ident(p), s);
        else if (is_wstring(p))
          fprintf(fout, "\n\tcase %s:\n\t{\twchar_t **s;\n\t\ts = soap_in_%s(soap, tag, NULL, \"%s\");\n\t\treturn s ? *s : NULL;\n\t}", soap_type(p), c_ident(p), s);
        else
          fprintf(fout, "\n\tcase %s:\n\t\treturn soap_in_%s(soap, tag, NULL, \"%s\");", soap_type(p), c_ident(p), s);
      }
    }
  }
}

void
in_defs2(void)
{
  int i, j;
  Tnode *p;
  const char *s;
  for (i = 0; i < TYPES; i++)
  {
    /* make sure (wrapper) classes are checked first */
    if (i == 0)
      j = Tclass;
    else if (i == Tclass)
      continue;
    else
      j = i;
    for (p = Tptr[j]; p; p = p->next)
    {
      if (!is_element(p) && !is_transient(p) && !is_template(p) && p->type != Twchar && p->type != Tfun && p->type != Treference && p->type != Trvalueref && p->type != Tunion && !is_XML(p) && !is_header_or_fault(p) && !is_body(p))
      {
        s = xsi_type(p);
        if (!*s)
          s = wsdl_type(p, "");
        if (*s == '-')
          continue;
        if (*s)
        {
          if (is_dynamic_array(p) && !is_binary(p) && !has_ns(p) && !is_untyped(p))
            fprintf(fout, "\n\t\tif (*soap->arrayType && !soap_match_array(soap, \"%s\"))\n\t\t{\t*type = %s;\n\t\t\treturn soap_in_%s(soap, tag, NULL, NULL);\n\t\t}", s, soap_type(p), c_ident(p));
          else if (is_string(p))
            fprintf(fout, "\n\t\tif (!soap_match_tag(soap, t, \"%s\"))\n\t\t{\tchar **s;\n\t\t\t*type = %s;\n\t\t\ts = soap_in_%s(soap, tag, NULL, NULL);\n\t\t\treturn s ? *s : NULL;\n\t\t}", s, soap_type(p), c_ident(p));
          else if (is_wstring(p))
            fprintf(fout, "\n\t\tif (!soap_match_tag(soap, t, \"%s\"))\n\t\t{\twchar_t **s;\n\t\t\t*type = %s;\n\t\t\ts = soap_in_%s(soap, tag, NULL, NULL);\n\t\t\treturn s ? *s : NULL;\n\t\t}", s, soap_type(p), c_ident(p));
          else if (p->type != Tpointer)
            fprintf(fout, "\n\t\tif (!soap_match_tag(soap, t, \"%s\"))\n\t\t{\t*type = %s;\n\t\t\treturn soap_in_%s(soap, tag, NULL, NULL);\n\t\t}", s, soap_type(p), c_ident(p));
        }
      }
    }
  }
}

void
in_defs3(void)
{
  int i;
  Tnode *p;
  const char *s;
  for (i = 0; i < TYPES; i++)
  {
    for (p = Tptr[i]; p; p = p->next)
    {
      if (is_element(p) && !is_transient(p) && !is_template(p) && p->type != Twchar && p->type != Tfun && p->type != Treference && p->type != Trvalueref && p->type != Tunion && !is_XML(p) && !is_header_or_fault(p) && !is_body(p))
      {
        s = xsi_type(p);
        if (!*s)
          s = wsdl_type(p, "");
        if (*s == '-')
          continue;
        if (*s)
        {
          if (is_dynamic_array(p) && !is_binary(p) && !has_ns(p) && !is_untyped(p))
            fprintf(fout, "\n\t\tif (*soap->arrayType && !soap_match_array(soap, \"%s\"))\n\t\t{\t*type = %s;\n\t\t\treturn soap_in_%s(soap, NULL, NULL, NULL);\n\t\t}", s, soap_type(p), c_ident(p));
          else if (is_string(p))
            fprintf(fout, "\n\t\tif (!soap_match_tag(soap, t, \"%s\"))\n\t\t{\tchar **s;\n\t\t\t*type = %s;\n\t\t\ts = soap_in_%s(soap, NULL, NULL, NULL);\n\t\t\treturn s ? *s : NULL;\n\t\t}", s, soap_type(p), c_ident(p));
          else if (is_wstring(p))
            fprintf(fout, "\n\t\tif (!soap_match_tag(soap, t, \"%s\"))\n\t\t{\twchar_t **s;\n\t\t\t*type = %s;\n\t\t\ts = soap_in_%s(soap, NULL, NULL, NULL);\n\t\t\treturn s ? *s : NULL;\n\t\t}", s, soap_type(p), c_ident(p));
          else if (p->type != Tpointer)
            fprintf(fout, "\n\t\tif (!soap_match_tag(soap, t, \"%s\"))\n\t\t{\t*type = %s;\n\t\t\treturn soap_in_%s(soap, NULL, NULL, NULL);\n\t\t}", s, soap_type(p), c_ident(p));
        }
      }
    }
  }
}

void
out_defs(void)
{
  int i;
  const char *s;
  Tnode *p;
  for (i = 0; i < TYPES; i++)
  {
    for (p = Tptr[i]; p; p = p->next)
    {
      if (is_transient(p) || is_template(p) || is_XML(p) || is_header_or_fault(p) || is_body(p))
        continue;
      if (is_element(p))
      {
        s = wsdl_type(p, "");
        if (*s == '-')
          continue;
        if (p->type == Tarray)
          fprintf(fout, "\n\tcase %s:\n\t\treturn soap_out_%s(soap, \"%s\", id, (%s)ptr, \"\");", soap_type(p), c_ident(p), s, c_type_id((Tnode*)p->ref, "(*)"));
        else if (p->type == Tclass && !is_external(p) && !is_volatile(p) && !is_typedef(p))
          fprintf(fout, "\n\tcase %s:\n\t\treturn ((%s)ptr)->soap_out(soap, \"%s\", id, \"\");", soap_type(p), c_type_id(p, "*"), s);
        else if (is_string(p))
          fprintf(fout, "\n\tcase %s:\n\t\treturn soap_out_string(soap, \"%s\", id, (char*const*)(void*)&ptr, \"\");", soap_type(p), s);
        else if (is_wstring(p))
          fprintf(fout, "\n\tcase %s:\n\t\treturn soap_out_wstring(soap, \"%s\", id, (wchar_t*const*)(void*)&ptr, \"\");", soap_type(p), s);
        else if (p->type == Tpointer)
          fprintf(fout, "\n\tcase %s:\n\t\treturn soap_out_%s(soap, \"%s\", id, (%s)ptr, \"\");", soap_type(p), c_ident(p), s, c_type_constptr_id(p, "const*"));
        else if (p->type != Tnone && p->type != Ttemplate && p->type != Twchar && !is_void(p) && p->type != Tfun && p->type != Treference && p->type != Trvalueref && p->type != Tunion)
          fprintf(fout, "\n\tcase %s:\n\t\treturn soap_out_%s(soap, \"%s\", id, (const %s)ptr, \"\");", soap_type(p), c_ident(p), s, c_type_id(p, "*"));
      }
      else
      {
        s = xsi_type(p);
        if (!*s)
          s = wsdl_type(p, "");
        if (*s == '-')
          continue;
        if (p->type == Tarray)
          fprintf(fout, "\n\tcase %s:\n\t\treturn soap_out_%s(soap, tag, id, (%s)ptr, \"%s\");", soap_type(p), c_ident(p), c_type_id((Tnode*)p->ref, "(*)"), s);
        else if (p->type == Tclass && !is_external(p) && !is_volatile(p) && !is_typedef(p))
          fprintf(fout, "\n\tcase %s:\n\t\treturn ((%s)ptr)->soap_out(soap, tag, id, \"%s\");", soap_type(p), c_type_id(p, "*"), s);
        else if (is_string(p))
          fprintf(fout, "\n\tcase %s:\n\t\treturn soap_out_string(soap, tag, id, (char*const*)(void*)&ptr, \"%s\");", soap_type(p), s);
        else if (is_wstring(p))
          fprintf(fout, "\n\tcase %s:\n\t\treturn soap_out_wstring(soap, tag, id, (wchar_t*const*)(void*)&ptr, \"%s\");", soap_type(p), s);
        else if (p->type == Tpointer)
          fprintf(fout, "\n\tcase %s:\n\t\treturn soap_out_%s(soap, tag, id, (%s)ptr, \"%s\");", soap_type(p), c_ident(p), c_type_constptr_id(p, "const*"), s);
        else if (p->type != Tnone && p->type != Ttemplate && p->type != Twchar && !is_void(p) && p->type != Tfun && p->type != Treference && p->type != Trvalueref && p->type != Tunion)
          fprintf(fout, "\n\tcase %s:\n\t\treturn soap_out_%s(soap, tag, id, (const %s)ptr, \"%s\");", soap_type(p), c_ident(p), c_type_id(p, "*"), s);
      }
    }
  }
}

void
mark_defs()
{
  int i;
  Tnode *p;
  for (i = 0; i < TYPES; i++)
  {
    for (p = Tptr[i]; p; p = p->next)
    {
      if (is_transient(p) || is_template(p) || is_XML(p) || is_header_or_fault(p) || is_body(p) || is_void(p))
        continue;
      if (p->type == Tarray)
        continue;
      else if (p->type == Tclass && !is_external(p) && !is_volatile(p) && !is_typedef(p))
        fprintf(fout, "\n\tcase %s:\n\t\t((%s)ptr)->soap_serialize(soap);\n\t\tbreak;", soap_type(p), c_type_id(p, "*"));
      else if (is_string(p))
        fprintf(fout, "\n\tcase %s:\n\t\tsoap_serialize_string(soap, (char*const*)(void*)&ptr);\n\t\tbreak;", soap_type(p));
      else if (is_wstring(p))
        fprintf(fout, "\n\tcase %s:\n\t\tsoap_serialize_wstring(soap, (wchar_t*const*)(void*)&ptr);\n\t\tbreak;", soap_type(p));
      else if (p->type == Tpointer)
        fprintf(fout, "\n\tcase %s:\n\t\tsoap_serialize_%s(soap, (%s)ptr);\n\t\tbreak;", soap_type(p), c_ident(p), c_type_constptr_id(p, "const*"));
      else if (p->type == Ttemplate && p->ref)
        fprintf(fout, "\n\tcase %s:\n\t\tsoap_serialize_%s(soap, (const %s)ptr);\n\t\tbreak;", soap_type(p), c_ident(p), c_type_id(p, "*"));
      else if (!is_primitive(p) && p->type != Tnone && p->type != Ttemplate && !is_void(p) && p->type != Tfun && p->type != Treference && p->type != Trvalueref && p->type != Tunion)
        fprintf(fout, "\n\tcase %s:\n\t\tsoap_serialize_%s(soap, (const %s)ptr);\n\t\tbreak;", soap_type(p), c_ident(p), c_type_id(p, "*"));
    }
  }
}

void
dup_defs()
{
  int i;
  Tnode *p;
  for (i = 0; i < TYPES; i++)
  {
    for (p = Tptr[i]; p; p = p->next)
    {
      if (is_transient(p) || is_template(p) || is_XML(p) || is_header_or_fault(p) || is_body(p) || is_void(p))
        continue;
      if (p->type == Tarray)
        continue;
      else if (p->type == Tclass && !is_external(p) && !is_volatile(p) && !is_typedef(p))
        fprintf(fout, "\n\tcase %s:\n\t\treturn (void*)((%s)ptr)->soap_dup(soap);", soap_type(p), c_type_id(p, "*"));
      else if (is_string(p))
        fprintf(fout, "\n\tcase %s:\n\t\treturn (void*)soap_strdup(soap, (const char*)ptr);", soap_type(p));
      else if (is_wstring(p))
        fprintf(fout, "\n\tcase %s:\n\t\treturn (void*)soap_wstrdup(soap, (const wchar_t*)ptr);", soap_type(p));
      else if (p->type == Tpointer)
        fprintf(fout, "\n\tcase %s:\n\t\treturn (void*)%s_dup_%s(soap, NULL, (%s)ptr);", soap_type(p), fprefix, c_ident(p), c_type_constptr_id(p, "const*"));
      else if (p->type == Ttemplate && p->ref)
        fprintf(fout, "\n\tcase %s:\n\t\treturn (void*)%s_dup_%s(soap, NULL, (const %s)ptr);", soap_type(p), fprefix, c_ident(p), c_type_id(p, "*"));
      else if (!is_primitive(p) && p->type != Tnone && p->type != Ttemplate && !is_void(p) && p->type != Tfun && p->type != Treference && p->type != Trvalueref && p->type != Tunion)
        fprintf(fout, "\n\tcase %s:\n\t\treturn (void*)%s_dup_%s(soap, NULL, (const %s)ptr);", soap_type(p), fprefix, c_ident(p), c_type_id(p, "*"));
      else if (p->type != Tnone && p->type != Ttemplate && !is_void(p) && p->type != Tfun && p->type != Treference && p->type != Trvalueref && p->type != Tunion)
        fprintf(fout, "\n\tcase %s:\n\t\treturn soap_memdup(soap, ptr, sizeof(%s));", soap_type(p), c_type(p));
    }
  }
}

void
del_defs()
{
  int i;
  Tnode *p;
  for (i = 0; i < TYPES; i++)
  {
    for (p = Tptr[i]; p; p = p->next)
    {
      if (is_transient(p) || is_template(p) || is_XML(p) || is_header_or_fault(p) || is_body(p) || is_void(p))
        continue;
      if (p->type == Tarray)
        continue;
      else if (p->type == Tclass && !is_external(p) && !is_volatile(p) && !is_typedef(p))
        fprintf(fout, "\n\tcase %s:\n\t\tstatic_cast<const %s*>(ptr)->soap_del();\n\t\tSOAP_DELETE(NULL, static_cast<const %s*>(ptr), %s);\n\t\tbreak;", soap_type(p), c_type(p), c_type(p), c_type(p));
      else if (is_string(p))
        fprintf(fout, "\n\tcase %s:\n\t\t%s_del_string((char*const*)(void*)&ptr);\n\t\tbreak;", soap_type(p), fprefix);
      else if (is_wstring(p))
        fprintf(fout, "\n\tcase %s:\n\t\t%s_del_wstring((wchar_t*const*)(void*)&ptr);\n\t\tbreak;", soap_type(p), fprefix);
      else if (p->type == Tpointer)
        fprintf(fout, "\n\tcase %s:\n\t\t%s_del_%s((%s)ptr);\n\t\tSOAP_FREE(NULL, ptr);\n\t\tbreak;", soap_type(p), fprefix, c_ident(p), c_type_constptr_id(p, "const*"));
      else if (p->type == Ttemplate && p->ref)
        fprintf(fout, "\n\tcase %s:\n\t\t%s_del_%s((const %s)ptr);\n\t\tSOAP_DELETE(NULL, static_cast<const %s*>(ptr), %s);\n\t\tbreak;", soap_type(p), fprefix, c_ident(p), c_type_id(p, "*"), c_type(p), c_type(p));
      else if (!is_primitive(p) && p->type != Tnone && p->type != Ttemplate && !is_void(p) && p->type != Tfun && p->type != Treference && p->type != Trvalueref && p->type != Tunion)
      {
        if (!cflag && (p->type == Tstruct || p->type == Tclass))
          fprintf(fout, "\n\tcase %s:\n\t\t%s_del_%s((const %s)ptr);\n\t\tSOAP_DELETE(NULL, static_cast<const %s*>(ptr), %s);\n\t\tbreak;", soap_type(p), fprefix, c_ident(p), c_type_id(p, "*"), c_type(p), c_type(p));
        else
          fprintf(fout, "\n\tcase %s:\n\t\t%s_del_%s((const %s)ptr);\n\t\tSOAP_FREE(NULL, ptr);\n\t\tbreak;", soap_type(p), fprefix, c_ident(p), c_type_id(p, "*"));
      }
      else if (p->type != Tnone && p->type != Ttemplate && !is_void(p) && p->type != Tfun && p->type != Treference && p->type != Trvalueref && p->type != Tunion)
        fprintf(fout, "\n\tcase %s:\n\t\tSOAP_FREE(NULL, (const %s)ptr);\n\t\tbreak;", soap_type(p), c_type_id(p, "*"));
    }
  }
}

void
in_attach(void)
{
  int i;
  Tnode *p;
  for (i = 0; i < TYPES; i++)
  {
    for (p = Tptr[i]; p; p = p->next)
    {
      if (is_attachment(p))
      {
        if (p->type == Tclass)
          fprintf(fout, "\n\t\tcase %s:\n\t\t{\t%s a;\n\t\t\ta = (%s)soap_id_enter(soap, soap->dime.id, NULL, %s, sizeof(%s), NULL, NULL, %s_instantiate, %s_fbase);\n\t\t\tif (a)\n\t\t\t{\ta->__ptr = (unsigned char*)soap->dime.ptr;\n\t\t\t\ta->__size = soap->dime.size;\n\t\t\t\ta->id = (char*)soap->dime.id;\n\t\t\t\ta->type = (char*)soap->dime.type;\n\t\t\t\ta->options = (char*)soap->dime.options;\n\t\t\t}\n\t\t\telse\n\t\t\t\treturn soap->error;\n\t\t\tbreak;\n\t\t}", soap_type(p), c_type_id(p, "*"), c_type_id(p, "*"), soap_type(p), c_type(p), prefix, prefix);
        else
          fprintf(fout, "\n\t\tcase %s:\n\t\t{\t%s a;\n\t\t\ta = (%s)soap_id_enter(soap, soap->dime.id, NULL, %s, sizeof(%s), NULL, NULL, NULL, NULL);\n\t\t\tif (!a)\n\t\t\t\treturn soap->error;\n\t\t\ta->__ptr = (unsigned char*)soap->dime.ptr;\n\t\t\ta->__size = soap->dime.size;\n\t\t\ta->id = (char*)soap->dime.id;\n\t\t\ta->type = (char*)soap->dime.type;\n\t\t\ta->options = (char*)soap->dime.options;\n\t\t\tbreak;\n\t\t}", soap_type(p), c_type_id(p, "*"), c_type_id(p, "*"), soap_type(p), c_type(p));
      }
      else if (is_binary(p) && !is_transient(p))
      {
        if (p->type == Tclass)
          fprintf(fout, "\n\t\tcase %s:\n\t\t{\t%s a;\n\t\t\ta = (%s)soap_id_enter(soap, soap->dime.id, NULL, %s, sizeof(%s), NULL, NULL, %s_instantiate, %s_fbase);\n\t\t\tif (!a)\n\t\t\t\treturn soap->error;\n\t\t\ta->__ptr = (unsigned char*)soap->dime.ptr;\n\t\t\ta->__size = soap->dime.size;\n\t\t\tbreak;\n\t\t}", soap_type(p), c_type_id(p, "*"), c_type_id(p, "*"), soap_type(p), c_type(p), prefix, prefix);
        else
          fprintf(fout, "\n\t\tcase %s:\n\t\t{\t%s a;\n\t\t\ta = (%s)soap_id_enter(soap, soap->dime.id, NULL, %s, sizeof(%s), NULL, NULL, NULL, NULL);\n\t\t\tif (!a)\n\t\t\t\treturn soap->error;\n\t\t\ta->__ptr = (unsigned char*)soap->dime.ptr;\n\t\t\ta->__size = soap->dime.size;\n\t\t\tbreak;\n\t\t}", soap_type(p), c_type_id(p, "*"), c_type_id(p, "*"), soap_type(p), c_type(p));
      }
    }
  }
}

void
soap_instantiate(Tnode *typ)
{
  Table *Tptr;
  Entry *Eptr;
  int derclass = 0, flag = 0;
  const char *s;

  if (is_XML(typ))
    return;
  if (typ->type == Tarray)
    return;
  if (typ->type == Tunion)
    return;
  if (typ->type == Tpointer && !is_string(typ) && !is_wstring(typ))
    return;

  if (cflag)
  {
    if ((is_typedef(typ) && !is_external(typ)) || is_restriction(typ))
      fprintf(fhead, "\n#define soap_new_%s soap_new_%s\n", c_ident(typ), t_ident(typ));
    else
    {
      fprintf(fhead, "\n\nSOAP_FMAC3 %s * SOAP_FMAC4 soap_new_%s(struct soap *soap, int n);", c_type(typ), c_ident(typ));
      fprintf(fout, "\n\nSOAP_FMAC3 %s * SOAP_FMAC4 soap_new_%s(struct soap *soap, int n)\n{\n\t%s *p;\n\t%s *a = (%s*)soap_malloc((soap), (n = (n < 0 ? 1 : n)) * sizeof(%s));\n\tfor (p = a; p && n--; p++)\n\t\tsoap_default_%s(soap, p);\n\treturn a;\n}", c_type(typ), c_ident(typ), c_type(typ), c_type(typ), c_type(typ), c_type(typ), c_ident(typ));
    }
    return;
  }

  if (typ->type != Tclass || !typ->sym || !is_eq(typ->sym->name, "xsd__QName") || is_imported(typ))
  {
    if ((is_typedef(typ) && !is_external(typ)) || is_restriction(typ))
    {
      fprintf(fhead, "\n\n#define %s_instantiate_%s %s_instantiate_%s\n", fprefix, c_ident(typ), fprefix, t_ident(typ));
      fprintf(fhead, "\n\n#define soap_new_%s soap_new_%s\n", c_ident(typ), t_ident(typ));
      if ((typ->type == Tclass || typ->type == Tstruct) && typ->ref)
      {
        fprintf(fhead, "\n\n#define soap_new_req_%s soap_new_req_%s\n", c_ident(typ), t_ident(typ));
        fprintf(fhead, "\n\n#define soap_new_set_%s soap_new_set_%s\n", c_ident(typ), t_ident(typ));
      }
      return;
    }
  }

  if (is_primitive(typ) || is_string(typ) || is_wstring(typ))
  {
    fprintf(fhead, "\n\nSOAP_FMAC3 %s * SOAP_FMAC4 soap_new_%s(struct soap *soap, int n = -1);", c_type(typ), c_ident(typ));
    fprintf(fout, "\n\nSOAP_FMAC3 %s * SOAP_FMAC4 soap_new_%s(struct soap *soap, int n)\n{\n\t%s *a = static_cast<%s *>(soap_malloc(soap, (n = (n < 0 ? 1 : n)) * sizeof(%s)));\n\tfor (%s *p = a; p && n--; ++p)\n\t\tsoap_default_%s(soap, p);\n\treturn a;\n}", c_type(typ), c_ident(typ), c_type(typ), c_type(typ), c_type(typ), c_type(typ), c_ident(typ));
  }

  /* NO LONGER CONSIDERED: soap_new_copy_Name1 may clash with soap_new_Name2 and soap_copy_X is already in use
  if (is_primitive(typ))
    fprintf(fhead, "\n\ninline %s * soap_new_copy_%s(struct soap *soap, const %s& a)\n{\n\t%s *p = (%s*)soap_malloc(soap, sizeof(%s));\n\tif (p)\n\t\t*p = a;\n\treturn p;\n}", c_type(typ), c_ident(typ), c_type(typ), c_type(typ), c_type(typ), c_type(typ));
  else if (is_string(typ))
    fprintf(fhead, "\n\ninline char * soap_new_copy_string(struct soap *soap, const char *a)\n{\n\treturn soap_strdup(soap, a);\n}");
  else if (is_wstring(typ))
    fprintf(fhead, "\n\ninline wchar_t * soap_new_copy_wstring(struct soap *soap, const wchar_t *a)\n{\n\treturn soap_wstrdup(soap, a);\n}");
  */

  if (typ->type != Tstruct && typ->type != Tclass && typ->type != Ttemplate)
    return;

  fprintf(fhead, "\nSOAP_FMAC1 %s * SOAP_FMAC2 %s_instantiate_%s(struct soap*, int, const char*, const char*, size_t*);", c_type(typ), fprefix, c_ident(typ));

  if (namespaceid && !is_external(typ))
    fprintf(fhead, "\n\ninline %s * soap_new_%s(struct soap *soap, int n = -1)\n{\n\treturn %s::%s_instantiate_%s(soap, n, NULL, NULL, NULL);\n}", c_type(typ), c_ident(typ), namespaceid, fprefix, c_ident(typ));
  else
    fprintf(fhead, "\n\ninline %s * soap_new_%s(struct soap *soap, int n = -1)\n{\n\treturn %s_instantiate_%s(soap, n, NULL, NULL, NULL);\n}", c_type(typ), c_ident(typ), fprefix, c_ident(typ));

  /* NO LONGER CONSIDERED: soap_new_copy_Name1 may clash with soap_new_Name2
  if (is_stdstring(typ))
  {
    fprintf(fhead, "\n\ninline std::string * soap_new_copy_%s(struct soap *soap, const std::string& a)\n{\n\tstd::string *p = soap_new_%s(soap);\n\tif (p)\n\t\t*p = a;\n\treturn p;\n}", c_ident(typ), c_ident(typ));
    fprintf(fhead, "\n\ninline std::string * soap_new_copy_%s(struct soap *soap, const char *a)\n{\n\tstd::string *p = soap_new_%s(soap);\n\tif (p)\n\t\tp->assign(a);\n\treturn p;\n}", c_ident(typ), c_ident(typ));
  }
  else if (is_stdwstring(typ))
  {
    fprintf(fhead, "\n\ninline std::wstring * soap_new_copy_%s(struct soap *soap, const std::wstring& a)\n{\n\tstd::wstring *p = soap_new_%s(soap);\n\tif (p)\n\t\t*p = a;\n\treturn p;\n}", c_ident(typ), c_ident(typ));
    fprintf(fhead, "\n\ninline std::wstring * soap_new_copy_%s(struct soap *soap, const wchar_t *a)\n{\n\tstd::wstring *p = soap_new_%s(soap);\n\tif (p)\n\t\tp->assign(a);\n\treturn p;\n}", c_ident(typ), c_ident(typ));
  }
  else if (!is_template(typ))
  {
    fprintf(fhead, "\n\ninline %s * soap_new_copy_%s(struct soap *soap, const %s& a)\n{\n\t%s *p = soap_new_%s(soap);\n\tif (p)\n\t\t*p = a;\n\treturn p;\n}", c_type(typ), c_ident(typ), c_type(typ), c_type(typ), c_ident(typ));
  }
  */

  if (typ->type == Tclass || typ->type == Tstruct)
  {
    fprintf(fhead, "\n\ninline %s * soap_new_req_%s(\n\tstruct soap *soap", c_type(typ), c_ident(typ));
    if (!is_dynamic_array(typ))
    {
      for (Tptr = (Table*)typ->ref, derclass = 0; Tptr; Tptr = Tptr->prev, derclass++)
      {
        for (Eptr = Tptr->list; Eptr; Eptr = Eptr->next)
        {
          if (is_repetition(Eptr) || is_anytype(Eptr))
            flag = 2;
          if ((Eptr->info.minOccurs > 0 || flag) && !(Eptr->info.sto & (Sprivate | Sprotected | Sconst | Sstatic | Stypedef)) && Eptr->info.typ->type != Tfun && !is_soapref(Eptr->info.typ))
          {
            if (flag)
              flag--;
            if (is_smart(Eptr->info.typ))
            {
              if (is_smart_shared(Eptr->info.typ))
                fprintf(fhead, ",\n\t%s %s", c_type_id(Eptr->info.typ, "&"), ident(Eptr->sym->name));
              else
                fprintf(fhead, ",\n\t%s %s", c_type_id(Eptr->info.typ->ref, "*"), ident(Eptr->sym->name));
            }
            else if (Eptr->info.typ->type == Ttemplate && is_smart(Eptr->info.typ->ref) && !is_smart_shared(Eptr->info.typ->ref))
              continue;
            else if (Eptr->info.typ->type == Tclass || Eptr->info.typ->type == Tstruct || Eptr->info.typ->type == Tunion || Eptr->info.typ->type == Ttemplate)
              fprintf(fhead, ",\n\tconst %s& %s", c_type(Eptr->info.typ), ident(Eptr->sym->name));
            else if ((Eptr->info.sto & Sconstptr))
              fprintf(fhead, ",\n\tconst %s", c_type_id(Eptr->info.typ, Eptr->sym->name));
            else if (Eptr->info.typ->type == Tarray)
              fprintf(fhead, ",\n\t%s", c_type_id(Eptr->info.typ, Eptr->sym->name));
            else
              fprintf(fhead, ",\n\t%s", c_type_id(Eptr->info.typ, Eptr->sym->name));
            if (derclass && Eptr->info.typ->type != Tarray)
              fprintf(fhead, "__%d", derclass);
          }
        }
      }
    }
    if (namespaceid && !is_external(typ))
      fprintf(fhead, ")\n{\n\t%s = %s::soap_new_%s(soap);\n\tif (_p)\n\t{\t", c_type_id(typ, "*_p"), namespaceid, c_ident(typ));
    else if (!is_external(typ))
      fprintf(fhead, ")\n{\n\t%s = ::soap_new_%s(soap);\n\tif (_p)\n\t{\t", c_type_id(typ, "*_p"), c_ident(typ));
    else
      fprintf(fhead, ")\n{\n\t%s = soap_new_%s(soap);\n\tif (_p)\n\t{\t", c_type_id(typ, "*_p"), c_ident(typ));
    if (!is_external(typ))
    {
      if (typ->type == Tclass && !is_volatile(typ))
        fprintf(fhead, "_p->soap_default(soap);");
      else if (namespaceid && !is_external(typ))
        fprintf(fhead, "%s::soap_default_%s(soap, _p);", namespaceid, c_ident(typ));
      else if (!is_external(typ))
        fprintf(fhead, "::soap_default_%s(soap, _p);", c_ident(typ));
      else
        fprintf(fhead, "soap_default_%s(soap, _p);", c_ident(typ));
    }
    flag = 0;
    if (!is_dynamic_array(typ))
    {
      for (Tptr = (Table*)typ->ref, derclass = 0; Tptr; Tptr = Tptr->prev, derclass++)
      {
        for (Eptr = Tptr->list; Eptr; Eptr = Eptr->next)
        {
          if (is_repetition(Eptr) || is_anytype(Eptr))
            flag = 2;
          if ((Eptr->info.minOccurs > 0 || flag) && !(Eptr->info.sto & (Sprivate | Sprotected | Sconst | Sstatic | Stypedef)) && Eptr->info.typ->type != Tfun && !is_soapref(Eptr->info.typ))
          {
            if (flag)
              flag--;
            if (is_smart(Eptr->info.typ) && !is_smart_shared(Eptr->info.typ)) /* smart but not shared */
            {
              if (typ->type == Tclass)
                fprintf(fhead, "\n\t\t_p->%s::%s = %s(%s", ident(Tptr->sym->name), ident(Eptr->sym->name), c_type(Eptr->info.typ), ident(Eptr->sym->name));
              else
                fprintf(fhead, "\n\t\t_p->%s = %s(%s", ident(Eptr->sym->name), c_type(Eptr->info.typ), ident(Eptr->sym->name));
            }
            else if (Eptr->info.typ->type == Tarray)
            {
              int cardinality;
              Tnode *ref = get_item_type(Eptr->info.typ, &cardinality);
              if (cardinality > 1)
              {
                const char *t = c_type(ref);
                if (typ->type == Tclass)
                  fprintf(fhead, "\n\t\tfor (int i = 0; i < %d; i++)\n\t\t\t((%s*)(_p->%s::%s))[i] = ((%s*)%s)", get_dimension_product(Eptr->info.typ), t, ident(Tptr->sym->name), ident(Eptr->sym->name), t, ident(Eptr->sym->name));
                else
                  fprintf(fhead, "\n\t\tfor (int i = 0; i < %d; i++)\n\t\t\t((%s*)(_p->%s))[i] = ((%s*)%s)", get_dimension_product(Eptr->info.typ), t, ident(Eptr->sym->name), t, ident(Eptr->sym->name));
              }
              else
              {
                if (typ->type == Tclass)
                  fprintf(fhead, "\n\t\tfor (int i = 0; i < %d; i++)\n\t\t\t_p->%s::%s[i] = %s", get_dimension(Eptr->info.typ), ident(Tptr->sym->name), ident(Eptr->sym->name), ident(Eptr->sym->name));
                else
                  fprintf(fhead, "\n\t\tfor (int i = 0; i < %d; i++)\n\t\t\t_p->%s[i] = %s", get_dimension(Eptr->info.typ), ident(Eptr->sym->name), ident(Eptr->sym->name));
              }
            }
            else if (Eptr->info.typ->type == Ttemplate && is_smart(Eptr->info.typ->ref) && !is_smart_shared(Eptr->info.typ->ref))
              continue;
            else if (typ->type == Tclass)
              fprintf(fhead, "\n\t\t_p->%s::%s = %s", ident(Tptr->sym->name), ident(Eptr->sym->name), ident(Eptr->sym->name));
            else
              fprintf(fhead, "\n\t\t_p->%s = %s", ident(Eptr->sym->name), ident(Eptr->sym->name));
            if (derclass && Eptr->info.typ->type != Tarray)
              fprintf(fhead, "__%d", derclass);
            if (Eptr->info.typ->type == Tarray)
              fprintf(fhead, "[i];");
            else if (is_smart(Eptr->info.typ) && !is_smart_shared(Eptr->info.typ)) /* smart but not shared */
              fprintf(fhead, ");");
            else
              fprintf(fhead, ";");
          }
        }
      }
    }
    fprintf(fhead, "\n\t}\n\treturn _p;\n}");
    fprintf(fhead, "\n\ninline %s * soap_new_set_%s(\n\tstruct soap *soap", c_type(typ), c_ident(typ));
    for (Tptr = (Table*)typ->ref, derclass = 0; Tptr; Tptr = Tptr->prev, derclass++)
    {
      for (Eptr = Tptr->list; Eptr; Eptr = Eptr->next)
      {
        if (!(Eptr->info.sto & (Sprivate | Sprotected | Sconst | Sstatic | Stypedef)) && Eptr->info.typ->type != Tfun && strcmp(Eptr->sym->name, "soap"))
        {
          if (is_smart(Eptr->info.typ))
          {
            if (is_smart_shared(Eptr->info.typ))
              fprintf(fhead, ",\n\t%s %s", c_type_id(Eptr->info.typ, "&"), ident(Eptr->sym->name));
            else
              fprintf(fhead, ",\n\t%s %s", c_type_id(Eptr->info.typ->ref, "*"), ident(Eptr->sym->name));
          }
          else if (Eptr->info.typ->type == Ttemplate && is_smart(Eptr->info.typ->ref) && !is_smart_shared(Eptr->info.typ->ref))
            continue;
          else if (Eptr->info.typ->type == Tclass || Eptr->info.typ->type == Tstruct || Eptr->info.typ->type == Tunion || Eptr->info.typ->type == Ttemplate)
            fprintf(fhead, ",\n\tconst %s& %s", c_type(Eptr->info.typ), ident(Eptr->sym->name));
          else if ((Eptr->info.sto & Sconstptr))
            fprintf(fhead, ",\n\tconst %s", c_type_id(Eptr->info.typ, Eptr->sym->name));
          else if (Eptr->info.typ->type == Tarray)
            fprintf(fhead, ",\n\t%s", c_type_id(Eptr->info.typ, Eptr->sym->name));
          else
            fprintf(fhead, ",\n\t%s", c_type_id(Eptr->info.typ, Eptr->sym->name));
          if (derclass && Eptr->info.typ->type != Tarray)
            fprintf(fhead, "__%d", derclass);
        }
      }
    }
    if (namespaceid && !is_external(typ))
      fprintf(fhead, ")\n{\n\t%s = %s::soap_new_%s(soap);\n\tif (_p)\n\t{\t", c_type_id(typ, "*_p"), namespaceid, c_ident(typ));
    else if (!is_external(typ))
      fprintf(fhead, ")\n{\n\t%s = ::soap_new_%s(soap);\n\tif (_p)\n\t{\t", c_type_id(typ, "*_p"), c_ident(typ));
    else
      fprintf(fhead, ")\n{\n\t%s = soap_new_%s(soap);\n\tif (_p)\n\t{\t", c_type_id(typ, "*_p"), c_ident(typ));
    if (!is_external(typ))
    {
      if (typ->type == Tclass && !is_volatile(typ))
        fprintf(fhead, "_p->soap_default(soap);");
      else if (namespaceid && !is_external(typ))
        fprintf(fhead, "%s::soap_default_%s(soap, _p);", namespaceid, c_ident(typ));
      else if (!is_external(typ))
        fprintf(fhead, "::soap_default_%s(soap, _p);", c_ident(typ));
      else
        fprintf(fhead, "soap_default_%s(soap, _p);", c_ident(typ));
    }
    for (Tptr = (Table*)typ->ref, derclass = 0; Tptr; Tptr = Tptr->prev, derclass++)
    {
      for (Eptr = Tptr->list; Eptr; Eptr = Eptr->next)
      {
        if (!(Eptr->info.sto & (Sprivate | Sprotected | Sconst | Sstatic | Stypedef)) && Eptr->info.typ->type != Tfun && strcmp(Eptr->sym->name, "soap"))
        {
          if (is_smart(Eptr->info.typ) && !is_smart_shared(Eptr->info.typ)) /* smart but not shared */
          {
            if (typ->type == Tclass)
              fprintf(fhead, "\n\t\t_p->%s::%s = %s(%s", ident(Tptr->sym->name), ident(Eptr->sym->name), c_type(Eptr->info.typ), ident(Eptr->sym->name));
            else
              fprintf(fhead, "\n\t\t_p->%s = %s(%s", ident(Eptr->sym->name), c_type(Eptr->info.typ), ident(Eptr->sym->name));
          }
          else if (Eptr->info.typ->type == Tarray)
          {
            int cardinality;
            Tnode *ref = get_item_type(Eptr->info.typ, &cardinality);
            if (cardinality > 1)
            {
              const char *t = c_type(ref);
              if (typ->type == Tclass)
                fprintf(fhead, "\n\t\tfor (int i = 0; i < %d; i++)\n\t\t\t((%s*)(_p->%s::%s))[i] = ((%s*)%s)", get_dimension_product(Eptr->info.typ), t, ident(Tptr->sym->name), ident(Eptr->sym->name), t, ident(Eptr->sym->name));
              else
                fprintf(fhead, "\n\t\tfor (int i = 0; i < %d; i++)\n\t\t\t((%s*)(_p->%s))[i] = ((%s*)%s)", get_dimension_product(Eptr->info.typ), t, ident(Eptr->sym->name), t, ident(Eptr->sym->name));
            }
            else
            {
              if (typ->type == Tclass)
                fprintf(fhead, "\n\t\tfor (int i = 0; i < %d; i++)\n\t\t\t_p->%s::%s[i] = %s", get_dimension(Eptr->info.typ), ident(Tptr->sym->name), ident(Eptr->sym->name), ident(Eptr->sym->name));
              else
                fprintf(fhead, "\n\t\tfor (int i = 0; i < %d; i++)\n\t\t\t_p->%s[i] = %s", get_dimension(Eptr->info.typ), ident(Eptr->sym->name), ident(Eptr->sym->name));
            }
          }
          else if (Eptr->info.typ->type == Ttemplate && is_smart(Eptr->info.typ->ref) && !is_smart_shared(Eptr->info.typ->ref))
            continue;
          else if (typ->type == Tclass)
            fprintf(fhead, "\n\t\t_p->%s::%s = %s", ident(Tptr->sym->name), ident(Eptr->sym->name), ident(Eptr->sym->name));
          else
            fprintf(fhead, "\n\t\t_p->%s = %s", ident(Eptr->sym->name), ident(Eptr->sym->name));
          if (derclass && Eptr->info.typ->type != Tarray)
            fprintf(fhead, "__%d", derclass);
          if (Eptr->info.typ->type == Tarray)
            fprintf(fhead, "[i];");
          else if (is_smart(Eptr->info.typ) && !is_smart_shared(Eptr->info.typ)) /* smart but not shared */
            fprintf(fhead, ");");
          else
            fprintf(fhead, ";");
        }
      }
    }
    fprintf(fhead, "\n\t}\n\treturn _p;\n}");
  }

  /* deprecated fprintf(fhead, "\n\ninline void soap_delete_%s(struct soap *soap, %s) { soap_delete(soap, p); }", c_ident(typ), c_type_id(typ, "*p")); */

  fprintf(fout, "\n\nSOAP_FMAC1 %s * SOAP_FMAC2 %s_instantiate_%s(struct soap *soap, int n, const char *type, const char *arrayType, size_t *size)", c_type(typ), fprefix, c_ident(typ));
  fprintf(fout, "\n{");
  fprintf(fout, "\n\tDBGLOG(TEST, SOAP_MESSAGE(fdebug, \"%s_instantiate_%s(%%p, %%d, %%s, %%s)\\n\", (void*)soap, n, type?type:\"\", arrayType?arrayType:\"\"));", fprefix, c_ident(typ));
  fprintf(fout, "\n\t(void)type; (void)arrayType; /* appease -Wall -Werror */");
  for (Eptr = classtable->list; Eptr; Eptr = Eptr->next)
  {
    Tptr = ((Table *) Eptr->info.typ->ref);
    if (Tptr == ((Table *) typ->ref))
    {
      continue;
    }
    derclass = 0;
    while (Tptr)
    {
      if (Tptr == (Table*)typ->ref)
      {
        derclass = 1;
      }
      Tptr = Tptr->prev;
    }
    if (derclass == 1 && !is_transient(Eptr->info.typ))
    {
      if (is_dynamic_array(Eptr->info.typ) && !is_binary(Eptr->info.typ) && !has_ns(Eptr->info.typ) && !is_untyped(Eptr->info.typ))
        fprintf(fout, "\n\tif (soap && arrayType && !soap_match_tag(soap, arrayType, \"%s\"))", xsi_type(Eptr->info.typ));
      else
        fprintf(fout, "\n\tif (soap && type && !soap_match_tag(soap, type, \"%s\"))", the_type(Eptr->info.typ));
      fprintf(fout, "\n\t\treturn %s_instantiate_%s(soap, n, NULL, NULL, size);", fprefix, c_ident(Eptr->info.typ));
      derclass = 0;
    }
  }
  fprintf(fout, "\n\t%s;\n\tsize_t k = sizeof(%s);", c_type_id(typ, "*p"), c_type(typ));
  fprintf(fout, "\n\tstruct soap_clist *cp = soap_link(soap, %s, n, %s_fdelete);", soap_type(typ), prefix);
  fprintf(fout, "\n\tif (!cp && soap && n != SOAP_NO_LINK_TO_DELETE)\n\t\treturn NULL;");
  fprintf(fout, "\n\tif (n < 0)");
  fprintf(fout, "\n\t{\tp = SOAP_NEW(soap, %s);", c_type(typ));
  if ((s = has_soapref(typ)))
    fprintf(fout, "\n\t\tif (p)\n\t\t\tp->%s = soap;", s);
  fprintf(fout, "\n\t}\n\telse");
  fprintf(fout, "\n\t{\tp = SOAP_NEW_ARRAY(soap, %s, n);", c_type(typ));
  fprintf(fout, "\n\t\tk *= n;");
  if (s)
    fprintf(fout, "\n\t\tif (p)\n\t\t\tfor (int i = 0; i < n; i++)\n\t\t\t\tp[i].%s = soap;", s);
  fprintf(fout, "\n\t}");
  fprintf(fout, "\n\tDBGLOG(TEST, SOAP_MESSAGE(fdebug, \"Instantiated %s location=%%p n=%%d\\n\", (void*)p, n));", c_type(typ));
  fprintf(fout, "\n\tif (size)\n\t\t*size = k;");
  fprintf(fout, "\n\tif (!p)\n\t\tsoap->error = SOAP_EOM;");
  fprintf(fout, "\n\telse if (cp)\n\t\tcp->ptr = (void*)p;");
  fprintf(fout, "\n\treturn p;");
  fprintf(fout, "\n}");
}

void
soap_dup(Tnode *typ)
{
  if (!Ecflag)
    return;
  if (is_XML(typ))
    return;
  if (typ->type != Tstruct && typ->type != Tclass && typ->type != Ttemplate && typ->type != Tpointer)
    return;
  if (typ->type != Tclass || !(typ->sym && (is_stdstring(typ) || is_stdwstring(typ)) && is_eq(typ->sym->name, "xsd__QName")) || is_imported(typ))
  {
    if ((is_typedef(typ) && !is_external(typ)) || is_restriction(typ))
    {
      fprintf(fhead, "\n\n#define %s_dup_%s %s_dup_%s\n", fprefix, c_ident(typ), fprefix, t_ident(typ));
      return;
    }
  }
  if (typ->type == Tstruct && (is_anyType(typ) || is_anyAttribute(typ)))
  {
    if (strcmp(fprefix, "soap"))
      fprintf(fhead, "\n\n#define %s_dup_%s soap_dup_%s\n", fprefix, c_ident(typ), c_ident(typ));
    return;
  }
  fprintf(fhead, "\n\nSOAP_FMAC1 %s * SOAP_FMAC2 %s_dup_%s(struct soap*, %s*, %s);", c_type(typ), fprefix, c_ident(typ), c_type(typ), c_type_constptr_id(typ, "const*"));
  fprintf(fout, "\n\nSOAP_FMAC1 %s * SOAP_FMAC2 %s_dup_%s(struct soap *soap, %s *d, %s)\n{", c_type(typ), fprefix, c_ident(typ), c_type(typ), c_type_constptr_id(typ, "const*a"));
  if (typ->type == Tclass || typ->type == Tstruct)
  {
    fprintf(fout, "\n\tstruct soap_plist *pp = NULL;");
    if (typ->recursive)
      fprintf(fout, "\n\tchar *mark = NULL;");
  }
  fprintf(fout, "\n\tif (!a)\n\t\treturn NULL;");
  if (typ->type == Tclass || typ->type == Tstruct)
  {
    if (typ->recursive)
      fprintf(fout, "\n\tif (!d && ((d = (%s*)soap_mark_lookup(soap, (const void*)a, %s, &pp, &mark)) || soap_mark_cycle(soap, pp)))\n\t\treturn d;", c_type(typ), soap_type(typ));
    else
      fprintf(fout, "\n\tif (!d && (d = (%s*)soap_mark_lookup(soap, (const void*)a, %s, &pp, NULL)))\n\t\treturn d;", c_type(typ), soap_type(typ));
  }
  if (cflag)
    fprintf(fout, "\n\tif (!d && !(d = (%s*)soap_malloc(soap, sizeof(%s))))\n\t\treturn NULL; /* ERROR */", c_type(typ), c_type(typ));
  else if (typ->type == Tpointer)
    fprintf(fout, "\n\tif (!d && !(d = (%s*)soap_malloc(soap, sizeof(%s))))\n\t\treturn NULL; /* ERROR */", c_type(typ), c_type(typ));
  else
    fprintf(fout, "\n\tif (!d && !(d = soap_new_%s(soap)))\n\t\treturn NULL; /* ERROR */", c_ident(typ));
  if (typ->type == Tclass || typ->type == Tstruct)
    fprintf(fout, "\n\tsoap_mark_dup(soap, (void*)d, pp);");
  fflush(fout);
  if (is_string(typ))
  {
    fprintf(fout, "\n\t*d = NULL;\n\tif (*a)\n\t{\tstruct soap_plist *pp = NULL;\n\t\tif (!(*d = (%s)soap_mark_lookup(soap, (const void*)*a, %s, &pp, NULL)))", c_type(typ), soap_type(typ));
    fprintf(fout, "\n\t\t\tsoap_mark_dup(soap, *d = soap_strdup(soap, *a), pp);\n\t}\n\telse\n\t\t*d = NULL;");
  }
  else if (is_wstring(typ))
  {
    fprintf(fout, "\n\tif (*a)\n\t{\tstruct soap_plist *pp = NULL;\n\t\tif (!(*d = (%s)soap_mark_lookup(soap, (const void*)*a, %s, &pp, NULL)))", c_type(typ), soap_type(typ));
    fprintf(fout, "\n\t\t\tsoap_mark_dup(soap, *d = soap_wstrdup(soap, *a), pp);\n\t}\n\telse\n\t\t*d = NULL;");
  }
  else if (typ->type == Tpointer)
  {
    Tnode *ref = typ->ref;
    if (ref->type == Tclass && !is_external(ref) && !is_volatile(ref) && !is_typedef(ref))
    {
      fprintf(fout, "\n\tif (*a)\n\t\t*d = (*a)->soap_dup(soap);\n\telse\n\t\t*d = NULL;");
    }
    else if (is_XML(ref) && is_string(ref))
    {
      if (cflag)
        fprintf(fout, "\n\tif (*a && (*d = (%s)soap_malloc(soap, sizeof(%s))))\n\t\t**d = soap_strdup(soap, **a);\n\telse\n\t\t*d = NULL;", c_type(typ), c_type(ref));
      else
        fprintf(fout, "\n\tif (*a && (*d = (%s)soap_malloc(soap, sizeof(%s))))\n\t\t**d = soap_strdup(soap, **a);\n\telse\n\t\t*d = NULL;", c_type(typ), c_type(ref));
    }
    else if (is_XML(ref) && is_wstring(ref))
    {
      if (cflag)
        fprintf(fout, "\n\tif (*a && (*d = (%s)soap_malloc(soap, sizeof(%s))))\n\t\t**d = soap_wstrdup(soap, **a);\n\telse\n\t\t*d = NULL;", c_type(typ), c_type(ref));
      else
        fprintf(fout, "\n\tif (*a && (*d = (%s)soap_malloc(soap, sizeof(%s))))\n\t\t**d = soap_wstrdup(soap, **a);\n\telse\n\t\t*d = NULL;", c_type(typ), c_type(ref));
    }
    else if (is_primitive(ref) || is_external(typ) || is_volatile(ref) || is_transient(ref))
    { 
      if (cflag)
        fprintf(fout, "\n\tif (*a && (*d = (%s)soap_malloc(soap, sizeof(%s))))\n\t\t**d = **a;\n\telse\n\t\t*d = NULL;", c_type(typ), c_type(ref));
      else if (is_primitive(ref))
        fprintf(fout, "\n\tif (*a && (*d = (%s)soap_malloc(soap, sizeof(%s))))\n\t\t**d = **a;\n\telse\n\t\t*d = NULL;", c_type(typ), c_type(ref));
      else
        fprintf(fout, "\n\tif (*a && (*d = soap_new_%s(soap)))\n\t\t**d = **a;\n\telse\n\t\t*d = NULL;", c_ident(ref));
      if (!is_primitive(ref) && is_transient(ref))
        fprintf(fout, " /* transient (shallow copy) */");
    }
    else if (ref->type == Tclass || ref->type == Tstruct || ref->type == Ttemplate || ref->type == Tpointer)
    {
      fprintf(fout, "\n\tif (*a)\n\t\t*d = %s_dup_%s(soap, NULL, *a);\n\telse\n\t\t*d = NULL;", fprefix, c_ident(ref));
    }
  }
  else if (is_smart(typ))
  {
    Tnode *ref = typ->ref;
    if (is_smart_shared(typ))
    {
      fprintf(fout, "\n\tif (*a)\n\t{\tstruct soap_plist *pp = NULL;\n\t\tchar *mark = NULL;\n\t\t%s *sp = (%s*)soap_mark_lookup(soap, (const void*)a->get(), %s, &pp, &mark);\n\t\tif (sp)\n\t\t\t*d = *sp;\n\t\telse if (soap_mark_cycle(soap, pp))\n\t\t\treturn d;\n\t\telse\n\t\t{\t", c_type(typ), c_type(typ), soap_type(typ));
      if (ref->type == Tclass && !is_external(ref) && !is_volatile(ref) && !is_typedef(ref) && !is_transient(ref))
        fprintf(fout, "*d = %s((*a)->soap_alloc());\n\t\t\tsoap_mark_dup(soap, (void*)d, pp);\n\t\t\t(*a)->soap_dup(soap, (void*)d->get());", c_type(typ));
      else if (is_primitive(ref) || is_external(ref) || is_volatile(ref) || is_transient(ref))
        fprintf(fout, "*d = %s<%s>(**a);", make_shared(typ), c_type(ref));
      else if (ref->type == Tclass || ref->type == Tstruct || ref->type == Ttemplate || ref->type == Tpointer)
        fprintf(fout, "*d = %s<%s>();\n\t\t\tsoap_mark_dup(soap, (void*)d, pp);\n\t\t\t%s_dup_%s(soap, d->get(), a->get());", make_shared(typ), c_type(ref), fprefix, c_ident(ref));
      fprintf(fout, "\n\t\t\tsoap_unmark(soap, mark);\n\t\t}\n\t}");
    }
    else
    {
      if (ref->type == Tclass && !is_external(ref) && !is_volatile(ref) && !is_typedef(ref) && !is_transient(ref))
        fprintf(fout, "\n\tif (a->get())\n\t\t*d = %s((*a)->soap_dup(soap, (void*)(*a)->soap_alloc()));", c_type(typ));
      else if (is_primitive(ref) || is_transient(ref))
        fprintf(fout, "\n\tif (a->get() && (*d = %s(SOAP_NEW(soap, %s))).get())\n\t\t**d = **a;", c_type(typ), c_type(ref));
      else if (ref->type == Tclass || ref->type == Tstruct || ref->type == Ttemplate || ref->type == Tpointer)
        fprintf(fout, "\n\tif (a->get())\n\t\t*d = %s(%s_dup_%s(soap, SOAP_NEW(soap, %s), a->get()));", c_type(typ), fprefix, c_ident(ref), c_type(ref));
    }
  }
  else if (typ->type == Ttemplate)
  {
    Tnode *ref = typ->ref;
    if (strcmp(typ->id->name, "std::set") || is_primitive(ref) || is_external(ref) || is_volatile(ref) || is_transient(ref))
      fprintf(fout, "\n\t*d = *a;");
    if (!is_primitive(ref) && !is_external(ref) && !is_volatile(ref) && !is_transient(ref))
    {
      if (!strcmp(typ->id->name, "std::set"))
      {
        fprintf(fout, "\n\t%s v;\n\tfor (%s::const_iterator i = a->begin(); i != a->end(); ++i)\n\t{\t", c_type(ref), c_type(typ));
        if (is_XML(ref) && is_string(ref))
          fprintf(fout, "v = soap_strdup(soap, *i);\n\t\td->insert(v);");
        else if (is_XML(ref) && is_wstring(ref))
          fprintf(fout, "v = soap_wstrdup(soap, *i);\n\t\td->insert(v);");
        else if (ref->type == Tclass || ref->type == Tstruct || ref->type == Ttemplate || ref->type == Tpointer)
          fprintf(fout, "%s_dup_%s(soap, &v, &*i);\n\t\td->insert(v);", fprefix, c_ident(ref));
        else
          fprintf(fout, "\n\t\td->insert(*i);");
      }
      else
      {
        fprintf(fout, "\n\t%s::iterator j = d->begin();\n\tfor (%s::const_iterator i = a->begin(); i != a->end(); ++i, ++j)\n\t{\t", c_type(typ), c_type(typ));
        if (is_XML(ref) && is_string(ref))
          fprintf(fout, "*j = soap_strdup(soap, *i);");
        else if (is_XML(ref) && is_wstring(ref))
          fprintf(fout, "*j = soap_wstrdup(soap, *i);");
        else if (ref->type == Tclass || ref->type == Tstruct || ref->type == Ttemplate || ref->type == Tpointer)
          fprintf(fout, "%s_dup_%s(soap, &*j, &*i);", fprefix, c_ident(ref));
      }
      fprintf(fout, "\n\t}");
    }
  }
  else if (typ->ref)
  {
    Entry *p = is_dynamic_array(typ);
    if (p)
    {
      if (is_binary(typ))
      {
        if (cflag)
          fprintf(fout, "\n\td->__ptr = NULL;\n\td->__size = a->__size;\n\tif (a->__ptr && a->__size > 0)\n\t{\td->__ptr = (unsigned char*)soap_malloc(soap, a->__size);\n\t\tsoap_memcpy(d->__ptr, d->__size, a->__ptr, a->__size);\n\t}");
        else
          fprintf(fout, "\n\td->__ptr = NULL;\n\td->__size = a->__size;\n\tif (a->__ptr && a->__size > 0)\n\t{\td->__ptr = (unsigned char*)soap_malloc(soap, a->__size);\n\t\tsoap_memcpy(d->__ptr, d->__size, a->__ptr, a->__size);\n\t}");
        if (is_attachment(typ))
          fprintf(fout, "\n\td->id = soap_strdup(soap, a->id);\n\td->type = soap_strdup(soap, a->type);\n\td->options = soap_strdup(soap, a->options); /* WARNING: cannot copy binary DIME attachment options correctly, MIME/MTOM is OK */");
      }
      else
      {
        Tnode *ref = p->info.typ->ref;
        int dim = get_Darraydims(typ);
        const char *d = ident(p->sym->name);
        fprintf(fout, "\n\tif (a->%s)\n\t{", d);
        if (dim)
          fprintf(fout, "\tsize_t i, n = soap_size(a->__size, %d);", dim);
        else
          fprintf(fout, "\tint i, n = a->__size;");
        if (cflag)
          fprintf(fout, "\n\t\td->%s = (%s)soap_malloc(soap, n * sizeof(%s));", d, c_type(p->info.typ), c_type(ref));
        else if (is_primitive(ref) || ref->type == Tpointer)
          fprintf(fout, "\n\t\td->%s = (%s)soap_malloc(soap, n * sizeof(%s));", d, c_type(p->info.typ), c_type(ref));
        else
          fprintf(fout, "\n\t\td->%s = soap_new_%s(soap, n);", d, c_ident(ref));
        fprintf(fout, "\n\t\tfor (i = 0; i < n; i++)");
        if (is_XML(ref) && is_string(ref))
          fprintf(fout, "\n\t\t\td->%s[i] = soap_strdup(soap, a->%s[i]);", d, d);
        else if (is_XML(ref) && is_wstring(ref))
          fprintf(fout, "\n\t\t\td->%s[i] = soap_wstrdup(soap, a->%s[i]);", d, d);
        else if (is_primitive(ref) || is_transient(ref))
          fprintf(fout, "\n\t\t\td->%s[i] = a->%s[i];", d, d);
        else if (ref->type == Tclass || ref->type == Tstruct || ref->type == Ttemplate || ref->type == Tpointer)
          fprintf(fout, "\n\t\t\t%s_dup_%s(soap, &d->%s[i], &a->%s[i]);", fprefix, c_ident(ref), d, d);
        fprintf(fout, "\n\t}\n\telse\n\t\td->%s = NULL;", d);
        if (dim)
          fprintf(fout, "\n\tsoap_memcpy(&d->__size, sizeof(d->__size), &a->__size, sizeof(a->__size));");
        else
          fprintf(fout, "\n\td->__size = a->__size;");
        if (has_offset(typ))
        {
          if (dim)
            fprintf(fout, "\n\tsoap_memcpy(&d->__offset, sizeof(d->__offset), &a->__offset, sizeof(a->__offset));");
          else
            fprintf(fout, "\n\td->__offset = a->__offset;");
        }
      }
    }
    else
    {
      Table *t = (Table*)typ->ref;
      const char *b = "";
      const char *c = "";
      if (typ->type == Tclass)
      {
        b = ident(t->sym->name);
        c = "::";
      }
      if (t->prev)
        fprintf(fout, "\n\t%s_dup_%s(soap, d, a);", fprefix, ident(t->prev->sym->name));
      for (p = ((Table*)typ->ref)->list; p; p = p->next)
      {
        const char *d = ident(p->sym->name);
        if (p->info.typ->type == Tfun || p->info.typ->type == Tunion)
        {
          continue;
        }
        else if (p->info.sto & Sconst)
        {
          fprintf(fout, "\n\t/* const %s skipped */", d);
        }
        else if (p->info.sto & Sstatic)
        {
          fprintf(fout, "\n\t/* static %s skipped */", d);
        }
        else if (p->info.sto & Stypedef)
        {
          fprintf(fout, "\n\t/* typedef %s skipped */", d);
        }
        else if (is_repetition(p))
        {
          Tnode *ref = p->next->info.typ->ref;
          const char *e = ident(p->next->sym->name);
          fprintf(fout, "\n\td->%s%s%s = a->%s%s%s;\n\tif (a->%s%s%s > 0 && a->%s%s%s)", b, c, d, b, c, d, b, c, d, b, c, e);
          if (cflag)
          {
            if (ref->type == Tpointer && (p->next->info.sto & Sconstptr))
              fprintf(fout, "\n\t{\tint i;\n\t\td->%s%s%s = (const %s)soap_malloc(soap, a->%s%s%s * sizeof(%s));", b, c, e, c_type(p->next->info.typ), b, c, d, c_type(ref));
            else
              fprintf(fout, "\n\t{\tint i;\n\t\td->%s%s%s = (%s)soap_malloc(soap, a->%s%s%s * sizeof(%s));", b, c, e, c_type(p->next->info.typ), b, c, d, c_type(ref));
          }
          else if (is_primitive(ref) || ref->type == Tpointer)
          {
            if (ref->type == Tpointer && (p->next->info.sto & Sconstptr))
              fprintf(fout, "\n\t{\tint i;\n\t\td->%s%s%s = (const %s)soap_malloc(soap, a->%s%s%s * sizeof(%s));", b, c, e, c_type(p->next->info.typ), b, c, d, c_type(ref));
            else
              fprintf(fout, "\n\t{\tint i;\n\t\td->%s%s%s = (%s)soap_malloc(soap, a->%s%s%s * sizeof(%s));", b, c, e, c_type(p->next->info.typ), b, c, d, c_type(ref));
          }
          else
          {
            fprintf(fout, "\n\t{\tint i;\n\t\td->%s%s%s = soap_new_%s(soap, a->%s%s%s);", b, c, e, c_ident(ref), b, c, d);
          }
          fprintf(fout, "\n\t\tfor (i = 0; i < (int)a->%s%s%s; i++)", b, c, d);
          if (is_XML(ref) && is_string(ref))
            fprintf(fout, "\n\t\t\td->%s%s%s[i] = soap_strdup(soap, a->%s%s%s[i]);", b, c, e, b, c, e);
          else if (is_XML(ref) && is_wstring(ref))
            fprintf(fout, "\n\t\t\td->%s%s%s[i] = soap_wstrdup(soap, a->%s%s%s[i]);", b, c, e, b, c, e);
          else if (is_primitive(ref))
            fprintf(fout, "\n\t\t\td->%s%s%s[i] = a->%s%s%s[i];", b, c, e, b, c, e);
          else if (ref->type == Tpointer && (p->next->info.sto & Sconstptr))
            fprintf(fout, "\n\t\t\t%s_dup_%s(soap, (%s)&d->%s%s%s[i], (%s)&a->%s%s%s[i]);", fprefix, c_ident(ref), c_type_id(ref, "*"), b, c, e, c_type_id(ref, "const*"), b, c, e);
          else if (ref->type == Tclass || ref->type == Tstruct || ref->type == Ttemplate || ref->type == Tpointer)
            fprintf(fout, "\n\t\t\t%s_dup_%s(soap, &d->%s%s%s[i], &a->%s%s%s[i]);", fprefix, c_ident(ref), b, c, e, b, c, e);
          fprintf(fout, "\n\t}\n\telse\n\t\td->%s%s%s = NULL;", b, c, e);
          p = p->next;
        }
        else if (is_anytype(p))
        {
          fprintf(fout, "\n\td->%s%s%s = soap_dupelement(soap, a->%s%s%s, d->%s%s%s = a->%s%s%s);", b, c, ident(p->next->sym->name), b, c, ident(p->next->sym->name), b, c, d, b, c, d);
          p = p->next;
        }
        else if (is_choice(p))
        {
          Entry *q;
          const char *e = ident(p->next->sym->name);
          fprintf(fout, "\n\td->%s%s%s = a->%s%s%s;", b, c, d, b, c, d);
          fprintf(fout, "\n\tswitch (a->%s%s%s)\n\t{", b, c, d);
          t = (Table*)p->next->info.typ->ref;
          if (t)
          {
            for (q = t->list; q; q = q->next)
            {
              if (q->info.typ->type != Tfun && q->info.typ->type != Tunion)
              {
                const char *f = ident(q->sym->name);
                fprintf(fout, "\n\t\tcase %s:", soap_union_member(p->next->info.typ, q));
                if (is_XML(q->info.typ) && is_string(q->info.typ))
                  fprintf(fout, "\n\t\t\td->%s%s%s.%s = soap_strdup(soap, a->%s%s%s.%s);", b, c, e, f, b, c, e, f);
                else if (is_XML(q->info.typ) && is_wstring(q->info.typ))
                  fprintf(fout, "\n\t\t\td->%s%s%s.%s = soap_wstrdup(soap, a->%s%s%s.%s);", b, c, e, f, b, c, e, f);
                else if (is_primitive(q->info.typ))
                  fprintf(fout, "\n\t\t\td->%s%s%s.%s = a->%s%s%s.%s;", b, c, e, f, b, c, e, f);
                else if (is_transient(q->info.typ))
                  fprintf(fout, "\n\t\t\td->%s%s%s.%s = a->%s%s%s.%s; /* transient (shallow copy) */", b, c, e, f, b, c, e, f);
                else if (q->info.typ->type == Tclass || q->info.typ->type == Tstruct || q->info.typ->type == Ttemplate || q->info.typ->type == Tpointer)
                  fprintf(fout, "\n\t\t\t%s_dup_%s(soap, &d->%s%s%s.%s, &a->%s%s%s.%s);", fprefix, c_ident(q->info.typ), b, c, e, f, b, c, e, f);
                fprintf(fout, "\n\t\t\tbreak;");
              }
            }
          }
          fprintf(fout, "\n\t}");
          p = p->next;
        }
        else if (is_XML(p->info.typ) && is_string(p->info.typ))
        {
          fprintf(fout, "\n\td->%s%s%s = soap_strdup(soap, (char*)a->%s%s%s);", b, c, d, b, c, d);
        }
        else if (is_XML(p->info.typ) && is_wstring(p->info.typ))
        {
          fprintf(fout, "\n\td->%s%s%s = soap_wstrdup(soap, (char*)a->%s%s%s);", b, c, d, b, c, d);
        }
        else if (is_soapref(p->info.typ))
        {
          fprintf(fout, "\n\td->%s%s%s = soap;", b, c, d);
        }
        else if (is_primitive(p->info.typ))
        {
          fprintf(fout, "\n\td->%s%s%s = a->%s%s%s;", b, c, d, b, c, d);
        }
        else if (is_pointer_to_derived(p))
        {
          Tnode *ref = p->info.typ->ref;
          if (ref->type == Tclass && !is_external(ref) && !is_volatile(ref) && !is_typedef(ref))
            fprintf(fout, "\n\td->%s%s%s = a->%s%s%s ? a->%s%s%s->soap_dup(soap) : NULL;", b, c, d, b, c, d, b, c, d);
          else if (ref->type == Tstruct || ref->type == Tclass)
            fprintf(fout, "\n\td->%s%s%s = a->%s%s%s ? %s_dup_%s(soap, NULL, a->%s%s%s) : NULL;", b, c, d, b, c, d, fprefix, c_ident(ref), b, c, d);
          else if (is_primitive(ref))
            fprintf(fout, "\n\tif (a->%s%s%s && (d->%s%s%s = (%s)soap_malloc(soap, sizeof(%s))) != NULL)\n\t\t*d->%s%s%s = *a->%s%s%s;\n\telse\n\t\td->%s%s%s = NULL;", b, c, d, b, c, d, c_type(p->info.typ), c_type(ref), b, c, d, b, c, d, b, c, d);
        }
        else if (is_transient(p->info.typ))
        {
          fprintf(fout, "\n\td->%s%s%s = a->%s%s%s; /* transient (shallow copy) */", b, c, d, b, c, d);
        }
        else if (p->info.typ->type == Tarray)
        {
          int cardinality;
          Tnode *ref = get_item_type(p->info.typ, &cardinality);
          if (cardinality > 1)
          {
            const char *t = c_type(ref);
            fprintf(fout, "\n\t{\tint i;\n\t\tfor (i = 0; i < %d; i++)", get_dimension_product(p->info.typ));
            if (is_XML(ref) && is_string(ref))
              fprintf(fout, "\n\t\t\t((%s*)(d->%s%s%s))[i] = soap_strdup(soap, ((%s*)(a->%s%s%s))[i]);", t, b, c, d, t, b, c, d);
            else if (is_XML(ref) && is_wstring(ref))
              fprintf(fout, "\n\t\t\t((%s*)(d->%s%s%s))[i] = soap_wstrdup(soap, ((%s*)(a->%s%s%s))[i]);", t, b, c, d, t, b, c, d);
            else if (is_primitive(ref))
              fprintf(fout, "\n\t\t\t((%s*)(d->%s%s%s))[i] = ((%s*)(a->%s%s%s))[i];", t, b, c, d, t, b, c, d);
            else if (is_transient(ref))
              fprintf(fout, "\n\t\t\t((%s*)(d->%s%s%s))[i] = ((%s*)(a->%s%s%s))[i]; /* transient (shallow copy) */", t, b, c, d, t, b, c, d);
            else if (ref->type == Tclass || ref->type == Tstruct || ref->type == Ttemplate || ref->type == Tpointer)
              fprintf(fout, "\n\t\t\t%s_dup_%s(soap, &((%s*)(d->%s%s%s))[i], &((%s*)(a->%s%s%s))[i]);", fprefix, c_ident(ref), t, b, c, d, t, b, c, d);
          }
          else
          {
            fprintf(fout, "\n\t{\tint i;\n\t\tfor (i = 0; i < %d; i++)", get_dimension(p->info.typ));
            if (is_XML(ref) && is_string(ref))
              fprintf(fout, "\n\t\t\td->%s%s%s[i] = soap_strdup(soap, a->%s%s%s[i]);", b, c, d, b, c, d);
            else if (is_XML(ref) && is_wstring(ref))
              fprintf(fout, "\n\t\t\td->%s%s%s[i] = soap_wstrdup(soap, a->%s%s%s[i]);", b, c, d, b, c, d);
            else if (is_primitive(ref))
              fprintf(fout, "\n\t\t\td->%s%s%s[i] = a->%s%s%s[i];", b, c, d, b, c, d);
            else if (is_transient(ref))
              fprintf(fout, "\n\t\t\td->%s%s%s[i] = a->%s%s%s[i]; /* transient (shallow copy) */", b, c, d, b, c, d);
            else if (ref->type == Tclass || ref->type == Tstruct || ref->type == Ttemplate || ref->type == Tpointer)
              fprintf(fout, "\n\t\t\t%s_dup_%s(soap, &d->%s%s%s[i], &a->%s%s%s[i]);", fprefix, c_ident(ref), b, c, d, b, c, d);
          }
          fprintf(fout, "\n\t}");
        }
        else if (p->info.typ->type == Tpointer && (p->info.sto & Sconstptr))
        {
          fprintf(fout, "\n\t%s_dup_%s(soap, (%s)&d->%s%s%s, (%s)&a->%s%s%s);", fprefix, c_ident(p->info.typ), c_type_id(p->info.typ, "*"), b, c, d, c_type_id(p->info.typ, "const*"), b, c, d);
        }
        else if (p->info.typ->type == Tclass || p->info.typ->type == Tstruct || p->info.typ->type == Ttemplate || p->info.typ->type == Tpointer)
        {
          fprintf(fout, "\n\t%s_dup_%s(soap, &d->%s%s%s, &a->%s%s%s);", fprefix, c_ident(p->info.typ), b, c, d, b, c, d);
        }
        else
        {
          fprintf(fout, "\n\t/* %s skipped */", d);
        }
      }
    }
  }
  else
  {
    fprintf(fout, "\n\t*d = *a;");
  }
  if ((typ->type == Tclass || typ->type == Tstruct) && typ->recursive)
    fprintf(fout, "\n\tsoap_unmark(soap, mark);");
  fprintf(fout, "\n\treturn d;\n}");
  fflush(fout);
}

void
soap_del(Tnode *typ)
{
  if (!Edflag)
    return;
  if (is_XML(typ))
    return;
  if (typ->type != Tstruct && typ->type != Tclass && typ->type != Ttemplate && typ->type != Tpointer)
    return;
  if (typ->type != Tclass || !(typ->sym && (is_stdstring(typ) || is_stdwstring(typ)) && is_eq(typ->sym->name, "xsd__QName")) || is_imported(typ))
  {
    if ((is_typedef(typ) && !is_external(typ)) || is_restriction(typ))
    {
      fprintf(fhead, "\n\n#define %s_del_%s %s_del_%s\n", fprefix, c_ident(typ), fprefix, t_ident(typ));
      return;
    }
  }
  if (typ->type == Tstruct && (is_anyType(typ) || is_anyAttribute(typ)))
  {
    if (strcmp(fprefix, "soap"))
      fprintf(fhead, "\n\n#define %s_del_%s soap_del_%s\n", fprefix, c_ident(typ), c_ident(typ));
    return;
  }
  fprintf(fhead, "\n\nSOAP_FMAC1 void SOAP_FMAC2 %s_del_%s(%s);", fprefix, c_ident(typ), c_type_constptr_id(typ, "const*"));
  fprintf(fout, "\n\nSOAP_FMAC1 void SOAP_FMAC2 %s_del_%s(%s)\n{", fprefix, c_ident(typ), c_type_constptr_id(typ, "const*a"));
  fprintf(fout, "\n\tif (!a)\n\t\treturn;");
  fflush(fout);
  if (is_string(typ) || is_wstring(typ))
  {
    if (cflag)
      fprintf(fout, "\n\tif (*a)\n\t\tSOAP_FREE(NULL, *a);");
    else
      fprintf(fout, "\n\tif (*a)\n\t\tSOAP_FREE(NULL, *a);");
  }
  else if (typ->type == Tpointer)
  {
    Tnode *ref = typ->ref;
    if (ref->type == Tclass && !is_external(ref) && !is_volatile(ref) && !is_typedef(ref))
      fprintf(fout, "\n\tif (*a)\n\t{\t(*a)->soap_del();\n\t\tSOAP_DELETE(NULL, *a, %s);\n\t}", c_type(ref));
    else if (is_XML(ref))
    {
      if (cflag)
        fprintf(fout, "\n\tif (*a)\n\t{\tif (**a)\n\t\t\tSOAP_FREE(NULL, **a);\n\t\tSOAP_FREE(NULL, *a);\n\t}");
      else
        fprintf(fout, "\n\tif (*a)\n\t{\tif (**a)\n\t\t\tSOAP_FREE(NULL, **a);\n\t\tSOAP_FREE(NULL, *a);\n\t}");
    }
    else if (is_primitive(ref) || is_external(typ) || is_volatile(ref) || is_transient(ref))
    { 
      if (cflag)
        fprintf(fout, "\n\tif (*a)\n\t\tSOAP_FREE(NULL, *a);");
      else if (is_primitive(ref))
        fprintf(fout, "\n\tif (*a)\n\t\tSOAP_FREE(NULL, *a);");
      else
        fprintf(fout, "\n\tif (*a)\n\t\tSOAP_DELETE(NULL, *a, %s);", c_type(ref));
    }
    else if (cflag)
      fprintf(fout, "\n\tif (*a)\n\t{\t%s_del_%s(*a);\n\t\tSOAP_FREE(NULL, *a);\n\t}", fprefix, c_ident(ref));
    else if (ref->type == Tpointer)
      fprintf(fout, "\n\tif (*a)\n\t{\t%s_del_%s(*a);\n\t\tSOAP_FREE(NULL, *a);\n\t}", fprefix, c_ident(ref));
    else if (ref->type == Tclass || ref->type == Tstruct || ref->type == Ttemplate)
      fprintf(fout, "\n\tif (*a)\n\t{\t%s_del_%s(*a);\n\t\tSOAP_DELETE(NULL, *a, %s);\n\t}", fprefix, c_ident(ref), c_type(ref));
  }
  else if (is_smart(typ))
  {
    Tnode *ref = typ->ref;
    if (is_smart_shared(typ))
    {
      if (ref->type == Tclass && !is_external(ref) && !is_volatile(ref) && !is_typedef(ref) && !is_transient(ref))
        fprintf(fout, "\n\tif (*a)\n\t\t(*a)->soap_del();");
      else if (is_primitive(ref) || is_transient(ref))
        ;
      else if (ref->type == Tclass || ref->type == Tstruct || ref->type == Ttemplate || ref->type == Tpointer)
        fprintf(fout, "\n\tif (*a)\n\t\t%s_del_%s(a->get());", fprefix, c_ident(ref));
    }
    else
    {
      if (ref->type == Tclass && !is_external(ref) && !is_volatile(ref) && !is_typedef(ref) && !is_transient(ref))
        fprintf(fout, "\n\tif (a->get())\n\t\t(*a)->soap_del();");
      else if (is_primitive(ref) || is_transient(ref))
        ;
      else if (ref->type == Tclass || ref->type == Tstruct || ref->type == Ttemplate || ref->type == Tpointer)
        fprintf(fout, "\n\tif (a->get())\n\t\t%s_del_%s(a->get());", fprefix, c_ident(ref));
    }
  }
  else if (typ->type == Ttemplate)
  {
    Tnode *ref = typ->ref;
    if (!is_primitive(ref) && !is_external(typ) && !is_volatile(ref) && !is_transient(ref))
    {
      fprintf(fout, "\n\tfor (%s::const_iterator i = a->begin(); i != a->end(); ++i)", c_type(typ));
      if (is_XML(ref))
        fprintf(fout, "\n\t\tSOAP_FREE(NULL, *i);");
      else if (ref->type == Tclass || ref->type == Tstruct || ref->type == Ttemplate || ref->type == Tpointer)
        fprintf(fout, "\n\t\t%s_del_%s(&*i);", fprefix, c_ident(ref));
      else
        fprintf(fout, "\n\t\t; /* skipped */");
    }
  }
  else if (typ->ref)
  {
    Entry *p = is_dynamic_array(typ);
    if (p)
    {
      if (is_binary(typ))
      {
        if (cflag)
          fprintf(fout, "\n\tif (a->__ptr)\n\t\tSOAP_FREE(NULL, a->__ptr);");
        else
          fprintf(fout, "\n\tif (a->__ptr)\n\t\tSOAP_FREE(NULL, a->__ptr);");
        if (is_attachment(typ))
        {
          if (cflag)
            fprintf(fout, "\n\tif (a->id)\n\t\tSOAP_FREE(NULL, a->id);\n\tif (a->type)\n\t\tSOAP_FREE(NULL, a->type);\n\tif (a->options)\n\t\tSOAP_FREE(NULL, a->options);");
          else
            fprintf(fout, "\n\tif (a->id)\n\t\tSOAP_FREE(NULL, a->id);\n\tif (a->type)\n\t\tSOAP_FREE(NULL, a->type);\n\tif (a->options)\n\t\tSOAP_FREE(NULL, a->options);");
        }
      }
      else
      {
        Tnode *ref = p->info.typ->ref;
        int dim = get_Darraydims(typ);
        const char *d = ident(p->sym->name);
        fprintf(fout, "\n\tif (a->%s)\n\t{", d);
        if (!is_primitive(ref) && !is_transient(ref))
        {
          if (dim)
            fprintf(fout, "\tsize_t i, n = soap_size(a->__size, %d);", dim);
          else
            fprintf(fout, "\tint i, n = a->__size;");
          fprintf(fout, "\n\t\tfor (i = 0; i < n; i++)");
          if (is_XML(ref))
            fprintf(fout, "\n\t\t\tSOAP_FREE(NULL, a->%s[i]);", d);
          else if (ref->type == Tclass || ref->type == Tstruct || ref->type == Ttemplate || ref->type == Tpointer)
            fprintf(fout, "\n\t\t\t%s_del_%s(&a->%s[i]);", fprefix, c_ident(ref), d);
        }
        if (cflag || is_primitive(ref) || ref->type == Tpointer)
          fprintf(fout, "\n\t\tSOAP_FREE(NULL, a->%s);\n\t}", d);
        else
          fprintf(fout, "\n\t\tSOAP_DELETE_ARRAY(NULL, a->%s, %s);\n\t}", d, c_type(ref));
      }
    }
    else
    {
      Table *t = (Table*)typ->ref;
      const char *b = "";
      const char *c = "";
      if (typ->type == Tclass)
      {
        b = ident(t->sym->name);
        c = "::";
      }
      if (t->prev)
        fprintf(fout, "\n\t%s_del_%s(a);", fprefix, ident(t->prev->sym->name));
      for (p = ((Table*)typ->ref)->list; p; p = p->next)
      {
        const char *d = ident(p->sym->name);
        if (p->info.typ->type == Tfun || p->info.typ->type == Tunion || is_soapref(p->info.typ))
        {
          continue;
        }
        else if (p->info.sto & Sconst)
        {
          fprintf(fout, "\n\t/* const %s skipped */", d);
        }
        else if (p->info.sto & Sstatic)
        {
          fprintf(fout, "\n\t/* static %s skipped */", d);
        }
        else if (p->info.sto & Stypedef)
        {
          fprintf(fout, "\n\t/* typedef %s skipped */", d);
        }
        else if (is_repetition(p))
        {
          Tnode *ref = p->next->info.typ->ref;
          const char *e = ident(p->next->sym->name);
          fprintf(fout, "\n\tif (a->%s%s%s)\n\t{", b, c, e);
          if (!is_primitive(ref) && !is_transient(ref))
          {
            fprintf(fout, "\tint i;\n\t\tfor (i = 0; i < (int)a->%s%s%s; i++)", b, c, d);
            if (is_XML(ref))
              fprintf(fout, "\n\t\t\tSOAP_FREE(NULL, a->%s%s%s[i]);", b, c, e);
            else if (ref->type == Tpointer && (p->next->info.sto & Sconstptr))
              fprintf(fout, "\n\t\t\t%s_del_%s((%s)&a->%s%s%s[i]);", fprefix, c_ident(ref), c_type_id(ref, "*"), b, c, e);
            else if (ref->type == Tclass || ref->type == Tstruct || ref->type == Ttemplate || ref->type == Tpointer)
              fprintf(fout, "\n\t\t\t%s_del_%s(&a->%s%s%s[i]);", fprefix, c_ident(ref), b, c, e);
          }
          if (cflag)
            fprintf(fout, "\n\t\tSOAP_FREE(NULL, a->%s%s%s);\n\t}", b, c, e);
          else if (is_primitive(ref) || ref->type == Tpointer)
            fprintf(fout, "\n\t\tSOAP_FREE(NULL, a->%s%s%s);\n\t}", b, c, e);
          else
            fprintf(fout, "\n\t\tSOAP_DELETE_ARRAY(soap, a->%s%s%s, %s);\n\t}", b, c, e, c_type(ref));
          p = p->next;
        }
        else if (is_anytype(p))
        {
          fprintf(fout, "\n\tsoap_delelement(a->%s%s%s, a->%s%s%s);", b, c, ident(p->next->sym->name), b, c, d);
          p = p->next;
        }
        else if (is_choice(p))
        {
          Entry *q;
          const char *e = ident(p->next->sym->name);
          fprintf(fout, "\n\tswitch (a->%s%s%s)\n\t{", b, c, d);
          t = (Table*)p->next->info.typ->ref;
          if (t)
          {
            for (q = t->list; q; q = q->next)
            {
              if (!is_primitive(q->info.typ) && !is_transient(q->info.typ))
              {
                const char *f = ident(q->sym->name);
                fprintf(fout, "\n\t\tcase %s:", soap_union_member(p->next->info.typ, q));
                if (is_XML(q->info.typ))
                  fprintf(fout, "\n\t\t\tif (a->%s%s%s.%s)\n\t\t\t\tSOAP_FREE(NULL, a->%s%s%s.%s);", b, c, e, f, b, c, e, f);
                else if (q->info.typ->type == Tclass || q->info.typ->type == Tstruct || q->info.typ->type == Ttemplate || q->info.typ->type == Tpointer)
                  fprintf(fout, "\n\t\t\t%s_del_%s(&a->%s%s%s.%s);", fprefix, c_ident(q->info.typ), b, c, e, f);
                fprintf(fout, "\n\t\t\tbreak;");
              }
            }
          }
          fprintf(fout, "\n\t}");
          p = p->next;
        }
        else if (is_XML(p->info.typ))
        {
          fprintf(fout, "\n\tif (a->%s%s%s)\n\t\tSOAP_FREE(NULL, a->%s%s%s);", b, c, d, b, c, d);
        }
        else if (p->info.typ->type == Tpointer && (p->info.sto & Sconstptr))
        {
          fprintf(fout, "\n\t%s_del_%s((%s)&a->%s%s%s);", fprefix, c_ident(p->info.typ), c_type_id(p->info.typ, "*"), b, c, d);
        }
        else if (is_primitive(p->info.typ))
        {
          fprintf(fout, "\n\t/* %s skipped */", d);
        }
        else if (is_pointer_to_derived(p))
        {
          Tnode *ref = p->info.typ->ref;
          if (ref->type == Tclass && !is_external(ref) && !is_volatile(ref) && !is_typedef(ref))
            fprintf(fout, "\n\tif (a->%s%s%s)\n\t{\ta->%s%s%s->soap_del();\n\t\tSOAP_DELETE(NULL, a->%s%s%s, %s);\n\t}", b, c, d, b, c, d, b, c, d, c_type(ref));
          else if (ref->type == Tstruct || ref->type == Tclass)
            fprintf(fout, "\n\tif (a->%s%s%s)\n\t{\t%s_del_%s(a->%s%s%s);\n\t\tSOAP_FREE(NULL, a->%s%s%s);\n\t}", b, c, d, fprefix, c_ident(ref), b, c, d, b, c, d);
          else if (is_primitive(ref))
            fprintf(fout, "\n\tif (a->%s%s%s)\n\t\tSOAP_FREE(NULL, a->%s%s%s);", b, c, d, b, c, d);
        }
        else if (is_transient(p->info.typ))
        {
          fprintf(fout, "\n\t/* transient %s skipped */", d);
        }
        else if (p->info.typ->type == Tarray)
        {
          int cardinality;
          Tnode *ref = get_item_type(p->info.typ, &cardinality);
          if (!is_primitive(ref) && !is_transient(ref))
          {
            if (cardinality > 1)
            {
              const char *t = c_type(ref);
              fprintf(fout, "\n\t{\tint i;\n\t\tfor (i = 0; i < %d; i++)", get_dimension_product(p->info.typ));
              if (is_XML(ref))
                fprintf(fout, "\n\t\t\tif (((%s*)(a->%s%s%s))[i])\n\t\t\t\tSOAP_FREE(NULL, ((%s*)(a->%s%s%s))[i]);", t, b, c, d, t, b, c, d);
              else if (ref->type == Tclass || ref->type == Tstruct || ref->type == Ttemplate || ref->type == Tpointer)
                fprintf(fout, "\n\t\t\t%s_del_%s(&((%s*)(a->%s%s%s))[i]);", fprefix, c_ident(ref), t, b, c, d);
            }
            else
            {
              fprintf(fout, "\n\t{\tint i;\n\t\tfor (i = 0; i < %d; i++)", get_dimension(p->info.typ));
              if (is_XML(ref))
                fprintf(fout, "\n\t\t\tif (a->%s%s%s[i])\n\t\t\t\tSOAP_FREE(NULL, a->%s%s%s[i]);", b, c, d, b, c, d);
              else if (ref->type == Tclass || ref->type == Tstruct || ref->type == Ttemplate || ref->type == Tpointer)
                fprintf(fout, "\n\t\t\t%s_del_%s(&a->%s%s%s[i]);", fprefix, c_ident(ref), b, c, d);
            }
            fprintf(fout, "\n\t}");
          }
        }
        else if (p->info.typ->type == Tclass || p->info.typ->type == Tstruct || p->info.typ->type == Ttemplate || p->info.typ->type == Tpointer)
        {
          fprintf(fout, "\n\t%s_del_%s(&a->%s%s%s);", fprefix, c_ident(p->info.typ), b, c, d);
        }
        else
        {
          fprintf(fout, "\n\t/* %s skipped */", d);
        }
      }
    }
  }
  fprintf(fout, "\n}");
  fflush(fout);
}

int
get_dimension(Tnode *typ)
{
  if (((Tnode*)typ->ref)->width)
    return typ->width / ((Tnode*) typ->ref)->width;
  return 0;
}

int
get_dimension_product(Tnode *typ)
{
  int total = 1;
  while (typ->type == Tarray)
  {
    total *= get_dimension(typ);
    typ = (Tnode*)typ->ref;
  }
  return total;
}

Tnode *
get_item_type(Tnode *typ, int *depth)
{
  *depth = 0;
  while (typ->type == Tarray)
  {
    typ = (Tnode*)typ->ref;
    (*depth)++;
  }
  return typ;
}

void
soap_serialize(Tnode *typ)
{
  int d;
  Table *table, *t;
  Entry *p;
  Tnode* temp;
  int cardinality;
  const char *self;

  if (is_XML(typ))
    return;

  if (is_primitive(typ))
    return;

  if (is_typedef(typ) && (is_template(typ) || is_element(typ) || is_restriction(typ) || is_external(typ) || is_imported(typ)) && (!is_external(typ) || is_volatile(typ)))
  {
    if (typ->type == Tclass && !is_stdstring(typ) && !is_stdwstring(typ) && !is_volatile(typ))
      fprintf(fhead, "\n\n#define soap_serialize_%s(soap, a) (a)->soap_serialize(soap)\n", c_ident(typ));
    else
    {
      if (typ->type == Tstruct && is_element(typ)) /* don't permit for typedef'd elements, see soap_put() */
        fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_serialize_%s(struct soap*, const %s);", t_ident(typ), c_type_id(typ, "*"));
      fprintf(fhead, "\n\n#define soap_serialize_%s soap_serialize_%s\n", c_ident(typ), t_ident(typ));
    }
    return;
  }
  if ((p = is_dynamic_array(typ)))
  {
    if (typ->type == Tclass && !is_typedef(typ) && !is_volatile(typ))
    {
      if (is_external(typ))
        return;
      fprintf(fout, "\n\nvoid %s::soap_serialize(struct soap *soap) const\n{", c_ident(typ));
      fprintf(fout, "\n\t(void)soap; /* appease -Wall -Werror */\n#ifndef WITH_NOIDREF");
      if (is_binary(typ))
      {
        if (is_attachment(typ))
          fprintf(fout, "\n\tif (this->__ptr)\n\t\t(void)soap_attachment_reference(soap, this, this->__ptr, this->__size, %s, this->id, this->type);\n#endif\n}", soap_type(typ));
        else
          fprintf(fout, "\n\tif (this->__ptr)\n\t\t(void)soap_array_reference(soap, this, this->__ptr, this->__size, %s);\n#endif\n}", soap_type(typ));
        fflush(fout);
        return;
      }
      else
      {
        d = get_Darraydims(typ);
        if (d)
        {
          fprintf(fout, "\n\tif (this->%s && !soap_array_reference(soap, this, this->%s, %s, %s))", ident(p->sym->name), ident(p->sym->name), get_Darraysize("this", d), soap_type(typ));
          fprintf(fout, "\n\t{\tsize_t i, n = soap_size(this->__size, %d);", d);
          fprintf(fout, "\n\t\tfor (i = 0; i < n; i++)");
        }
        else
        {
          fprintf(fout, "\n\tif (this->%s && !soap_array_reference(soap, this, this->%s, this->__size, %s))", ident(p->sym->name), ident(p->sym->name), soap_type(typ));
          fprintf(fout, "\n\t{\tfor (size_t i = 0; i < (size_t)this->__size; i++)");
        }
        fprintf(fout, "\n\t\t{");
        if (has_ptr((Tnode*)p->info.typ->ref))
          fprintf(fout, "\tsoap_embedded(soap, this->%s + i, %s);", ident(p->sym->name), soap_type((Tnode*)p->info.typ->ref));
        if (((Tnode*)p->info.typ->ref)->type == Tclass && !is_XML((Tnode*)p->info.typ->ref) && !is_external((Tnode*)p->info.typ->ref) && !is_volatile((Tnode*)p->info.typ->ref) && !is_typedef((Tnode*)p->info.typ->ref))
          fprintf(fout, "\n\t\t\tthis->%s[i].soap_serialize(soap);", ident(p->sym->name));
        else if (is_string((Tnode*)p->info.typ->ref) && !is_XML((Tnode*)p->info.typ->ref))
          fprintf(fout, "\n\t\t\tsoap_serialize_%s(soap, (char*const*)(this->%s + i));", c_ident((Tnode*)p->info.typ->ref), ident(p->sym->name));
        else if (is_wstring((Tnode*)p->info.typ->ref) && !is_XML((Tnode*)p->info.typ->ref))
          fprintf(fout, "\n\t\t\tsoap_serialize_%s(soap, (wchar_t*const*)(this->%s + i));", c_ident((Tnode*)p->info.typ->ref), ident(p->sym->name));
        else if (!is_XML((Tnode*)p->info.typ->ref) && !is_primitive((Tnode*)p->info.typ->ref))
          fprintf(fout, "\n\t\t\tsoap_serialize_%s(soap, this->%s + i);", c_ident((Tnode*)p->info.typ->ref), ident(p->sym->name));
        fprintf(fout, "\n\t\t}\n\t}\n#endif\n}");
        return;
      }
    }
    else
    {
      if (is_external(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_serialize_%s(struct soap*, const %s);", c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_serialize_%s(struct soap*, const %s);", c_ident(typ), c_type_id(typ, "*"));
      fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_serialize_%s(struct soap *soap, const %s)\n{", c_ident(typ), c_type_id(typ, "*a"));
      if (is_binary(typ))
      {
        fprintf(fout, "\n\t(void)soap; (void)a; /* appease -Wall -Werror */\n#ifndef WITH_NOIDREF");
        if (is_attachment(typ))
          fprintf(fout, "\n\tif (a->%s)\n\t\t(void)soap_attachment_reference(soap, a, a->%s, a->__size, %s, a->id, a->type);\n#endif\n}", ident(p->sym->name), ident(p->sym->name), soap_type(typ));
        else
          fprintf(fout, "\n\tif (a->%s)\n\t\t(void)soap_array_reference(soap, a, a->%s, a->__size, %s);\n#endif\n}", ident(p->sym->name), ident(p->sym->name), soap_type(typ));
        fflush(fout);
        return;
      }
      else
      {
        fprintf(fout, "\n\t(void)soap; (void)a; /* appease -Wall -Werror */\n#ifndef WITH_NOIDREF");
        d = get_Darraydims(typ);
        if (d)
        {
          fprintf(fout, "\n\tif (a->%s && !soap_array_reference(soap, a, a->%s, %s, %s))", ident(p->sym->name), ident(p->sym->name), get_Darraysize("a", d), soap_type(typ));
          fprintf(fout, "\n\t{\tsize_t i, n = soap_size(a->__size, %d);", d);
          fprintf(fout, "\n\t\tfor (i = 0; i < n; i++)");
        }
        else
        {
          fprintf(fout, "\n\tif (a->%s && !soap_array_reference(soap, a, a->%s, a->__size, %s))", ident(p->sym->name), ident(p->sym->name), soap_type(typ));
          fprintf(fout, "\n\t{\tint i;\n\t\tfor (i = 0; i < a->__size; i++)");
        }
        fprintf(fout, "\n\t\t{");
        if (has_ptr((Tnode*)p->info.typ->ref))
          fprintf(fout, "\tsoap_embedded(soap, a->%s + i, %s);", ident(p->sym->name), soap_type((Tnode*)p->info.typ->ref));
        if (((Tnode*)p->info.typ->ref)->type == Tclass && !is_XML((Tnode*)p->info.typ->ref) && !is_external((Tnode*)p->info.typ->ref) && !is_volatile((Tnode*)p->info.typ->ref) && !is_typedef((Tnode*)p->info.typ->ref))
          fprintf(fout, "\n\t\t\ta->%s[i].soap_serialize(soap);", ident(p->sym->name));
        else if (is_string((Tnode*)p->info.typ->ref) && !is_XML((Tnode*)p->info.typ->ref))
          fprintf(fout, "\n\t\t\tsoap_serialize_%s(soap, (char*const*)(a->%s + i));", c_ident((Tnode*)p->info.typ->ref), ident(p->sym->name));
        else if (is_wstring((Tnode*)p->info.typ->ref) && !is_XML((Tnode*)p->info.typ->ref))
          fprintf(fout, "\n\t\t\tsoap_serialize_%s(soap, (wchar_t*const*)(a->%s + i));", c_ident((Tnode*)p->info.typ->ref), ident(p->sym->name));
        else if (!is_XML((Tnode*)p->info.typ->ref) && !is_primitive((Tnode*)p->info.typ->ref))
          fprintf(fout, "\n\t\t\tsoap_serialize_%s(soap, a->%s + i);", c_ident((Tnode*)p->info.typ->ref), ident(p->sym->name));
        fprintf(fout, "\n\t\t}\n\t}\n#endif\n}");
        fflush(fout);
        return;
      }
    }
  }
  if (is_stdstring(typ) || is_stdwstring(typ))
  {
    fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_serialize_%s(struct soap*, const %s);", c_ident(typ), c_type_id(typ, "*"));
    fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_serialize_%s(struct soap *soap, const %s)\n{\t(void)soap; (void)a; /* appease -Wall -Werror */\n}", c_ident(typ), c_type_id(typ, "*a"));
    return;
  }
  switch(typ->type)
  {
    case Tclass:
      if (!is_volatile(typ) && typ->ref) /* fall through to switch case Tstruct */
      {
        if (is_external(typ))
        {
          fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_serialize_%s(struct soap*, const %s);", c_ident(typ), c_type_id(typ, "*"));
          return;
        }
        if (!is_typedef(typ))
        {
          self = "this";
          fprintf(fout, "\n\nvoid %s::soap_serialize(struct soap *soap) const\n{\n\t(void)soap; /* appease -Wall -Werror */", ident(typ->id->name));
        }
        else
        {
          self = "p";
          fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_serialize_%s(struct soap*, const %s);", c_ident(typ), c_type_id(typ, "*"));
          fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_serialize_%s(struct soap *soap, const %s)\n{\n\t(void)soap; (void)p; /* appease -Wall -Werror */", c_ident(typ), c_type_id(typ, "*p"));
        }
        fprintf(fout, "\n#ifndef WITH_NOIDREF");
        table = (Table*)typ->ref;
        if (table && !is_invisible(typ->id->name))
        {
          for (p = table->list; p; p = p->next)
          {
            if (is_pointer_to_derived(p))
            {
              Tnode *ref = p->info.typ->ref;
              if (ref->type == Tclass && !is_external(ref) && !is_volatile(ref) && !is_typedef(ref))
                fprintf(fout, "\n\tif (this->%s)\n\t{\tthis->%s->soap_serialize(soap);\n\t\treturn;\n\t}", ident(p->sym->name), ident(p->sym->name));
              else if (!is_primitive_or_string(ref))
                fprintf(fout, "\n\tif (this->%s)\n\t{\tsoap_serialize_%s(soap, this->%s);\n\t\treturn;\n\t}", ident(p->sym->name), c_ident(ref), ident(p->sym->name));
              else
                fprintf(fout, "\n\tif (this->%s)\n\t\treturn;", ident(p->sym->name));
            }
          }
        }
        for (p = table->list; p; p = p->next)
        {
          if (p->info.typ->type == Tfun || p->info.typ->type == Tunion || is_soapref(p->info.typ))
            ;
          else if (p->info.sto & (Sconst | Sprivate | Sprotected))
            fprintf(fout, "\n\t/* non-serializable %s skipped */", ident(p->sym->name));
          else if ((p->info.sto & Sattribute))
            ;
          else if (is_repetition(p))
          {
            if (!is_XML(p->next->info.typ))
            {
              fprintf(fout, "\n\tif (%s->%s::%s)", self, ident(table->sym->name), ident(p->next->sym->name));
              fprintf(fout, "\n\t{\tint i;\n\t\tfor (i = 0; i < (int)%s->%s::%s; i++)\n\t\t{", self, ident(table->sym->name), ident(p->sym->name));
              if (!is_invisible(p->next->sym->name))
                if (has_ptr((Tnode*)p->next->info.typ->ref))
                  fprintf(fout, "\n\t\t\tsoap_embedded(soap, %s->%s::%s + i, %s);", self, ident(table->sym->name), ident(p->next->sym->name), soap_type((Tnode*)p->next->info.typ->ref));
              if (((Tnode*)p->next->info.typ->ref)->type == Tclass && !is_external((Tnode*)p->next->info.typ->ref) && !is_volatile((Tnode*)p->next->info.typ->ref) && !is_typedef((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\t%s->%s::%s[i].soap_serialize(soap);", self, ident(table->sym->name), ident(p->next->sym->name));
              else if (is_string((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\tsoap_serialize_%s(soap, (char*const*)(%s->%s::%s + i));", c_ident((Tnode*)p->next->info.typ->ref), self, ident(table->sym->name), ident(p->next->sym->name));
              else if (is_wstring((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\tsoap_serialize_%s(soap, (wchar_t*const*)(%s->%s::%s + i));", c_ident((Tnode*)p->next->info.typ->ref), self, ident(table->sym->name), ident(p->next->sym->name));
              else if (!is_primitive((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\tsoap_serialize_%s(soap, %s->%s::%s + i);", c_ident((Tnode*)p->next->info.typ->ref), self, ident(table->sym->name), ident(p->next->sym->name));
              fprintf(fout, "\n\t\t}\n\t}");
            }
            p = p->next;
          }
          else if (is_anytype(p))
          {
            fprintf(fout, "\n\tsoap_markelement(soap, %s->%s, %s->%s);", self, ident(p->next->sym->name), self, ident(p->sym->name));
            p = p->next;
          }
          else if (is_choice(p))
          {
            fprintf(fout, "\n\tsoap_serialize_%s(soap, %s->%s::%s, &%s->%s::%s);", c_ident(p->next->info.typ), self, ident(table->sym->name), ident(p->sym->name), self, ident(table->sym->name), ident(p->next->sym->name));
            p = p->next;
          }
          else if (is_transient(p->info.typ))
          {
            if (!is_pointer_to_derived(p))
              fprintf(fout, "\n\t/* transient %s skipped */", ident(p->sym->name));
          }
          else if (p->info.typ->type == Tarray)
          {
            if (has_ptr(p->info.typ))
              fprintf(fout, "\n\tsoap_embedded(soap, %s->%s::%s, %s);", self, ident(table->sym->name), ident(p->sym->name), soap_type(p->info.typ));
            fprintf(fout, "\n\tsoap_serialize_%s(soap, %s->%s::%s);", c_ident(p->info.typ), self, ident(table->sym->name), ident(p->sym->name));
          }
          else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
          {
            if (has_ptr(p->info.typ))
              fprintf(fout, "\n\tsoap_embedded(soap, &%s->%s::%s, %s);", self, ident(table->sym->name), ident(p->sym->name), soap_type(p->info.typ));
            fprintf(fout, "\n\t%s->%s::%s.soap_serialize(soap);", self, ident(table->sym->name), ident(p->sym->name));
          }
          else if (!is_void(p->info.typ) && !is_XML(p->info.typ))
          {
            if (!is_template(p->info.typ))
              if (has_ptr(p->info.typ))
                fprintf(fout, "\n\tsoap_embedded(soap, &%s->%s::%s, %s);", self, ident(table->sym->name), ident(p->sym->name), soap_type(p->info.typ));
            if (is_string(p->info.typ))
              fprintf(fout, "\n\tsoap_serialize_%s(soap, (char*const*)&%s->%s::%s);", c_ident(p->info.typ), self, ident(table->sym->name), ident(p->sym->name));
            else if (is_wstring(p->info.typ))
              fprintf(fout, "\n\tsoap_serialize_%s(soap, (wchar_t*const*)&%s->%s::%s);", c_ident(p->info.typ), self, ident(table->sym->name), ident(p->sym->name));
            else if ((p->info.typ->type == Treference || p->info.typ->type == Trvalueref) && ((Tnode*)(p->info.typ->ref))->type == Tclass && !is_external(p->info.typ->ref) && !is_volatile(p->info.typ->ref))
              fprintf(fout, "\n\t%s->%s::%s.soap_serialize(soap);", self, ident(table->sym->name), ident(p->sym->name));
            else if ((p->info.typ->type == Treference || p->info.typ->type == Trvalueref) && !is_primitive(p->info.typ->ref))
              fprintf(fout, "\n\tsoap_serialize_%s(soap, &%s->%s::%s);", c_ident(p->info.typ), self, ident(table->sym->name), ident(p->sym->name));
            else if (!is_primitive(p->info.typ) && p->info.typ->type != Treference && p->info.typ->type != Trvalueref)
              fprintf(fout, "\n\tsoap_serialize_%s(soap, &%s->%s::%s);", c_ident(p->info.typ), self, ident(table->sym->name), ident(p->sym->name));
            else if (!is_primitive(p->info.typ))
              fprintf(fout, "\n\t/* %s skipped */", ident(p->sym->name));
          }
        }
        if (table && table->prev)
          fprintf(fout, "\n\t%s->%s::soap_serialize(soap);", self, ident(table->prev->sym->name));
        fprintf(fout, "\n#endif\n}");
        break;
      }
      /* fall through to next case when class is volatile, since serializers cannot be member functions */
    case Tstruct:
      if (is_external(typ) && !is_volatile(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_serialize_%s(struct soap*, const %s);", c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_serialize_%s(struct soap*, const %s);", c_ident(typ), c_type_id(typ, "*"));
      fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_serialize_%s(struct soap *soap, const %s)\n{", c_ident(typ), c_type_id(typ, "*a"));
      fprintf(fout, "\n\t(void)soap; (void)a; /* appease -Wall -Werror */\n#ifndef WITH_NOIDREF");
      table = (Table*)typ->ref;
      if (table && !is_invisible(typ->id->name))
      {
        for (p = table->list; p; p = p->next)
        {
          if (is_pointer_to_derived(p))
          {
            Tnode *ref = p->info.typ->ref;
            if (ref->type == Tclass && !is_external(ref) && !is_volatile(ref) && !is_typedef(ref))
              fprintf(fout, "\n\tif (a->%s)\n\t{\ta->%s->soap_serialize(soap);\n\t\treturn;\n\t}", ident(p->sym->name), ident(p->sym->name));
            else if (!is_primitive_or_string(ref))
              fprintf(fout, "\n\tif (a->%s)\n\t{\tsoap_serialize_%s(soap, a->%s);\n\t\treturn;\n\t}", ident(p->sym->name), c_ident(ref), ident(p->sym->name));
            else
              fprintf(fout, "\n\tif (a->%s)\n\t\treturn;", ident(p->sym->name));
          }
        }
      }
      for (t = table; t; t = t->prev)
      {
        for (p = t->list; p; p = p->next)
        {
          if (p->info.typ->type == Tfun || p->info.typ->type == Tunion || is_soapref(p->info.typ))
            ;
          else if (p->info.sto & (Sconst | Sprivate | Sprotected))
            fprintf(fout, "\n\t/* non-serializable %s skipped */", ident(p->sym->name));
          else if (p->info.sto & Sattribute)
            ;
          else if (is_repetition(p))
          {
            if (!is_XML(p->next->info.typ))
            {
              fprintf(fout, "\n\tif (a->%s)", ident(p->next->sym->name));
              fprintf(fout, "\n\t{\tint i;\n\t\tfor (i = 0; i < (int)a->%s; i++)\n\t\t{", ident(p->sym->name));
              if (!is_invisible(p->next->sym->name))
                if (has_ptr((Tnode*)p->next->info.typ->ref))
                  fprintf(fout, "\n\t\t\tsoap_embedded(soap, a->%s + i, %s);", ident(p->next->sym->name), soap_type((Tnode*)p->next->info.typ->ref));
              if (((Tnode*)p->next->info.typ->ref)->type == Tclass && !is_external((Tnode*)p->next->info.typ->ref) && !is_volatile((Tnode*)p->next->info.typ->ref) && !is_typedef((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\ta->%s[i].soap_serialize(soap);", ident(p->next->sym->name));
              else if (is_string((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\tsoap_serialize_%s(soap, (char*const*)(a->%s + i));", c_ident((Tnode*)p->next->info.typ->ref), ident(p->next->sym->name));
              else if (is_wstring((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\tsoap_serialize_%s(soap, (wchar_t*const*)(a->%s + i));", c_ident((Tnode*)p->next->info.typ->ref), ident(p->next->sym->name));
              else if (!is_primitive((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\tsoap_serialize_%s(soap, a->%s + i);", c_ident((Tnode*)p->next->info.typ->ref), ident(p->next->sym->name));
              fprintf(fout, "\n\t\t}\n\t}");
            }
            p = p->next;
          }
          else if (is_anytype(p))
          {
            fprintf(fout, "\n\tsoap_markelement(soap, a->%s, a->%s);", ident(p->next->sym->name), ident(p->sym->name));
            p = p->next;
          }
          else if (is_choice(p))
          {
            fprintf(fout, "\n\tsoap_serialize_%s(soap, a->%s, &a->%s);", c_ident(p->next->info.typ), ident(p->sym->name), ident(p->next->sym->name));
            p = p->next;
          }
          else if (is_transient(p->info.typ))
          {
            if (!is_pointer_to_derived(p))
              fprintf(fout, "\n\t/* transient %s skipped */", ident(p->sym->name));
          }
          else if (p->info.typ->type == Tarray)
          {
            if (has_ptr(p->info.typ))
              fprintf(fout, "\n\tsoap_embedded(soap, a->%s, %s);", ident(p->sym->name), soap_type(p->info.typ));
            fprintf(fout, "\n\tsoap_serialize_%s(soap, a->%s);", c_ident(p->info.typ), ident(p->sym->name));
          }
          else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
          {
            if (has_ptr(p->info.typ))
              fprintf(fout, "\n\tsoap_embedded(soap, &a->%s, %s);", ident(p->sym->name), soap_type(p->info.typ));
            fprintf(fout, "\n\ta->%s.soap_serialize(soap);", ident(p->sym->name));
          }
          else if (!is_void(p->info.typ) && !is_XML(p->info.typ))
          {
            if (!is_template(p->info.typ))
              if (has_ptr(p->info.typ))
                fprintf(fout, "\n\tsoap_embedded(soap, &a->%s, %s);", ident(p->sym->name), soap_type(p->info.typ));
            if (is_string(p->info.typ))
              fprintf(fout, "\n\tsoap_serialize_%s(soap, (char*const*)&a->%s);", c_ident(p->info.typ), ident(p->sym->name));
            else if (is_wstring(p->info.typ))
              fprintf(fout, "\n\tsoap_serialize_%s(soap, (wchar_t*const*)&a->%s);", c_ident(p->info.typ), ident(p->sym->name));
            else if ((p->info.typ->type == Treference || p->info.typ->type == Trvalueref) && ((Tnode*)(p->info.typ->ref))->type == Tclass && !is_external(p->info.typ->ref) && !is_volatile(p->info.typ->ref))
              fprintf(fout, "\n\ta->%s.soap_serialize(soap);", ident(p->sym->name));
            else if ((p->info.typ->type == Treference || p->info.typ->type == Trvalueref) && !is_primitive(p->info.typ->ref))
              fprintf(fout, "\n\tsoap_serialize_%s(soap, &a->%s);", c_ident(p->info.typ), ident(p->sym->name));
            else if (!is_primitive(p->info.typ) && p->info.typ->type != Treference && p->info.typ->type != Trvalueref)
              fprintf(fout, "\n\tsoap_serialize_%s(soap, &a->%s);", c_ident(p->info.typ), ident(p->sym->name));
            else if (!is_primitive(p->info.typ))
              fprintf(fout, "\n\t/* %s skipped */", ident(p->sym->name));
          }
        }
      }
      fprintf(fout, "\n#endif\n}");
      break;
    case Tunion:
      if (is_external(typ) && !is_volatile(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_serialize_%s(struct soap*, int, const %s);", c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_serialize_%s(struct soap*, int, const %s);", c_ident(typ), c_type_id(typ, "*"));
      if (!typ->ref)
        return;
      fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_serialize_%s(struct soap *soap, int choice, const %s)\n{", c_ident(typ), c_type_id(typ, "*a"));
      fprintf(fout, "\n\t(void)soap; (void)choice; (void)a; /* appease -Wall -Werror */\n#ifndef WITH_NOIDREF");
      fprintf(fout, "\n\tswitch (choice)\n\t{");
      table = (Table*)typ->ref;
      for (t = table; t; t = t->prev)
      {
        for (p = t->list; p; p = p->next)
        {
          if (p->info.typ->type == Tfun || p->info.typ->type == Tunion)
            ;
          else if (p->info.sto & (Sconst | Sprivate | Sprotected))
            fprintf(fout, "\n\t/* non-serializable %s skipped */", ident(p->sym->name));
          else if (p->info.sto & Sattribute)
            ;
          else if (is_repetition(p))
            ;
          else if (is_anytype(p))
            ;
          else if (is_transient(p->info.typ))
          {
            if (!is_pointer_to_derived(p))
              fprintf(fout, "\n\t/* transient %s skipped */", ident(p->sym->name));
          }
          else if (p->info.typ->type == Tarray)
          {
            fprintf(fout, "\n\tcase %s:", soap_union_member(typ, p));
            if (has_ptr(p->info.typ))
              fprintf(fout, "\n\t\tsoap_embedded(soap, a->%s, %s);", ident(p->sym->name), soap_type(p->info.typ));
            fprintf(fout, "\n\t\tsoap_serialize_%s(soap, a->%s);", c_ident(p->info.typ), ident(p->sym->name));
            fprintf(fout, "\n\t\tbreak;");
          }
          else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
          {
            fprintf(fout, "\n\tcase %s:", soap_union_member(typ, p));
            if (has_ptr(p->info.typ))
              fprintf(fout, "\n\t\tsoap_embedded(soap, &a->%s, %s);", ident(p->sym->name), soap_type(p->info.typ));
            fprintf(fout, "\n\t\ta->%s.soap_serialize(soap);", ident(p->sym->name));
            fprintf(fout, "\n\t\tbreak;");
          }
          else if (p->info.typ->type != Tfun && !is_void(p->info.typ) && !is_XML(p->info.typ))
          {
            fprintf(fout, "\n\tcase %s:", soap_union_member(typ, p));
            if (has_ptr(p->info.typ))
              fprintf(fout, "\n\t\tsoap_embedded(soap, &a->%s, %s);", ident(p->sym->name), soap_type(p->info.typ));
            if (!is_primitive(p->info.typ))
              fprintf(fout, "\n\t\tsoap_serialize_%s(soap, &a->%s);", c_ident(p->info.typ), ident(p->sym->name));
            fprintf(fout, "\n\t\tbreak;");
          }
        }
      }
      fprintf(fout, "\n\tdefault:\n\t\tbreak;\n\t}\n#endif\n}");
      break;
    case Tpointer:
      if (is_external(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_serialize_%s(struct soap*, %s);", c_ident(typ), c_type_constptr_id(typ, "const*"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_serialize_%s(struct soap*, %s);", c_ident(typ), c_type_constptr_id(typ, "const*"));
      fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_serialize_%s(struct soap *soap, %s)\n{\n\t(void)soap; (void)a; /* appease -Wall -Werror */\n#ifndef WITH_NOIDREF", c_ident(typ), c_type_constptr_id(typ, "const*a"));
      temp = (Tnode*)typ->ref;
      if (!temp)
        return;
      if (is_string(typ) || is_wstring(typ))
        fprintf(fout, "\n\t(void)soap_reference(soap, *a, %s);", soap_type(typ));
      else if (is_primitive(temp))
        fprintf(fout, "\n\t(void)soap_reference(soap, *a, %s);", soap_type(temp));
      else
      {
        if (is_dynamic_array(temp))
          fprintf(fout, "\n\tif (*a)");
        else
          fprintf(fout, "\n\tif (!soap_reference(soap, *a, %s))", soap_type(temp));
        if (temp->type == Tclass && !is_external(temp) && !is_volatile(temp) && !is_typedef(temp))
          fprintf(fout, "\n\t\t(*a)->soap_serialize(soap);");
        else
          fprintf(fout, "\n\t\tsoap_serialize_%s(soap, *a);", c_ident(temp));
      }
      fprintf(fout, "\n#endif\n}");
      break;
    case Tarray:
      if (is_external(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_serialize_%s(struct soap*, %s);", c_ident(typ), c_type_id(typ, "const"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_serialize_%s(struct soap*, %s);", c_ident(typ), c_type_id(typ, "const"));
      fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_serialize_%s(struct soap *soap, %s)\n{", c_ident(typ), c_type_id(typ, "const a"));
      if (!is_primitive((Tnode*)typ->ref))
      {
        fprintf(fout, "\n\tint i;");
        fprintf(fout, "\n\t(void)soap; (void)a; /* appease -Wall -Werror */\n#ifndef WITH_NOIDREF");
        fprintf(fout, "\n\tfor (i = 0; i < %d; i++)", get_dimension(typ));
        (void)get_item_type(typ, &cardinality);
        fprintf(fout, "\n\t{");
        if (has_ptr((Tnode*)typ->ref))
        {
          fprintf(fout, "\tsoap_embedded(soap, a");
          if (cardinality > 1)
            fprintf(fout, "[i]");
          else
            fprintf(fout, "+i");
          fprintf(fout, ", %s);", soap_type((Tnode*)typ->ref));
        }
        if (((Tnode *)typ->ref)->type == Tclass && !is_external((Tnode*)typ->ref) && !is_volatile((Tnode*)typ->ref) && !is_typedef((Tnode*)typ->ref))
        {
          fprintf(fout, "\n\t\ta[i].soap_serialize(soap)");
        }
        else if (!is_primitive((Tnode*)typ->ref))
        {
          fprintf(fout, "\n\t\tsoap_serialize_%s(soap, a", c_ident((Tnode*)typ->ref));
          if (cardinality > 1)
          {
            fprintf(fout, "[i])");
          }
          else
          {
            fprintf(fout, "+i)");
          }
        }
        fprintf(fout, ";\n\t}");
        fprintf(fout, "\n#endif\n}");
      }
      else
        fprintf(fout, "\n\t(void)soap; (void)a; /* appease -Wall -Werror */\n}");
      break;
    case Ttemplate:
      if (is_external(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_serialize_%s(struct soap*, const %s);", c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_serialize_%s(struct soap*, const %s);", c_ident(typ), c_type_id(typ, "*"));
      temp = (Tnode*)typ->ref;
      if (!temp)
        return;
      fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_serialize_%s(struct soap *soap, const %s)\n{\n\t(void)soap; (void)a;/* appease -Wall -Werror */\n#ifndef WITH_NOIDREF", c_ident(typ), c_type_id(typ, "*a"));
      if (!is_XML(temp) && temp->type != Tfun && !is_void(temp))
      {
        if (is_smart(typ))
        {
          if (is_primitive(temp))
            fprintf(fout, "\n\t(void)soap_reference(soap, a->get(), %s);", soap_type(temp));
          else
          {
            if (is_dynamic_array(temp))
              fprintf(fout, "\n\tif (a->get())");
            else
              fprintf(fout, "\n\tif (!soap_reference(soap, a->get(), %s))", soap_type(temp));
            if (temp->type == Tclass && !is_external(temp) && !is_volatile(temp) && !is_typedef(temp))
              fprintf(fout, "\n\t\t(*a)->soap_serialize(soap);");
            else
              fprintf(fout, "\n\t\tsoap_serialize_%s(soap, a->get());", c_ident(temp));
          }
        }
        else if (!is_primitive(temp))
        {
          fprintf(fout, "\n\tfor (%s::const_iterator i = a->begin(); i != a->end(); ++i)", c_type(typ));
          if (temp->type == Tclass && !is_external(temp) && !is_volatile(temp) && !is_typedef(temp))
            fprintf(fout, "\n\t\t(*i).soap_serialize(soap);");
          else
            fprintf(fout, "\n\t\tsoap_serialize_%s(soap, &(*i));", c_ident(temp));
        }
      }
      fprintf(fout, "\n#endif\n}");
    default:
      break;
  }
}

void
soap_default(Tnode* typ)
{
  int i, d;
  Table *table, *t;
  Entry *p;
  const char *s;
  int cardinality;

  if (is_XML(typ))
    return;

  if (typ->type == Tpointer && !is_string(typ) && !is_wstring(typ))
    return;

  if (is_typedef(typ) && (is_template(typ) || is_element(typ) || is_restriction(typ) || is_external(typ) || is_imported(typ)) && (!is_external(typ) || is_volatile(typ)))
  {
    if (typ->type == Tclass && !is_stdstring(typ) && !is_stdwstring(typ) && !is_volatile(typ))
      fprintf(fhead, "\n\n#define soap_default_%s(soap, a) (a)->%s::soap_default(soap)\n", c_ident(typ), t_ident(typ));
    else if (typ->type == Tclass && is_eq(typ->sym->name, "xsd__QName"))
      fprintf(fhead, "\n\n#define soap_default_%s soap_default_std__string\n", c_ident(typ));
    else
      fprintf(fhead, "\n\n#define soap_default_%s soap_default_%s\n", c_ident(typ), t_ident(typ));
    return;
  }
  p = is_dynamic_array(typ);
  if (p)
  {
    if (typ->type == Tclass && !is_typedef(typ) && !is_volatile(typ))
    {
      if (is_external(typ))
        return;
      fprintf(fout, "\n\nvoid %s::soap_default(struct soap *soap)\n{", c_ident(typ));
      if ((s = has_soapref(typ)))
        fprintf(fout, "\n\tthis->%s = soap;", s);
      else
        fprintf(fout, "\n\t(void)soap; /* appease -Wall -Werror */");
      if (is_smart(p->info.typ))
        fprintf(fout, "\n\tthis->%s.reset();", ident(p->sym->name));
      else
        fprintf(fout, "\n\tthis->%s = NULL;", ident(p->sym->name));
      d = get_Darraydims(typ);
      if (d)
      {
        for (i = 0; i < d; i++)
        {
          fprintf(fout, "\n\tthis->__size[%d] = 0;", i);
          if (has_offset(typ) && (((Table*)typ->ref)->list->next->next->info.sto & Sconst) == 0)
            fprintf(fout, "\n\tthis->__offset[%d] = 0;", i);
        }
      }
      else
      {
        fprintf(fout, "\n\tthis->__size = 0;");
        if (has_offset(typ) && (((Table*)typ->ref)->list->next->next->info.sto & Sconst) == 0)
          fprintf(fout, "\n\tthis->__offset = 0;");
      }
      if (is_attachment(typ))
        fprintf(fout, "\n\tthis->id = NULL;\n\tthis->type = NULL;\n\tthis->options = NULL;");
      fprintf(fout, "\n}");
    }
    else
    {
      if (is_external(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_default_%s(struct soap*, %s);", c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_default_%s(struct soap*, %s);", c_ident(typ), c_type_id(typ, "*"));
      fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_default_%s(struct soap *soap, %s)\n{", c_ident(typ), c_type_id(typ, "*a"));
      if ((s = has_soapref(typ)))
        fprintf(fout, "\n\ta->%s = soap;", s);
      else
        fprintf(fout, "\n\t(void)soap; /* appease -Wall -Werror */");
      fprintf(fout, "\n\ta->%s = NULL;", ident(p->sym->name));
      d = get_Darraydims(typ);
      if (d)
      {
        for (i = 0; i < d; i++)
        {
          fprintf(fout, "\n\ta->__size[%d] = 0;", i);
          if (has_offset(typ) && (((Table*)typ->ref)->list->next->next->info.sto & Sconst) == 0)
            fprintf(fout, "\n\ta->__offset[%d] = 0;", i);
        }
      }
      else
      {
        fprintf(fout, "\n\ta->__size = 0;");
        if (has_offset(typ) && (((Table*)typ->ref)->list->next->next->info.sto & Sconst) == 0)
          fprintf(fout, "\n\ta->__offset = 0;");
      }
      if (is_attachment(typ))
        fprintf(fout, "\n\ta->id = NULL;\n\ta->type = NULL;\n\ta->options = NULL;");
      fprintf(fout, "\n}");
    }
    fflush(fout);
    return;
  }
  if (is_primitive(typ) || is_string(typ) || is_wstring(typ))
  {
    if (is_external(typ))
    {
      fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_default_%s(struct soap*, %s);", c_ident(typ), c_type_id(typ, "*"));
      return;
    }
    if (is_imported(typ) && typ->type == Tint && !typ->sym)
      fprintf(fout, "\n\nSOAP_FMAC1 void SOAP_FMAC2 soap_default_%s(struct soap *soap, %s)\n{\n\t(void)soap; /* appease -Wall -Werror */\n#ifdef SOAP_DEFAULT_%s\n\t*a = SOAP_DEFAULT_%s;\n#else\n\t*a = (%s)0;\n#endif\n}", c_ident(typ), c_type_id(typ, "*a"), c_ident(typ), c_ident(typ), c_type(typ));
    else if (cflag)
    {
      fprintf(fhead, "\n\n#ifdef SOAP_DEFAULT_%s\n#define soap_default_%s(soap, a) (*(a) = SOAP_DEFAULT_%s)\n#else\n#define soap_default_%s(soap, a) (*(a) = (%s)0)\n#endif", c_ident(typ), c_ident(typ), c_ident(typ), c_ident(typ), c_type(typ));
    }
    else
      fprintf(fhead, "\n\ninline void soap_default_%s(struct soap *soap, %s)\n{\n\t(void)soap; /* appease -Wall -Werror */\n#ifdef SOAP_DEFAULT_%s\n\t*a = SOAP_DEFAULT_%s;\n#else\n\t*a = (%s)0;\n#endif\n}", c_ident(typ), c_type_id(typ, "*a"), c_ident(typ), c_ident(typ), c_type(typ));
    return;
  }
  if (is_fixedstring(typ))
  {
    fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_default_%s(struct soap*, char[]);", c_ident(typ));
    fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_default_%s(struct soap *soap, char a[])\n{\n\t(void)soap; /* appease -Wall -Werror */\n\ta[0] = '\\0';\n}", c_ident(typ));
    return;
  }
  if (is_stdstring(typ) || is_stdwstring(typ))
  {
    fprintf(fhead, "\n\ninline void soap_default_%s(struct soap *soap, %s)\n{\n\t(void)soap; /* appease -Wall -Werror */\n\tp->erase();\n}", c_ident(typ), c_type_id(typ, "*p"));
    return;
  }
  switch(typ->type)
  {
    case Tclass:
      if (!is_volatile(typ) && !is_typedef(typ) && typ->ref) /* fall through to switch case Tstruct */
      {
        if (is_external(typ))
        {
          fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_default_%s(struct soap*, %s);", c_ident(typ), c_type_id(typ, "*"));
          return;
        }
        table=(Table*)typ->ref;
        fprintf(fout, "\n\nvoid %s::soap_default(struct soap *soap)\n{", ident(typ->id->name));
        if ((s = has_soapref(typ)))
          fprintf(fout, "\n\tthis->%s = soap;", s);
        else
          fprintf(fout, "\n\t(void)soap; /* appease -Wall -Werror */");
        fflush(fout);
        if (table)
        {
          if (table->prev)
            fprintf(fout, "\n\tthis->%s::soap_default(soap);", ident(table->prev->sym->name));
          for (p = table->list; p; p = p->next)
          {
            if (p->info.typ->type == Tfun || p->info.typ->type == Tunion || is_soapref(p->info.typ))
              continue;
            if (p->info.sto & Sconst)
              fprintf(fout, "\n\t/* const %s skipped */", ident(p->sym->name));
            else if (is_choice(p))
            {
              fprintf(fout, "\n\tthis->%s::%s = %d;", ident(table->sym->name), ident(p->sym->name), required_choice(p->next->info.typ));
              p = p->next;
            }
            else if (is_repetition(p) || is_anytype(p))
            {
              fprintf(fout, "\n\tthis->%s::%s = 0;\n\tthis->%s::%s = NULL;", ident(table->sym->name), ident(p->sym->name), ident(table->sym->name), ident(p->next->sym->name));
              p = p->next;
            }
            else
            {
              if (is_fixedstring(p->info.typ))
              {
                if (p->info.hasval)
                  fprintf(fout, "\n\tsoap_strcpy(this->%s::%s, %d, \"%s\");", ident(table->sym->name), ident(p->sym->name), get_dimension(p->info.typ), cstring(p->info.val.s, 0));
                else
                  fprintf(fout, "\n\tthis->%s::%s[0] = '\\0';", ident(table->sym->name), ident(p->sym->name));
              }
              else if (p->info.typ->type == Tarray)
                fprintf(fout, "\n\tsoap_default_%s(soap, this->%s::%s);", c_ident(p->info.typ), ident(table->sym->name), ident(p->sym->name));
              else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ) && !is_transient(p->info.typ))
                fprintf(fout, "\n\tthis->%s::%s.%s::soap_default(soap);", ident(table->sym->name), ident(p->sym->name), c_ident(p->info.typ));
              else if (p->info.hasval)
                fprintf(fout, "\n\tthis->%s::%s%s;", ident(table->sym->name), ident(p->sym->name), c_init(p));
              else if (p->info.typ->type == Tpointer && (!is_string(p->info.typ) || is_XML(p->info.typ) || (p->info.sto & Sconstptr)))
                fprintf(fout, "\n\tthis->%s::%s = NULL;", ident(table->sym->name), ident(p->sym->name));
              else if (is_transient(p->info.typ) || is_void(p->info.typ))
                fprintf(fout, "\n\t/* transient %s skipped */", ident(p->sym->name));
              else if (p->info.typ->type != Treference && p->info.typ->type != Trvalueref)
                fprintf(fout, "\n\tsoap_default_%s(soap, &this->%s::%s);", c_ident(p->info.typ), ident(table->sym->name), ident(p->sym->name));
              else
                fprintf(fout, "\n\t/* %s skipped */", ident(p->sym->name));
            }
          }
        }
        fprintf(fout, "\n}");
        fflush(fout);
        break;
      }
    case Tstruct:
      if (is_external(typ) && !is_volatile(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_default_%s(struct soap*, %s);", c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_default_%s(struct soap*, %s);", c_ident(typ), c_type_id(typ, "*"));
      fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_default_%s(struct soap *soap, %s)\n{", c_ident(typ), c_type_id(typ, "*a"));
      fflush(fout);
      if ((s = has_soapref(typ)))
        fprintf(fout, "\n\ta->%s = soap;", s);
      else
        fprintf(fout, "\n\t(void)soap; (void)a; /* appease -Wall -Werror */");
      table = (Table*)typ->ref;
      for (t = table; t; t = t->prev)
      {
        for (p = t->list; p; p = p->next)
        {
          if (p->info.typ->type == Tfun || p->info.typ->type == Tunion || is_soapref(p->info.typ))
            continue;
          if (p->info.sto & Sconst)
            fprintf(fout, "\n\t/* const %s skipped */", ident(p->sym->name));
          else if (p->info.sto & (Sprivate | Sprotected))
            fprintf(fout, "\n\t/* private/protected %s skipped */", ident(p->sym->name));
          else if (is_choice(p))
          {
            fprintf(fout, "\n\ta->%s = %d;", ident(p->sym->name), required_choice(p->next->info.typ));
            p = p->next;
          }
          else if (is_repetition(p) || is_anytype(p))
          {
            fprintf(fout, "\n\ta->%s = 0;\n\ta->%s = NULL;", ident(p->sym->name), ident(p->next->sym->name));
            p = p->next;
          }
          else
          {
            if (is_fixedstring(p->info.typ))
            {
              if (p->info.hasval)
                fprintf(fout, "\n\tsoap_strcpy(a->%s, %d, \"%s\");", ident(p->sym->name), get_dimension(p->info.typ), cstring(p->info.val.s, 0));
              else
                fprintf(fout, "\n\ta->%s[0] = '\\0';", ident(p->sym->name));
            }
            else if (p->info.typ->type == Tarray)
              fprintf(fout, "\n\tsoap_default_%s(soap, a->%s);", c_ident(p->info.typ), ident(p->sym->name));
            else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ) && !is_transient(p->info.typ))
              fprintf(fout, "\n\ta->%s.%s::soap_default(soap);", ident(p->sym->name), c_ident(p->info.typ));
            else if (p->info.hasval)
              fprintf(fout, "\n\ta->%s%s;", ident(p->sym->name), c_init(p));
            else if (p->info.typ->type == Tpointer && (!is_string(p->info.typ) || is_XML(p->info.typ) || (p->info.sto & Sconstptr)))
              fprintf(fout, "\n\ta->%s = NULL;", ident(p->sym->name));
            else if (is_transient(p->info.typ) || is_void(p->info.typ))
              fprintf(fout, "\n\t/* transient %s skipped */", ident(p->sym->name));
            else if (p->info.typ->type != Treference && p->info.typ->type != Trvalueref)
              fprintf(fout, "\n\tsoap_default_%s(soap, &a->%s);", c_ident(p->info.typ), ident(p->sym->name));
            else
              fprintf(fout, "\n\t/* %s skipped */", ident(p->sym->name));
          }
        }
      }
      fprintf(fout, "\n}");
      fflush(fout);
      break;
    case Tarray:
      if (is_external(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_default_%s(struct soap*, %s);", c_ident(typ), c_type_id(typ, ""));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_default_%s(struct soap*, %s);", c_ident(typ), c_type_id(typ, ""));
      fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_default_%s(struct soap *soap, %s)\n{", c_ident(typ), c_type_id(typ, "a"));
      fprintf(fout, "\n\tint i;");
      fprintf(fout, "\n\t(void)soap; /* appease -Wall -Werror */");
      fprintf(fout, "\n\tfor (i = 0; i < %d; i++)", get_dimension(typ));
      (void)get_item_type(typ, &cardinality);
      if (((Tnode *)typ->ref)->type == Tclass && !is_external((Tnode*)typ->ref) && !is_volatile((Tnode*)typ->ref))
      {
        if (cardinality>1)
          fprintf(fout, "a[i].%s::soap_default(soap)", t_ident((Tnode*)typ->ref));
        else
          fprintf(fout, "(a+i)->soap_default(soap)");
      }
      else if (((Tnode*)typ->ref)->type == Tpointer)
        fprintf(fout, "\n\t\ta[i] = NULL");
      else
      {
        fprintf(fout, "\n\t\tsoap_default_%s(soap, a", c_ident((Tnode*)typ->ref));
        if (cardinality>1)
          fprintf(fout, "[i])");
        else
          fprintf(fout, "+i)");
      }
      fprintf(fout, ";\n}");
      break;
    case Ttemplate:
      if (is_external(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_default_%s(struct soap*, %s);", c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_default_%s(struct soap*, %s);", c_ident(typ), c_type_id(typ, "*"));
      fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_default_%s(struct soap *soap, %s)\n{", c_ident(typ), c_type_id(typ, "*p"));
      fprintf(fout, "\n\t(void)soap; /* appease -Wall -Werror */");
      if (is_smart(typ))
        fprintf(fout, "\n\tp->reset();");
      else
        fprintf(fout, "\n\tp->clear();");
      fprintf(fout, "\n}");
      fflush(fout);
      break;
    default:
      break;
  }
}

void
soap_traverse(Tnode* typ)
{
  int d;
  Table *table, *t;
  Entry *p;
  Tnode* temp;
  int cardinality;
  if (is_primitive_or_string(typ) || is_fixedstring(typ))
  {
    fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_traverse_%s(struct soap*, %s, const char *s, soap_walker p, soap_walker q);", c_ident(typ), c_type_id(typ, "*"));
    fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_traverse_%s(struct soap *soap, %s, const char *s, soap_walker p, soap_walker q)\n{\t(void)soap; (void)q; /* appease -Wall -Werror */", c_ident(typ), c_type_id(typ, "*a"));
    fprintf(fout, "\n\tif (p) p(soap, (void*)a, %s, s, \"%s\");", soap_type(typ), c_type(typ));
    fprintf(fout, "\n\tif (q) q(soap, (void*)a, %s, s, \"%s\");\n}", soap_type(typ), c_type(typ));
    return;
  }
  if (typ->type != Tclass || !(typ->sym && (is_stdstring(typ) || is_stdwstring(typ)) && is_eq(typ->sym->name, "xsd__QName")) || is_external(typ) || is_imported(typ))
    if (is_typedef(typ) && !is_external(typ))
    {
      if (typ->type == Tclass && !is_stdstring(typ) && !is_stdwstring(typ) && !is_volatile(typ))
        fprintf(fhead, "\n\n#define soap_traverse_%s(soap, a, s, p, q) (a)->soap_traverse(soap, s, p, q)\n", c_ident(typ));
      else if (typ->type == Tclass && is_eq(typ->sym->name, "xsd__QName"))
        fprintf(fhead, "\n\n#define soap_traverse_%s(soap, a, s, p, q) soap_traverse_std__string(soap, a, s, p, q)\n", c_ident(typ));
      else
        fprintf(fhead, "\n\n#define soap_traverse_%s(soap, a, s, p, q) soap_traverse_%s(soap, a, s, p, q)\n", c_ident(typ), t_ident(typ));
      return;
    }
  if (is_XML(typ))
  {
    fprintf(fhead, "\n\n#define soap_traverse_%s(soap, a, s, p, q) soap_traverse_%s(soap, a, s, p, q)\n", c_ident(typ), t_ident(typ));
    return;
  }
  if ((p = is_dynamic_array(typ)))
  {
    if (typ->type == Tclass && !is_volatile(typ))
    {
      if (is_external(typ))
        return;
      fprintf(fout, "\n\nvoid %s::soap_traverse(struct soap *soap, const char *s, soap_walker p, soap_walker q)\n{", c_ident(typ));
      if (is_binary(typ))
      {
        if (is_attachment(typ))
          fprintf(fout, "\n\tif (this->%s && soap_attachment_reference(soap, this, this->%s, this->__size, %s, this->id, this->type))\n\t{", ident(p->sym->name), ident(p->sym->name), soap_type(typ));
        else
          fprintf(fout, "\n\tif (this->%s && !soap_array_reference(soap, this, this->%s, this->__size, %s))\n\t{", ident(p->sym->name), ident(p->sym->name), soap_type(typ));
        fprintf(fout, "\n\t\tif (p) p(soap, (void*)this, %s, s, \"%s\");", soap_type(typ), c_type(typ));
        fprintf(fout, "\n\t\tif (p) p(soap, (void*)this->%s, 0, \"%s\", NULL);", ident(p->sym->name), p->sym->name);
        if (is_attachment(typ))
        {
          fprintf(fout, "\n\t\tif (p) p(soap, (void*)this->id, SOAP_TYPE_string, \"id\", NULL);");
          fprintf(fout, "\n\t\tif (q) q(soap, (void*)this->id, SOAP_TYPE_string, \"id\", NULL);");
          fprintf(fout, "\n\t\tif (p) p(soap, (void*)this->type, SOAP_TYPE_string, \"type\", NULL);");
          fprintf(fout, "\n\t\tif (q) q(soap, (void*)this->type, SOAP_TYPE_string, \"type\", NULL);");
          fprintf(fout, "\n\t\tif (p) p(soap, (void*)this->options, 0, \"options\", NULL);");
          fprintf(fout, "\n\t\tif (q) q(soap, (void*)this->options, 0, \"options\", NULL);");
        }
        fprintf(fout, "\n\t\tif (q) q(soap, (void*)this->%s, 0, \"%s\", NULL);", ident(p->sym->name), p->sym->name);
        fprintf(fout, "\n\t\tif (q) q(soap, (void*)this, %s, s, \"%s\");\n\t}\n}", soap_type(typ), c_type(typ));
        fflush(fout);
        return;
      }
      else
      {
        d = get_Darraydims(typ);
        if (d)
        {
          fprintf(fout, "\n\tif (this->%s && !soap_array_reference(soap, this, this->%s, %s, %s))\n\t{", ident(p->sym->name), ident(p->sym->name), get_Darraysize("this", d), soap_type(typ));
          fprintf(fout, "\n\t\tif (p) p(soap, (void*)this, %s, s, \"%s\");", soap_type(typ), c_type(typ));
          fprintf(fout, "\n\t\tfor (size_t i = 0; i < soap_size(this->__size, %d); i++)", d);
        }
        else
        {
          fprintf(fout, "\n\tif (this->%s && !soap_array_reference(soap, this, this->%s, this->__size, %s))\n\t{", ident(p->sym->name), ident(p->sym->name), soap_type(typ));
          fprintf(fout, "\n\t\tif (p) p(soap, (void*)this, %s, s, \"%s\");", soap_type(typ), c_type(typ));
          fprintf(fout, "\n\t\tfor (size_t i = 0; i < (size_t)this->__size; i++)");
        }
        fprintf(fout, "\n\t\t{");
        if (has_ptr((Tnode*)p->info.typ->ref))
          fprintf(fout, "\tsoap_embedded(soap, this->%s + i, %s);", ident(p->sym->name), soap_type((Tnode*)p->info.typ->ref));
        if (((Tnode*)p->info.typ->ref)->type == Tclass && !is_external((Tnode*)p->info.typ->ref) && !is_volatile((Tnode*)p->info.typ->ref) && !is_typedef((Tnode*)p->info.typ->ref))
          fprintf(fout, "\n\t\t\tthis->%s[i].soap_traverse(soap, \"%s\", p, q);", ident(p->sym->name), p->sym->name);
        else
          fprintf(fout, "\n\t\t\tsoap_traverse_%s(soap, this->%s + i, \"%s\", p, q);", c_ident((Tnode*)p->info.typ->ref), ident(p->sym->name), ident(p->sym->name));
        fprintf(fout, "\n\t\t}\n\t\tif (q) q(soap, (void*)this, %s, s, \"%s\");", soap_type(typ), c_type(typ));
        fprintf(fout, "\n\t}\n}");
        return;
      }
    }
    else
    {
      if (is_external(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_traverse_%s(struct soap*, %s, const char *s, soap_walker p, soap_walker q);", c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_traverse_%s(struct soap*, %s, const char *s, soap_walker p, soap_walker q);", c_ident(typ), c_type_id(typ, "*"));
      fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_traverse_%s(struct soap *soap, %s, const char *s, soap_walker p, soap_walker q)\n{", c_ident(typ), c_type_id(typ, "*a"));
      if (is_binary(typ))
      {
        if (is_attachment(typ))
          fprintf(fout, "\n\tif (a->%s && soap_attachment_reference(soap, a, a->%s, a->__size, %s, a->id, a->type))\n\t{", ident(p->sym->name), ident(p->sym->name), soap_type(typ));
        else
          fprintf(fout, "\n\tif (a->%s && !soap_array_reference(soap, a, a->%s, a->__size, %s))\n\t{", ident(p->sym->name), ident(p->sym->name), soap_type(typ));
        fprintf(fout, "\n\t\tif (p) p(soap, (void*)a, %s, s, \"%s\");", soap_type(typ), c_type(typ));
        fprintf(fout, "\n\t\tif (p) p(soap, (void*)a->%s, 0, \"%s\", NULL);", ident(p->sym->name), p->sym->name);
        if (is_attachment(typ))
        {
          fprintf(fout, "\n\t\tif (p) p(soap, (void*)a->id, SOAP_TYPE_string, \"id\", NULL);");
          fprintf(fout, "\n\t\tif (q) q(soap, (void*)a->id, SOAP_TYPE_string, \"id\", NULL);");
          fprintf(fout, "\n\t\tif (p) p(soap, (void*)a->type, SOAP_TYPE_string, \"type\", NULL);");
          fprintf(fout, "\n\t\tif (q) q(soap, (void*)a->type, SOAP_TYPE_string, \"type\", NULL);");
          fprintf(fout, "\n\t\tif (p) p(soap, (void*)a->options, 0, \"options\", NULL);");
          fprintf(fout, "\n\t\tif (q) q(soap, (void*)a->options, 0, \"options\", NULL);");
        }
        fprintf(fout, "\n\t\tif (q) q(soap, (void*)a->%s, 0, \"%s\", NULL);", ident(p->sym->name), p->sym->name);
        fprintf(fout, "\n\t\tif (q) q(soap, (void*)a, %s, s, \"%s\");\n\t}\n}", soap_type(typ), c_type(typ));
        fflush(fout);
        return;
      }
      else
      {
        fprintf(fout, "\n\tsize_t i;");
        d = get_Darraydims(typ);
        if (d)
        {
          fprintf(fout, "\n\tif (a->%s && !soap_array_reference(soap, a, a->%s, %s, %s))\n\t{", ident(p->sym->name), ident(p->sym->name), get_Darraysize("a", d), soap_type(typ));
          fprintf(fout, "\n\t\tif (p) p(soap, (void*)a, %s, s, \"%s\");", soap_type(typ), c_type(typ));
          fprintf(fout, "\n\t\tfor (i = 0; i < soap_size(a->__size, %d); i++)", d);
        }
        else
        {
          fprintf(fout, "\n\tif (a->%s && !soap_array_reference(soap, a, a->%s, a->__size, %s))\n\t{", ident(p->sym->name), ident(p->sym->name), soap_type(typ));
          fprintf(fout, "\n\t\tif (p) p(soap, (void*)a, %s, s, \"%s\");", soap_type(typ), c_type(typ));
          fprintf(fout, "\n\t\tfor (i = 0; i < (size_t)a->__size; i++)");
        }
        fprintf(fout, "\n\t\t{");
        if (has_ptr((Tnode*)p->info.typ->ref))
          fprintf(fout, "\tsoap_embedded(soap, a->%s + i, %s);", ident(p->sym->name), soap_type((Tnode*)p->info.typ->ref));
        if (((Tnode*)p->info.typ->ref)->type == Tclass && !is_external((Tnode*)p->info.typ->ref) && !is_volatile((Tnode*)p->info.typ->ref) && !is_typedef((Tnode*)p->info.typ->ref))
          fprintf(fout, "\n\t\t\ta->%s[i].soap_traverse(soap, \"%s\", p, q);", ident(p->sym->name), p->sym->name);
        else
          fprintf(fout, "\n\t\t\tsoap_traverse_%s(soap, a->%s + i, \"%s\", p, q);", c_ident((Tnode*)p->info.typ->ref), ident(p->sym->name), p->sym->name);
        fprintf(fout, "\n\t\t}\n\t\tif (q) q(soap, (void*)a, %s, s, \"%s\");", soap_type(typ), c_type(typ));
        fprintf(fout, "\n\t}\n}");
        fflush(fout);
        return;
      }
    }
  }
  switch(typ->type)
  {
    case Tclass:
      if (!is_volatile(typ))
      {
        if (is_external(typ))
        {
          fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_traverse_%s(struct soap*, %s, const char *s, soap_walker p, soap_walker q);", c_ident(typ), c_type_id(typ, "*"));
          return;
        }
        table=(Table*)typ->ref;
        fprintf(fout, "\n\nvoid %s::soap_traverse(struct soap *soap, const char *s, soap_walker p, soap_walker q)\n{", ident(typ->id->name));
        fprintf(fout, "\n\t(void)soap; /* appease -Wall -Werror */");
        fprintf(fout, "\n\tif (p) p(soap, (void*)this, %s, s, \"%s\");", soap_type(typ), c_type(typ));
        for (t = table; t; t = t->prev)
        {
          for (p = t->list; p; p = p->next)
          {
            if (p->info.typ->type == Tfun || p->info.typ->type == Tunion || is_soapref(p->info.typ))
              ;
            else if (p->info.sto & (Sconst | Sprivate | Sprotected))
              fprintf(fout, "\n\t/* non-serializable %s skipped */", ident(p->sym->name));
            else if (is_transient(p->info.typ))
              fprintf(fout, "\n\t/* transient %s skipped */", ident(p->sym->name));
            else if (p->info.sto & Sattribute)
              ;
            else if (is_repetition(p))
            {
              fprintf(fout, "\n\tif (this->%s::%s)", ident(t->sym->name), ident(p->next->sym->name));
              fprintf(fout, "\n\t{\tint i;\n\t\tfor (i = 0; i < (int)this->%s::%s; i++)\n\t\t{", ident(t->sym->name), ident(p->sym->name));
              if (!is_invisible(p->next->sym->name))
                if (has_ptr((Tnode*)p->next->info.typ->ref))
                  fprintf(fout, "\n\t\t\tsoap_embedded(soap, this->%s::%s + i, %s);", ident(t->sym->name), ident(p->next->sym->name), soap_type((Tnode*)p->next->info.typ->ref));
              if (((Tnode*)p->next->info.typ->ref)->type == Tclass && !is_external((Tnode*)p->next->info.typ->ref) && !is_volatile((Tnode*)p->next->info.typ->ref) && !is_typedef((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\tthis->%s::%s[i].soap_traverse(soap, \"%s\", p, q);", ident(t->sym->name), ident(p->next->sym->name), p->next->sym->name);
              else
                fprintf(fout, "\n\t\t\tsoap_traverse_%s(soap, this->%s::%s + i, \"%s\", p, q);", c_ident((Tnode*)p->next->info.typ->ref), ident(t->sym->name), ident(p->next->sym->name), p->next->sym->name);
              fprintf(fout, "\n\t\t}\n\t}");
              p = p->next;
            }
            else if (is_anytype(p))
            {
              p = p->next;
            }
            else if (is_choice(p))
            {
              fprintf(fout, "\n\tsoap_traverse_%s(soap, this->%s::%s, &this->%s::%s, \"%s\", p, q);", c_ident(p->next->info.typ), ident(t->sym->name), ident(p->sym->name), ident(t->sym->name), ident(p->next->sym->name), p->next->sym->name);
              p = p->next;
            }
            else if (p->info.typ->type == Tarray)
            {
              if (has_ptr(p->info.typ))
                fprintf(fout, "\n\tsoap_embedded(soap, this->%s::%s, %s);", ident(t->sym->name), ident(p->sym->name), soap_type(p->info.typ));
              fprintf(fout, "\n\tsoap_traverse_%s(soap, this->%s::%s, \"%s\", p, q);", c_ident(p->info.typ), ident(t->sym->name), ident(p->sym->name), p->sym->name);
            }
            else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
            {
              if (has_ptr(p->info.typ))
                fprintf(fout, "\n\tsoap_embedded(soap, &this->%s::%s, %s);", ident(t->sym->name), ident(p->sym->name), soap_type(p->info.typ));
              fprintf(fout, "\n\tthis->%s::%s.soap_traverse(soap, \"%s\", p, q);", ident(t->sym->name), ident(p->sym->name), p->sym->name);
            }
            else if (p->info.typ->type != Tfun && !is_void(p->info.typ))
            {
              if (!is_template(p->info.typ))
                if (has_ptr(p->info.typ))
                  fprintf(fout, "\n\tsoap_embedded(soap, &this->%s::%s, %s);", ident(t->sym->name), ident(p->sym->name), soap_type(p->info.typ));
              fprintf(fout, "\n\tsoap_traverse_%s(soap, &this->%s::%s, \"%s\", p, q);", c_ident(p->info.typ), ident(t->sym->name), ident(p->sym->name), p->sym->name);
            }
          }
        }
        fprintf(fout, "\n\tif (q) q(soap, (void*)this, %s, s, \"%s\");", soap_type(typ), c_type(typ));
        fprintf(fout, "\n}");
        break;
      }
      /* fall through to next case when class is volatile, since serializers cannot be member functions */
    case Tstruct:
      if (is_external(typ) && !is_volatile(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_traverse_%s(struct soap*, %s, const char *s, soap_walker p, soap_walker q);", c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_traverse_%s(struct soap*, %s, const char *s, soap_walker p, soap_walker q);", c_ident(typ), c_type_id(typ, "*"));
      if (!typ->ref)
        return;
      fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_traverse_%s(struct soap *soap, %s, const char *s, soap_walker p, soap_walker q)\n{", c_ident(typ), c_type_id(typ, "*a"));
      fprintf(fout, "\n\tif (p) p(soap, (void*)a, %s, s, \"%s\");", soap_type(typ), c_type(typ));
      table=(Table*)typ->ref;
      for (t = table; t; t = t->prev)
      {
        for (p = t->list; p; p = p->next)
        {
          if (p->info.typ->type == Tfun || p->info.typ->type == Tunion || is_soapref(p->info.typ))
            ;
          else if (p->info.sto & (Sconst | Sprivate | Sprotected))
            fprintf(fout, "\n\t/* non-serializable %s skipped */", ident(p->sym->name));
          else if (is_transient(p->info.typ))
            fprintf(fout, "\n\t/* transient %s skipped */", ident(p->sym->name));
          else if (p->info.sto & Sattribute)
            ;
          else if (is_repetition(p))
          {
            fprintf(fout, "\n\tif (a->%s)", ident(p->next->sym->name));
            fprintf(fout, "\n\t{\tint i;\n\t\tfor (i = 0; i < (int)a->%s; i++)\n\t\t{", ident(p->sym->name));
            if (!is_invisible(p->next->sym->name))
              if (has_ptr((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\tsoap_embedded(soap, a->%s + i, %s);", ident(p->next->sym->name), soap_type((Tnode*)p->next->info.typ->ref));
            if (((Tnode*)p->next->info.typ->ref)->type == Tclass && !is_external((Tnode*)p->next->info.typ->ref) && !is_volatile((Tnode*)p->next->info.typ->ref) && !is_typedef((Tnode*)p->next->info.typ->ref))
              fprintf(fout, "\n\t\t\ta->%s[i].soap_traverse(soap, \"%s\", p, q);", ident(p->next->sym->name), p->next->sym->name);
            else
              fprintf(fout, "\n\t\t\tsoap_traverse_%s(soap, a->%s + i, \"%s\", p, q);", c_ident((Tnode*)p->next->info.typ->ref), ident(p->next->sym->name), p->next->sym->name);
            fprintf(fout, "\n\t\t}\n\t}");
            p = p->next;
          }
          else if (is_anytype(p))
          {
            p = p->next;
          }
          else if (is_choice(p))
          {
            fprintf(fout, "\n\tsoap_traverse_%s(soap, a->%s, &a->%s, \"%s\", p, q);", c_ident(p->next->info.typ), ident(p->sym->name), ident(p->next->sym->name), p->next->sym->name);
            p = p->next;
          }
          else if (p->info.typ->type == Tarray)
          {
            if (has_ptr(p->info.typ))
              fprintf(fout, "\n\tsoap_embedded(soap, a->%s, %s);", ident(p->sym->name), soap_type(p->info.typ));
            fprintf(fout, "\n\tsoap_traverse_%s(soap, a->%s, \"%s\", p, q);", c_ident(p->info.typ), ident(p->sym->name), p->sym->name);
          }
          else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
          {
            if (has_ptr(p->info.typ))
              fprintf(fout, "\n\tsoap_embedded(soap, &a->%s, %s);", ident(p->sym->name), soap_type(p->info.typ));
            fprintf(fout, "\n\ta->%s.soap_traverse(soap, \"%s\", p, q);", ident(p->sym->name), p->sym->name);
          }
          else if (p->info.typ->type != Tfun && !is_void(p->info.typ))
          {
            if (!is_template(p->info.typ))
              if (has_ptr(p->info.typ))
                fprintf(fout, "\n\tsoap_embedded(soap, &a->%s, %s);", ident(p->sym->name), soap_type(p->info.typ));
            fprintf(fout, "\n\tsoap_traverse_%s(soap, &a->%s, \"%s\", p, q);", c_ident(p->info.typ), ident(p->sym->name), p->sym->name);
          }
        }
      }
      fprintf(fout, "\n\tif (q) q(soap, (void*)a, %s, s, \"%s\");", soap_type(typ), c_type(typ));
      fprintf(fout, "\n}");
      break;
    case Tunion:
      if (is_external(typ) && !is_volatile(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_traverse_%s(struct soap*, int, %s, const char *s, soap_walker p, soap_walker q);", c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      table = (Table*)typ->ref;
      fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_traverse_%s(struct soap*, int, %s, const char *s, soap_walker p, soap_walker q);", c_ident(typ), c_type_id(typ, "*"));
      fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_traverse_%s(struct soap *soap, int choice, %s, const char *s, soap_walker p, soap_walker q)\n{", c_ident(typ), c_type_id(typ, "*a"));
      fprintf(fout, "\n\tif (p) p(soap, (void*)a, %s, s, \"%s\");", soap_type(typ), c_type(typ));
      fprintf(fout, "\n\tswitch (choice)\n\t{");
      for (t = table; t; t = t->prev)
      {
        for (p = t->list; p; p = p->next)
        {
          if (p->info.typ->type == Tfun || p->info.typ->type == Tunion)
            ;
          else if (p->info.sto & (Sconst | Sprivate | Sprotected))
            fprintf(fout, "\n\t/* non-serializable %s skipped */", ident(p->sym->name));
          else if (is_transient(p->info.typ))
            fprintf(fout, "\n\t/* transient %s skipped */", ident(p->sym->name));
          else if (p->info.sto & Sattribute)
            ;
          else if (is_repetition(p))
            ;
          else if (is_anytype(p))
            ;
          else if (p->info.typ->type == Tarray)
          {
            fprintf(fout, "\n\tcase %s:", soap_union_member(typ, p));
            if (has_ptr(p->info.typ))
              fprintf(fout, "\n\t\tsoap_embedded(soap, a->%s, %s);", ident(p->sym->name), soap_type(p->info.typ));
            fprintf(fout, "\n\t\tsoap_traverse_%s(soap, a->%s, \"%s\", p, q);", c_ident(p->info.typ), ident(p->sym->name), p->sym->name);
            fprintf(fout, "\n\t\tbreak;");
          }
          else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
          {
            fprintf(fout, "\n\tcase %s:", soap_union_member(typ, p));
            if (has_ptr(p->info.typ))
              fprintf(fout, "\n\t\tsoap_embedded(soap, &a->%s, %s);", ident(p->sym->name), soap_type(p->info.typ));
            fprintf(fout, "\n\t\ta->%s.soap_traverse(soap, \"%s\", p, q);", ident(p->sym->name), p->sym->name);
            fprintf(fout, "\n\t\tbreak;");
          }
          else if (p->info.typ->type != Tfun && !is_void(p->info.typ))
          {
            fprintf(fout, "\n\tcase %s:", soap_union_member(typ, p));
            if (has_ptr(p->info.typ))
              fprintf(fout, "\n\t\tsoap_embedded(soap, &a->%s, %s);", ident(p->sym->name), soap_type(p->info.typ));
            fprintf(fout, "\n\t\tsoap_traverse_%s(soap, &a->%s, \"%s\", p, q);", c_ident(p->info.typ), ident(p->sym->name), p->sym->name);
            fprintf(fout, "\n\t\tbreak;");
          }
        }
      }
      fprintf(fout, "\n\tdefault:\n\t\tbreak;\n\t}");
      fprintf(fout, "\n\tif (q) q(soap, (void*)a, %s, s, \"%s\");", soap_type(typ), c_type(typ));
      fprintf(fout, "\n}");
      break;
    case Tpointer:
      if (((Tnode*)typ->ref)->type == Tclass && !is_external((Tnode*)typ->ref) && !is_volatile((Tnode*)typ->ref) && !is_typedef((Tnode*)typ->ref))
      {
        if (is_external(typ))
        {
          fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_traverse_%s(struct soap*, %s, const char *s, soap_walker p, soap_walker q);", c_ident(typ), c_type_id(typ, "*"));
          return;
        }
        fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_traverse_%s(struct soap*, %s, const char *s, soap_walker p, soap_walker q);", c_ident(typ), c_type_id(typ, "*"));
        fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_traverse_%s(struct soap *soap, %s, const char *s, soap_walker p, soap_walker q)\n{", c_ident(typ), c_type_id(typ, "*a"));
        p = is_dynamic_array((Tnode*)typ->ref);
        if (p)
        {
          d = get_Darraydims((Tnode*)typ->ref);
          if (d)
            fprintf(fout, "\n\tif (*a)");
          else
            fprintf(fout, "\n\tif (*a)");
        }
        else
          fprintf(fout, "\n\tif (!soap_reference(soap, *a, %s))", soap_type((Tnode*)typ->ref));
        fprintf(fout, "\n\t\t(*a)->soap_traverse(soap, s, p, q);\n}");
      }
      else
      {
        if (is_external(typ))
        {
          fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_traverse_%s(struct soap*, %s, const char *s, soap_walker p, soap_walker q);", c_ident(typ), c_type_id(typ, "*"));
          return;
        }
        fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_traverse_%s(struct soap*, %s, const char *s, soap_walker p, soap_walker q);", c_ident(typ), c_type_id(typ, "*"));
        fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_traverse_%s(struct soap *soap, %s, const char *s, soap_walker p, soap_walker q)\n{", c_ident(typ), c_type_id(typ, "*a"));
        if (is_primitive((Tnode*)typ->ref))
        {
          fprintf(fout, "\n\tif (!soap_reference(soap, *a, %s))\n\t{", soap_type(typ));
          fprintf(fout, "\n\t\tif (p) p(soap, (void*)*a, %s, s, \"%s\");", soap_type((Tnode*)typ->ref), c_type((Tnode*)typ->ref));
          fprintf(fout, "\n\t\tif (q) q(soap, (void*)*a, %s, s, \"%s\");\n\t}\n}", soap_type((Tnode*)typ->ref), c_type((Tnode*)typ->ref));
        }
        else if ((p = is_dynamic_array((Tnode*)typ->ref)) != NULL)
        {
          d = get_Darraydims((Tnode*)typ->ref);
          if (d)
            fprintf(fout, "\n\tif (*a)");
          else
            fprintf(fout, "\n\tif (*a)");
          fprintf(fout, "\n\t\tsoap_traverse_%s(soap, *a, s, p, q);\n}", c_ident((Tnode*)typ->ref));
        }
        else
        {
          fprintf(fout, "\n\tif (!soap_reference(soap, *a, %s))", soap_type((Tnode*)typ->ref));
          fprintf(fout, "\n\t\tsoap_traverse_%s(soap, *a, s, p, q);\n}", c_ident((Tnode*)typ->ref));
        }
      }
      break;
    case Tarray:
      if (is_external(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_traverse_%s(struct soap*, %s, const char *s, soap_walker p, soap_walker q);", c_ident(typ), c_type(typ));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_traverse_%s(struct soap*, %s, const char *s, soap_walker p, soap_walker q);", c_ident(typ), c_type(typ));
      fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_traverse_%s(struct soap *soap, %s, const char *s, soap_walker p, soap_walker q)", c_ident(typ), c_type_id(typ, "a"));
      if (is_primitive((Tnode*)typ->ref))
      {
        fprintf(fout, "\n{");
        fprintf(fout, "\n\t(void)soap; (void)a; /* appease -Wall -Werror */");
        fprintf(fout, "\n\tif (p) p(soap, (void*)a, %s, s, \"%s\");", soap_type(typ), c_type(typ));
      }
      else
      {
        fprintf(fout, "\n{\n\tsize_t i;");
        fprintf(fout, "\n\tif (p) p(soap, (void*)a, %s, s, \"%s\");", soap_type(typ), c_type(typ));
        fprintf(fout, "\n\tfor (i = 0; i < %d; i++)", get_dimension(typ));
        (void)get_item_type(typ, &cardinality);
        fprintf(fout, "\n\t{");
        if (has_ptr((Tnode*)typ->ref))
        {
          fprintf(fout, "\tsoap_embedded(soap, a");
          if (cardinality > 1)
            fprintf(fout, "[i]");
          else
            fprintf(fout, "+i");
          fprintf(fout, ", %s);", soap_type((Tnode*)typ->ref));
        }
        if (((Tnode *)typ->ref)->type == Tclass && !is_external((Tnode*)typ->ref) && !is_volatile((Tnode*)typ->ref) && !is_typedef((Tnode*)typ->ref))
        {
          fprintf(fout, "\n\ta[i].soap_traverse(soap, s, p, q)");
        }
        else if (!is_primitive((Tnode*)typ->ref))
        {
          fprintf(fout, "\n\tsoap_traverse_%s(soap, a", c_ident((Tnode*)typ->ref));
          if (cardinality > 1)
            fprintf(fout, "[i], s, p, q)");
          else
            fprintf(fout, "+i, s, p, q)");
        }
        fprintf(fout, ";\n\t}");
      }
      fprintf(fout, "\n\tif (q) q(soap, (void*)a, %s, s, \"%s\");", soap_type(typ), c_type(typ));
      fprintf(fout, "\n}");
      break;
    case Ttemplate:
      if (is_external(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 void SOAP_FMAC2 soap_traverse_%s(struct soap*, %s, const char *s, soap_walker p, soap_walker q);", c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 void SOAP_FMAC4 soap_traverse_%s(struct soap*, %s, const char *s, soap_walker p, soap_walker q);", c_ident(typ), c_type_id(typ, "*"));
      temp = (Tnode*)typ->ref;
      if (!temp)
        return;
      fprintf(fout, "\n\nSOAP_FMAC3 void SOAP_FMAC4 soap_traverse_%s(struct soap *soap, %s, const char *s, soap_walker p, soap_walker q)\n{\n\t(void)soap; (void)a; (void)s; (void)p; (void)q; /* appease -Wall -Werror */", c_ident(typ), c_type_id(typ, "*a"));
      if (!is_primitive(temp) && temp->type != Tfun && !is_void(temp))
      {
        fprintf(fout, "\n\tif (p) p(soap, (void*)a, %s, s, \"%s\");", soap_type(typ), c_type(typ));
        fprintf(fout, "\n\tfor (%s::iterator i = a->begin(); i != a->end(); ++i)", c_type(typ));
        if (temp->type == Tclass && !is_external(temp) && !is_volatile(temp) && !is_typedef(temp))
          fprintf(fout, "\n\t\t(*i).soap_traverse(soap, s, p, q);");
        else
          fprintf(fout, "\n\t\tsoap_traverse_%s(soap, &(*i), s, p, q);", c_ident(temp));
        fprintf(fout, "\n\tif (q) q(soap, (void*)a, %s, s, \"%s\");", soap_type(typ), c_type(typ));
      }
      fprintf(fout, "\n}");
    default:
      break;
  }
}

void
soap_put(Tnode *typ)
{
  const char *ci = c_ident(typ);
  const char *ct = c_type(typ);

  if (is_XML(typ))
    return;

  if (typ->type == Ttemplate || typ->type == Tunion)
    return;

  if (is_typedef(typ) && (is_template(typ) /*|| is_element(typ) don't permit for typedef'd elements */ || is_synonym(typ) || is_external(typ) || is_imported(typ)) && (!is_external(typ) || is_volatile(typ)))
  {
    fprintf(fhead, "\n\n#define soap_put_%s soap_put_%s\n", c_ident(typ), t_ident(typ));
    fprintf(fhead, "\n\n#define soap_write_%s soap_write_%s\n", c_ident(typ), t_ident(typ));
    fprintf(fhead, "\n\n#define soap_PUT_%s soap_PUT_%s\n", c_ident(typ), t_ident(typ));
    fprintf(fhead, "\n\n#define soap_PATCH_%s soap_PATCH_%s\n", c_ident(typ), t_ident(typ));
    fprintf(fhead, "\n\n#define soap_POST_send_%s soap_POST_send_%s\n", c_ident(typ), t_ident(typ));
    return;
  }

  if (typ->type == Tarray)
  {
    fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_put_%s(struct soap*, %s, const char*, const char*);", ci, c_type_id(typ, "const"));
    fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_put_%s(struct soap *soap, %s, const char *tag, const char *type)\n{", ci, c_type_id(typ, "const a"));
  }
  else if (typ->type == Tclass && !is_external(typ) && !is_volatile(typ) && !is_typedef(typ))
    fprintf(fout, "\n\nint %s::soap_put(struct soap *soap, const char *tag, const  char *type) const\n{", ct);
  else if (typ->type == Tpointer)
  {
    fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_put_%s(struct soap*, %s, const char*, const char*);", ci, c_type_constptr_id(typ, "const*"));
    fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_put_%s(struct soap *soap, %s, const char *tag, const char *type)\n{", ci, c_type_constptr_id(typ, "const*a"));
  }
  else
  {
    fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_put_%s(struct soap*, const %s, const char*, const char*);", ci, c_type_id(typ, "*"));
    fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_put_%s(struct soap *soap, const %s, const char *tag, const char *type)\n{", ci, c_type_id(typ, "*a"));
  }
  fflush(fout);
  if (typ->type == Tclass && !is_external(typ) && !is_volatile(typ) && !is_typedef(typ))
    fprintf(fout, "\n\tif (soap_out_%s(soap, tag ? tag : \"%s\", -2, this, type))\n\t\treturn soap->error;", ci, xml_tag(typ));
  else
    fprintf(fout, "\n\tif (soap_out_%s(soap, tag ? tag : \"%s\", -2, a, type))\n\t\treturn soap->error;", ci, xml_tag(typ));
  if (!is_invisible(typ->id->name))
    fprintf(fout, "\n\treturn soap_putindependent(soap);\n}");
  else
    fprintf(fout, "\n\treturn SOAP_OK;\n}");
  if ((typ->type != Tpointer || is_string(typ) || is_wstring(typ)) && typ->type != Tarray && typ->type != Treference && typ->type != Trvalueref && !is_template(typ) && !is_anyAttribute(typ))
  {
    const char *x = xsi_type_u(typ);
    if (typ->type == Tclass && !is_external(typ) && !is_volatile(typ) && (!is_typedef(typ) || is_element(typ) /* don't permit for typedef'd elements, see above */))
    {
      fprintf(fhead, "\n\ninline int soap_write_%s(struct soap *soap, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_begin_send(soap) || (p->soap_serialize(soap), 0) || p->soap_put(soap, \"%s\", p->soap_type() == %s ? \"%s\" : NULL) || soap_end_send(soap))\n\t\treturn soap->error;\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), xml_tag(typ), soap_type(typ), x);
      fprintf(fhead, "\n\ninline int soap_PUT_%s(struct soap *soap, const char *URL, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_PUT(soap, URL, NULL, \"text/xml; charset=utf-8\") || (p->soap_serialize(soap), 0) || p->soap_put(soap, \"%s\", p->soap_type() == %s ? \"%s\" : NULL) || soap_end_send(soap) || soap_recv_empty_response(soap))\n\t\treturn soap_closesock(soap);\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), xml_tag(typ), soap_type(typ), x);
      fprintf(fhead, "\n\ninline int soap_PATCH_%s(struct soap *soap, const char *URL, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_PATCH(soap, URL, NULL, \"text/xml; charset=utf-8\") || (p->soap_serialize(soap), 0) || p->soap_put(soap, \"%s\", p->soap_type() == %s ? \"%s\" : NULL) || soap_end_send(soap) || soap_recv_empty_response(soap))\n\t\treturn soap_closesock(soap);\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), xml_tag(typ), soap_type(typ), x);
      fprintf(fhead, "\n\ninline int soap_POST_send_%s(struct soap *soap, const char *URL, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_POST(soap, URL, NULL, \"text/xml; charset=utf-8\") || (p->soap_serialize(soap), 0) || p->soap_put(soap, \"%s\", p->soap_type() == %s ? \"%s\" : NULL) || soap_end_send(soap))\n\t\treturn soap_closesock(soap);\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), xml_tag(typ), soap_type(typ), x);
    }
    else if (is_primitive_or_string(typ))
    {
      if (cflag)
        fprintf(fhead, "\n\n#ifndef soap_write_%s\n#define soap_write_%s(soap, data) ( soap_free_temp(soap), soap_begin_send(soap) || soap_put_%s(soap, data, \"%s\", \"%s\") || soap_end_send(soap), (soap)->error )\n#endif\n", c_ident(typ), c_ident(typ), c_ident(typ), xml_tag(typ), x);
      else if (namespaceid && !is_external(typ))
        fprintf(fhead, "\n\ninline int soap_write_%s(struct soap *soap, %s)\n{\n\tsoap_free_temp(soap);\n\tif (p)\n\t{\tif (soap_begin_send(soap) || %s::soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap))\n\t\t\treturn soap->error;\n\t}\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), namespaceid, c_ident(typ), xml_tag(typ), x);
      else if (!is_external(typ))
        fprintf(fhead, "\n\ninline int soap_write_%s(struct soap *soap, %s)\n{\n\tsoap_free_temp(soap);\n\tif (p)\n\t{\tif (soap_begin_send(soap) || ::soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap))\n\t\t\treturn soap->error;\n\t}\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), c_ident(typ), xml_tag(typ), x);
      else
        fprintf(fhead, "\n\ninline int soap_write_%s(struct soap *soap, %s)\n{\n\tsoap_free_temp(soap);\n\tif (p)\n\t{\tif (soap_begin_send(soap) || soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap))\n\t\t\treturn soap->error;\n\t}\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), c_ident(typ), xml_tag(typ), x);
      if (cflag)
        fprintf(fhead, "\n\n#ifndef soap_PUT_%s\n#define soap_PUT_%s(soap, URL, data) ( soap_free_temp(soap), soap_PUT(soap, URL, NULL, \"text/xml; charset=utf-8\") || soap_put_%s(soap, data, \"%s\", \"%s\") || soap_end_send(soap) || soap_recv_empty_response(soap), soap_closesock(soap) )\n#endif\n", c_ident(typ), c_ident(typ), c_ident(typ), xml_tag(typ), x);
      else if (namespaceid && !is_external(typ))
        fprintf(fhead, "\n\ninline int soap_PUT_%s(struct soap *soap, const char *URL, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_PUT(soap, URL, NULL, \"text/xml; charset=utf-8\") || %s::soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap) || soap_recv_empty_response(soap))\n\t\treturn soap_closesock(soap);\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), namespaceid, c_ident(typ), xml_tag(typ), x);
      else if (!is_external(typ))
        fprintf(fhead, "\n\ninline int soap_PUT_%s(struct soap *soap, const char *URL, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_PUT(soap, URL, NULL, \"text/xml; charset=utf-8\") || ::soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap) || soap_recv_empty_response(soap))\n\t\treturn soap_closesock(soap);\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), c_ident(typ), xml_tag(typ), x);
      else
        fprintf(fhead, "\n\ninline int soap_PUT_%s(struct soap *soap, const char *URL, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_PUT(soap, URL, NULL, \"text/xml; charset=utf-8\") || soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap) || soap_recv_empty_response(soap))\n\t\treturn soap_closesock(soap);\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), c_ident(typ), xml_tag(typ), x);
      if (cflag)
        fprintf(fhead, "\n\n#ifndef soap_PATCH_%s\n#define soap_PATCH_%s(soap, URL, data) ( soap_free_temp(soap), soap_PATCH(soap, URL, NULL, \"text/xml; charset=utf-8\") || soap_put_%s(soap, data, \"%s\", \"%s\") || soap_end_send(soap) || soap_recv_empty_response(soap), soap_closesock(soap) )\n#endif\n", c_ident(typ), c_ident(typ), c_ident(typ), xml_tag(typ), x);
      else if (namespaceid && !is_external(typ))
        fprintf(fhead, "\n\ninline int soap_PATCH_%s(struct soap *soap, const char *URL, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_PATCH(soap, URL, NULL, \"text/xml; charset=utf-8\") || %s::soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap) || soap_recv_empty_response(soap))\n\t\treturn soap_closesock(soap);\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), namespaceid, c_ident(typ), xml_tag(typ), x);
      else if (!is_external(typ))
        fprintf(fhead, "\n\ninline int soap_PATCH_%s(struct soap *soap, const char *URL, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_PATCH(soap, URL, NULL, \"text/xml; charset=utf-8\") || ::soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap) || soap_recv_empty_response(soap))\n\t\treturn soap_closesock(soap);\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), c_ident(typ), xml_tag(typ), x);
      else
        fprintf(fhead, "\n\ninline int soap_PATCH_%s(struct soap *soap, const char *URL, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_PATCH(soap, URL, NULL, \"text/xml; charset=utf-8\") || soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap) || soap_recv_empty_response(soap))\n\t\treturn soap_closesock(soap);\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), c_ident(typ), xml_tag(typ), x);
      if (cflag)
        fprintf(fhead, "\n\n#ifndef soap_POST_send_%s\n#define soap_POST_send_%s(soap, URL, data) ( soap_free_temp(soap), ( soap_POST(soap, URL, NULL, \"text/xml; charset=utf-8\") || soap_put_%s(soap, data, \"%s\", \"%s\") || soap_end_send(soap) ) && soap_closesock(soap), (soap)->error )\n#endif\n", c_ident(typ), c_ident(typ), c_ident(typ), xml_tag(typ), x);
      else if (namespaceid && !is_external(typ))
        fprintf(fhead, "\n\ninline int soap_POST_send_%s(struct soap *soap, const char *URL, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_POST(soap, URL, NULL, \"text/xml; charset=utf-8\") || %s::soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap))\n\t\treturn soap_closesock(soap);\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), namespaceid, c_ident(typ), xml_tag(typ), x);
      else if (!is_external(typ))
        fprintf(fhead, "\n\ninline int soap_POST_send_%s(struct soap *soap, const char *URL, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_POST(soap, URL, NULL, \"text/xml; charset=utf-8\") || ::soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap))\n\t\treturn soap_closesock(soap);\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), c_ident(typ), xml_tag(typ), x);
      else
        fprintf(fhead, "\n\ninline int soap_POST_send_%s(struct soap *soap, const char *URL, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_POST(soap, URL, NULL, \"text/xml; charset=utf-8\") || soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap))\n\t\treturn soap_closesock(soap);\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), c_ident(typ), xml_tag(typ), x);
    }
    else
    {
      if (cflag)
        fprintf(fhead, "\n\n#ifndef soap_write_%s\n#define soap_write_%s(soap, data) ( soap_free_temp(soap), soap_begin_send(soap) || (soap_serialize_%s(soap, data), 0) || soap_put_%s(soap, data, \"%s\", \"%s\") || soap_end_send(soap), (soap)->error )\n#endif\n", c_ident(typ), c_ident(typ), c_ident(typ), c_ident(typ), xml_tag(typ), x);
      else if (namespaceid && !is_external(typ))
        fprintf(fhead, "\n\ninline int soap_write_%s(struct soap *soap, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_begin_send(soap) || (%s::soap_serialize_%s(soap, p), 0) || %s::soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap))\n\t\t\treturn soap->error;\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), namespaceid, c_ident(typ), namespaceid, c_ident(typ), xml_tag(typ), x);
      else if (!is_external(typ))
        fprintf(fhead, "\n\ninline int soap_write_%s(struct soap *soap, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_begin_send(soap) || (::soap_serialize_%s(soap, p), 0) || ::soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap))\n\t\t\treturn soap->error;\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), c_ident(typ), c_ident(typ), xml_tag(typ), x);
      else
        fprintf(fhead, "\n\ninline int soap_write_%s(struct soap *soap, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_begin_send(soap) || (soap_serialize_%s(soap, p), 0) || soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap))\n\t\t\treturn soap->error;\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), c_ident(typ), c_ident(typ), xml_tag(typ), x);
      if (cflag)
        fprintf(fhead, "\n\n#ifndef soap_PUT_%s\n#define soap_PUT_%s(soap, URL, data) ( soap_free_temp(soap), soap_PUT(soap, URL, NULL, \"text/xml; charset=utf-8\") || (soap_serialize_%s(soap, data), 0) || soap_put_%s(soap, data, \"%s\", \"%s\") || soap_end_send(soap) || soap_recv_empty_response(soap), soap_closesock(soap) )\n#endif\n", c_ident(typ), c_ident(typ), c_ident(typ), c_ident(typ), xml_tag(typ), x);
      else if (namespaceid && !is_external(typ))
        fprintf(fhead, "\n\ninline int soap_PUT_%s(struct soap *soap, const char *URL, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_PUT(soap, URL, NULL, \"text/xml; charset=utf-8\") || (%s::soap_serialize_%s(soap, p), 0) || %s::soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap) || soap_recv_empty_response(soap))\n\t\treturn soap_closesock(soap);\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), namespaceid, c_ident(typ), namespaceid, c_ident(typ), xml_tag(typ), x);
      else if (!is_external(typ))
        fprintf(fhead, "\n\ninline int soap_PUT_%s(struct soap *soap, const char *URL, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_PUT(soap, URL, NULL, \"text/xml; charset=utf-8\") || (::soap_serialize_%s(soap, p), 0) || ::soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap) || soap_recv_empty_response(soap))\n\t\treturn soap_closesock(soap);\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), c_ident(typ), c_ident(typ), xml_tag(typ), x);
      else
        fprintf(fhead, "\n\ninline int soap_PUT_%s(struct soap *soap, const char *URL, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_PUT(soap, URL, NULL, \"text/xml; charset=utf-8\") || (soap_serialize_%s(soap, p), 0) || soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap) || soap_recv_empty_response(soap))\n\t\treturn soap_closesock(soap);\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), c_ident(typ), c_ident(typ), xml_tag(typ), x);
      if (cflag)
        fprintf(fhead, "\n\n#ifndef soap_PATCH_%s\n#define soap_PATCH_%s(soap, URL, data) ( soap_free_temp(soap), soap_PATCH(soap, URL, NULL, \"text/xml; charset=utf-8\") || (soap_serialize_%s(soap, data), 0) || soap_put_%s(soap, data, \"%s\", \"%s\") || soap_end_send(soap) || soap_recv_empty_response(soap), soap_closesock(soap) )\n#endif\n", c_ident(typ), c_ident(typ), c_ident(typ), c_ident(typ), xml_tag(typ), x);
      else if (namespaceid && !is_external(typ))
        fprintf(fhead, "\n\ninline int soap_PATCH_%s(struct soap *soap, const char *URL, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_PATCH(soap, URL, NULL, \"text/xml; charset=utf-8\") || (%s::soap_serialize_%s(soap, p), 0) || %s::soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap) || soap_recv_empty_response(soap))\n\t\treturn soap_closesock(soap);\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), namespaceid, c_ident(typ), namespaceid, c_ident(typ), xml_tag(typ), x);
      else if (!is_external(typ))
        fprintf(fhead, "\n\ninline int soap_PATCH_%s(struct soap *soap, const char *URL, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_PATCH(soap, URL, NULL, \"text/xml; charset=utf-8\") || (::soap_serialize_%s(soap, p), 0) || ::soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap) || soap_recv_empty_response(soap))\n\t\treturn soap_closesock(soap);\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), c_ident(typ), c_ident(typ), xml_tag(typ), x);
      else
        fprintf(fhead, "\n\ninline int soap_PATCH_%s(struct soap *soap, const char *URL, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_PATCH(soap, URL, NULL, \"text/xml; charset=utf-8\") || (soap_serialize_%s(soap, p), 0) || soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap) || soap_recv_empty_response(soap))\n\t\treturn soap_closesock(soap);\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), c_ident(typ), c_ident(typ), xml_tag(typ), x);
      if (cflag)
        fprintf(fhead, "\n\n#ifndef soap_POST_send_%s\n#define soap_POST_send_%s(soap, URL, data) ( soap_free_temp(soap), ( soap_POST(soap, URL, NULL, \"text/xml; charset=utf-8\") || (soap_serialize_%s(soap, data), 0) || soap_put_%s(soap, data, \"%s\", \"%s\") || soap_end_send(soap) ) && soap_closesock(soap), (soap)->error )\n#endif\n", c_ident(typ), c_ident(typ), c_ident(typ), c_ident(typ), xml_tag(typ), x);
      else if (namespaceid && !is_external(typ))
        fprintf(fhead, "\n\ninline int soap_POST_send_%s(struct soap *soap, const char *URL, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_POST(soap, URL, NULL, \"text/xml; charset=utf-8\") || (%s::soap_serialize_%s(soap, p), 0) || %s::soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap))\n\t\treturn soap_closesock(soap);\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), namespaceid, c_ident(typ), namespaceid, c_ident(typ), xml_tag(typ), x);
      else if (!is_external(typ))
        fprintf(fhead, "\n\ninline int soap_POST_send_%s(struct soap *soap, const char *URL, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_POST(soap, URL, NULL, \"text/xml; charset=utf-8\") || (::soap_serialize_%s(soap, p), 0) || ::soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap))\n\t\treturn soap_closesock(soap);\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), c_ident(typ), c_ident(typ), xml_tag(typ), x);
      else
        fprintf(fhead, "\n\ninline int soap_POST_send_%s(struct soap *soap, const char *URL, %s)\n{\n\tsoap_free_temp(soap);\n\tif (soap_POST(soap, URL, NULL, \"text/xml; charset=utf-8\") || (soap_serialize_%s(soap, p), 0) || soap_put_%s(soap, p, \"%s\", \"%s\") || soap_end_send(soap))\n\t\treturn soap_closesock(soap);\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_constptr_id(typ, "const*p"), c_ident(typ), c_ident(typ), xml_tag(typ), x);
    }
  }
  fflush(fout);
}

Entry *
is_dynamic_array(Tnode *typ)
{
  Entry *p;
  Table *t;
  if ((typ->type == Tstruct || typ->type == Tclass) && typ->ref)
  {
    for (t = (Table*)typ->ref; t; t = t->prev)
    {
      p = t->list;
      while (p && p->info.typ->type == Tfun)
        p = p->next;
      if (p && (p->info.typ->type == Tpointer || is_smart(p->info.typ)) && !strncmp(ident(p->sym->name), "__ptr", 5))
        if (p->next && (p->next->info.typ->type == Tint || (p->next->info.typ->type == Tarray && (((Tnode*)p->next->info.typ->ref)->type == Tint))) && !strncmp(ident(p->next->sym->name), "__size", 6))
          return p;
    }
  }
  return NULL;
}

Entry *
is_discriminant(Tnode *typ)
{
  Entry *p;
  Table *t;
  if ((typ->type == Tstruct || typ->type == Tclass) && typ->ref)
  {
    t = (Table*)typ->ref;
    /* only if this struct/class has a union and is not derived */
    if (t && !t->prev)
    {
      p = t->list;
      if (p && p->info.typ->type == Tint && ((p->info.sto & Sspecial) || !strncmp(ident(p->sym->name), "__union", 7)))
        if (p->next && p->next->info.typ->type == Tunion)
        {
          Entry *q;
          for (q = p->next->next; q; q = q->next)
          {
            if (q->info.typ->type != Tfun
                && !is_void(q->info.typ)
                && !is_transient(q->info.typ))
              return NULL;
          }
          return p;
        }
    }
  }
  return NULL;
}

int
get_Darraydims(Tnode *typ)
{
  Entry *p;
  Table *t;
  if ((typ->type == Tstruct || typ->type == Tclass) && typ->ref)
  {
    for (t = (Table*)typ->ref; t; t = t->prev)
    {
      p = t->list;
      while (p && p->info.typ->type == Tfun)
        p = p->next;
      if (p && (p->info.typ->type == Tpointer || is_smart(p->info.typ)) && !strncmp(ident(p->sym->name), "__ptr", 5))
        if (p->next && p->next->info.typ->type == Tarray && ((Tnode*)p->next->info.typ->ref)->type == Tint && !strncmp(ident(p->next->sym->name), "__size", 6))
          return get_dimension(p->next->info.typ);
    }
  }
  return 0;
}

const char*
get_Darraysize(const char *a, int d)
{
  int i;
  char *s = (char*)emalloc(d * (strlen(a) + 16) + 1);
  *s = '\0';
  for (i = 0; i < d; i++)
  {
    size_t l = strlen(s);
    sprintf(s + l, "%s%s->__size[%d]", i ? " * " : "", a, i);
  }
  return s;
}

int
has_offset(Tnode *typ)
{
  Entry *p;
  Table *t;
  if (typ->type == Tstruct || typ->type == Tclass)
  {
    for (t = (Table*)typ->ref; t; t = t->prev)
    {
      for (p = t->list; p; p = p->next)
      {
        if ((p->info.typ->type == Tint || (p->info.typ->type == Tarray && ((Tnode*)p->info.typ->ref)->type == Tint)) && !strcmp(ident(p->sym->name), "__offset"))
          return 1;
      }
    }
  }
  return 0;
}

int
is_bool(Tnode *typ)
{
  return (typ->type == Tenum && (Table*)typ->ref == booltable);
}

int
is_boolean(Tnode *typ)
{
  if (typ->type == Tenum)
  {
    if ((Table*)typ->ref == booltable)
      return 1;
    else
    {
      size_t n = strlen(ident(typ->id->name));
      return n >= 7 && is_eq(ident(typ->id->name) + n - 7, "boolean");
    }
  }
  return 0;
}

int
is_hexBinary(Tnode *typ)
{
  Table *t;
  if (!is_binary(typ))
    return 0;
  if ((typ->synonym && strstr(typ->synonym->name, "hex"))
   || (typ->sym && strstr(typ->sym->name, "hex"))
   || (typ->id && strstr(typ->id->name, "hex")))
    return 1;
  for (t = (Table*)typ->ref; t; t = t->prev)
    if (t->sym && strstr(t->sym->name, "hex"))
      return 1;
  return 0;
}

int
is_binary(Tnode *typ)
{
  if (!has_ns(typ) && !is_element(typ))
    return 0;
  if (typ->type == Tstruct || typ->type == Tclass)
  {
    Table *t;
    for (t = (Table*)typ->ref; t; t = t->prev)
    {
      Entry *p;
      p = t->list;
      while (p && p->info.typ->type == Tfun)
        p = p->next;
      if (p && (p->info.typ->type == Tpointer || is_smart(p->info.typ)) && ((Tnode*)p->info.typ->ref)->type == Tuchar && !strcmp(ident(p->sym->name), "__ptr"))
      {
        p = p->next;
        return p && p->info.typ->type == Tint && !strncmp(ident(p->sym->name), "__size", 6);
      }
    }
  }
  return 0;
}

int
is_attachment(Tnode *typ)
{
  Entry *p;
  Table *t;
  if (!is_binary(typ) || is_transient(typ))
    return 0;
  for (t = (Table*)typ->ref; t; t = t->prev)
  {
    for (p = t->list; p; p = p->next)
    {
      if (is_string(p->info.typ) && !strcmp(p->sym->name, "id"))
      {
        p = p->next;
        if (!p || !is_string(p->info.typ) || strcmp(p->sym->name, "type"))
          break;
        p = p->next;
        if (!p || !is_string(p->info.typ) || strcmp(p->sym->name, "options"))
          break;
        return 1;
      }
    }
  }
  return 0;
}

int
is_mutable(Entry *e)
{
  return e->info.typ->transient == -4 || is_header_or_fault(e->info.typ);
}

int
is_header_or_fault(Tnode *typ)
{
  if (typ->type == Tpointer || typ->type == Treference)
    return is_header_or_fault((Tnode*)typ->ref);
  return (typ->type == Tstruct || typ->type == Tclass) && (!strcmp(ident(typ->id->name), "SOAP_ENV__Header") || !strcmp(ident(typ->id->name), "SOAP_ENV__Fault") || !strcmp(ident(typ->id->name), "SOAP_ENV__Text") || !strcmp(ident(typ->id->name), "SOAP_ENV__Code") || !strcmp(ident(typ->id->name), "SOAP_ENV__Detail") || !strcmp(ident(typ->id->name), "SOAP_ENV__Reason"));
}

int
is_body(Tnode *typ)
{
  if (typ->type == Tpointer || typ->type == Treference)
    return is_body((Tnode*)typ->ref);
  return (typ->type == Tstruct || typ->type == Tclass) && !strcmp(ident(typ->id->name), "SOAP_ENV__Body");
}

long
minlen(Tnode *typ)
{
  if (!typ->hasmin || typ->imin < 0)
    return 0;
  if (typ->imin > 2147483647)
    return 2147483647;
  if (typ->incmin)
    return (long)typ->imin;
  return (long)typ->imin + 1;
}

long
maxlen(Tnode *typ)
{
  if (!typ->hasmax || typ->imax < 0)
    return -1;
  if (typ->imax > 2147483647)
    return 2147483647;
  if (typ->incmax)
    return (long)typ->imax;
  return (long)typ->imax - 1;
}

const char*
pattern(Tnode *typ)
{
  if (!typ->pattern || (typ->pattern[0] == '%' && typ->pattern[1]))
    return "NULL";
  return cstring(typ->pattern, 1);
}

int
is_soap12(const char *enc)
{
  return !strcmp(envURI, "http://www.w3.org/2003/05/soap-envelope") || (enc && !strcmp(enc, "http://www.w3.org/2003/05/soap-encoding"));
}

int
is_document(const char *style)
{
  return soap_version < 0 || (!eflag && !style) || (style && !strcmp(style, "document"));
}

int
is_literal(const char *encoding)
{
  return soap_version < 0 || (!eflag && !encoding) || (encoding && !strcmp(encoding, "literal"));
}

const char *
has_soapref(Tnode *typ)
{
  Entry *p;
  Table *t;
  if (typ->type == Tstruct || typ->type == Tclass)
    for (t = (Table*)typ->ref; t; t = t->prev)
      for (p = t->list; p; p = p->next)
        if (is_soapref(p->info.typ) && (t == (Table*)typ->ref || !(p->info.sto & Sprivate)))
          return ident(p->sym->name);
  return NULL;
}

int
is_soapref(Tnode *typ)
{
  return typ->type == Tpointer && ((Tnode*)typ->ref)->type == Tstruct && ((Tnode*)typ->ref)->id == lookup("soap");
}

const char *
union_member(Tnode *typ)
{
  if (Tptr[Tunion] != NULL)
  {
    Table *t;
    Entry *p, *q;
    for (p = classtable->list; p; p = p->next)
    {
      if (p->info.typ->type == Tunion)
      {
        for (t = (Table*)p->info.typ->ref; t; t = t->prev)
          for (q = t->list; q; q = q->next)
            if (typ == q->info.typ)
              return p->info.typ->id->name;
      }
      else
      {
        for (t = (Table*)p->info.typ->ref; t; t = t->prev)
        {
          for (q = t->list; q; q = q->next)
          {
            if (typ == q->info.typ)
            {
              const char *s = union_member(p->info.typ);
              if (s)
                return s;
            }
          }
        }
      }
    }
  }
  return NULL;
}

int
has_union(Tnode *typ)
{
  Table *t;
  Entry *p;
  if (typ->type == Tclass || typ->type == Tstruct)
  {
    for (t = (Table*)typ->ref; t; t = t->prev)
    {
      for (p = t->list; p; p = p->next)
      {
        if (p->info.typ->type == Tunion)
          return 1;
        else if ((p->info.typ->type == Tstruct || p->info.typ->type == Tclass) && has_union(p->info.typ))
          return 1;
      }
    }
  }
  return 0;
}

int
has_constructor(Tnode *typ)
{
  Table *t;
  Entry *p;
  if (typ->type == Tclass || typ->type == Tstruct)
    for (t = (Table*)typ->ref; t; t = t->prev)
      for (p = t->list; p; p = p->next)
        if (p->info.typ->type == Tfun && !strcmp(p->sym->name, typ->id->name) && ((FNinfo *)p->info.typ->ref)->ret->type == Tnone)
          if (!((FNinfo*)p->info.typ->ref)->args->list)
            return 1;
  return 0;
}

int
has_destructor(Tnode *typ)
{
  Entry *p;
  Table *t;
  if (typ->type == Tclass || typ->type == Tstruct)
    for (t = (Table*)typ->ref; t; t = t->prev)
      for (p = t->list; p; p = p->next)
        if (p->info.typ->type == Tfun && strchr(p->sym->name, '~'))
          return 1;
  return 0;
}

int
has_getter(Tnode *typ)
{
  Entry *p, *q;
  Table *t;
  if (typ->type == Tclass || (!cflag && typ->type == Tstruct))
  {
    for (t = (Table*)typ->ref; t; t = t->prev)
    {
      for (p = t->list; p; p = p->next)
      {
        if (p->info.typ->type == Tfun && !strcmp(p->sym->name, "get") && ((FNinfo *)p->info.typ->ref)->ret->type == Tint)
        {
          q = ((FNinfo*)p->info.typ->ref)->args->list;
          if (q && q->info.typ->type == Tpointer && ((Tnode*)q->info.typ->ref)->type == Tstruct && ((Tnode*)q->info.typ->ref)->id == lookup("soap"))
            return 1;
        }
      }
    }
  }
  return 0;
}

int
has_setter(Tnode *typ)
{
  Entry *p, *q;
  Table *t;
  if (typ->type == Tclass || (!cflag && typ->type == Tstruct))
  {
    for (t = (Table*)typ->ref; t; t = t->prev)
    {
      for (p = t->list; p; p = p->next)
      {
        if (p->info.typ->type == Tfun && !strcmp(p->sym->name, "set") && ((FNinfo *)p->info.typ->ref)->ret->type == Tint)
        {
          q = ((FNinfo*)p->info.typ->ref)->args->list;
          if (q && q->info.typ->type == Tpointer && ((Tnode*)q->info.typ->ref)->type == Tstruct && ((Tnode*)q->info.typ->ref)->id == lookup("soap"))
            return 1;
        }
      }
    }
  }
  return 0;
}

const char*
kind_of(Tnode *typ)
{
  if (is_bool(typ))
    return "bool";
  if (is_attachment(typ))
    return "base64 binary or attachment";
  if (is_hexBinary(typ))
    return "hex binary";
  if (is_binary(typ))
    return "base64 binary";
  if (is_stdstring(typ))
    return "std::string";
  if (is_stdwstring(typ))
    return "std::wstring";
  if (is_external(typ))
    return "custom";
  if (is_string(typ))
    return "string";
  if (is_wstring(typ))
    return "wide string";
  if (typ->type == Tenum || typ->type == Tenumsc)
    return "enum";
  if (is_primitive(typ))
    return c_type(typ);
  if (typ->type == Tclass)
    return "class";
  if (typ->type == Tstruct)
    return "struct";
  if (is_smart(typ))
    return "smart pointer";
  if (is_template(typ))
    return "container";
  return "";
}

int
is_primitive_or_string(Tnode *typ)
{
  return is_primitive(typ) || is_string(typ) || is_wstring(typ) || is_stdstring(typ) || is_stdwstring(typ) || is_qname(typ) || is_stdqname(typ);
}

int
is_primitive(Tnode *typ)
{
  return typ->type >= Tchar && typ->type <= Tenumsc;
}

int
is_string(Tnode *typ)
{
  return typ->type == Tpointer && ((Tnode*)typ->ref)->type == Tchar && !((Tnode*)typ->ref)->sym;
}

int
is_fixedstring(Tnode *typ)
{
  return bflag && typ->type == Tarray && ((Tnode*)typ->ref)->type == Tchar;
}

int
is_wstring(Tnode *typ)
{
  return typ->type == Tpointer && ((Tnode*)typ->ref)->type == Twchar && !((Tnode*)typ->ref)->sym;
}

int
is_stdstring(Tnode *typ)
{
  return typ->type == Tclass && typ->id == lookup("std::string");
}

int
is_stdwstring(Tnode *typ)
{
  return typ->type == Tclass && typ->id == lookup("std::wstring");
}

int
is_stdstr(Tnode *typ)
{
  if (typ->type == Tpointer)
    return is_stdstring((Tnode*)typ->ref) || is_stdwstring((Tnode*)typ->ref);
  return is_stdstring(typ) || is_stdwstring(typ);
}

int
is_typedef(Tnode *typ)
{
  return typ->sym && !is_transient(typ);
}

int
is_restriction(Tnode *typ)
{
  return is_typedef(typ) && typ->restriction;
}

int
has_restriction_base(Tnode *typ, const char *base)
{
  while (typ)
  {
    Entry *p;
    if (typ->sym && is_eq(typ->sym->name, base))
      return 1;
    if (!typ->restriction)
      break;
    if (is_eq(typ->restriction->name, base))
      return 1;
    p = entry(typetable, typ->restriction);
    if (!p)
      break;
    typ = p->info.typ;
  }
  return 0;
}

int
is_synonym(Tnode *typ)
{
  return is_typedef(typ) && typ->synonym;
}

int
reflevel(Tnode *typ)
{
  int level;
  for (level = 0; typ->type == Tpointer || is_smart(typ); level++)
    typ = (Tnode*)typ->ref;
  return level;
}

Tnode *
reftype(Tnode *typ)
{
  while ((typ->type == Tpointer && !is_string(typ) && !is_wstring(typ)) ||
      typ->type == Treference ||
      typ->type == Trvalueref ||
      is_smart(typ))
    typ = (Tnode*)typ->ref;
  return typ;
}

void
soap_set_attr(Entry *p, const char *obj, const char *name, const char *tag)
{
  Tnode *typ = p->info.typ;
  if (p->info.sto & (Sconst | Sprivate | Sprotected))
    return;
  if (typ->type == Treference || typ->type == Trvalueref)
    typ = typ->ref;
  if (p->info.minOccurs == 0 && (typ->type != Tpointer || is_string(typ) || is_wstring(typ)) && p->info.hasval && !p->info.ptrval && !p->info.fixed)
  {
    if (is_string(typ))
      fprintf(fout, "\n\tif (!%s->%s || strcmp(%s->%s, %s))\n\t{\t", obj, name, obj, name, c_init_a(p, ""));
    else if (is_wstring(typ))
      fprintf(fout, "\n\tif (!%s->%s || wcscmp(%s->%s, %s))\n\t{\t", obj, name, obj, name, c_init_a(p, ""));
    else
      fprintf(fout, "\n\tif (%s->%s != %s)\n\t{\t", obj, name, c_init_a(p, ""));
  }
  else
    fprintf(fout, "\n\t");
  if ((is_external(typ) || is_typedef(typ)) && !is_anyAttribute(typ) && !is_anyType(typ) && !is_smart(typ) && typ->type != Tpointer && !is_stdstr(typ) && !is_string(typ) && !is_wstring(typ) && !is_binary(typ))
    fprintf(fout, "soap_set_attr(soap, \"%s\", soap_%s2s(soap, %s->%s), 1);", tag, c_ident(typ), obj, name);
  else if (is_XML(typ))
  {
    if (p->info.minOccurs > 0)
      fprintf(fout, "soap_set_attr(soap, \"%s\", %s->%s ? %s->%s : \"%s\", 1);", tag, obj, name, obj, name, default_value(p));
    else
      fprintf(fout, "if (%s->%s)\n\t\tsoap_set_attr(soap, \"%s\", %s->%s, 1);", obj, name, tag, obj, name);
  }
  else if (is_string(typ))
  {
    if (p->info.minOccurs > 0)
      fprintf(fout, "soap_set_attr(soap, \"%s\", %s->%s ? soap_%s2s(soap, %s->%s) : \"%s\", 1);", tag, obj, name, c_ident(typ), obj, name, default_value(p));
    else
      fprintf(fout, "if (%s->%s)\n\t\tsoap_set_attr(soap, \"%s\", soap_%s2s(soap, %s->%s), 1);", obj, name, tag, c_ident(typ), obj, name);
  }
  else if (is_wstring(typ))
  {
    if (p->info.minOccurs > 0)
      fprintf(fout, "soap_set_attr(soap, \"%s\", %s->%s ? soap_%s2s(soap, %s->%s) : \"%s\", 2);", tag, obj, name, c_ident(typ), obj, name, default_value(p));
    else
      fprintf(fout, "if (%s->%s)\n\t\tsoap_set_attr(soap, \"%s\", soap_%s2s(soap, %s->%s), 2);", obj, name, tag, c_ident(typ), obj, name);
  }
  else if (is_stdstring(typ))
  {
    fprintf(fout, "soap_set_attr(soap, \"%s\", soap_%s2s(soap, %s->%s), 1);", tag, c_ident(typ), obj, name);
  }
  else if (is_stdwstring(typ))
  {
    fprintf(fout, "soap_set_attr(soap, \"%s\", soap_%s2s(soap, %s->%s), 2);", tag, c_ident(typ), obj, name);
  }
  else if (is_smart(typ) || (typ->type == Tpointer && !is_string(typ) && !is_wstring(typ)))
  {
    Tnode *ptr = (Tnode*)typ->ref;
    if (is_smart(typ))
      fprintf(fout, "if (%s->%s.get())\n\t{\t", obj, name);
    else
      fprintf(fout, "if (%s->%s)\n\t{\t", obj, name);
    if ((is_external(ptr) || is_typedef(ptr)) && !is_anyAttribute(ptr) && !is_anyType(ptr) && !is_stdstr(ptr) && !is_string(ptr) && !is_wstring(ptr) && !is_binary(ptr))
      fprintf(fout, "soap_set_attr(soap, \"%s\", soap_%s2s(soap, *%s->%s), 1);", tag, c_ident(ptr), obj, name);
    else if (is_XML(ptr))
      fprintf(fout, "if (*%s->%s)\n\t\tsoap_set_attr(soap, \"%s\", *%s->%s, 1);", obj, name, tag, obj, name);
    else if (is_string(ptr))
      fprintf(fout, "if (*%s->%s)\n\t\t\tsoap_set_attr(soap, \"%s\", soap_%s2s(soap, *%s->%s), 1);", obj, name, tag, c_ident(ptr), obj, name);
    else if (is_wstring(ptr))
      fprintf(fout, "if (*%s->%s)\n\t\t\tsoap_set_attr(soap, \"%s\", soap_%s2s(soap, *%s->%s), 2);", obj, name, tag, c_ident(ptr), obj, name);
    else if (is_stdwstring(ptr))
      fprintf(fout, "soap_set_attr(soap, \"%s\", soap_%s2s(soap, *%s->%s), 2);", tag, c_ident(ptr), obj, name);
    else if (is_primitive_or_string(ptr) || is_binary(ptr))
      fprintf(fout, "soap_set_attr(soap, \"%s\", soap_%s2s(soap, *%s->%s), 1);", tag, c_ident(ptr), obj, name);
    else if (is_anyAttribute(ptr))
      fprintf(fout, "if (soap_out_%s(soap, \"%s\", -1, %s->%s, \"%s\"))\n\t\t\treturn soap->error;", c_ident(ptr), tag, obj, name, xsi_type_u(ptr));
    else
    {
      sprintf(errbuf, "Member '%s' cannot be serialized as an XML attribute", name);
      semwarn(errbuf);
    }
    fprintf(fout, "\n\t}");
    if (p->info.minOccurs > 0)
    {
      fprintf(fout, "\n\telse\n\t\t");
      if (is_wstring(ptr) || is_stdwstring(ptr))
        fprintf(fout, "soap_set_attr(soap, \"%s\", \"%s\", 2);", tag, default_value(p));
      else
        fprintf(fout, "soap_set_attr(soap, \"%s\", \"%s\", 1);", tag, default_value(p));
    }
  }
  else if (is_primitive_or_string(typ) || is_binary(typ))
    fprintf(fout, "soap_set_attr(soap, \"%s\", soap_%s2s(soap, %s->%s), 1);", tag, c_ident(typ), obj, name);
  else if (is_anyAttribute(typ))
    fprintf(fout, "if (soap_out_%s(soap, \"%s\", -1, &%s->%s, \"%s\"))\n\t\treturn soap->error;", c_ident(typ), tag, obj, name, xsi_type_u(typ));
  else
  {
    sprintf(errbuf, "Member '%s' cannot be serialized as an XML attribute", name);
    semwarn(errbuf);
  }
  if (p->info.minOccurs == 0 && (typ->type != Tpointer || is_string(typ) || is_wstring(typ)) && p->info.hasval && !p->info.ptrval && !p->info.fixed)
    fprintf(fout, "\n\t}");
}

void
soap_attr_value(Entry *p, const char *obj, const char *name, const char *tag)
{
  int occurs = 0;
  Tnode *typ = p->info.typ;
  if (p->info.sto & (Sconst | Sprivate | Sprotected))
    return;
  if (typ->type == Treference || typ->type == Trvalueref)
    typ = typ->ref;
  if (p->info.maxOccurs == 0)
    occurs = 2; /* prohibited */
  else if (p->info.minOccurs >= 1)
    occurs = 1; /* required */
  if (sflag && occurs)
    occurs += 2; /* prohibited/required if SOAP_XML_STRICT */
  if ((is_external(typ) || is_typedef(typ)) && !is_anyAttribute(typ) && !is_anyType(typ) && !is_smart(typ) && typ->type != Tpointer && !is_stdstr(typ) && !is_string(typ) && !is_wstring(typ) && !is_binary(typ))
    fprintf(fout, "\n\tif (soap_s2%s(soap, soap_attr_value(soap, \"%s\", %d, %d), &%s->%s))\n\t\treturn NULL;", c_ident(typ), tag, property(typ), occurs, obj, name);
  else if (is_smart(typ) || (typ->type == Tpointer && !is_string(typ) && !is_wstring(typ)))
  {
    Tnode *ptr = (Tnode*)typ->ref;
    const char *get = "";
    if (!is_anyAttribute(ptr) && !is_anyType(ptr))
      fprintf(fout, "\n\t{\n\t\tconst char *t = soap_attr_value(soap, \"%s\", %d, %d);\n\t\tif (t)\n\t\t{", tag, property(ptr), occurs);
    if (is_smart(typ))
    {
      fprintf(fout, "\n\t\t\t%s->%s = %s(SOAP_NEW(soap, %s));", obj, name, c_type(typ), c_type(ptr));
      get = ".get()";
    }
    else if (is_stdstr(ptr) || ptr->type == Tclass)
      fprintf(fout, "\n\t\t\tif (!(%s->%s = soap_new_%s(soap)))\n\t\t\t{\tsoap->error = SOAP_EOM;\n\t\t\t\treturn NULL;\n\t\t\t}", obj, name, c_ident(ptr));
    else
      fprintf(fout, "\n\t\t\tif (!(%s->%s = (%s)soap_malloc(soap, sizeof(%s))))\n\t\t\t{\tsoap->error = SOAP_EOM;\n\t\t\t\treturn NULL;\n\t\t\t}", obj, name, c_type(typ), c_type(ptr));
    if ((is_external(ptr) || is_typedef(ptr)) && !is_anyAttribute(ptr) && !is_anyType(ptr) && !is_stdstr(ptr) && !is_string(ptr) && !is_wstring(ptr) && !is_binary(ptr))
      fprintf(fout, "\n\t\t\tif (soap_s2%s(soap, t, %s->%s%s))\n\t\t\t\treturn NULL;", c_ident(ptr), obj, name, get);
    else if (is_smart(typ) && is_anyAttribute(ptr))
      fprintf(fout, "\n\t\t\tif (!soap_in_%s(soap, \"%s\", %s->%s%s, \"%s\")\n\t\t\t\t%s->%s.reset();", c_ident(ptr), tag, obj, name, get, xsi_type(ptr), obj, name);
    else if (is_anyAttribute(ptr))
      fprintf(fout, "\n\t\t\t%s->%s = soap_in_%s(soap, \"%s\", %s->%s, \"%s\");", obj, name, c_ident(ptr), tag, obj, name, xsi_type(ptr));
    else if (is_XML(ptr))
      fprintf(fout, "\n\t\t\tif (soap_s2char(soap, t, %s->%s%s, 0, %ld, %ld, %s))\n\t\t\t\treturn NULL;", obj, name, get, minlen(ptr), maxlen(ptr), pattern(ptr));
    else if (is_primitive_or_string(ptr) || is_binary(ptr))
      fprintf(fout, "\n\t\t\tif (soap_s2%s(soap, t, %s->%s%s))\n\t\t\t\treturn NULL;", c_ident(ptr), obj, name, get);
    if (!is_anyAttribute(ptr) && !is_anyType(ptr))
    {
      fixed_check(fout, p, NULL, "\t\t\t");
      fprintf(fout, "\n\t\t}\n\t\telse if (soap->error)\n\t\t\treturn NULL;\n\t}");
    }
  }
  else if (is_anyAttribute(typ))
    fprintf(fout, "\n\tsoap_in_%s(soap, \"%s\", &%s->%s, \"%s\");", c_ident(typ), tag, obj, name, xsi_type(typ));
  else if (is_XML(typ))
    fprintf(fout, "\n\tif (soap_s2char(soap, soap_attr_value(soap, \"%s\", 0, %d), &%s->%s, 0, %ld, %ld, %s))\n\t\treturn NULL;", tag, occurs, obj, name, minlen(typ), maxlen(typ), pattern(typ));
  else if (is_primitive_or_string(typ) || is_binary(typ))
    fprintf(fout, "\n\tif (soap_s2%s(soap, soap_attr_value(soap, \"%s\", %d, %d), &%s->%s))\n\t\treturn NULL;", c_ident(typ), tag, property(typ), occurs, obj, name);
  if (!is_smart(typ) && !(typ->type == Tpointer && !is_string(typ) && !is_wstring(typ)))
    fixed_check(fout, p, NULL, "\t");
}

const char *
ptr_cast(Table *t, const char *name)
{
  const char *type = ident(t->sym->name);
  char *s = (char*)emalloc(strlen(type) + strlen(name) + 6);
  sprintf(s, "((%s*)%s)", type, name);
  return s;
}

void
soap_out(Tnode *typ)
{
  Table *table, *t;
  Entry *p = NULL;
  int cardinality, i, j, d;
  Tnode *temp;
  const char *nse = ns_qualifiedElement(typ);
  const char *nsa = ns_qualifiedAttribute(typ);
  const char *x;

  if (is_XML(typ))
    return;

  if (is_dynamic_array(typ))
  {
    soap_out_Darray(typ);
    return;
  }

  if (is_external(typ) && !is_volatile(typ))
    fprintf(fhead, "\nSOAP_FMAC3S const char* SOAP_FMAC4S soap_%s2s(struct soap*, %s);", c_ident(typ), c_type(typ));
  else if (is_qname(typ))
    fprintf(fhead, "\n\n#define soap_%s2s(soap, a) soap_QName2s(soap, (a))", c_ident(typ));
  else if (is_string(typ))
    fprintf(fhead, "\n\n#define soap_%s2s(soap, a) (a)", c_ident(typ));
  else if (is_wstring(typ))
    fprintf(fhead, "\n\n#define soap_%s2s(soap, a) soap_wchar2s((soap), (a))", c_ident(typ));
  else if (is_stdqname(typ))
    fprintf(fhead, "\n\n#define soap_%s2s(soap, a) soap_QName2s((soap), (a).c_str())", c_ident(typ));
  else if (is_stdstring(typ))
    fprintf(fhead, "\n\n#define soap_%s2s(soap, a) ((a).c_str())", c_ident(typ));
  else if (is_stdwstring(typ))
    fprintf(fhead, "\n\n#define soap_%s2s(soap, a) soap_wchar2s((soap), (a).c_str())", c_ident(typ));
  else if (is_typedef(typ) && (!is_external(typ) || is_volatile(typ)) && !is_qname(typ) && !is_stdqname(typ))
  {
    if ((typ->type == Tfloat || typ->type == Tdouble || typ->type == Tldouble) && typ->pattern && typ->pattern[0] == '%' && typ->pattern[1])
    {
      fprintf(fhead, "\nSOAP_FMAC3S const char* SOAP_FMAC4S soap_%s2s(struct soap*, %s);", c_ident(typ), c_type(typ));
      fprintf(fout, "\n\nSOAP_FMAC3S const char* SOAP_FMAC4S soap_%s2s(struct soap *soap, %s)", c_ident(typ), c_type_id(typ, "n"));
      fprintf(fout, "\n{\n\tconst char *s;");
      if (typ->type == Tfloat)
        fprintf(fout, "\n\tconst char *t = soap->float_format;\n\tsoap->float_format = \"%s\";", typ->pattern);
      else if (typ->type == Tdouble)
        fprintf(fout, "\n\tconst char *t = soap->double_format;\n\tsoap->double_format = \"%s\";", typ->pattern);
      else if (typ->type == Tldouble)
        fprintf(fout, "\n\tconst char *t = soap->long_double_format;\n\tsoap->long_double_format = \"%s\";", typ->pattern);
      fprintf(fout, "\n\ts = soap_%s2s(soap, n);", t_ident(typ));
      if (typ->type == Tfloat)
        fprintf(fout, "\n\tsoap->float_format = t;");
      else if (typ->type == Tdouble)
        fprintf(fout, "\n\tsoap->double_format = t;");
      else if (typ->type == Tldouble)
        fprintf(fout, "\n\tsoap->long_double_format = t;");
      fprintf(fout, "\n\treturn s;\n}");
    }
    else
    {
      fprintf(fhead, "\n\n#define soap_%s2s soap_%s2s\n", c_ident(typ), t_ident(typ));
    }
  }

  if (is_typedef(typ) && (is_element(typ) || is_synonym(typ) || is_external(typ)) && (!is_external(typ) || is_volatile(typ)))
  {
    fprintf(fhead, "\n\n#define soap_out_%s soap_out_%s\n", c_ident(typ), t_ident(typ));
    return;
  }

  if (is_primitive(typ) && typ->type != Tenum &&  typ->type != Tenumsc)
  {
    if (is_external(typ))
    {
      fprintf(fhead, "\nSOAP_FMAC1 int SOAP_FMAC2 soap_out_%s(struct soap*, const char*, int, const %s, const char*);", c_ident(typ), c_type_id(typ, "*"));
      return;
    }
    fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap*, const char*, int, const %s, const char*);", c_ident(typ), c_type_id(typ, "*"));
    fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap *soap, const char *tag, int id, const %s, const char *type)\n{", c_ident(typ), c_type_id(typ, "*a"));
    if (typ->pattern && typ->pattern[0] == '%' && typ->pattern[1])
    {
      fprintf(fout, "\n\tint err;");
      if (typ->type == Tfloat)
        fprintf(fout, "\n\tconst char *s = soap->float_format;\n\tsoap->float_format = \"%s\";", typ->pattern);
      else if (typ->type == Tdouble)
        fprintf(fout, "\n\tconst char *s = soap->double_format;\n\tsoap->double_format = \"%s\";", typ->pattern);
      else if (typ->type == Tldouble)
        fprintf(fout, "\n\tconst char *s = soap->long_double_format;\n\tsoap->long_double_format = \"%s\";", typ->pattern);
      x = xsi_type_u(typ);
      if (x && *x)
        fprintf(fout, "\n\tif (!type)\n\t\ttype = \"%s\";", x);
      if (typ->type == Tllong || typ->type == Tullong)
        fprintf(fout, "\n\terr = soap_out%s(soap, tag, id, a, type, %s);", c_type(typ), soap_type(typ));
      else
        fprintf(fout, "\n\terr = soap_out%s(soap, tag, id, a, type, %s);", the_type(typ), soap_type(typ));
      if (typ->type == Tfloat)
        fprintf(fout, "\n\tsoap->float_format = s;");
      else if (typ->type == Tdouble)
        fprintf(fout, "\n\tsoap->double_format = s;");
      else if (typ->type == Tldouble)
        fprintf(fout, "\n\tsoap->long_double_format = s;");
      fprintf(fout, "\n\treturn err;\n}");
    }
    else
    {
      x = xsi_type_u(typ);
      if (x && *x)
        fprintf(fout, "\n\tif (!type)\n\t\ttype = \"%s\";", x);
      if (typ->type == Tllong || typ->type == Tullong)
        fprintf(fout, "\n\treturn soap_out%s(soap, tag, id, a, type, %s);\n}", c_type(typ), soap_type(typ));
      else
        fprintf(fout, "\n\treturn soap_out%s(soap, tag, id, a, type, %s);\n}", the_type(typ), soap_type(typ));
    }
    return;
  }
  if (is_fixedstring(typ))
  {
    fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap*, const char*, int, const char[], const char*);", c_ident(typ));
    fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap *soap, const char *tag, int id, const char a[], const char *type)\n{", c_ident(typ));
    fprintf(fout, "\n\treturn soap_outstring(soap, tag, id, (char*const*)(void*)&a, type, %s);\n}", soap_type(typ));
    return;
  }
  if (is_string(typ))
  {
    if (is_external(typ))
    {
      fprintf(fhead, "\nSOAP_FMAC1 int SOAP_FMAC2 soap_out_%s(struct soap*, const char*, int, char*const*, const char*);", c_ident(typ));
      return;
    }
    fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap*, const char*, int, char*const*, const char*);", c_ident(typ));
    fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap *soap, const char *tag, int id, char *const*a, const char *type)\n{", c_ident(typ));
    fprintf(fout, "\n\treturn soap_outstring(soap, tag, id, a, type, %s);\n}", soap_type(typ));
    return;
  }
  if (is_wstring(typ))
  {
    if (is_external(typ))
    {
      fprintf(fhead, "\nSOAP_FMAC1 int SOAP_FMAC2 soap_out_%s(struct soap*, const char*, int, wchar_t*const*, const char*);", c_ident(typ));
      return;
    }
    fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap*, const char*, int, wchar_t*const*, const char*);", c_ident(typ));
    fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap *soap, const char *tag, int id, wchar_t *const*a, const char *type)\n{", c_ident(typ));
    fprintf(fout, "\n\treturn soap_outwstring(soap, tag, id, a, type, %s);\n}", soap_type(typ));
    return;
  }
  if (is_stdstring(typ))
  {
    if (is_external(typ))
    {
      fprintf(fhead, "\nSOAP_FMAC1 int SOAP_FMAC2 soap_out_%s(struct soap*, const char*, int, const std::string*, const char*);", c_ident(typ));
      return;
    }
    fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap*, const char*, int, const std::string*, const char*);", c_ident(typ));
    if (is_stdXML(typ))
      fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap *soap, const char *tag, int id, const std::string *s, const char *type)\n{\n\t(void)id; /* appease -Wall -Werror */\n\tconst char *t = s->c_str();\n\treturn soap_outliteral(soap, tag, (char*const*)(void*)&t, type);\n}", c_ident(typ));
    else
      fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap *soap, const char *tag, int id, const std::string *s, const char *type)\n{\n\tif ((soap->mode & SOAP_C_NILSTRING) && s->empty())\n\t\treturn soap_element_null(soap, tag, id, type);\n\tif (soap_element_begin_out(soap, tag, soap_embedded_id(soap, id, s, %s), type) || soap_string_out(soap, s->c_str(), 0) || soap_element_end_out(soap, tag))\n\t\treturn soap->error;\n\treturn SOAP_OK;\n}", c_ident(typ), soap_type(typ));
    return;
  }
  if (is_stdwstring(typ))
  {
    if (is_external(typ))
    {
      fprintf(fhead, "\nSOAP_FMAC1 int SOAP_FMAC2 soap_out_%s(struct soap*, const char*, int, const std::wstring*, const char*);", c_ident(typ));
      return;
    }
    fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap*, const char*, int, const std::wstring*, const char*);", c_ident(typ));
    if (is_stdXML(typ))
      fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap *soap, const char *tag, int id, const std::wstring *s, const char *type)\n{\n\tconst wchar_t *t = s->c_str();\n\treturn soap_outwliteral(soap, tag, (wchar_t*const*)(void*)&t, type);\n}", c_ident(typ));
    else
      fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap *soap, const char *tag, int id, const std::wstring *s, const char *type)\n{\n\tif ((soap->mode & SOAP_C_NILSTRING) && s->empty())\n\t\treturn soap_element_null(soap, tag, id, type);\n\tif (soap_element_begin_out(soap, tag, soap_embedded_id(soap, id, s, %s), type) || soap_wstring_out(soap, s->c_str(), 0) || soap_element_end_out(soap, tag))\n\t\treturn soap->error;\n\treturn SOAP_OK;\n}", c_ident(typ), soap_type(typ));
    return;
  }
  switch(typ->type)
  {
    case Tstruct:
      if (is_external(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 int SOAP_FMAC2 soap_out_%s(struct soap*, const char*, int, const %s, const char*);", c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap*, const char*, int, const %s, const char*);", c_ident(typ), c_type_id(typ, "*"));
      fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap *soap, const char *tag, int id, const %s, const char *type)\n{", c_ident(typ), c_type_id(typ, "*a"));
      table = (Table*)typ->ref;
      for (t = table; t; t = t->prev)
      {
        for (p = t->list; p; p = p->next)
        {
          if (is_repetition(p) || is_anytype(p) || is_choice(p))
            p = p->next;
          else if (is_transient(p->info.typ))
            ;
          else if (p->info.sto & Sattribute)
            ;
          else if (is_qname(p->info.typ))
            fprintf(fout, "\n\tconst char *soap_tmp_%s;", ident(p->sym->name));
          else if (is_stdqname(p->info.typ))
            fprintf(fout, "\n\tstd::string soap_tmp_%s;", ident(p->sym->name));
          else if ((p->info.typ->type == Tpointer || is_smart_shared(p->info.typ)) && is_qname((Tnode*)p->info.typ->ref))
            fprintf(fout, "\n\tconst char *soap_tmp_%s;", ident(p->sym->name));
          else if (is_smart(p->info.typ) && is_qname((Tnode*)p->info.typ->ref))
            fprintf(fout, "\n\tconst char *soap_tmp_%s;", ident(p->sym->name));
        }
      }
      if (has_setter(typ))
        fprintf(fout, "\n\tif (((%s)a)->set(soap))\n\t\treturn soap->error;", c_type_id(typ, "*"));
      if (table && !is_invisible(typ->id->name))
      {
        for (p = table->list; p; p = p->next)
        {
          if (is_pointer_to_derived(p))
            fprintf(fout, "\n\tif (a->%s)\n\t\treturn soap_out_%s(soap, tag, id, a->%s, \"%s\");", ident(p->sym->name), c_ident(p->info.typ->ref), ident(p->sym->name), xsi_type(p->info.typ->ref));
        }
      }
      x = xsi_type_u(typ);
      if (x && *x)
        fprintf(fout, "\n\tif (!type)\n\t\ttype = \"%s\";", x);
      for (t = table; t; t = t->prev)
      {
        for (p = t->list; p; p = p->next)
        {
          if (is_repetition(p) || is_anytype(p) || is_choice(p))
            p = p->next;
          else if (is_transient(p->info.typ))
            ;
          else if (p->info.sto & Sattribute)
            soap_set_attr(p, "a", ident(p->sym->name), ns_add(p, nsa));
          else if (is_qname(p->info.typ))
            fprintf(fout, "\n\tsoap_tmp_%s = soap_QName2s(soap, a->%s);", ident(p->sym->name), ident(p->sym->name));
          else if (is_stdqname(p->info.typ))
            fprintf(fout, "\n\tsoap_tmp_%s = soap_QName2s(soap, a->%s.c_str());", ident(p->sym->name), ident(p->sym->name));
          else if ((p->info.typ->type == Tpointer || is_smart_shared(p->info.typ)) && is_qname((Tnode*)p->info.typ->ref))
            fprintf(fout, "\n\tsoap_tmp_%s = a->%s ? soap_QName2s(soap, *a->%s) : NULL;", ident(p->sym->name), ident(p->sym->name), ident(p->sym->name));
          else if (is_smart(p->info.typ) && is_qname((Tnode*)p->info.typ->ref))
            fprintf(fout, "\n\tsoap_tmp_%s = a->%s.get() ? soap_QName2s(soap, *a->%s) : NULL;", ident(p->sym->name), ident(p->sym->name), ident(p->sym->name));
          else if ((p->info.typ->type == Tpointer || is_smart_shared(p->info.typ)) && is_stdqname((Tnode*)p->info.typ->ref))
            fprintf(fout, "\n\tstd::string soap_temp_%s(a->%s ? soap_QName2s(soap, a->%s->c_str()) : \"\"), *soap_tmp_%s = a->%s ? &soap_temp_%s : NULL;", ident(p->sym->name), ident(p->sym->name), ident(p->sym->name), ident(p->sym->name), ident(p->sym->name), ident(p->sym->name));
          else if (is_smart(p->info.typ) && is_stdqname((Tnode*)p->info.typ->ref))
            fprintf(fout, "\n\tstd::string soap_temp_%s(a->%s.get() ? soap_QName2s(soap, a->%s->c_str()) : \"\"), *soap_tmp_%s = a->%s.get() ? &soap_temp_%s : NULL;", ident(p->sym->name), ident(p->sym->name), ident(p->sym->name), ident(p->sym->name), ident(p->sym->name), ident(p->sym->name));
        }
      }
      fprintf(fout, "\n\t(void)soap; (void)tag; (void)id; (void)a; (void)type; /* appease -Wall -Werror */");
      if (!table)
      {
        fprintf(fout, "\n\treturn SOAP_OK;\n}"); 
      }
      else if (is_primclass(typ))
      {
        for (table = (Table*)typ->ref; table; table = table->prev)
        {
          p = table->list;
          while (p && !is_item(p))
            p = p->next;
          if (p)
            break;
        }
        if ((p->info.sto & SmustUnderstand) && !(p->info.sto & (Sconst | Sprivate | Sprotected)) && !(p->info.sto & Sattribute) && !is_transient(p->info.typ) && !is_void(p->info.typ) && p->info.typ->type != Tfun)
          fprintf(fout, "\n\tsoap->mustUnderstand = 1;");
        if (p->info.typ->type == Tarray)
          fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, a->%s, \"%s\");", c_ident(p->info.typ), ident(p->sym->name), xsi_type_u(typ));
        else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
          fprintf(fout, "\n\treturn a->%s.soap_out(soap, tag, id, \"%s\");", ident(p->sym->name), xsi_type_u(typ));
        else if (is_qname(p->info.typ))
          fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, (char*const*)(void*)&soap_tmp_%s, \"%s\");", c_ident(p->info.typ), ident(p->sym->name), xsi_type_u(typ));
        else if (is_stdqname(p->info.typ))
          fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, &soap_tmp_%s, \"%s\");", c_ident(p->info.typ), ident(p->sym->name), xsi_type_u(typ));
        else if ((p->info.typ->type == Tpointer || is_smart(p->info.typ)) && is_qname((Tnode*)p->info.typ->ref))
          fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, (char*const*)(void)&soap_tmp_%s, \"%s\");", c_ident((Tnode*)p->info.typ->ref), ident(p->sym->name), xsi_type_u(typ->ref));
        else if ((p->info.typ->type == Tpointer || is_smart(p->info.typ)) && is_stdqname((Tnode*)p->info.typ->ref))
          fprintf(fout, "\n\tif (soap_tmp_%s)\n\t\treturn soap_out_%s(soap, tag, id, soap_tmp_%s, \"%s\");", ident(p->sym->name), c_ident(p->info.typ->ref), ident(p->sym->name), xsi_type_u(typ->ref));
        else if (is_XML(p->info.typ) && is_string(p->info.typ))
          fprintf(fout, "\n\treturn soap_outliteral(soap, tag, (char*const*)&a->%s, NULL);", ident(p->sym->name));
        else if (is_XML(p->info.typ) && is_wstring(p->info.typ))
          fprintf(fout, "\n\treturn soap_outwliteral(soap, tag, (wchar_t*const*)&a->%s, NULL);", ident(p->sym->name));
        else if (is_string(p->info.typ))
          fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, (char*const*)&a->%s, \"%s\");", c_ident(p->info.typ), ident(p->sym->name), xsi_type_u(typ));
        else if (is_wstring(p->info.typ))
          fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, (wchar_t*const*)&a->%s, \"%s\");", c_ident(p->info.typ), ident(p->sym->name), xsi_type_u(typ));
        else if (p->info.typ->type != Tfun && !is_void(p->info.typ))
          fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, &a->%s, \"%s\");", c_ident(p->info.typ), ident(p->sym->name), xsi_type_u(typ));
        else
          fprintf(fout, "\n\treturn SOAP_OK;"); 
        fprintf(fout, "\n}");
      }
      else
      {
        if (!is_invisible(typ->id->name))
          fprintf(fout, "\n\tif (soap_element_begin_out(soap, tag, soap_embedded_id(soap, id, a, %s), type))\n\t\treturn soap->error;", soap_type(typ));
        fflush(fout);
        for (t = table; t; t = t->prev)
        {
          for (p = t->list; p; p = p->next)
          {
            if ((p->info.sto & Sreturn))
            {
              if (nse || has_ns_eq(NULL, p->sym->name))
              {
                if (p->info.typ->type == Tpointer)
                  fprintf(fout, "\n\tif (a->%s)\n\t\tsoap_element_result(soap, \"%s\");", ident(p->sym->name), ns_add(p, nse));
                else
                  fprintf(fout, "\n\tsoap_element_result(soap, \"%s\");", ns_add(p, nse));
              }
            }
            if ((p->info.sto & SmustUnderstand) && !(p->info.sto & (Sconst | Sprivate | Sprotected)) && !is_transient(p->info.typ) && !is_void(p->info.typ) && p->info.typ->type != Tfun)
              fprintf(fout, "\n\tsoap->mustUnderstand = 1;");
            needs_lang(p);
            if (p->info.typ->type == Tfun || p->info.typ->type == Tunion || is_soapref(p->info.typ))
              ;
            else if ((p->info.sto & (Sconst | Sprivate | Sprotected)))
              fprintf(fout, "\n\t/* non-serializable %s skipped */", ident(p->sym->name));
            else if ((p->info.sto & Sattribute))
              ;
            else if (is_repetition(p))
            {
              fprintf(fout, "\n\tif (a->%s)", ident(p->next->sym->name));
              fprintf(fout, "\n\t{\tint i;\n\t\tfor (i = 0; i < (int)a->%s; i++)", ident(p->sym->name));
              if (((Tnode*)p->next->info.typ->ref)->type == Tclass && !is_external((Tnode*)p->next->info.typ->ref) && !is_volatile((Tnode*)p->next->info.typ->ref) && !is_typedef((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\tif (a->%s[i].soap_out(soap, \"%s\", -1, \"%s\"))\n\t\t\t\treturn soap->error;", ident(p->next->sym->name), ns_add(p->next, nse), xsi_type_u((Tnode*)p->next->info.typ->ref));
              else if (is_qname((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t{\tconst char *soap_tmp_%s = soap_QName2s(soap, a->%s[i]);\n\t\t\tif (soap_out_%s(soap, \"%s\", -1, (char*const*)(void*)&soap_tmp_%s, \"%s\"))\n\t\t\t\treturn soap->error;\n\t\t}", ident(p->next->sym->name), ident(p->next->sym->name), c_ident((Tnode*)p->next->info.typ->ref), ns_add(p->next, nse), ident(p->next->sym->name), xsi_type_u((Tnode*)p->next->info.typ->ref));
              else if (is_XML((Tnode*)p->next->info.typ->ref) && is_string((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\tif (soap_outliteral(soap, \"%s\", (char*const*)(a->%s + i), NULL))\n\t\t\t\treturn soap->error;", ns_add(p->next, nse), ident(p->next->sym->name));
              else if (is_XML((Tnode*)p->next->info.typ->ref) && is_wstring((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\tif (soap_outwliteral(soap, \"%s\", (wchar_t*const*)(a->%s + i), NULL))\n\t\t\t\treturn soap->error;", ns_add(p->next, nse), ident(p->next->sym->name));
              else if (is_string((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\tif (soap_out_string(soap, \"%s\", -1, (char*const*)(a->%s + i), \"%s\"))\n\t\t\t\treturn soap->error;", ns_add(p->next, nse), ident(p->next->sym->name), xsi_type_u((Tnode*)p->next->info.typ->ref));
              else if (is_wstring((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\tif (soap_out_wstring(soap, \"%s\", -1, (wchar_t*const*)(a->%s + i), \"%s\"))\n\t\t\t\treturn soap->error;", ns_add(p->next, nse), ident(p->next->sym->name), xsi_type_u((Tnode*)p->next->info.typ->ref));
              else
                fprintf(fout, "\n\t\t\tif (soap_out_%s(soap, \"%s\", -1, a->%s + i, \"%s\"))\n\t\t\t\treturn soap->error;", c_ident((Tnode*)p->next->info.typ->ref), ns_add(p->next, nse), ident(p->next->sym->name), xsi_type_u((Tnode*)p->next->info.typ->ref));
              fprintf(fout, "\n\t}");
              p = p->next;
            }
            else if (is_anytype(p) && is_invisible(p->next->sym->name))
            {
              fprintf(fout, "\n\tif (soap_putelement(soap, a->%s, tag, -1, a->%s))\n\t\treturn soap->error;", ident(p->next->sym->name), ident(p->sym->name));
              p = p->next;
            }
            else if (is_anytype(p))
            {
              fprintf(fout, "\n\tif (soap_putelement(soap, a->%s, \"%s\", -1, a->%s))\n\t\treturn soap->error;", ident(p->next->sym->name), ns_add(p->next, nse), ident(p->sym->name));
              p = p->next;
            }
            else if (is_choice(p))
            {
              fprintf(fout, "\n\tif (soap_out_%s(soap, a->%s, &a->%s))\n\t\treturn soap->error;", c_ident(p->next->info.typ), ident(p->sym->name), ident(p->next->sym->name));
              p = p->next;
            }
            else if (is_transient(p->info.typ))
            {
              if (!is_pointer_to_derived(p))
                fprintf(fout, "\n\t/* transient %s skipped */", ident(p->sym->name));
            }
            else if (p->info.typ->type == Tarray)
              fprintf(fout, "\n\tif (soap_out_%s(soap, %s, -1, a->%s, \"%s\"))\n\t\treturn soap->error;", c_ident(p->info.typ), field(p, nse), ident(p->sym->name), xsi_type_u(p->info.typ));
            else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
              fprintf(fout, "\n\tif (a->%s.soap_out(soap, %s, -1, \"%s\"))\n\t\treturn soap->error;", ident(p->sym->name), field(p, nse), xsi_type_u(p->info.typ));
            else if (is_stdqname(p->info.typ))
              fprintf(fout, "\n\tif (soap_out_%s(soap, %s, -1, &soap_tmp_%s, \"%s\"))\n\t\treturn soap->error;", c_ident(p->info.typ), field(p, nse), ident(p->sym->name), xsi_type_u(p->info.typ));
            else
            {
              if (zflag != 2 && p->info.typ->type == Tpointer && !is_void(p->info.typ) && p->info.minOccurs > 0)
              {
                /* xsi:nil only if nillable, otherwise empty element */
                if (p->info.nillable)
                  fprintf(fout, "\n\tif (!a->%s)\n\t{\tif (soap_element_nil(soap, %s))\n\t\t\treturn soap->error;\n\t}\n\telse ", ident(p->sym->name), field(p, nse));
                else
                  fprintf(fout, "\n\tif (!a->%s)\n\t{\tif (soap_element_empty(soap, %s, 0, NULL))\n\t\t\treturn soap->error;\n\t}\n\telse ", ident(p->sym->name), field(p, nse));
              }
              else
              {
                fprintf(fout, "\n\t");
              }
              if (is_qname(p->info.typ))
                fprintf(fout, "if (soap_out_%s(soap, %s, -1, (char*const*)(void*)&soap_tmp_%s, \"%s\"))\n\t\treturn soap->error;", c_ident(p->info.typ), field(p, nse), ident(p->sym->name), xsi_type_u(p->info.typ));
              else if ((p->info.typ->type == Tpointer || is_smart(p->info.typ)) && is_qname((Tnode*)p->info.typ->ref))
                fprintf(fout, "if (soap_out_%s(soap, %s, -1, (char*const*)(void*)&soap_tmp_%s, \"%s\"))\n\t\treturn soap->error;", c_ident(p->info.typ->ref), field(p, nse), ident(p->sym->name), xsi_type_u(p->info.typ->ref));
              else if ((p->info.typ->type == Tpointer || is_smart(p->info.typ)) && is_stdqname((Tnode*)p->info.typ->ref))
                fprintf(fout, "if (soap_tmp_%s && soap_out_%s(soap, %s, -1, soap_tmp_%s, \"%s\"))\n\t\treturn soap->error;", ident(p->sym->name), c_ident(p->info.typ->ref), field(p, nse), ident(p->sym->name), xsi_type_u(p->info.typ->ref));
              else if (is_XML(p->info.typ) && is_string(p->info.typ))
                fprintf(fout, "if (soap_outliteral(soap, %s, (char*const*)&a->%s, NULL))\n\t\treturn soap->error;", field(p, nse), ident(p->sym->name));
              else if (is_XML(p->info.typ) && is_wstring(p->info.typ))
                fprintf(fout, "if (soap_outwliteral(soap, %s, (wchar_t*const*)&a->%s, NULL))\n\t\treturn soap->error;", field(p, nse), ident(p->sym->name));
              else if (is_string(p->info.typ))
                fprintf(fout, "if (soap_out_%s(soap, %s, -1, (char*const*)&a->%s, \"%s\"))\n\t\treturn soap->error;", c_ident(p->info.typ), field(p, nse), ident(p->sym->name), xsi_type_u(p->info.typ));
              else if (is_wstring(p->info.typ))
                fprintf(fout, "if (soap_out_%s(soap, %s, -1, (wchar_t*const*)&a->%s, \"%s\"))\n\t\treturn soap->error;", c_ident(p->info.typ), field(p, nse), ident(p->sym->name), xsi_type_u(p->info.typ));
              else if (p->info.typ->type != Tfun && !is_void(p->info.typ))
                fprintf(fout, "if (soap_out_%s(soap, %s, -1, &a->%s, \"%s\"))\n\t\treturn soap->error;", c_ident(p->info.typ), field(p, nse), ident(p->sym->name), xsi_type_u(p->info.typ));
              else
                fprintf(fout, "{ }");
            }
          }
        }
        if (!is_invisible(typ->id->name))
          fprintf(fout, "\n\treturn soap_element_end_out(soap, tag);\n}");
        else
          fprintf(fout, "\n\treturn SOAP_OK;\n}");
      }
      fflush(fout);
      break;
    case Tclass:
      if (is_external(typ) && !is_volatile(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 int SOAP_FMAC2 soap_out_%s(struct soap*, const char*, int, const %s, const char*);", c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      if (!is_volatile(typ) && !is_typedef(typ))
      {
        if (is_external(typ))
        {
          fprintf(fhead, "\nSOAP_FMAC1 int SOAP_FMAC2 soap_out_%s(struct soap*, const char*, int, const %s, const char*);", c_ident(typ), c_type_id(typ, "*"));
          return;
        }
        fprintf(fout, "\n\nint %s::soap_out(struct soap *soap, const char *tag, int id, const char *type) const", ident(typ->id->name));
        fprintf(fout, "\n{\n\treturn soap_out_%s(soap, tag, id, this, type);\n}", c_ident(typ));
      }
      fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap*, const char*, int, const %s, const char*);", c_ident(typ), c_type_id(typ, "*"));
      fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap *soap, const char *tag, int id, const %s, const char *type)\n{", c_ident(typ), c_type_id(typ, "*a"));
      fflush(fout);
      if (has_setter(typ))
        fprintf(fout, "\n\tif (((%s)a)->set(soap))\n\t\treturn soap->error;", c_type_id(typ, "*"));
      table = (Table*)typ->ref;
      if (table && !is_invisible(typ->id->name))
      {
        for (p = table->list; p; p = p->next)
        {
          if (is_pointer_to_derived(p))
            fprintf(fout, "\n\tif (a->%s)\n\t\treturn soap_out_%s(soap, tag, id, a->%s, \"%s\");", ident(p->sym->name), c_ident(p->info.typ->ref), ident(p->sym->name), xsi_type(p->info.typ->ref));
        }
      }
      x = xsi_type_u(typ);
      if (x && *x)
        fprintf(fout, "\n\tif (!type)\n\t\ttype = \"%s\";", x);
      for (t = table; t; t = t->prev)
      {
        Entry *e = entry(classtable, t->sym);
        const char *nsa1 = e ? ns_qualifiedAttribute(e->info.typ) : nsa;
        for (p = t->list; p; p = p->next)
        {
          if (is_repetition(p) || is_anytype(p) || is_choice(p))
            p = p->next;
          else if (is_transient(p->info.typ))
            ;
          else if (p->info.sto & Sattribute)
            soap_set_attr(p, ptr_cast(t, "a"), ident(p->sym->name), ns_add(p, nsa1));
          else if (is_qname(p->info.typ))
            fprintf(fout, "\n\tconst char *soap_tmp_%s = soap_QName2s(soap, a->%s);", ident(p->sym->name), ident(p->sym->name));
          else if (is_stdqname(p->info.typ))
            fprintf(fout, "\n\tstd::string soap_tmp_%s(soap_QName2s(soap, a->%s.c_str()));", ident(p->sym->name), ident(p->sym->name));
          else if ((p->info.typ->type == Tpointer || is_smart_shared(p->info.typ)) && is_qname((Tnode*)p->info.typ->ref))
            fprintf(fout, "\n\tconst char *soap_tmp_%s = a->%s ? soap_QName2s(soap, *a->%s) : NULL;", ident(p->sym->name), ident(p->sym->name), ident(p->sym->name));
          else if (is_smart(p->info.typ) && is_qname((Tnode*)p->info.typ->ref))
            fprintf(fout, "\n\tconst char *soap_tmp_%s = a->%s.get() ? soap_QName2s(soap, *a->%s) : NULL;", ident(p->sym->name), ident(p->sym->name), ident(p->sym->name));
          else if ((p->info.typ->type == Tpointer || is_smart_shared(p->info.typ)) && is_stdqname((Tnode*)p->info.typ->ref))
            fprintf(fout, "\n\tstd::string soap_temp_%s(a->%s ? soap_QName2s(soap, a->%s->c_str()) : \"\"), *soap_tmp_%s = a->%s ? &soap_temp_%s : NULL;", ident(p->sym->name), ident(p->sym->name), ident(p->sym->name), ident(p->sym->name), ident(p->sym->name), ident(p->sym->name));
          else if (is_smart(p->info.typ) && is_stdqname((Tnode*)p->info.typ->ref))
            fprintf(fout, "\n\tstd::string soap_temp_%s(a->%s.get() ? soap_QName2s(soap, a->%s->c_str()) : \"\"), *soap_tmp_%s = a->%s.get() ? &soap_temp_%s : NULL;", ident(p->sym->name), ident(p->sym->name), ident(p->sym->name), ident(p->sym->name), ident(p->sym->name), ident(p->sym->name));
        }
      }
      fprintf(fout, "\n\t(void)soap; (void)tag; (void)id; (void)a; (void)type; /* appease -Wall -Werror */");
      if (!table)
      {
        fprintf(fout, "\n\treturn SOAP_OK;\n}"); 
      }
      else if (is_primclass(typ))
      {
        for (t = table; t; t = t->prev)
        {
          p = t->list;
          while (p && !is_item(p))
            p = p->next;
          if (p)
            break;
        }
        if ((p->info.sto & SmustUnderstand) && !(p->info.sto & (Sconst | Sprivate | Sprotected)) && !(p->info.sto & Sattribute) && !is_transient(p->info.typ) && !is_void(p->info.typ) && p->info.typ->type != Tfun)
          fprintf(fout, "\n\tsoap->mustUnderstand = 1;");
        if (table && table->prev)
        {
          if (is_XML(p->info.typ) && is_string(p->info.typ))
            fprintf(fout, "\n\treturn soap_outliteral(soap, tag, (char*const*)&a->%s::%s, \"%s\");", ident(t->sym->name), ident(p->sym->name), xsi_type(typ));
          else if (is_XML(p->info.typ) && is_wstring(p->info.typ))
            fprintf(fout, "\n\treturn soap_outwliteral(soap, tag, (wchar_t*const*)&a->%s::%s, \"%s\");", ident(t->sym->name), ident(p->sym->name), xsi_type(typ));
          else if (is_qname(p->info.typ))
            fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, (char*const*)(void*)&soap_tmp_%s, \"%s\");", c_ident(p->info.typ), ident(p->sym->name), xsi_type(typ));
          else if (is_string(p->info.typ))
            fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, (char*const*)&a->%s::%s, \"%s\");", c_ident(p->info.typ), ident(t->sym->name), ident(p->sym->name), xsi_type(typ));
          else if (is_wstring(p->info.typ))
            fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, (wchar_t*const*)&a->%s::%s, \"%s\");", c_ident(p->info.typ), ident(t->sym->name), ident(p->sym->name), xsi_type(typ));
          else if (p->info.typ->type == Tarray)
            fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, a->%s::%s, \"%s\");", c_ident(p->info.typ), ident(t->sym->name), ident(p->sym->name), xsi_type(typ));
          else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
            fprintf(fout, "\n\treturn (a->%s::%s).soap_out(soap, tag, id, \"%s\");", ident(t->sym->name), ident(p->sym->name), xsi_type(typ));
          else if (is_stdqname(p->info.typ))
            fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, &soap_tmp_%s, \"%s\");", c_ident(p->info.typ), ident(p->sym->name), xsi_type(typ));
          else if ((p->info.typ->type == Tpointer || is_smart(p->info.typ)) && is_qname((Tnode*)p->info.typ->ref))
            fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, (char*const*)(void*)&soap_tmp_%s, \"%s\");", c_ident((Tnode*)p->info.typ->ref), ident(p->sym->name), xsi_type_u(typ->ref));
          else if ((p->info.typ->type == Tpointer || is_smart(p->info.typ)) && is_stdqname((Tnode*)p->info.typ->ref))
            fprintf(fout, "\n\tif (soap_tmp_%s)\n\t\treturn soap_out_%s(soap, tag, id, soap_tmp_%s, \"%s\");", ident(p->sym->name), c_ident(p->info.typ->ref), ident(p->sym->name), xsi_type_u(typ->ref));
          else if (p->info.typ->type != Tfun && !is_void(p->info.typ))
            fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, &a->%s::%s, \"%s\");", c_ident(p->info.typ), ident(t->sym->name), ident(p->sym->name), xsi_type(typ));
          else
            fprintf(fout, "\n\treturn SOAP_OK;");
        }
        else
        {
          if (is_XML(p->info.typ) && is_string(p->info.typ))
            fprintf(fout, "\n\treturn soap_outliteral(soap, tag, (char*const*)&a->%s::%s, NULL);", ident(t->sym->name), ident(p->sym->name));
          else if (is_XML(p->info.typ) && is_wstring(p->info.typ))
            fprintf(fout, "\n\treturn soap_outwliteral(soap, tag, (wchar_t*const*)&a->%s::%s, NULL);", ident(t->sym->name), ident(p->sym->name));
          else if (is_qname(p->info.typ))
            fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, (char*const*)(void*)&soap_tmp_%s, \"%s\");", c_ident(p->info.typ), ident(p->sym->name), xsi_type_u(typ));
          else if (is_string(p->info.typ))
            fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, (char*const*)&a->%s::%s, \"%s\");", c_ident(p->info.typ), ident(t->sym->name), ident(p->sym->name), xsi_type_u(typ));
          else if (is_wstring(p->info.typ))
            fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, (wchar_t*const*)&a->%s::%s, \"%s\");", c_ident(p->info.typ), ident(t->sym->name), ident(p->sym->name), xsi_type_u(typ));
          else if (p->info.typ->type == Tarray)
            fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, a->%s::%s, \"%s\");", c_ident(p->info.typ), ident(t->sym->name), ident(p->sym->name), xsi_type_u(typ));
          else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
            fprintf(fout, "\n\treturn (a->%s::%s).soap_out(soap, tag, id, \"%s\");", ident(t->sym->name), ident(p->sym->name), xsi_type_u(typ));
          else if (is_stdqname(p->info.typ))
            fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, &soap_tmp_%s, \"%s\");", c_ident(p->info.typ), ident(p->sym->name), xsi_type_u(typ));
          else if ((p->info.typ->type == Tpointer || is_smart(p->info.typ)) && is_qname((Tnode*)p->info.typ->ref))
            fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, (char*const*)(void*)&soap_tmp_%s, \"%s\");", c_ident((Tnode*)p->info.typ->ref), ident(p->sym->name), xsi_type_u(typ->ref));
          else if ((p->info.typ->type == Tpointer || is_smart(p->info.typ)) && is_stdqname((Tnode*)p->info.typ->ref))
            fprintf(fout, "\n\tif (soap_tmp_%s)\n\t\treturn soap_out_%s(soap, tag, id, soap_tmp_%s, \"%s\");", ident(p->sym->name), c_ident(p->info.typ->ref), ident(p->sym->name), xsi_type_u(typ->ref));
          else if (p->info.typ->type != Tfun && !is_void(p->info.typ))
            fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, &a->%s::%s, \"%s\");", c_ident(p->info.typ), ident(t->sym->name), ident(p->sym->name), xsi_type_u(typ));
          else
            fprintf(fout, "\n\treturn SOAP_OK;");
        }
        fprintf(fout, "\n}");
      }
      else
      {
        if (!is_invisible(typ->id->name))
        {
          if (table && table->prev)
            fprintf(fout, "\n\tif (soap_element_begin_out(soap, tag, soap_embedded_id(soap, id, a, %s), type ? type : \"%s\"))\n\t\treturn soap->error;", soap_type(typ), xsi_type(typ));
          else
            fprintf(fout, "\n\tif (soap_element_begin_out(soap, tag, soap_embedded_id(soap, id, a, %s), type))\n\t\treturn soap->error;", soap_type(typ));
        }
        fflush(fout);
        i = 0;
        /* Get the depth of the inheritance hierarchy */
        for (t = table; t; t = t->prev)
          i++;
        /* Call routines to output the member data of the class */
        /* Data members of the Base Classes are outputed first
           followed by the data members of the Derived classes.
           Overridden data members are output twice once for the base class
           they are defined in and once for the derived class that overwrites
           them */
        for (; i > 0; i--)
        {
          Entry *e;
          const char *nse1;
          t = table;
          for (j = 0; j < i-1; j++)
            t = t->prev;
          e = entry(classtable, t->sym);
          nse1 = e ? ns_qualifiedElement(e->info.typ) : nse;
          for (p = t->list; p != (Entry*) 0; p = p->next)
          {
            if (p->info.sto & Sreturn)
            {
              if (nse1 || has_ns_eq(NULL, p->sym->name))
              {
                if (p->info.typ->type == Tpointer)
                  fprintf(fout, "\n\tif (a->%s)\n\t\tsoap_element_result(soap, \"%s\");", ident(p->sym->name), ns_add(p, nse1));
                else
                  fprintf(fout, "\n\tsoap_element_result(soap, \"%s\");", ns_add(p, nse1));
              }
            }
            if ((p->info.sto & SmustUnderstand) && !(p->info.sto & (Sconst | Sprivate | Sprotected)) && !(p->info.sto & Sattribute) && !is_transient(p->info.typ) && !is_void(p->info.typ) && p->info.typ->type != Tfun)
              fprintf(fout, "\n\tsoap->mustUnderstand = 1;");
            needs_lang(p);
            if (p->info.typ->type == Tfun || p->info.typ->type == Tunion || is_soapref(p->info.typ))
              ;
            else if (p->info.sto & (Sconst | Sprivate | Sprotected))
              fprintf(fout, "\n\t/* non-serializable %s skipped */", ident(p->sym->name));
            else if (p->info.sto & Sattribute)
              ;
            else if (is_repetition(p))
            {
              fprintf(fout, "\n\tif (a->%s::%s)", ident(t->sym->name), ident(p->next->sym->name));
              fprintf(fout, "\n\t{\tint i;\n\t\tfor (i = 0; i < (int)a->%s::%s; i++)", ident(t->sym->name), ident(p->sym->name));
              if (((Tnode*)p->next->info.typ->ref)->type == Tclass && !is_external((Tnode*)p->next->info.typ->ref) && !is_volatile((Tnode*)p->next->info.typ->ref) && !is_typedef((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\tif (a->%s::%s[i].soap_out(soap, %s, -1, \"%s\"))\n\t\t\t\treturn soap->error;", ident(t->sym->name), ident(p->next->sym->name), field_overridden(t, p->next, nse1), xsi_type_u((Tnode*)p->next->info.typ->ref));
              else if (is_qname((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t{\tconst char *soap_tmp_%s = soap_QName2s(soap, a->%s[i]);\n\t\t\tif (soap_out_%s(soap, %s, -1, (char*const*)(void*)&soap_tmp_%s, \"%s\"))\n\t\t\t\treturn soap->error;\n\t\t}", ident(p->next->sym->name), ident(p->next->sym->name), c_ident((Tnode*)p->next->info.typ->ref), field_overridden(t, p->next, nse1), ident(p->next->sym->name), xsi_type_u((Tnode*)p->next->info.typ->ref));
              else if (is_XML((Tnode*)p->next->info.typ->ref) && is_string((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\tif (soap_outliteral(soap, %s, (char*const*)(a->%s::%s + i), NULL))\n\t\t\t\treturn soap->error;", field_overridden(t, p->next, nse1), ident(t->sym->name), ident(p->next->sym->name));
              else if (is_XML((Tnode*)p->next->info.typ->ref) && is_wstring((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\tif (soap_outwliteral(soap, %s, (wchar_t*const*)(a->%s::%s + i), NULL))\n\t\t\t\treturn soap->error;", field_overridden(t, p->next, nse1), ident(t->sym->name), ident(p->next->sym->name));
              else if (is_string((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\tif (soap_out_string(soap, \"%s\", -1, (char*const*)(a->%s::%s + i), \"%s\"))\n\t\t\t\treturn soap->error;", ns_add(p->next, nse), ident(t->sym->name), ident(p->next->sym->name), xsi_type_u((Tnode*)p->next->info.typ->ref));
              else if (is_wstring((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\tif (soap_out_wstring(soap, \"%s\", -1, (wchar_t*const*)(a->%s::%s + i), \"%s\"))\n\t\t\t\treturn soap->error;", ns_add(p->next, nse), ident(t->sym->name), ident(p->next->sym->name), xsi_type_u((Tnode*)p->next->info.typ->ref));
              else
                fprintf(fout, "\n\t\t\tif (soap_out_%s(soap, %s, -1, a->%s::%s + i, \"%s\"))\n\t\t\t\treturn soap->error;", c_ident((Tnode*)p->next->info.typ->ref), field_overridden(t, p->next, nse1), ident(t->sym->name), ident(p->next->sym->name), xsi_type_u((Tnode*)p->next->info.typ->ref));
              fprintf(fout, "\n\t}");
              p = p->next;
            }
            else if (is_anytype(p) && is_invisible(p->next->sym->name))
            {
              fprintf(fout, "\n\tif (soap_putelement(soap, a->%s::%s, tag, -1, a->%s::%s))\n\t\treturn soap->error;", ident(t->sym->name), ident(p->next->sym->name), ident(t->sym->name), ident(p->sym->name));
              p = p->next;
            }
            else if (is_anytype(p))
            {
              fprintf(fout, "\n\tif (soap_putelement(soap, a->%s::%s, %s, -1, a->%s::%s))\n\t\treturn soap->error;", ident(t->sym->name), ident(p->next->sym->name), field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name));
              p = p->next;
            }
            else if (is_choice(p))
            {
              fprintf(fout, "\n\tif (soap_out_%s(soap, a->%s::%s, &a->%s::%s))\n\t\treturn soap->error;", c_ident(p->next->info.typ), ident(t->sym->name), ident(p->sym->name), ident(t->sym->name), ident(p->next->sym->name));
              p = p->next;
            }
            else if (is_item(p))
            {
            }
            else if (is_transient(p->info.typ))
            {
              if (!is_pointer_to_derived(p))
                fprintf(fout, "\n\t/* transient %s skipped */", ident(p->sym->name));
            }
            else if (p->info.typ->type == Tarray)
              fprintf(fout, "\n\tif (soap_out_%s(soap, %s, -1, a->%s::%s, \"%s\"))\n\t\treturn soap->error;", c_ident(p->info.typ), field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name), xsi_type_u(p->info.typ));
            else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
              fprintf(fout, "\n\tif ((a->%s::%s).soap_out(soap, %s, -1, \"%s\"))\n\t\treturn soap->error;", ident(t->sym->name), ident(p->sym->name), field_overridden(t, p, nse1), xsi_type_u(p->info.typ));
            else if (is_stdqname(p->info.typ))
              fprintf(fout, "\n\tif (soap_out_%s(soap, %s, -1, &soap_tmp_%s, \"%s\"))\n\t\treturn soap->error;", c_ident(p->info.typ), field_overridden(t, p, nse1), ident(p->sym->name), xsi_type_u(p->info.typ));
            else
            {
              if (zflag != 2 && p->info.typ->type == Tpointer && !is_void(p->info.typ) && p->info.minOccurs > 0)
              {
                /* xsi:nil only if nillable, otherwise empty element */
                if (p->info.nillable)
                  fprintf(fout, "\n\tif (!a->%s::%s)\n\t{\tif (soap_element_nil(soap, %s))\n\t\t\treturn soap->error;\n\t}\n\telse ", ident(t->sym->name), ident(p->sym->name), field_overridden(t, p, nse1));
                else
                  fprintf(fout, "\n\tif (!a->%s::%s)\n\t{\tif (soap_element_empty(soap, %s, 0, NULL))\n\t\t\treturn soap->error;\n\t}\n\telse ", ident(t->sym->name), ident(p->sym->name), field_overridden(t, p, nse1));
              }
              else
              {
                fprintf(fout, "\n\t");
              }
              if (is_qname(p->info.typ))
                fprintf(fout, "if (soap_out_%s(soap, %s, -1, (char*const*)(void*)&soap_tmp_%s, \"%s\"))\n\t\treturn soap->error;", c_ident(p->info.typ), field_overridden(t, p, nse1), ident(p->sym->name), xsi_type_u(p->info.typ));
              else if ((p->info.typ->type == Tpointer || is_smart(p->info.typ)) && is_qname((Tnode*)p->info.typ->ref))
                fprintf(fout, "if (soap_out_%s(soap, %s, -1, (char*const*)(void*)&soap_tmp_%s, \"%s\"))\n\t\treturn soap->error;", c_ident((Tnode*)p->info.typ->ref), field_overridden(t, p, nse1), ident(p->sym->name), xsi_type_u(p->info.typ->ref));
              else if ((p->info.typ->type == Tpointer || is_smart(p->info.typ)) && is_stdqname((Tnode*)p->info.typ->ref))
                fprintf(fout, "if (soap_tmp_%s && soap_out_%s(soap, %s, -1, soap_tmp_%s, \"%s\"))\n\t\treturn soap->error;", ident(p->sym->name), c_ident(p->info.typ->ref), field_overridden(t, p, nse1), ident(p->sym->name), xsi_type_u(p->info.typ->ref));
              else if (is_XML(p->info.typ) && is_string(p->info.typ))
                fprintf(fout, "if (soap_outliteral(soap, %s, (char*const*)&a->%s::%s, NULL))\n\t\treturn soap->error;", field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name));
              else if (is_XML(p->info.typ) && is_wstring(p->info.typ))
                fprintf(fout, "if (soap_outwliteral(soap, %s, (wchar_t*const*)&a->%s::%s, NULL))\n\t\treturn soap->error;", field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name));
              else if (is_string(p->info.typ))
                fprintf(fout, "if (soap_out_%s(soap, %s, -1, (char*const*)&a->%s::%s, \"%s\"))\n\t\treturn soap->error;", c_ident(p->info.typ), field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name), xsi_type_u(p->info.typ));
              else if (is_wstring(p->info.typ))
                fprintf(fout, "if (soap_out_%s(soap, %s, -1, (wchar_t*const*)&a->%s::%s, \"%s\"))\n\t\treturn soap->error;", c_ident(p->info.typ), field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name), xsi_type_u(p->info.typ));
              else if (p->info.typ->type != Tfun && !is_void(p->info.typ))
                fprintf(fout, "if (soap_out_%s(soap, %s, -1, &a->%s::%s, \"%s\"))\n\t\treturn soap->error;", c_ident(p->info.typ), field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name), xsi_type_u(p->info.typ));
              else
                fprintf(fout, "{ }");
            }
          }
        }
        i = 0;
        /* Get the depth of the inheritance hierarchy */
        for (t = table; t; t = t->prev)
          i++;
        /* output __item(s) and inherited DOM at the end */
        for (; i > 0; i--)
        {
          Entry *e;
          const char *nse1;
          t = table;
          for (j = 0; j < i-1; j++)
            t = t->prev;
          e = entry(classtable, t->sym);
          if (!t->prev && e && e->info.typ && e->info.typ->baseid && !strcmp(e->info.typ->baseid->name, "soap_dom_element"))
            fprintf(fout, "\n\tif (soap_out_xsd__anyType(soap, NULL, -1, static_cast<const soap_dom_element*>(a), NULL))\n\t\treturn soap->error;");
          nse1 = e ? ns_qualifiedElement(e->info.typ) : nse;
          for (p = t->list; p != (Entry*) 0; p = p->next)
          {
            if (p->info.typ->type == Tfun || p->info.typ->type == Tunion)
              ;
            else if (p->info.sto & (Sconst | Sprivate | Sprotected | Sattribute))
              ;
            else if (is_repetition(p) || is_anytype(p) || is_choice(p))
              p = p->next;
            else if (!is_item(p))
              ;
            else if (is_transient(p->info.typ))
              ;
            else if (p->info.typ->type == Tarray)
              fprintf(fout, "\n\tif (soap_out_%s(soap, %s, -1, a->%s::%s, \"%s\"))\n\t\treturn soap->error;", c_ident(p->info.typ), field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name), xsi_type_u(p->info.typ));
            else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
              fprintf(fout, "\n\tif ((a->%s::%s).soap_out(soap, %s, -1, \"%s\"))\n\t\treturn soap->error;", ident(t->sym->name), ident(p->sym->name), field_overridden(t, p, nse1), xsi_type_u(p->info.typ));
            else if (is_stdqname(p->info.typ))
              fprintf(fout, "\n\tif (soap_out_%s(soap, %s, -1, &soap_tmp_%s, \"%s\"))\n\t\treturn soap->error;", c_ident(p->info.typ), field_overridden(t, p, nse1), ident(p->sym->name), xsi_type_u(p->info.typ));
            else
            {
              if (zflag != 2 && p->info.typ->type == Tpointer && !is_void(p->info.typ) && p->info.minOccurs > 0)
              {
                /* xsi:nil only if nillable, otherwise empty element */
                if (p->info.nillable)
                  fprintf(fout, "\n\tif(a->%s::%s)\n\t{\tif (soap_element_nil(soap, %s))\n\t\t\treturn soap->error;\n\t}\n\telse", ident(t->sym->name), ident(p->sym->name), field_overridden(t, p, nse1));
                else
                  fprintf(fout, "\n\tif(a->%s::%s)\n\t{\tif (soap_element_empty(soap, %s, 0, NULL))\n\t\t\treturn soap->error;\n\t}\n\telse", ident(t->sym->name), ident(p->sym->name), field_overridden(t, p, nse1));
              }
              else
              {
                fprintf(fout, "\n\t");
              }
              if (is_qname(p->info.typ))
                fprintf(fout, "if (soap_out_%s(soap, %s, -1, (char*const*)(void*)&soap_tmp_%s, \"%s\"))\n\t\treturn soap->error;", c_ident(p->info.typ), field_overridden(t, p, nse1), ident(p->sym->name), xsi_type_u(p->info.typ));
              else if ((p->info.typ->type == Tpointer || is_smart(p->info.typ)) && is_qname((Tnode*)p->info.typ->ref))
                fprintf(fout, "if (soap_out_%s(soap, %s, -1, (char*const*)(void*)&soap_tmp_%s, \"%s\"))\n\t\treturn soap->error;", c_ident((Tnode*)p->info.typ->ref), field_overridden(t, p, nse1), ident(p->sym->name), xsi_type_u(p->info.typ->ref));
              else if ((p->info.typ->type == Tpointer || is_smart(p->info.typ)) && is_stdqname((Tnode*)p->info.typ->ref))
                fprintf(fout, "if (soap_tmp_%s && soap_out_%s(soap, %s, -1, soap_tmp_%s, \"%s\"))\n\t\treturn soap->error;", ident(p->sym->name), c_ident(p->info.typ->ref), field_overridden(t, p, nse1), ident(p->sym->name), xsi_type_u(p->info.typ->ref));
              else if (is_XML(p->info.typ) && is_string(p->info.typ))
                fprintf(fout, "if (soap_outliteral(soap, %s, (char*const*)&a->%s::%s, NULL))\n\t\treturn soap->error;", field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name));
              else if (is_XML(p->info.typ) && is_wstring(p->info.typ))
                fprintf(fout, "if (soap_outwliteral(soap, %s, (wchar_t*const*)&a->%s::%s, NULL))\n\t\treturn soap->error;", field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name));
              else if (is_string(p->info.typ))
                fprintf(fout, "if (soap_out_%s(soap, %s, -1, (char*const*)&a->%s::%s, \"%s\"))\n\t\treturn soap->error;", c_ident(p->info.typ), field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name), xsi_type_u(p->info.typ));
              else if (is_wstring(p->info.typ))
                fprintf(fout, "if (soap_out_%s(soap, %s, -1, (wchar_t*const*)&a->%s::%s, \"%s\"))\n\t\treturn soap->error;", c_ident(p->info.typ), field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name), xsi_type_u(p->info.typ));
              else if (p->info.typ->type != Tfun && !is_void(p->info.typ))
                fprintf(fout, "if (soap_out_%s(soap, %s, -1, &a->%s::%s, \"%s\"))\n\t\treturn soap->error;", c_ident(p->info.typ), field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name), xsi_type_u(p->info.typ));
              else
                fprintf(fout, "{ }");
            }
            fflush(fout);
          }
        }
        if (!is_invisible(typ->id->name))
          fprintf(fout, "\n\treturn soap_element_end_out(soap, tag);\n}");
        else
          fprintf(fout, "\n\treturn SOAP_OK;\n}");
      }
      fflush(fout);
      break;
    case Tunion:
      if (is_external(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 int SOAP_FMAC2 soap_out_%s(struct soap*, int, const %s);", c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap*, int, const %s);", c_ident(typ), c_type_id(typ, "*"));
      fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap *soap, int choice, const %s)\n{", c_ident(typ), c_type_id(typ, "*a"));
      table = (Table*)typ->ref;
      fprintf(fout, "\n\t(void)soap; (void)a; /* appease -Wall -Werror */");
      fprintf(fout, "\n\tswitch (choice)\n\t{");
      if (table)
      {
        for (p = table->list; p; p = p->next)
        {
          if (p->info.typ->type == Tfun || p->info.typ->type == Tunion)
            ;
          else if (p->info.sto & (Sconst | Sprivate | Sprotected))
            fprintf(fout, "\n\t/* non-serializable %s skipped */", ident(p->sym->name));
          else if (p->info.sto & Sattribute)
            ;
          else if (is_repetition(p))
            ;
          else if (is_anytype(p))
            ;
          else if (is_transient(p->info.typ))
            fprintf(fout, "\n\t/* transient %s skipped */", ident(p->sym->name));
          else if (p->info.typ->type == Tarray)
          {
            fprintf(fout, "\n\tcase %s:", soap_union_member(typ, p));
            fprintf(fout, "\n\t\treturn soap_out_%s(soap, \"%s\", -1, a->%s, \"%s\");", c_ident(p->info.typ), ns_add(p, nse), ident(p->sym->name), xsi_type_u(p->info.typ));
          }
          else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
          {
            fprintf(fout, "\n\tcase %s:", soap_union_member(typ, p));
            fprintf(fout, "\n\t\treturn a->%s.soap_out(soap, \"%s\", -1, \"%s\");", ident(p->sym->name), ns_add(p, nse), xsi_type_u(p->info.typ));
          }
          else if (is_qname(p->info.typ) || is_stdqname(p->info.typ))
          {
            fprintf(fout, "\n\tcase %s:", soap_union_member(typ, p));
            fprintf(fout, "\n\t{\tconst char *soap_tmp_%s = soap_QName2s(soap, a->%s);", ident(p->sym->name), ident(p->sym->name));
            fprintf(fout, "\n\t\treturn soap_out_%s(soap, \"%s\", -1, (char*const*)(void*)&soap_tmp_%s, \"%s\");\n\t}", c_ident(p->info.typ), ns_add(p, nse), ident(p->sym->name), xsi_type_u(p->info.typ));
          }
          else if (is_XML(p->info.typ) && is_string(p->info.typ))
          {
            fprintf(fout, "\n\tcase %s:", soap_union_member(typ, p));
            fprintf(fout, "\n\t\treturn soap_outliteral(soap, \"%s\", (char*const*)&a->%s, NULL);", ns_add(p, nse), ident(p->sym->name));
          }
          else if (is_XML(p->info.typ) && is_wstring(p->info.typ))
          {
            fprintf(fout, "\n\tcase %s:", soap_union_member(typ, p));
            fprintf(fout, "\n\t\treturn soap_outwliteral(soap, \"%s\", (wchar_t*const*)&a->%s, NULL);", ns_add(p, nse), ident(p->sym->name));
          }
          else if (is_string(p->info.typ))
          {
            fprintf(fout, "\n\tcase %s:", soap_union_member(typ, p));
            fprintf(fout, "\n\t\treturn soap_out_%s(soap, \"%s\", -1, (char*const*)&a->%s, \"%s\");", c_ident(p->info.typ), ns_add(p, nse), ident(p->sym->name), xsi_type_u(p->info.typ));
          }
          else if (is_wstring(p->info.typ))
          {
            fprintf(fout, "\n\tcase %s:", soap_union_member(typ, p));
            fprintf(fout, "\n\t\treturn soap_out_%s(soap, \"%s\", -1, (wchar_t*const*)&a->%s, \"%s\");", c_ident(p->info.typ), ns_add(p, nse), ident(p->sym->name), xsi_type_u(p->info.typ));
          }
          else if (p->info.typ->type != Tfun && !is_void(p->info.typ))
          {
            fprintf(fout, "\n\tcase %s:", soap_union_member(typ, p));
            fprintf(fout, "\n\t\treturn soap_out_%s(soap, \"%s\", -1, &a->%s, \"%s\");", c_ident(p->info.typ), ns_add(p, nse), ident(p->sym->name), xsi_type_u(p->info.typ));
          }
        }
      }
      fprintf(fout, "\n\tdefault:\n\t\tbreak;\n\t}\n\treturn SOAP_OK;\n}");
      fflush(fout);
      break;
    case Tpointer:
      if (is_external(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 int SOAP_FMAC2 soap_out_%s(struct soap*, const char *, int, %s, const char *);", c_ident(typ), c_type_constptr_id(typ, "const*"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap*, const char *, int, %s, const char *);", c_ident(typ), c_type_constptr_id(typ, "const*"));
      fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap *soap, const char *tag, int id, %s, const char *type)\n{", c_ident(typ), c_type_constptr_id(typ, "const*a"));
      if (is_template(typ))
      {
        fprintf(fout, "\n\tif (!*a)");
        fprintf(fout, "\n\t\treturn soap_element_null(soap, tag, id, type);");
        fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, *a, type);", c_ident((Tnode*)typ->ref));
      }
      else
      {
        p = is_dynamic_array((Tnode*)typ->ref);
        if (p)
        {
          d = get_Darraydims((Tnode*)typ->ref);
          if (d)
            fprintf(fout, "\n\tid = soap_element_id(soap, tag, id, *a, *a ? (*a)->%s : NULL, *a ? %s : 0, type, %s, NULL);", ident(p->sym->name), get_Darraysize("(*a)", d), soap_type((Tnode*)typ->ref));
          else
            fprintf(fout, "\n\tid = soap_element_id(soap, tag, id, *a, *a ? (*a)->%s : NULL, *a ? (*a)->__size : 0, type, %s, NULL);", ident(p->sym->name), soap_type((Tnode*)typ->ref));
          fprintf(fout, "\n\tif (!*a || id < 0)\n\t\treturn soap->error;");
        }
        else
        {
          if (((Tnode*)typ->ref)->recursive)
            fprintf(fout, "\n\tchar *mark;\n\tid = soap_element_id(soap, tag, id, *a, NULL, 0, type, %s, &mark);", soap_type((Tnode*)typ->ref));
          else
            fprintf(fout, "\n\tid = soap_element_id(soap, tag, id, *a, NULL, 0, type, %s, NULL);", soap_type((Tnode*)typ->ref));
          fprintf(fout, "\n\tif (id < 0)\n\t\treturn soap->error;");
        }
        if (((Tnode *) typ->ref)->type == Tclass && !is_external((Tnode*)typ->ref) && !is_volatile((Tnode*)typ->ref) && !is_typedef((Tnode*)typ->ref))
        {
          if (!p && ((Tnode*)typ->ref)->recursive)
            fprintf(fout, "\n\t(void)(*a)->soap_out(soap, tag, id, (*a)->soap_type() == %s ? type : NULL);\n\tsoap_unmark(soap, mark);\n\treturn soap->error;", soap_type((Tnode*)typ->ref));
          else
            fprintf(fout, "\n\treturn (*a)->soap_out(soap, tag, id, (*a)->soap_type() == %s ? type : NULL);", soap_type((Tnode*)typ->ref));
        }
        else
        {
          if (!p && ((Tnode*)typ->ref)->recursive)
            fprintf(fout, "\n\t(void)soap_out_%s(soap, tag, id, *a, type);\n\tsoap_unmark(soap, mark);\n\treturn soap->error;", c_ident((Tnode*)typ->ref));
          else
            fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, *a, type);", c_ident((Tnode*)typ->ref));
        }
      }
      fprintf(fout, "\n}");
      break;
    case Tarray:
      if (is_external(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 int SOAP_FMAC2 soap_out_%s(struct soap*, const char*, int, %s, const char*);", c_ident(typ), c_type_id(typ, "const"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap*, const char*, int, %s, const char*);", c_ident(typ), c_type_id(typ, "const"));
      fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap *soap, const char *tag, int id, %s, const char *type)\n{", c_ident(typ), c_type_id(typ, "const a"));
      fprintf(fout, "\n\tsize_t i;\n\t(void)type;");
      fprintf(fout, "\n\tsoap_array_begin_out(soap, tag, soap_embedded_id(soap, id, a, %s), \"%s[%d]\", 0);", soap_type(typ), xsi_type_Tarray(typ), get_dimension(typ));
      (void)get_item_type(typ, &cardinality);
      fprintf(fout, "\n\tfor (i = 0; i < %d; i++)\n\t{", get_dimension(typ));
      if (((Tnode *)typ->ref)->type == Tclass && !is_external((Tnode*)typ->ref) && !is_volatile((Tnode*)typ->ref) && !is_typedef((Tnode*)typ->ref))
      {
        if (cardinality>1)
          fprintf(fout, "\n\t\tif (a[i].soap_out(soap, \"item\", -1, \"%s\")", xsi_type_u((Tnode*)typ->ref));
        else fprintf(fout, "\n\t\tif ((a+i)->soap_out(soap, \"item\", -1, \"%s\")", xsi_type_u((Tnode*)typ->ref));
      }
      else
      {
        if (((Tnode *)typ->ref)->type != Tarray)
        {
          if (((Tnode *)typ->ref)->type == Tpointer)
            fprintf(fout, "\n\t\tsoap->position = 1;\n\t\tsoap->positions[0] = i;\n\t\tif (soap_out_%s(soap, \"item\", -1, a", c_ident((Tnode*)typ->ref));
          else
            fprintf(fout, "\n\t\tif (soap_out_%s(soap, \"item\", -1, a", c_ident((Tnode*)typ->ref));
        }
        else
          fprintf(fout, "\n\t\tif (soap_out_%s(soap, \"item\", -1, a", c_ident((Tnode*)typ->ref));
        if (cardinality>1)
          fprintf(fout, "[i], \"%s\")", xsi_type_u((Tnode*)typ->ref));
        else
          fprintf(fout, "+i, \"%s\")", xsi_type_u((Tnode*)typ->ref));
      }
      fprintf(fout, ")\n\t\t\treturn soap->error;");
      if (((Tnode *)typ->ref)->type == Tpointer)
        fprintf(fout, "\n\t}\n\tsoap->position = 0;\n\treturn soap_element_end_out(soap, tag);\n}");
      else
        fprintf(fout, "\n\t}\n\treturn soap_element_end_out(soap, tag);\n}");
      break;
    case Tenum:
    case Tenumsc:
      if (is_external(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 int SOAP_FMAC2 soap_out_%s(struct soap*, const char*, int, const %s, const char*);", c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap*, const char*, int, const %s, const char*);", c_ident(typ), c_type_id(typ, "*"));
      if (!is_typedef(typ))
      {
        fprintf(fout, "\n\nstatic const struct soap_code_map soap_codes_%s[] =\n{", c_ident(typ));
        for (t = (Table*)typ->ref; t; t = t->prev)
        {
          for (p = t->list; p; p = p->next)
          {
            if (typ->type == Tenumsc)
              fprintf(fout, "\t{ (LONG64)%s::%s, \"%s\" },\n", c_ident(typ), ident(p->sym->name), ns_remove2(p->sym->name, c_ident(typ)));
            else
              fprintf(fout, "\t{ (LONG64)%s, \"%s\" },\n", ident(p->sym->name), ns_remove2(p->sym->name, c_ident(typ)));
          }
        }
        fprintf(fout, "\t{ 0, NULL }\n");
        fprintf(fout, "};");
      }
      if (!is_typedef(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC3S const char* SOAP_FMAC4S soap_%s2s(struct soap*, %s);", c_ident(typ), c_type(typ));
        fprintf(fout, "\n\nSOAP_FMAC3S const char* SOAP_FMAC4S soap_%s2s(struct soap *soap, %s)", c_ident(typ), c_type_id(typ, "n"));
        if (is_boolean(typ))
        {
          fprintf(fout, "\n{\n\t(void)soap; /* appease -Wall -Werror */\n\treturn soap_code_str(soap_codes_%s, n != 0);\n}", c_ident(typ));
        }
        else if (!is_mask(typ))
        {
          fprintf(fout, "\n{\n\tconst char *s = soap_code_str(soap_codes_%s, (long)n);", c_ident(typ));
          fprintf(fout, "\n\tif (s)\n\t\treturn s;");
          fprintf(fout, "\n\treturn soap_long2s(soap, (long)n);");
          fprintf(fout, "\n}");
        }
        else
        {
          fprintf(fout, "\n{\n\treturn soap_code_list(soap, soap_codes_%s, (long)n);\n}", c_ident(typ));
        }
      }
      fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap *soap, const char *tag, int id, const %s, const char *type)\n{", c_ident(typ), c_type_id(typ, "*a"));
      x = xsi_type_u(typ);
      if (x && *x)
        fprintf(fout, "\n\tif (!type)\n\t\ttype = \"%s\";", x);
      fprintf(fout, "\n\tif (soap_element_begin_out(soap, tag, soap_embedded_id(soap, id, a, %s), type)", soap_type(typ));
      fprintf(fout, " || soap_send(soap, soap_%s2s(soap, *a)))\n\t\treturn soap->error;", c_ident(typ));
      fprintf(fout, "\n\treturn soap_element_end_out(soap, tag);\n}");
      break;
    case Ttemplate:
      if (is_external(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 int SOAP_FMAC2 soap_out_%s(struct soap*, const char*, int, const %s, const char*);", c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      if (is_typedef(typ))
      {
        fprintf(fhead, "\n\n#define soap_out_%s soap_out_%s\n", c_ident(typ), t_ident(typ));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap*, const char*, int, const %s, const char*);", c_ident(typ), c_type_id(typ, "*"));
      temp = (Tnode*)typ->ref;
      if (!temp)
        return;
      fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap *soap, const char *tag, int id, const %s, const char *type)\n{", c_ident(typ), c_type_id(typ, "*a"));
      fprintf(fout, "\n\t(void)id; (void)type; /* appease -Wall -Werror */");
      if (is_smart(typ))
      {
        p = is_dynamic_array((Tnode*)typ->ref);
        if (p)
        {
          d = get_Darraydims((Tnode*)typ->ref);
          if (d)
            fprintf(fout, "\n\tid = soap_element_id(soap, tag, id, a->get(), a->get() ? (*a)->%s : NULL, a->get() ? %s : 0, type, %s, NULL);", ident(p->sym->name), get_Darraysize("(*a)", d), soap_type((Tnode*)typ->ref));
          else
            fprintf(fout, "\n\tid = soap_element_id(soap, tag, id, a->get(), a->get() ? (*a)->%s : NULL, a->get() ? (*a)->__size : 0, type, %s, NULL);", ident(p->sym->name), soap_type((Tnode*)typ->ref));
        }
        else if (((Tnode*)typ->ref)->recursive)
          fprintf(fout, "\n\tchar *mark;\n\tid = soap_element_id(soap, tag, id, a->get(), NULL, 0, type, %s, &mark);", soap_type((Tnode*)typ->ref));
        else
          fprintf(fout, "\n\tid = soap_element_id(soap, tag, id, a->get(), NULL, 0, type, %s, NULL);", soap_type((Tnode*)typ->ref));
        fprintf(fout, "\n\tif (id < 0)\n\t\treturn soap->error;");
        if (((Tnode *) typ->ref)->type == Tclass && !is_external((Tnode*)typ->ref) && !is_volatile((Tnode*)typ->ref) && !is_typedef((Tnode*)typ->ref))
        {
          if (!p && ((Tnode*)typ->ref)->recursive)
            fprintf(fout, "\n\t(void)(*a)->soap_out(soap, tag, id, (*a)->soap_type() == %s ? type : NULL);\n\tsoap_unmark(soap, mark);\n\treturn soap->error;", soap_type((Tnode*)typ->ref));
          else
            fprintf(fout, "\n\treturn (*a)->soap_out(soap, tag, id, (*a)->soap_type() == %s ? type : NULL);", soap_type((Tnode*)typ->ref));
        }
        else
        {
          if (!p && ((Tnode*)typ->ref)->recursive)
            fprintf(fout, "\n\t(void)soap_out_%s(soap, tag, id, a->get(), type);\n\tsoap_unmark(soap, mark);\n\treturn soap->error;", c_ident((Tnode*)typ->ref));
          else
            fprintf(fout, "\n\treturn soap_out_%s(soap, tag, id, a->get(), type);", c_ident((Tnode*)typ->ref));
        }
        fprintf(fout, "\n}");
      }
      else
      {
        fprintf(fout, "\n\tfor (%s::const_iterator i = a->begin(); i != a->end(); ++i)\n\t{", c_type(typ));
        if (temp->type == Tarray)
          fprintf(fout, "\n\t\tif (soap_out_%s(soap, tag, id, *i, \"%s\"))", c_ident(temp), xsi_type_u(typ));
        else if (temp->type == Tclass && !is_external(temp) && !is_volatile(temp) && !is_typedef(temp))
          fprintf(fout, "\n\t\tif ((*i).soap_out(soap, tag, id, \"%s\"))", xsi_type_u(typ));
        else if (is_qname(temp))
          fprintf(fout, "\n\t\tconst char *soap_tmp = soap_QName2s(soap, *i);\n\t\tif (soap_out_%s(soap, tag, id, (char*const*)(void*)&soap_tmp, \"%s\"))", c_ident(temp), xsi_type_u(typ));
        else if (is_stdqname(temp))
          fprintf(fout, "\n\t\tstd::string soap_tmp(soap_QName2s(soap, (*i).c_str()));\n\t\tif (soap_out_%s(soap, tag, id, &soap_tmp, \"%s\"))", c_ident(temp), xsi_type_u(typ));
        else if (is_XML(temp) && is_string(temp))
          fprintf(fout, "\n\t\tif (soap_outliteral(soap, tag, (char*const*)&(*i), NULL))");
        else if (is_XML(temp) && is_wstring(temp))
          fprintf(fout, "\n\t\tif (soap_outwliteral(soap, tag, (wchar_t*const*)&(*i), NULL))");
        else if (is_string(temp))
          fprintf(fout, "\n\t\tif (soap_out_%s(soap, tag, -1, (char*const*)&(*i), \"%s\"))", c_ident(temp), xsi_type_u(typ));
        else if (is_wstring(temp))
          fprintf(fout, "\n\t\tif (soap_out_%s(soap, tag, -1, (wchar_t*const*)&(*i), \"%s\"))", c_ident(temp), xsi_type_u(typ));
        else if (is_bool(temp))
          fprintf(fout, "\n\t\tbool b = (*i);\n\t\tif (soap_out_%s(soap, tag, id, &b, \"%s\"))", c_ident(temp), xsi_type_u(typ));
        else
          fprintf(fout, "\n\t\tif (soap_out_%s(soap, tag, id, &(*i), \"%s\"))", c_ident(temp), xsi_type_u(typ));
        fprintf(fout, "\n\t\t\treturn soap->error;");
        fprintf(fout, "\n\t}\n\treturn SOAP_OK;\n}");
      }
      break;
    default:
      break;
  }
}

void
soap_out_Darray(Tnode *typ)
{
  int i, j, d = 0;
  Table *t, *table;
  Entry *p;
  const char *nse = ns_qualifiedElement(typ);
  const char *nsa = ns_qualifiedAttribute(typ);
  const char *item;

  table = (Table*)typ->ref;
  fprintf(fhead, "\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap*, const char*, int, const %s, const char*);", c_ident(typ), c_type_id(typ, "*"));

  if (is_external(typ))
    return;

  if (is_binary(typ))
  {
    fprintf(fhead, "\nSOAP_FMAC3S const char* SOAP_FMAC4S soap_%s2s(struct soap*, %s);", c_ident(typ), c_type(typ));
    fprintf(fout, "\n\nSOAP_FMAC3S const char* SOAP_FMAC4S soap_%s2s(struct soap *soap, %s)\n{", c_ident(typ), c_type_id(typ, "a"));
    if (is_hexBinary(typ))
      fprintf(fout, "\n\treturn soap_s2hex(soap, a.__ptr, NULL, a.__size);");
    else
      fprintf(fout, "\n\treturn soap_s2base64(soap, a.__ptr, NULL, a.__size);");
    fprintf(fout, "\n}");
  }

  if (typ->type == Tclass && !is_volatile(typ) && !is_typedef(typ))
  {
    fprintf(fout, "\n\nint %s::soap_out(struct soap *soap, const char *tag, int id, const char *type) const", c_type(typ));
    fprintf(fout, "\n{\n\treturn soap_out_%s(soap, tag, id, this, type);\n}", c_ident(typ));
  }
  fflush(fout);
  fprintf(fout, "\n\nSOAP_FMAC3 int SOAP_FMAC4 soap_out_%s(struct soap *soap, const char *tag, int id, const %s, const char *type)\n{", c_ident(typ), c_type_id(typ, "*a"));
  if (has_setter(typ))
    fprintf(fout, "\n\t((%s)a)->set(soap);", c_type_id(typ, "*"));
  if (!is_binary(typ))
  {
    p = is_dynamic_array(typ);
    d = get_Darraydims(typ);
    if (d)
      fprintf(fout, "\n\tsize_t i, n = soap_size(a->__size, %d);", d);
    else
      fprintf(fout, "\n\tint i, n = a->__size;");
  }
  if (typ->type == Tclass)
  {
    for (t = table; t; t = t->prev)
    {
      for (p = t->list; p; p = p->next)
      {
        if (p->info.sto & Sattribute)
          soap_set_attr(p, ptr_cast(t, "a"), ident(p->sym->name), ns_add(p, nsa));
      }
    }
  }
  else
  {
    for (t = table; t; t = t->prev)
    {
      for (p = t->list; p; p = p->next)
      {
        if (p->info.sto & Sattribute)
          soap_set_attr(p, "a", ident(p->sym->name), ns_add(p, nsa));
      }
    }
  }
  p = is_dynamic_array(typ);
  if (p->sym->name[5])
    item = ns_addx(ns_convert(p->sym->name + 5), nse);
  else
    item = ns_addx("item", nse);
  if (!has_ns(typ) && !is_untyped(typ) && !is_binary(typ))
  {
    const char *s = xsi_type(typ);
    if (is_untyped(p->info.typ))
      s = wsdl_type(p->info.typ, "xsd");
    if (d)
    {
      if (has_offset(typ))
        fprintf(fout, "\n\tchar *t = a->%s ? soap_putsizesoffsets(soap, \"%s\", a->__size, a->__offset, %d) : NULL;", ident(p->sym->name), s, d);
      else
        fprintf(fout, "\n\tchar *t = a->%s ? soap_putsizesoffsets(soap, \"%s\", a->__size, NULL, %d) : NULL;", ident(p->sym->name), s, d);
    }
    else
    {
      if (has_offset(typ))
        fprintf(fout, "\n\tchar *t = a->%s ? soap_putsizesoffsets(soap, \"%s\", &a->__size, &a->__offset, 1) : NULL;", ident(p->sym->name), s);
      else
        fprintf(fout, "\n\tchar *t = a->%s ? soap_putsizesoffsets(soap, \"%s\", &a->__size, NULL, 1) : NULL;", ident(p->sym->name), s);
    }
  }
  if (d)
    fprintf(fout, "\n\tid = soap_element_id(soap, tag, id, a, a->%s, %s, type, %s, NULL);", ident(p->sym->name), get_Darraysize("a", d), soap_type(typ));
  else if (is_attachment(typ))
  {
    fprintf(fout, "\n#ifndef WITH_LEANER\n\tid = soap_attachment(soap, tag, id, a, a->%s, a->__size, a->id, a->type, a->options, type, %s);", ident(p->sym->name), soap_type(typ));
    fprintf(fout, "\n#else\n\tid = soap_element_id(soap, tag, id, a, a->%s, a->__size, type, %s, NULL);\n#endif", ident(p->sym->name), soap_type(typ));
  }
  else
    fprintf(fout, "\n\tid = soap_element_id(soap, tag, id, a, a->%s, a->__size, type, %s, NULL);", ident(p->sym->name), soap_type(typ));
  fprintf(fout, "\n\tif (id < 0)\n\t\treturn soap->error;");
  fprintf(fout, "\n\tif (");
  if (has_ns(typ) || is_untyped(typ) || is_binary(typ))
  {
    if (table && table->prev)
      fprintf(fout, "soap_element_begin_out(soap, tag, id, type ? type : \"%s\")", xsi_type(typ));
    else
      fprintf(fout, "soap_element_begin_out(soap, tag, id, type)");
  }
  else if (has_offset(typ))
  {
    if (d)
      fprintf(fout, "soap_array_begin_out(soap, tag, id, t, soap_putoffsets(soap, a->__offset, %d))", d);
    else
      fprintf(fout, "soap_array_begin_out(soap, tag, id, t, soap_putoffsets(soap, &a->__offset, 1))");
  }
  else
    fprintf(fout, "soap_array_begin_out(soap, tag, id, t, NULL)");
  fprintf(fout, ")\n\t\treturn soap->error;");
  if (is_hexBinary(typ))
    fprintf(fout, "\n\tif (soap_puthex(soap, a->__ptr, a->__size))\n\t\treturn soap->error;");
  else if (is_binary(typ))
    fprintf(fout, "\n\tif (soap_putbase64(soap, a->__ptr, a->__size))\n\t\treturn soap->error;");
  else
  {
    fprintf(fout, "\n\tfor (i = 0; i < n; i++)\n\t{");
    if (!has_ns(typ) && !is_untyped(typ))
    {
      if (d)
      {
        fprintf(fout, "\n\t\tsoap->position = %d;", d);
        for (i = 0; i < d; i++)
        {
          fprintf(fout, "\n\t\tsoap->positions[%d] = i", i);
          for (j = i+1; j < d; j++)
            fprintf(fout, "/a->__size[%d]", j);
          fprintf(fout, "%%a->__size[%d];", i);
        }
        fprintf(fout, "\n\t\tif (");
        if (is_XML((Tnode*)p->info.typ->ref) && is_string((Tnode*)p->info.typ->ref))
          fprintf(fout, "soap_outliteral(soap, \"%s\", (char*const*)&a->%s[i], NULL)", item, ident(p->sym->name));
        else if (is_XML((Tnode*)p->info.typ->ref) && is_wstring((Tnode*)p->info.typ->ref))
          fprintf(fout, "soap_outwliteral(soap, \"%s\", (wchar_t*const*)&a->%s[i], NULL)", item, ident(p->sym->name));
        else if (is_string((Tnode*)p->info.typ->ref))
          fprintf(fout, "soap_out_%s(soap, \"%s\", -1, (char*const*)&a->%s[i], \"%s\")", c_ident(((Tnode *)p->info.typ->ref)), item, ident(p->sym->name), xsi_type_u(((Tnode *)p->info.typ->ref)));
        else if (is_wstring((Tnode*)p->info.typ->ref))
          fprintf(fout, "soap_out_%s(soap, \"%s\", -1, (wchar_t*const*)&a->%s[i], \"%s\")", c_ident(((Tnode *)p->info.typ->ref)), item, ident(p->sym->name), xsi_type_u(((Tnode *)p->info.typ->ref)));
        else if (((Tnode *)p->info.typ->ref)->type == Tclass && !is_external((Tnode*)p->info.typ->ref) && !is_volatile((Tnode*)p->info.typ->ref) && !is_typedef((Tnode*)p->info.typ->ref))
          fprintf(fout, "a->%s[i].soap_out(soap, \"%s\", -1, \"%s\")", ident(p->sym->name), item, xsi_type_u(((Tnode *)p->info.typ->ref)));
        else
          fprintf(fout, "soap_out_%s(soap, \"%s\", -1, &a->%s[i], \"%s\")", c_ident(((Tnode *)p->info.typ->ref)), item, ident(p->sym->name), xsi_type_u(((Tnode *)p->info.typ->ref)));
        fprintf(fout, ")\n\t\t\treturn soap->error;");
      }
      else
      {
        fprintf(fout, "\n\t\tsoap->position = 1;\n\t\tsoap->positions[0] = i;");
        fprintf(fout, "\n\t\tif (");
        if (is_XML((Tnode*)p->info.typ->ref) && is_string((Tnode*)p->info.typ->ref))
          fprintf(fout, "soap_outliteral(soap, \"%s\", &a->%s[i], NULL)", item, ident(p->sym->name));
        else if (is_XML((Tnode*)p->info.typ->ref) && is_wstring((Tnode*)p->info.typ->ref))
          fprintf(fout, "soap_outwliteral(soap, \"%s\", &a->%s[i], NULL)", item, ident(p->sym->name));
        else if (is_string((Tnode*)p->info.typ->ref))
          fprintf(fout, "soap_out_%s(soap, \"%s\", -1, (char*const*)&a->%s[i], \"%s\")", c_ident(((Tnode *)p->info.typ->ref)), item, ident(p->sym->name), xsi_type_u(((Tnode *)p->info.typ->ref)));
        else if (is_wstring((Tnode*)p->info.typ->ref))
          fprintf(fout, "soap_out_%s(soap, \"%s\", -1, (wchar_t*const*)&a->%s[i], \"%s\")", c_ident(((Tnode *)p->info.typ->ref)), item, ident(p->sym->name), xsi_type_u(((Tnode *)p->info.typ->ref)));
        else if (((Tnode *)p->info.typ->ref)->type == Tclass && !is_external((Tnode*)p->info.typ->ref) && !is_volatile((Tnode*)p->info.typ->ref) && !is_typedef((Tnode*)p->info.typ->ref))
          fprintf(fout, "a->%s[i].soap_out(soap, \"%s\", -1, \"%s\")", ident(p->sym->name), item, xsi_type_u(((Tnode *)p->info.typ->ref)));
        else
          fprintf(fout, "soap_out_%s(soap, \"%s\", -1, &a->%s[i], \"%s\")", c_ident(((Tnode *)p->info.typ->ref)), item, ident(p->sym->name), xsi_type_u(((Tnode *)p->info.typ->ref)));
        fprintf(fout, ")\n\t\t\treturn soap->error;");
      }
    }
    else
    {
      fprintf(fout, "\n\t\tif (");
      if (is_XML((Tnode*)p->info.typ->ref) && is_string((Tnode*)p->info.typ->ref))
        fprintf(fout, "soap_outliteral(soap, \"%s\", &a->%s[i], NULL)", item, ident(p->sym->name));
      else if (is_XML((Tnode*)p->info.typ->ref) && is_wstring((Tnode*)p->info.typ->ref))
        fprintf(fout, "soap_outwliteral(soap, \"%s\", &a->%s[i], NULL)", item, ident(p->sym->name));
      else if (is_string((Tnode*)p->info.typ->ref))
        fprintf(fout, "soap_out_%s(soap, \"%s\", -1, (char*const*)&a->%s[i], \"%s\")", c_ident(((Tnode *)p->info.typ->ref)), item, ident(p->sym->name), xsi_type_u(((Tnode *)p->info.typ->ref)));
      else if (is_wstring((Tnode*)p->info.typ->ref))
        fprintf(fout, "soap_out_%s(soap, \"%s\", -1, (wchar_t*const*)&a->%s[i], \"%s\")", c_ident(((Tnode *)p->info.typ->ref)), item, ident(p->sym->name), xsi_type_u(((Tnode *)p->info.typ->ref)));
      else if (((Tnode *)p->info.typ->ref)->type == Tclass && !is_external((Tnode*)p->info.typ->ref) && !is_volatile((Tnode*)p->info.typ->ref) && !is_typedef((Tnode*)p->info.typ->ref))
        fprintf(fout, "a->%s[i].soap_out(soap, \"%s\", -1, \"%s\")", ident(p->sym->name), item, xsi_type_u(((Tnode *)p->info.typ->ref)));
      else
        fprintf(fout, "soap_out_%s(soap, \"%s\", -1, &a->%s[i], \"%s\")", c_ident(((Tnode *)p->info.typ->ref)), item, ident(p->sym->name), xsi_type_u(((Tnode *)p->info.typ->ref)));
      fprintf(fout, ")\n\t\t\treturn soap->error;");
    }
  }
  if (is_binary(typ))
    fprintf(fout, "\n\treturn soap_element_end_out(soap, tag);\n}");
  else if (!has_ns(typ) && !is_untyped(typ))
    fprintf(fout, "\n\t}\n\tsoap->position = 0;\n\treturn soap_element_end_out(soap, tag);\n}");
  else
    fprintf(fout, "\n\t}\n\treturn soap_element_end_out(soap, tag);\n}");
}

void
soap_get(Tnode *typ)
{
  Tnode *temp;
  int cardinality;

  if (is_XML(typ))
    return;

  if (typ->type == Ttemplate || typ->type == Tunion)
    return;

  if (is_typedef(typ) && (is_template(typ) || is_element(typ) || is_synonym(typ) || is_external(typ) || is_imported(typ)) && (!is_external(typ) || is_volatile(typ)))
  {
    fprintf(fhead, "\n\n#define soap_get_%s soap_get_%s\n", c_ident(typ), t_ident(typ));
    fprintf(fhead, "\n\n#define soap_read_%s soap_read_%s\n", c_ident(typ), t_ident(typ));
    fprintf(fhead, "\n\n#define soap_GET_%s soap_GET_%s\n", c_ident(typ), t_ident(typ));
    fprintf(fhead, "\n\n#define soap_POST_recv_%s soap_POST_recv_%s\n", c_ident(typ), t_ident(typ));
    return;
  }

  if (typ->type == Tarray)
  {
    /* ARRAY */
    temp = get_item_type(typ, &cardinality);
    fprintf(fhead, "\nSOAP_FMAC3 %s * SOAP_FMAC4 soap_get_%s(struct soap*, %s, const char*, const char*);", c_type(temp), c_ident(typ), c_type_id(typ, ""));
    fprintf(fout, "\n\nSOAP_FMAC3 %s * SOAP_FMAC4 soap_get_%s(struct soap *soap, %s, const char *tag, const char *type)", c_type(temp), c_ident(typ), c_type_id(typ, "a"));
    fprintf(fout, "\n{\n\t%s;", c_type_id(temp, "(*p)"));
    fprintf(fout, "\n\tif ((p = soap_in_%s(soap, tag, a, type)))", c_ident(typ));
  }
  else if (typ->type == Tclass && !is_external(typ) && !is_volatile(typ) && !is_typedef(typ))
  {
    /* CLASS  */
    fprintf(fout, "\n\nvoid *%s::soap_get(struct soap *soap, const char *tag, const char *type)", c_type(typ));
    fprintf(fout, "\n{\n\treturn soap_get_%s(soap, this, tag, type);\n}", c_ident(typ));
    fprintf(fhead, "\nSOAP_FMAC3 %s SOAP_FMAC4 soap_get_%s(struct soap*, %s, const char*, const char*);", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*"));
    fprintf(fout, "\n\nSOAP_FMAC3 %s SOAP_FMAC4 soap_get_%s(struct soap *soap, %s, const char *tag, const char *type)\n{", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*p"));
    fprintf(fout, "\n\tif ((p = soap_in_%s(soap, tag, p, type)))", c_ident(typ));
  }
  else
  {
    fprintf(fhead, "\nSOAP_FMAC3 %s SOAP_FMAC4 soap_get_%s(struct soap*, %s, const char*, const char*);", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*"));
    fprintf(fout, "\n\nSOAP_FMAC3 %s SOAP_FMAC4 soap_get_%s(struct soap *soap, %s, const char *tag, const char *type)\n{", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*p"));
    fprintf(fout, "\n\tif ((p = soap_in_%s(soap, tag, p, type)))", c_ident(typ));
  }
  fprintf(fout, "\n\t\tif (soap_getindependent(soap))\n\t\t\treturn NULL;");
  fprintf(fout, "\n\treturn p;\n}");
  if ((typ->type != Tpointer || is_string(typ) || is_wstring(typ)) && typ->type != Tarray && typ->type != Treference && typ->type != Trvalueref && !is_template(typ) && !is_anyAttribute(typ))
  {
    if (typ->type == Tclass && !is_external(typ) && !is_volatile(typ) && !is_typedef(typ))
    {
      if (namespaceid && !is_external(typ))
        fprintf(fhead, "\n\ninline int soap_read_%s(struct soap *soap, %s)\n{\n\tif (p)\n\t{\tp->soap_default(soap);\n\t\tif (soap_begin_recv(soap) || %s::soap_get_%s(soap, p, NULL, NULL) == NULL || soap_end_recv(soap))\n\t\t\treturn soap->error;\n\t}\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_id(typ, "*p"), namespaceid, c_ident(typ));
      else if (!is_external(typ))
        fprintf(fhead, "\n\ninline int soap_read_%s(struct soap *soap, %s)\n{\n\tif (p)\n\t{\tp->soap_default(soap);\n\t\tif (soap_begin_recv(soap) || ::soap_get_%s(soap, p, NULL, NULL) == NULL || soap_end_recv(soap))\n\t\t\treturn soap->error;\n\t}\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_id(typ, "*p"), c_ident(typ));
      else
        fprintf(fhead, "\n\ninline int soap_read_%s(struct soap *soap, %s)\n{\n\tif (p)\n\t{\tp->soap_default(soap);\n\t\tif (soap_begin_recv(soap) || soap_get_%s(soap, p, NULL, NULL) == NULL || soap_end_recv(soap))\n\t\t\treturn soap->error;\n\t}\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_id(typ, "*p"), c_ident(typ));
    }
    else if (is_primitive_or_string(typ))
    {
      if (cflag)
        fprintf(fhead, "\n\n#ifndef soap_read_%s\n#define soap_read_%s(soap, data) ( soap_begin_recv(soap) || !soap_get_%s(soap, (data), NULL, NULL) || soap_end_recv(soap), (soap)->error )\n#endif\n", c_ident(typ), c_ident(typ), c_ident(typ));
      else if (namespaceid && !is_external(typ))
        fprintf(fhead, "\n\ninline int soap_read_%s(struct soap *soap, %s)\n{\n\tif (p)\n\t{\tif (soap_begin_recv(soap) || %s::soap_get_%s(soap, p, NULL, NULL) == NULL || soap_end_recv(soap))\n\t\t\treturn soap->error;\n\t}\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_id(typ, "*p"), namespaceid, c_ident(typ));
      else if (!is_external(typ))
        fprintf(fhead, "\n\ninline int soap_read_%s(struct soap *soap, %s)\n{\n\tif (p)\n\t{\tif (soap_begin_recv(soap) || ::soap_get_%s(soap, p, NULL, NULL) == NULL || soap_end_recv(soap))\n\t\t\treturn soap->error;\n\t}\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_id(typ, "*p"), c_ident(typ));
      else
        fprintf(fhead, "\n\ninline int soap_read_%s(struct soap *soap, %s)\n{\n\tif (p)\n\t{\tif (soap_begin_recv(soap) || soap_get_%s(soap, p, NULL, NULL) == NULL || soap_end_recv(soap))\n\t\t\treturn soap->error;\n\t}\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_id(typ, "*p"), c_ident(typ));
    }
    else
    {
      if (cflag)
        fprintf(fhead, "\n\n#ifndef soap_read_%s\n#define soap_read_%s(soap, data) ( ((data) ? (soap_default_%s(soap, (data)), 0) : 0) || soap_begin_recv(soap) || !soap_get_%s(soap, (data), NULL, NULL) || soap_end_recv(soap), (soap)->error )\n#endif\n", c_ident(typ), c_ident(typ), c_ident(typ), c_ident(typ));
      else if (namespaceid && !is_external(typ))
        fprintf(fhead, "\n\ninline int soap_read_%s(struct soap *soap, %s)\n{\n\tif (p)\n\t{\t%s::soap_default_%s(soap, p);\n\t\tif (soap_begin_recv(soap) || %s::soap_get_%s(soap, p, NULL, NULL) == NULL || soap_end_recv(soap))\n\t\t\treturn soap->error;\n\t}\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_id(typ, "*p"), namespaceid, c_ident(typ), namespaceid, c_ident(typ));
      else if (!is_external(typ))
        fprintf(fhead, "\n\ninline int soap_read_%s(struct soap *soap, %s)\n{\n\tif (p)\n\t{\t::soap_default_%s(soap, p);\n\t\tif (soap_begin_recv(soap) || ::soap_get_%s(soap, p, NULL, NULL) == NULL || soap_end_recv(soap))\n\t\t\treturn soap->error;\n\t}\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_id(typ, "*p"), c_ident(typ), c_ident(typ));
      else
        fprintf(fhead, "\n\ninline int soap_read_%s(struct soap *soap, %s)\n{\n\tif (p)\n\t{\tsoap_default_%s(soap, p);\n\t\tif (soap_begin_recv(soap) || soap_get_%s(soap, p, NULL, NULL) == NULL || soap_end_recv(soap))\n\t\t\treturn soap->error;\n\t}\n\treturn SOAP_OK;\n}", c_ident(typ), c_type_id(typ, "*p"), c_ident(typ), c_ident(typ));
    }
    if (cflag)
      fprintf(fhead, "\n\n#ifndef soap_GET_%s\n#define soap_GET_%s(soap, URL, data) ( soap_GET(soap, URL, NULL) || soap_read_%s(soap, (data)), soap_closesock(soap) )\n#endif\n", c_ident(typ), c_ident(typ), c_ident(typ));
    else if (namespaceid && !is_external(typ))
      fprintf(fhead, "\n\ninline int soap_GET_%s(struct soap *soap, const char *URL, %s)\n{\n\tif (soap_GET(soap, URL, NULL) || %s::soap_read_%s(soap, p))\n\t\treturn soap_closesock(soap);\n\treturn soap_closesock(soap);\n}", c_ident(typ), c_type_id(typ, "*p"), namespaceid, c_ident(typ));
    else if (!is_external(typ))
      fprintf(fhead, "\n\ninline int soap_GET_%s(struct soap *soap, const char *URL, %s)\n{\n\tif (soap_GET(soap, URL, NULL) || ::soap_read_%s(soap, p))\n\t\treturn soap_closesock(soap);\n\treturn soap_closesock(soap);\n}", c_ident(typ), c_type_id(typ, "*p"), c_ident(typ));
    else
      fprintf(fhead, "\n\ninline int soap_GET_%s(struct soap *soap, const char *URL, %s)\n{\n\tif (soap_GET(soap, URL, NULL) || soap_read_%s(soap, p))\n\t\treturn soap_closesock(soap);\n\treturn soap_closesock(soap);\n}", c_ident(typ), c_type_id(typ, "*p"), c_ident(typ));
    if (cflag)
      fprintf(fhead, "\n\n#ifndef soap_POST_recv_%s\n#define soap_POST_recv_%s(soap, data) ( soap_read_%s(soap, (data)) || soap_closesock(soap), (soap)->error )\n#endif\n", c_ident(typ), c_ident(typ), c_ident(typ));
    else if (namespaceid && !is_external(typ))
      fprintf(fhead, "\n\ninline int soap_POST_recv_%s(struct soap *soap, %s)\n{\n\tif (%s::soap_read_%s(soap, p))\n\t\treturn soap_closesock(soap);\n\treturn soap_closesock(soap);\n}", c_ident(typ), c_type_id(typ, "*p"), namespaceid, c_ident(typ));
    else if (!is_external(typ))
      fprintf(fhead, "\n\ninline int soap_POST_recv_%s(struct soap *soap, %s)\n{\n\tif (::soap_read_%s(soap, p))\n\t\treturn soap_closesock(soap);\n\treturn soap_closesock(soap);\n}", c_ident(typ), c_type_id(typ, "*p"), c_ident(typ));
    else
      fprintf(fhead, "\n\ninline int soap_POST_recv_%s(struct soap *soap, %s)\n{\n\tif (soap_read_%s(soap, p))\n\t\treturn soap_closesock(soap);\n\treturn soap_closesock(soap);\n}", c_ident(typ), c_type_id(typ, "*p"), c_ident(typ));
  }
  fflush(fout);
}

void
soap_in(Tnode *typ)
{
  Entry *p = NULL;
  Table *table, *t;
  int strict, nonempty, flag, cardinality, i, j;
  Tnode *n, *temp;
  const char *nse = ns_qualifiedElement(typ);
  const char *nsa = ns_qualifiedAttribute(typ);
  int der = 0;

  if (is_XML(typ))
    return;
  if (is_dynamic_array(typ))
  {
    soap_in_Darray(typ);
    return;
  }
  if (is_external(typ) && !is_volatile(typ))
  {
    fprintf(fhead, "\nSOAP_FMAC3S int SOAP_FMAC4S soap_s2%s(struct soap*, const char*, %s);", c_ident(typ), c_type_id(typ, "*"));
  }
  else if (is_qname(typ))
    fprintf(fhead, "\n\n#define soap_s2%s(soap, s, a) soap_s2QName((soap), (s), (char**)(a), %ld, %ld, %s)", c_ident(typ), minlen(typ), maxlen(typ), pattern(typ));
  else if (is_string(typ))
    fprintf(fhead, "\n\n#define soap_s2%s(soap, s, a) soap_s2char((soap), (s), (char**)(a), %d, %ld, %ld, %s)", c_ident(typ), property(typ), minlen(typ), maxlen(typ), pattern(typ));
  else if (is_wstring(typ))
    fprintf(fhead, "\n\n#define soap_s2%s(soap, s, a) soap_s2wchar((soap), (s), (wchar_t**)(a), %d, %ld, %ld, %s)", c_ident(typ), property(typ), minlen(typ), maxlen(typ), pattern(typ));
  else if (is_stdqname(typ))
    fprintf(fhead, "\n\n#define soap_s2%s(soap, s, a) soap_s2stdQName((soap), (s), (a), %ld, %ld, %s)", c_ident(typ), minlen(typ), maxlen(typ), pattern(typ));
  else if (is_stdstring(typ))
    fprintf(fhead, "\n\n#define soap_s2%s(soap, s, a) soap_s2stdchar((soap), (s), (a), %d, %ld, %ld, %s)", c_ident(typ), property(typ), minlen(typ), maxlen(typ), pattern(typ));
  else if (is_stdwstring(typ))
    fprintf(fhead, "\n\n#define soap_s2%s(soap, s, a) soap_s2stdwchar((soap), (s), (a), %d, %ld, %ld, %s)", c_ident(typ), property(typ), minlen(typ), maxlen(typ), pattern(typ));
  else if (is_typedef(typ) && typ->type <= Tstruct && (!is_external(typ) || is_volatile(typ)) && !is_qname(typ) && !is_stdqname(typ))
  {
    if (!is_synonym(typ) && (typ->hasmin || typ->hasmax || (typ->pattern && typ->pattern[0] != '%')))
    {
      fprintf(fhead, "\nSOAP_FMAC3S int SOAP_FMAC4S soap_s2%s(struct soap*, const char*, %s);", c_ident(typ), c_type_id(typ, "*"));
      fprintf(fout, "\n\nSOAP_FMAC3S int SOAP_FMAC4S soap_s2%s(struct soap *soap, const char *s, %s)\n{", c_ident(typ), c_type_id(typ, "*a"));
      if (is_string(typ) || is_wstring(typ) || is_stdstr(typ))
        fprintf(fout, "\n\tint err = soap_s2%s(soap, s, a, %ld, %ld, %s);\n\tif (!err)\n\t{", t_ident(typ), minlen(typ), maxlen(typ), pattern(typ));
      else
      {
        fprintf(fout, "\n\tint err = soap_s2%s(soap, s, a);\n\tif (!err)\n\t{", t_ident(typ));
        if (typ->hasmin)
        {
          if (!cflag && (typ->type == Tclass || typ->type == Tstruct))
          {
            long min = minlen(typ);
            if (min > 0)
              fprintf(fout, "\n\t\tif (a->size() < %ld)\n\t\t\treturn soap->error = SOAP_LENGTH;", min);
          }
          else if ((typ->type >= Tfloat && typ->type <= Tldouble) || is_external(typ))
          {
            fprintf(fout, "\n\t\tif (*a %s %.17lG)\n\t\t\treturn soap->error = SOAP_LENGTH;", typ->incmin ? "<" : "<=", typ->rmin);
          }
          else if (typ->imin > 0 || typ->type < Tuchar || typ->type > Tullong)
          {
            int check = 1;
            switch (typ->type)
            {
              case Tchar:
                check = typ->incmin ? typ->imin > -128 : typ->imin >= -128;
                break;
              case Tshort:
                check = typ->incmin ? typ->imin > -32768 : typ->imin >= -32768;
                break;
              case Tint:
              case Tlong:
                check = typ->incmin ? typ->imin > -2147483648LL : typ->imin >= -2147483648LL;
                break;
              default:
                break;
            }
            if (check)
              fprintf(fout, "\n\t\tif (*a %s " SOAP_LONG_FORMAT ")\n\t\t\treturn soap->error = SOAP_LENGTH;", typ->incmin ? "<" : "<=", typ->imin);
          }
        }
        if (typ->hasmax)
        {
          if (!cflag && (typ->type == Tclass || typ->type == Tstruct))
          {
            long max = maxlen(typ);
            if (max >= 0)
              fprintf(fout, "\n\t\tif (a->size() > %ld)\n\t\t\treturn soap->error = SOAP_LENGTH;", max);
          }
          else if ((typ->type >= Tfloat && typ->type <= Tldouble) || is_external(typ))
          {
            fprintf(fout, "\n\t\tif (*a %s %.17lG)\n\t\t\treturn soap->error = SOAP_LENGTH;", typ->incmax ? ">" : ">=", typ->rmax);
          }
          else if (typ->imax >= 0 || typ->type < Tuchar || typ->type > Tullong)
          {
            int check = 1;
            switch (typ->type)
            {
              case Tchar:
                check = typ->incmax ? typ->imax < 127 : typ->imax <= 127;
                break;
              case Tshort:
                check = typ->incmax ? typ->imax < 32767 : typ->imax <= 32767;
                break;
              case Tint:
              case Tlong:
                check = typ->incmax ? typ->imax < 2147483647 : typ->imax <= 2147483647;
                break;
              case Tuchar:
                check = typ->incmax ? typ->imax < 255 : typ->imax <= 255;
                break;
              case Tushort:
                check = typ->incmax ? typ->imax < 65535 : typ->imax <= 65535;
                break;
              case Tuint:
              case Tulong:
                check = typ->incmax ? typ->imax < 4294967295 : typ->imax <= 4294967295;
                break;
              default:
                break;
            }
            if (check)
              fprintf(fout, "\n\t\tif (*a %s " SOAP_LONG_FORMAT ")\n\t\t\treturn soap->error = SOAP_LENGTH;", typ->incmax ? ">" : ">=", typ->imax);
          }
        }
      }
      fprintf(fout, "\n\t}\n\treturn err;\n}");
    }
    else
      fprintf(fhead, "\n\n#define soap_s2%s soap_s2%s\n", c_ident(typ), t_ident(typ));
  }
  if (is_typedef(typ) && (is_element(typ) || is_synonym(typ)) && (!is_external(typ) || is_volatile(typ)))
  {
    fprintf(fhead, "\n\n#define soap_in_%s soap_in_%s\n", c_ident(typ), t_ident(typ));
    return;
  }
  if ((is_primitive_or_string(typ) && typ->type != Tenum && typ->type != Tenumsc) || (is_external(typ) && is_volatile(typ)))
  {
    if (is_stdstring(typ))
    {
      if (is_external(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 std::string * SOAP_FMAC2 soap_in_%s(struct soap*, const char*, std::string*, const char*);", c_ident(typ));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 std::string * SOAP_FMAC4 soap_in_%s(struct soap*, const char*, std::string*, const char*);", c_ident(typ));
      if (is_stdXML(typ))
        fprintf(fout, "\n\nSOAP_FMAC3 std::string * SOAP_FMAC4 soap_in_%s(struct soap *soap, const char *tag, std::string *s, const char *type)\n{\n\tchar *t;\n\t(void)type; /* appease -Wall -Werror */\n\tif (soap_inliteral(soap, tag, &t))\n\t{\tif (!s)\n\t\t\ts = soap_new_std__string(soap, -1);\n\t\ts->assign(t);\n\t\treturn s;\n\t}\n\treturn NULL;\n}", c_ident(typ));
      else
      {
        fprintf(fout, "\n\nSOAP_FMAC3 std::string * SOAP_FMAC4 soap_in_%s(struct soap *soap, const char *tag, std::string *s, const char *type)\n{\n\t(void)type; /* appease -Wall -Werror */\n\tif (soap_element_begin_in(soap, tag, 1, NULL))\n\t\treturn NULL;\n\tif (!s)\n\t\ts = soap_new_std__string(soap, -1);\n\tif (soap->null)\n\t\tif (s)\n\t\t\ts->erase();", c_ident(typ));
        fprintf(fout, "\n\tif (soap->body && *soap->href != '#')\n\t{\tchar *t;\n\t\ts = (std::string*)soap_id_enter(soap, soap->id, s, %s, sizeof(std::string), soap->type, soap->arrayType, %s_instantiate, %s_fbase);\n\t\tif (s)\n\t\t{\tif (!(t = soap_string_in(soap, %d, %ld, %ld, %s)))\n\t\t\t\treturn NULL;\n\t\t\ts->assign(t);\n\t\t}\n\t}\n\telse\n\t\ts = (std::string*)soap_id_forward(soap, soap->href, soap_id_enter(soap, soap->id, s, %s, sizeof(std::string), soap->type, soap->arrayType, %s_instantiate, %s_fbase), 0, %s, %s, sizeof(std::string), 0, %s_finsert, NULL);\n\tif (soap->body && soap_element_end_in(soap, tag))\n\t\treturn NULL;\n\treturn s;\n}", soap_type(typ), prefix, prefix, property(typ), minlen(typ), maxlen(typ), pattern(typ), soap_type(typ), prefix, prefix, soap_type(typ), soap_type(typ), prefix);
      }
      return;
    }
    if (is_stdwstring(typ))
    {
      if (is_external(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC3 std::wstring * SOAP_FMAC4 soap_in_%s(struct soap*, const char*, std::wstring*, const char*);", c_ident(typ));
        return;
      }
      if (is_stdXML(typ))
        fprintf(fout, "\n\nSOAP_FMAC3 std::wstring * SOAP_FMAC4 soap_in_%s(struct soap *soap, const char *tag, std::wstring *s, const char *type)\n{\n\twchar_t *t;\n\t(void)type; /* appease -Wall -Werror */\n\tif (soap_inwliteral(soap, tag, &t))\n\t{\tif (!s)\n\t\t\ts = soap_new_std__wstring(soap, -1);\n\t\ts->assign(t);\n\t\treturn s;\n\t}\n\treturn NULL;\n}", c_ident(typ));
      else
      {
        fprintf(fhead, "\nSOAP_FMAC3 std::wstring * SOAP_FMAC4 soap_in_%s(struct soap*, const char*, std::wstring*, const char*);", c_ident(typ));
        fprintf(fout, "\n\nSOAP_FMAC3 std::wstring * SOAP_FMAC4 soap_in_%s(struct soap *soap, const char *tag, std::wstring *s, const char *type)\n{\n\t(void)type; /* appease -Wall -Werror */\n\tif (soap_element_begin_in(soap, tag, 1, NULL))\n\t\treturn NULL;\n\tif (!s)\n\t\ts = soap_new_std__wstring(soap, -1);\n\tif (soap->null)\n\t\tif (s)\n\t\t\ts->erase();", c_ident(typ));
        fprintf(fout, "\n\tif (soap->body && *soap->href != '#')\n\t{\twchar_t *t;\n\t\ts = (std::wstring*)soap_id_enter(soap, soap->id, s, %s, sizeof(std::wstring), soap->type, soap->arrayType, %s_instantiate, %s_fbase);\n\t\tif (s)\n\t\t{\tif (!(t = soap_wstring_in(soap, %d, %ld, %ld, %s)))\n\t\t\t\treturn NULL;\n\t\t\ts->assign(t);\n\t\t}\n\t}\n\telse\n\t\ts = (std::wstring*)soap_id_forward(soap, soap->href, soap_id_enter(soap, soap->id, s, %s, sizeof(std::wstring), soap->type, soap->arrayType, %s_instantiate, %s_fbase), 0, %s, %s, sizeof(std::wstring), 0, %s_finsert, NULL);\n\tif (soap->body && soap_element_end_in(soap, tag))\n\t\treturn NULL;\n\treturn s;\n}", soap_type(typ), prefix, prefix, property(typ), minlen(typ), maxlen(typ), pattern(typ), soap_type(typ), prefix, prefix, soap_type(typ), soap_type(typ), prefix);
      }
      return;
    }
    if (is_external(typ) && !is_volatile(typ))
    {
      fprintf(fhead, "\nSOAP_FMAC1 %s * SOAP_FMAC2 soap_in_%s(struct soap*, const char*, %s, const char*);", c_type(typ), c_ident(typ), c_type_id(typ, "*"));
      return;
    }
    fprintf(fhead, "\nSOAP_FMAC3 %s * SOAP_FMAC4 soap_in_%s(struct soap*, const char*, %s, const char*);", c_type(typ), c_ident(typ), c_type_id(typ, "*"));
    fprintf(fout, "\n\nSOAP_FMAC3 %s * SOAP_FMAC4 soap_in_%s(struct soap *soap, const char *tag, %s, const char *type)\n{", c_type(typ), c_ident(typ), c_type_id(typ, "*a"));
    if (is_wstring(typ))
      fprintf(fout, "\n\ta = soap_inwstring(soap, tag, a, type, %s, %d, %ld, %ld, %s);", soap_type(typ), property(typ), minlen(typ), maxlen(typ), pattern(typ));
    else if (is_string(typ))
      fprintf(fout, "\n\ta = soap_instring(soap, tag, a, type, %s, %d, %ld, %ld, %s);", soap_type(typ), property(typ), minlen(typ), maxlen(typ), pattern(typ));
    else
    {
      if (typ->type == Tllong || typ->type == Tullong)
        fprintf(fout, "\n\ta = soap_in%s(soap, tag, a, type, %s);", c_type(typ), soap_type(typ));
      else if (is_primitive_or_string(typ))
        fprintf(fout, "\n\ta = soap_in%s(soap, tag, a, type, %s);", the_type(typ), soap_type(typ));
      else
        fprintf(fout, "\n\ta = soap_in_%s(soap, tag, a, type);", t_ident(typ));
      if (typ->hasmin)
      {
        if (!cflag && (typ->type == Tclass || typ->type == Tstruct))
        {
          long min = minlen(typ);
          if (min > 0)
            fprintf(fout, "\n\tif (a && a->size() < %ld)\n\t{\tsoap->error = SOAP_LENGTH;\n\t\treturn NULL;\n\t}", min);
        }
        else if ((typ->type >= Tfloat && typ->type <= Tldouble) || is_external(typ))
        {
          fprintf(fout, "\n\tif (a && *a %s %.17lG)\n\t{\tsoap->error = SOAP_LENGTH;\n\t\treturn NULL;\n\t}", typ->incmin ? "<" : "<=", typ->rmin);
        }
        else if (typ->imin > 0 || typ->type < Tuchar || typ->type > Tullong)
        {
          int check = 1;
          switch (typ->type)
          {
            case Tchar:
              check = typ->incmin ? typ->imin > -128 : typ->imin >= -128;
              break;
            case Tshort:
              check = typ->incmin ? typ->imin > -32768 : typ->imin >= -32768;
              break;
            case Tint:
            case Tlong:
              check = typ->incmin ? typ->imin > -2147483648LL : typ->imin >= -2147483648LL;
              break;
            default:
              break;
          }
          if (check)
            fprintf(fout, "\n\tif (a && *a %s " SOAP_LONG_FORMAT ")\n\t{\tsoap->error = SOAP_LENGTH;\n\t\treturn NULL;\n\t}", typ->incmin ? "<" : "<=", typ->imin);
        }
      }
      if (typ->hasmax)
      {
        if (!cflag && (typ->type == Tclass || typ->type == Tstruct))
        {
          long max = maxlen(typ);
          if (max >= 0)
            fprintf(fout, "\n\tif (a && a->size() > %ld)\n\t{\tsoap->error = SOAP_LENGTH;\n\t\treturn NULL;\n\t}", max);
        }
        else if ((typ->type >= Tfloat && typ->type <= Tldouble) || is_external(typ))
        {
          fprintf(fout, "\n\tif (a && *a %s %.17lG)\n\t{\tsoap->error = SOAP_LENGTH;\n\t\treturn NULL;\n\t}", typ->incmax ? ">" : ">=", typ->rmax);
        }
        else if (typ->imax >= 0 || typ->type < Tuchar || typ->type > Tullong)
        {
          int check = 1;
          switch (typ->type)
          {
            case Tchar:
              check = typ->incmax ? typ->imax < 127 : typ->imax <= 127;
              break;
            case Tshort:
              check = typ->incmax ? typ->imax < 32767 : typ->imax <= 32767;
              break;
            case Tint:
            case Tlong:
              check = typ->incmax ? typ->imax < 2147483647 : typ->imax <= 2147483647;
              break;
            case Tuchar:
              check = typ->incmax ? typ->imax < 255 : typ->imax <= 255;
              break;
            case Tushort:
              check = typ->incmax ? typ->imax < 65535 : typ->imax <= 65535;
              break;
            case Tuint:
            case Tulong:
              check = typ->incmax ? typ->imax < 4294967295 : typ->imax <= 4294967295;
              break;
            default:
              break;
          }
          if (check)
            fprintf(fout, "\n\tif (a && *a %s " SOAP_LONG_FORMAT ")\n\t{\tsoap->error = SOAP_LENGTH;\n\t\treturn NULL;\n\t}", typ->incmax ? ">" : ">=", typ->imax);
        }
      }
    }
    fprintf(fout, "\n\treturn a;\n}");
    fflush(fout);
    return;
  }
  if (is_fixedstring(typ))
  {
    int n = typ->width / ((Tnode*)typ->ref)->width;
    fprintf(fhead, "\nSOAP_FMAC3 char* SOAP_FMAC4 soap_in_%s(struct soap*, const char*, char[], const char*);", c_ident(typ));
    fprintf(fout, "\n\nSOAP_FMAC3 char* SOAP_FMAC4 soap_in_%s(struct soap *soap, const char *tag, char a[], const char *type)\n{\tchar *p = NULL;\n\tif (!soap_instring(soap, tag, &p, type, %s, 1, 0, %d, %s))\n\t\treturn NULL;\n\tif (!p)\n\t{\tif (*soap->href == '#')\n\t\t\tsoap_id_nullify(soap, soap->href);\n\t\telse\n\t\t\tsoap->error = SOAP_NULL;\n\t\treturn NULL;\n\t}\n\tsoap_strncpy(a, %d, p, %d);\n\treturn a;\n}", c_ident(typ), soap_type(typ), n - 1, pattern(typ), n, n - 1);
    return;
  }
  switch(typ->type)
  {
    case Tstruct:
      if (is_external(typ) && !is_volatile(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 %s SOAP_FMAC2 soap_in_%s(struct soap*, const char*, %s, const char*);", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 %s SOAP_FMAC4 soap_in_%s(struct soap*, const char*, %s, const char*);", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*"));
      fprintf(fout, "\n\nSOAP_FMAC3 %s SOAP_FMAC4 soap_in_%s(struct soap *soap, const char *tag, %s, const char *type)\n{", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*a"));
      table = (Table*)typ->ref;
      if (!table)
      {
        fprintf(fout, "\n\t(void)soap; (void)tag; (void)a; (void)type; /* appease -Wall -Werror */");
        fprintf(fout, "\n\tif (!a)\n\t\tsoap->error = SOAP_TAG_MISMATCH;\n\treturn a;\n}");
      }
      else if (is_primclass(typ))
      {
        for (p = table->list; p; p = p->next)
        {
          if (is_pointer_to_derived(p))
          {
            der = 1;
            break;
          }
        }
        if (der)
          fprintf(fout, "\n\tint err = soap_element_begin_in(soap, tag, 1, type);\n\tif (err && err != SOAP_TYPE)\n\t\treturn NULL;");
        else
          fprintf(fout, "\n\tif (soap_element_begin_in(soap, tag, 1, NULL))\n\t\treturn NULL;");
        fprintf(fout, "\n\t(void)type; /* appease -Wall -Werror */");
        if (!cflag)
          fprintf(fout, "\n\tif (!(a = (%s)soap_id_enter(soap, soap->id, a, %s, sizeof(%s), soap->type, soap->arrayType, %s_instantiate, %s_fbase)))\n\t\treturn NULL;", c_type_id(typ, "*"), soap_type(typ), c_type(typ), prefix, prefix);
        else
          fprintf(fout, "\n\tif (!(a = (%s)soap_id_enter(soap, soap->id, a, %s, sizeof(%s), NULL, NULL, NULL, NULL)))\n\t\treturn NULL;", c_type_id(typ, "*"), soap_type(typ), c_type(typ));
        fprintf(fout, "\n\tsoap_revert(soap);\n\t*soap->id = '\\0';");
        fprintf(fout, "\n\tsoap_default_%s(soap, a);", c_ident(typ));
        if (der)
        {
          fprintf(fout, "\n\tif (err == SOAP_TYPE)\n\t{");
          for (p = table->list; p; p = p->next)
          {
            if (is_pointer_to_derived(p))
            {
              fprintf(fout, "\n\t\tif ((");
              gen_match_derived(fout, p->info.typ->ref);
              fprintf(fout, ") && (a->%s = soap_in_%s(soap, tag, NULL, NULL)) != NULL)\n\t\t\treturn a;", ident(p->sym->name), c_ident(p->info.typ->ref));
            }
          }
          fprintf(fout, "\n\t\treturn NULL;\n\t}");
        }
        for (t = (Table*)typ->ref; t; t = t->prev)
          for (p = t->list; p; p = p->next)
            if (p->info.sto & Sattribute)
              soap_attr_value(p, "a", ident(p->sym->name), ns_add(p, nsa));
        fflush(fout);
        for (table = (Table*)typ->ref; table; table = table->prev)
        {
          p = table->list;
          while (p && !is_item(p))
            p = p->next;
          if (p)
            break;
        }
        if (is_XML(p->info.typ) && is_string(p->info.typ))
          fprintf(fout, "\n\tif (!soap_inliteral(soap, tag, (char**)&a->%s))", ident(p->sym->name));
        else if (is_XML(p->info.typ) && is_wstring(p->info.typ))
          fprintf(fout, "\n\tif (!soap_inwliteral(soap, tag, (wchar_t**)&a->%s))", ident(p->sym->name));
        else if (is_string(p->info.typ))
          fprintf(fout, "\n\tif (!soap_in_%s(soap, tag, (char**)&a->%s, \"%s\"))", c_ident(p->info.typ), ident(p->sym->name), xsi_type(typ));
        else if (is_wstring(p->info.typ))
          fprintf(fout, "\n\tif (!soap_in_%s(soap, tag, (wchar_t**)&a->%s, \"%s\"))", c_ident(p->info.typ), ident(p->sym->name), xsi_type(typ));
        else if (p->info.typ->type == Tarray)
          fprintf(fout, "\n\tif (!soap_in_%s(soap, tag, a->%s, \"%s\"))", c_ident(p->info.typ), ident(p->sym->name), xsi_type(typ));
        else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
          fprintf(fout, "\n\tif (!a->%s.soap_in(soap, tag, \"%s\"))", ident(p->sym->name), xsi_type(typ));
        else if (p->info.typ->type != Tfun && !is_void(p->info.typ))
          fprintf(fout, "\n\tif (!soap_in_%s(soap, tag, &a->%s, \"%s\"))", c_ident(p->info.typ), ident(p->sym->name), xsi_type(typ));
        fprintf(fout, "\n\t\treturn NULL;");
        fixed_check(fout, p, NULL, "\t");
        if (has_getter(typ))
          fprintf(fout, "\n\tif (a->get(soap))\n\t\treturn NULL;");
        fprintf(fout, "\n\treturn a;\n}");
      }
      else
      {
        table = (Table*)typ->ref;
        if (!is_discriminant(typ))
        {
          for (t = table; t; t = t->prev)
          {
            for (p = t->list; p; p = p->next)
            {
              if (!(p->info.sto & (Sconst | Sprivate | Sprotected)) && !(p->info.sto & Sattribute) && p->info.typ->type != Tfun && !is_void(p->info.typ))
              {
                if (is_anytype(p) || is_choice(p))
                {
                  p = p->next;
                  fprintf(fout, "\n\tsize_t soap_flag_%s = " SOAP_LONG_FORMAT ";", ident(p->sym->name), p->info.maxOccurs);
                }
                else if (is_repetition(p))
                {
                  fprintf(fout, "\n\tstruct soap_blist *soap_blist_%s = NULL;", ident(p->next->sym->name));
                  p = p->next;
                }
                else if (!is_transient(p->info.typ) && !is_container(p->info.typ))
                  fprintf(fout, "\n\tsize_t soap_flag_%s = " SOAP_LONG_FORMAT ";", ident(p->sym->name), p->info.maxOccurs);
              }
            }
          }
        }
        if (!is_invisible(typ->id->name))
        {
          for (p = table->list; p; p = p->next)
          {
            if (is_pointer_to_derived(p))
            {
              der = 1;
              break;
            }
          }
          if (der)
            fprintf(fout, "\n\tint err = soap_element_begin_in(soap, tag, 0, type);\n\tif (err && err != SOAP_TYPE)\n\t\treturn NULL;");
          else
            fprintf(fout, "\n\tif (soap_element_begin_in(soap, tag, 0, NULL))\n\t\treturn NULL;");
          fprintf(fout, "\n\t(void)type; /* appease -Wall -Werror */");
        }
        else if (!is_discriminant(typ))
        {
          if (table && (table->prev || table->list))
            fprintf(fout, "\n\tshort soap_flag;");
          fprintf(fout, "\n\t(void)tag; (void)type; /* appease -Wall -Werror */");
        }
        else
        {
          fprintf(fout, "\n\t(void)tag; (void)type; /* appease -Wall -Werror */");
        }
        if (has_class(typ))
        {
          if (is_invisible(typ->id->name))
            fprintf(fout, "\n\ta = (%s*)soap_id_enter(soap, \"\", a, %s, sizeof(%s), soap->type, soap->arrayType, %s_instantiate, %s_fbase);", c_type(typ), soap_type(typ), c_type(typ), prefix, prefix);
          else
            fprintf(fout, "\n\ta = (%s*)soap_id_enter(soap, soap->id, a, %s, sizeof(%s), soap->type, soap->arrayType, %s_instantiate, %s_fbase);", c_type(typ), soap_type(typ), c_type(typ), prefix, prefix);
        }
        else if (is_invisible(typ->id->name))
          fprintf(fout, "\n\ta = (%s*)soap_id_enter(soap, \"\", a, %s, sizeof(%s), NULL, NULL, NULL, NULL);", c_type(typ), soap_type(typ), c_type(typ));
        else
          fprintf(fout, "\n\ta = (%s*)soap_id_enter(soap, soap->id, a, %s, sizeof(%s), NULL, NULL, NULL, NULL);", c_type(typ), soap_type(typ), c_type(typ));
        fprintf(fout, "\n\tif (!a)\n\t\treturn NULL;");
        fprintf(fout, "\n\tsoap_default_%s(soap, a);", c_ident(typ));
        if (zflag == 0 || zflag > 4)
        {
          for (t = table; t; t = t->prev)
          {
            for (p = t->list; p; p = p->next)
            {
              if (is_choice(p) || is_repetition(p))
              {
                p = p->next;
                continue;
              }
              if (!p->info.hasval || p->info.minOccurs > 0 || p->info.typ->type == Tfun || p->info.typ->type == Tunion || is_soapref(p->info.typ) || (p->info.sto & Sconst) || (p->info.sto & (Sprivate | Sprotected)) || (p->info.sto & Sattribute) || is_anytype(p) || is_transient(p->info.typ))
                continue;
              if (is_fixedstring(p->info.typ))
                fprintf(fout, "\n\ta->%s[0] = '\\0';", ident(p->sym->name));
              else if (is_string(p->info.typ) || is_wstring(p->info.typ))
                fprintf(fout, "\n\ta->%s = NULL;", ident(p->sym->name));
            }
          }
        }
        if (der)
        {
          fprintf(fout, "\n\tif (err == SOAP_TYPE)\n\t{");
          for (p = table->list; p; p = p->next)
          {
            if (is_pointer_to_derived(p))
            {
              fprintf(fout, "\n\t\tif ((");
              gen_match_derived(fout, p->info.typ->ref);
              fprintf(fout, ") && (a->%s = soap_in_%s(soap, tag, NULL, \"%s\")) != NULL)\n\t\t\treturn a;", ident(p->sym->name), c_ident(p->info.typ->ref), xsi_type(p->info.typ->ref));
            }
          }
          fprintf(fout, "\n\t\treturn NULL;\n\t}");
        }
        for (t = table; t; t = t->prev)
          for (p = t->list; p; p = p->next)
            if (p->info.sto & Sattribute)
              soap_attr_value(p, "a", ident(p->sym->name), ns_add(p, nsa));
        if (!is_invisible(typ->id->name))
        {
          if (!is_discriminant(typ))
          {
            fprintf(fout, "\n\tif (soap->body && *soap->href != '#')\n\t{");
            fprintf(fout, "\n\t\tfor (;;)\n\t\t{\tsoap->error = SOAP_TAG_MISMATCH;");
          }
          else
            fprintf(fout, "\n\tif (!tag || *tag == '-' || (soap->body && *soap->href != '#'))\n\t{");
        }
        else if (!is_discriminant(typ))
        {
          if (table && (table->prev || table->list))
            fprintf(fout, "\n\t\tfor (soap_flag = 0;; soap_flag = 1)\n\t\t{\tsoap->error = SOAP_TAG_MISMATCH;");
        }
        flag = 0;
        for (t = table; t; t = t->prev)
        {
          for (p = t->list; p; p = p->next)
          {
            if (p->info.typ->type == Tfun || p->info.typ->type == Tunion || is_soapref(p->info.typ))
              ;
            else if (p->info.sto & (Sconst | Sprivate | Sprotected))
              fprintf(fout, "\n\t\t/* non-serializable %s skipped */", ident(p->sym->name));
            else if (p->info.sto & Sattribute)
              ;
            else if (is_repetition(p))
            {
              if (is_unmatched(p->next->sym) || is_invisible(p->next->sym->name))
              {
                p = p->next;
                continue;
              }
              flag = 1;
              fprintf(fout, "\n\t\t\tif (soap->error == SOAP_TAG_MISMATCH && ");
              fprintf(fout, "!soap_element_begin_in(soap, %s, 1, NULL))", field(p->next, nse));
              fprintf(fout, "\n\t\t\t{\tif (a->%s == NULL)\n\t\t\t\t{\tif (soap_blist_%s == NULL)\n\t\t\t\t\t\tsoap_blist_%s = soap_alloc_block(soap);", ident(p->next->sym->name), ident(p->next->sym->name), ident(p->next->sym->name));
              if (((Tnode*)p->next->info.typ->ref)->type == Tclass
               || ((Tnode*)p->next->info.typ->ref)->type == Ttemplate
               || has_class((Tnode*)p->next->info.typ->ref)
               || (!cflag && ((Tnode*)p->next->info.typ->ref)->type == Tstruct))
                fprintf(fout, "\n\t\t\t\t\ta->%s = soap_block<%s>::push(soap, soap_blist_%s);\n\t\t\t\t\tif (a->%s == NULL)\n\t\t\t\t\t\treturn NULL;", ident(p->next->sym->name), c_type(p->next->info.typ->ref), ident(p->next->sym->name), ident(p->next->sym->name));
              else if (((Tnode*)p->next->info.typ->ref)->type == Tpointer && (p->next->info.sto & Sconstptr))
                fprintf(fout, "\n\t\t\t\t\ta->%s = (const %s)soap_push_block_max(soap, soap_blist_%s, sizeof(%s));\n\t\t\t\t\tif (a->%s == NULL)\n\t\t\t\t\t\treturn NULL;", ident(p->next->sym->name), c_type(p->next->info.typ), ident(p->next->sym->name), c_type((Tnode*)p->next->info.typ->ref), ident(p->next->sym->name));
              else
                fprintf(fout, "\n\t\t\t\t\ta->%s = (%s)soap_push_block_max(soap, soap_blist_%s, sizeof(%s));\n\t\t\t\t\tif (a->%s == NULL)\n\t\t\t\t\t\treturn NULL;", ident(p->next->sym->name), c_type(p->next->info.typ), ident(p->next->sym->name), c_type((Tnode*)p->next->info.typ->ref), ident(p->next->sym->name));
              if (((Tnode*)p->next->info.typ->ref)->type == Tclass && !is_external((Tnode*)p->next->info.typ->ref) && !is_volatile((Tnode*)p->next->info.typ->ref) && !is_typedef((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\t\t\ta->%s->soap_default(soap);", ident(p->next->sym->name));
              else if (((Tnode*)p->next->info.typ->ref)->type != Tpointer  && !is_XML((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\t\t\tsoap_default_%s(soap, a->%s);", c_ident((Tnode*)p->next->info.typ->ref), ident(p->next->sym->name));
              else
                fprintf(fout, "\n\t\t\t\t\t*a->%s = NULL;", ident(p->next->sym->name));
              fprintf(fout, "\n\t\t\t\t}");
              fprintf(fout, "\n\t\t\t\tsoap_revert(soap);");
              if (is_XML((Tnode*)p->next->info.typ->ref) && is_string((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\t\tif (soap_inliteral(soap, %s, (char**)a->%s))", field(p->next, nse), ident(p->next->sym->name));
              else if (is_XML((Tnode*)p->next->info.typ->ref) && is_wstring((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\t\tif (soap_inwliteral(soap, %s, (wchar_t**)a->%s))", field(p->next, nse), ident(p->next->sym->name));
              else if (is_string((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\t\tif (soap_in_%s(soap, %s, (char**)a->%s, \"%s\"))", c_ident((Tnode*)p->next->info.typ->ref), field(p->next, nse), ident(p->next->sym->name), xsi_type((Tnode*)p->next->info.typ->ref));
              else if (is_wstring((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\t\tif (soap_in_%s(soap, %s, (wchar_t**)a->%s, \"%s\"))", c_ident((Tnode*)p->next->info.typ->ref), field(p->next, nse), ident(p->next->sym->name), xsi_type((Tnode*)p->next->info.typ->ref));
              else
                fprintf(fout, "\n\t\t\t\tif (soap_in_%s(soap, %s, a->%s, \"%s\"))", c_ident((Tnode*)p->next->info.typ->ref), field(p->next, nse), ident(p->next->sym->name), xsi_type((Tnode*)p->next->info.typ->ref));
              fprintf(fout, "\n\t\t\t\t{\ta->%s++;\n\t\t\t\t\ta->%s = NULL;\n\t\t\t\t\tcontinue;\n\t\t\t\t}\n\t\t\t}", ident(p->sym->name), ident(p->next->sym->name));
              p = p->next;
            }
            else if (is_anytype(p))
            {
              flag = 1;
              fprintf(fout, "\n\t\t\tif (soap_flag_%s && soap->error == SOAP_TAG_MISMATCH)", ident(p->next->sym->name));
              if (is_self(p->next))
                fprintf(fout, "\n\t\t\t{\tif ((a->%s = soap_getelement(soap, tag, &a->%s)))", ident(p->next->sym->name), ident(p->sym->name));
              else if (is_invisible(p->next->sym->name))
                fprintf(fout, "\n\t\t\t{\tif ((a->%s = soap_getelement(soap, NULL, &a->%s)))", ident(p->next->sym->name), ident(p->sym->name));
              else
                fprintf(fout, "\n\t\t\t{\tif ((a->%s = soap_getelement(soap, %s, &a->%s)))", ident(p->next->sym->name), field(p->next, nse), ident(p->sym->name));
              fprintf(fout, "\n\t\t\t\t{\tsoap_flag_%s = 0;", ident(p->next->sym->name));
              fprintf(fout, "\n\t\t\t\t\tcontinue;");
              fprintf(fout, "\n\t\t\t\t}\n\t\t\t}");
              p = p->next;
            }
            else if (is_discriminant(typ) && p->next)
            {
              flag = 1;
              fprintf(fout, "\n\t\tif (!soap_in_%s(soap, &a->%s, &a->%s))", c_ident(p->next->info.typ), ident(p->sym->name), ident(p->next->sym->name));
              fprintf(fout, "\n\t\t\treturn NULL;");
              break;
            }
            else if (is_choice(p))
            {
              flag = 1;
              fprintf(fout, "\n\t\t\tif (soap_flag_%s && soap->error == SOAP_TAG_MISMATCH)", ident(p->next->sym->name));
              fprintf(fout, "\n\t\t\t{\tif (soap_in_%s(soap, &a->%s, &a->%s))", c_ident(p->next->info.typ), ident(p->sym->name), ident(p->next->sym->name));
              fprintf(fout, "\n\t\t\t\t{\tsoap_flag_%s = 0;", ident(p->next->sym->name));
              fprintf(fout, "\n\t\t\t\t\tcontinue;");
              fprintf(fout, "\n\t\t\t\t}\n\t\t\t}");
              p = p->next;
            }
            else if (is_transient(p->info.typ))
            {
              if (!is_pointer_to_derived(p))
                fprintf(fout, "\n\t\t\t/* transient %s skipped */", ident(p->sym->name));
            }
            else
            {
              flag = 1;
              if (!is_invisible(p->sym->name) && !is_primclass(typ) && p->info.typ->type != Tfun && !is_void(p->info.typ))
              {
                if (is_string(p->info.typ) || is_wstring(p->info.typ) || is_stdstr(p->info.typ))
                  fprintf(fout, "\n\t\t\tif (soap_flag_%s && (soap->error == SOAP_TAG_MISMATCH || soap->error == SOAP_NO_TAG))", ident(p->sym->name));
                else if (is_container(p->info.typ))
                  fprintf(fout, "\n\t\t\tif (soap->error == SOAP_TAG_MISMATCH)");
                else
                  fprintf(fout, "\n\t\t\tif (soap_flag_%s && soap->error == SOAP_TAG_MISMATCH)", ident(p->sym->name));
                fprintf(fout, "\n\t\t\t{\t");
              }
              if (is_unmatched(p->sym))
              {
                if (is_XML(p->info.typ) && is_string(p->info.typ))
                {
                  fprintf(fout, "if (soap_inliteral(soap, NULL, (char**)&a->%s))", ident(p->sym->name));
                }
                else if (is_XML(p->info.typ) && is_wstring(p->info.typ))
                {
                  fprintf(fout, "if (soap_inwliteral(soap, NULL, (wchar_t**)&a->%s))", ident(p->sym->name));
                }
                else if (is_string(p->info.typ))
                {
                  fprintf(fout, "if (soap_in_%s(soap, NULL, (char**)&a->%s, \"%s\"))", c_ident(p->info.typ), ident(p->sym->name), xsi_type(p->info.typ));
                }
                else if (is_wstring(p->info.typ))
                {
                  fprintf(fout, "if (soap_in_%s(soap, NULL, (wchar_t**)&a->%s, \"%s\"))", c_ident(p->info.typ), ident(p->sym->name), xsi_type(p->info.typ));
                }
                else if (p->info.typ->type == Tarray)
                {
                  fprintf(fout, "if (soap_in_%s(soap, NULL, a->%s, \"%s\"))", c_ident(p->info.typ), ident(p->sym->name), xsi_type(p->info.typ));
                }
                else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
                {
                  fprintf(fout, "if (a->%s.soap_in(soap, NULL, \"%s\"))", ident(p->sym->name), xsi_type(p->info.typ));
                }
                else if (p->info.typ->type != Tfun && !is_void(p->info.typ))
                {
                  fprintf(fout, "if (soap_in_%s(soap, NULL, &a->%s, \"%s\"))", c_ident(p->info.typ), ident(p->sym->name), xsi_type(p->info.typ));
                }
              }
              else if (!is_invisible(p->sym->name))
              {
                if (is_XML(p->info.typ) && is_string(p->info.typ))
                {
                  fprintf(fout, "if (soap_inliteral(soap, %s, (char**)&a->%s))", field(p, nse), ident(p->sym->name));
                }
                else if (is_XML(p->info.typ) && is_wstring(p->info.typ))
                {
                  fprintf(fout, "if (soap_inwliteral(soap, %s, (wchar_t**)&a->%s))", field(p, nse), ident(p->sym->name));
                }
                else if (is_string(p->info.typ))
                {
                  fprintf(fout, "if (soap_in_%s(soap, %s, (char**)&a->%s, \"%s\"))", c_ident(p->info.typ), field(p, nse), ident(p->sym->name), xsi_type(p->info.typ));
                }
                else if (is_wstring(p->info.typ))
                {
                  fprintf(fout, "if (soap_in_%s(soap, %s, (wchar_t**)&a->%s, \"%s\"))", c_ident(p->info.typ), field(p, nse), ident(p->sym->name), xsi_type(p->info.typ));
                }
                else if (p->info.typ->type == Tarray)
                {
                  fprintf(fout, "if (soap_in_%s(soap, %s, a->%s, \"%s\"))", c_ident(p->info.typ), field(p, nse), ident(p->sym->name), xsi_type(p->info.typ));
                }
                else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
                {
                  fprintf(fout, "if (a->%s.soap_in(soap, %s, \"%s\"))", ident(p->sym->name), field(p, nse), xsi_type(p->info.typ));
                }
                else if (p->info.typ->type != Tfun && !is_void(p->info.typ))
                {
                  fprintf(fout, "if (soap_in_%s(soap, %s, &a->%s, \"%s\"))", c_ident(p->info.typ), field(p, nse), ident(p->sym->name), xsi_type(p->info.typ));
                }
              }
              if (!is_invisible(p->sym->name) && !is_primclass(typ) && p->info.typ->type != Tfun && !is_void(p->info.typ))
              {
                if (is_container(p->info.typ))
                {
                  fprintf(fout, "\n\t\t\t\t\tcontinue;");
                }
                else
                {
                  fprintf(fout, "\n\t\t\t\t{\tsoap_flag_%s--;", ident(p->sym->name));
                  fixed_check(fout, p, NULL, "\t\t\t\t\t");
                  if (p->info.hasval && (is_string(p->info.typ) || is_wstring(p->info.typ)))
                  {
                    fprintf(fout, "\n\t\t\t\t\tif (*soap->href != '#' && a->%s && ", ident(p->sym->name));
                    fprintf(fout, "!*a->%s)", ident(p->sym->name));
                    fprintf(fout, "\n\t\t\t\t\t\ta->%s%s;", ident(p->sym->name), c_init(p));
                  }
                  else if (p->info.hasval && is_fixedstring(p->info.typ))
                  {
                    fprintf(fout, "\n\t\t\t\t\tif (*soap->href != '#' && ");
                    fprintf(fout, "!*a->%s)", ident(p->sym->name));
                    fprintf(fout, "\n\t\t\t\t\t\tsoap_strcpy(a->%s, %d, \"%s\");", ident(p->sym->name), get_dimension(p->info.typ), cstring(p->info.val.s, 0));
                  }
                  else if (p->info.ptrval && (is_string(p->info.typ->ref) || is_wstring(p->info.typ->ref) || is_stdstring(p->info.typ->ref) || is_stdwstring(p->info.typ->ref)))
                  {
                    Tnode *ptr = (Tnode*)p->info.typ->ref;
                    if (is_smart(p->info.typ))
                      fprintf(fout, "\n\t\t\t\t\tif (*soap->href != '#' && a->%s.get() && ", ident(p->sym->name));
                    else
                      fprintf(fout, "\n\t\t\t\t\tif (*soap->href != '#' && a->%s && ", ident(p->sym->name));
                    if (is_string(ptr) || is_wstring(ptr))
                      fprintf(fout, "*a->%s && !**a->%s)", ident(p->sym->name), ident(p->sym->name));
                    else
                      fprintf(fout, "a->%s->empty())", ident(p->sym->name));
                    fprintf(fout, "\n\t\t\t\t\t\t*a->%s%s;", ident(p->sym->name), c_init(p));
                  }
                  fprintf(fout, "\n\t\t\t\t\tcontinue;");
                  fprintf(fout, "\n\t\t\t\t}");
                  if (p->info.ptrval && !p->info.fixed && !(is_string(p->info.typ->ref) || is_wstring(p->info.typ->ref) || is_stdstring(p->info.typ->ref) || is_stdwstring(p->info.typ->ref)))
                  {
                    Tnode *ptr = (Tnode*)p->info.typ->ref;
                    fprintf(fout, "\n\t\t\t\tif (soap->error == SOAP_EMPTY)\n\t\t\t\t{\t");
                    if (is_smart(p->info.typ))
                    {
                      if (is_smart_shared(p->info.typ))
                        fprintf(fout, "a->%s = %s<%s>();", ident(p->sym->name), make_shared(p->info.typ), c_type(ptr));
                      else if (ptr->type == Tclass && !is_external(ptr) && !is_volatile(ptr) && !is_typedef(ptr))
                        fprintf(fout, "a->%s = %s(%s_instantiate_%s(soap, SOAP_NO_LINK_TO_DELETE, soap->type, soap->arrayType, NULL));", ident(p->sym->name), c_type(p->info.typ), fprefix, c_ident(ptr));
                      else
                        fprintf(fout, "a->%s = %s(SOAP_NEW(soap, %s));", ident(p->sym->name), c_type(p->info.typ), c_type(ptr));
                    }
                    else if (ptr->type == Tclass)
                    {
                      fprintf(fout, "if (!(a->%s = (%s)%s_instantiate_%s(soap, -1, soap->type, soap->arrayType, NULL)))\n\t\t\t\t\t\treturn NULL;", ident(p->sym->name), c_type(p->info.typ), fprefix, c_ident(ptr));
                    }
                    else
                    {
                      fprintf(fout, "if (!(a->%s = (%s)soap_malloc(soap, sizeof(%s))))\n\t\t\t\t\t\treturn NULL;", ident(p->sym->name), c_type(p->info.typ), c_type(ptr));
                    }
                    fprintf(fout, "\n\t\t\t\t\t*a->%s%s;", ident(p->sym->name), c_init(p));
                    fprintf(fout, "\n\t\t\t\t\tsoap->error = SOAP_OK;\n\t\t\t\t\tsoap_flag_%s--;\n\t\t\t\t\tcontinue;\n\t\t\t\t}", ident(p->sym->name));
                  }
                }
                fprintf(fout, "\n\t\t\t}");
              }
            }
            fflush(fout);
          }
        }
        if (!is_discriminant(typ))
        {
          for (t = table; t; t = t->prev)
          {
            for (p = t->list; p; p = p->next)
            {
              if (is_repetition(p) && (is_unmatched(p->next->sym) || (is_invisible(p->next->sym->name))))
              {
                flag = 1;
                fprintf(fout, "\n\t\t\tif (soap->error == SOAP_TAG_MISMATCH && ");
                if (is_unmatched(p->next->sym))
                  fprintf(fout, "!soap_element_begin_in(soap, NULL, 1, NULL))");
                else if (is_invisible(p->next->sym->name))
                  fprintf(fout, "!soap_peek_element(soap))");
                fprintf(fout, "\n\t\t\t{\tif (a->%s == NULL)\n\t\t\t\t{\tif (soap_blist_%s == NULL)\n\t\t\t\t\t\tsoap_blist_%s = soap_alloc_block(soap);", ident(p->next->sym->name), ident(p->next->sym->name), ident(p->next->sym->name));
                if (((Tnode*)p->next->info.typ->ref)->type == Tclass
                 || ((Tnode*)p->next->info.typ->ref)->type == Ttemplate
                 || has_class((Tnode*)p->next->info.typ->ref)
                 || (!cflag && ((Tnode*)p->next->info.typ->ref)->type == Tstruct))
                  fprintf(fout, "\n\t\t\t\t\ta->%s = soap_block<%s>::push(soap, soap_blist_%s);\n\t\t\t\t\tif (a->%s == NULL)\n\t\t\t\t\t\treturn NULL;", ident(p->next->sym->name), c_type(p->next->info.typ->ref), ident(p->next->sym->name), ident(p->next->sym->name));
                else if (((Tnode*)p->next->info.typ->ref)->type == Tpointer && (p->next->info.sto & Sconstptr))
                  fprintf(fout, "\n\t\t\t\t\ta->%s = (const %s)soap_push_block_max(soap, soap_blist_%s, sizeof(%s));\n\t\t\t\t\tif (a->%s == NULL)\n\t\t\t\t\t\treturn NULL;", ident(p->next->sym->name), c_type(p->next->info.typ), ident(p->next->sym->name), c_type((Tnode*)p->next->info.typ->ref), ident(p->next->sym->name));
                else
                  fprintf(fout, "\n\t\t\t\t\ta->%s = (%s)soap_push_block_max(soap, soap_blist_%s, sizeof(%s));\n\t\t\t\t\tif (a->%s == NULL)\n\t\t\t\t\t\treturn NULL;", ident(p->next->sym->name), c_type(p->next->info.typ), ident(p->next->sym->name), c_type((Tnode*)p->next->info.typ->ref), ident(p->next->sym->name));
                if (((Tnode*)p->next->info.typ->ref)->type == Tclass && !is_external((Tnode*)p->next->info.typ->ref) && !is_volatile((Tnode*)p->next->info.typ->ref) && !is_typedef((Tnode*)p->next->info.typ->ref))
                  fprintf(fout, "\n\t\t\t\t\ta->%s->soap_default(soap);", ident(p->next->sym->name));
                else if (((Tnode*)p->next->info.typ->ref)->type != Tpointer  && !is_XML((Tnode*)p->next->info.typ->ref))
                  fprintf(fout, "\n\t\t\t\t\tsoap_default_%s(soap, a->%s);", c_ident((Tnode*)p->next->info.typ->ref), ident(p->next->sym->name));
                else
                  fprintf(fout, "\n\t\t\t\t\t*a->%s = NULL;", ident(p->next->sym->name));
                fprintf(fout, "\n\t\t\t\t}");
                if (!is_invisible(p->next->sym->name))
                  fprintf(fout, "\n\t\t\t\tsoap_revert(soap);");
                if (is_unmatched(p->next->sym))
                {
                  if (is_XML((Tnode*)p->next->info.typ->ref) && is_string((Tnode*)p->next->info.typ->ref))
                    fprintf(fout, "\n\t\t\t\tif (soap_inliteral(soap, NULL, (char**)a->%s))", ident(p->next->sym->name));
                  else if (is_XML((Tnode*)p->next->info.typ->ref) && is_wstring((Tnode*)p->next->info.typ->ref))
                    fprintf(fout, "\n\t\t\t\tif (soap_inwliteral(soap, NULL, (wchar_t**)a->%s))", ident(p->next->sym->name));
                  else if (is_string((Tnode*)p->next->info.typ->ref))
                    fprintf(fout, "\n\t\t\t\tif (soap_in_%s(soap, NULL, (char**)a->%s, \"%s\"))", c_ident((Tnode*)p->next->info.typ->ref), ident(p->next->sym->name), xsi_type((Tnode*)p->next->info.typ->ref));
                  else if (is_wstring((Tnode*)p->next->info.typ->ref))
                    fprintf(fout, "\n\t\t\t\tif (soap_in_%s(soap, NULL, (wchar_t**)a->%s, \"%s\"))", c_ident((Tnode*)p->next->info.typ->ref), ident(p->next->sym->name), xsi_type((Tnode*)p->next->info.typ->ref));
                  else
                    fprintf(fout, "\n\t\t\t\tif (soap_in_%s(soap, NULL, a->%s, \"%s\"))", c_ident((Tnode*)p->next->info.typ->ref), ident(p->next->sym->name), xsi_type((Tnode*)p->next->info.typ->ref));
                }
                else
                {
                  if (is_XML((Tnode*)p->next->info.typ->ref) && is_string((Tnode*)p->next->info.typ->ref))
                    fprintf(fout, "\n\t\t\t\tif (soap_inliteral(soap, %s, (char**)a->%s))", field(p->next, nse), ident(p->next->sym->name));
                  else if (is_XML((Tnode*)p->next->info.typ->ref) && is_wstring((Tnode*)p->next->info.typ->ref))
                    fprintf(fout, "\n\t\t\t\tif (soap_inwliteral(soap, %s, (wchar_t**)a->%s))", field(p->next, nse), ident(p->next->sym->name));
                  else if (is_string((Tnode*)p->next->info.typ->ref))
                    fprintf(fout, "\n\t\t\t\tif (soap_in_%s(soap, %s, (char**)a->%s, \"%s\"))", c_ident((Tnode*)p->next->info.typ->ref), field(p->next, nse), ident(p->next->sym->name), xsi_type((Tnode*)p->next->info.typ->ref));
                  else if (is_wstring((Tnode*)p->next->info.typ->ref))
                    fprintf(fout, "\n\t\t\t\tif (soap_in_%s(soap, %s, (wchar_t**)a->%s, \"%s\"))", c_ident((Tnode*)p->next->info.typ->ref), field(p->next, nse), ident(p->next->sym->name), xsi_type((Tnode*)p->next->info.typ->ref));
                  else
                    fprintf(fout, "\n\t\t\t\tif (soap_in_%s(soap, %s, a->%s, \"%s\"))", c_ident((Tnode*)p->next->info.typ->ref), field(p->next, nse), ident(p->next->sym->name), xsi_type((Tnode*)p->next->info.typ->ref));
                }
                fprintf(fout, "\n\t\t\t\t{\ta->%s++;\n\t\t\t\t\ta->%s = NULL;\n\t\t\t\t\tcontinue;\n\t\t\t\t}\n\t\t\t}", ident(p->sym->name), ident(p->next->sym->name));
                p = p->next;
              }
              else if (is_repetition(p) || is_anytype(p) || is_choice(p))
              {
                p = p->next;
                continue;
              }
              else if (is_invisible(p->sym->name)
                  && !(p->info.sto & (Sconst | Sprivate | Sprotected))
                  && !is_transient(p->info.typ)
                  && !(p->info.sto & Sattribute))
              {
                flag = 1;
                if (is_string(p->info.typ) || is_wstring(p->info.typ) || is_stdstr(p->info.typ))
                  fprintf(fout, "\n\t\t\tif (soap_flag_%s && (soap->error == SOAP_TAG_MISMATCH || soap->error == SOAP_NO_TAG))", ident(p->sym->name));
                else if (is_container(p->info.typ))
                  fprintf(fout, "\n\t\t\tif (soap->error == SOAP_TAG_MISMATCH)");
                else
                  fprintf(fout, "\n\t\t\tif (soap_flag_%s && soap->error == SOAP_TAG_MISMATCH)", ident(p->sym->name));
                fprintf(fout, "\n\t\t\t{\t");
                if (is_XML(p->info.typ) && is_string(p->info.typ))
                  fprintf(fout, "if (soap_inliteral(soap, %s, (char**)&a->%s))", field(p, nse), ident(p->sym->name));
                else if (is_XML(p->info.typ) && is_wstring(p->info.typ))
                  fprintf(fout, "if (soap_inwliteral(soap, %s, (wchar_t**)&a->%s))", field(p, nse), ident(p->sym->name));
                else if (is_string(p->info.typ))
                  fprintf(fout, "if (soap_in_%s(soap, %s, (char**)&a->%s, \"%s\"))", c_ident(p->info.typ), field(p, nse), ident(p->sym->name), xsi_type(p->info.typ));
                else if (is_wstring(p->info.typ))
                  fprintf(fout, "if (soap_in_%s(soap, %s, (wchar_t**)&a->%s, \"%s\"))", c_ident(p->info.typ), field(p, nse), ident(p->sym->name), xsi_type(p->info.typ));
                else if (p->info.typ->type == Tarray)
                  fprintf(fout, "if (soap_in_%s(soap, %s, a->%s, \"%s\"))", c_ident(p->info.typ), field(p, nse), ident(p->sym->name), xsi_type(p->info.typ));
                else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
                  fprintf(fout, "if (a->%s.soap_in(soap, %s, \"%s\"))", ident(p->sym->name), field(p, nse), xsi_type(p->info.typ));
                else if (p->info.typ->type != Tfun && !is_void(p->info.typ))
                  fprintf(fout, "if (soap_in_%s(soap, %s, &a->%s, \"%s\"))", c_ident(p->info.typ), field(p, nse), ident(p->sym->name), xsi_type(p->info.typ));
                if (is_container(p->info.typ))
                  fprintf(fout, "\n\t\t\t\t\tcontinue;");
                else
                {
                  fprintf(fout, "\n\t\t\t\t{\tsoap_flag_%s--;", ident(p->sym->name));
                  fprintf(fout, "\n\t\t\t\t\tcontinue;");
                  fprintf(fout, "\n\t\t\t\t}");
                }
                fprintf(fout, "\n\t\t\t}");
              }
            }
          }
          for (t = table; t; t = t->prev)
            for (p = t->list; p; p = p->next)
              if (p->info.sto & Sreturn)
                if (nse || has_ns_eq(NULL, p->sym->name))
                  fprintf(fout, "\n\t\t\tsoap_check_result(soap, \"%s\");", ns_add(p, nse));
          if (!flag && is_invisible(typ->id->name))
            fprintf(fout, "\n\tsoap->error = SOAP_TAG_MISMATCH;\n\ta = NULL;");
          if (!is_invisible(typ->id->name) || (table && (table->prev || table->list)))
          {
            if (!is_invisible(typ->id->name) || is_discriminant(typ))
              fprintf(fout, "\n\t\t\tif (soap->error == SOAP_TAG_MISMATCH)\n\t\t\t\tsoap->error = soap_ignore_element(soap);");
            else
              fprintf(fout, "\n\t\t\tif (soap->error == SOAP_TAG_MISMATCH && soap_flag)\n\t\t\t{\tsoap->error = SOAP_OK;\n\t\t\t\tbreak;\n\t\t\t}");
            if (!is_invisible(typ->id->name))
              fprintf(fout, "\n\t\t\tif (soap->error == SOAP_NO_TAG)");
            else
              fprintf(fout, "\n\t\t\tif (soap_flag && soap->error == SOAP_NO_TAG)");
            fprintf(fout, "\n\t\t\t\tbreak;");
            fprintf(fout, "\n\t\t\tif (soap->error)\n\t\t\t\treturn NULL;");
            fprintf(fout, "\n\t\t}");
          }
        }
        if (table && !is_discriminant(typ))
        {
          for (p = table->list; p; p = p->next)
          {
            if (is_repetition(p))
            {
              if (((Tnode*)p->next->info.typ->ref)->type == Tclass
               || ((Tnode*)p->next->info.typ->ref)->type == Ttemplate
               || has_class((Tnode*)p->next->info.typ->ref)
               || (!cflag && ((Tnode*)p->next->info.typ->ref)->type == Tstruct))
              {
                fprintf(fout, "\n\t\tif (a->%s)\n\t\t\tsoap_block<%s>::pop(soap, soap_blist_%s);", ident(p->next->sym->name), c_type(p->next->info.typ->ref), ident(p->next->sym->name));
                fprintf(fout, "\n\t\tif (a->%s)\n\t\t{\ta->%s = soap_new_%s(soap, a->%s);\n\t\t\tif (!a->%s)\n\t\t\t\treturn NULL;\n\t\t\tsoap_block<%s>::save(soap, soap_blist_%s, a->%s);\n\t\t}\n\t\telse\n\t\t{\ta->%s = NULL;\n\t\t\tif (soap_blist_%s)\n\t\t\t\tsoap_block<%s>::end(soap, soap_blist_%s);\n\t\t}", ident(p->sym->name), ident(p->next->sym->name), c_ident(p->next->info.typ->ref), ident(p->sym->name), ident(p->next->sym->name), c_type(p->next->info.typ->ref), ident(p->next->sym->name), ident(p->next->sym->name), ident(p->next->sym->name), ident(p->next->sym->name), c_type(p->next->info.typ->ref), ident(p->next->sym->name));
              }
              else
              {
                fprintf(fout, "\n\t\tif (a->%s)\n\t\t\tsoap_pop_block(soap, soap_blist_%s);", ident(p->next->sym->name), ident(p->next->sym->name));
                if ((((Tnode*)p->next->info.typ->ref)->type == Tpointer && (p->next->info.sto & Sconstptr)))
                  fprintf(fout, "\n\t\tif (a->%s)\n\t\t{\ta->%s = (const %s)soap_save_block(soap, soap_blist_%s, NULL, 1);\n\t\t}\n\t\telse\n\t\t{\ta->%s = NULL;\n\t\t\tif (soap_blist_%s)\n\t\t\t\tsoap_end_block(soap, soap_blist_%s);\n\t\t}", ident(p->sym->name), ident(p->next->sym->name), c_type(p->next->info.typ), ident(p->next->sym->name), ident(p->next->sym->name), ident(p->next->sym->name), ident(p->next->sym->name));
                else
                  fprintf(fout, "\n\t\tif (a->%s)\n\t\t{\ta->%s = (%s)soap_save_block(soap, soap_blist_%s, NULL, 1);\n\t\t}\n\t\telse\n\t\t{\ta->%s = NULL;\n\t\t\tif (soap_blist_%s)\n\t\t\t\tsoap_end_block(soap, soap_blist_%s);\n\t\t}", ident(p->sym->name), ident(p->next->sym->name), c_type(p->next->info.typ), ident(p->next->sym->name), ident(p->next->sym->name), ident(p->next->sym->name), ident(p->next->sym->name));
              }
              p = p->next;
            }
          }
        }
        if (!is_invisible(typ->id->name))
        {
          if (is_discriminant(typ))
            fprintf(fout, "\n\t\tif (tag && *tag != '-')\n\t\t{\tsoap->error = soap_ignore_element(soap);\n\t\t\tif (soap->error == SOAP_TAG_MISMATCH || soap_element_end_in(soap, tag))\n\t\t\t\treturn NULL;\n\t\t}");
          else
            fprintf(fout, "\n\t\tif (soap_element_end_in(soap, tag))\n\t\t\treturn NULL;");
        }
        strict = 0;
        nonempty = 0;
        if (table && !is_discriminant(typ))
        {
          for (p = table->list; p; p = p->next)
          {
            if (p->info.minOccurs > 0 && p->info.maxOccurs >= 0 && !(p->info.sto & (Sconst | Sprivate | Sprotected)) && !(p->info.sto & Sattribute) && p->info.typ->type != Tfun && !is_void(p->info.typ) && !is_transient(p->info.typ) && !is_container(p->info.typ) && !is_repetition(p) && !is_choice(p))
            {
              if (is_item(p))
                continue;
              if (is_anytype(p))
              {
                p = p->next;
                if (p->info.minOccurs <= 0)
                  continue;
              }
              if ((p->info.typ->type != Tpointer && !is_smart(p->info.typ)) || p->info.nillable)
              {
                if (strict == 0)
                {
                  fprintf(fout, "\n\t\tif (%s(soap_flag_%s > " SOAP_LONG_FORMAT, strict_check(), ident(p->sym->name), p->info.maxOccurs - p->info.minOccurs);
                  strict = 1;
                }
                else
                  fprintf(fout, " || soap_flag_%s > " SOAP_LONG_FORMAT, ident(p->sym->name), p->info.maxOccurs - p->info.minOccurs);
              }
              else if (strict == 0)
              {
                if (is_smart(p->info.typ))
                  fprintf(fout, "\n\t\tif (%s(!a->%s.get()", strict_check(), ident(p->sym->name));
                else
                  fprintf(fout, "\n\t\tif (%s(!a->%s", strict_check(), ident(p->sym->name));
                strict = 1;
              }
              else
              {
                if (is_smart(p->info.typ))
                  fprintf(fout, " || !a->%s.get()", ident(p->sym->name));
                else
                  fprintf(fout, " || !a->%s", ident(p->sym->name));
              }
              nonempty = 1;
            }
            else if (is_container(p->info.typ))
            {
              if (p->info.minOccurs > 0)
              {
                if (p->info.typ->type == Tpointer)
                {
                  if (strict == 0)
                  {
                    fprintf(fout, "\n\t\tif (%s(!a->%s || a->%s->size() < " SOAP_LONG_FORMAT, strict_check(), ident(p->sym->name), ident(p->sym->name), p->info.minOccurs);
                    strict = 1;
                  }
                  else
                    fprintf(fout, " || !a->%s || a->%s->size() < " SOAP_LONG_FORMAT, ident(p->sym->name), ident(p->sym->name), p->info.minOccurs);
                }
                else
                {
                  if (strict == 0)
                  {
                    fprintf(fout, "\n\t\tif (%s(a->%s.size() < " SOAP_LONG_FORMAT, strict_check(), ident(p->sym->name), p->info.minOccurs);
                    strict = 1;
                  }
                  else
                    fprintf(fout, " || a->%s.size() < " SOAP_LONG_FORMAT, ident(p->sym->name), p->info.minOccurs);
                }
                nonempty = 1;
              }
              if (p->info.maxOccurs > 1)
              {
                if (p->info.typ->type == Tpointer)
                {
                  if (strict == 0)
                  {
                    fprintf(fout, "\n\t\tif (%s((a->%s && a->%s->size() > " SOAP_LONG_FORMAT ")", strict_check(), ident(p->sym->name), ident(p->sym->name), p->info.maxOccurs);
                    strict = 1;
                  }
                  else
                    fprintf(fout, " || (a->%s && a->%s->size() > " SOAP_LONG_FORMAT ")", ident(p->sym->name), ident(p->sym->name), p->info.maxOccurs);
                }
                else
                {
                  if (strict == 0)
                  {
                    fprintf(fout, "\n\t\tif (%s(a->%s.size() > " SOAP_LONG_FORMAT, strict_check(), ident(p->sym->name), p->info.maxOccurs);
                    strict = 1;
                  }
                  else
                    fprintf(fout, " || a->%s.size() > " SOAP_LONG_FORMAT, ident(p->sym->name), p->info.maxOccurs);
                }
              }
            }
            else if (is_repetition(p))
            {
              if (p->info.minOccurs > 0)
              {
                if (strict == 0)
                {
                  fprintf(fout, "\n\t\tif (%s(a->%s < " SOAP_LONG_FORMAT, strict_check(), ident(p->sym->name), p->info.minOccurs);
                  strict = 1;
                }
                else
                  fprintf(fout, " || a->%s < " SOAP_LONG_FORMAT, ident(p->sym->name), p->info.minOccurs);
                nonempty = 1;
              }
              if (p->info.maxOccurs > 1)
              {
                if (strict == 0)
                {
                  fprintf(fout, "\n\t\tif (%s(a->%s > " SOAP_LONG_FORMAT, strict_check(), ident(p->sym->name), p->info.maxOccurs);
                  strict = 1;
                }
                else
                  fprintf(fout, " || a->%s > " SOAP_LONG_FORMAT, ident(p->sym->name), p->info.maxOccurs);
              }
              p = p->next;
            }
            else if (is_choice(p))
            {
              if (p->info.minOccurs != 0 && required_choice(p->next->info.typ) < 0)
              {
                if (strict == 0)
                {
                  fprintf(fout, "\n\t\tif (%s(soap_flag_%s", strict_check(), ident(p->next->sym->name));
                  strict = 1;
                }
                else
                  fprintf(fout, " || soap_flag_%s", ident(p->next->sym->name));
                nonempty = 1;
              }
              p = p->next;
            }
          }
          if (strict)
            fprintf(fout, "))\n\t\t{\tsoap->error = SOAP_OCCURS;\n\t\t\treturn NULL;\n\t\t}");
        }
        if (has_getter(typ))
          fprintf(fout, "\n\t\tif (a->get(soap))\n\t\t\treturn NULL;");
        if (!is_invisible(typ->id->name))
        {
          if (nonempty)
            fprintf(fout, "\n\t}\n\telse if (%s*soap->href != '#')\n\t{\tsoap->error = SOAP_OCCURS;\n\t\treturn NULL;", strict_check());
          fprintf(fout, "\n\t}\n\telse\n\t{\t");
          if (!cflag)
            fprintf(fout, "a = (%s)soap_id_forward(soap, soap->href, (void*)a, 0, %s, %s, sizeof(%s), 0, %s_finsert, NULL);", c_type_id(typ, "*"), soap_type(typ), soap_type(typ), c_type(typ), prefix);
          else
            fprintf(fout, "a = (%s)soap_id_forward(soap, soap->href, (void*)a, 0, %s, %s, sizeof(%s), 0, NULL, NULL);", c_type_id(typ, "*"), soap_type(typ), soap_type(typ), c_type(typ));
          fprintf(fout, "\n\t\tif (soap->body && soap_element_end_in(soap, tag))\n\t\t\treturn NULL;\n\t}");
        }
        fprintf(fout, "\n\treturn a;\n}");
      }
      break;
    case Tclass:
      if (is_external(typ) && !is_volatile(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 %s SOAP_FMAC2 soap_in_%s(struct soap*, const char*, %s, const char*);", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 %s SOAP_FMAC4 soap_in_%s(struct soap*, const char*, %s, const char*);", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*"));
      if (!is_volatile(typ) && !is_typedef(typ))
      {
        fprintf(fout, "\n\nvoid *%s::soap_in(struct soap *soap, const char *tag, const char *type)", c_type(typ));
        fprintf(fout, "\n{\n\treturn soap_in_%s(soap, tag, this, type);\n}", c_ident(typ));
        fflush(fout);
      }
      fprintf(fout, "\n\nSOAP_FMAC3 %s SOAP_FMAC4 soap_in_%s(struct soap *soap, const char *tag, %s, const char *type)\n{", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*a"));
      table = (Table*)typ->ref;
      if (!table)
      {
        fprintf(fout, "\n\t(void)soap; (void)tag; (void)a; (void)type; /* appease -Wall -Werror */");
        fprintf(fout, "\n\tif (!a)\n\t\tsoap->error = SOAP_TAG_MISMATCH;\n\treturn a;\n}");
      }
      else if (is_primclass(typ))
      {
        fprintf(fout, "\n\t(void)type; /* appease -Wall -Werror */");
        for (p = table->list; p; p = p->next)
        {
          if (is_pointer_to_derived(p))
          {
            der = 1;
            break;
          }
        }
        if (der)
          fprintf(fout, "\n\tint err = soap_element_begin_in(soap, tag, 1, type);\n\tif (err && err != SOAP_TYPE)\n\t\treturn NULL;");
        else
          fprintf(fout, "\n\tif (soap_element_begin_in(soap, tag, 1, NULL))\n\t\treturn NULL;");
        fprintf(fout, "\n\tif (!(a = (%s*)soap_id_enter(soap, soap->id, a, %s, sizeof(%s), soap->type, soap->arrayType, %s_instantiate, %s_fbase)))\n\t{\tsoap->error = SOAP_TAG_MISMATCH;\n\t\treturn NULL;\n\t}", c_type(typ), soap_type(typ), c_type(typ), prefix, prefix);
        fprintf(fout, "\n\tsoap_revert(soap);\n\t*soap->id = '\\0';");
        fprintf(fout, "\n\tif (soap->alloced && soap->alloced != %s)", soap_type(typ));
        fprintf(fout, "\n\t\treturn (%s)a->soap_in(soap, tag, type);", c_type_id(typ, "*"));
        fprintf(fout, "\n\tif (soap->alloced)\n\t\ta->soap_default(soap);");
        if (der)
        {
          fprintf(fout, "\n\tif (err == SOAP_TYPE)\n\t{");
          for (p = table->list; p; p = p->next)
          {
            if (is_pointer_to_derived(p))
            {
              fprintf(fout, "\n\t\tif ((");
              gen_match_derived(fout, p->info.typ->ref);
              fprintf(fout, ") && (a->%s = soap_in_%s(soap, tag, NULL, \"%s\")) != NULL)\n\t\t\treturn a;", ident(p->sym->name), c_ident(p->info.typ->ref), xsi_type(p->info.typ->ref));
            }
          }
          fprintf(fout, "\n\t\treturn NULL;\n\t}");
        }
        for (t = table; t; t = t->prev)
        {
          Entry *e = entry(classtable, t->sym);
          const char *nsa1 = e ? ns_qualifiedAttribute(e->info.typ) : nsa;
          for (p = t->list; p; p = p->next)
            if (p->info.sto & Sattribute)
              soap_attr_value(p, ptr_cast(t, "a"), ident(p->sym->name), ns_add(p, nsa1));
        }
        fflush(fout);
        for (table = (Table*)typ->ref; table; table = table->prev)
        {
          p = table->list;
          while (p && !is_item(p))
            p = p->next;
          if (p)
            break;
        }
        if (is_XML(p->info.typ) && is_string(p->info.typ))
          fprintf(fout, "\n\tif (!soap_inliteral(soap, tag, (char**)&a->%s::%s))", ident(table->sym->name), ident(p->sym->name));
        else if (is_XML(p->info.typ) && is_wstring(p->info.typ))
          fprintf(fout, "\n\tif (!soap_inwliteral(soap, tag, (wchar_t**)&a->%s::%s))", ident(table->sym->name), ident(p->sym->name));
        else if (is_string(p->info.typ))
          fprintf(fout, "\n\tif (!soap_in_%s(soap, tag, (char**)&a->%s::%s, \"%s\"))", c_ident(p->info.typ), ident(table->sym->name), ident(p->sym->name), xsi_type(typ));
        else if (is_wstring(p->info.typ))
          fprintf(fout, "\n\tif (!soap_in_%s(soap, tag, (wchar_t**)&a->%s::%s, \"%s\"))", c_ident(p->info.typ), ident(table->sym->name), ident(p->sym->name), xsi_type(typ));
        else if (p->info.typ->type == Tarray)
          fprintf(fout, "\n\tif (!soap_in_%s(soap, tag, a->%s::%s, \"%s\"))", c_ident(p->info.typ), ident(table->sym->name), ident(p->sym->name), xsi_type(typ));
        else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
          fprintf(fout, "\n\tif (!(a->%s::%s).soap_in(soap, tag, \"%s\"))", ident(table->sym->name), ident(p->sym->name), xsi_type(typ));
        else if (p->info.typ->type != Tfun && !is_void(p->info.typ))
          fprintf(fout, "\n\tif (!soap_in_%s(soap, tag, &a->%s::%s, \"%s\"))", c_ident(p->info.typ), ident(table->sym->name), ident(p->sym->name), xsi_type(typ));
        fprintf(fout, "\n\t\treturn NULL;");
        fixed_check(fout, p, t, "\t");
        if (has_getter(typ))
          fprintf(fout, "\n\tif (a->get(soap))\n\t\treturn NULL;");
        fprintf(fout, "\n\treturn a;\n}");
      }
      else
      {
        if (!is_invisible(typ->id->name))
        {
          fprintf(fout, "\n\t(void)type; /* appease -Wall -Werror */");
          for (p = table->list; p; p = p->next)
          {
            if (is_pointer_to_derived(p))
            {
              der = 1;
              break;
            }
          }
          if (der)
            fprintf(fout, "\n\tint err = soap_element_begin_in(soap, tag, 0, type);\n\tif (err && err != SOAP_TYPE)\n\t\treturn NULL;");
          else
            fprintf(fout, "\n\tif (soap_element_begin_in(soap, tag, 0, NULL))\n\t\treturn NULL;");
          fprintf(fout, "\n\ta = (%s*)soap_id_enter(soap, soap->id, a, %s, sizeof(%s), soap->type, soap->arrayType, %s_instantiate, %s_fbase);", c_type(typ), soap_type(typ), c_type(typ), prefix, prefix);
        }
        else
        {
          fprintf(fout, "\n\t(void)tag; (void)type; /* appease -Wall -Werror */");
          fprintf(fout, "\n\ta = (%s*)soap_id_enter(soap, \"\", a, %s, sizeof(%s), soap->type, soap->arrayType, %s_instantiate, %s_fbase);", c_type(typ), soap_type(typ), c_type(typ), prefix, prefix);
        }
        fprintf(fout, "\n\tif (!a)\n\t\treturn NULL;");
        table = (Table*)typ->ref;
        if (!is_discriminant(typ))
        {
          if (!is_invisible(typ->id->name))
          {
            fprintf(fout, "\n\tif (soap->alloced && soap->alloced != %s)", soap_type(typ));
            fprintf(fout, "\n\t{\tsoap_revert(soap);");
            fprintf(fout, "\n\t\t*soap->id = '\\0';");
            if (is_volatile(typ) || is_typedef(typ))
              fprintf(fout, "\n\t\treturn soap_in_%s(soap, tag, a, type);", c_ident(typ));
            else
              fprintf(fout, "\n\t\treturn (%s)a->soap_in(soap, tag, type);", c_type_id(typ, "*"));
            fprintf(fout, "\n\t}");
          }
        }
        fprintf(fout, "\n\tif (soap->alloced)");
        if (is_volatile(typ) || is_typedef(typ))
          fprintf(fout, "\n\t\tsoap_default_%s(soap, a);", c_ident(typ));
        else
          fprintf(fout, "\n\t\ta->soap_default(soap);");
        if (zflag == 0 || zflag > 4)
        {
          for (t = table; t; t = t->prev)
          {
            for (p = t->list; p; p = p->next)
            {
              if (is_choice(p) || is_repetition(p))
              {
                p = p->next;
                continue;
              }
              if (!p->info.hasval || p->info.minOccurs > 0 || p->info.typ->type == Tfun || p->info.typ->type == Tunion || is_soapref(p->info.typ) || (p->info.sto & Sconst) || (p->info.sto & (Sprivate | Sprotected)) || (p->info.sto & Sattribute) || is_anytype(p) || is_transient(p->info.typ))
                continue;
              if (is_fixedstring(p->info.typ))
                fprintf(fout, "\n\ta->%s::%s[0] = '\\0';", ident(t->sym->name), ident(p->sym->name));
              else if (is_string(p->info.typ) || is_wstring(p->info.typ))
                fprintf(fout, "\n\ta->%s::%s = NULL;", ident(t->sym->name), ident(p->sym->name));
            }
          }
        }
        if (der)
        {
          fprintf(fout, "\n\tif (err == SOAP_TYPE)\n\t{");
          for (p = table->list; p; p = p->next)
          {
            if (is_pointer_to_derived(p))
            {
              fprintf(fout, "\n\t\tif ((");
              gen_match_derived(fout, p->info.typ->ref);
              fprintf(fout, ") && (a->%s = soap_in_%s(soap, tag, NULL, \"%s\")) != NULL)\n\t\t\treturn a;", ident(p->sym->name), c_ident(p->info.typ->ref), xsi_type(p->info.typ->ref));
            }
          }
          fprintf(fout, "\n\t\treturn NULL;\n\t}");
        }
        for (t = table; t; t = t->prev)
        {
          Entry *e = entry(classtable, t->sym);
          const char *nsa1 = e ? ns_qualifiedAttribute(e->info.typ) : nsa;
          for (p = t->list; p; p = p->next)
            if (p->info.sto & Sattribute)
              soap_attr_value(p, ptr_cast(t, "a"), ident(p->sym->name), ns_add(p, nsa1));
        }
        fflush(fout);
        i = 0;
        if (!is_discriminant(typ))
        {
          for (t = table; t; t = t->prev)
            i++;
          for (; i > 0; i--)
          {
            t = table;
            for (j = 0; j < i-1; j++)
              t = t->prev;
            if (!t->prev)
            {
              Entry *e = entry(classtable, t->sym);
              if (e && e->info.typ && e->info.typ->baseid && !strcmp(e->info.typ->baseid->name, "soap_dom_element"))
                fprintf(fout, "\n\tsize_t soap_flag_soap_dom_element = 1;");
            }
            for (p = t->list; p; p = p->next)
            {
              if (!(p->info.sto & (Sconst | Sprivate | Sprotected)) && !(p->info.sto & Sattribute) && p->info.typ->type != Tfun && !is_void(p->info.typ))
              {
                if (is_anytype(p) || is_choice(p))
                {
                  p = p->next;
                  fprintf(fout, "\n\tsize_t soap_flag_%s%d = " SOAP_LONG_FORMAT ";", ident(p->sym->name), i, p->info.maxOccurs);
                }
                else if (is_repetition(p))
                {
                  fprintf(fout, "\n\tstruct soap_blist *soap_blist_%s%d = NULL;", ident(p->next->sym->name), i);
                  p = p->next;
                }
                else if (!is_transient(p->info.typ) && !is_container(p->info.typ))
                  fprintf(fout, "\n\tsize_t soap_flag_%s%d = " SOAP_LONG_FORMAT ";", ident(p->sym->name), i, p->info.maxOccurs);
              }
            }
          }
        }
        fflush(fout);
        if (!is_invisible(typ->id->name))
        {
          if (!is_discriminant(typ))
          {
            fprintf(fout, "\n\tif (soap->body && *soap->href != '#')\n\t{");
            fprintf(fout, "\n\t\tfor (;;)\n\t\t{\tsoap->error = SOAP_TAG_MISMATCH;");
          }
          else
            fprintf(fout, "\n\tif (!tag || *tag == '-' || (soap->body && *soap->href != '#'))\n\t{");
        }
        else if (!is_discriminant(typ))
        {
          if (table && (table->prev || table->list))
            fprintf(fout, "\n\t\tfor (short soap_flag = 0;; soap_flag = 1)\n\t\t{\tsoap->error = SOAP_TAG_MISMATCH;");
        }
        table = (Table*)typ->ref;
        i = 0;
        flag = 0;
        for (t = table; t; t = t->prev)
          i++;
        for (; i > 0; i--)
        {
          Entry *e;
          const char *nse1;
          t = table;
          for (j = 0; j < i-1; j++)
            t = t->prev;
          e = entry(classtable, t->sym);
          nse1 = e ? ns_qualifiedElement(e->info.typ) : nse;
          for (p = t->list; p; p = p->next)
          {
            if (p->info.typ->type == Tfun || p->info.typ->type == Tunion || is_soapref(p->info.typ))
              ;
            else if (is_item(p))
              ;
            else if (p->info.sto & (Sconst | Sprivate | Sprotected))
              fprintf(fout, "\n\t\t\t/* non-serializable %s skipped */", ident(p->sym->name));
            else if (p->info.sto & Sattribute)
              ;
            else if (is_repetition(p))
            {
              if (is_unmatched(p->next->sym) || is_invisible(p->next->sym->name))
              {
                p = p->next;
                continue;
              }
              flag = 1;
              fprintf(fout, "\n\t\t\tif (soap->error == SOAP_TAG_MISMATCH && ");
              fprintf(fout, "!soap_element_begin_in(soap, %s, 1, NULL))", field(p->next, nse1));
              fprintf(fout, "\n\t\t\t{\tif (a->%s::%s == NULL)\n\t\t\t\t{\tif (soap_blist_%s%d == NULL)\n\t\t\t\t\t\tsoap_blist_%s%d = soap_alloc_block(soap);", ident(t->sym->name), ident(p->next->sym->name), ident(p->next->sym->name), i, ident(p->next->sym->name), i);
              if (((Tnode*)p->next->info.typ->ref)->type == Tclass
               || ((Tnode*)p->next->info.typ->ref)->type == Ttemplate
               || has_class((Tnode*)p->next->info.typ->ref)
               || (!cflag && ((Tnode*)p->next->info.typ->ref)->type == Tstruct))
                fprintf(fout, "\n\t\t\t\t\ta->%s::%s = soap_block<%s>::push(soap, soap_blist_%s%d);\n\t\t\t\t\tif (a->%s::%s == NULL)\n\t\t\t\t\t\treturn NULL;", ident(t->sym->name), ident(p->next->sym->name), c_type(p->next->info.typ->ref), ident(p->next->sym->name), i, ident(t->sym->name), ident(p->next->sym->name));
              else if (((Tnode*)p->next->info.typ->ref)->type == Tpointer && (p->next->info.sto & Sconstptr))
                fprintf(fout, "\n\t\t\t\t\ta->%s::%s = (const %s)soap_push_block_max(soap, soap_blist_%s%d, sizeof(%s));\n\t\t\t\t\tif (a->%s::%s == NULL)\n\t\t\t\t\t\treturn NULL;", ident(t->sym->name), ident(p->next->sym->name), c_type(p->next->info.typ), ident(p->next->sym->name), i, c_type((Tnode*)p->next->info.typ->ref), ident(t->sym->name), ident(p->next->sym->name));
              else
                fprintf(fout, "\n\t\t\t\t\ta->%s::%s = (%s)soap_push_block_max(soap, soap_blist_%s%d, sizeof(%s));\n\t\t\t\t\tif (a->%s::%s == NULL)\n\t\t\t\t\t\treturn NULL;", ident(t->sym->name), ident(p->next->sym->name), c_type(p->next->info.typ), ident(p->next->sym->name), i, c_type((Tnode*)p->next->info.typ->ref), ident(t->sym->name), ident(p->next->sym->name));
              if (((Tnode*)p->next->info.typ->ref)->type == Tclass && !is_external((Tnode*)p->next->info.typ->ref) && !is_volatile((Tnode*)p->next->info.typ->ref) && !is_typedef((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\t\t\ta->%s::%s->soap_default(soap);", ident(t->sym->name), ident(p->next->sym->name));
              else if (((Tnode*)p->next->info.typ->ref)->type != Tpointer  && !is_XML((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\t\t\tsoap_default_%s(soap, a->%s::%s);", c_ident((Tnode*)p->next->info.typ->ref), ident(t->sym->name), ident(p->next->sym->name));
              else
                fprintf(fout, "\n\t\t\t\t\t*a->%s::%s = NULL;", ident(t->sym->name), ident(p->next->sym->name));
              fprintf(fout, "\n\t\t\t\t}");
              fprintf(fout, "\n\t\t\t\tsoap_revert(soap);");
              if (is_XML((Tnode*)p->next->info.typ->ref) && is_string((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\t\tif (soap_inliteral(soap, %s, (char**)a->%s::%s))", field(p->next, nse1), ident(t->sym->name), ident(p->next->sym->name));
              else if (is_XML((Tnode*)p->next->info.typ->ref) && is_wstring((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\t\tif (soap_inwliteral(soap, %s, (wchar_t**)a->%s::%s))", field(p->next, nse1), ident(t->sym->name), ident(p->next->sym->name));
              else if (is_string((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\t\tif (soap_in_%s(soap, %s, (char**)a->%s::%s, \"%s\"))", c_ident((Tnode*)p->next->info.typ->ref), field(p->next, nse1), ident(t->sym->name), ident(p->next->sym->name), xsi_type((Tnode*)p->next->info.typ->ref));
              else if (is_wstring((Tnode*)p->next->info.typ->ref))
                fprintf(fout, "\n\t\t\t\tif (soap_in_%s(soap, %s, (wchar_t**)a->%s::%s, \"%s\"))", c_ident((Tnode*)p->next->info.typ->ref), field(p->next, nse1), ident(t->sym->name), ident(p->next->sym->name), xsi_type((Tnode*)p->next->info.typ->ref));
              else
                fprintf(fout, "\n\t\t\t\tif (soap_in_%s(soap, %s, a->%s::%s, \"%s\"))", c_ident((Tnode*)p->next->info.typ->ref), field(p->next, nse1), ident(t->sym->name), ident(p->next->sym->name), xsi_type((Tnode*)p->next->info.typ->ref));
              fprintf(fout, "\n\t\t\t\t{\ta->%s::%s++;\n\t\t\t\t\ta->%s::%s = NULL;\n\t\t\t\t\tcontinue;\n\t\t\t\t}\n\t\t\t}", ident(t->sym->name), ident(p->sym->name), ident(t->sym->name), ident(p->next->sym->name));
              p = p->next;
            }
            else if (is_anytype(p))
            {
              flag = 1;
              fprintf(fout, "\n\t\t\tif (soap_flag_%s%d && soap->error == SOAP_TAG_MISMATCH)", ident(p->next->sym->name), i);
              if (is_self(p->next))
                fprintf(fout, "\n\t\t\t{\tif ((a->%s::%s = soap_getelement(soap, tag, &a->%s::%s)))", ident(t->sym->name), ident(p->next->sym->name), ident(t->sym->name), ident(p->sym->name));
              else if (is_invisible(p->next->sym->name))
                fprintf(fout, "\n\t\t\t{\tif ((a->%s::%s = soap_getelement(soap, NULL, &a->%s::%s)))", ident(t->sym->name), ident(p->next->sym->name), ident(t->sym->name), ident(p->sym->name));
              else
                fprintf(fout, "\n\t\t\t{\tif ((a->%s::%s = soap_getelement(soap, %s, &a->%s::%s)))", ident(t->sym->name), ident(p->next->sym->name), field(p->next, nse1), ident(t->sym->name), ident(p->sym->name));
              fprintf(fout, "\n\t\t\t\t{\tsoap_flag_%s%d = 0;", ident(p->next->sym->name), i);
              fprintf(fout, "\n\t\t\t\t\tcontinue;");
              fprintf(fout, "\n\t\t\t\t}\n\t\t\t}");
              p = p->next;
            }
            else if (is_discriminant(typ) && p->next)
            {
              flag = 1;
              fprintf(fout, "\n\t\tif (!soap_in_%s(soap, &a->%s, &a->%s))", c_ident(p->next->info.typ), ident(p->sym->name), ident(p->next->sym->name));
              fprintf(fout, "\n\t\t\treturn NULL;");
              i = 0;
              break;
            }
            else if (is_choice(p))
            {
              flag = 1;
              fprintf(fout, "\n\t\t\tif (soap_flag_%s%d && soap->error == SOAP_TAG_MISMATCH)", ident(p->next->sym->name), i);
              fprintf(fout, "\n\t\t\t{\tif (soap_in_%s(soap, &a->%s::%s, &a->%s::%s))", c_ident(p->next->info.typ), ident(t->sym->name), ident(p->sym->name), ident(t->sym->name), ident(p->next->sym->name));
              fprintf(fout, "\n\t\t\t\t{\tsoap_flag_%s%d = 0;", ident(p->next->sym->name), i);
              fprintf(fout, "\n\t\t\t\t\tcontinue;");
              fprintf(fout, "\n\t\t\t\t}\n\t\t}");
              p = p->next;
            }
            else if (is_transient(p->info.typ))
            {
              if (!is_pointer_to_derived(p))
                fprintf(fout, "\n\t\t\t/* transient %s skipped */", ident(p->sym->name));
            }
            else
            {
              flag = 1;
              if (!is_invisible(p->sym->name) && !is_primclass(typ) && p->info.typ->type != Tfun && !is_void(p->info.typ))
              {
                if (is_string(p->info.typ) || is_wstring(p->info.typ) || is_stdstr(p->info.typ))
                  fprintf(fout, "\n\t\t\tif (soap_flag_%s%d && (soap->error == SOAP_TAG_MISMATCH || soap->error == SOAP_NO_TAG))", ident(p->sym->name), i);
                else if (is_container(p->info.typ))
                  fprintf(fout, "\n\t\t\tif (soap->error == SOAP_TAG_MISMATCH)");
                else
                  fprintf(fout, "\n\t\t\tif (soap_flag_%s%d && soap->error == SOAP_TAG_MISMATCH)", ident(p->sym->name), i);
                fprintf(fout, "\n\t\t\t{\t");
              }
              if (is_unmatched(p->sym))
              {
                if (is_XML(p->info.typ) && is_string(p->info.typ))
                {
                  fprintf(fout, "if (soap_inliteral(soap, NULL, (char**)&a->%s::%s))", ident(t->sym->name), ident(p->sym->name));
                }
                else if (is_XML(p->info.typ) && is_wstring(p->info.typ))
                {
                  fprintf(fout, "if (soap_inwliteral(soap, NULL, (wchar_t**)&a->%s::%s))", ident(t->sym->name), ident(p->sym->name));
                }
                else if (is_string(p->info.typ))
                {
                  fprintf(fout, "if (soap_in_%s(soap, NULL, (char**)&a->%s::%s, \"%s\"))", c_ident(p->info.typ), ident(t->sym->name), ident(p->sym->name), xsi_type(p->info.typ));
                }
                else if (is_wstring(p->info.typ))
                {
                  fprintf(fout, "if (soap_in_%s(soap, NULL, (wchar_t**)&a->%s::%s, \"%s\"))", c_ident(p->info.typ), ident(t->sym->name), ident(p->sym->name), xsi_type(p->info.typ));
                }
                else if (p->info.typ->type == Tarray)
                {
                  fprintf(fout, "if (soap_in_%s(soap, NULL, a->%s::%s, \"%s\"))", c_ident(p->info.typ), ident(t->sym->name), ident(p->sym->name), xsi_type(p->info.typ));
                }
                else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
                {
                  fprintf(fout, "if ((a->%s::%s).soap_in(soap, NULL, \"%s\"))", ident(t->sym->name), ident(p->sym->name), xsi_type(p->info.typ));
                }
                else if (p->info.typ->type != Tfun && !is_void(p->info.typ))
                {
                  fprintf(fout, "if (soap_in_%s(soap, NULL, &a->%s::%s, \"%s\"))", c_ident(p->info.typ), ident(t->sym->name), ident(p->sym->name), xsi_type(p->info.typ));
                }
              }
              else if (!is_invisible(p->sym->name))
              {
                if (is_XML(p->info.typ) && is_string(p->info.typ))
                {
                  fprintf(fout, "if (soap_inliteral(soap, %s, (char**)&a->%s::%s))", field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name));
                }
                else if (is_XML(p->info.typ) && is_wstring(p->info.typ))
                {
                  fprintf(fout, "if (soap_inwliteral(soap, %s, (wchar_t**)&a->%s::%s))", field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name));
                }
                else if (is_string(p->info.typ))
                {
                  fprintf(fout, "if (soap_in_%s(soap, %s, (char**)&a->%s::%s, \"%s\"))", c_ident(p->info.typ), field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name), xsi_type(p->info.typ));
                }
                else if (is_wstring(p->info.typ))
                {
                  fprintf(fout, "if (soap_in_%s(soap, %s, (wchar_t**)&a->%s::%s, \"%s\"))", c_ident(p->info.typ), field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name), xsi_type(p->info.typ));
                }
                else if (p->info.typ->type == Tarray)
                {
                  fprintf(fout, "if (soap_in_%s(soap, %s, a->%s::%s, \"%s\"))", c_ident(p->info.typ), field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name), xsi_type(p->info.typ));
                }
                else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
                {
                  fprintf(fout, "if ((a->%s::%s).soap_in(soap, %s, \"%s\"))", ident(t->sym->name), ident(p->sym->name), field_overridden(t, p, nse1), xsi_type(p->info.typ));
                }
                else if (p->info.typ->type != Tfun && !is_void(p->info.typ))
                {
                  fprintf(fout, "if (soap_in_%s(soap, %s, &a->%s::%s, \"%s\"))", c_ident(p->info.typ), field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name), xsi_type(p->info.typ));
                }
              }
              if (!is_invisible(p->sym->name) && !is_primclass(typ) && p->info.typ->type != Tfun && !is_void(p->info.typ))
              {
                if (is_container(p->info.typ))
                {
                  fprintf(fout, "\n\t\t\t\t\tcontinue;");
                }
                else
                {
                  fprintf(fout, "\n\t\t\t\t{\tsoap_flag_%s%d--;", ident(p->sym->name), i);
                  fixed_check(fout, p, t, "\t\t\t\t\t");
                  if (p->info.hasval && (is_string(p->info.typ) || is_wstring(p->info.typ)))
                  {
                    fprintf(fout, "\n\t\t\t\t\tif (*soap->href != '#' && a->%s::%s && ", ident(t->sym->name), ident(p->sym->name));
                    fprintf(fout, "!*a->%s::%s)", ident(t->sym->name), ident(p->sym->name));
                    fprintf(fout, "\n\t\t\t\t\t\ta->%s::%s%s;", ident(t->sym->name), ident(p->sym->name), c_init(p));
                  }
                  else if (p->info.hasval && is_fixedstring(p->info.typ))
                  {
                    fprintf(fout, "\n\t\t\t\t\tif (*soap->href != '#' && ");
                    fprintf(fout, "!*a->%s::%s)", ident(t->sym->name), ident(p->sym->name));
                    fprintf(fout, "\n\t\t\t\t\t\tsoap_strcpy(a->%s::%s, %d, \"%s\");", ident(t->sym->name), ident(p->sym->name), get_dimension(p->info.typ), cstring(p->info.val.s, 0));
                  }
                  else if (p->info.ptrval && (is_string(p->info.typ->ref) || is_wstring(p->info.typ->ref) || is_stdstring(p->info.typ->ref) || is_stdwstring(p->info.typ->ref)))
                  {
                    Tnode *ptr = (Tnode*)p->info.typ->ref;
                    if (is_smart(p->info.typ))
                      fprintf(fout, "\n\t\t\t\t\tif (*soap->href != '#' && a->%s::%s.get() && ", ident(t->sym->name), ident(p->sym->name));
                    else
                      fprintf(fout, "\n\t\t\t\t\tif (*soap->href != '#' && a->%s::%s && ", ident(t->sym->name), ident(p->sym->name));
                    if (is_string(ptr) || is_wstring(ptr))
                      fprintf(fout, "*a->%s::%s && !**a->%s::%s)", ident(t->sym->name), ident(p->sym->name), ident(t->sym->name), ident(p->sym->name));
                    else
                      fprintf(fout, "a->%s::%s->empty())", ident(t->sym->name), ident(p->sym->name));
                    fprintf(fout, "\n\t\t\t\t\t\t*a->%s::%s%s;", ident(t->sym->name), ident(p->sym->name), c_init(p));
                  }
                  fprintf(fout, "\n\t\t\t\t\tcontinue;");
                  fprintf(fout, "\n\t\t\t\t}");
                  if (p->info.ptrval && !p->info.fixed && !(is_string(p->info.typ->ref) || is_wstring(p->info.typ->ref) || is_stdstring(p->info.typ->ref) || is_stdwstring(p->info.typ->ref)))
                  {
                    Tnode *ptr = (Tnode*)p->info.typ->ref;
                    fprintf(fout, "\n\t\t\t\tif (soap->error == SOAP_EMPTY)\n\t\t\t\t{\t");
                    if (is_smart(p->info.typ))
                    {
                      if (is_smart_shared(p->info.typ))
                        fprintf(fout, "a->%s::%s = %s<%s>();", ident(t->sym->name), ident(p->sym->name), make_shared(p->info.typ), c_type(ptr));
                      else if (ptr->type == Tclass && !is_external(ptr) && !is_volatile(ptr) && !is_typedef(ptr))
                        fprintf(fout, "a->%s::%s = %s(%s_instantiate_%s(soap, SOAP_NO_LINK_TO_DELETE, soap->type, soap->arrayType, NULL));", ident(t->sym->name), ident(p->sym->name), c_type(p->info.typ), fprefix, c_ident(ptr));
                      else
                        fprintf(fout, "a->%s::%s = %s(SOAP_NEW(soap, %s));", ident(t->sym->name), ident(p->sym->name), c_type(p->info.typ), c_type(ptr));
                    }
                    else if (ptr->type == Tclass)
                    {
                      fprintf(fout, "if (!(a->%s::%s = (%s)%s_instantiate_%s(soap, -1, soap->type, soap->arrayType, NULL)))\n\t\t\t\t\t\treturn NULL;", ident(t->sym->name), ident(p->sym->name), c_type(p->info.typ), fprefix, c_ident(ptr));
                    }
                    else
                    {
                      fprintf(fout, "if (!(a->%s::%s = (%s)soap_malloc(soap, sizeof(%s))))\n\t\t\t\t\t\treturn NULL;", ident(t->sym->name), ident(p->sym->name), c_type(p->info.typ), c_type(ptr));
                    }
                    fprintf(fout, "\n\t\t\t\t\t*a->%s::%s%s;", ident(t->sym->name), ident(p->sym->name), c_init(p));
                    fprintf(fout, "\n\t\t\t\t\tsoap->error = SOAP_OK;\n\t\t\t\t\tsoap_flag_%s%d--;\n\t\t\t\t\tcontinue;\n\t\t\t\t}", ident(p->sym->name), i);
                  }
                }
                fprintf(fout, "\n\t\t\t}");
              }
              fflush(fout);
            }
          }
        }
        if (!is_discriminant(typ))
        {
          Entry *e;
          const char *nse1;
          i = 0;
          for (t = table; t; t = t->prev)
          {
            i++;
            e = entry(classtable, t->sym);
            nse1 = e ? ns_qualifiedElement(e->info.typ) : nse;
            for (p = t->list; p; p = p->next)
            {
              if (is_repetition(p) && (is_unmatched(p->next->sym) || is_invisible(p->next->sym->name)))
              {
                flag = 1;
                fprintf(fout, "\n\t\t\tif (soap->error == SOAP_TAG_MISMATCH && ");
                if (is_unmatched(p->next->sym))
                  fprintf(fout, "!soap_element_begin_in(soap, NULL, 1, NULL))");
                else if (is_invisible(p->next->sym->name))
                  fprintf(fout, "!soap_peek_element(soap))");
                fprintf(fout, "\n\t\t\t{\tif (a->%s::%s == NULL)\n\t\t\t\t{\tif (soap_blist_%s%d == NULL)\n\t\t\t\t\t\tsoap_blist_%s%d = soap_alloc_block(soap);", ident(t->sym->name), ident(p->next->sym->name), ident(p->next->sym->name), i, ident(p->next->sym->name), i);
                if (((Tnode*)p->next->info.typ->ref)->type == Tclass
                 || ((Tnode*)p->next->info.typ->ref)->type == Ttemplate
                 || has_class((Tnode*)p->next->info.typ->ref)
                 || (!cflag && ((Tnode*)p->next->info.typ->ref)->type == Tstruct))
                  fprintf(fout, "\n\t\t\t\t\ta->%s::%s = soap_block<%s>::push(soap, soap_blist_%s%d);\n\t\t\t\t\tif (a->%s::%s == NULL)\n\t\t\t\t\t\treturn NULL;", ident(t->sym->name), ident(p->next->sym->name), c_type(p->next->info.typ->ref), ident(p->next->sym->name), i, ident(t->sym->name), ident(p->next->sym->name));
                else if (((Tnode*)p->next->info.typ->ref)->type == Tpointer && (p->next->info.sto & Sconstptr))
                  fprintf(fout, "\n\t\t\t\t\ta->%s::%s = (const %s)soap_push_block_max(soap, soap_blist_%s%d, sizeof(%s));\n\t\t\t\t\tif (a->%s::%s == NULL)\n\t\t\t\t\t\treturn NULL;", ident(t->sym->name), ident(p->next->sym->name), c_type(p->next->info.typ), ident(p->next->sym->name), i, c_type((Tnode*)p->next->info.typ->ref), ident(t->sym->name), ident(p->next->sym->name));
                else
                  fprintf(fout, "\n\t\t\t\t\ta->%s::%s = (%s)soap_push_block_max(soap, soap_blist_%s%d, sizeof(%s));\n\t\t\t\t\tif (a->%s::%s == NULL)\n\t\t\t\t\t\treturn NULL;", ident(t->sym->name), ident(p->next->sym->name), c_type(p->next->info.typ), ident(p->next->sym->name), i, c_type((Tnode*)p->next->info.typ->ref), ident(t->sym->name), ident(p->next->sym->name));
                if (((Tnode*)p->next->info.typ->ref)->type == Tclass && !is_external((Tnode*)p->next->info.typ->ref) && !is_volatile((Tnode*)p->next->info.typ->ref) && !is_typedef((Tnode*)p->next->info.typ->ref))
                  fprintf(fout, "\n\t\t\t\t\ta->%s::%s->soap_default(soap);", ident(t->sym->name), ident(p->next->sym->name));
                else if (((Tnode*)p->next->info.typ->ref)->type != Tpointer  && !is_XML((Tnode*)p->next->info.typ->ref))
                  fprintf(fout, "\n\t\t\t\t\tsoap_default_%s(soap, a->%s::%s);", c_ident((Tnode*)p->next->info.typ->ref), ident(t->sym->name), ident(p->next->sym->name));
                else
                  fprintf(fout, "\n\t\t\t\t\t*a->%s::%s = NULL;", ident(t->sym->name), ident(p->next->sym->name));
                fprintf(fout, "\n\t\t\t\t}");
                if (!is_invisible(p->next->sym->name))
                  fprintf(fout, "\n\t\t\t\tsoap_revert(soap);");
                if (is_unmatched(p->next->sym))
                {
                  if (is_XML((Tnode*)p->next->info.typ->ref) && is_string((Tnode*)p->next->info.typ->ref))
                    fprintf(fout, "\n\t\t\t\tif (soap_inliteral(soap, NULL, (char**)a->%s::%s))", ident(t->sym->name), ident(p->next->sym->name));
                  else if (is_XML((Tnode*)p->next->info.typ->ref) && is_wstring((Tnode*)p->next->info.typ->ref))
                    fprintf(fout, "\n\t\t\t\tif (soap_inwliteral(soap, NULL, (wchar_t**)a->%s::%s))", ident(t->sym->name), ident(p->next->sym->name));
                  else if (is_string((Tnode*)p->next->info.typ->ref))
                    fprintf(fout, "\n\t\t\t\tif (soap_in_%s(soap, NULL, (char**)a->%s::%s, \"%s\"))", c_ident((Tnode*)p->next->info.typ->ref), ident(t->sym->name), ident(p->next->sym->name), xsi_type((Tnode*)p->next->info.typ->ref));
                  else if (is_wstring((Tnode*)p->next->info.typ->ref))
                    fprintf(fout, "\n\t\t\t\tif (soap_in_%s(soap, NULL, (wchar_t**)a->%s::%s, \"%s\"))", c_ident((Tnode*)p->next->info.typ->ref), ident(t->sym->name), ident(p->next->sym->name), xsi_type((Tnode*)p->next->info.typ->ref));
                  else
                    fprintf(fout, "\n\t\t\t\tif (soap_in_%s(soap, NULL, a->%s::%s, \"%s\"))", c_ident((Tnode*)p->next->info.typ->ref), ident(t->sym->name), ident(p->next->sym->name), xsi_type((Tnode*)p->next->info.typ->ref));
                }
                else
                {
                  if (is_XML((Tnode*)p->next->info.typ->ref) && is_string((Tnode*)p->next->info.typ->ref))
                    fprintf(fout, "\n\t\t\t\tif (soap_inliteral(soap, %s, (char**)a->%s::%s))", field(p->next, nse1), ident(t->sym->name), ident(p->next->sym->name));
                  else if (is_XML((Tnode*)p->next->info.typ->ref) && is_wstring((Tnode*)p->next->info.typ->ref))
                    fprintf(fout, "\n\t\t\t\tif (soap_inwliteral(soap, %s, (wchar_t**)a->%s::%s))", field(p->next, nse1), ident(t->sym->name), ident(p->next->sym->name));
                  else if (is_string((Tnode*)p->next->info.typ->ref))
                    fprintf(fout, "\n\t\t\t\tif (soap_in_%s(soap, %s, (char**)a->%s::%s, \"%s\"))", c_ident((Tnode*)p->next->info.typ->ref), field(p->next, nse1), ident(t->sym->name), ident(p->next->sym->name), xsi_type((Tnode*)p->next->info.typ->ref));
                  else if (is_wstring((Tnode*)p->next->info.typ->ref))
                    fprintf(fout, "\n\t\t\t\tif (soap_in_%s(soap, %s, (wchar_t**)a->%s::%s, \"%s\"))", c_ident((Tnode*)p->next->info.typ->ref), field(p->next, nse1), ident(t->sym->name), ident(p->next->sym->name), xsi_type((Tnode*)p->next->info.typ->ref));
                  else
                    fprintf(fout, "\n\t\t\t\tif (soap_in_%s(soap, %s, a->%s::%s, \"%s\"))", c_ident((Tnode*)p->next->info.typ->ref), field(p->next, nse1), ident(t->sym->name), ident(p->next->sym->name), xsi_type((Tnode*)p->next->info.typ->ref));
                }
                fprintf(fout, "\n\t\t\t\t{\ta->%s::%s++;\n\t\t\t\t\ta->%s::%s = NULL;\n\t\t\t\t\tcontinue;\n\t\t\t\t}\n\t\t\t}", ident(t->sym->name), ident(p->sym->name), ident(t->sym->name), ident(p->next->sym->name));
                p = p->next;
              }
              else if (is_repetition(p) || is_anytype(p) || is_choice(p))
              {
                p = p->next;
                continue;
              }
              else if (is_invisible(p->sym->name)
                  && !(p->info.sto & (Sconst | Sprivate | Sprotected))
                  && !is_transient(p->info.typ)
                  && !(p->info.sto & Sattribute))
              {
                flag = 1;
                if (is_string(p->info.typ) || is_wstring(p->info.typ) || is_stdstr(p->info.typ))
                  fprintf(fout, "\n\t\t\tif (soap_flag_%s%d && (soap->error == SOAP_TAG_MISMATCH || soap->error == SOAP_NO_TAG))", ident(p->sym->name), i);
                else if (is_container(p->info.typ))
                  fprintf(fout, "\n\t\t\tif (soap->error == SOAP_TAG_MISMATCH)");
                else
                  fprintf(fout, "\n\t\t\tif (soap_flag_%s%d && soap->error == SOAP_TAG_MISMATCH)", ident(p->sym->name), i);
                fprintf(fout, "\n\t\t\t{\t");
                if (is_XML(p->info.typ) && is_string(p->info.typ))
                {
                  fprintf(fout, "if (soap_inliteral(soap, %s, (char**)&a->%s::%s))", field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name));
                }
                else if (is_XML(p->info.typ) && is_wstring(p->info.typ))
                {
                  fprintf(fout, "if (soap_inwliteral(soap, %s, (wchar_t**)&a->%s::%s))", field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name));
                }
                else if (is_string(p->info.typ))
                {
                  fprintf(fout, "if (soap_in_%s(soap, %s, (char**)&a->%s::%s, \"%s\"))", c_ident(p->info.typ), field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name), xsi_type(p->info.typ));
                }
                else if (is_wstring(p->info.typ))
                {
                  fprintf(fout, "if (soap_in_%s(soap, %s, (wchar_t**)&a->%s::%s, \"%s\"))", c_ident(p->info.typ), field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name), xsi_type(p->info.typ));
                }
                else if (p->info.typ->type == Tarray)
                {
                  fprintf(fout, "if (soap_in_%s(soap, %s, a->%s::%s, \"%s\"))", c_ident(p->info.typ), field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name), xsi_type(p->info.typ));
                }
                else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
                {
                  fprintf(fout, "if ((a->%s::%s).soap_in(soap, %s, \"%s\"))", ident(t->sym->name), ident(p->sym->name), field_overridden(t, p, nse1), xsi_type(p->info.typ));
                }
                else if (p->info.typ->type != Tfun && !is_void(p->info.typ))
                {
                  fprintf(fout, "if (soap_in_%s(soap, %s, &a->%s::%s, \"%s\"))", c_ident(p->info.typ), field_overridden(t, p, nse1), ident(t->sym->name), ident(p->sym->name), xsi_type(p->info.typ));
                }
                if (is_container(p->info.typ))
                  fprintf(fout, "\n\t\t\t\t\tcontinue;");
                else
                {
                  fprintf(fout, "\n\t\t\t\t{\tsoap_flag_%s%d--;", ident(p->sym->name), i);
                  fprintf(fout, "\n\t\t\t\t\tcontinue;");
                  fprintf(fout, "\n\t\t\t\t}");
                }
                fprintf(fout, "\n\t\t\t}");
              }
            }
            if (!t->prev && e && e->info.typ && e->info.typ->baseid && !strcmp(e->info.typ->baseid->name, "soap_dom_element"))
            {
              fprintf(fout, "\n\t\t\tif (soap_flag_soap_dom_element && soap->error == SOAP_TAG_MISMATCH)\n\t\t\t\tif (soap_in_xsd__anyType(soap, NULL, static_cast<soap_dom_element*>(a), NULL))");
              fprintf(fout, "\n\t\t\t\t{\tsoap_flag_soap_dom_element = 0;");
              fprintf(fout, "\n\t\t\t\t\tcontinue;");
              fprintf(fout, "\n\t\t\t\t}");
            }
          }
          for (t = table; t; t = t->prev)
            for (p = t->list; p; p = p->next)
              if (p->info.sto & Sreturn)
                if (nse || has_ns_eq(NULL, p->sym->name))
                  fprintf(fout, "\n\t\t\tsoap_check_result(soap, \"%s\");", ns_add(p, nse));
          if (!flag && is_invisible(typ->id->name))
            fprintf(fout, "\n\tsoap->error = SOAP_TAG_MISMATCH;\n\ta = NULL;");
          if (!is_invisible(typ->id->name) || (table && (table->prev || table->list)))
          {
            if (!is_invisible(typ->id->name) || is_discriminant(typ))
              fprintf(fout, "\n\t\t\tif (soap->error == SOAP_TAG_MISMATCH)\n\t\t\t\tsoap->error = soap_ignore_element(soap);");
            else
              fprintf(fout, "\n\t\t\tif (soap->error == SOAP_TAG_MISMATCH && soap_flag)\n\t\t\t{\n\t\t\t\tsoap->error = SOAP_OK;\n\t\t\t\tbreak;\n\t\t\t}");
            if (!is_invisible(typ->id->name))
              fprintf(fout, "\n\t\t\tif (soap->error == SOAP_NO_TAG)");
            else
              fprintf(fout, "\n\t\t\tif (soap_flag && soap->error == SOAP_NO_TAG)");
            fprintf(fout, "\n\t\t\t\tbreak;");
            fprintf(fout, "\n\t\t\tif (soap->error)\n\t\t\t\treturn NULL;");
            fprintf(fout, "\n\t\t}");
          }
        }
        if (!is_discriminant(typ))
        {
          i = 0;
          for (t = table; t; t = t->prev)
            i++;
          for (; i > 0; i--)
          {
            t = table;
            for (j = 0; j < i-1; j++)
              t = t->prev;
            for (p = t->list; p; p = p->next)
            {
              if (is_repetition(p))
              {
                if (((Tnode*)p->next->info.typ->ref)->type == Tclass
                 || ((Tnode*)p->next->info.typ->ref)->type == Ttemplate
                 || has_class((Tnode*)p->next->info.typ->ref)
                 || (((Tnode*)p->next->info.typ->ref)->type == Tstruct && !cflag))
                {
                  fprintf(fout, "\n\t\tif (a->%s::%s)\n\t\t\tsoap_block<%s>::pop(soap, soap_blist_%s%d);", ident(t->sym->name), ident(p->next->sym->name), c_type(p->next->info.typ->ref), ident(p->next->sym->name), i);
                  fprintf(fout, "\n\t\tif (a->%s::%s)\n\t\t{\ta->%s::%s = soap_new_%s(soap, a->%s::%s);\n\t\t\tif (!a->%s::%s)\n\t\t\t\treturn NULL;\n\t\t\tsoap_block<%s>::save(soap, soap_blist_%s%d, a->%s::%s);\n\t\t}\n\t\telse\n\t\t{\ta->%s::%s = NULL;\n\t\t\tif (soap_blist_%s%d)\n\t\t\t\tsoap_block<%s>::end(soap, soap_blist_%s%d);\n\t\t}", ident(t->sym->name), ident(p->sym->name), ident(t->sym->name), ident(p->next->sym->name), c_ident(p->next->info.typ->ref), ident(t->sym->name), ident(p->sym->name), ident(t->sym->name), ident(p->next->sym->name), c_type(p->next->info.typ->ref), ident(p->next->sym->name), i, ident(t->sym->name), ident(p->next->sym->name), ident(t->sym->name), ident(p->next->sym->name), ident(p->next->sym->name), i, c_type(p->next->info.typ->ref), ident(p->next->sym->name), i);
                }
                else
                {
                  fprintf(fout, "\n\t\tif (a->%s::%s)\n\t\t\tsoap_pop_block(soap, soap_blist_%s%d);", ident(t->sym->name), ident(p->next->sym->name), ident(p->next->sym->name), i);
                  if ((((Tnode*)p->next->info.typ->ref)->type == Tpointer && (p->next->info.sto & Sconstptr)))
                    fprintf(fout, "\n\t\tif (a->%s::%s)\n\t\t{\ta->%s::%s = (const %s)soap_save_block(soap, soap_blist_%s%d, NULL, 1);\n\t\t}\n\t\telse\n\t\t{\ta->%s::%s = NULL;\n\t\t\tif (soap_blist_%s%d)\n\t\t\t\tsoap_end_block(soap, soap_blist_%s%d);\n\t\t}", ident(t->sym->name), ident(p->sym->name), ident(t->sym->name), ident(p->next->sym->name), c_type(p->next->info.typ), ident(p->next->sym->name), i, ident(t->sym->name), ident(p->next->sym->name), ident(p->next->sym->name), i, ident(p->next->sym->name), i);
                  else
                    fprintf(fout, "\n\t\tif (a->%s::%s)\n\t\t{\ta->%s::%s = (%s)soap_save_block(soap, soap_blist_%s%d, NULL, 1);\n\t\t}\n\t\telse\n\t\t{\ta->%s::%s = NULL;\n\t\t\tif (soap_blist_%s%d)\n\t\t\t\tsoap_end_block(soap, soap_blist_%s%d);\n\t\t}", ident(t->sym->name), ident(p->sym->name), ident(t->sym->name), ident(p->next->sym->name), c_type(p->next->info.typ), ident(p->next->sym->name), i, ident(t->sym->name), ident(p->next->sym->name), ident(p->next->sym->name), i, ident(p->next->sym->name), i);
                }
                p = p->next;
              }
            }
          }
        }
        if (!is_invisible(typ->id->name))
        {
          if (is_discriminant(typ))
            fprintf(fout, "\n\t\tif (tag && *tag != '-')\n\t\t{\tsoap->error = soap_ignore_element(soap);\n\t\t\tif (soap->error == SOAP_TAG_MISMATCH || soap_element_end_in(soap, tag))\n\t\t\t\treturn NULL;\n\t\t}");
          else
            fprintf(fout, "\n\t\tif (soap_element_end_in(soap, tag))\n\t\t\treturn NULL;");
        }
        strict = 0;
        nonempty = 0;
        if (!is_discriminant(typ))
        {
          i = 0;
          for (t = table; t; t = t->prev)
            i++;
          for (; i > 0; i--)
          {
            t = table;
            for (j = 0; j < i-1; j++)
              t = t->prev;
            for (p = t->list; p; p = p->next)
            {
              if (p->info.minOccurs > 0 && p->info.maxOccurs >= 0 && !(p->info.sto & (Sconst | Sprivate | Sprotected)) && !(p->info.sto & Sattribute) && p->info.typ->type != Tfun && !is_void(p->info.typ) && !is_transient(p->info.typ) && !is_container(p->info.typ) && !is_repetition(p) && !is_choice(p))
              {
                if (is_item(p))
                  continue;
                if (is_anytype(p))
                {
                  p = p->next;
                  if (p->info.minOccurs <= 0)
                    continue;
                }
                if ((p->info.typ->type != Tpointer && !is_smart(p->info.typ)) || p->info.nillable)
                {
                  if (strict == 0)
                  {
                    fprintf(fout, "\n\t\tif (%s(soap_flag_%s%d > " SOAP_LONG_FORMAT, strict_check(), ident(p->sym->name), i, p->info.maxOccurs - p->info.minOccurs);
                    strict = 1;
                  }
                  else
                    fprintf(fout, " || soap_flag_%s%d > " SOAP_LONG_FORMAT, ident(p->sym->name), i, p->info.maxOccurs - p->info.minOccurs);
                }
                else if (strict == 0)
                {
                  if (is_smart(p->info.typ))
                    fprintf(fout, "\n\t\tif (%s(!a->%s::%s.get()", strict_check(), ident(t->sym->name), ident(p->sym->name));
                  else
                    fprintf(fout, "\n\t\tif (%s(!a->%s::%s", strict_check(), ident(t->sym->name), ident(p->sym->name));
                  strict = 1;
                }
                else
                {
                  if (is_smart(p->info.typ))
                    fprintf(fout, " || !a->%s::%s.get()", ident(t->sym->name), ident(p->sym->name));
                  else
                    fprintf(fout, " || !a->%s::%s", ident(t->sym->name), ident(p->sym->name));
                }
                nonempty = 1;
              }
              else if (is_container(p->info.typ))
              {
                if (p->info.minOccurs > 0)
                {
                  if (p->info.typ->type == Tpointer)
                  {
                    if (strict == 0)
                    {
                      fprintf(fout, "\n\t\tif (%s(!a->%s::%s || a->%s::%s->size() < " SOAP_LONG_FORMAT, strict_check(), ident(t->sym->name), ident(p->sym->name), ident(t->sym->name), ident(p->sym->name), p->info.minOccurs);
                      strict = 1;
                    }
                    else
                      fprintf(fout, " || !a->%s::%s || a->%s::%s->size() < " SOAP_LONG_FORMAT, ident(t->sym->name), ident(p->sym->name), ident(t->sym->name), ident(p->sym->name), p->info.minOccurs);
                  }
                  else
                  {
                    if (strict == 0)
                    {
                      fprintf(fout, "\n\t\tif (%s(a->%s::%s.size() < " SOAP_LONG_FORMAT, strict_check(), ident(t->sym->name), ident(p->sym->name), p->info.minOccurs);
                      strict = 1;
                    }
                    else
                      fprintf(fout, " || a->%s::%s.size() < " SOAP_LONG_FORMAT, ident(t->sym->name), ident(p->sym->name), p->info.minOccurs);
                  }
                  nonempty = 1;
                }
                if (p->info.maxOccurs > 1)
                {
                  if (p->info.typ->type == Tpointer)
                  {
                    if (strict == 0)
                    {
                      fprintf(fout, "\n\t\tif (%s((a->%s::%s && a->%s::%s->size() > " SOAP_LONG_FORMAT ")", strict_check(), ident(t->sym->name), ident(p->sym->name), ident(t->sym->name), ident(p->sym->name), p->info.maxOccurs);
                      strict = 1;
                    }
                    else
                      fprintf(fout, " || (a->%s::%s && a->%s::%s->size() > " SOAP_LONG_FORMAT ")", ident(t->sym->name), ident(p->sym->name), ident(t->sym->name), ident(p->sym->name), p->info.maxOccurs);
                  }
                  else
                  {
                    if (strict == 0)
                    {
                      fprintf(fout, "\n\t\tif (%s(a->%s::%s.size() > " SOAP_LONG_FORMAT, strict_check(), ident(t->sym->name), ident(p->sym->name), p->info.maxOccurs);
                      strict = 1;
                    }
                    else
                      fprintf(fout, " || a->%s::%s.size() > " SOAP_LONG_FORMAT, ident(t->sym->name), ident(p->sym->name), p->info.maxOccurs);
                  }
                }
              }
              else if (is_repetition(p))
              {
                if (p->info.minOccurs > 0)
                {
                  if (strict == 0)
                  {
                    fprintf(fout, "\n\t\tif (%s(a->%s::%s < " SOAP_LONG_FORMAT, strict_check(), ident(t->sym->name), ident(p->sym->name), p->info.minOccurs);
                    strict = 1;
                  }
                  else
                    fprintf(fout, " || a->%s::%s < " SOAP_LONG_FORMAT, ident(t->sym->name), ident(p->sym->name), p->info.minOccurs);
                  nonempty = 1;
                }
                if (p->info.maxOccurs > 1)
                {
                  if (strict == 0)
                  {
                    fprintf(fout, "\n\t\tif (%s(a->%s::%s > " SOAP_LONG_FORMAT, strict_check(), ident(t->sym->name), ident(p->sym->name), p->info.maxOccurs);
                    strict = 1;
                  }
                  else
                    fprintf(fout, " || a->%s::%s > " SOAP_LONG_FORMAT, ident(t->sym->name), ident(p->sym->name), p->info.maxOccurs);
                }
                p = p->next;
              }
              else if (is_choice(p))
              {
                if (p->info.minOccurs != 0 && required_choice(p->next->info.typ) < 0)
                {
                  if (strict == 0)
                  {
                    fprintf(fout, "\n\t\tif (%s(soap_flag_%s%d", strict_check(), ident(p->next->sym->name), i);
                    strict = 1;
                  }
                  else
                    fprintf(fout, " || soap_flag_%s%d", ident(p->next->sym->name), i);
                  nonempty = 1;
                }
                p = p->next;
              }
            }
          }
          if (strict)
            fprintf(fout, "))\n\t\t{\tsoap->error = SOAP_OCCURS;\n\t\t\treturn NULL;\n\t\t}");
        }
        if (has_getter(typ))
          fprintf(fout, "\n\t\tif (a->get(soap))\n\t\t\treturn NULL;");
        if (!is_invisible(typ->id->name))
        {
          if (nonempty)
            fprintf(fout, "\n\t}\n\telse if (%s*soap->href != '#')\n\t{\tsoap->error = SOAP_OCCURS;\n\t\treturn NULL;", strict_check());
          fprintf(fout, "\n\t}\n\telse\n\t{");
          fprintf(fout, "\ta = (%s)soap_id_forward(soap, soap->href, (void*)a, 0, %s, %s, sizeof(%s), 0, %s_finsert, %s_fbase);", c_type_id(typ, "*"), soap_type(typ), soap_type(typ), c_type(typ), prefix, prefix);
          fprintf(fout, "\n\t\tif (soap->body && soap_element_end_in(soap, tag))\n\t\t\treturn NULL;\n\t}");
        }
        fprintf(fout, "\n\treturn a;\n}");
      }
      break;
    case Tunion:
      if (is_external(typ) && !is_volatile(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 %s SOAP_FMAC2 soap_in_%s(struct soap*, int*, %s);", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 %s SOAP_FMAC4 soap_in_%s(struct soap*, int*, %s);", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*"));
      fprintf(fout, "\n\nSOAP_FMAC3 %s SOAP_FMAC4 soap_in_%s(struct soap *soap, int *choice, %s)\n{", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*a"));
      fprintf(fout, "\n\t(void)a; /* appease -Wall -Werror */");
      fprintf(fout, "\n\tsoap->error = SOAP_TAG_MISMATCH;");
      table = (Table *)typ->ref;
      if (table)
      {
        for (p = table->list; p; p = p->next)
        {
          if (p->info.typ->type == Tfun || p->info.typ->type == Tunion)
            ;
          else if (p->info.sto & (Sconst | Sprivate | Sprotected))
            fprintf(fout, "\n\t/* non-serializable %s skipped */", ident(p->sym->name));
          else if (p->info.sto & Sattribute)
            ;
          else if (is_transient(p->info.typ))
          {
            if (!is_pointer_to_derived(p))
              fprintf(fout, "\n\t/* transient %s skipped */", ident(p->sym->name));
          }
          else if (!is_invisible(p->sym->name))
          {
            if (is_unmatched(p->sym))
            {
              if (is_XML(p->info.typ) && is_string(p->info.typ))
                fprintf(fout, "\n\tif (soap->error == SOAP_TAG_MISMATCH && soap_inliteral(soap, NULL, (char**)&a->%s))", ident(p->sym->name));
              else if (is_XML(p->info.typ) && is_wstring(p->info.typ))
                fprintf(fout, "\n\tif (soap->error == SOAP_TAG_MISMATCH && soap_inwliteral(soap, NULL, (wchar_t**)&a->%s))", ident(p->sym->name));
              else if (p->info.typ->type == Tarray)
                fprintf(fout, "\n\tif (soap->error == SOAP_TAG_MISMATCH && soap_in_%s(soap, NULL, a->%s, \"%s\"))", c_ident(p->info.typ), ident(p->sym->name), xsi_type(p->info.typ));
              else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
                fprintf(fout, "\n\tif (soap->error == SOAP_TAG_MISMATCH && a->%s.soap_in(soap, NULL, \"%s\"))", ident(p->sym->name), xsi_type(p->info.typ));
              else if (p->info.typ->type != Tfun && !is_void(p->info.typ))
              {
                if (p->info.typ->type == Tpointer)
                  fprintf(fout, "\n\ta->%s = NULL;", ident(p->sym->name));
                fprintf(fout, "\n\tif (soap->error == SOAP_TAG_MISMATCH && soap_in_%s(soap, NULL, &a->%s, \"%s\"))", c_ident(p->info.typ), ident(p->sym->name), xsi_type(p->info.typ));
              }
            }
            else
            {
              if (is_XML(p->info.typ) && is_string(p->info.typ))
                fprintf(fout, "\n\tif (soap->error == SOAP_TAG_MISMATCH && soap_inliteral(soap, \"%s\", (char**)&a->%s))", ns_add(p, nse), ident(p->sym->name));
              else if (is_XML(p->info.typ) && is_wstring(p->info.typ))
                fprintf(fout, "\n\tif (soap->error == SOAP_TAG_MISMATCH && soap_inwliteral(soap, \"%s\", (wchar_t**)&a->%s))", ns_add(p, nse), ident(p->sym->name));
              else if (p->info.typ->type == Tarray)
                fprintf(fout, "\n\tif (soap->error == SOAP_TAG_MISMATCH && soap_in_%s(soap, \"%s\", a->%s, \"%s\"))", c_ident(p->info.typ), ns_add(p, nse), ident(p->sym->name), xsi_type(p->info.typ));
              else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
                fprintf(fout, "\n\tif (soap->error == SOAP_TAG_MISMATCH && a->%s.soap_in(soap, \"%s\", \"%s\"))", ident(p->sym->name), ns_add(p, nse), xsi_type(p->info.typ));
              else if (p->info.typ->type != Tfun && !is_void(p->info.typ))
              {
                if (p->info.typ->type == Tpointer)
                  fprintf(fout, "\n\ta->%s = NULL;", ident(p->sym->name));
                fprintf(fout, "\n\tif (soap->error == SOAP_TAG_MISMATCH && soap_in_%s(soap, \"%s\", &a->%s, \"%s\"))", c_ident(p->info.typ), ns_add(p, nse), ident(p->sym->name), xsi_type(p->info.typ));
              }
            }
            fprintf(fout, "\n\t{\t*choice = %s;", soap_union_member(typ, p));
            fprintf(fout, "\n\t\treturn a;");
            fprintf(fout, "\n\t}");
            fflush(fout);
          }
        }
      }
      table = (Table *)typ->ref;
      if (table)
      {
        for (p = table->list; p; p = p->next)
        {
          if (p->info.typ->type == Tfun || p->info.typ->type == Tunion)
            ;
          else if (p->info.sto & (Sconst | Sprivate | Sprotected))
            ;
          else if (is_transient(p->info.typ))
            ;
          else if (p->info.sto & Sattribute)
            ;
          else if (is_invisible(p->sym->name))
          {
            if (is_XML(p->info.typ) && is_string(p->info.typ))
              fprintf(fout, "\n\tif (soap->error == SOAP_TAG_MISMATCH && soap_inliteral(soap, \"%s\", (char**)&a->%s))", ns_add(p, nse), ident(p->sym->name));
            else if (is_XML(p->info.typ) && is_wstring(p->info.typ))
              fprintf(fout, "\n\tif (soap->error == SOAP_TAG_MISMATCH && soap_inwliteral(soap, \"%s\", (wchar_t**)&a->%s))", ns_add(p, nse), ident(p->sym->name));
            else if (p->info.typ->type == Tarray)
              fprintf(fout, "\n\tif (soap->error == SOAP_TAG_MISMATCH && soap_in_%s(soap, \"%s\", a->%s, NULL))", c_ident(p->info.typ), ns_add(p, nse), ident(p->sym->name));
            else if (p->info.typ->type == Tclass && !is_external(p->info.typ) && !is_volatile(p->info.typ) && !is_typedef(p->info.typ))
              fprintf(fout, "\n\tif (soap->error == SOAP_TAG_MISMATCH && a->%s.soap_in(soap, \"%s\", NULL))", ident(p->sym->name), ns_add(p, nse));
            else if (p->info.typ->type != Tfun && !is_void(p->info.typ))
            {
              if (p->info.typ->type == Tpointer)
                fprintf(fout, "\n\ta->%s = NULL;", ident(p->sym->name));
              fprintf(fout, "\n\tif (soap->error == SOAP_TAG_MISMATCH && soap_in_%s(soap, \"%s\", &a->%s, NULL))", c_ident(p->info.typ), ns_add(p, nse), ident(p->sym->name));
            }
            fprintf(fout, "\n\t{\t*choice = %s;", soap_union_member(typ, p));
            fprintf(fout, "\n\t\treturn a;");
            fprintf(fout, "\n\t}");
            fflush(fout);
          }
        }
      }
      fprintf(fout, "\n\t*choice = %d;", required_choice(typ));
      fprintf(fout, "\n\tif (!soap->error)\n\t\tsoap->error = SOAP_TAG_MISMATCH;\n\treturn NULL;\n}");
      break;
    case Tpointer:
      if (is_external(typ) && !is_volatile(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 %s SOAP_FMAC2 soap_in_%s(struct soap*, const char*, %s, const char*);", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 %s SOAP_FMAC4 soap_in_%s(struct soap*, const char*, %s, const char*);", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*"));
      fprintf(fout, "\n\nSOAP_FMAC3 %s SOAP_FMAC4 soap_in_%s(struct soap *soap, const char *tag, %s, const char *type)\n{", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*a"));
      fprintf(fout, "\n\t(void)type; /* appease -Wall -Werror */\n\tif (soap_element_begin_in(soap, tag, 1, NULL))");
      fprintf(fout, "\n\t\treturn NULL;");
      if (is_template(typ->ref))
      {
        fprintf(fout, "\n\tsoap_revert(soap);");
        fprintf(fout, "\n\tif (!a)\n\t{\tif (!(a = (%s)soap_malloc(soap, sizeof(%s))))\n\t\t\treturn NULL;\n\t\t*a = NULL;\n\t}", c_type_id(typ, "*"), c_type(typ));
        fprintf(fout, "\n\tif (!(*a = soap_in_%s(soap, tag, *a, type)))\n\t\treturn NULL;", c_ident((Tnode*)typ->ref));
        fprintf(fout, "\n\treturn a;\n}");
      }
      else if (((Tnode *) typ->ref)->type == Tclass && !is_external((Tnode*)typ->ref) && !is_volatile((Tnode*)typ->ref) && !is_typedef((Tnode*)typ->ref))
      {
        fprintf(fout, "\n\tif (!a)\n\t\tif (!(a = (%s)soap_malloc(soap, sizeof(%s))))\n\t\t\treturn NULL;", c_type_id(typ, "*"), c_type(typ));
        fprintf(fout, "\n\t*a = NULL;\n\tif (!soap->null && *soap->href != '#')");
        fprintf(fout, "\n\t{\tsoap_revert(soap);");
        fprintf(fout, "\n\t\tif (!(*a = (%s)%s_instantiate_%s(soap, -1, soap->type, soap->arrayType, NULL)))", c_type(typ), fprefix, c_ident((Tnode*)typ->ref));
        fprintf(fout, "\n\t\t\treturn NULL;");
        fprintf(fout, "\n\t\t(*a)->soap_default(soap);");
        fprintf(fout, "\n\t\tif (!(*a)->soap_in(soap, tag, NULL))");
        fprintf(fout, "\n\t\t{\t*a = NULL;\n\t\t\treturn NULL;\n\t\t}");
        fprintf(fout, "\n\t}\n\telse\n\t{\ta = (%s)soap_id_lookup(soap, soap->href, (void**)a, %s, sizeof(%s), %d, %s_fbase);", c_type_id(typ, "*"), soap_type(reftype(typ->ref)), c_type(reftype(typ->ref)), reflevel(typ->ref), prefix);
        fprintf(fout, "\n\t\tif (soap->body && soap_element_end_in(soap, tag))\n\t\t\treturn NULL;");
        fprintf(fout, "\n\t}\n\treturn a;\n}");
      }
      else
      {
        fprintf(fout, "\n\tif (!a)\n\t\tif (!(a = (%s)soap_malloc(soap, sizeof(%s))))\n\t\t\treturn NULL;", c_type_id(typ, "*"), c_type(typ));
        fprintf(fout, "\n\t*a = NULL;\n\tif (!soap->null && *soap->href != '#')");
        fprintf(fout, "\n\t{\tsoap_revert(soap);");
        fprintf(fout, "\n\t\tif (!(*a = soap_in_%s(soap, tag, *a, type)))", c_ident((Tnode*)typ->ref));
        fprintf(fout, "\n\t\t\treturn NULL;");
        fprintf(fout, "\n\t}\n\telse\n\t{\ta = (%s)soap_id_lookup(soap, soap->href, (void**)a, %s, sizeof(%s), %d, NULL);", c_type_id(typ, "*"), soap_type(reftype(typ->ref)), c_type(reftype(typ->ref)), reflevel(typ->ref));
        fprintf(fout, "\n\t\tif (soap->body && soap_element_end_in(soap, tag))\n\t\t\treturn NULL;");
        fprintf(fout, "\n\t}\n\treturn a;\n}");
      }
      break;
    case Tarray:
      temp = get_item_type(typ, &cardinality);
      if (is_external(typ) && !is_volatile(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 %s * SOAP_FMAC2 soap_in_%s(struct soap*, const char*, %s, const char*);", c_type(temp), c_ident(typ), c_type_id(typ, ""));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 %s * SOAP_FMAC4 soap_in_%s(struct soap*, const char*, %s, const char*);", c_type(temp), c_ident(typ), c_type_id(typ, ""));
      fprintf(fout, "\n\nSOAP_FMAC3 %s * SOAP_FMAC4 soap_in_%s(struct soap *soap, const char *tag, %s, const char *type)\n{", c_type(temp), c_ident(typ), c_type_id(typ, "a"));
      fprintf(fout, "\n\tif (soap_element_begin_in(soap, tag, 0, NULL))");
      fprintf(fout, "\n\t\treturn NULL;");
      fprintf(fout, "\n\tif (soap_match_array(soap, type))");
      fprintf(fout, "\n\t{\tsoap->error = SOAP_TYPE;\n\t\treturn NULL;\n\t}");
      fprintf(fout, "\n\ta = (%s)soap_id_enter(soap, soap->id, a, %s, sizeof(%s), NULL, NULL, NULL, NULL);", c_type_id(typ->ref, "(*)"), soap_type(typ), c_type_id(typ, ""));
      fprintf(fout, "\n\tif (!a)\n\t\treturn NULL;");
      fprintf(fout, "\n\tsoap_default_%s(soap, a);", c_ident(typ));
      fprintf(fout, "\n\tif (soap->body && *soap->href != '#')");
      fprintf(fout, "\n\t{\tint i;\n\t\tfor (i = 0; i < %d; i++)", get_dimension(typ));
      fprintf(fout, "\n\t\t{\tsoap_peek_element(soap);\n\t\t\tif (soap->position)\n\t\t\t{\ti = soap->positions[0];\n\t\t\t\tif (i < 0 || i >= %d)\n\t\t\t\t{\tsoap->error = SOAP_IOB;\n\t\t\t\t\treturn NULL;\n\t\t\t\t}\n\t\t\t}", get_dimension(typ));
      fprintf(fout, "\n\t\t\tif (!soap_in_%s(soap, NULL, a", c_ident((Tnode*)typ->ref));
      if (cardinality > 1)
        fprintf(fout, "[i]");
      else
        fprintf(fout, "+i");
      fprintf(fout, ", \"%s\"))", xsi_type((Tnode*)typ->ref));
      fprintf(fout, "\n\t\t\t{\tif (soap->error != SOAP_NO_TAG)\n\t\t\t\t\treturn NULL;");
      fprintf(fout, "\n\t\t\t\tsoap->error = SOAP_OK;");
      fprintf(fout, "\n\t\t\t\tbreak;");
      fprintf(fout, "\n\t\t\t}");
      fprintf(fout, "\n\t\t}");
      fprintf(fout, "\n\t\tif ((soap->mode & SOAP_C_NOIOB))\n\t\t\twhile (soap_element_end_in(soap, tag) == SOAP_SYNTAX_ERROR)\n\t\t\t{\tsoap->peeked = 1;\n\t\t\t\tsoap_ignore_element(soap);\n\t\t\t}");
      fprintf(fout, "\n\t\telse if (soap_element_end_in(soap, tag))\n\t\t{\tif (soap->error == SOAP_SYNTAX_ERROR)\n\t\t\t\tsoap->error = SOAP_IOB;\n\t\t\treturn NULL;\n\t\t}");
      if (!cflag)
        fprintf(fout, "\n\t}\n\telse\n\t{\ta = (%s)soap_id_forward(soap, soap->href, (void*)soap_id_enter(soap, soap->id, a, %s, sizeof(%s), NULL, NULL, NULL, NULL), 0, %s, %s, sizeof(%s), 0, %s_finsert, NULL);", c_type_id(typ->ref, "(*)"), soap_type(typ), c_type_id(typ, ""), soap_type(typ), soap_type(typ), c_type_id(typ, ""), prefix);
      else
        fprintf(fout, "\n\t}\n\telse\n\t{\ta = (%s)soap_id_forward(soap, soap->href, (void*)soap_id_enter(soap, soap->id, a, %s, sizeof(%s), NULL, NULL, NULL, NULL), 0, %s, %s, sizeof(%s), 0, NULL, NULL);", c_type_id(typ->ref, "(*)"), soap_type(typ), c_type_id(typ, ""), soap_type(typ), soap_type(typ), c_type_id(typ, ""));
      fprintf(fout, "\n\t\tif (soap->body && soap_element_end_in(soap, tag))\n\t\t\treturn NULL;");
      fprintf(fout, "\n\t}\n\treturn (%s*)a;\n}", c_type(temp));
      break;
    case Tenum:
    case Tenumsc:
      if (is_external(typ) && !is_volatile(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 %s SOAP_FMAC2 soap_in_%s(struct soap*, const char*, %s, const char*);", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      fprintf(fhead, "\nSOAP_FMAC3 %s SOAP_FMAC4 soap_in_%s(struct soap*, const char*, %s, const char*);", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*"));
      if (!is_typedef(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC3S int SOAP_FMAC4S soap_s2%s(struct soap*, const char*, %s);", c_ident(typ), c_type_id(typ, "*"));
        fprintf(fout, "\n\nSOAP_FMAC3S int SOAP_FMAC4S soap_s2%s(struct soap *soap, const char *s, %s)\n{", c_ident(typ), c_type_id(typ, "*a"));
        if (!is_mask(typ))
        {
          LONG64 min = 0;
          LONG64 max = 0;
          fprintf(fout, "\n\tconst struct soap_code_map *map;");
          t = (Table*)typ->ref;
          if (t && t->list && has_ns_eq(NULL, ns_remove3(t->list->sym->name, c_ident(typ))))
          {
            fprintf(fout, "\n\tchar *t;");
            fprintf(fout, "\n\tif (!s)\n\t\treturn soap->error;");
            fprintf(fout, "\n\tsoap_s2QName(soap, s, &t, %ld, %ld, %s);", minlen(typ), maxlen(typ), pattern(typ));
            fprintf(fout, "\n\tmap = soap_code(soap_codes_%s, t);", c_ident(typ));
          }
          else
          {
            fprintf(fout, "\n\tif (!s)\n\t\treturn soap->error;");
            fprintf(fout, "\n\tmap = soap_code(soap_codes_%s, s);", c_ident(typ));
          }
          for (t = (Table*)typ->ref; t; t = t->prev)
          {
            for (p = t->list; p; p = p->next)
            {
              if (p->info.val.i < min)
                min = p->info.val.i;
              if (p->info.val.i > max)
                max = p->info.val.i;
            }
          }
          if (is_boolean(typ))
            fprintf(fout, "\n\tif (map)\n\t\t*a = (%s)(map->code != 0);\n\telse if (!*s)\n\t\treturn soap->error = SOAP_EMPTY;\n\telse\n\t{\tlong n;\n\t\tif (soap_s2long(soap, s, &n) || n < 0 || n > 1)\n\t\t\treturn soap->error = SOAP_TYPE;\n\t\t*a = (%s)(n != 0);\n\t}\n\treturn SOAP_OK;\n}", c_type(typ), c_type(typ));
          else if (sflag)
            fprintf(fout, "\n\tif (map)\n\t\t*a = (%s)map->code;\n\telse if (!*s)\n\t\treturn soap->error = SOAP_EMPTY;\n\telse\n\t\treturn soap->error = SOAP_TYPE;\n\treturn SOAP_OK;\n}", c_type(typ));
          else if (min >= -2147483648LL && min <= 2147483647LL && max >= -2147483648LL && max <= 2147483647LL)
            fprintf(fout, "\n\tif (map)\n\t\t*a = (%s)map->code;\n\telse if (!*s)\n\t\treturn soap->error = SOAP_EMPTY;\n\telse\n\t{\tint n;\n\t\tif (soap_s2int(soap, s, &n) || n < " SOAP_LONG_FORMAT " || n > " SOAP_LONG_FORMAT ")\n\t\t\treturn soap->error = SOAP_TYPE;\n\t\t*a = (%s)n;\n\t}\n\treturn SOAP_OK;\n}", c_type(typ), min, max, c_type(typ));
          else
            fprintf(fout, "\n\tif (map)\n\t\t*a = (%s)map->code;\n\telse if (!*s)\n\t\treturn soap->error = SOAP_EMPTY;\n\telse\n\t{\tLONG64 n;\n\t\tif (soap_s2LONG64(soap, s, &n) || n < " SOAP_LONG_FORMAT "LL || n > " SOAP_LONG_FORMAT "LL)\n\t\t\treturn soap->error = SOAP_TYPE;\n\t\t*a = (%s)n;\n\t}\n\treturn SOAP_OK;\n}", c_type(typ), min, max, c_type(typ));
        }
        else
        {
          t = (Table*)typ->ref;
          if (t && t->list && has_ns_eq(NULL, ns_remove3(t->list->sym->name, c_ident(typ))))
          {
            fprintf(fout, "\n\tchar *t;");
            fprintf(fout, "\n\tsoap_s2QName(soap, s, &t, %ld, %ld, %s);", minlen(typ), maxlen(typ), pattern(typ));
            fprintf(fout, "\n\t*a = (%s)soap_code_bits(soap_codes_%s, t);", c_type(typ), c_ident(typ));
          }
          else
            fprintf(fout, "\n\t(void)soap; /* appease -Wall -Werror */\n\t*a = (%s)soap_code_bits(soap_codes_%s, s);", c_type(typ), c_ident(typ));
          fprintf(fout, "\n\treturn SOAP_OK;\n}");
        }
      }
      fprintf(fout, "\n\nSOAP_FMAC3 %s SOAP_FMAC4 soap_in_%s(struct soap *soap, const char *tag, %s, const char *type)\n{", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*a"));
      if (is_boolean(typ))
      {
        fprintf(fout, "\n\tif (soap_element_begin_in(soap, tag, 0, NULL))");
        fprintf(fout, "\n\t\treturn NULL;");
        fprintf(fout, "\n\tif (*soap->type && soap_match_tag(soap, soap->type, type) && soap_match_tag(soap, soap->type, \":boolean\"))");
        fprintf(fout, "\n\t{\tsoap->error = SOAP_TYPE;\n\t\treturn NULL;\n\t}");
      }
      else if (typ->sym)
      {
        fprintf(fout, "\n\tif (soap_element_begin_in(soap, tag, 0, NULL))");
        fprintf(fout, "\n\t\treturn NULL;");
        fprintf(fout, "\n\tif (*soap->type && soap_match_tag(soap, soap->type, type) && soap_match_tag(soap, soap->type, \"%s\"))", base_type(typ, ""));
        fprintf(fout, "\n\t{\tsoap->error = SOAP_TYPE;\n\t\treturn NULL;\n\t}");
      }
      else
      {
        fprintf(fout, "\n\tif (soap_element_begin_in(soap, tag, 0, type))");
        fprintf(fout, "\n\t\treturn NULL;");
      }
      fprintf(fout, "\n\ta = (%s*)soap_id_enter(soap, soap->id, a, %s, sizeof(%s), NULL, NULL, NULL, NULL);", c_type(typ), soap_type(typ), c_type(typ));
      fprintf(fout, "\n\tif (!a)\n\t\treturn NULL;");
      fprintf(fout, "\n\tif (*soap->href != '#')\n\t{");
      fprintf(fout, "\tint err = soap_s2%s(soap, soap_value(soap), a);\n\t\tif ((soap->body && soap_element_end_in(soap, tag)) || err)\n\t\t\treturn NULL;", c_ident(typ));
      fprintf(fout, "\n\t}\n\telse\n\t{\ta = (%s)soap_id_forward(soap, soap->href, (void*)a, 0, %s, %s, sizeof(%s), 0, NULL, NULL);", c_type_id(typ, "*"), soap_type(typ), soap_type(typ), c_type(typ));
      fprintf(fout, "\n\t\tif (soap->body && soap_element_end_in(soap, tag))\n\t\t\treturn NULL;");
      fprintf(fout, "\n\t}\n\treturn a;\n}");
      break;
    case Ttemplate:
      if (is_external(typ) && !is_volatile(typ))
      {
        fprintf(fhead, "\nSOAP_FMAC1 %s SOAP_FMAC2 soap_in_%s(struct soap*, const char*, %s, const char*);", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*"));
        return;
      }
      if (is_typedef(typ))
      {
        fprintf(fhead, "\n\n#define soap_in_%s soap_in_%s\n", c_ident(typ), t_ident(typ));
        return;
      }
      n = (Tnode*)typ->ref;
      if (n->type == Tfun || is_void(n))
        return;
      fprintf(fhead, "\nSOAP_FMAC3 %s SOAP_FMAC4 soap_in_%s(struct soap*, const char*, %s, const char*);", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*"));
      fprintf(fout, "\n\nSOAP_FMAC3 %s SOAP_FMAC4 soap_in_%s(struct soap *soap, const char *tag, %s, const char *type)\n{", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*a"));
      fprintf(fout, "\n\t(void)type; /* appease -Wall -Werror */");
      if (is_smart(typ))
      {
        fprintf(fout, "\n\tif (soap_element_begin_in(soap, tag, 1, NULL))");
        fprintf(fout, "\n\t\treturn NULL;");
        fprintf(fout, "\n\tif (!a && !(a = soap_new_%s(soap)))\n\t\treturn NULL;", c_ident(typ));
        fprintf(fout, "\n\tif (!soap->null && *soap->href != '#')\n\t{");
        if (is_smart_shared(typ))
        {
          fprintf(fout, "\tvoid **x = soap_id_smart(soap, soap->id, %s, sizeof(%s));", soap_type(n), c_type(n));
          if (n->type == Tclass && !is_external(n) && !is_volatile(n) && !is_typedef(n))
            fprintf(fout, "\n\t\tif (x && *x)\n\t\t\t*a = *(%s)(*x);\n\t\telse if ((*a = %s(%s_instantiate_%s(soap, SOAP_NO_LINK_TO_DELETE, soap->type, soap->arrayType, NULL))))\n\t\t{\tif (x)\n\t\t\t\t*x = (void*)a;\n\t\t}\n\t\telse\n\t\t\treturn NULL;", c_type_id(typ, "*"), c_type(typ), fprefix, c_ident(n));
          else
            fprintf(fout, "\n\t\tif (x && *x)\n\t\t\t*a = *(%s)(*x);\n\t\telse\n\t\t{\t*a = %s<%s>();\n\t\t\tif (x)\n\t\t\t\t*x = (void*)a;\n\t\t}", c_type_id(typ, "*"), make_shared(typ), c_type(n));
        }
        else if (n->type == Tclass && !is_external(n) && !is_volatile(n) && !is_typedef(n))
          fprintf(fout, "\tif (!(*a = %s(%s_instantiate_%s(soap, SOAP_NO_LINK_TO_DELETE, soap->type, soap->arrayType, NULL))))\n\t\t\treturn NULL;", c_type(typ), fprefix, c_ident(n));
        else
          fprintf(fout, "\tif (!(*a = %s(SOAP_NEW(soap, %s))))\n\t\t\treturn NULL;", c_type(typ), c_type(n));
        fprintf(fout, "\n\t\tsoap_revert(soap);");
        if (is_XML(n) && is_string(n))
          fprintf(fout, "\n\t\tif (!soap_inliteral(soap, tag, (char*const*)a->get()))\n\t\t\treturn NULL;");
        else if (is_XML(n) && is_wstring(n))
          fprintf(fout, "\n\t\tif (!soap_inwliteral(soap, tag, (wchar_t*const*)a->get()))\n\t\t\treturn NULL;");
        else if (n->type == Tarray)
          fprintf(fout, "\n\t\tif (!soap_in_%s(soap, tag, a->get(), \"%s\"))\n\t\t\treturn NULL;", c_ident(n), xsi_type(n));
        else if (n->type == Tclass && !is_external(n) && !is_volatile(n) && !is_typedef(n))
        {
          fprintf(fout, "\n\t\t(*a)->soap_default(soap);");
          fprintf(fout, "\n\t\tif (!(*a)->soap_in(soap, tag, NULL))\n\t\t\treturn NULL;");
        }
        else
        {
          fprintf(fout, "\n\t\tsoap_default_%s(soap, a->get());", c_ident(n));
          fprintf(fout, "\n\t\tif (!soap_in_%s(soap, tag, a->get(), \"%s\"))\n\t\t\treturn NULL;", c_ident(n), xsi_type(n));
        }
        fprintf(fout, "\n\t}\n\telse\n\t{");
        fprintf(fout, "\tif (!soap_id_forward(soap, soap->href, a, 0, %s, %s, sizeof(%s), %d, %s_finsert, %s_fbase))\n\t\t\treturn NULL;", soap_type(reftype(n)), soap_type(typ), c_type(reftype(n)), reflevel(n) + 1, prefix, prefix); /* reflevel + 1 since smart pointer accepts derived types */
        fprintf(fout, "\n\t\tif (soap->body && soap_element_end_in(soap, tag))\n\t\t\treturn NULL;");
        fprintf(fout, "\n\t}\n\treturn a;\n}");
      }
      else if (c11flag && (!strcmp(typ->id->name, "std::deque") || !strcmp(typ->id->name, "std::list") || (!strcmp(typ->id->name, "std::vector") && !is_bool(n)))) /* use C++11 emplace_back */
      {
        fprintf(fout, "\n\tshort soap_flag;");
        fprintf(fout, "\n\tfor (soap_flag = 0;; soap_flag = 1)\n\t{");
        fprintf(fout, "\n\t\tif (tag && *tag != '-')\n\t\t{\tif (soap_element_begin_in(soap, tag, 1, NULL))\n\t\t\t\tbreak;\n\t\t\tsoap_revert(soap);\n\t\t}");
        fprintf(fout, "\n\t\tif (!a && !(a = soap_new_%s(soap)))\n\t\t\treturn NULL;", c_ident(typ));
        if (!strcmp(typ->id->name, "std::vector") && !is_primitive(n) && n->type != Tpointer)
        {
          fprintf(fout, "\n\t\tif (!a->empty() && a->size() == a->capacity())\n\t\t{\tconst void *p = &a->front();");
          fprintf(fout, "\n\t\t\ta->emplace_back();");
          fprintf(fout, "\n\t\t\tDBGLOG(TEST, SOAP_MESSAGE(fdebug, \"Vector capacity increased to %%lu to fit %%lu items: updating pointers\\n\", a->capacity(), a->size()));");
          fprintf(fout, "\n\t\t\tsoap_update_pointers(soap, (const char*)&a->front(), (const char*)p, (a->size() - 1) * sizeof(%s));", c_type(n));
          fprintf(fout, "\n\t\t}\n\t\telse\n\t\t{\ta->emplace_back();\n\t\t}");
          fprintf(fout, "\n\t\t%s *n = &a->back();\n\t\t", c_type(n));
        }
        else
        {
          fprintf(fout, "\n\t\ta->emplace_back();\n\t\t%s *n = &a->back();\n\t\t", c_type(n));
        }
        if (n->type == Tpointer)
          fprintf(fout, "*n = NULL;");
        else if (n->type == Tarray)
          fprintf(fout, "soap_default_%s(soap, n);", c_ident(n));
        else if (n->type == Tclass && !is_external(n) && !is_volatile(n) && !is_typedef(n))
          fprintf(fout, "n->soap_default(soap);");
        else if (n->type != Tfun && !is_void(n) && !is_XML(n))
          fprintf(fout, "soap_default_%s(soap, n);", c_ident(n));
        if (!is_primitive(n) && n->type != Tpointer)
          fprintf(fout, "\n\t\tshort soap_shaky = soap_begin_shaky(soap);");
        fprintf(fout, "\n\t\tif (tag && *tag != '-' && (*soap->id || *soap->href == '#'))\n\t\t{\tif (");
        fprintf(fout, "!soap_id_forward(soap, *soap->id?soap->id:soap->href, a, (size_t)a->size() - 1, %s, %s, sizeof(%s), %d, %s_finsert, %s_fbase))\n\t\t\t\tbreak;\n\t\t\tif (", soap_type(reftype(n)), soap_type(typ), c_type(reftype(n)), reflevel(n), prefix, prefix);
        if (is_XML(n) && is_string(n))
          fprintf(fout, "!soap_inliteral(soap, tag, NULL)");
        else if (is_XML(n) && is_wstring(n))
          fprintf(fout, "!soap_inwliteral(soap, tag, NULL)");
        else if (n->type == Tarray)
          fprintf(fout, "!soap_in_%s(soap, tag, NULL, \"%s\")", c_ident(n), xsi_type(n));
        else
          fprintf(fout, "!soap_in_%s(soap, tag, NULL, \"%s\")", c_ident(n), xsi_type(n));
        fprintf(fout, ")\n\t\t\t\tbreak;\n\t\t}\n\t\telse\n\t\t{\t");
        if (is_XML(n) && is_string(n))
          fprintf(fout, "if (!soap_inliteral(soap, tag, n))");
        else if (is_XML(n) && is_wstring(n))
          fprintf(fout, "if (!soap_inwliteral(soap, tag, n))");
        else if (n->type == Tarray)
          fprintf(fout, "if (!soap_in_%s(soap, tag, n, \"%s\"))", c_ident(n), xsi_type(n));
        else
          fprintf(fout, "if (!soap_in_%s(soap, tag, n, \"%s\"))", c_ident(n), xsi_type(n));
        fprintf(fout, "\n\t\t\t{\ta->pop_back();\n\t\t\t\tbreak;\n\t\t\t}\n\t\t}");
        if (!is_primitive(n) && n->type != Tpointer)
          fprintf(fout, "\n\t\tsoap_end_shaky(soap, soap_shaky);");
        fprintf(fout, "\n\t\tif (!tag || *tag == '-')\n\t\t\treturn a;\n\t}\n\tif (soap_flag && (soap->error == SOAP_TAG_MISMATCH || soap->error == SOAP_NO_TAG))\n\t{\tsoap->error = SOAP_OK;\n\t\treturn a;\n\t}\n\treturn NULL;\n}");
      }
      else
      {
        fprintf(fout, "\n\tshort soap_flag;");
        fprintf(fout, "\n\tfor (soap_flag = 0;; soap_flag = 1)\n\t{");
        fprintf(fout, "\n\t\tif (tag && *tag != '-')\n\t\t{\tif (soap_element_begin_in(soap, tag, 1, NULL))\n\t\t\t\tbreak;\n\t\t\tsoap_revert(soap);\n\t\t}");
        fprintf(fout, "\n\t\tif (!a && !(a = soap_new_%s(soap)))\n\t\t\treturn NULL;", c_ident(typ));
        fprintf(fout, "\n\t\telse if (static_cast<size_t>(a->size()) > soap->maxoccurs)\n\t\t{\tsoap->error = SOAP_OCCURS;\n\t\t\treturn NULL;\n\t\t}");
        fprintf(fout, "\n\t\t%s;\n\t\t", c_type_id(n, "n"));
        if (n->type == Tpointer)
          fprintf(fout, "n = NULL;");
        else if (n->type == Tarray)
          fprintf(fout, "soap_default_%s(soap, &n);", c_ident(n));
        else if (n->type == Tclass && !is_external(n) && !is_volatile(n) && !is_typedef(n))
          fprintf(fout, "n.soap_default(soap);");
        else if (n->type != Tfun && !is_void(n) && !is_XML(n) && !is_smart(n))
          fprintf(fout, "soap_default_%s(soap, &n);", c_ident(n));
        if (!is_primitive(n) && n->type != Tpointer)
          fprintf(fout, "\n\t\tshort soap_shaky = soap_begin_shaky(soap);");
        fprintf(fout, "\n\t\tif (tag && *tag != '-' && (*soap->id || *soap->href == '#'))\n\t\t{\tif (");
        fprintf(fout, "!soap_id_forward(soap, *soap->id?soap->id:soap->href, a, static_cast<size_t>(a->size()), %s, %s, sizeof(%s), %d, %s_finsert, %s_fbase))\n\t\t\t\tbreak;\n\t\t\tif (", soap_type(reftype(n)), soap_type(typ), c_type(reftype(n)), reflevel(n), prefix, prefix);
        if (is_XML(n) && is_string(n))
          fprintf(fout, "!soap_inliteral(soap, tag, NULL)");
        else if (is_XML(n) && is_wstring(n))
          fprintf(fout, "!soap_inwliteral(soap, tag, NULL)");
        else if (n->type == Tarray)
          fprintf(fout, "!soap_in_%s(soap, tag, NULL, \"%s\")", c_ident(n), xsi_type(n));
        else
          fprintf(fout, "!soap_in_%s(soap, tag, NULL, \"%s\")", c_ident(n), xsi_type(n));
        fprintf(fout, ")\n\t\t\t\tbreak;\n\t\t}\n\t\telse\n\t\t{\t");
        if (is_XML(n) && is_string(n))
          fprintf(fout, "if (!soap_inliteral(soap, tag, &n))");
        else if (is_XML(n) && is_wstring(n))
          fprintf(fout, "if (!soap_inwliteral(soap, tag, &n))");
        else if (n->type == Tarray)
          fprintf(fout, "if (!soap_in_%s(soap, tag, &n, \"%s\"))", c_ident(n), xsi_type(n));
        else
          fprintf(fout, "if (!soap_in_%s(soap, tag, &n, \"%s\"))", c_ident(n), xsi_type(n));
        fprintf(fout, "\n\t\t\t\tbreak;\n\t\t}");
        if (!is_primitive(n) && n->type != Tpointer)
          fprintf(fout, "\n\t\tsoap_end_shaky(soap, soap_shaky);");
        if (!(!strcmp(typ->id->name, "std::list") || !strcmp(typ->id->name, "std::deque") || !strcmp(typ->id->name, "std::set")) && !is_primitive(n) && n->type != Tpointer && !is_smart(n))
        {
          fprintf(fout, "\n\t\tif (a->size())\n\t\t{\tconst void *p = &*a->begin();\n\t\t\tsoap_update_pointers(soap, (const char*)&(*a->insert(a->end(), n)), (const char*)&n, sizeof(%s));", c_type(n));
          fprintf(fout, "\n\t\t\tif (p != &*a->begin())\n\t\t\t{\tDBGLOG(TEST, SOAP_MESSAGE(fdebug, \"Container capacity increased: updating pointers\\n\"));");
          fprintf(fout, "\n\t\t\t\tsoap_update_pointers(soap, (const char*)&*a->begin(), (const char*)p, (a->size() - 1) * sizeof(%s));\n\t\t\t}", c_type(n));
          fprintf(fout, "\n\t\t}\n\t\telse\n\t\t{\tsoap_update_pointers(soap, (const char*)&(*a->insert(a->end(), n)), (const char*)&n, sizeof(%s));", c_type(n));
          fprintf(fout, "\n\t\t}");
        }
        else
        {
          if (is_primitive(n) || n->type == Tpointer || is_smart(n))
            fprintf(fout, "\n\t\ta->insert(a->end(), n);");
          else
            fprintf(fout, "\n\t\tsoap_update_pointers(soap, (const char*)&(*a->insert(a->end(), n)), (const char*)&n, sizeof(%s));", c_type(n));
        }
        fprintf(fout, "\n\t\tif (!tag || *tag == '-')\n\t\t\treturn a;\n\t}\n\tif (soap_flag && (soap->error == SOAP_TAG_MISMATCH || soap->error == SOAP_NO_TAG))\n\t{\tsoap->error = SOAP_OK;\n\t\treturn a;\n\t}\n\treturn NULL;\n}");
      }
      break;
    default:
      break;
  }
  fflush(fout);
}

void
soap_in_Darray(Tnode *typ)
{
  int i, j, d;
  Entry *p;
  Table *t;
  const char *nsa = ns_qualifiedAttribute(typ);

  p = is_dynamic_array(typ);
  d = get_Darraydims(typ);

  fprintf(fhead, "\nSOAP_FMAC3 %s SOAP_FMAC4 soap_in_%s(struct soap*, const char*, %s, const char*);", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*"));

  if (is_external(typ))
    return;

  if (is_binary(typ))
  {
    fprintf(fhead, "\nSOAP_FMAC3S int SOAP_FMAC4S soap_s2%s(struct soap*, const char*, %s);", c_ident(typ), c_type_id(typ, "*"));
    fprintf(fout, "\n\nSOAP_FMAC3S int SOAP_FMAC4S soap_s2%s(struct soap *soap, const char *s, %s)\n{", c_ident(typ), c_type_id(typ, "*a"));
    if (is_hexBinary(typ))
      fprintf(fout, "\n\ta->__ptr = (unsigned char*)soap_hex2s(soap, s, NULL, 0, &a->__size);");
    else
      fprintf(fout, "\n\ta->__ptr = (unsigned char*)soap_base642s(soap, s, NULL, 0, &a->__size);");
    fprintf(fout, "\n\tif (!a->__ptr)\n\t\treturn soap->error;");
    if (typ->hasmin)
    {
      long min = minlen(typ);
      if (min > 0)
        fprintf(fout, "\n\tif (a->__size < %ld)\n\t\treturn soap->error = SOAP_LENGTH;", min);
    }
    if (typ->hasmax)
    {
      long max = maxlen(typ);
      if (max >= 0)
        fprintf(fout, "\n\tif (a->__size > %ld)\n\t\treturn soap->error = SOAP_LENGTH;", max);
    }
    fprintf(fout, "\n\treturn SOAP_OK;\n}");
  }

  if (typ->type == Tclass && !is_volatile(typ) && !is_typedef(typ))
  {
    fprintf(fout, "\n\nvoid *%s::soap_in(struct soap *soap, const char *tag, const char *type)", c_type(typ));
    fprintf(fout, "\n{\n\treturn soap_in_%s(soap, tag, this, type);\n}", c_ident(typ));
  }
  fflush(fout);
  fprintf(fout, "\n\nSOAP_FMAC3 %s SOAP_FMAC4 soap_in_%s(struct soap *soap, const char *tag, %s, const char *type)", c_type_id(typ, "*"), c_ident(typ), c_type_id(typ, "*a"));
  if (is_binary(typ))
    fprintf(fout, "\n{");
  else if (d)
    fprintf(fout, "\n{\n\tsize_t i, n;\n\tint j;\n\t%s;", c_type_id(p->info.typ, "p"));
  else
    fprintf(fout, "\n{\n\tint i, j;\n\t%s;", c_type_id(p->info.typ, "p"));
  fprintf(fout, "\n\tif (soap_element_begin_in(soap, tag, 1, NULL))\n\t\treturn NULL;");
  if (is_hexBinary(typ))
    fprintf(fout, "\n\tif (*soap->type && soap_match_tag(soap, soap->type, type) && soap_match_tag(soap, soap->type, \":hexBinary\"))");
  else if (is_binary(typ))
    fprintf(fout, "\n\tif (*soap->type && soap_match_tag(soap, soap->type, type) && soap_match_tag(soap, soap->type, \":base64Binary\") && soap_match_tag(soap, soap->type, \":base64\"))");
  else if (has_ns(typ) || is_untyped(typ))
    fprintf(fout, "\n\tif (*soap->type && soap_match_array(soap, \"%s\") && soap_match_tag(soap, soap->type, type))", xsi_type((Tnode*)p->info.typ->ref));
  else
    fprintf(fout, "\n\tif (soap_match_array(soap, type))");
  fprintf(fout, "\n\t{\tsoap->error = SOAP_TYPE;\n\t\treturn NULL;\n\t}");
  if (typ->type == Tclass)
  {
    fprintf(fout, "\n\ta = (%s*)soap_id_enter(soap, soap->id, a, %s, sizeof(%s), soap->type, soap->arrayType, %s_instantiate, %s_fbase);", c_type(typ), soap_type(typ), c_type(typ), prefix, prefix);
    fprintf(fout, "\n\tif (!a)\n\t\treturn NULL;");
    fprintf(fout, "\n\ta->soap_default(soap);");
    for (t = (Table*)typ->ref; t; t = t->prev)
    {
      for (p = t->list; p; p = p->next)
        if (p->info.sto & Sattribute)
          soap_attr_value(p, ptr_cast(t, "a"), ident(p->sym->name), ns_add(p, nsa));
    }
  }
  else
  {
    fprintf(fout, "\n\ta = (%s*)soap_id_enter(soap, soap->id, a, %s, sizeof(%s), NULL, NULL, NULL, NULL);", c_type(typ), soap_type(typ), c_type(typ));
    fprintf(fout, "\n\tif (!a)\n\t\treturn NULL;");
    fprintf(fout, "\n\tsoap_default_%s(soap, a);", c_ident(typ));
    for (t = (Table*)typ->ref; t; t = t->prev)
    {
      for (p = t->list; p; p = p->next)
        if (p->info.sto & Sattribute)
          soap_attr_value(p, "a", ident(p->sym->name), ns_add(p, nsa));
    }
  }
  if (is_attachment(typ))
    fprintf(fout, "\n\tif (soap->body && !*soap->href)\n\t{");
  else
    fprintf(fout, "\n\tif (soap->body && *soap->href != '#')\n\t{");
  p = is_dynamic_array(typ);
  if (is_binary(typ))
  {
    if (is_hexBinary(typ))
      fprintf(fout, "\n\t\ta->__ptr = soap_gethex(soap, &a->__size);");
    else
    {
      fprintf(fout, "\n\t\ta->__ptr = soap_getbase64(soap, &a->__size, 0);");
      if (is_attachment(typ))
        fprintf(fout, "\n#ifndef WITH_LEANER\n\t\tif (soap_xop_forward(soap, &a->__ptr, &a->__size, &a->id, &a->type, &a->options))\n\t\t\treturn NULL;\n#endif");
    }
    fprintf(fout, "\n\t\tif ((!a->__ptr && soap->error) || soap_element_end_in(soap, tag))\n\t\t\treturn NULL;");
    if (typ->hasmin)
    {
      long min = minlen(typ);
      if (min > 0)
        fprintf(fout, "\n\t\tif (a->__size < %ld)\n\t\t{\tsoap->error = SOAP_LENGTH;\n\t\t\treturn NULL;\n\t\t}", min);
    }
    if (typ->hasmax)
    {
      long max = maxlen(typ);
      if (max >= 0)
        fprintf(fout, "\n\t\tif (a->__size > %ld)\n\t\t{\tsoap->error = SOAP_LENGTH;\n\t\t\treturn NULL;\n\t\t}", max);
    }
  }
  else
  {
    fprintf(fout, "\n\t\tif (*soap->arraySize)\n\t\t{");
    if (d)
    {
      fprintf(fout, "\n\t\t\tn = soap_getsizes(soap->arraySize, a->__size, %d);", d);
      if (has_offset(typ))
        fprintf(fout, "\n\t\t\tn -= j = soap_getoffsets(soap->arrayOffset, a->__size, a->__offset, %d);", d);
      else
        fprintf(fout, "\n\t\t\tn -= j = soap_getoffsets(soap->arrayOffset, a->__size, NULL, %d);", d);
      if (p->info.minOccurs > 0)
        fprintf(fout, "\n\t\t\tif (%sn < " SOAP_LONG_FORMAT ")\n\t\t\t{\tsoap->error = SOAP_OCCURS;\n\t\t\t\treturn NULL;\n\t\t\t}", strict_check(), p->info.minOccurs);
      if (p->info.maxOccurs > 1)
        fprintf(fout, "\n\t\t\t\tif (%sn > " SOAP_LONG_FORMAT ")\n\t\t\t{\tsoap->error = SOAP_OCCURS;\n\t\t\t\treturn NULL;\n\t\t\t}", strict_check(), p->info.maxOccurs);
      else
        fprintf(fout, "\n\t\t\tif (n > soap->maxoccurs)\n\t\t\t{\tsoap->error = SOAP_OCCURS;\n\t\t\t\treturn NULL;\n\t\t\t}");
      if (((Tnode*)p->info.typ->ref)->type == Tclass
          || (((Tnode*)p->info.typ->ref)->type == Tstruct && !cflag))
      {
        fprintf(fout, "\n\t\t\ta->%s = soap_new_%s(soap, n);", ident(p->sym->name), c_ident((Tnode*)p->info.typ->ref));
        if (!is_external((Tnode*)p->info.typ->ref) && !is_volatile((Tnode*)p->info.typ->ref) && !is_typedef((Tnode*)p->info.typ->ref) && ((Tnode*)p->info.typ->ref)->type == Tclass)
          fprintf(fout, "\n\t\t\tfor (i = 0; i < n; i++)\n\t\t\t\t(a->%s+i)->%s::soap_default(soap);", ident(p->sym->name), c_type((Tnode*)p->info.typ->ref));
        else if (((Tnode*)p->info.typ->ref)->type == Tpointer)
          fprintf(fout, "\n\t\t\tfor (i = 0; i < n; i++)\n\t\t\t\tsoap_default_%s(soap, a->%s+i);", c_ident((Tnode*)p->info.typ->ref), ident(p->sym->name));
      }
      else if (has_class((Tnode*)p->info.typ->ref))
      {
        fprintf(fout, "\n\t\t\ta->%s = soap_new_%s(soap, n);", ident(p->sym->name), c_ident((Tnode*)p->info.typ->ref));
        fprintf(fout, "\n\t\t\tfor (i = 0; i < n; i++)\n\t\t\t\tsoap_default_%s(soap, a->%s+i);", c_ident((Tnode*)p->info.typ->ref), ident(p->sym->name));
      }
      else
      {
        fprintf(fout, "\n\t\t\ta->%s = (%s)soap_malloc(soap, n * sizeof(%s));", ident(p->sym->name), c_type_id((Tnode*)p->info.typ->ref, "*"),  c_type((Tnode*)p->info.typ->ref));
        if (((Tnode*)p->info.typ->ref)->type == Tpointer)
          fprintf(fout, "\n\t\t\tfor (i = 0; i < n; i++)\n\t\t\t\ta->%s[i] = NULL;", ident(p->sym->name));
        else if (!is_XML((Tnode*)p->info.typ->ref))
          fprintf(fout, "\n\t\t\tfor (i = 0; i < n; i++)\n\t\t\t\tsoap_default_%s(soap, a->%s+i);", c_ident((Tnode*)p->info.typ->ref), ident(p->sym->name));
      }
      fprintf(fout, "\n\t\t\tfor (i = 0; i < n; i++)");
      fprintf(fout, "\n\t\t\t{\tsoap_peek_element(soap);\n\t\t\t\tif (soap->position == %d)", d);
      fprintf(fout, "\n\t\t\t\t{\ti = (size_t)(");
      for (i = 0; i < d; i++)
      {
        fprintf(fout, "soap->positions[%d]", i);
        for (j = 1; j < d-i; j++)
          fprintf(fout, "*a->__size[%d]", j);
        if (i < d-1)
          fprintf(fout, "+");
      }
      fprintf(fout, "-j);");
      fprintf(fout, "\n\t\t\t\t\tif (i >= n)\n\t\t\t\t\t{\tsoap->error = SOAP_IOB;\n\t\t\t\t\t\treturn NULL;\n\t\t\t\t\t}\n\t\t\t\t}");
      fprintf(fout, "\n\t\t\t\tif (!soap_in_%s(soap, NULL, a->%s + i, \"%s\"))", c_ident((Tnode*)p->info.typ->ref), ident(p->sym->name), xsi_type((Tnode*)p->info.typ->ref));
      fprintf(fout, "\n\t\t\t\t{\tif (soap->error != SOAP_NO_TAG)\n\t\t\t\t\t\treturn NULL;");
      fprintf(fout, "\n\t\t\t\t\tsoap->error = SOAP_OK;");
      fprintf(fout, "\n\t\t\t\t\tbreak;");
      fprintf(fout, "\n\t\t\t\t}");
    }
    else
    {
      fprintf(fout, "\n\t\t\tsoap_getsizes(soap->arraySize, &a->__size, 1);");
      if (has_offset(typ))
        fprintf(fout, "\n\t\t\ta->__size -= j = soap_getoffsets(soap->arrayOffset, &a->__size, &a->__offset, 1);");
      else
        fprintf(fout, "\n\t\t\ta->__size -= j = soap_getoffsets(soap->arrayOffset, &a->__size, NULL, 1);");
      if (p->info.minOccurs > 0)
        fprintf(fout, "\n\t\t\tif (%sa->__size < " SOAP_LONG_FORMAT ")\n\t\t\t{\tsoap->error = SOAP_OCCURS;\n\t\t\t\treturn NULL;\n\t\t\t}", strict_check(), p->info.minOccurs);
      if (p->info.maxOccurs > 1)
        fprintf(fout, "\n\t\t\tif (%sa->__size > " SOAP_LONG_FORMAT ")\n\t\t\t{\tsoap->error = SOAP_OCCURS;\n\t\t\t\treturn NULL;\n\t\t\t}", strict_check(), p->info.maxOccurs);
      else
        fprintf(fout, "\n\t\t\tif ((size_t)a->__size > soap->maxoccurs)\n\t\t\t{\tsoap->error = SOAP_OCCURS;\n\t\t\t\treturn NULL;\n\t\t\t}");
      if (((Tnode*)p->info.typ->ref)->type == Tclass
          || (((Tnode*)p->info.typ->ref)->type == Tstruct && !cflag))
      {
        fprintf(fout, "\n\t\t\ta->%s = soap_new_%s(soap, a->__size);", ident(p->sym->name), c_ident((Tnode*)p->info.typ->ref));
        if (!is_external((Tnode*)p->info.typ->ref) && !is_volatile((Tnode*)p->info.typ->ref) && !is_typedef((Tnode*)p->info.typ->ref) && ((Tnode*)p->info.typ->ref)->type == Tclass)
          fprintf(fout, "\n\t\t\tfor (i = 0; i < a->__size; i++)\n\t\t\t\t(a->%s+i)->%s::soap_default(soap);", ident(p->sym->name), c_type((Tnode*)p->info.typ->ref));
        else
          fprintf(fout, "\n\t\t\tfor (i = 0; i < a->__size; i++)\n\t\t\t\tsoap_default_%s(soap, a->%s+i);", c_ident((Tnode*)p->info.typ->ref), ident(p->sym->name));
      }
      else if (has_class((Tnode*)p->info.typ->ref))
      {
        fprintf(fout, "\n\t\t\ta->%s = soap_new_%s(soap, a->__size);", ident(p->sym->name), c_ident((Tnode*)p->info.typ->ref));
        fprintf(fout, "\n\t\t\tfor (i = 0; i < a->__size; i++)\n\t\t\t\tsoap_default_%s(soap, a->%s+i);", c_ident((Tnode*)p->info.typ->ref), ident(p->sym->name));
      }
      else
      {
        fprintf(fout, "\n\t\t\ta->%s = (%s)soap_malloc(soap, sizeof(%s) * a->__size);", ident(p->sym->name), c_type_id((Tnode*)p->info.typ->ref, "*"),  c_type((Tnode*)p->info.typ->ref));
        if (((Tnode*)p->info.typ->ref)->type == Tpointer)
          fprintf(fout, "\n\t\t\tfor (i = 0; i < a->__size; i++)\n\t\t\t\ta->%s[i] = NULL;", ident(p->sym->name));
        else if (!is_XML((Tnode*)p->info.typ->ref))
          fprintf(fout, "\n\t\t\tfor (i = 0; i < a->__size; i++)\n\t\t\t\tsoap_default_%s(soap, a->%s+i);", c_ident((Tnode*)p->info.typ->ref), ident(p->sym->name));
      }
      fprintf(fout, "\n\t\t\tfor (i = 0; i < a->__size; i++)");
      fprintf(fout, "\n\t\t\t{\tsoap_peek_element(soap);\n\t\t\t\tif (soap->position)\n\t\t\t\t{\ti = soap->positions[0]-j;\n\t\t\t\t\tif (i < 0 || i >= a->__size)\n\t\t\t\t\t{\tsoap->error = SOAP_IOB;\n\t\t\t\t\t\treturn NULL;\n\t\t\t\t\t}\n\t\t\t\t}");
      if (is_XML((Tnode*)p->info.typ->ref) && is_string((Tnode*)p->info.typ->ref))
        fprintf(fout, "\n\t\t\t\tif (!soap_inliteral(soap, NULL, (char**)(a->%s + i)))", ident(p->sym->name));
      else if (is_XML((Tnode*)p->info.typ->ref) && is_wstring((Tnode*)p->info.typ->ref))
        fprintf(fout, "\n\t\t\t\tif (!soap_inwliteral(soap, NULL, (wchar_t**)(a->%s + i)))", ident(p->sym->name));
      else if (is_string((Tnode*)p->info.typ->ref))
        fprintf(fout, "\n\t\t\t\tif (!soap_in_%s(soap, NULL, (char**)(a->%s + i), \"%s\"))", c_ident((Tnode*)p->info.typ->ref), ident(p->sym->name), xsi_type((Tnode*)p->info.typ->ref));
      else if (is_wstring((Tnode*)p->info.typ->ref))
        fprintf(fout, "\n\t\t\t\tif (!soap_in_%s(soap, NULL, (wchar_t**)(a->%s + i), \"%s\"))", c_ident((Tnode*)p->info.typ->ref), ident(p->sym->name), xsi_type((Tnode*)p->info.typ->ref));
      else
        fprintf(fout, "\n\t\t\t\tif (!soap_in_%s(soap, NULL, a->%s + i, \"%s\"))", c_ident((Tnode*)p->info.typ->ref), ident(p->sym->name), xsi_type((Tnode*)p->info.typ->ref));
      fprintf(fout, "\n\t\t\t\t{\tif (soap->error != SOAP_NO_TAG)\n\t\t\t\t\t\treturn NULL;");
      fprintf(fout, "\n\t\t\t\t\tsoap->error = SOAP_OK;");
      fprintf(fout, "\n\t\t\t\t\tbreak;");
      fprintf(fout, "\n\t\t\t\t}");
    }
    fprintf(fout, "\n\t\t\t}\n\t\t}\n\t\telse");
    fprintf(fout, "\n\t\t{\tif (soap_alloc_block(soap) == NULL)\n\t\t\t\treturn NULL;");
    if (d)
    {
      for (i = 1; i < d; i++)
        fprintf(fout, "\n\t\t\ta->__size[%d] = 1;", i);
      fprintf(fout, "\n\t\t\tfor (a->__size[0] = 0; ; a->__size[0]++)\n\t\t\t{");
    }
    else
    {
      fprintf(fout, "\n\t\t\tfor (a->__size = 0; ; a->__size++)\n\t\t\t{");
    }
    if (((Tnode*)p->info.typ->ref)->type == Tclass
     || ((Tnode*)p->info.typ->ref)->type == Ttemplate
     || has_class((Tnode*)p->info.typ->ref)
     || (!cflag && ((Tnode*)p->info.typ->ref)->type == Tstruct))
      fprintf(fout, "\tp = soap_block<%s>::push(soap, NULL);\n\t\t\t\tif (!p)\n\t\t\t\t\tbreak;", c_type((Tnode*)p->info.typ->ref));
    else
      fprintf(fout, "\tp = (%s)soap_push_block(soap, NULL, sizeof(%s));\n\t\t\t\tif (!p)\n\t\t\t\t\tbreak;", c_type(p->info.typ), c_type((Tnode*)p->info.typ->ref));
    if (((Tnode*)p->info.typ->ref)->type == Tclass && !is_external((Tnode*)p->info.typ->ref) && !is_volatile((Tnode*)p->info.typ->ref) && !is_typedef((Tnode*)p->info.typ->ref))
      fprintf(fout, "\n\t\t\t\tp->soap_default(soap);");
    else if (((Tnode*)p->info.typ->ref)->type == Tpointer)
      fprintf(fout, "\n\t\t\t\t*p = NULL;");
    else if (!is_XML((Tnode*)p->info.typ->ref))
      fprintf(fout, "\n\t\t\t\tsoap_default_%s(soap, p);", c_ident((Tnode*)p->info.typ->ref));
    if (is_XML((Tnode*)p->info.typ->ref) && is_string((Tnode*)p->info.typ->ref))
      fprintf(fout, "\n\t\t\t\tif (!soap_inliteral(soap, NULL, p))");
    else if (is_XML((Tnode*)p->info.typ->ref) && is_wstring((Tnode*)p->info.typ->ref))
      fprintf(fout, "\n\t\t\t\tif (!soap_inwliteral(soap, NULL, p))");
    else
      fprintf(fout, "\n\t\t\t\tif (!soap_in_%s(soap, NULL, p, \"%s\"))", c_ident((Tnode*)p->info.typ->ref), xsi_type((Tnode*)p->info.typ->ref));
    fprintf(fout, "\n\t\t\t\t{\tif (soap->error == SOAP_NO_TAG)");
    fprintf(fout, "\n\t\t\t\t\t\tsoap->error = SOAP_OK;");
    if (((Tnode*)p->info.typ->ref)->type == Tclass
     || ((Tnode*)p->info.typ->ref)->type == Ttemplate
     || has_class((Tnode*)p->info.typ->ref)
     || (!cflag && ((Tnode*)p->info.typ->ref)->type == Tstruct))
      fprintf(fout, "\n\t\t\t\t\tsoap_block<%s>::pop(soap, NULL);", c_type((Tnode*)p->info.typ->ref));
    else
      fprintf(fout, "\n\t\t\t\t\tsoap_pop_block(soap, NULL);");
    fprintf(fout, "\n\t\t\t\t\tbreak;");
    fprintf(fout, "\n\t\t\t\t}");
    if (d)
    {
      if (p->info.maxOccurs > 1)
        fprintf(fout, "\n\t\t\t\tif (%sa->__size[0] > " SOAP_LONG_FORMAT ")", strict_check(), p->info.maxOccurs);
      else
        fprintf(fout, "\n\t\t\t\tif ((size_t)a->__size[0] > soap->maxoccurs)");
    }
    else
    {
      if (p->info.maxOccurs > 1)
        fprintf(fout, "\n\t\t\t\tif (%sa->__size > " SOAP_LONG_FORMAT ")", strict_check(), p->info.maxOccurs);
      else
        fprintf(fout, "\n\t\t\t\tif ((size_t)a->__size > soap->maxoccurs)");
    }
    fprintf(fout, "\n\t\t\t\t{\tsoap->error = SOAP_OCCURS;\n\t\t\t\t\treturn NULL;\n\t\t\t\t}");
    fprintf(fout, "\n\t\t\t}");
    if (p->info.minOccurs > 0)
    {
      if (d)
        fprintf(fout, "\n\t\t\tif (%sa->__size[0] < " SOAP_LONG_FORMAT ")\n\t\t\t{\tsoap->error = SOAP_OCCURS;\n\t\t\t\treturn NULL;\n\t\t\t}", strict_check(), p->info.minOccurs);
      else
        fprintf(fout, "\n\t\t\tif (%sa->__size < " SOAP_LONG_FORMAT ")\n\t\t\t{\tsoap->error = SOAP_OCCURS;\n\t\t\t\treturn NULL;\n\t\t\t}", strict_check(), p->info.minOccurs);
    }
    if (((Tnode*)p->info.typ->ref)->type == Tclass
     || ((Tnode*)p->info.typ->ref)->type == Ttemplate
     || has_class((Tnode*)p->info.typ->ref)
     || (!cflag && ((Tnode*)p->info.typ->ref)->type == Tstruct))
    {
      fprintf(fout, "\n\t\t\tif (soap->blist->size)\n\t\t\t\ta->%s = soap_new_%s(soap, soap->blist->size/sizeof(%s));\n\t\t\telse\n\t\t\t\ta->%s = NULL;", ident(p->sym->name), c_ident((Tnode*)p->info.typ->ref), c_type((Tnode*)p->info.typ->ref), ident(p->sym->name));
      fprintf(fout, "\n\t\t\tif (a->%s)\n\t\t\t\tsoap_block<%s>::save(soap, NULL, a->%s);\n\t\t\telse\n\t\t\t\tsoap_block<%s>::end(soap, NULL);", ident(p->sym->name), c_type((Tnode*)p->info.typ->ref), ident(p->sym->name), c_type((Tnode*)p->info.typ->ref));
    }
    else
      fprintf(fout, "\n\t\t\ta->%s = (%s)soap_save_block(soap, NULL, NULL, 1);", ident(p->sym->name), c_type(p->info.typ));
    fprintf(fout, "\n\t\t}");
    fprintf(fout, "\n\t\tif (soap->error || soap_element_end_in(soap, tag))\n\t\t\treturn NULL;");
  }
  if (has_getter(typ))
    fprintf(fout, "\n\t\tif (a->get(soap))\n\t\t\treturn NULL;");
  fprintf(fout, "\n\t}\n\telse\n\t{\t");
  if (is_attachment(typ))
    fprintf(fout, "\n#ifndef WITH_LEANER\n\t\tif (*soap->href != '#')\n\t\t{\tif (soap_attachment_forward(soap, &a->__ptr, &a->__size, &a->id, &a->type, &a->options))\n\t\t\t\treturn NULL;\n\t\t}\n\t\telse\n#endif\n\t\t\t");
  if (typ->type == Tclass || (!cflag && typ->type == Tstruct))
    fprintf(fout, "a = (%s)soap_id_forward(soap, soap->href, (void*)a, 0, %s, %s, sizeof(%s), 0, %s_finsert, %s_fbase);", c_type_id(typ, "*"), soap_type(typ), soap_type(typ), c_type(typ), prefix, prefix);
  else
    fprintf(fout, "a = (%s)soap_id_forward(soap, soap->href, (void*)a, 0, %s, %s, sizeof(%s), 0, NULL, NULL);", c_type_id(typ, "*"), soap_type(typ), soap_type(typ), c_type(typ));
  fprintf(fout, "\n\t\tif (soap->body && soap_element_end_in(soap, tag))\n\t\t\treturn NULL;");
  fprintf(fout, "\n\t}");
  fprintf(fout, "\n\treturn a;\n}");
}

const char *
cstring(const char *s, int q)
{
  size_t n;
  char *t;
  const char *r;
  for (n = 0, r = s; *r; n++, r++)
  {
    if (*r == '"' || *r == '\\')
      n++;
    else if (*r < 32)
      n += 3;
  }
  r = t = (char*)emalloc(n + 2*q + 1);
  if (q)
    *t++ = '"';
  for (; *s; s++)
  {
    if (*s == '"' || *s == '\\')
    {
      *t++ = '\\';
      *t++ = *s;
    }
    else if (*s < 32)
    {
      sprintf(t, "\\%03o", (unsigned int)(unsigned char)*s);
      t += 4;
    }
    else
      *t++ = *s;
  }
  if (q)
    *t++ = '"';
  *t = '\0';
  return r;
}

const char *
xstring(const char *s)
{
  size_t n;
  char *t;
  const char *r;
  for (n = 0, r = s; *r; n++, r++)
  {
    if (*r < 32 || *r >= 127)
      n += 4;
    else if (*r == '<' || *r == '>')
      n += 3;
    else if (*r == '&')
      n += 4;
    else if (*r == '"')
      n += 5;
    else if (*r == '\\')
      n += 1;
  }
  r = t = (char*)emalloc(n + 1);
  for (; *s; s++)
  {
    if (*s < 32 || *s >= 127)
    {
      sprintf(t, "&#%.2x;", (unsigned char)*s);
      t += 5;
    }
    else if (*s == '<')
    {
      strcpy(t, "&lt;");
      t += 4;
    }
    else if (*s == '>')
    {
      strcpy(t, "&gt;");
      t += 4;
    }
    else if (*s == '&')
    {
      strcpy(t, "&amp;");
      t += 5;
    }
    else if (*s == '"')
    {
      strcpy(t, "&quot;");
      t += 6;
    }
    else if (*s == '\\')
    {
      strcpy(t, "\\\\");
      t += 2;
    }
    else
      *t++ = *s;
  }
  *t = '\0';
  return r;
}

void set_namespace(const char *id)
{
  namespaceid = id;
  if (id)
  {
    const char *s = id;
    int h = 0;
    while (*s)
      h = 65599*h + *s++;
    if (h == 0)
      h = 1;
    h <<= 12; /* permits 4096 unique types */
    if (h < 0)
      h = -h;
    typeNO = h;
  }
}
