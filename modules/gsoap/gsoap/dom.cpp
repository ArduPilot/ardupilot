/*
        dom.c[pp]

        DOM API v5 gSOAP 2.8.114

        See gsoap/doc/dom/html/index.html for the new DOM API v5 documentation
        Also located in /gsoap/samples/dom/README.md

gSOAP XML Web services tools
Copyright (C) 2000-2021, Robert van Engelen, Genivia, Inc. All Rights Reserved.
This part of the software is released under ONE of the following licenses:
GPL or the gSOAP public license.
--------------------------------------------------------------------------------
gSOAP public license.

The contents of this file are subject to the gSOAP Public License Version 1.3
(the "License"); you may not use this file except in compliance with the
License. You may obtain a copy of the License at
http://www.cs.fsu.edu/~engelen/soaplicense.html
Software distributed under the License is distributed on an "AS IS" basis,
WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License
for the specific language governing rights and limitations under the License.

The Initial Developer of the Original Code is Robert A. van Engelen.
Copyright (C) 2000-2021, Robert van Engelen, Genivia Inc., All Rights Reserved.
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

/** Compatibility requirement with gSOAP engine version */
#define GSOAP_LIB_VERSION 208114

#include "stdsoap2.h"

#if GSOAP_VERSION != GSOAP_LIB_VERSION
# error "GSOAP VERSION MISMATCH IN LIBRARY: PLEASE REINSTALL PACKAGE"
#endif

#ifdef __BORLANDC__
# pragma warn -8060
#endif

SOAP_FMAC3 void SOAP_FMAC4 soap_serialize_xsd__anyType(struct soap*, const struct soap_dom_element *);
SOAP_FMAC3 void SOAP_FMAC4 soap_traverse_xsd__anyType(struct soap*, struct soap_dom_element*, const char*, soap_walker, soap_walker);
SOAP_FMAC1 void SOAP_FMAC2 soap_default_xsd__anyType(struct soap*, struct soap_dom_element *);
SOAP_FMAC3 int SOAP_FMAC4 soap_put_xsd__anyType(struct soap*, const struct soap_dom_element*, const char*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_out_xsd__anyType(struct soap*, const char*, int, const struct soap_dom_element*, const char*);
SOAP_FMAC3 struct soap_dom_element * SOAP_FMAC4 soap_get_xsd__anyType(struct soap*, struct soap_dom_element*, const char*, const char*);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_in_xsd__anyType(struct soap*, const char*, struct soap_dom_element *, const char*);

SOAP_FMAC3 void SOAP_FMAC4 soap_serialize_xsd__anyAttribute(struct soap*, const struct soap_dom_attribute*);
SOAP_FMAC3 void SOAP_FMAC4 soap_traverse_xsd__anyAttribute(struct soap*, struct soap_dom_attribute*, const char*, soap_walker, soap_walker);
SOAP_FMAC1 void SOAP_FMAC2 soap_default_xsd__anyAttribute(struct soap*, struct soap_dom_attribute*);
SOAP_FMAC3 int SOAP_FMAC4 soap_put_xsd__anyAttribute(struct soap*, const struct soap_dom_attribute*, const char*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_out_xsd__anyAttribute(struct soap*, const char*, int, const struct soap_dom_attribute*, const char*);
SOAP_FMAC3 struct soap_dom_attribute * SOAP_FMAC4 soap_get_xsd__anyAttribute(struct soap*, struct soap_dom_attribute*, const char*, const char*);
SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_in_xsd__anyAttribute(struct soap*, const char*, struct soap_dom_attribute *, const char*);

#ifdef SOAP_DOM_EXTERNAL_NAMESPACE

namespace SOAP_DOM_EXTERNAL_NAMESPACE {

#ifndef WITH_NOIDREF
SOAP_FMAC3 void SOAP_FMAC4 soap_markelement(struct soap*, const void*, int);
#endif

SOAP_FMAC3 void * SOAP_FMAC4 soap_getelement(struct soap*, const char*, int*);
SOAP_FMAC3 int SOAP_FMAC4 soap_putelement(struct soap*, const void*, const char*, int, int);
SOAP_FMAC3 void * SOAP_FMAC4 soap_dupelement(struct soap*, const void*, int);
SOAP_FMAC3 void SOAP_FMAC4 soap_delelement(const void*, int);

}

#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WITH_NOIDREF
SOAP_FMAC3 void SOAP_FMAC4 soap_markelement(struct soap*, const void*, int);
#endif

SOAP_FMAC3 void * SOAP_FMAC4 soap_getelement(struct soap*, const char*, int*);
SOAP_FMAC3 int SOAP_FMAC4 soap_putelement(struct soap*, const void*, const char*, int, int);
SOAP_FMAC3 void * SOAP_FMAC4 soap_dupelement(struct soap*, const void*, int);
SOAP_FMAC3 void SOAP_FMAC4 soap_delelement(const void*, int);

#ifdef __cplusplus
}
#endif

/** Format string for generating DOM namespace prefixes (< 16 chars total) */
#ifndef SOAP_DOMID_FORMAT
#define SOAP_DOMID_FORMAT "_%d"
#endif

static int out_element(struct soap*, const struct soap_dom_element*, const char*, const char*);
static int out_attribute(struct soap*, const char*, const char*, const char*, int);

/* namespace name (URI) lookup and store routines */

static const char *soap_push_prefix(struct soap*, const char*, size_t, const char*, int, int);
static const char *soap_prefix_of(struct soap*, const char*);
static const char *soap_ns_to_set(struct soap*, const char*);
static const char *soap_ns_to_get(struct soap*, const char*);
static const char *soap_ns_to_find(struct soap*, const char*);

static struct soap_dom_element *new_element(struct soap*);
static struct soap_dom_attribute *new_attribute(struct soap*);

static int soap_tag_match(const char*, const char*);
static int soap_patt_match(const char*, const char*);
static int soap_name_match(const char*, const char*);

/******************************************************************************\
 *
 *      DOM custom (de)serializers
 *
\******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_serialize_xsd__anyType(struct soap *soap, const struct soap_dom_element *node)
{
  if (node)
  {
    if (node->type && node->node)
    {
#ifdef SOAP_DOM_EXTERNAL_NAMESPACE
      SOAP_DOM_EXTERNAL_NAMESPACE::soap_markelement(soap, node->node, node->type);
      ::soap_markelement(soap, node->node, node->type);
#else
      soap_markelement(soap, node->node, node->type);
#endif
    }
    else
    {
      const struct soap_dom_element *elt;
      for (elt = node->elts; elt; elt = elt->next)
        soap_serialize_xsd__anyType(soap, elt);
    }
  }
}

SOAP_FMAC3
void
SOAP_FMAC4
soap_traverse_xsd__anyType(struct soap *soap, struct soap_dom_element *node, const char *s, soap_walker p, soap_walker q)
{
  (void)soap; (void)node; (void)s; (void)p; (void)q; 
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_serialize_xsd__anyAttribute(struct soap *soap, const struct soap_dom_attribute *node)
{
  (void)soap; (void)node;
}

SOAP_FMAC1
void
SOAP_FMAC2
soap_traverse_xsd__anyAttribute(struct soap *soap, struct soap_dom_attribute *node, const char *s, soap_walker p, soap_walker q)
{
  (void)soap; (void)node; (void)s; (void)p; (void)q; 
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_default_xsd__anyType(struct soap *soap, struct soap_dom_element *node)
{
  node->next = NULL;
  node->prnt = NULL;
  node->elts = NULL;
  node->atts = NULL;
  node->nstr = NULL;
  node->name = NULL;
  node->lead = NULL;
  node->text = NULL;
  node->code = NULL;
  node->tail = NULL;
  node->node = NULL;
  node->type = 0;
  node->soap = soap;
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_default_xsd__anyAttribute(struct soap *soap, struct soap_dom_attribute *node)
{
  node->next = NULL;
  node->nstr = NULL;
  node->name = NULL;
  node->text = NULL;
  node->soap = soap;
}

/******************************************************************************/

static int 
out_element(struct soap *soap, const struct soap_dom_element *node, const char *prefix, const char *name)
{
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DOM node lead '%s'\n", node->lead ? node->lead : ""));
  if (node->lead && soap_send(soap, node->lead))
    return soap->error;
  if (node->type && node->node)
  {
    char *s = NULL;
    struct Namespace *p;
    if (prefix && *prefix)
    {
      size_t l = strlen(prefix) + strlen(name);
      if (l + 2 < l || (SOAP_MAXALLOCSIZE > 0 && l + 2 > SOAP_MAXALLOCSIZE))
        return soap->error = SOAP_EOM;
      s = (char*)SOAP_MALLOC(soap, l + 2);
      if (!s)
        return soap->error = SOAP_EOM;
      (SOAP_SNPRINTF(s, l + 2, l + 1), "%s:%s", prefix, name);
    }
    if (!(soap->mode & SOAP_DOM_ASIS))
    {
      for (p = soap->local_namespaces; p && p->id; p++)
      {
        if (p->ns && soap_push_prefix(soap, p->id, strlen(p->id), p->ns, 1, 0) == NULL)
        {
          if (s)
            SOAP_FREE(soap, s);
          return soap->error;
        }
      }
    }
#ifdef SOAP_DOM_EXTERNAL_NAMESPACE
    if (SOAP_DOM_EXTERNAL_NAMESPACE::soap_putelement(soap, node->node, s ? s : name, 0, node->type)
     || ::soap_putelement(soap, node->node, s ? s : name, 0, node->type))
    {
      if (s)
        SOAP_FREE(soap, s);
      return soap->error;
    }
#else
    if (soap_putelement(soap, node->node, s ? s : name, 0, node->type))
    {
      if (s)
        SOAP_FREE(soap, s);
      return soap->error;
    }
#endif
    if (s)
      SOAP_FREE(soap, s);
  }
  else if (prefix && *prefix)
  {
    size_t l = strlen(prefix) + strlen(name);
    char *s;
    if (l + 2 < l || (SOAP_MAXALLOCSIZE > 0 && l + 2 > SOAP_MAXALLOCSIZE))
      return soap->error = SOAP_EOM;
    if (l + 1 < sizeof(soap->msgbuf))
    {
      s = soap->msgbuf;
    }
    else
    {
      s = (char*)SOAP_MALLOC(soap, l + 2);
      if (!s)
        return soap->error = SOAP_EOM;
    }
    (SOAP_SNPRINTF(s, l + 2, l + 1), "%s:%s", prefix, name);
    (void)soap_element(soap, s, 0, NULL);
    if (s != soap->msgbuf)
      SOAP_FREE(soap, s);
  }
  else if (*name != '-')
  {
    soap_mode m = soap->mode;
    if ((soap->mode & SOAP_DOM_ASIS))
      soap->mode &= ~(SOAP_XML_INDENT | SOAP_XML_DEFAULTNS);
    (void)soap_element(soap, name, 0, NULL);
    soap->mode = m;
  }
  return soap->error;
}

/******************************************************************************/

static int
out_attribute(struct soap *soap, const char *prefix, const char *name, const char *text, int isearly)
{
  int err = SOAP_OK;
  char *s;
  const char *t;
  size_t l;
  if (!text)
    text = "";
  if (!prefix || !*prefix)
  {
    if ((soap->mode & SOAP_XML_CANONICAL) && !strncmp(name, "xmlns", 5) && ((name[5] == ':') || name[5] == '\0'))
      return soap_attribute(soap, name, text);
    if (isearly)
      return soap_set_attr(soap, name, text, 2);
    return soap_attribute(soap, name, text);
  }
  t = strchr(name, ':');
  if (t)
    t++;
  else
    t = name;
  l = strlen(prefix) + strlen(t);
  if (l + 2 < l || (SOAP_MAXALLOCSIZE > 0 && l + 2 > SOAP_MAXALLOCSIZE))
    return soap->error = SOAP_EOM;
  if (l + 1 < sizeof(soap->msgbuf))
  {
    s = soap->msgbuf;
  }
  else
  {
    s = (char*)SOAP_MALLOC(soap, l + 2);
    if (!s)
      return soap->error = SOAP_EOM;
  } 
  (SOAP_SNPRINTF(s, l + 2, l + 1), "%s:%s", prefix, t);
  if (isearly)
    err = soap_set_attr(soap, s, text, 2);
  else
    err = soap_attribute(soap, s, text);
  if (s != soap->msgbuf)
    SOAP_FREE(soap, s);
  return err;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_out_xsd__anyType(struct soap *soap, const char *tag, int id, const struct soap_dom_element *node, const char *type)
{
  (void)id; (void)type;
  if (node)
  {
    const char *prefix; /* namespace prefix, if namespace is present */
    if (!(soap->mode & SOAP_DOM_ASIS) && !(soap->mode & SOAP_XML_CANONICAL))
    {
      const struct soap_dom_attribute *att;
      for (att = node->atts; att; att = att->next)
      {
        if (att->name && att->text && !strncmp(att->name, "xmlns:", 6))
        {
          if (soap_push_namespace(soap, att->name + 6, att->text) == NULL)
            return soap->error;
        }
        else if (att->name && att->text && !strcmp(att->name, "xmlns"))
        {
          if (soap_push_namespace(soap, "", att->text) == NULL)
            return soap->error;
        }
      }
    }
    if (node->name)
      tag = node->name;
    else if (!tag)
      tag = "-";
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DOM node '%s' start at level=%u\n", tag, soap->level));
    prefix = NULL;
    if (!(soap->mode & SOAP_DOM_ASIS))
    {
      const struct soap_nlist *np;
      size_t n = 0;
      prefix = strchr(tag, ':');
      if (prefix)
      {
        n = prefix - tag;
        if (n && node->nstr)
          prefix = soap_prefix_of(soap, node->nstr);
        else
          prefix = NULL;
      }
      np = soap_lookup_ns(soap, tag, n);
      if (!prefix && ((n && !np) || (node->nstr && (!np || !np->ns || strcmp(node->nstr, np->ns)))))
      {
        prefix = soap_push_prefix(soap, tag, n, node->nstr, 1, 1);
        if (!(prefix))
          return soap->error;
      }
      if (n && prefix && *prefix)
        tag += n + 1;
    }
    if (!soap->ns)
      soap->ns = 2;
    if (out_element(soap, node, prefix, tag))
      return soap->error;
    if (!node->type || !node->node)
    {
      struct soap_dom_attribute *att;
      struct soap_dom_element *elt;
      for (att = node->atts; att; att = att->next)
      {
        if (att->name)
        {
          if (!(soap->mode & SOAP_XML_CANONICAL) || strcmp(att->name, "xmlns") || (!prefix && !strchr(tag, ':')))
          {
            if (!(soap->mode & SOAP_DOM_ASIS))
            {
              const char *p = NULL;
              if (strncmp(att->name, "xml", 3))
              {
                if (att->nstr)
                  p = soap_prefix_of(soap, att->nstr);
                if (!p)
                {
                  const struct soap_nlist *np;
                  size_t n = 0;
                  p = strchr(att->name, ':');
                  if (p)
                    n = p - att->name;
                  p = NULL;
                  np = soap_lookup_ns(soap, att->name, n);
                  if ((n && !np) || (att->nstr && (!np || !np->ns || strcmp(att->nstr, np->ns))))
                  {
                    p = soap_push_prefix(soap, att->name, n, att->nstr, 0, 0);
                    if (!p)
                      return soap->error;
                  }
                }
              }
              if (out_attribute(soap, p, att->name, att->text, 0))
                return soap->error;
            }
            else if (soap_attribute(soap, att->name, att->text))
            {
              return soap->error;
            }
          }
        }
      }
      if (!node->text && !node->code && !node->tail && !node->elts && !(soap->mode & SOAP_XML_CANONICAL))
      {
        soap_mode m = soap->mode;
        soap->mode &= ~SOAP_XML_INDENT;
        if (*tag != '-' && soap_element_start_end_out(soap, tag))
          return soap->error;
        soap->mode = m;
      }
      else
      {
        if (*tag != '-' && soap_element_start_end_out(soap, NULL))
          return soap->error;
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DOM node text '%s'\n", node->text ? node->text : ""));
        if (node->text && soap_string_out(soap, node->text, 0))
          return soap->error;
        for (elt = node->elts; elt; elt = elt->next)
          if (soap_out_xsd__anyType(soap, NULL, 0, elt, NULL))
            return soap->error;
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DOM node code '%s'\n", node->code ? node->code : ""));
        if (node->code && soap_send(soap, node->code))
          return soap->error;
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DOM node tail '%s'\n", node->tail ? node->tail : ""));
        if (node->tail && soap_send(soap, node->tail))
          return soap->error;
        if (!prefix || !*prefix)
        {
          soap_mode m = soap->mode;
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DOM node '%s' end\n", tag));
          if ((soap->mode & SOAP_DOM_ASIS))
            soap->mode &= ~SOAP_XML_INDENT;
          if (soap_element_end_out(soap, tag))
            return soap->error;
          soap->mode = m;
        }
        else
        {
          int err = SOAP_OK;
          char *s;
          size_t l = strlen(prefix) + strlen(tag);
          if (l + 2 < l || (SOAP_MAXALLOCSIZE > 0 && l + 2 > SOAP_MAXALLOCSIZE))
            return soap->error = SOAP_EOM;
          if (l + 1 < sizeof(soap->msgbuf))
          {
            s = soap->msgbuf;
          }
          else
          {
            s = (char*)SOAP_MALLOC(soap, l + 2);
            if (!s)
              return soap->error = SOAP_EOM;
          }
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DOM node '%s' end\n", tag));
          (SOAP_SNPRINTF(s, l + 2, l + 1), "%s:%s", prefix, tag);
          err = soap_element_end_out(soap, s);
          if (s != soap->msgbuf)
            SOAP_FREE(soap, s);
          if (err)
            return err;
        }
      }
    }
  }
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_out_xsd__anyAttribute(struct soap *soap, const char *tag, int id, const struct soap_dom_attribute *node, const char *type)
{
  const struct soap_dom_attribute *att;
  (void)tag; (void)id; (void)type;
  if (!(soap->mode & SOAP_DOM_ASIS) && !(soap->mode & SOAP_XML_CANONICAL))
  {
    for (att = node; att; att = att->next)
    {
      if (att->name && att->text && !strncmp(att->name, "xmlns:", 6))
      {
        if (soap_push_namespace(soap, att->name + 6, att->text) == NULL)
          return soap->error;
      }
      else if (att->name && att->text && !strcmp(att->name, "xmlns"))
      {
        if (soap_push_namespace(soap, "", att->text) == NULL)
          return soap->error;
      }
    }
  }
  for (att = node; att; att = att->next)
  {
    if (att->name)
    {
      if (!(soap->mode & SOAP_DOM_ASIS))
      {
        const char *p = NULL;
        if (strncmp(att->name, "xml", 3))
        {
          if (att->nstr)
            p = soap_prefix_of(soap, att->nstr);
          if (!p)
          {
            const struct soap_nlist *np;
            size_t n = 0;
            p = strchr(att->name, ':');
            if (p)
              n = p - att->name;
            p = NULL;
            np = soap_lookup_ns(soap, att->name, n);
            if ((n && !np) || (att->nstr && (!np || !np->ns || strcmp(att->nstr, np->ns))))
            {
              p = soap_push_prefix(soap, att->name, n, att->nstr, 1, 0);
              if (!p)
                return soap->error;
            }
          }
        }
        if (out_attribute(soap, p, att->name, att->text, 1))
          return soap->error;
      }
      else if (out_attribute(soap, NULL, att->name, att->text, 1))
      {
        return soap->error;
      }
    }
  }
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_in_xsd__anyType(struct soap *soap, const char *tag, struct soap_dom_element *node, const char *type)
{
  struct soap_attribute *tp;
  struct soap_dom_attribute **att;
  soap_mode m = soap->mode;
  soap->mode |= SOAP_C_UTFSTRING;
  (void)tag; (void)type;
  if (soap_peek_element(soap))
  {
    const char *s;
    if (soap->error != SOAP_NO_TAG)
      return NULL;
    s = soap_string_in(soap, 3, -1, -1, NULL);
    if (!s || !*s)
    {
      soap->mode = m;
      return NULL;
    }
    soap->mode = m;
    if (!node)
    {
      node = new_element(soap);
      if (!node)
      {
        soap->error = SOAP_EOM;
        return NULL;
      }
    }
    node->text = s;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DOM node with cdata\n"));
    soap->error = SOAP_OK;
    soap->peeked = 0;
    return node;
  }
  if (!node)
  {
    node = new_element(soap);
    if (!node)
    {
      soap->error = SOAP_EOM;
      return NULL;
    }
  }
  else
  {
    soap_default_xsd__anyType(soap, node);
  }
  node->nstr = soap_current_namespace_tag(soap, soap->tag);
  node->name = soap_strdup(soap, soap->tag);
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DOM node element='%s' start namespace='%s'\n", node->name, node->nstr?node->nstr:"(null)"));
  if ((soap->mode & SOAP_DOM_NODE) || (!(soap->mode & SOAP_DOM_TREE) && *soap->id && (!type || strcmp(type, "xsd:anyType"))))
  {
    soap->mode = m;
#ifdef SOAP_DOM_EXTERNAL_NAMESPACE
    node->node = SOAP_DOM_EXTERNAL_NAMESPACE::soap_getelement(soap, NULL, &node->type);
    if ((!node->node || !node->type) && soap->error == SOAP_TAG_MISMATCH)
      node->node = ::soap_getelement(soap, NULL, &node->type);
#else
    node->node = soap_getelement(soap, NULL, &node->type);
#endif
    if (node->node && node->type)
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DOM node contains type %d from xsi:type or matching element tag name\n", node->type));
      return node;
    }
    if (soap->error == SOAP_TAG_MISMATCH)
      soap->error = SOAP_OK;
    else
      return NULL;
    soap->mode |= SOAP_C_UTFSTRING;
  }
  att = &node->atts;
  for (tp = soap->attributes; tp; tp = tp->next)
  {
    if (tp->visible)
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DOM node attribute='%s'\n", tp->name));
      *att = new_attribute(soap);
      if (!*att)
      {
        soap->error = SOAP_EOM;
        return NULL;
      }
      (*att)->next = NULL;
      (*att)->nstr = soap_current_namespace_att(soap, tp->name);
      (*att)->name = soap_strdup(soap, tp->name);
      if (tp->visible == 2)
        (*att)->text = soap_strdup(soap, tp->value);
      else
        (*att)->text = NULL;
      (*att)->soap = soap;
      att = &(*att)->next;
      tp->visible = 0;
    }
  }
  if (soap_element_begin_in(soap, NULL, 1, NULL))
    return NULL;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DOM node '%s' pulled (level=%u)\n", node->name, soap->level));
  if (soap->body)
  {
    struct soap_dom_element **elt;
    if (soap_peek_element(soap))
    {
      if (soap->error != SOAP_NO_TAG)
        return NULL;
      node->text = soap_string_in(soap, 3, -1, -1, NULL);
      if (!node->text)
        return NULL;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DOM node '%s' has cdata\n", node->name));
      soap->peeked = 0;
    }
    elt = &node->elts;
    soap->mode = m;
    for (;;)
    {
      *elt = soap_in_xsd__anyType(soap, NULL, NULL, NULL);
      if (!*elt)
      {
        if (soap->error && soap->error != SOAP_NO_TAG)
          return NULL;
        break;
      }
      (*elt)->prnt = node;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DOM node '%s' has subelement '%s'\n", node->name, (*elt)->name));
      if (node->text)
      {
        if (*node->text)
        {
          struct soap_dom_element *elts = new_element(soap);
          if (!elts)
          {
            soap->error = SOAP_EOM;
            return NULL;
          }
          elts->next = *elt;
          elts->prnt = node;
          elts->text = node->text;
          node->elts = elts;
          elt = &elts->next;
        }
        node->text = NULL;
      }
      elt = &(*elt)->next;
    }
    if (!node->text && !node->elts)
      node->tail = ""; /* has body so tail is non-NULL for non-empty element tag */
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DOM node '%s' end (level=%u)\n", node->name, soap->level));
    if (soap_element_end_in(soap, NULL))
      return NULL;
    if (strcmp(soap->tag, node->name))
    {
      soap->error = SOAP_SYNTAX_ERROR;
      return NULL;
    }
  }
  soap->mode = m;
  return node;
}

/******************************************************************************/

SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_in_xsd__anyAttribute(struct soap *soap, const char *tag, struct soap_dom_attribute *node, const char *type)
{
  struct soap_attribute *tp;
  struct soap_dom_attribute *tmp = node;
  struct soap_dom_attribute *att = node;
  (void)tag; (void)type;
  for (tp = soap->attributes; tp; tp = tp->next)
  {
    if (tp->visible)
    {
      if (!att)
      {
        att = new_attribute(soap);
        if (tmp)
          tmp->next = att;
        else
          node = att;
        tmp = att;
      }
      if (!att)
      {
        soap->error = SOAP_EOM;
        return NULL;
      }
      att->next = NULL;
      att->nstr = soap_current_namespace_att(soap, tp->name);
      att->name = soap_strdup(soap, tp->name);
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DOM node attribute='%s' namespace='%s'\n", att->name, att->nstr ? att->nstr : "(null)"));
      if (tp->visible == 2)
        att->text = soap_strdup(soap, tp->value);
      else
        att->text = NULL;
      att->soap = soap;
      att = NULL;
    }
  }
  return node;
}

SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_dup_xsd__anyType(struct soap *soap, struct soap_dom_element *d, const struct soap_dom_element *a)
{
  struct soap_dom_element *elt = NULL;
  if (!a)
    return NULL;
  if (!d && (d = new_element(soap)) == NULL)
    return NULL;
  d->next = NULL;
  d->nstr = soap_strdup(soap, a->nstr);
  d->name = soap_strdup(soap, a->name);
  d->lead = soap_strdup(soap, a->lead);
  d->text = soap_strdup(soap, a->text);
  d->code = soap_strdup(soap, a->code);
  d->tail = soap_strdup(soap, a->tail);
#ifdef SOAP_DOM_EXTERNAL_NAMESPACE
  if (a->node)
  {
    d->node = SOAP_DOM_EXTERNAL_NAMESPACE::soap_dupelement(soap, a->node, a->type);
    if (!d->node)
      d->node = ::soap_dupelement(soap, a->node, a->type);
  }
  else
  {
    d->node = NULL;
  }
#else
  d->node = soap_dupelement(soap, a->node, a->type);
#endif
  d->type = a->type;
  d->atts = soap_dup_xsd__anyAttribute(soap, NULL, a->atts);
  for (a = a->elts; a; a = a->next)
  {
    if (elt)
      elt = elt->next = soap_dup_xsd__anyType(soap, NULL, a);
    else
      elt = d->elts = soap_dup_xsd__anyType(soap, NULL, a);
    elt->prnt = d;
  }
  d->soap = soap;
  return d;
}

SOAP_FMAC1
void
SOAP_FMAC2
soap_del_xsd__anyType(const struct soap_dom_element *a)
{
  const struct soap_dom_element *p = NULL;
  while (a)
  {
    if (a->nstr)
      SOAP_FREE(NULL, a->nstr);
    if (a->name)
      SOAP_FREE(NULL, a->name);
    if (a->lead)
      SOAP_FREE(NULL, a->lead);
    if (a->text)
      SOAP_FREE(NULL, a->text);
    if (a->code)
      SOAP_FREE(NULL, a->code);
    if (a->tail)
      SOAP_FREE(NULL, a->tail);
#ifdef SOAP_DOM_EXTERNAL_NAMESPACE
    if (a->node)
    {
      SOAP_DOM_EXTERNAL_NAMESPACE::soap_delelement(a->node, a->type);
      ::soap_delelement(a->node, a->type);
    }
#else
    soap_delelement(a->node, a->type);
#endif
    if (a->atts)
    {
      soap_del_xsd__anyAttribute(a->atts);
      SOAP_FREE(NULL, a->atts);
    }
    if (a->elts)
    {
      soap_del_xsd__anyType(a->elts);
      SOAP_FREE(NULL, a->elts);
    }
    a = a->next;
    if (p)
      SOAP_FREE(NULL, p);
    p = a;
  }
}

SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_dup_xsd__anyAttribute(struct soap *soap, struct soap_dom_attribute *d, const struct soap_dom_attribute *a)
{
  struct soap_dom_attribute *att;
  if (!a)
    return NULL;
  if (!d && (d = new_attribute(soap)) == NULL)
    return NULL;
  att = d;
  while (a)
  {
    att->nstr = soap_strdup(soap, a->nstr);
    att->name = soap_strdup(soap, a->name);
    att->text = soap_strdup(soap, a->text);
    a = a->next;
    if (a)
    {
      att->next = new_attribute(soap);
      if (!att->next)
        break;
      att = att->next;
    }
  }
  d->soap = soap;
  return d;
}

SOAP_FMAC1
void
SOAP_FMAC2
soap_del_xsd__anyAttribute(const struct soap_dom_attribute *a)
{
  const struct soap_dom_attribute *p = NULL;
  while (a)
  {
    if (a->nstr)
      SOAP_FREE(NULL, a->nstr);
    if (a->name)
      SOAP_FREE(NULL, a->name);
    if (a->text)
      SOAP_FREE(NULL, a->text);
    a = a->next;
    if (p)
      SOAP_FREE(NULL, p);
    p = a;
  }
}

/******************************************************************************\
 *
 *      Namespace prefix lookup/store
 *
\******************************************************************************/

static const char *
soap_push_prefix(struct soap *soap, const char *id, size_t n, const char *ns, int isearly, int iselement)
{
  struct Namespace *p, *ln = soap->local_namespaces;
  struct soap_nlist *np;
  if (!n)
    id = NULL;
  if (!ns && id)
  {
    for (p = ln; p && p->id; p++)
      if (!strncmp(p->id, id, n) && !p->id[n])
        break;
    if (p && p->id)
    {
      id = p->id;
      ns = p->out;
      if (!ns)
        ns = p->ns;
    }
    if (!ns)
      return "";
  }
  else if (!id)
  {
    if (!ns)
      return "";
    if (iselement)
    {
      id = ""; /* gen xmlsn="ns" for elements with nstr but no prefix */
    }
    else
    {
      for (p = ln; p && p->id; p++)
        if (p->ns && !strcmp(p->ns, ns))
          break;
      if (p && p->id)
        id = p->id;
      else
      {
        int i = 0;
        for (np = soap->nlist; np; np = np->next)
        {
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DOM find binding %s = '%s' level = %d index = %d\n", np->id, np->ns, np->level, np->index));
          if (np->level)
            i++;
        }
        (SOAP_SNPRINTF(soap->tag, sizeof(soap->tag), sizeof(SOAP_DOMID_FORMAT) + 20), SOAP_DOMID_FORMAT, i);
        id = soap->tag;
      }
    }
  }
  else
  {
    for (np = soap->nlist; np; np = np->next)
    {
      if (!strncmp(np->id, id, n) && !np->id[n])
      {
        if (np->ns)
        {
          if (!strcmp(np->ns, ns))
            return ""; /* xmlns:id="ns" already in scope */
        }
        else
        {
          if (!strcmp(ln[np->index].ns, ns) || (ln[np->index].out && !strcmp(ln[np->index].out, ns)))
            return ""; /* xmlns:id="ns" already in scope */
        }
        break;
      }
    }
    (void)soap_strncpy(soap->tag, sizeof(soap->tag), id, n);
    id = soap->tag;
    soap->local_namespaces = NULL; /* do not permit a replacement id, when ns is in table */
  }
  /* fix advance generation of xmlns, when element (at level) is not output yet */
  if (isearly)
    soap->level++;
  np = soap_push_namespace(soap, id, ns);
  soap->local_namespaces = ln;
  if (isearly)
    soap->level--;
  if (!np)
    return NULL;
  if (!np->ns)
  {
    np->ns = ln[np->index].out;
    if (!np->ns)
      np->ns = ln[np->index].ns;
  }
  np->index = 0; /* for C14N visibly utilized mark */
  /* gen xmlns:id="ns" */
  if (*np->id)
  {
    (SOAP_SNPRINTF(soap->msgbuf, sizeof(soap->msgbuf), strlen(np->id) + 6), "xmlns:%s", np->id);
    if (out_attribute(soap, NULL, soap->msgbuf, ns, isearly))
      return NULL;
  }
  else
  {
    if (out_attribute(soap, NULL, "xmlns", ns, isearly))
      return NULL;
  }
  return np->id;
}

/******************************************************************************/

static const char *
soap_prefix_of(struct soap *soap, const char *ns)
{
  struct soap_nlist *np;
  for (np = soap->nlist; np; np = np->next)
    if (np->ns && !strcmp(np->ns, ns)) 
      return np->id;
  return NULL;
}

/******************************************************************************/

static const char *
soap_ns_to_set(struct soap *soap, const char *tag)
{
  const char *s;
  size_t n;
  const struct Namespace *p;
  if (!tag)
    return NULL;
  s = strchr(tag, ':');
  if (!s)
    return NULL;
  n = s - tag;
  if (soap)
    for (p = soap->namespaces; p && p->id; p++)
      if (!strncmp(p->id, tag, n) && !p->id[n])
        return p->ns;
  return NULL;
}

/******************************************************************************/

static const char *
soap_ns_to_get(struct soap *soap, const char *tag)
{
  const char *s;
  size_t n;
  const struct Namespace *p;
  if (!tag)
    return "";
  s = strchr(tag, ':');
  if (!s)
    return "";
  n = s - tag;
  if (soap)
    for (p = soap->namespaces; p && p->id; p++)
      if (!strncmp(p->id, tag, n) && !p->id[n])
        return p->out ? p->out : p->ns;
  return "";
}

/******************************************************************************/

static const char *
soap_ns_to_find(struct soap *soap, const char *tag)
{
  const char *s;
  size_t n;
  const struct Namespace *p;
  if (!tag || (tag[0] == '*' && !tag[1]))
    return NULL;
  s = strchr(tag, ':');
  if (!s)
    return "";
  if (*tag == '*')
    return NULL;
  n = s - tag;
  if (soap)
    for (p = soap->namespaces; p && p->id; p++)
      if (!strncmp(p->id, tag, n) && !p->id[n])
        return p->out ? p->out : p->ns;
  return NULL;
}

/******************************************************************************\
 *
 *      Tag matching
 *
\******************************************************************************/

static int soap_tag_match(const char *name, const char *tag)
{
  const char *s;
  if (!name)
    return !*tag;
  s = strchr(name, ':');
  if (s)
    name = s + 1;
  s = strchr(tag, ':');
  if (s)
    tag = s + 1;
  return !strcmp(name, tag);
}

/******************************************************************************/

static int soap_patt_match(const char *name, const char *patt)
{
  const char *s;
  if (!name)
    return !*patt;
  s = strchr(name, ':');
  if (s)
    name = s + 1;
  s = strchr(patt, ':');
  if (s)
    patt = s + 1;
  return soap_name_match(name, patt);
}

/******************************************************************************/

static int soap_name_match(const char *name, const char *patt)
{
  for (;;)
  {
    int c1 = *name;
    int c2 = *patt;
    if (!c1)
      break;
    if (c1 != c2)
    {
      if (c2 != '*')
        return 0;
      c2 = *++patt;
      if (!c2)
        return 1;
      for (;;)
      {
        c1 = *name;
        if (!c1)
          break;
        if (c1 == c2 && soap_name_match(name + 1, patt + 1))
          return 1;
        name++;
      }
      break;
    }
    name++;
    patt++;
  }
  if (*patt == '*' && !patt[1])
    return 1;
  return !*patt;
}

/******************************************************************************\
 *
 *      Memory
 *
\******************************************************************************/

static struct soap_dom_element *new_element(struct soap *soap)
{
  struct soap_dom_element *elt;
  elt = (struct soap_dom_element*)soap_malloc(soap, sizeof(struct soap_dom_element));
  if (elt)
  {
#ifdef __cplusplus
    SOAP_PLACEMENT_NEW(soap, elt, soap_dom_element);
#endif
    soap_default_xsd__anyType(soap, elt);
  }
  return elt;
}

/******************************************************************************/

static struct soap_dom_attribute *new_attribute(struct soap *soap)
{
  struct soap_dom_attribute *att;
  att = (struct soap_dom_attribute*)soap_malloc(soap, sizeof(struct soap_dom_attribute));
  if (att)
  {
#ifdef __cplusplus
    SOAP_PLACEMENT_NEW(soap, att, soap_dom_attribute);
#endif
    soap_default_xsd__anyAttribute(soap, att);
  }
  return att;
}

/******************************************************************************\
 *
 *      soap_dom_element construction
 *
\******************************************************************************/

/**
@brief Returns pointer to new xsd__anyType DOM element node
@param soap context that manages this object
@param ns namespace URI string or NULL
@param tag (un)qualified tag name string or NULL (unnamed node)
@return pointer to xsd__anyType DOM element node
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_new(struct soap *soap, const char *ns, const char *tag)
{
  return soap_elt_set(new_element(soap), ns, tag);
}

/******************************************************************************/

/**
@brief Returns pointer to new xsd__anyType DOM element node
@param soap context that manages this object
@param ns namespace URI string or NULL
@param tag (un)qualified tag name wide string or NULL (unnamed node)
@return pointer to xsd__anyType DOM element node
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_new_w(struct soap *soap, const char *ns, const wchar_t *tag)
{
  return soap_elt_set_w(new_element(soap), ns, tag);
}

/******************************************************************************/

/**
@brief Set xsd__anyType DOM element namespace URI and tag name
@param elt pointer to xsd__anyType DOM element to set
@param ns namespace URI string or NULL
@param tag (un)qualified tag name string or NULL (unnamed node)
@return elt
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_set(struct soap_dom_element *elt, const char *ns, const char *tag)
{
  if (elt)
  {
    if (tag && !*tag)
      tag = NULL;
    elt->name = soap_strdup(elt->soap, tag);
    if (!ns)
      elt->nstr = soap_ns_to_set(elt->soap, tag);
    else
      elt->nstr = soap_strdup(elt->soap, ns);
  }
  return elt;
}

/******************************************************************************/

/**
@brief Set xsd__anyType DOM element namespace URI and tag name
@param elt pointer to xsd__anyType DOM element to set
@param ns namespace URI string or NULL
@param tag (un)qualified tag name wide string or NULL (unnamed node)
@return elt
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_set_w(struct soap_dom_element *elt, const char *ns, const wchar_t *tag)
{
  if (elt)
  {
    if (tag && !*tag)
      tag = NULL;
    elt->name = soap_wchar2s(elt->soap, tag);
    if (!ns)
      elt->nstr = soap_ns_to_set(elt->soap, elt->name);
    else
      elt->nstr = soap_strdup(elt->soap, ns);
  }
  return elt;
}

/******************************************************************************/

/**
@brief Populate xsd__anyType DOM element node with an attribute node, if the attribute does not already exists
@param elt pointer to xsd__anyType DOM element to populate
@param ns namespace URI string or NULL of attribute
@param tag (un)qualified tag name string of attribute
@return pointer to xsd__anyAttribute attribute node (new attribute node if none matches)
*/
SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_att(struct soap_dom_element *elt, const char *ns, const char *tag)
{
  if (!elt)
    return NULL;
  if (elt->atts)
    return soap_att_add(elt->atts, ns, tag);
  return elt->atts = soap_att_new(elt->soap, ns, tag);
}

/******************************************************************************/

/**
@brief Populate xsd__anyType DOM element node with an attribute node, if the attribute does not already exists
@param elt pointer to xsd__anyType DOM element to populate
@param ns namespace URI string or NULL of attribute
@param tag (un)qualified tag name wide string of attribute
@return pointer to xsd__anyAttribute attribute node (new attribute node if none matches)
*/
SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_att_w(struct soap_dom_element *elt, const char *ns, const wchar_t *tag)
{
  if (!elt)
    return NULL;
  if (elt->atts)
    return soap_att_add_w(elt->atts, ns, tag);
  return elt->atts = soap_att_new_w(elt->soap, ns, tag);
}

/******************************************************************************/

/**
@brief Populate xsd__anyType DOM element node with a child element node
@param elt pointer to xsd__anyType DOM element to populate
@param ns namespace URI string or NULL of child element
@param tag (un)qualified tag name string of child element or NULL (unnamed node)
@return pointer to child element node (new child element node if none matches)
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt(struct soap_dom_element *elt, const char *ns, const char *tag)
{
  struct soap_dom_element *node = NULL;
  if (elt)
  {
    struct soap_dom_element *last = NULL;
    const char *ns1 = ns;
    if (!ns1)
      ns1 = soap_ns_to_set(elt->soap, tag);
    for (node = elt->elts; node; node = node->next)
    {
      if (tag && soap_tag_match(node->name, tag) &&
          (ns1 == node->nstr || (ns1 && node->nstr && !strcmp(node->nstr, ns1))))
        return node;
      if (!node->next)
        last = node;
    }
    node = soap_elt_new(elt->soap, ns, tag);
    if (node)
      node->prnt = elt;
    if (last)
      last->next = node;
    else
      elt->elts = node;
  }
  return node;
}

/******************************************************************************/

/**
@brief Populate xsd__anyType DOM element node with a child element node
@param elt pointer to xsd__anyType DOM element to populate
@param ns namespace URI string or NULL of child element
@param tag (un)qualified tag name wide string of child element or NULL (unnamed node)
@return pointer to xsd__anyType child element node (new child element node if none matches)
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_w(struct soap_dom_element *elt, const char *ns, const wchar_t *tag)
{
  struct soap_dom_element *node = NULL;
  if (elt)
  {
    const char *name = soap_wchar2s(elt->soap, tag);
    struct soap_dom_element *last = NULL;
    const char *ns1 = ns;
    if (!ns1)
      ns1 = soap_ns_to_set(elt->soap, name);
    for (node = elt->elts; node; node = node->next)
    {
      if (name && soap_tag_match(node->name, name) &&
          (ns1 == node->nstr || (ns1 && node->nstr && !strcmp(node->nstr, ns1))))
        return node;
      if (!node->next)
        last = node;
    }
    node = soap_elt_new(elt->soap, ns, name);
    if (node)
      node->prnt = elt;
    if (last)
      last->next = node;
    else
      elt->elts = node;
  }
  return node;
}

/******************************************************************************/

/**
@brief Populate xsd__anyType DOM element node with an N-th child element node
@param elt pointer to xsd__anyType DOM element to populate
@param ns namespace URI string or NULL of child element
@param tag (un)qualified tag name string of child element or NULL (unnamed node)
@param n child element position, counting from one (1) XPath style
@return pointer to xsd__anyType child element node (new child element node if none matches)
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_nth_elt(struct soap_dom_element *elt, const char *ns, const char *tag, size_t n)
{
  return soap_nth(soap_elt(elt, ns, tag), n);
}

/******************************************************************************/

/**
@brief Populate xsd__anyType DOM element node with an N-th child element node
@param elt pointer to xsd__anyType DOM element to populate
@param ns namespace URI string or NULL of child element
@param tag (un)qualified tag name wide string of child element or NULL (unnamed node)
@param n child element position, counting from one (1) XPath style
@return pointer to xsd__anyType child element node (new child element node if none matches)
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_nth_elt_w(struct soap_dom_element *elt, const char *ns, const wchar_t *tag, size_t n)
{
  return soap_nth(soap_elt_w(elt, ns, tag), n);
}

/******************************************************************************/

/**
@brief Add a N-th child element with the same namespace and tag name as a xsd__anyType DOM child element node at position one (1) in a sibling list
@param elt pointer to xsd__anyType DOM child element at position one (1)
@param n additional child element position requested, counting from one (1) XPath style
@return pointer to xsd__anyType child element node (new child element node if none exists at position n)
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_nth(struct soap_dom_element *elt, size_t n)
{
  struct soap_dom_element *node;
  struct soap_dom_element *prev = elt;
  if (!elt || n <= 1)
    return elt;
  for (node = elt->next; node; node = node->next)
  {
    if ((elt->name == node->name || (elt->name && soap_tag_match(node->name, elt->name))) &&
        (elt->nstr == node->nstr || (elt->nstr && node->nstr && !strcmp(node->nstr, elt->nstr))))
    {
      if (--n <= 1)
        return node;
    }
    prev = node;
  }
  while (n-- > 1)
  {
    node = new_element(elt->soap);
    node->next = prev->next;
    node->prnt = elt->prnt;
    node->nstr = elt->nstr;
    node->name = elt->name;
    prev->next = node;
    prev = node;
  }
  return prev;
}

/******************************************************************************/

/**
@brief Add an attribute node to an xsd__anyType DOM element node
@param elt pointer to xsd__anyType DOM element to populate
@param node pointer to xsd__anyAttribute attribute node to copy and add to DOM element elt
@return elt
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_add_att(struct soap_dom_element *elt, const struct soap_dom_attribute *node)
{
  if (elt && node)
  {
    struct soap_dom_attribute *last;
    struct soap_dom_attribute **next = &elt->atts;
    for (last = elt->atts; last; last = last->next)
      if (!last->next)
        next = &last->next;
    *next = new_attribute(elt->soap);
    if (*next)
      soap_att_copy(*next, node);
  }
  return elt;
}

/******************************************************************************/

/**
@brief Add a child element node to an xsd__anyType DOM element node
@param elt pointer to xsd__anyType DOM element to populate
@param node pointer to element node to copy and add as a child to DOM element elt
@return elt
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_add_elt(struct soap_dom_element *elt, const struct soap_dom_element *node)
{
  if (elt && node)
  {
    struct soap_dom_element *last;
    struct soap_dom_element **next = &elt->elts;
    for (last = elt->elts; last; last = last->next)
      if (!last->next)
        next = &last->next;
    *next = new_element(elt->soap);
    if (*next)
    {
      soap_elt_copy(*next, node);
      (*next)->prnt = elt;
    }
  }
  return elt;
}

/******************************************************************************/

/**
@brief Copy list of attribute nodes to an xsd__anyType DOM element node
@param elt pointer to xsd__anyType DOM element to populate
@param atts pointer to list of xsd__anyAttribute attribute nodes to copy and add to DOM element elt
@return elt
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_add_atts(struct soap_dom_element *elt, const struct soap_dom_attribute *atts)
{
  if (elt && atts)
  {
    struct soap_dom_attribute *last;
    struct soap_dom_attribute **next = &elt->atts;
    for (last = elt->atts; last; last = last->next)
      if (!last->next)
        next = &last->next;
    for (; atts; atts = atts->next)
    {
      *next = new_attribute(elt->soap);
      if (*next)
      {
        soap_att_copy(*next, atts);
        next = &(*next)->next;
      }
    }
  }
  return elt;
}

/******************************************************************************/

/**
@brief Copy list of element nodes to an xsd__anyType DOM element node as children
@param elt pointer to xsd__anyType DOM element to populate
@param elts pointer to list of xsd__anyType DOM element nodes to copy and add as children to DOM element elt
@return elt
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_add_elts(struct soap_dom_element *elt, const struct soap_dom_element *elts)
{
  if (elt && elts)
  {
    struct soap_dom_element *last;
    struct soap_dom_element **next = &elt->elts;
    for (last = elt->elts; last; last = last->next)
      if (!last->next)
        next = &last->next;
    for (; elts; elts = elts->next)
    {
      *next = new_element(elt->soap);
      if (*next)
      {
        soap_elt_copy(*next, elts);
        (*next)->prnt = elt;
        next = &(*next)->next;
      }
    }
  }
  return elt;
}

/******************************************************************************\
 *
 *      soap_dom_element assignment
 *
\******************************************************************************/

/**
@brief Set text of an xsd__anyType DOM element node to Boolean "false" or "true"
@param elt pointer to xsd__anyType DOM element node to set
@param b Boolean value (zero = false, nonzero = true)
@return elt
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_bool(struct soap_dom_element *elt, LONG64 b)
{
  if (elt)
    elt->text = b ? "true" : "false";
  return elt;
}

/******************************************************************************/

/**
@brief Set text of an xsd__anyType DOM element node to a 64 bit integer value
@param elt pointer to xsd__anyType DOM element node to set
@param n 64 bit integer value
@return elt
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_int(struct soap_dom_element *elt, LONG64 n)
{
  return soap_elt_text(elt, soap_LONG642s(elt->soap, n));
}

/******************************************************************************/

/**
@brief Set text of an xsd__anyType DOM element node to a double float value
@param elt pointer to xsd__anyType DOM element node to set
@param x double float value (NaN and +/-INF are also supported)
@return elt
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_double(struct soap_dom_element *elt, double x)
{
  return soap_elt_text(elt, soap_double2s(elt->soap, x));
}

/******************************************************************************/

/**
@brief Set text of an xsd__anyType DOM element node
@param elt pointer to xsd__anyType DOM element node to set
@param text string or NULL
@return elt
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_text(struct soap_dom_element *elt, const char *text)
{
  if (elt)
    elt->text = soap_strdup(elt->soap, text);
  return elt;
}

/******************************************************************************/

/**
@brief Set text of an xsd__anyType DOM element node
@param elt pointer to xsd__anyType DOM element node to set
@param text wide string or NULL
@return elt
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_text_w(struct soap_dom_element *elt, const wchar_t *text)
{
  if (elt)
    elt->text = soap_wchar2s(elt->soap, text);
  return elt;
}

/******************************************************************************/

/**
@brief Set an xsd__anyType DOM element node to point to a serializable object
@param elt pointer to xsd__anyType DOM element node to set
@param node pointer to serializable object
@param type SOAP_TYPE_T type of the serializable object of type T
@return elt
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_node(struct soap_dom_element *elt, const void *node, int type)
{
  if (elt)
  {
    elt->node = node;
    elt->type = type;
  }
  return elt;
}

/******************************************************************************/

/**
@brief Copy an xsd__anyType DOM element node to another (not a deep copy)
@param elt pointer to xsd__anyType DOM element node to copy to (destination)
@param node pointer to xsd__anyType DOM element node to copy from (source)
@return elt
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_copy(struct soap_dom_element *elt, const struct soap_dom_element *node)
{
  if (!elt)
    return NULL;
  if (!elt->soap)
    elt->soap = node->soap;
  elt->nstr = node->nstr;
  elt->name = node->name;
  elt->lead = node->lead;
  elt->text = node->text;
  elt->code = node->code;
  elt->tail = node->tail;
  elt->node = node->node;
  elt->type = node->type;
  soap_add_atts(elt, node->atts);
  return soap_add_elts(elt, node->elts);
}

/******************************************************************************\
 *
 *      soap_dom_element properties
 *
\******************************************************************************/

/**
@brief Match the namespace URI and tag name of an xsd__anyType DOM element node against a pattern
@param elt pointer to xsd__anyType DOM element node
@param ns namespace URI string pattern ("*" matches any, NULL and "" match the null (empty) namespace)
@param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any, "" matches unnamed node)
@return nonzero if match
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_elt_match(const struct soap_dom_element *elt, const char *ns, const char *patt)
{
  if (!elt || !elt->name)
    return 0;
  if (!ns && patt)
    ns = soap_ns_to_find(elt->soap, patt);
  if (patt && !soap_patt_match(elt->name, patt))
    return 0;
  if (ns && (elt->nstr || *ns) && (!elt->nstr || !soap_name_match(elt->nstr, ns)))
    return 0;
  return 1;
}

/******************************************************************************/

/**
@brief Match the namespace URI and tag name of an xsd__anyType DOM element node against a pattern
@param elt pointer to xsd__anyType DOM element node
@param ns namespace URI string pattern ("*" matches any, NULL and "" match the null (empty) namespace)
@param patt (un)qualified tag name wide string pattern (NULL, "*", and "*:*" match any, "" matches unnamed node)
@return nonzero if match
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_elt_match_w(const struct soap_dom_element *elt, const char *ns, const wchar_t *patt)
{
  const char *s;
  int r = 1;
  if (!elt || !elt->name)
    return 0;
  s = soap_wchar2s(NULL, patt);
  if (!ns && s)
    ns = soap_ns_to_find(elt->soap, s);
  if (s && !soap_patt_match(elt->name, s))
    r = 0;
  else if (ns && (elt->nstr || *ns) && (!elt->nstr || !soap_name_match(elt->nstr, ns)))
    r = 0;
  if (s)
    free((void*)s);
  return r;
}

/******************************************************************************/

/**
@brief Get the namespace URI of an xsd__anyType DOM element node
@param elt pointer to xsd__anyType DOM element node
@return namespace URI string or NULL
*/
SOAP_FMAC1
const char*
SOAP_FMAC2
soap_elt_get_ns(const struct soap_dom_element *elt)
{
  if (elt)
    return elt->nstr;
  return NULL;
}

/******************************************************************************/

/**
@brief Get the tag name of an xsd__anyType DOM element node, if any
@param elt pointer to xsd__anyType DOM element node
@return tag name string or NULL
*/
SOAP_FMAC1
const char*
SOAP_FMAC2
soap_elt_get_tag(const struct soap_dom_element *elt)
{
  if (elt)
    return elt->name;
  return NULL;
}

/******************************************************************************/

/**
@brief Return nonzero if xsd__anyType DOM element node text is Boolean "true" or "1"
@param elt pointer to xsd__anyType DOM element node
@return zero (not "true" or "1") or nonzero ("true" or "1")
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_elt_is_true(const struct soap_dom_element *elt)
{
  return elt->text && (!strcmp(elt->text, "true") || !strcmp(elt->text, "1"));
}

/******************************************************************************/

/**
@brief Return nonzero if xsd__anyType DOM element node text is Boolean "false" or "0"
@param elt pointer to xsd__anyType DOM element node
@return zero (not "false" or "0") or nonzero ("false" or "0")
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_elt_is_false(const struct soap_dom_element *elt)
{
  return elt->text && (!strcmp(elt->text, "false") || !strcmp(elt->text, "0"));
}

/******************************************************************************/

/**
@brief Return integer value of numeric text of xsd__anyType DOM element node, requires non-NULL soap context in the DOM
@param elt pointer to xsd__anyType DOM element node
@return integer value or 0 if text is not numeric
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_elt_get_int(const struct soap_dom_element *elt)
{
  if (elt)
  {
    int n;
    if (elt->text && !soap_s2int(elt->soap, elt->text, &n))
      return n;
    elt->soap->error = SOAP_OK;
  }
  return 0;
}

/******************************************************************************/

/**
@brief Return long integer value of numeric text of xsd__anyType DOM element node, requires non-NULL soap context in the DOM
@param elt pointer to xsd__anyType DOM element node
@return long integer value or 0 if text is not numeric
*/
SOAP_FMAC1
long
SOAP_FMAC2
soap_elt_get_long(const struct soap_dom_element *elt)
{
  if (elt)
  {
    long n;
    if (elt->text && !soap_s2long(elt->soap, elt->text, &n))
      return n;
    elt->soap->error = SOAP_OK;
  }
  return 0;
}

/******************************************************************************/

/**
@brief Return 64 bit integer value of numeric text of xsd__anyType DOM element node, requires non-NULL soap context in the DOM
@param elt pointer to xsd__anyType DOM element node
@return 64 bit integer value or 0 if text is not numeric
*/
SOAP_FMAC1
LONG64
SOAP_FMAC2
soap_elt_get_LONG64(const struct soap_dom_element *elt)
{
  if (elt)
  {
    LONG64 n;
    if (elt->text && !soap_s2LONG64(elt->soap, elt->text, &n))
      return n;
    elt->soap->error = SOAP_OK;
  }
  return 0;
}

/******************************************************************************/

/**
@brief Return double float value of decimal text of xsd__anyType DOM element node, requires non-NULL soap context in the DOM
@param elt pointer to xsd__anyType DOM element node
@return double float value or NaN if text is not numeric
*/
SOAP_FMAC1
double
SOAP_FMAC2
soap_elt_get_double(const struct soap_dom_element *elt)
{
  if (elt)
  {
    double x;
    if (elt->text && !soap_s2double(elt->soap, elt->text, &x))
      return x;
    elt->soap->error = SOAP_OK;
  }
  return DBL_NAN;
}

/******************************************************************************/

/**
@brief Return text of xsd__anyType DOM element node
@param elt pointer to xsd__anyType DOM element node
@return string or NULL
*/
SOAP_FMAC1
const char*
SOAP_FMAC2
soap_elt_get_text(const struct soap_dom_element *elt)
{
  if (elt)
    return elt->text;
  return NULL;
}

/******************************************************************************/

/**
@brief Get pointer to deserialized object stored in xsd__anyType DOM element node
@param elt pointer to xsd__anyType DOM element node
@param type SOAP_TYPE_T requested type to match
@return pointer to deserialized object or NULL if type mismatch or when no deserialized object is present
*/
SOAP_FMAC1 const void * SOAP_FMAC2
soap_elt_get_node(const struct soap_dom_element *elt, int type)
{
  if (!elt || elt->type != type)
    return NULL;
  return elt->node;
}

/******************************************************************************/

/**
@brief Get pointer to deserialized object stored in xsd__anyType DOM element node
@param elt pointer to xsd__anyType DOM element node
@param node pointer to pointer to void, to be set to point to deserialized object
@return SOAP_TYPE_T type of deserialized object, or zero (0) if none
*/
SOAP_FMAC1 int SOAP_FMAC2
soap_elt_get_type(const struct soap_dom_element *elt, const void **node)
{
  if (!elt)
    return 0;
  *node = &elt->node;
  return elt->type;
}

/******************************************************************************/

/**
@brief Return pointer to parent of an xsd__anyType DOM element node
@param elt pointer to xsd__anyType DOM element node
@return pointer to xsd__anyType DOM element or NULL
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_parent(const struct soap_dom_element *elt)
{
  if (!elt)
    return NULL;
  return elt->prnt;
}

/******************************************************************************/

/**
@brief Return depth from the root node of an xsd__anyType DOM element node
@param elt pointer to xsd__anyType DOM element node
@return depth from root node, zero if node is a root node and has no parent
*/
SOAP_FMAC1
size_t
SOAP_FMAC2
soap_elt_depth(const struct soap_dom_element *elt)
{
  size_t n = 0;
  while (elt)
  {
    elt = elt->prnt;
    n++;
  }
  return n;
}

/******************************************************************************/

/**
@brief Return child index of an xsd__anyType DOM child element node in sibling list
@param elt pointer to xsd__anyType DOM element node
@return nonzero index of child among siblings, or 0 if node is a root node and has no parent
*/
SOAP_FMAC1
size_t
SOAP_FMAC2
soap_elt_index(const struct soap_dom_element *elt)
{
  size_t n = 0;
  if (elt && elt->prnt)
  {
    const struct soap_dom_element *node;
    n++;
    for (node = elt->prnt->elts; node && node != elt; n++, node = node->next)
      continue;
  }
  return n;
}

/******************************************************************************/

/**
@brief Return number of siblings of an xsd__anyType DOM child node that have the same namespace URI and tag name
@param elt pointer to xsd__anyType DOM element node
@return number of siblings plus one (for self) that have the same namespace URI and tag name
*/
SOAP_FMAC1
size_t
SOAP_FMAC2
soap_elt_len(const struct soap_dom_element *elt)
{
  size_t n = 0;
  if (elt && elt->prnt)
  {
    const struct soap_dom_element *node;
    for (node = soap_elt_get(elt->prnt, elt->nstr, elt->name); node; n++, node = soap_elt_get_next(node))
      continue;
  }
  return n;
}

/******************************************************************************/

/**
@brief Return index of an xsd__anyType DOM child node among siblings that have the same namespace URI and tag name
@param elt pointer to xsd__anyType DOM element node
@return nonzero N-th index (1 <= nth <= len), or 0 if element is root or is singular (has no siblings with the same namespace URI and tag name)
*/
SOAP_FMAC1
size_t
SOAP_FMAC2
soap_elt_nth(const struct soap_dom_element *elt)
{
  size_t n = 0;
  if (elt && elt->prnt)
  {
    const struct soap_dom_element *node;
    for (node = soap_elt_get(elt->prnt, elt->nstr, elt->name); node && node != elt; n++, node = soap_elt_get_next(node))
      continue;
    if (node && (n > 0 || soap_elt_get_next(node)))
      n++;
  }
  return n;
}

/******************************************************************************\
 *
 *      soap_dom_attribute construction
 *
\******************************************************************************/

/**
@brief Returns pointer to new xsd__anyAttribute DOM attribute node
@param soap context that manages this object
@param ns namespace URI string or NULL
@param tag (un)qualified tag name string
@return pointer to xsd__anyAttribute DOM attribute node
*/
SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_att_new(struct soap *soap, const char *ns, const char *tag)
{
  return soap_att_set(new_attribute(soap), ns, tag);
}

/******************************************************************************/

/**
@brief Returns pointer to new xsd__anyAttribute DOM attribute node
@param soap context that manages this object
@param ns namespace URI string or NULL
@param tag (un)qualified tag name wide string
@return pointer to xsd__anyAttribute DOM attribute node
*/
SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_att_new_w(struct soap *soap, const char *ns, const wchar_t *tag)
{
  return soap_att_set_w(new_attribute(soap), ns, tag);
}

/******************************************************************************/

/**
@brief Set xsd__anyAttribute DOM attribute namespace URI and tag name
@param att pointer to xsd__anyAttribute DOM attribute to set
@param ns namespace URI string or NULL
@param tag (un)qualified tag name string
@return att
*/
SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_att_set(struct soap_dom_attribute *att, const char *ns, const char *tag)
{
  if (att)
  {
    att->name = soap_strdup(att->soap, tag);
    if (!ns)
      att->nstr = soap_ns_to_set(att->soap, tag);
    else
      att->nstr = soap_strdup(att->soap, ns);
  }
  return att;
}

/******************************************************************************/

/**
@brief Set xsd__anyAttribute DOM attribute namespace URI and tag name
@param att pointer to xsd__anyAttribute DOM attribute to set
@param ns namespace URI string or NULL
@param tag (un)qualified tag name wide string
@return att
*/
SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_att_set_w(struct soap_dom_attribute *att, const char *ns, const wchar_t *tag)
{
  if (att)
  {
    att->name = soap_wchar2s(att->soap, tag);
    if (!ns)
      att->nstr = soap_ns_to_set(att->soap, att->name);
    else
      att->nstr = soap_strdup(att->soap, ns);
  }
  return att;
}

/******************************************************************************/

/**
@brief Add an attribute node to an xsd__anyAttribute DOM attribute node, if the attribute does not already exists, to create or extend an attribute list
@param att pointer to xsd__anyAttribute DOM attribute
@param ns namespace URI string or NULL of attribute
@param tag (un)qualified tag name string of attribute
@return pointer to xsd__anyAttribute attribute node (new attribute node if none matches in the list)
*/
SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_att_add(struct soap_dom_attribute *att, const char *ns, const char *tag)
{
  if (att && tag)
  {
    const char *ns1 = ns;
    if (!att->name)
      return soap_att_set(att, ns, tag);
    if (!ns1)
      ns1 = soap_ns_to_set(att->soap, tag);
    while (att)
    {
      if (att->name && soap_tag_match(att->name, tag) &&
          (ns1 == att->nstr || (ns1 && att->nstr && !strcmp(ns1, att->nstr))))
        return att;
      if (!att->next)
        return att->next = soap_att_new(att->soap, ns, tag);
      att = att->next;
    }
  }
  return att;
}

/******************************************************************************/

/**
@brief Add an attribute node to an xsd__anyAttribute DOM attribute node, if the attribute does not already exists, to create or extend an attribute list
@param att pointer to xsd__anyAttribute DOM attribute
@param ns namespace URI string or NULL of attribute
@param tag (un)qualified tag name wide string of attribute
@return pointer to xsd__anyAttribute attribute node (new attribute node if none matches in the list)
*/
SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_att_add_w(struct soap_dom_attribute *att, const char *ns, const wchar_t *tag)
{
  if (att && tag)
  {
    const char *name = soap_wchar2s(att->soap, tag);
    const char *ns1 = ns;
    if (!att->name)
      return soap_att_set(att, ns, name);
    if (!ns1)
      ns1 = soap_ns_to_set(att->soap, name);
    while (att)
    {
      if (att->name && soap_tag_match(att->name, name) &&
          (ns1 == att->nstr || (ns1 && att->nstr && !strcmp(ns1, att->nstr))))
        return att;
      if (!att->next)
      {
        att->next = soap_att_new(att->soap, ns, NULL);
        if (att->next)
          att->next->name = name;
        return att->next;
      }
      att = att->next;
    }
  }
  return att;
}

/******************************************************************************\
 *
 *      soap_dom_attribute assignment
 *
\******************************************************************************/

/**
@brief Set text of an xsd__anyAttribute DOM attribute node to Boolean "false" or "true"
@param att pointer to xsd__anyAttribute DOM attribute node to set
@param b Boolean value (zero = false, nonzero = true)
@return att
*/
SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_att_bool(struct soap_dom_attribute *att, LONG64 b)
{
  if (att)
    att->text = b ? "true" : "false";
  return att;
}

/******************************************************************************/

/**
@brief Set text of an xsd__anyAttribute DOM attribute node to a 64 bit integer value
@param att pointer to xsd__anyAttribute DOM attribute node to set
@param n 64 bit integer value
@return att
*/
SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_att_int(struct soap_dom_attribute *att, LONG64 n)
{
  return soap_att_text(att, soap_LONG642s(att->soap, n));
}

/******************************************************************************/

/**
@brief Set text of an xsd__anyAttribute DOM attribute node to a double float value
@param att pointer to xsd__anyAttribute DOM attribute node to set
@param x double float value (NaN and +/-INF are also supported)
@return att
*/
SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_att_double(struct soap_dom_attribute *att, double x)
{
  return soap_att_text(att, soap_double2s(att->soap, x));
}

/******************************************************************************/

/**
@brief Set text of an xsd__anyAttribute DOM attribute node
@param att pointer to xsd__anyAttribute DOM attribute node to set
@param text string or NULL
@return att
*/
SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_att_text(struct soap_dom_attribute *att, const char *text)
{
  if (att)
    att->text = soap_strdup(att->soap, text);
  return att;
}

/******************************************************************************/

/**
@brief Set text of an xsd__anyAttribute DOM attribute node
@param att pointer to xsd__anyAttribute DOM attribute node to set
@param text wide string or NULL
@return att
*/
SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_att_text_w(struct soap_dom_attribute *att, const wchar_t *text)
{
  if (att)
    att->text = soap_wchar2s(att->soap, text);
  return att;
}

/******************************************************************************/

/**
@brief Copy an xsd__anyAttribute DOM attribute node to another (not a deep copy)
@param att pointer to xsd__anyAttribute DOM attribute node to copy to (destination)
@param node pointer to xsd__anyAttribute DOM attribute node to copy from (source)
@return att
*/
SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_att_copy(struct soap_dom_attribute *att, const struct soap_dom_attribute *node)
{
  if (att)
  {
    if (!att->soap)
      att->soap = node->soap;
    att->nstr = node->nstr;
    att->name = node->name;
    att->text = node->text;
  }
  return att;
}

/******************************************************************************\
 *
 *      soap_dom_attribute properties
 *
\******************************************************************************/

/**
@brief Match the namespace URI and tag name of an xsd__anyAttribute DOM attribute node against a pattern
@param att pointer to xsd__anyAttribute DOM attribute node
@param ns namespace URI string pattern ("*" matches any, NULL and "" match the null (empty) namespace)
@param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any)
@return nonzero if match
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_att_match(const struct soap_dom_attribute *att, const char *ns, const char *patt)
{
  if (!att || !att->name)
    return 0;
  if (!ns && patt)
    ns = soap_ns_to_find(att->soap, patt);
  if (patt && !soap_patt_match(att->name, patt))
    return 0;
  if (ns && (att->nstr || *ns) && (!att->nstr || !soap_name_match(att->nstr, ns)))
    return 0;
  return 1;
}

/******************************************************************************/

/**
@brief Match the namespace URI and tag name of an xsd__anyAttribute DOM attribute node against a pattern
@param att pointer to xsd__anyAttribute DOM attribute node
@param ns namespace URI string pattern ("*" matches any, NULL and "" match the null (empty) namespace)
@param patt (un)qualified tag name wide string pattern (NULL, "*", and "*:*" match any)
@return nonzero if match
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_att_match_w(const struct soap_dom_attribute *att, const char *ns, const wchar_t *patt)
{
  const char *s;
  int r = 1;
  if (!att || !att->name)
    return 0;
  s = soap_wchar2s(NULL, patt);
  if (!ns && s)
    ns = soap_ns_to_find(att->soap, s);
  if (s && !soap_patt_match(att->name, s))
    r = 0;
  else if (ns && (att->nstr || *ns) && (!att->nstr || !soap_name_match(att->nstr, ns)))
    r = 0;
  if (s)
    free((void*)s);
  return r;
}

/******************************************************************************/

/**
@brief Get the namespace URI of an xsd__anyAttribute DOM attribute node
@param att pointer to xsd__anyAttribute DOM attribute node
@return namespace URI string or NULL
*/
SOAP_FMAC1
const char*
SOAP_FMAC2
soap_att_get_ns(const struct soap_dom_attribute *att)
{
  if (att)
    return att->nstr;
  return NULL;
}

/******************************************************************************/

/**
@brief Get the tag name of an xsd__anyAttribute DOM attribute node
@param att pointer to xsd__anyAttribute DOM attribute node
@return tag name string or NULL
*/
SOAP_FMAC1
const char*
SOAP_FMAC2
soap_att_get_tag(const struct soap_dom_attribute *att)
{
  if (att)
    return att->name;
  return NULL;
}

/******************************************************************************/

/**
@brief Return nonzero if xsd__anyAttribute DOM attribute node text is Boolean "true" or "1"
@param att pointer to xsd__anyAttribute DOM attribute node
@return zero (not "true" or "1") or nonzero ("true" or "1")
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_att_is_true(const struct soap_dom_attribute *att)
{
  return att->text && (!strcmp(att->text, "true") || !strcmp(att->text, "1"));
}

/******************************************************************************/

/**
@brief Return nonzero if xsd__anyAttribute DOM attribute node text is Boolean "false" or "0"
@param att pointer to xsd__anyAttribute DOM attribute node
@return zero (not "false" or "0") or nonzero ("false" or "0")
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_att_is_false(const struct soap_dom_attribute *att)
{
  return att->text && (!strcmp(att->text, "false") || !strcmp(att->text, "0"));
}

/******************************************************************************/

/**
@brief Return integer value of numeric text of xsd__anyAttribute DOM attribute node, requires non-NULL soap context in the DOM
@param att pointer to xsd__anyAttribute DOM attribute node
@return integer value or 0 if text is not numeric
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_att_get_int(const struct soap_dom_attribute *att)
{
  if (att)
  {
    int n;
    if (att->text && !soap_s2int(att->soap, att->text, &n))
      return n;
    att->soap->error = SOAP_OK;
  }
  return 0;
}

/******************************************************************************/

/**
@brief Return long integer value of numeric text of xsd__anyAttribute DOM attribute node, requires non-NULL soap context in the DOM
@param att pointer to xsd__anyAttribute DOM attribute node
@return long integer value or 0 if text is not numeric
*/
SOAP_FMAC1
long
SOAP_FMAC2
soap_att_get_long(const struct soap_dom_attribute *att)
{
  if (att)
  {
    long n;
    if (att->text && !soap_s2long(att->soap, att->text, &n))
      return n;
    att->soap->error = SOAP_OK;
  }
  return 0;
}

/******************************************************************************/

/**
@brief Return 64 bit integer value of numeric text of xsd__anyAttribute DOM attribute node, requires non-NULL soap context in the DOM
@param att pointer to xsd__anyAttribute DOM attribute node
@return 64 bit integer value or 0 if text is not numeric
*/
SOAP_FMAC1
LONG64
SOAP_FMAC2
soap_att_get_LONG64(const struct soap_dom_attribute *att)
{
  if (att)
  {
    LONG64 n;
    if (att->text && !soap_s2LONG64(att->soap, att->text, &n))
      return n;
    att->soap->error = SOAP_OK;
  }
  return 0;
}

/******************************************************************************/

/**
@brief Return double float value of decimal text of xsd__anyAttribute DOM attribute node, requires non-NULL soap context in the DOM
@param att pointer to xsd__anyAttribute DOM attribute node
@return double float value or NaN if text is not numeric
*/
SOAP_FMAC1
double
SOAP_FMAC2
soap_att_get_double(const struct soap_dom_attribute *att)
{
  if (att)
  {
    double x;
    if (att->text && !soap_s2double(att->soap, att->text, &x))
      return x;
    att->soap->error = SOAP_OK;
  }
  return DBL_NAN;
}

/******************************************************************************/

/**
@brief Return text of xsd__anyAttribute DOM attribute node
@param att pointer to xsd__anyAttribute DOM attribute node
@return string or NULL
*/
SOAP_FMAC1
const char*
SOAP_FMAC2
soap_att_get_text(const struct soap_dom_attribute *att)
{
  if (att)
    return att->text;
  return NULL;
}

/******************************************************************************\
 *
 *      DOM local traversal
 *
\******************************************************************************/

/**
@brief Returns pointer to first xsd__anyAttribute DOM attribute node of xsd__anyType DOM element node, if any
@param elt pointer to xsd__anyType DOM element node
@return pointer to xsd__anyAttribute DOM attribute node or NULL if none
*/
SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_att_first(struct soap_dom_element *elt)
{
  return elt->atts;
}

/******************************************************************************/

/**
@brief Returns pointer to next xsd__anyAttribute DOM attribute node, if any
@param att pointer to current xsd__anyAttribute DOM attribute node in attribute list
@return pointer to xsd__anyAttribute DOM attribute node or NULL if none
*/
SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_att_next(const struct soap_dom_attribute *att)
{
  return att->next;
}

/******************************************************************************/

/**
@brief Returns pointer to first child element node of xsd__anyType DOM element node, if any
@param elt pointer to xsd__anyType DOM element node
@return pointer to xsd__anyType DOM child element node or NULL if none
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_first(struct soap_dom_element *elt)
{
  return elt->elts;
}

/******************************************************************************/

/**
@brief Returns pointer to next xsd__anyType DOM child element node, if any
@param elt pointer to current xsd__anyType DOM child element node in sibling list
@return pointer to xsd__anyType DOM element node or NULL if none
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_next(const struct soap_dom_element *elt)
{
  return elt->next;
}

/******************************************************************************\
 *
 *      DOM local retrieval
 *
\******************************************************************************/

/**
@brief Get attribute node of an xsd__anyType DOM element node, if attribute exists
@param elt pointer to xsd__anyType DOM element node
@param ns namespace URI string (NULL matches the null (empty) namespace if tag is unqualified) of attribute
@param tag (un)qualified tag name string of attribute
@return pointer to xsd__anyAttribute DOM attribute node or NULL if not exists
*/
SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_att_get(const struct soap_dom_element *elt, const char *ns, const char *tag)
{
  if (elt && tag)
  {
    struct soap_dom_attribute *att;
    if (!ns)
      ns = soap_ns_to_get(elt->soap, tag);
    for (att = elt->atts; att; att = att->next)
      if (att->name && soap_tag_match(att->name, tag) &&
          ((!att->nstr && !*ns) || (att->nstr && !strcmp(att->nstr, ns))))
        return att;
  }
  return NULL;
}

/******************************************************************************/

/**
@brief Get attribute node of an xsd__anyType DOM element node, if attribute exists
@param elt pointer to xsd__anyType DOM element node
@param ns namespace URI string (NULL matches the null (empty) namespace if tag is unqualified) of attribute
@param tag (un)qualified tag name wide string of attribute
@return pointer to xsd__anyAttribute DOM attribute node or NULL if not exists
*/
SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_att_get_w(const struct soap_dom_element *elt, const char *ns, const wchar_t *tag)
{
  if (elt && tag)
  {
    const char *name = soap_wchar2s(NULL, tag);
    struct soap_dom_attribute *att = NULL;
    if (!ns)
      ns = soap_ns_to_get(elt->soap, name);
    for (att = elt->atts; att; att = att->next)
      if (att->name && soap_tag_match(att->name, name) &&
          ((!att->nstr && !*ns) || (att->nstr && !strcmp(att->nstr, ns))))
        break;
    if (name)
      free((void*)name);
    return att;
  }
  return NULL;
}

/******************************************************************************/

/**
@brief Get child element node of an xsd__anyType DOM element node, if child element exists
@param elt pointer to xsd__anyType DOM element node
@param ns namespace URI string (NULL matches the null (empty) namespace if tag is unqualified) of child element
@param tag (un)qualified tag name string (NULL matches unnamed node) of child element
@return pointer to xsd__anyType DOM child element node or NULL if not exists
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_get(const struct soap_dom_element *elt, const char *ns, const char *tag)
{
  if (elt)
  {
    struct soap_dom_element *node;
    if (!ns)
      ns = soap_ns_to_get(elt->soap, tag);
    for (node = elt->elts; node; node = node->next)
      if ((tag == node->name || (tag && soap_tag_match(node->name, tag))) &&
          ((!node->nstr && !*ns) || (node->nstr && !strcmp(node->nstr, ns))))
        return node;
  }
  return NULL;
}

/******************************************************************************/

/**
@brief Get child element node of an xsd__anyType DOM element node, if child element exists
@param elt pointer to xsd__anyType DOM element node
@param ns namespace URI string (NULL matches the null (empty) namespace if tag is unqualified) of child element
@param tag (un)qualified tag name wide string (NULL matches unnamed node) of child element
@return pointer to xsd__anyType DOM child element node or NULL if not exists
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_get_w(const struct soap_dom_element *elt, const char *ns, const wchar_t *tag)
{
  if (elt)
  {
    const char *name = soap_wchar2s(NULL, tag);
    struct soap_dom_element *node;
    if (!ns)
      ns = soap_ns_to_get(elt->soap, name);
    for (node = elt->elts; node; node = node->next)
      if (((!name && !node->name) || (name && soap_tag_match(node->name, name))) &&
          ((!node->nstr && !*ns) || (node->nstr && !strcmp(node->nstr, ns))))
        break;
    if (name)
      free((void*)name);
    return node;
  }
  return NULL;
}

/******************************************************************************/

/**
@brief Get next child element node that has the same namespace URI and tag name as the current child element node
@param elt pointer to current xsd__anyType DOM child element node
@return pointer to xsd__anyType DOM child element node or NULL if not exists
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_get_next(const struct soap_dom_element *elt)
{
  if (elt)
  {
    const char *ns = elt->nstr;
    const char *tag = elt->name;
    struct soap_dom_element *node;
    for (node = elt->next; node; node = node->next)
      if ((tag == node->name || (tag && soap_tag_match(node->name, tag))) &&
          (ns == node->nstr || (ns && node->nstr && !strcmp(node->nstr, ns))))
        return node;
  }
  return NULL;
}

/******************************************************************************/

/**
@brief Get N-th child element node that has the same namespace URI and tag name as the current child element node at position one (1) in the sibling list
@param elt pointer to current xsd__anyType DOM child element node at position one (1) in the sibling list
@param n N-th element minus one located from the current element node at position one (1), 1 returns the current element (elt)
@return pointer to xsd__anyType DOM child element node or NULL if not exists
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_get_nth(struct soap_dom_element *elt, size_t n)
{
  while (n-- > 1)
    elt = soap_elt_get_next(elt);
  return elt;
}

/******************************************************************************\
 *
 *      DOM local search
 *
\******************************************************************************/

/**
@brief Find matching attribute node of an xsd__anyType DOM element node
@param elt pointer to xsd__anyType DOM element node
@param ns namespace URI string pattern ("*" matches any, "" matches the null (empty) namespace, NULL matches the null (empty) namespace if tag is unqualified or the namespace from namespace table if tag is qualified)
@param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any)
@return pointer to xsd__anyAttribute DOM attribute node or NULL if not exists
*/
SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_att_find(struct soap_dom_element *elt, const char *ns, const char *patt)
{
  struct soap_dom_attribute *att = NULL;
  if (elt)
  {
    att = elt->atts;
    if (att)
    {
      if (!ns && patt)
        ns = soap_ns_to_find(elt->soap, patt);
      if (patt && !soap_patt_match(att->name, patt))
        return soap_att_find_next(att, ns, patt);
      if (ns && (att->nstr || *ns) && (!att->nstr || !soap_name_match(att->nstr, ns)))
        return soap_att_find_next(att, ns, patt);
    }
  }
  return att;
}

/******************************************************************************/

/**
@brief Find next matching attribute node of an xsd__anyType DOM element node
@param att pointer to current xsd__anyAttribute DOM attribute node
@param ns namespace URI string pattern ("*" matches any, "" matches the null (empty) namespace, NULL matches the null (empty) namespace if tag is unqualified or the namespace from namespace table if tag is qualified)
@param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any)
@return pointer to xsd__anyAttribute DOM attribute node or NULL if not exists
*/
SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_att_find_next(const struct soap_dom_attribute *att, const char *ns, const char *patt)
{
  if (att)
  {
    struct soap_dom_attribute *node;
    if (!ns && patt)
      ns = soap_ns_to_find(att->soap, patt);
    for (node = att->next; node; node = node->next)
    {
      if (patt && !soap_patt_match(node->name, patt))
        continue;
      if (ns && (node->nstr || *ns) && (!node->nstr || !soap_name_match(node->nstr, ns)))
        continue;
      return node;
    }
  }
  return NULL;
}

/******************************************************************************/

/**
@brief Find matching child element node of an xsd__anyType DOM element node
@param elt pointer to xsd__anyType DOM element node
@param ns namespace URI string pattern ("*" matches any, "" matches the null (empty) namespace, NULL matches the null (empty) namespace if tag is unqualified or the namespace from namespace table if tag is qualified)
@param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any, "" matches unnamed node)
@return pointer to xsd__anyType DOM child element node or NULL if not exists
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_find(struct soap_dom_element *elt, const char *ns, const char *patt)
{
  return soap_elt_find_type(elt, ns, patt, 0);
}

/******************************************************************************/

/**
@brief Find next matching child element node of an xsd__anyType DOM element node
@param elt pointer to current xsd__anyType DOM child element node
@param ns namespace URI string pattern ("*" matches any, "" matches the null (empty) namespace, NULL matches the null (empty) namespace if tag is unqualified or the namespace from namespace table if tag is qualified)
@param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any, "" matches unnamed node)
@return pointer to xsd__anyType DOM child element node or NULL if not exists
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_find_next(const struct soap_dom_element *elt, const char *ns, const char *patt)
{
  return soap_elt_find_next_type(elt, ns, patt, 0);
}

/******************************************************************************/

/**
@brief Find matching child element node of an xsd__anyType DOM element node that has a deserialized object
@param elt pointer to xsd__anyType DOM element node
@param ns namespace URI string pattern ("*" matches any, "" matches the null (empty) namespace, NULL matches the null (empty) namespace if tag is unqualified or the namespace from namespace table if tag is qualified)
@param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any, "" matches unnamed node)
@param type SOAP_TYPE_T type of deserialized object of type T to match or 0
@return pointer to xsd__anyType DOM child element node or NULL if not exists
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_find_type(struct soap_dom_element *elt, const char *ns, const char *patt, int type)
{
  if (elt)
  {
    elt = elt->elts;
    if (elt)
    {
      if (!ns && patt)
        ns = soap_ns_to_find(elt->soap, patt);
      if (patt && !soap_patt_match(elt->name, patt))
        return soap_elt_find_next_type(elt, ns, patt, type);
      if (ns && (elt->nstr || *ns) && (!elt->nstr || !soap_name_match(elt->nstr, ns)))
        return soap_elt_find_next_type(elt, ns, patt, type);
      if (type && type != elt->type)
        return soap_elt_find_next_type(elt, ns, patt, type);
    }
  }
  return elt;
}

/******************************************************************************/

/**
@brief Find next matching child element node of an xsd__anyType DOM element node that has a deserialized object
@param elt pointer to current xsd__anyType DOM child element node
@param ns namespace URI string pattern ("*" matches any, "" matches the null (empty) namespace, NULL matches the null (empty) namespace if tag is unqualified or the namespace from namespace table if tag is qualified)
@param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any, "" matches unnamed node)
@param type SOAP_TYPE_T type of deserialized object of type T to match or 0
@return pointer to xsd__anyType DOM child element node or NULL if not exists
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_elt_find_next_type(const struct soap_dom_element *elt, const char *ns, const char *patt, int type)
{
  if (elt)
  {
    struct soap_dom_element *node;
    if (!ns && patt)
      ns = soap_ns_to_find(elt->soap, patt);
    for (node = elt->next; node; node = node->next)
    {
      if (patt && !soap_patt_match(node->name, patt))
        continue;
      if (ns && (node->nstr || *ns) && (!node->nstr || !soap_name_match(node->nstr, ns)))
        continue;
      if (type && type != node->type)
        continue;
      return node;
    }
  }
  return NULL;
}

/******************************************************************************\
 *
 *      DOM size of local search results
 *
\******************************************************************************/

/**
@brief Return number of matching attribute nodes of an xsd__anyType DOM element node
@param elt pointer to xsd__anyType DOM element node
@param ns namespace URI string pattern ("*" matches any, "" matches the null (empty) namespace, NULL matches the null (empty) namespace if tag is unqualified or the namespace from namespace table if tag is qualified)
@param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any)
@return number of matches found
*/
SOAP_FMAC1
size_t
SOAP_FMAC2
soap_att_size(struct soap_dom_element *elt, const char *ns, const char *patt)
{
  size_t n = 0;
  if (elt)
  {
    struct soap_dom_attribute *att = soap_att_find(elt, ns, patt);
    if (att)
    {
      n++;
      while ((att = soap_att_find_next(att, ns, patt)))
        n++;
    }
  }
  return n;
}

/******************************************************************************/

/**
@brief Return number of matching child element nodes of an xsd__anyType DOM element node
@param elt pointer to xsd__anyType DOM element node
@param ns namespace URI string pattern ("*" matches any, "" matches the null (empty) namespace, NULL matches the null (empty) namespace if tag is unqualified or the namespace from namespace table if tag is qualified)
@param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any, "" matches unnamed node)
@return number of matches found
*/
SOAP_FMAC1
size_t
SOAP_FMAC2
soap_elt_size(struct soap_dom_element *elt, const char *ns, const char *patt)
{
  return soap_elt_size_type(elt, ns, patt, 0);
}

/******************************************************************************/

/**
@brief Return number of matching child element nodes of an xsd__anyType DOM element node that have deserialized objects
@param elt pointer to xsd__anyType DOM element node
@param ns namespace URI string pattern ("*" matches any, "" matches the null (empty) namespace, NULL matches the null (empty) namespace if tag is unqualified or the namespace from namespace table if tag is qualified)
@param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any, "" matches unnamed node)
@param type SOAP_TYPE_T type of deserialized object of type T to match or 0
@return number of matches found
*/
SOAP_FMAC1
size_t
SOAP_FMAC2
soap_elt_size_type(struct soap_dom_element *elt, const char *ns, const char *patt, int type)
{
  size_t n = 0;
  if (elt)
  {
    elt = soap_elt_find_type(elt, ns, patt, type);
    if (elt)
    {
      n++;
      while ((elt = soap_elt_find_next_type(elt, ns, patt, type)))
        n++;
    }
  }
  return n;
}

/******************************************************************************\
 *
 *      DOM deep traversal
 *
\******************************************************************************/

/**
@brief Return next xsd__anyAttribute DOM attribute node in attribute list
@param att pointer to current xsd__anyAttribute DOM attribute node
@return pointer to xsd__anyAttribute DOM node or NULL if none
*/
SOAP_FMAC1
struct soap_dom_attribute *
SOAP_FMAC2
soap_dom_next_attribute(const struct soap_dom_attribute *att)
{
  return att->next;
}

/******************************************************************************/

/**
@brief Return next xsd__anyType DOM element node in depth-first traversal of node graph (XPath recursive descent)
@param elt pointer to current xsd__anyType DOM element node
@param end pointer to the ending xsd__anyType DOM element node (exclusive), usually the starting node of the graph traversal
@return pointer to xsd__anyType DOM node or NULL if none or if next node would be the ending node
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_dom_next_element(const struct soap_dom_element *elt, const struct soap_dom_element *end)
{
  struct soap_dom_element *node;
  if (elt->elts)
    return elt->elts;
  if (elt == end)
    return NULL;
  if (elt->next)
    return elt->next;
  for (node = elt->prnt; node && !node->next; node = node->prnt)
    if (node == end)
      return NULL;
  if (node)
    node = node->next;
  return node;
}

/******************************************************************************\
 *
 *      DOM deep search
 *
\******************************************************************************/

/**
@brief Find matching xsd__anyType DOM element node in depth-first traversal of node graph (XPath recursive descent)
@param begin pointer to starting xsd__anyType DOM element node of node graph to search
@param end pointer to the ending xsd__anyType DOM element node (exclusive), usually the starting node of the graph traversal
@param ns namespace URI string pattern ("*" matches any, "" matches the null (empty) namespace, NULL matches the null (empty) namespace if tag is unqualified or the namespace from namespace table if tag is qualified)
@param patt (un)qualified tag name string pattern (use '@' to match attributes, NULL, "*" and "*:*" match any, "" matches unnamed node)
@param type SOAP_TYPE_T type of deserialized object of type T to match or 0
@return pointer to xsd__anyType DOM node or NULL if none or if next node would be the ending node
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_dom_find(struct soap_dom_element *begin, const struct soap_dom_element *end, const char *ns, const char *patt, int type)
{
  if (begin)
  {
    if (patt && *patt == '@')
    {
      if (soap_att_find(begin, ns, patt + 1))
        return begin;
      return soap_dom_find_next(begin, end, ns, patt, type);
    }
    if (!ns && patt)
      ns = soap_ns_to_find(begin->soap, patt);
    if (patt && !soap_patt_match(begin->name, patt))
      return soap_dom_find_next(begin, end, ns, patt, type);
    if (ns && (begin->nstr || *ns) && (!begin->nstr || !soap_name_match(begin->nstr, ns)))
      return soap_dom_find_next(begin, end, ns, patt, type);
    if (type && type != begin->type)
      return soap_dom_find_next(begin, end, ns, patt, type);
  }
  return begin;
}

/******************************************************************************/

/**
@brief Find next matching xsd__anyType DOM element node in depth-first traversal of node graph (XPath recursive descent)
@param elt pointer to current xsd__anyType DOM element node of node graph to search
@param end pointer to the ending xsd__anyType DOM element node, usually the starting node of the graph traversal
@param ns namespace URI string pattern ("*" matches any, "" matches the null (empty) namespace, NULL matches the null (empty) namespace if tag is unqualified or the namespace from namespace table if tag is qualified)
@param patt (un)qualified tag name string pattern (use '@' to match attributes, NULL, "*" and "*:*" match any, "" matches unnamed node)
@param type SOAP_TYPE_T type of deserialized object of type T to match or 0
@return pointer to xsd__anyType DOM node or NULL if none or if next node would be the ending node
*/
SOAP_FMAC1
struct soap_dom_element *
SOAP_FMAC2
soap_dom_find_next(const struct soap_dom_element *elt, const struct soap_dom_element *end, const char *ns, const char *patt, int type)
{
  if (elt)
  {
    struct soap_dom_element *node;
    if (!ns && patt)
      ns = soap_ns_to_find(elt->soap, patt + (*patt == '@'));
    for (node = soap_dom_next_element(elt, end); node; node = soap_dom_next_element(node, end))
    {
      if (patt && *patt == '@')
      {
        if (soap_att_find(node, ns, patt + 1))
          return node;
        continue;
      }
      if (patt && !soap_patt_match(node->name, patt))
        continue;
      if (ns && (node->nstr || *ns) && (!node->nstr || !soap_name_match(node->nstr, ns)))
        continue;
      if (type && type != node->type)
        continue;
      return node;
    }
  }
  return NULL;
}

/******************************************************************************\
 *
 *      C REST POST, GET, PUT, DELETE server invocation
 *
\******************************************************************************/

/*
@brief REST POST, GET, PUT, DELETE server invocation with XML
@param soap context to manage IO
@param endpoint server URL
@param optional SOAP Action or NULL
@param in request XML to be send to server, or NULL for GET or DELETE (both in and out are NULL)
@param out response to be populated with response XML received from server, or NULL for PUT or DELETE (both in and out are NULL)
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_dom_call(struct soap *soap, const char *endpoint, const char *action, const struct soap_dom_element *in, struct soap_dom_element *out)
{
  if (out)
    soap_default_xsd__anyType(soap, out);
  if (in)
    soap_serialize_xsd__anyType(soap, in);
  soap->http_content = "text/xml; charset=utf-8";
  if (soap_begin_count(soap)
   || ((soap->mode & SOAP_IO_LENGTH) && soap_out_xsd__anyType(soap, NULL, 0, in, NULL))
   || soap_end_count(soap)
   || soap_connect_command(soap, in && out ? SOAP_POST_FILE : out ? SOAP_GET : in ? SOAP_PUT : SOAP_DEL, endpoint, action)
   || soap_out_xsd__anyType(soap, NULL, 0, in, NULL)
   || soap_end_send(soap))
    return soap_closesock(soap);
  if (!out)
  {
    if (!soap_begin_recv(soap))
    {
#ifndef WITH_LEAN
      (void)soap_http_get_body(soap, NULL);
#endif
      (void)soap_end_recv(soap);
    }
    else if (soap->error == 200 || soap->error == 201 || soap->error == 202)
    {
      soap->error = SOAP_OK;
    }
    return soap_closesock(soap);
  }
  if (soap_begin_recv(soap)
   || soap_in_xsd__anyType(soap, NULL, out, NULL) == NULL
   || soap_end_recv(soap))
    return soap_closesock(soap);
  return soap_closesock(soap);
}

/******************************************************************************/

#ifdef __cplusplus

/******************************************************************************\
 *
 *      C++ REST POST, GET, PUT, DELETE server invocation
 *
\******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_dom_call(struct soap *soap, const char *endpoint, const char *action, const struct soap_dom_element& in, struct soap_dom_element& out)
{
  return soap_dom_call(soap, endpoint, action, &in, &out);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_dom_call(struct soap *soap, const char *endpoint, const char *action, const struct soap_dom_element *in, struct soap_dom_element& out)
{
  return soap_dom_call(soap, endpoint, action, in, &out);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_dom_call(struct soap *soap, const char *endpoint, const char *action, const struct soap_dom_element& in, struct soap_dom_element *out)
{
  return soap_dom_call(soap, endpoint, action, &in, out);
}

/******************************************************************************\
 *
 *      C++ soap_dom_element and soap_dom_attribute classes
 *
\******************************************************************************/

#if 0 /* DOXYGEN PROCESSING ONLY */

/**
@class soap_dom_attribute
@brief The xsd__anyAttribute DOM attribute node structure (xsd__anyAttribute is a typedef of soap_dom_attribute)
*/
struct soap_dom_attribute
{
  struct soap_dom_attribute *next; ///< next attribute node in list
  const char *nstr;                ///< namespace string
  const char *name;                ///< (un)qualified tag name
  const char *text;                ///< text cdata in UTF-8
  struct soap *soap;               ///< context that manages this object

  /** iterator over attribute list */
  typedef soap_dom_attribute_iterator iterator;

  /** const_iterator over attribute list */
  typedef soap_dom_attribute_iterator const_iterator;

  /**
  @brief Construct new xsd__anyAttribute DOM attribute that is empty (must be set later)
  @param soap context that manages this object
  */
  soap_dom_attribute(struct soap *soap = NULL);
  /**
  @brief Construct new xsd__anyAttribute DOM attribute by (shallow) copying another attribute
  @param att xsd__anyAttribute DOM attribute copied
  */
  soap_dom_attribute(const soap_dom_attribute& att);
  /**
  @brief Construct new xsd__anyAttribute DOM attribute with (un)qualified tag name, same as xsd__anyAttribute(soap, NULL, tag, NULL)
  @param soap context that manages this object
  @param tag (un)qualified tag name string
  */
  soap_dom_attribute(struct soap *soap, const char *tag);
  /**
  @brief Construct new xsd__anyAttribute DOM attribute with (un)qualified tag name, same as xsd__anyAttribute(soap, NULL, tag, NULL)
  @param soap context that manages this object
  @param tag (un)qualified tag name wide string
  */
  soap_dom_attribute(struct soap *soap, const wchar_t *tag);
  /**
  @brief Construct new xsd__anyAttribute DOM attribute with namespace URI (xmlns), (un)qualified tag name, and text
  @param soap context that manages this object
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name string
  @param text string
  */
  soap_dom_attribute(struct soap *soap, const char *ns, const char *tag, const char *text);
  /**
  @brief Construct new xsd__anyAttribute DOM attribute with namespace URI (xmlns), (un)qualified tag name, and text
  @param soap context that manages this object
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name string
  @param text wide string
  */
  soap_dom_attribute(struct soap *soap, const char *ns, const char *tag, const wchar_t *text);
  /**
  @brief Construct new xsd__anyAttribute DOM attribute with namespace URI (xmlns), (un)qualified tag name, and text
  @param soap context that manages this object
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name wide string
  @param text string
  */
  soap_dom_attribute(struct soap *soap, const char *ns, const wchar_t *tag, const char *text);
  /**
  @brief Construct new xsd__anyAttribute DOM attribute with namespace URI (xmlns), (un)qualified tag name, and text
  @param soap context that manages this object
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name wide string
  @param text wide string
  */
  soap_dom_attribute(struct soap *soap, const char *ns, const wchar_t *tag, const wchar_t *text);
  /**
  @brief Construct new xsd__anyAttribute DOM attribute with namespace URI (xmlns), (un)qualified tag name, and text
  @param soap context that manages this object
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name string
  @param text string
  */
  soap_dom_attribute(struct soap *soap, const char *ns, const char *tag, const std::string& text);
  /**
  @brief Construct new xsd__anyAttribute DOM attribute with namespace URI (xmlns), (un)qualified tag name, and text
  @param soap context that manages this object
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name string
  @param text wide string
  */
  soap_dom_attribute(struct soap *soap, const char *ns, const char *tag, const std::wstring& text);
  /**
  @brief Construct new xsd__anyAttribute DOM attribute with namespace URI (xmlns), (un)qualified tag name, and text
  @param soap context that manages this object
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name wide string
  @param text string
  */
  soap_dom_attribute(struct soap *soap, const char *ns, const wchar_t *tag, const std::string& text);
  /**
  @brief Construct new xsd__anyAttribute DOM attribute with namespace URI (xmlns), (un)qualified tag name, and text
  @param soap context that manages this object
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name wide string
  @param text wide string
  */
  soap_dom_attribute(struct soap *soap, const char *ns, const wchar_t *tag, const std::wstring& text);
  /**
  @brief Destructor (no-op, deletes are done by the managing context of this object)
  */
  ~soap_dom_attribute();

  /**
  @brief Set this xsd__anyAttribute DOM attribute namespace URI and tag name
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name string
  @return reference to *this
  */
  soap_dom_attribute& set(const char *ns, const char *tag);
  /**
  @brief Set this xsd__anyAttribute DOM attribute namespace URI and tag name
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name string
  @return reference to *this
  */
  soap_dom_attribute& set(const char *ns, const wchar_t *tag);
  /**
  @brief Set text of this xsd__anyAttribute DOM attribute node to Boolean "false" or "true"
  @param b Boolean value
  @return reference to *this
  */
  soap_dom_attribute& set(bool b);
  /**
  @brief Set text of this xsd__anyAttribute DOM attribute node to an integer value
  @param n integer value
  @return reference to *this
  */
  soap_dom_attribute& set(int n);
  /**
  @brief Set text of this xsd__anyAttribute DOM attribute node to a 64 bit integer value
  @param n 64 bit integer value
  @return reference to *this
  */
  soap_dom_attribute& set(LONG64 n);
  /**
  @brief Set text of this xsd__anyAttribute DOM attribute node to a float value
  @param x float value (NaN and +/-INF are also supported)
  @return reference to *this
  */
  soap_dom_attribute& set(float x);
  /**
  @brief Set text of this xsd__anyAttribute DOM attribute node to a double float value
  @param x double float value (NaN and +/-INF are also supported)
  @return reference to *this
  */
  soap_dom_attribute& set(double x);
  /**
  @brief Set text of this xsd__anyAttribute DOM attribute node
  @param text string or NULL
  @return reference to *this
  */
  soap_dom_attribute& set(const char *text);
  /**
  @brief Set text of this xsd__anyAttribute DOM attribute node
  @param text wide string or NULL
  @return reference to *this
  */
  soap_dom_attribute& set(const wchar_t *text);
  /**
  @brief Set text of this xsd__anyAttribute DOM attribute node
  @param text string or NULL
  @return reference to *this
  */
  soap_dom_attribute& set(const std::string& text);
  /**
  @brief Set text of this xsd__anyAttribute DOM attribute node
  @param text wide string or NULL
  @return reference to *this
  */
  soap_dom_attribute& set(const std::wstring& text);
  /**
  @brief Set text of this xsd__anyAttribute DOM attribute node to Boolean "false" or "true", same as set(b)
  @param b Boolean value
  @return reference to *this
  */
  soap_dom_attribute& operator=(bool b);
  /**
  @brief Set text of this xsd__anyAttribute DOM attribute node to an integer value, same as set(n)
  @param n integer value
  @return reference to *this
  */
  soap_dom_attribute& operator=(int n);
  /**
  @brief Set text of this xsd__anyAttribute DOM attribute node to a 64 bit integer value, same as set(n)
  @param n 64 bit integer value
  @return reference to *this
  */
  soap_dom_attribute& operator=(LONG64 n);
  /**
  @brief Set text of this xsd__anyAttribute DOM attribute node to a float value, same as set(x)
  @param x float value (NaN and +/-INF are also supported)
  @return reference to *this
  */
  soap_dom_attribute& operator=(float x);
  /**
  @brief Set text of this xsd__anyAttribute DOM attribute node to a double float value, same as set(x)
  @param x double float value (NaN and +/-INF are also supported)
  @return reference to *this
  */
  soap_dom_attribute& operator=(double x);
  /**
  @brief Set text of this xsd__anyAttribute DOM attribute node, same as set(text)
  @param text string or NULL
  @return reference to *this
  */
  soap_dom_attribute& operator=(const char *text);
  /**
  @brief Set text of this xsd__anyAttribute DOM attribute node, same as set(text)
  @param text wide string or NULL
  @return reference to *this
  */
  soap_dom_attribute& operator=(const wchar_t *text);
  /**
  @brief Set text of this xsd__anyAttribute DOM attribute node, same as set(text)
  @param text string or NULL
  @return reference to *this
  */
  soap_dom_attribute& operator=(const std::string& text);
  /**
  @brief Set text of this xsd__anyAttribute DOM attribute node, same as set(text)
  @param text wide string or NULL
  @return reference to *this
  */
  soap_dom_attribute& operator=(const std::wstring& text);
  /**
  @brief Copy an xsd__anyAttribute DOM attribute node to this node (not a deep copy)
  @param att pointer to xsd__anyAttribute DOM attribute node to copy from
  @return reference to *this
  */
  soap_dom_attribute& operator=(const soap_dom_attribute& att);
  /**
  @brief Add an attribute node to this xsd__anyAttribute DOM attribute node to create or extend an attribute list, same as att(NULL, tag)
  @param tag (un)qualified tag name string of attribute
  @return reference to xsd__anyAttribute attribute node (new attribute node if none matches in the list)
  */
  soap_dom_attribute& att(const char *tag);
  /**
  @brief Add an attribute node to this xsd__anyAttribute DOM attribute node to create or extend an attribute list, same as att(NULL, tag)
  @param tag (un)qualified tag name wide string of attribute
  @return reference to xsd__anyAttribute attribute node (new attribute node if none matches in the list)
  */
  soap_dom_attribute& att(const wchar_t *tag);
  /**
  @brief Add an attribute node to this xsd__anyAttribute DOM attribute node to create or extend an attribute list
  @param ns namespace URI string or NULL of attribute
  @param tag (un)qualified tag name string of attribute
  @return reference to xsd__anyAttribute attribute node (new attribute node if none matches in the list)
  */
  soap_dom_attribute& att(const char *ns, const char *tag);
  /**
  @brief Add an attribute node to this xsd__anyAttribute DOM attribute node to create or extend an attribute list
  @param ns namespace URI string or NULL of attribute
  @param tag (un)qualified tag name wide string of attribute
  @return reference to xsd__anyAttribute attribute node (new attribute node if none matches in the list)
  */
  soap_dom_attribute& att(const char *ns, const wchar_t *tag);
  /**
  @brief Match the namespace URI and tag name of this xsd__anyAttribute DOM attribute node against a pattern, same as match(NULL, patt)
  @param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any)
  @return true if match
  */
  bool match(const char *patt) const;
  /**
  @brief Match the namespace URI and tag name of this xsd__anyAttribute DOM attribute node against a pattern, same as match(NULL, patt)
  @param patt (un)qualified tag name wide string pattern (NULL, "*", and "*:*" match any)
  @return true if match
  */
  bool match(const wchar_t *patt) const;
  /**
  @brief Match the namespace URI and tag name of this xsd__anyAttribute DOM attribute node against a pattern
  @param ns namespace URI string pattern ("*" matches any, NULL and "" match the null (empty) namespace)
  @param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any)
  @return true if match
  */
  bool match(const char *ns, const char *patt) const;
  /**
  @brief Match the namespace URI and tag name of this xsd__anyAttribute DOM attribute node against a pattern
  @param ns namespace URI string pattern ("*" matches any, NULL and "" match the null (empty) namespace)
  @param patt (un)qualified tag name wide string pattern (NULL, "*", and "*:*" match any)
  @return true if match
  */
  bool match(const char *ns, const wchar_t *patt) const;
  /**
  @brief Get the namespace URI of this xsd__anyAttribute DOM attribute node
  @return namespace URI string or NULL
  */
  const char *ns() const;
  /**
  @brief Get the tag name of this xsd__anyAttribute DOM attribute node
  @return tag name string or NULL
  */
  const char *tag() const;
  /**
  @brief Return true if this xsd__anyAttribute DOM attribute node text is Boolean "true" or "1"
  @return true if "true" or "1"
  */
  bool is_true() const;
  /**
  @brief Return true if this xsd__anyAttribute DOM attribute node text is Boolean "false" or "0"
  @return true if "false" or "0"
  */
  bool is_false() const;
  /**
  @brief Return 64 bit integer value of numeric text of this xsd__anyAttribute DOM attribute node
  @return 64 bit integer value or 0 if text is not numeric
  */
  LONG64 get_int() const;
  /**
  @brief Return double float value of decimal text of this xsd__anyAttribute DOM attribute node
  @return double float value or NaN if text is not numeric
  */
  double get_double() const;
  /**
  @brief Return text of this xsd__anyAttribute DOM attribute node
  @return string or NULL
  */
  const char *get_text() const;
  /**
  @brief Return true if this xsd__anyAttribute DOM attribute node text is Boolean "true" or "1", same as is_true()
  @return true if "true" or "1"
  */
  operator bool() const;
  /**
  @brief Return integer value of numeric text of this xsd__anyAttribute DOM attribute node, same as (int)get_int()
  @return integer value or 0 if text is not numeric
  */
  operator int() const;
  /**
  @brief Return 64 bit integer value of numeric text of this xsd__anyAttribute DOM attribute node, same as get_int()
  @return 64 bit integer value or 0 if text is not numeric
  */
  operator LONG64() const;
  /**
  @brief Return double float value of decimal text of this xsd__anyAttribute DOM attribute node, same as get_double()
  @return double float value or NaN if text is not numeric
  */
  operator double() const;
  /**
  @brief Return text of this xsd__anyAttribute DOM attribute node, same as get_text()
  @return string or NULL
  */
  operator const char*() const;
  /**
  @brief Return const_iterator to begin of attribute nodes starting with this attribute, same as att_begin()
  @return xsd__anyAttribute::iterator
  */
  soap_dom_attribute::iterator cbegin();
  /**
  @brief Return const_iterator to end of attribute nodes, same as att_end()
  @return xsd__anyAttribute::iterator
  */
  soap_dom_attribute::iterator cend();
  /**
  @brief Return iterator to begin of attribute nodes starting with this attribute, same as att_begin()
  @return xsd__anyAttribute::iterator
  */
  soap_dom_attribute::iterator begin();
  /**
  @brief Return iterator to end of attribute nodes, same as att_end()
  @return xsd__anyAttribute::iterator
  */
  soap_dom_attribute::iterator end();
  /**
  @brief Return iterator to begin of attribute nodes starting with this attribute
  @return xsd__anyAttribute::iterator
  */
  soap_dom_attribute::iterator att_begin();
  /**
  @brief Return iterator to end of attribute nodes
  @return xsd__anyAttribute::iterator
  */
  soap_dom_attribute::iterator att_end();
  /**
  @brief Return iterator to search for matching attributes of this node, same as att_find(NULL, patt)
  @param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any)
  @return xsd__anyAttribute::iterator
  */
  soap_dom_attribute::iterator att_find(const char *patt);
  /**
  @brief Return iterator to search for matching attributes of this node, same as att_find(NULL, patt)
  @param patt (un)qualified tag name wide string pattern (NULL, "*", and "*:*" match any)
  @return xsd__anyAttribute::iterator
  */
  soap_dom_attribute::iterator att_find(const wchar_t *patt);
  /**
  @brief Return iterator to search for matching attributes of this node
  @param ns namespace URI string pattern ("*" matches any, "" matches the null (empty) namespace, NULL matches the null (empty) namespace if tag is unqualified or the namespace from namespace table if tag is qualified)
  @param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any)
  @return xsd__anyAttribute::iterator
  */
  soap_dom_attribute::iterator att_find(const char *ns, const char *patt);
  /**
  @brief Return iterator to search for matching attributes of this node
  @param ns namespace URI string pattern ("*" matches any, "" matches the null (empty) namespace, NULL matches the null (empty) namespace if tag is unqualified or the namespace from namespace table if tag is qualified)
  @param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any)
  @return xsd__anyAttribute::iterator
  */
  soap_dom_attribute::iterator att_find(const char *ns, const wchar_t *patt);
  void unlink();
};

/**
@brief The xsd__anyAttribute DOM attribute node structure (xsd__anyAttribute is a typedef of soap_dom_attribute)
*/
typedef soap_dom_attribute xsd__anyAttribute;

/**
@class soap_dom_element
@brief The xsd__anyType DOM element node structure (xsd__anyType is a typedef of soap_dom_element)
*/
struct soap_dom_element
{
  struct soap_dom_element *next;        ///< next sibling element node in list
  struct soap_dom_element *prnt;        ///< pointer to parent node
  struct soap_dom_element *elts;        ///< list of child element nodes
  struct soap_dom_attribute *atts;      ///< list of attribute nodes
  const char *nstr;                     ///< namespace string
  const char *name;                     ///< (un)qualified tag name
  const char *lead;                     ///< leading XML content before start tag (used with WITH_DOM)
  const char *text;                     ///< text cdata in UTF-8
  const char *code;                     ///< XML "code" (plain unconverted XML used with WTIH_DOM)
  const char *tail;                     ///< trailing XML content before end tag (used with WITH_DOM)
  const void *node;                     ///< pointer to serializable object
  int type;                             ///< SOAP_TYPE_T type of serializable object or 0
  struct soap *soap;                    ///< soap context that manages this object

  /** iterator over sibling element node list */
  typedef soap_dom_element_iterator iterator;

  /** const_iterator over sibling element node list */
  typedef soap_dom_element_iterator const_iterator;

  /**
  @brief Construct new xsd__anyType DOM element
  @param soap context that manages this object
  */
  soap_dom_element(struct soap *soap = NULL);
  /**
  @brief Construct new xsd__anyType DOM element by (shallow) copying another element
  @param elt xsd__anyType DOM element copied
  */
  soap_dom_element(const soap_dom_element& elt);
  /**
  @brief Construct new xsd__anyType DOM element with (un)qualified tag name, same as xsd__anyType(soap, NULL, tag)
  @param soap context that manages this object
  @param tag (un)qualified tag name string or NULL (unnamed node)
  */
  soap_dom_element(struct soap *soap, const char *tag);
  /**
  @brief Construct new xsd__anyType DOM element with (un)qualified tag name, same as xsd__anyType(soap, NULL, tag)
  @param soap context that manages this object
  @param tag (un)qualified tag name wide string or NULL (unnamed node)
  */
  soap_dom_element(struct soap *soap, const wchar_t *tag);
  /**
  @brief Construct new xsd__anyType DOM element with namespace URI (xmlns) and (un)qualified tag name
  @param soap context that manages this object
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name string or NULL (unnamed node)
  */
  soap_dom_element(struct soap *soap, const char *ns, const char *tag);
  /**
  @brief Construct new xsd__anyType DOM element with namespace URI (xmlns) and (un)qualified tag name
  @param soap context that manages this object
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name wide string or NULL (unnamed node)
  */
  soap_dom_element(struct soap *soap, const char *ns, const wchar_t *tag);
  /**
  @brief Construct new xsd__anyType DOM element with namespace URI (xmlns), (un)qualified tag name, and text
  @param soap context that manages this object
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name string or NULL (unnamed node)
  @param text string or NULL
  */
  soap_dom_element(struct soap *soap, const char *ns, const char *tag, const char *text);
  /**
  @brief Construct new xsd__anyType DOM element with namespace URI (xmlns), (un)qualified tag name, and text
  @param soap context that manages this object
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name string or NULL (unnamed node)
  @param text wide string or NULL
  */
  soap_dom_element(struct soap *soap, const char *ns, const char *tag, const wchar_t *text);
  /**
  @brief Construct new xsd__anyType DOM element with namespace URI (xmlns), (un)qualified tag name, and text
  @param soap context that manages this object
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name wide string or NULL (unnamed node)
  @param text string or NULL
  */
  soap_dom_element(struct soap *soap, const char *ns, const wchar_t *tag, const char *text);
  /**
  @brief Construct new xsd__anyType DOM element with namespace URI (xmlns), (un)qualified tag name, and text
  @param soap context that manages this object
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name wide string or NULL (unnamed node)
  @param text wide string or NULL
  */
  soap_dom_element(struct soap *soap, const char *ns, const wchar_t *tag, const wchar_t *text);
  /**
  @brief Construct new xsd__anyType DOM element with namespace URI (xmlns), (un)qualified tag name, and text
  @param soap context that manages this object
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name string or "" (unnamed node)
  @param text string
  */
  soap_dom_element(struct soap *soap, const char *ns, const char *tag, const std::string& text);
  /**
  @brief Construct new xsd__anyType DOM element with namespace URI (xmlns), (un)qualified tag name, and text
  @param soap context that manages this object
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name wide string or "" (unnamed node)
  @param text string
  */
  soap_dom_element(struct soap *soap, const char *ns, const char *tag, const std::wstring& text);
  /**
  @brief Construct new xsd__anyType DOM element with namespace URI (xmlns), (un)qualified tag name, and text
  @param soap context that manages this object
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name string or "" (unnamed node)
  @param text wide string
  */
  soap_dom_element(struct soap *soap, const char *ns, const wchar_t *tag, const std::string& text);
  /**
  @brief Construct new xsd__anyType DOM element with namespace URI (xmlns), (un)qualified tag name, and text
  @param soap context that manages this object
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name wide string or "" (unnamed node)
  @param text wide string
  */
  soap_dom_element(struct soap *soap, const char *ns, const wchar_t *tag, const std::wstring& text);
  /**
  @brief Construct new xsd__anyType DOM element node with namespace URI (xmlns), (un)qualified tag name, and pointer to a serializable object
  @param soap context that manages this object
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name string or NULL (unnamed node)
  @param node pointer to serializable object
  @param type SOAP_TYPE_T type of the serializable object of type T
  */
  soap_dom_element(struct soap *soap, const char *ns, const char *tag, const void *node, int type);
  /**
  @brief Construct new xsd__anyType DOM element node with namespace URI (xmlns), (un)qualified tag name, and pointer to a serializable object
  @param soap context that manages this object
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name wide string or NULL (unnamed node)
  @param node pointer to serializable object
  @param type SOAP_TYPE_T type of the serializable object of type T
  */
  soap_dom_element(struct soap *soap, const char *ns, const wchar_t *tag, const void *node, int type);
  /**
  @brief Destructor (no-op, deletes are done by the managing context of this object)
  */
  ~soap_dom_element();
  /**
  @brief Set xsd__anyType DOM element namespace URI and tag name
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name string or NULL (unnamed node)
  @return reference to *this
  */
  soap_dom_element& set(const char *ns, const char *tag);
  /**
  @brief Set xsd__anyType DOM element namespace URI and tag name
  @param ns namespace URI string or NULL
  @param tag (un)qualified tag name wide string or NULL (unnamed node)
  @return reference to *this
  */
  soap_dom_element& set(const char *ns, const wchar_t *tag);
  /**
  @brief Set text of this xsd__anyType DOM element node to Boolean "false" or "true"
  @param b Boolean value
  @return reference to *this
  */
  soap_dom_element& set(bool b);
  /**
  @brief Set text of this xsd__anyType DOM element node to an integer value
  @param n integer value
  @return reference to *this
  */
  soap_dom_element& set(int n);
  /**
  @brief Set text of this xsd__anyType DOM element node to an integer value
  @param n integer value
  @return reference to *this
  */
  soap_dom_element& set(LONG64 n);
  /**
  @brief Set text of this xsd__anyType DOM element node to a float value
  @param x float value (NaN and +/-INF are also supported)
  @return reference to *this
  */
  soap_dom_element& set(float x);
  /**
  @brief Set text of this xsd__anyType DOM element node to a double float value
  @param x double float value (NaN and +/-INF are also supported)
  @return reference to *this
  */
  soap_dom_element& set(double x);
  /**
  @brief Set text of this xsd__anyType DOM element node
  @param text string or NULL
  @return reference to *this
  */
  soap_dom_element& set(const char *text);
  /**
  @brief Set text of this xsd__anyType DOM element node
  @param text wide string or NULL
  @return reference to *this
  */
  soap_dom_element& set(const wchar_t *text);
  /**
  @brief Set text of this xsd__anyType DOM element node
  @param text string
  @return reference to *this
  */
  soap_dom_element& set(const std::string& text);
  /**
  @brief Set text of this xsd__anyType DOM element node
  @param text wide string
  @return reference to *this
  */
  soap_dom_element& set(const std::wstring& text);
  /**
  @brief Set this xsd__anyType DOM element node to point to a serializable object
  @param node pointer to serializable object
  @param type SOAP_TYPE_T type of the serializable object of type T
  @return reference to *this
  */
  soap_dom_element& set(const void *node, int type);
  /**
  @brief Add a child element node to this xsd__anyType DOM element node
  @param elt reference to element node to copy and add as a child
  @return reference to *this
  */
  soap_dom_element& add(soap_dom_element& elt);
  /**
  @brief Add a child element node to this xsd__anyType DOM element node
  @param elt pointer to element node to copy and add as a child
  @return reference to *this
  */
  soap_dom_element& add(soap_dom_element *elt);
  /**
  @brief Add an attribute node to this xsd__anyType DOM element node
  @param att reference to xsd__anyAttribute attribute node to copy and add
  @return reference to *this
  */
  soap_dom_element& add(soap_dom_attribute& att);
  /**
  @brief Add an attribute node to this xsd__anyType DOM element node
  @param att pointer to xsd__anyAttribute attribute node to copy and add
  @return reference to *this
  */
  soap_dom_element& add(soap_dom_attribute *att);
  /**
  @brief Copy list of element nodes to this xsd__anyType DOM element node as children
  @param elts reference to list of xsd__anyType DOM element nodes to copy and add as children
  @return reference to *this
  */
  soap_dom_element& adds(soap_dom_element& elts);
  /**
  @brief Copy list of element nodes to this xsd__anyType DOM element node as children
  @param elts pointer to list of xsd__anyType DOM element nodes to copy and add as children
  @return reference to *this
  */
  soap_dom_element& adds(soap_dom_element *elts);
  /**
  @brief Copy list of attribute nodes to this xsd__anyType DOM element node
  @param atts reference to list of xsd__anyAttribute attribute nodes to copy and add
  @return reference to *this
  */
  soap_dom_element& adds(soap_dom_attribute& atts);
  /**
  @brief Copy list of attribute nodes to this xsd__anyType DOM element node
  @param atts pointer to list of xsd__anyAttribute attribute nodes to copy and add
  @return reference to *this
  */
  soap_dom_element& adds(soap_dom_attribute *atts);
  /**
  @brief Set text of this xsd__anyType DOM element node to Boolean "false" or "true", same as set(b)
  @param b Boolean value
  @return reference to *this
  */
  soap_dom_element& operator=(bool b);
  /**
  @brief Set text of this xsd__anyType DOM element node to an integer value, same as set(n)
  @param n integer value
  @return reference to *this
  */
  soap_dom_element& operator=(int n);
  /**
  @brief Set text of this xsd__anyType DOM element node to an integer value, same as set(n)
  @param n 64 bit integer value
  @return reference to *this
  */
  soap_dom_element& operator=(LONG64 n);
  /**
  @brief Set text of this xsd__anyType DOM element node to a float value, same as set(x)
  @param x float value (NaN and +/-INF are also supported)
  @return reference to *this
  */
  soap_dom_element& operator=(float x);
  /**
  @brief Set text of this xsd__anyType DOM element node to a double float value, same as set(x)
  @param x double float value (NaN and +/-INF are also supported)
  @return reference to *this
  */
  soap_dom_element& operator=(double x);
  /**
  @brief Set text of this xsd__anyType DOM element node, same as set(text)
  @param text string or NULL
  @return reference to *this
  */
  soap_dom_element& operator=(const char *text);
  /**
  @brief Set text of this xsd__anyType DOM element node, same as set(text)
  @param text wide string or NULL
  @return reference to *this
  */
  soap_dom_element& operator=(const wchar_t *text);
  /**
  @brief Set text of this xsd__anyType DOM element node, same as set(text)
  @param text string
  @return reference to *this
  */
  soap_dom_element& operator=(const std::string& text);
  /**
  @brief Set text of this xsd__anyType DOM element node, same as set(text)
  @param text wide string
  @return reference to *this
  */
  soap_dom_element& operator=(const std::wstring& text);
  /**
  @brief Copy an xsd__anyType DOM element node to this node (not a deep copy)
  @param elt reference to xsd__anyType DOM element node to copy from
  @return reference to *this
  */
  soap_dom_element& operator=(const soap_dom_element& elt);
  /**
  @brief Set this xsd__anyType DOM element node to point to a serializable object
  @param obj reference to serializable object that has a soap_type() member
  @return reference to *this
  */
  template<class T> soap_dom_element& operator=(const T& obj);
  /**
  @brief Set this xsd__anyType DOM element node to point to a serializable object
  @param obj pointer to serializable object that has a soap_type() member
  @return reference to *this
  */
  template<class T> soap_dom_element& operator=(const T *obj);
  /**
  @brief Set this xsd__anyType DOM element node to point to a serializable object
  @param obj pointer to serializable object that has a soap_type() member
  @return reference to *this
  */
  template<class T> soap_dom_element& operator=(T *obj);
  /**
  @brief Populate this xsd__anyType DOM element node with an attribute node, same as att(NULL, tag), if the attribute does not already exists
  @param tag (un)qualified tag name string of attribute
  @return reference to xsd__anyAttribute attribute node (new attribute node if none matches)
  */
  soap_dom_attribute& att(const char *tag);
  /**
  @brief Populate this xsd__anyType DOM element node with an attribute node, same as att(NULL, tag), if the attribute does not already exists
  @param tag (un)qualified tag name wide string of attribute
  @return reference to xsd__anyAttribute attribute node (new attribute node if none matches)
  */
  soap_dom_attribute& att(const wchar_t *tag);
  /**
  @brief Populate this xsd__anyType DOM element node with an attribute node, if the attribute does not already exists
  @param ns namespace URI string or NULL of attribute
  @param tag (un)qualified tag name string of attribute
  @return reference to xsd__anyAttribute attribute node (new attribute node if none matches)
  */
  soap_dom_attribute& att(const char *ns, const char *tag);
  /**
  @brief Populate this xsd__anyType DOM element node with an attribute node, if the attribute does not already exists
  @param ns namespace URI string or NULL of attribute
  @param tag (un)qualified tag name wide string of attribute
  @return reference to xsd__anyAttribute attribute node (new attribute node if none matches)
  */
  soap_dom_attribute& att(const char *ns, const wchar_t *tag);
  /**
  @brief Populate this xsd__anyType DOM element node with an unnamed (tagless) child element node, same as elt(NULL, "")
  @return reference to child element node (new child element node if none matches)
  */
  soap_dom_element& elt();
  /**
  @brief Populate this xsd__anyType DOM element node with a child element node, same as elt(NULL, tag)
  @param tag (un)qualified tag name string of child element or NULL (unnamed node)
  @return reference to child element node (new child element node if none matches)
  */
  soap_dom_element& elt(const char *tag);
  /**
  @brief Populate this xsd__anyType DOM element node with a child element node, same as elt(NULL, tag)
  @param tag (un)qualified tag name wide string of child element or NULL (unnamed node)
  @return reference to child element node (new child element node if none matches)
  */
  soap_dom_element& elt(const wchar_t *tag);
  /**
  @brief Populate this xsd__anyType DOM element node with a child element node
  @param ns namespace URI string or NULL of child element
  @param tag (un)qualified tag name string of child element or NULL (unnamed node)
  @return reference to child element node (new child element node if none matches)
  */
  soap_dom_element& elt(const char *ns, const char *tag);
  /**
  @brief Populate this xsd__anyType DOM element node with a child element node
  @param ns namespace URI string or NULL of child element
  @param tag (un)qualified tag name wide string of child element or NULL (unnamed node)
  @return reference to child element node (new child element node if none matches)
  */
  soap_dom_element& elt(const char *ns, const wchar_t *tag);
  /**
  @brief Populate this xsd__anyType DOM element node with a child element node, same as elt(NULL, tag)
  @param tag (un)qualified tag name wide string of child element or NULL (unnamed node)
  @return reference to child element node (new child element node if none matches)
  */
  soap_dom_element& operator[](const char *tag);
  /**
  @brief Populate this xsd__anyType DOM element node with a child element node, same as elt(NULL, tag)
  @param tag (un)qualified tag name wide string of child element or NULL (unnamed node)
  @return reference to child element node (new child element node if none matches)
  */
  soap_dom_element& operator[](const wchar_t *tag);
  /**
  @brief Add a N-th child element with the same namespace and tag name as this xsd__anyType DOM element node that is a child element node at position one (1) in a sibling list
  @param n additional child element position requested, counting from one (1) XPath style
  @return reference to xsd__anyType child element node (new child element node if none exists at position n)
  */
  soap_dom_element& operator[](size_t n);
  /**
  @brief Get attribute node of this xsd__anyType DOM element node, if attribute exists, same as att_get(NULL, tag)
  @param tag (un)qualified tag name string of attribute
  @return pointer to xsd__anyAttribute DOM attribute node or NULL if not exists
  */
  soap_dom_attribute *att_get(const char *tag) const;
  /**
  @brief Get attribute node of this xsd__anyType DOM element node, if attribute exists, same as att_get(NULL, tag)
  @param tag (un)qualified tag name wide string of attribute
  @return pointer to xsd__anyAttribute DOM attribute node or NULL if not exists
  */
  soap_dom_attribute *att_get(const wchar_t *tag) const;
  /**
  @brief Get attribute node of this xsd__anyType DOM element node, if attribute exists
  @param ns namespace URI string (NULL matches the null (empty) namespace if tag is unqualified) of attribute
  @param tag (un)qualified tag name string of attribute
  @return pointer to xsd__anyAttribute DOM attribute node or NULL if not exists
  */
  soap_dom_attribute *att_get(const char *ns, const char *tag) const;
  /**
  @brief Get attribute node of this xsd__anyType DOM element node, if attribute exists
  @param ns namespace URI string (NULL matches the null (empty) namespace if tag is unqualified) of attribute
  @param tag (un)qualified tag name wide string of attribute
  @return pointer to xsd__anyAttribute DOM attribute node or NULL if not exists
  */
  soap_dom_attribute *att_get(const char *ns, const wchar_t *tag) const;
  /**
  @brief Get unnamed (tagless) child element node of this xsd__anyType DOM element node, if child element exists, same as elt_get(NULL, NULL) or elt_get(NULL, "");
  @return pointer to xsd__anyType DOM child element node or NULL if not exists
  */
  soap_dom_element *elt_get() const;
  /**
  @brief Get child element node of this xsd__anyType DOM element node, if child element exists
  @param tag (un)qualified tag name string (NULL matches unnamed node) of child element
  @return pointer to xsd__anyType DOM child element node or NULL if not exists
  */
  soap_dom_element *elt_get(const char *tag) const;
  /**
  @brief Get child element node of this xsd__anyType DOM element node, if child element exists, same as elt_get(NULL, tag)
  @param tag (un)qualified tag name wide string (NULL matches unnamed node) of child element
  @return pointer to xsd__anyType DOM child element node or NULL if not exists
  */
  soap_dom_element *elt_get(const wchar_t *tag) const;
  /**
  @brief Get child element node of this xsd__anyType DOM element node, if child element exists
  @param ns namespace URI string (NULL matches the null (empty) namespace if tag is unqualified) of child element
  @param tag (un)qualified tag name string (NULL matches unnamed node) of child element
  @return pointer to xsd__anyType DOM child element node or NULL if not exists
  */
  soap_dom_element *elt_get(const char *ns, const char *tag) const;
  /**
  @brief Get child element node of this xsd__anyType DOM element node, if child element exists
  @param ns namespace URI string (NULL matches the null (empty) namespace if tag is unqualified) of child element
  @param tag (un)qualified tag name wide string (NULL matches unnamed node) of child element
  @return pointer to xsd__anyType DOM child element node or NULL if not exists
  */
  soap_dom_element *elt_get(const char *ns, const wchar_t *tag) const;
  /**
  @brief Get next child element node that has the same namespace URI and tag name as this child element node in a sibling list
  @return pointer to xsd__anyType DOM child element node or NULL if not exists
  */
  soap_dom_element *get_next() const;
  /**
  @brief Get N-th child element node that has the same namespace URI and tag name as this child element node at position one (1) in a sibling list
  @param n N-th element minus one located from the current element node at position one (1), 1 returns this element node
  @return pointer to xsd__anyType DOM child element node or NULL if not exists
  */
  soap_dom_element *get_nth(size_t n);
  /**
  @brief Match the namespace URI and tag name of this xsd__anyType DOM element node against a pattern
  @param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any, "" matches unnamed node)
  @return true if match
  */
  bool match(const char *patt) const;
  /**
  @brief Match the namespace URI and tag name of this xsd__anyType DOM element node against a pattern, same as match(NULL, patt)
  @param patt (un)qualified tag name wide string pattern (NULL, "*", and "*:*" match any, "" matches unnamed node)
  @return true if match
  */
  bool match(const wchar_t *patt) const;
  /**
  @brief Match the namespace URI and tag name of this xsd__anyType DOM element node against a pattern, same as match(NULL, patt)
  @param ns namespace URI string pattern ("*" matches any, NULL and "" match the null (empty) namespace)
  @param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any, "" matches unnamed node)
  @return true if match
  */
  bool match(const char *ns, const char *patt) const;
  /**
  @brief Match the namespace URI and tag name of this xsd__anyType DOM element node against a pattern
  @param ns namespace URI string pattern ("*" matches any, NULL and "" match the null (empty) namespace)
  @param patt (un)qualified tag name wide string pattern (NULL, "*", and "*:*" match any, "" matches unnamed node)
  @return true if match
  */
  bool match(const char *ns, const wchar_t *patt) const;
  /**
  @brief Get the namespace URI of this xsd__anyType DOM element node
  @return namespace URI string or NULL
  */
  const char *ns() const;
  /**
  @brief Get the tag name of this xsd__anyType DOM element node
  @return tag name string or NULL
  */
  const char *tag() const;
  /**
  @brief Return pointer to parent of this xsd__anyType DOM element node, if any
  @return pointer to xsd__anyType DOM element or NULL
  */
  soap_dom_element *parent();
  /**
  @brief Return depth from the root node of this xsd__anyType DOM element node
  @return depth from root node, zero if node is a root node and has no parent
  */
  size_t depth() const;
  /**
  @brief Return child index of this xsd__anyType DOM child element node in its sibling list
  @return nonzero index of child among siblings, or 0 if node is a root node and has no parent
  */
  size_t index() const;
  /**
  @brief Return number of siblings of this xsd__anyType DOM child node that have the same namespace URI and tag name
  @return number of siblings plus one (for self) that have the same namespace URI and tag name
  */
  size_t len() const;
  /**
  @brief Return index of this xsd__anyType DOM child node among siblings that have the same namespace URI and tag name
  @return nonzero N-th index (1 <= nth <= len), or 0 if element is root or is singular (has no siblings with the same namespace URI and tag name)
  */
  size_t nth() const;
  /**
  @brief Return number of child element nodes of this xsd__anyType DOM element node, same as elt_size(NULL, NULL)
  @return number of child elements
  */
  size_t elt_size();
  /**
  @brief Return number of matching child element nodes of this xsd__anyType DOM element node, same as elt_size(NULL, patt, type)
  @param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any, "" matches unnamed node)
  @param type optional SOAP_TYPE_T type of deserialized object of type T to match or 0
  @return number of matches found
  */
  size_t elt_size(const char *patt, int type = 0);
  /**
  @brief Return number of matching child element nodes of this xsd__anyType DOM element node
  @param ns namespace URI string pattern ("*" matches any, "" matches the null (empty) namespace, NULL matches the null (empty) namespace if tag is unqualified or the namespace from namespace table if tag is qualified)
  @param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any, "" matches unnamed node)
  @param type optional SOAP_TYPE_T type of deserialized object of type T to match or 0
  @return number of matches found
  */
  size_t elt_size(const char *ns, const char *patt, int type = 0);
  /**
  @brief Return number of attribute nodes of this xsd__anyType DOM element node, same as att_size(NULL, NULL)
  @return number of attributes
  */
  size_t att_size();
  /**
  @brief Return number of matching attribute nodes of this xsd__anyType DOM element node, same as att_size(NULL, patt)
  @param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any)
  @return number of matches found
  */
  size_t att_size(const char *patt);
  /**
  @brief Return number of matching attribute nodes of this xsd__anyType DOM element node
  @param ns namespace URI string pattern ("*" matches any, "" matches the null (empty) namespace, NULL matches the null (empty) namespace if tag is unqualified or the namespace from namespace table if tag is qualified)
  @param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any)
  @return number of matches found
  */
  size_t att_size(const char *ns, const char *patt);
  /**
  @brief Return true if this xsd__anyType DOM element node text is Boolean "true" or "1"
  @return true if "true" or "1"
  */
  bool is_true() const;
  /**
  @brief Return true if this xsd__anyType DOM element node text is Boolean "false" or "0"
  @return true if "false" or "0"
  */
  bool is_false() const;
  /**
  @brief Return 64 bit integer value of numeric text of this xsd__anyType DOM element node
  @return 64 bit integer value or 0 if text is not numeric
  */
  LONG64 get_int() const;
  /**
  @brief Return double float value of decimal text of this xsd__anyType DOM element node
  @return double float value or NaN if text is not numeric
  */
  double get_double() const;
  /**
  @brief Return text of this xsd__anyType DOM element node
  @return string or NULL
  */
  const char *get_text() const;
  /**
  @brief Return true if this xsd__anyType DOM element node text is Boolean "true" or "1", same as is_true()
  @return true if "true" or "1"
  */
  operator bool() const;
  /**
  @brief Return integer value of numeric text of this xsd__anyType DOM element node, same as (int)get_int()
  @return integer value or 0 if text is not numeric
  */
  operator int() const;
  /**
  @brief Return 64 bit integer value of numeric text of this xsd__anyType DOM element node, same as get_int()
  @return 64 bit integer value or 0 if text is not numeric
  */
  operator LONG64() const;
  /**
  @brief Return double float value of decimal text of this xsd__anyType DOM element node, same as get_double()
  @return double float value or NaN if text is not numeric
  */
  operator double() const;
  /**
  @brief Return text of this xsd__anyType DOM element node, same as get_text()
  @return string or NULL
  */
  operator const char*() const;
  /**
  @brief Return const_iterator to begin of deep depth-first node graph traversal starting with this node
  @return xsd__anyType::iterator
  */
  soap_dom_element::const_iterator cbegin();
  /**
  @brief Return const_iterator to end of deep depth-first tnode graph traversal
  @return xsd__anyType::iterator
  */
  soap_dom_element::const_iterator cend();
  /**
  @brief Return iterator to begin of deep depth-first node graph traversal starting with this node
  @return xsd__anyType::iterator
  */
  soap_dom_element::iterator begin();
  /**
  @brief Return iterator to end of deep depth-first tnode graph traversal
  @return xsd__anyType::iterator
  */
  soap_dom_element::iterator end();
  /**
  @brief Return iterator to begin of child element nodes
  @return xsd__anyType::iterator
  */
  soap_dom_element::iterator elt_begin();
  /**
  @brief Return iterator to end of child element nodes
  @return xsd__anyType::iterator
  */
  soap_dom_element::iterator elt_end();
  /**
  @brief Return iterator to begin of attribute nodes
  @return xsd__anyAttribute::iterator
  */
  soap_dom_attribute::iterator att_begin();
  /**
  @brief Return iterator to end of attribute nodes
  @return xsd__anyAttribute::iterator
  */
  soap_dom_attribute::iterator att_end();
  /**
  @brief Return iterator to search deep depth-first over node graph starting from this node, same as find(NULL, patt, type)
  @param patt (un)qualified tag name string pattern (use '@' to match attributes, NULL, "*", and "*:*" match any, "" matches unnamed node)
  @param type optional SOAP_TYPE_T type of deserialized object of type T to match or 0
  @return xsd__anyType::iterator
  */
  soap_dom_element::iterator find(const char *patt, int type = 0);
  /**
  @brief Return iterator to search deep depth-first over node graph starting from this node, same as find(NULL, patt, type)
  @param patt (un)qualified tag name wide string pattern (use '@' to match attributes, NULL, "*", and "*:*" match any, "" matches unnamed node)
  @param type optional SOAP_TYPE_T type of deserialized object of type T to match or 0
  @return xsd__anyType::iterator
  */
  soap_dom_element::iterator find(const wchar_t *patt, int type = 0);
  /**
  @brief Return iterator to search deep depth-first over node graph starting from this node
  @param ns namespace URI string pattern ("*" matches any, "" matches the null (empty) namespace, NULL matches the null (empty) namespace if tag is unqualified or the namespace from namespace table if tag is qualified)
  @param patt (un)qualified tag name string pattern (use '@' to match attributes, NULL, "*", and "*:*" match any, "" matches unnamed node)
  @param type optional SOAP_TYPE_T type of deserialized object of type T to match or 0
  @return xsd__anyType::iterator
  */
  soap_dom_element::iterator find(const char *ns, const char *patt, int type = 0);
  /**
  @brief Return iterator to search deep depth-first over node graph starting from this node
  @param ns namespace URI string pattern ("*" matches any, "" matches the null (empty) namespace, NULL matches the null (empty) namespace if tag is unqualified or the namespace from namespace table if tag is qualified)
  @param patt (un)qualified tag name wide string pattern (use '@' to match attributes, NULL, "*", and "*:*" match any, "" matches unnamed node)
  @param type optional SOAP_TYPE_T type of deserialized object of type T to match or 0
  @return xsd__anyType::iterator
  */
  soap_dom_element::iterator find(const char *ns, const wchar_t *patt, int type = 0);
  /**
  @brief Return iterator to search deep depth-first over node graph to find deserialized objects, starting from this node
  @param type optional SOAP_TYPE_T type of deserialized object of type T to match or 0
  @return xsd__anyType::iterator
  */
  soap_dom_element::iterator find(int type);
  /**
  @brief Return iterator to search for matching child elements of this node, same as elt_find(NULL, patt, type)
  @param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any, "" matches unnamed node)
  @param type optional SOAP_TYPE_T type of deserialized object of type T to match or 0
  @return xsd__anyType::iterator
  */
  soap_dom_element::iterator elt_find(const char *patt, int type = 0);
  /**
  @brief Return iterator to search for matching child elements of this node, same as elt_find(NULL, patt, type)
  @param patt (un)qualified tag name wide string pattern (NULL, "*", and "*:*" match any, "" matches unnamed node)
  @param type optional SOAP_TYPE_T type of deserialized object of type T to match or 0
  @return xsd__anyType::iterator
  */
  soap_dom_element::iterator elt_find(const wchar_t *patt, int type = 0);
  /**
  @brief Return iterator to search for matching child elements of this node
  @param ns namespace URI string pattern ("*" matches any, "" matches the null (empty) namespace, NULL matches the null (empty) namespace if tag is unqualified or the namespace from namespace table if tag is qualified)
  @param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any, "" matches unnamed node)
  @param type optional SOAP_TYPE_T type of deserialized object of type T to match or 0
  @return xsd__anyType::iterator
  */
  soap_dom_element::iterator elt_find(const char *ns, const char *patt, int type = 0);
  /**
  @brief Return iterator to search for matching child elements of this node
  @param ns namespace URI string pattern ("*" matches any, "" matches the null (empty) namespace, NULL matches the null (empty) namespace if tag is unqualified or the namespace from namespace table if tag is qualified)
  @param patt (un)qualified tag name wide string pattern (NULL, "*", and "*:*" match any, "" matches unnamed node)
  @param type optional SOAP_TYPE_T type of deserialized object of type T to match or 0
  @return xsd__anyType::iterator
  */
  soap_dom_element::iterator elt_find(const char *ns, const wchar_t *patt, int type = 0);
  /**
  @brief Return iterator to search for child elements of this node that have deserialized objects
  @param type SOAP_TYPE_T type of deserialized object of type T to match or 0
  @return xsd__anyType::iterator
  */
  soap_dom_element::iterator elt_find(int type);
  /**
  @brief Return iterator to search for matching attributes of this node
  @param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any, "" matches unnamed node)
  @return xsd__anyAttribute::iterator
  */
  soap_dom_attribute::iterator att_find(const char *patt);
  /**
  @brief Return iterator to search for matching attributes of this node, same as att_find(NULL, patt)
  @param patt (un)qualified tag name wide string pattern (NULL, "*", and "*:*" match any)
  @return xsd__anyAttribute::iterator
  */
  soap_dom_attribute::iterator att_find(const wchar_t *patt);
  /**
  @brief Return iterator to search for matching attributes of this node, same as att_find(NULL, patt)
  @param ns namespace URI string pattern ("*" matches any, "" matches the null (empty) namespace, NULL matches the null (empty) namespace if tag is unqualified or the namespace from namespace table if tag is qualified)
  @param patt (un)qualified tag name string pattern (NULL, "*", and "*:*" match any)
  @return xsd__anyAttribute::iterator
  */
  soap_dom_attribute::iterator att_find(const char *ns, const char *patt);
  /**
  @brief Return iterator to search for matching attributes of this node
  @param ns namespace URI string pattern ("*" matches any, "" matches the null (empty) namespace, NULL matches the null (empty) namespace if tag is unqualified or the namespace from namespace table if tag is qualified)
  @param patt (un)qualified tag name wide string pattern (NULL, "*", and "*:*" match any)
  @return xsd__anyAttribute::iterator
  */
  soap_dom_attribute::iterator att_find(const char *ns, const wchar_t *patt);
  void unlink();
};

/**
@brief The xsd__anyType DOM element node structure (xsd__anyType is a typedef of soap_dom_element)
*/
typedef soap_dom_element xsd__anyType;

/**
@brief Write XML to current output (this is an auto-generted macro in soapH.h)
@param soap context that manages IO
@param dom root element node of the XML document
@return SOAP_OK or an error code
*/
int soap_write_xsd__anyType(struct soap *soap, xsd__anyType *dom);

/**
@brief Parse XML from current input (this is an auto-generted macro in soapH.h)
@param soap context that manages IO
@param dom root element node of the XML document
@return SOAP_OK or an error code
*/
int soap_read_xsd__anyType(struct soap *soap, xsd__anyType *dom);

#endif

/******************************************************************************\
 *
 *      soap_dom_element class
 *
\******************************************************************************/

soap_dom_element::soap_dom_element(struct soap *soap)
{
  soap_default_xsd__anyType(soap, this);
}

/******************************************************************************/

soap_dom_element::soap_dom_element(const soap_dom_element& elt)
{
  soap_default_xsd__anyType(elt.soap, this);
  (void)soap_elt_copy(this, &elt);
}

/******************************************************************************/

soap_dom_element::soap_dom_element(struct soap *soap, const char *tag)
{
  soap_default_xsd__anyType(soap, this);
  (void)soap_elt_set(this, NULL, tag);
}

/******************************************************************************/

soap_dom_element::soap_dom_element(struct soap *soap, const wchar_t *tag)
{
  soap_default_xsd__anyType(soap, this);
  (void)soap_elt_set_w(this, NULL, tag);
}

/******************************************************************************/

soap_dom_element::soap_dom_element(struct soap *soap, const char *ns, const char *tag)
{
  soap_default_xsd__anyType(soap, this);
  (void)soap_elt_set(this, ns, tag);
}

/******************************************************************************/

soap_dom_element::soap_dom_element(struct soap *soap, const char *ns, const wchar_t *tag)
{
  soap_default_xsd__anyType(soap, this);
  (void)soap_elt_set_w(this, ns, tag);
}

/******************************************************************************/

soap_dom_element::soap_dom_element(struct soap *soap, const char *ns, const char *tag, const char *str)
{
  soap_default_xsd__anyType(soap, this);
  (void)soap_elt_text(soap_elt_set(this, ns, tag), str);
}

/******************************************************************************/

soap_dom_element::soap_dom_element(struct soap *soap, const char *ns, const char *tag, const wchar_t *str)
{
  soap_default_xsd__anyType(soap, this);
  (void)soap_elt_text_w(soap_elt_set(this, ns, tag), str);
}

/******************************************************************************/

soap_dom_element::soap_dom_element(struct soap *soap, const char *ns, const wchar_t *tag, const char *str)
{
  soap_default_xsd__anyType(soap, this);
  (void)soap_elt_text(soap_elt_set_w(this, ns, tag), str);
}

/******************************************************************************/

soap_dom_element::soap_dom_element(struct soap *soap, const char *ns, const wchar_t *tag, const wchar_t *str)
{
  soap_default_xsd__anyType(soap, this);
  (void)soap_elt_text_w(soap_elt_set_w(this, ns, tag), str);
}

/******************************************************************************/

#ifndef WITH_COMPAT
soap_dom_element::soap_dom_element(struct soap *soap, const char *ns, const char *tag, const std::string& str)
{
  soap_default_xsd__anyType(soap, this);
  (void)soap_elt_text(soap_elt_set(this, ns, tag), str.c_str());
}
#endif

/******************************************************************************/

#ifndef WITH_COMPAT
soap_dom_element::soap_dom_element(struct soap *soap, const char *ns, const wchar_t *tag, const std::string& str)
{
  soap_default_xsd__anyType(soap, this);
  (void)soap_elt_text(soap_elt_set_w(this, ns, tag), str.c_str());
}
#endif

/******************************************************************************/

#ifndef WITH_COMPAT
soap_dom_element::soap_dom_element(struct soap *soap, const char *ns, const char *tag, const std::wstring& str)
{
  soap_default_xsd__anyType(soap, this);
  (void)soap_elt_text_w(soap_elt_set(this, ns, tag), str.c_str());
}
#endif

/******************************************************************************/

#ifndef WITH_COMPAT
soap_dom_element::soap_dom_element(struct soap *soap, const char *ns, const wchar_t *tag, const std::wstring& str)
{
  soap_default_xsd__anyType(soap, this);
  (void)soap_elt_text_w(soap_elt_set_w(this, ns, tag), str.c_str());
}
#endif

/******************************************************************************/

soap_dom_element::soap_dom_element(struct soap *soap, const char *ns, const char *tag, const void *nod, int typ)
{
  soap_default_xsd__anyType(soap, this);
  (void)soap_elt_node(soap_elt_set(this, ns, tag), nod, typ);
}

/******************************************************************************/

soap_dom_element::soap_dom_element(struct soap *soap, const char *ns, const wchar_t *tag, const void *nod, int typ)
{
  soap_default_xsd__anyType(soap, this);
  (void)soap_elt_node(soap_elt_set_w(this, ns, tag), nod, typ);
}

/******************************************************************************/

soap_dom_element::~soap_dom_element()
{ }

/******************************************************************************/

soap_dom_element::iterator soap_dom_element::begin()
{
  soap_dom_element::iterator iter = soap_dom_element::iterator(this);
  iter.stop = this;
  iter.deep = true;
  return iter;
}

/******************************************************************************/

soap_dom_element::iterator soap_dom_element::find(const char *ns, const char *pat, int typ)
{
  soap_dom_element::iterator iter(soap_dom_find(this, this, ns, pat, typ));
  iter.stop = this;
  iter.nstr = ns;
  iter.name = pat;
  iter.type = typ;
  iter.deep = true;
  return iter;
}

/******************************************************************************/

soap_dom_element::iterator soap_dom_element::find(const char *ns, const wchar_t *tag, int typ)
{
  const char *s = soap_wchar2s(NULL, tag);
  soap_dom_element::iterator iter = this->find(ns, s, typ);
  if (s)
    free((void*)s);
  return iter;
}

/******************************************************************************/

soap_dom_element::iterator soap_dom_element::find(int typ)
{
  return this->find((const char*)NULL, (const char*)NULL, typ);
}

/******************************************************************************/

soap_dom_element::iterator soap_dom_element::elt_find(const char *ns, const char *pat, int typ)
{
  soap_dom_element::iterator iter(soap_elt_find_type(this, ns, pat, typ));
  iter.nstr = ns;
  iter.name = pat;
  iter.type = typ;
  return iter;
}

/******************************************************************************/

soap_dom_element::iterator soap_dom_element::elt_find(const char *ns, const wchar_t *pat, int typ)
{
  const char *s = soap_wchar2s(NULL, pat);
  soap_dom_element::iterator iter = this->elt_find(ns, s, typ);
  if (s)
    free((void*)s);
  return iter;
}

/******************************************************************************/

soap_dom_element::iterator soap_dom_element::elt_find(int typ)
{
  return this->elt_find((const char*)NULL, (const char*)NULL, typ);
}

/******************************************************************************/

soap_dom_attribute::iterator soap_dom_element::att_find(const char *ns, const char *pat)
{
  soap_dom_attribute::iterator iter(soap_att_find(this, ns, pat));
  iter.nstr = ns;
  iter.name = pat;
  return iter;
}

/******************************************************************************/

soap_dom_attribute::iterator soap_dom_element::att_find(const char *ns, const wchar_t *pat)
{
  const char *s = soap_wchar2s(NULL, pat);
  soap_dom_attribute::iterator iter = this->att_find(ns, s);
  if (s)
    free((void*)s);
  return iter;
}

/******************************************************************************/

void soap_dom_element::unlink()
{
  soap_unlink(soap, this);
  soap_unlink(soap, nstr);
  soap_unlink(soap, name);
  soap_unlink(soap, text);
  if (elts)
    elts->unlink();
  if (atts)
    atts->unlink();
  if (next)
    next->unlink();
  node = NULL;
  type = 0;
}

/******************************************************************************\
 *
 *      soap_dom_attribute class
 *
\******************************************************************************/

soap_dom_attribute::soap_dom_attribute(struct soap *soap)
{
  soap_default_xsd__anyAttribute(soap, this);
}

/******************************************************************************/

soap_dom_attribute::soap_dom_attribute(const soap_dom_attribute& att)
{
  soap_default_xsd__anyAttribute(att.soap, this);
  (void)soap_att_copy(this, &att);
}

/******************************************************************************/

soap_dom_attribute::soap_dom_attribute(struct soap *soap, const char *tag)
{
  soap_default_xsd__anyAttribute(soap, this);
  (void)soap_att_set(this, NULL, tag);
}

/******************************************************************************/

soap_dom_attribute::soap_dom_attribute(struct soap *soap, const wchar_t *tag)
{
  soap_default_xsd__anyAttribute(soap, this);
  (void)soap_att_set_w(this, NULL, tag);
}

/******************************************************************************/

soap_dom_attribute::soap_dom_attribute(struct soap *soap, const char *ns, const char *tag, const char *str)
{
  soap_default_xsd__anyAttribute(soap, this);
  (void)soap_att_text(soap_att_set(this, ns, tag), str);
}

/******************************************************************************/

soap_dom_attribute::soap_dom_attribute(struct soap *soap, const char *ns, const char *tag, const wchar_t *str)
{
  soap_default_xsd__anyAttribute(soap, this);
  (void)soap_att_text_w(soap_att_set(this, ns, tag), str);
}

/******************************************************************************/

soap_dom_attribute::soap_dom_attribute(struct soap *soap, const char *ns, const wchar_t *tag, const char *str)
{
  soap_default_xsd__anyAttribute(soap, this);
  (void)soap_att_text(soap_att_set_w(this, ns, tag), str);
}

/******************************************************************************/

soap_dom_attribute::soap_dom_attribute(struct soap *soap, const char *ns, const wchar_t *tag, const wchar_t *str)
{
  soap_default_xsd__anyAttribute(soap, this);
  (void)soap_att_text_w(soap_att_set_w(this, ns, tag), str);
}

/******************************************************************************/

#ifndef WITH_COMPAT
soap_dom_attribute::soap_dom_attribute(struct soap *soap, const char *ns, const char *tag, const std::string& str)
{
  soap_default_xsd__anyAttribute(soap, this);
  (void)soap_att_text(soap_att_set(this, ns, tag), str.c_str());
}
#endif

/******************************************************************************/

#ifndef WITH_COMPAT
soap_dom_attribute::soap_dom_attribute(struct soap *soap, const char *ns, const char *tag, const std::wstring& str)
{
  soap_default_xsd__anyAttribute(soap, this);
  (void)soap_att_text_w(soap_att_set(this, ns, tag), str.c_str());
}
#endif

/******************************************************************************/

#ifndef WITH_COMPAT
soap_dom_attribute::soap_dom_attribute(struct soap *soap, const char *ns, const wchar_t *tag, const std::string& str)
{
  soap_default_xsd__anyAttribute(soap, this);
  (void)soap_att_text(soap_att_set_w(this, ns, tag), str.c_str());
}
#endif

/******************************************************************************/

#ifndef WITH_COMPAT
soap_dom_attribute::soap_dom_attribute(struct soap *soap, const char *ns, const wchar_t *tag, const std::wstring& str)
{
  soap_default_xsd__anyAttribute(soap, this);
  (void)soap_att_text_w(soap_att_set_w(this, ns, tag), str.c_str());
}
#endif

/******************************************************************************/

soap_dom_attribute::~soap_dom_attribute()
{ }

/******************************************************************************/

soap_dom_attribute::iterator soap_dom_attribute::att_find(const char *ns, const char *pat)
{
  soap_dom_attribute::iterator iter(this);
  iter.nstr = ns;
  iter.name = pat;
  if (pat)
  {
    if (!soap_patt_match(this->name, pat))
      return ++iter;
    if (ns && (!this->nstr || strcmp(this->nstr, ns)))
      return ++iter;
    if (!ns && this->nstr)
      return ++iter;
  }
  return iter;
}

/******************************************************************************/

soap_dom_attribute::iterator soap_dom_attribute::att_find(const char *ns, const wchar_t *pat)
{
  const char *s = soap_wchar2s(NULL, pat);
  soap_dom_attribute::iterator iter = this->att_find(ns, s);
  if (s)
    free((void*)s);
  return iter;
}

/******************************************************************************/

void soap_dom_attribute::unlink()
{
  soap_unlink(soap, this);
  soap_unlink(soap, nstr);
  soap_unlink(soap, name);
  soap_unlink(soap, text);
  if (next)
    next->unlink();
}

/******************************************************************************\
 *
 *      soap_dom_element_iterator class
 *
\******************************************************************************/

soap_dom_element_iterator::soap_dom_element_iterator()
{
  iter = NULL;
  stop = NULL;
  nstr = NULL;
  name = NULL;
  type = 0;
  deep = false;
}

/******************************************************************************/

soap_dom_element_iterator::soap_dom_element_iterator(soap_dom_element *node)
{
  iter = node;
  stop = NULL;
  nstr = NULL;
  name = NULL;
  type = 0;
  deep = false;
}

/******************************************************************************/

soap_dom_element_iterator::~soap_dom_element_iterator()
{ }

/******************************************************************************/

bool soap_dom_element_iterator::operator==(const soap_dom_element_iterator &it) const
{
  return this->iter == it.iter;
}

/******************************************************************************/

bool soap_dom_element_iterator::operator!=(const soap_dom_element_iterator &it) const
{
  return this->iter != it.iter;
}

/******************************************************************************/

soap_dom_element& soap_dom_element_iterator::operator*() const
{
  return *this->iter;
}

/******************************************************************************/

soap_dom_element *soap_dom_element_iterator::operator->() const
{
  return this->iter;
}

/******************************************************************************/

soap_dom_element_iterator &soap_dom_element_iterator::operator++()
{
  if (deep)
    this->iter = soap_dom_find_next(this->iter, this->stop, this->nstr, this->name, this->type);
  else
    this->iter = soap_elt_find_next_type(this->iter, this->nstr, this->name, this->type);
  return *this;
}

/******************************************************************************/

soap_dom_element_iterator soap_dom_element_iterator::operator++(int)
{
  soap_dom_element_iterator iter(*this);
  ++*this;
  return iter;
}

/******************************************************************************\
 *
 *      soap_dom_attribute_iterator class
 *
\******************************************************************************/

soap_dom_attribute_iterator::soap_dom_attribute_iterator()
{
  iter = NULL;
  nstr = NULL;
  name = NULL;
}

/******************************************************************************/

soap_dom_attribute_iterator::soap_dom_attribute_iterator(soap_dom_attribute *node)
{
  this->iter = node;
  nstr = NULL;
  name = NULL;
}

/******************************************************************************/

soap_dom_attribute_iterator::~soap_dom_attribute_iterator()
{ }

/******************************************************************************/

bool soap_dom_attribute_iterator::operator==(const soap_dom_attribute_iterator &it) const
{
  return this->iter == it.iter;
}

/******************************************************************************/

bool soap_dom_attribute_iterator::operator!=(const soap_dom_attribute_iterator &it) const
{
  return this->iter != it.iter;
}

/******************************************************************************/

soap_dom_attribute& soap_dom_attribute_iterator::operator*() const
{
  return *this->iter;
}

/******************************************************************************/

soap_dom_attribute *soap_dom_attribute_iterator::operator->() const
{
  return this->iter;
}

/******************************************************************************/

soap_dom_attribute_iterator &soap_dom_attribute_iterator::operator++()
{
  this->iter = soap_att_find_next(this->iter, this->nstr, this->name);
  return *this;
}

/******************************************************************************/

soap_dom_attribute_iterator soap_dom_attribute_iterator::operator++(int)
{
  soap_dom_attribute_iterator iter(*this);
  ++*this;
  return iter;
}

/******************************************************************************\
 *
 *      I/O
 *
\******************************************************************************/

#ifndef WITH_COMPAT

std::ostream &operator<<(std::ostream &o, const struct soap_dom_element &e)
{
  if (!e.soap)
  {
    struct soap *soap = soap_new1(SOAP_IO_DEFAULT | SOAP_DOM_TREE);
    if (soap)
    {
      soap->os = &o;
      soap_serialize_xsd__anyType(soap, &e);
      if (soap_begin_send(soap)
       || soap_out_xsd__anyType(soap, NULL, 0, &e, NULL)
       || soap_end_send(soap))
        o.clear(std::ios::failbit); /* writing failed (must be a stream error) */
      soap_destroy(soap);
      soap_end(soap);
      soap_free(soap);
    }
  }
  else
  {
    std::ostream *os = e.soap->os;
    e.soap->os = &o;
    soap_serialize_xsd__anyType(e.soap, &e);
    if (soap_begin_send(e.soap)
     || soap_out_xsd__anyType(e.soap, NULL, 0, &e, NULL)
     || soap_end_send(e.soap))
      o.clear(std::ios::failbit); /* writing failed (must be a stream error) */
    e.soap->os = os;
  }
  return o;
}

/******************************************************************************/

std::istream &operator>>(std::istream &i, struct soap_dom_element &e)
{
  if (!e.soap)
    e.soap = soap_new();
  if (e.soap)
  {
    std::istream *is = e.soap->is;
    e.soap->is = &i;
    if (soap_begin_recv(e.soap)
     || soap_in_xsd__anyType(e.soap, NULL, &e, NULL) == NULL
     || soap_end_recv(e.soap))
    { /* e.soap->error is now set and app should check */ }
    e.soap->is = is;
  }
  return i;
}

#endif

#endif
