/*

	options.c

	Parses command line options and provides options control panel with an
	interactive Web interface for the Web server (webserver.c).

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2004, Robert van Engelen, Genivia, Inc. All Rights Reserved.
This software is released under one of the following two licenses:
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

#define WITH_NOGLOBAL
#define SOAP_FMAC3 static
#include "optC.c"
#include "options.h"
#include "httpget.h"

#ifdef WIN32
#define OPTION_CHAR '/'
#else
#define OPTION_CHAR '-'
#endif

struct option *copy_options(const struct option *options);
int parse_options(int argc, char **argv, struct option *options);
void query_options(struct soap *soap, struct option *options);
int load_options(const char *file, const char *name, struct option *options);
int save_options(const char *file, const char *name, struct option *options);
int html_options(struct soap *soap, struct option *options);
int html_form_options(struct soap *soap, struct option *options);
static int set_option(const char *key, const char *val, struct option *options);
static int set_selection(const char *val, struct option *p);
static int find_selection(const char *value, const char *selections);
static void print_usage(int argc, char **argv, struct option *options);

struct option *copy_options(const struct option *options)
{
  struct option *p;
  int n;
  for (n = 0; options[n].name; n++)
    ;
  p = (struct option*)malloc((n + 1) * sizeof(struct option));
  if (p)
  {
    for (n = 0; options[n].name; n++)
    {
      p[n].name = options[n].name;
      p[n].selections = options[n].selections;
      p[n].selected = options[n].selected;
      if (options[n].value)
      {
        size_t l = strlen(options[n].value);
	p[n].value = (char*)malloc(l + 1);
        soap_strcpy(p[n].value, l + 1, options[n].value);
      }
      else
        p[n].value = NULL;
    }
    p[n].name = NULL;
    p[n].selections = NULL;
    p[n].selected = 0;
    p[n].value = NULL;
  }
  return p;
}

void free_options(struct option *options)
{
  if (options)
  {
    struct option *p;
    for (p = options; p->name; p++ )
    {
      if (p->value)
        free(p->value);
    }
    free(options);
  }
}

int parse_options(int argc, char **argv, struct option *options)
{
  int i;
  for (i = 1; i < argc; i++)
  {
    const char *key, *val;
    key = argv[i];
    if (*key == OPTION_CHAR)
    {
      int flag = 1;
      while (flag && *++key)
      {
        if (*key == '?' || !strcmp(key, "help"))
        {
          print_usage(argc, argv, options);
	  return 1;
        }
	if (i < argc)
	  val = argv[i + 1];
	else
	  val = NULL;
	switch (set_option(key, val, options))
	{
          case 2:
	    i++;
	  case 1:
	    flag = 0;
	  case 0:
	    break;
	  case -1:
            fprintf(stderr, "%s: missing argument value for option %c%s\n", argv[0], OPTION_CHAR, key);
	    print_usage(argc, argv, options);
	    return 1;
	  case -2:
            fprintf(stderr, "%s: unknown option %c%s\n", argv[0], OPTION_CHAR, key);
	    print_usage(argc, argv, options);
	    return 1;
	}
      }
    }
    else if (set_option(NULL, key, options) < 0)
      fprintf(stderr, "%s: invalid argument %s\n", argv[0], key);
  }
  return 0;
}

void query_options(struct soap *soap, struct option *options)
{
  char *s = soap_query(soap); /* get arguments from query string */
  while (s)
  {
    char *key = soap_query_key(soap, &s); /* decode next query string key */
    char *val = soap_query_val(soap, &s); /* decode next query string value */
    if (key)
      set_option(key, val, options);
  }
}

int load_options(const char *file, const char *name, struct option *options)
{
  struct soap soap;
  soap_init(&soap);
  if (file)
  {
    soap.recvfd = open(file, O_RDONLY);
    if (soap.recvfd < 0)
    {
      soap_done(&soap);
      return EOF;
    }
  }
  if (!soap_begin_recv(&soap) && !soap_element_begin_in(&soap, name, 1, NULL))
  {
    struct t__Option t;
    while (soap_in_t__Option(&soap, "option", &t, NULL))
      if (set_option(t.key, t.val, options) < 0)
        ; /* error, just ignore for now */
    if (!soap_element_end_in(&soap, name))
      soap_end_recv(&soap);
  }
  if (file)
    close(soap.recvfd);
  soap_end(&soap);
  soap_done(&soap);
  return soap.error;
}

int save_options(const char *file, const char *name, struct option *options)
{
  struct soap soap;
  soap_init1(&soap, SOAP_XML_TREE);
  soap.namespaces = NULL;
  soap.encodingStyle = NULL;
  if (file)
  {
    soap.sendfd = open(file, O_CREAT | O_RDWR, 0644);
    if (soap.sendfd < 0)
    {
      soap_done(&soap);
      return EOF;
    }
  }
  if (!soap_begin_send(&soap) && !soap_element_begin_out(&soap, name, 0, NULL))
  {
    struct option *p;
    struct t__Option t;
    for (p = options; p->name; p++)
    {
      t.val = NULL;
      if (!p->name[0])
      {
        if (!p->value)
          continue;
	t.key = NULL;
	t.val = p->value;
      }
      else if (p->name[1] == '.')
        t.key = (char*)p->name + 2;
      else
        t.key = (char*)p->name;
      if (p->selections && strchr(p->selections, ' '))
      {
        const char *s = p->selections - 1;
        char *r;
	int i;
        for (i = p->selected; i > 0; i--)
        {
          s = strchr(s + 1, ' ');
	  if (!s)
	    break;
	}
        if (s)
	{
          t.val = soap_strdup(&soap, s + 1);
          r = strchr(t.val, ' ');
	  if (r)
	    *r = '\0';
        }
      }
      else if (p->value)
        t.val = p->value;
      else if (!p->selected)
        continue;
      if (soap_out_t__Option(&soap, "option", 0, &t, NULL))
        break;
    }
    if (!soap_element_end_out(&soap, name))
      soap_end_send(&soap);
  }
  if (file)
    close(soap.sendfd);
  soap_end(&soap);
  soap_done(&soap);
  return soap.error;
}

int html_options(struct soap *soap, struct option *options)
{
  struct option *p;
  soap_send(soap, "<table border='0' cellspacing='0' cellpadding='0' bgcolor='#666666' nosave>\n");
  soap_send(soap, "<tr height='10'><td background='btl.gif'></td><td background='bt.gif'></td><td background='bt.gif'></td><td background='bt.gif'></td><td background='bt.gif'></td><td background='bt.gif'></td><td width='10' background='btr.gif'></td><td width='10' height='10' background='obls.gif'></td></tr>");
  for (p = options; p->name; p++)
  {
    const char *s, *t, *n;
    int i;
    n = p->name;
    if (n[0] && n[1] == '.')
      n += 2;
    s = p->selections;
    soap_send(soap, "<tr><td background='bl.gif'></td><td align='right'>");
    if (!n[0])
    {
      if (s)
        soap_send(soap, s);
      soap_send(soap, "</td><td></td><td></td><td></td><td>");
      if (p->value)
        soap_send(soap, p->value);
      soap_send(soap, "</td>");
    }
    else if (!s)
    {
      soap_send(soap, n);
      soap_send(soap, "</td><td width='10' background='ls.gif'></td><td bgcolor='#FFFFFF'><a href='?");
      soap_send(soap, n);
      if (p->selected)
        soap_send(soap, "='><img src='checked.gif' align='absmiddle' border='0'></a></td><td width='10' background='rs.gif'></td><td>on</td>");
      else
        soap_send(soap, "'><img src='unchecked.gif' align='absmiddle' border='0'></a></td><td width='10' background='rs.gif'></td><td>off</td>");
    }
    else if (strchr(s, ' '))
    {
      soap_send(soap, n);
      soap_send(soap, "</td>");
      for (i = 0; ; i++)
      {
        t = strchr(s, ' ');
        if (i == 0)
	  soap_send(soap, "<td width='10' background='ls.gif'></td><td bgcolor='#FFFFFF'>");
        else
	  soap_send(soap, "<tr><td background='bl.gif'></td><td></td><td width='10' background='ls.gif'></td><td bgcolor='#FFFFFF'>");
        if (i == p->selected)
          soap_send(soap, "<img src='selected.gif' align='absmiddle' border='0'></td><td width='10' background='rs.gif'></td><td>");
        else
        {
          soap_send(soap, "<a href='?");
          soap_send(soap, n);
          soap_send(soap, "=");
          if (t)
            soap_send_raw(soap, s, t - s);
          else
            soap_send(soap, s);
          soap_send(soap, "'>");
          soap_send(soap, "<img src='deselected.gif' align='absmiddle' border='0'></a></td><td width='10' background='rs.gif'></td><td>");
	}
        if (t)
          soap_send_raw(soap, s, t - s);
        else
          soap_send(soap, s);
        soap_send(soap, "</td>");
        if (!t)
          break;
        soap_send(soap, "<td width='10' background='br.gif'></td><td width='10' background='ls.gif'></td></tr>\n");
        s = t + 1;
      }
    }
    else
    {
      char buf[24];
      soap_send(soap, n);
      soap_send(soap, "</td><td width='10' background='ls.gif'></td><td bgcolor='#FFFFFF'>&nbsp;</td><td width='10' background='rs.gif'></td><td><input type='text' name='");
      soap_send(soap, n);
      soap_send(soap, "' value='");
      if (p->value)
        soap_send(soap, p->value);
      soap_send(soap, "' size='");
      (SOAP_SNPRINTF(buf, sizeof(buf), 20), "%d", p->selected > 0 ? p->selected : 24);
      soap_send(soap, buf);
      soap_send(soap, "'><input type='submit' value='Set ");
      soap_send(soap, s);
      soap_send(soap, "'></td>");
    }
    soap_send(soap, "<td width='10' background='br.gif'></td><td width='10' background='ls.gif'></td></tr>\n");
    // <tr height='1'><td height='1' background='bl.gif'></td><td></td><td></td><td></td><td></td><td width='10' height='1' background='br.gif'></td><td width='10' height='1' background='ls.gif'></td></tr>\n");
  }
  soap_send(soap, "<tr height='10'><td background='bbl.gif'></td><td background='bb.gif'></td><td background='bb.gif'></td><td background='bb.gif'></td><td background='bb.gif'></td><td background='bb.gif'></td><td width='10' background='bbr.gif'><td width='10' height='10' background='ls.gif'</td></tr>\n<tr height='10'><td width='10' height='10' background=otrs.gif></td><td height='10' background='ts.gif'></td><td height='10' background='ts.gif'></td><td height='10' background='ts.gif'></td><td height='10' background='ts.gif'></td><td height='10' background='ts.gif'></td><td height='10' background='ts.gif'></td><td width='10' height='10' background='otls.gif'></td></tr>\n</table>\n");
  return SOAP_OK;
}

int html_form_options(struct soap *soap, struct option *options)
{
  soap_send(soap, "<form action='' method='get' name='form'>");
  html_options(soap, options);
  return soap_send(soap, "</form>");
}

static int set_option(const char *key, const char *val, struct option *options)
{
  struct option *p;
  if (!key)
  {
    for (p = options; p->name; p++)
    {
      if (!p->name[0])
      {
        if (p->value)
        {
          size_t l = strlen(p->value) + strlen(val);
	  char *s = (char*)malloc(l + 2);
	  if (s)
	  {
            (SOAP_SNPRINTF(s, l + 2, l + 1), "%s %s", p->value, val);
	    free((void*)p->value);
	    p->value = s;
          }
        }
        else
        {
          size_t l = strlen(val);
	  p->value = (char*)malloc(l + 1);
          if (p->value)
            soap_strcpy(p->value, l + 1, val);
        }
        return 2;
      }
    }
    return -2;
  }
  for (p = options; p->name; p++)
  {
    if (p->name[0] && ((p->name[1] == '.' && !strcmp(key, p->name + 2)) || (p->name[1] != '.' && !strcmp(key, p->name))))
    {
      if (p->selections)
      {
        if (!val)
          return -1;
        if (set_selection(val, p) < 0)
          return -1;
        return 2;
      }
      if (val && !*val)
      {
        p->selected = 0;
        return 2;
      }
      p->selected = 1;
      return 1;
    }
  }
  for (p = options; p->name; p++)
  {
    if (p->name[0] == key[0] && p->name[1] == '.')
    {
      if (p->selections)
      {
        const char *s = key + 1;
        if (!s[0])
        {
          s = val;
          if (!s)
            return -1;
        }
        if (set_selection(s, p) < 0)
          return -1;
        if (s == val)
          return 2;
        return 1;
      }
      p->selected = 1;
      return 0;
    }
  }
  return -2;
}

static int set_selection(const char *val, struct option *p)
{
  size_t l;
  if (strchr(p->selections, ' '))
    return p->selected = find_selection(val, p->selections);
  if (p->value)
    free(p->value);
  l = strlen(val);
  p->value = (char*)malloc(l + 1);
  if (p->value)
    soap_strcpy(p->value, l + 1, val);
  return 0;
}

static int find_selection(const char *val, const char *selections)
{
  int i = 0;
  const char *s = selections;
  size_t n = strlen(val);
  for (;;)
  {
    if (!strncmp(s, val, n) && (s[n] == ' ' || s[n] == '\0'))
      return i;
    s = strchr(s, ' ');
    if (!s)
      break;
    s++;
    i++;
  }
  return -1;
}

static void print_usage(int argc, char **argv, struct option *options)
{
  struct option *p;
  fprintf(stderr, "Usage: %s", argv[0]);
  for (p = options; p->name; p++)
  {
    if (p->name[0])
    {
      int flag = (p->name[1] == '.');
      if (p->selections)
      {
        if (strchr(p->selections, ' '))
        {
          if (flag)
	    fprintf(stderr, " %c%c[%s]", OPTION_CHAR, p->name[0], p->selections);
          else
	    fprintf(stderr, " %c%s [%s]", OPTION_CHAR, p->name, p->selections);
        }
        else if (flag)
          fprintf(stderr, " %c%c<%s>", OPTION_CHAR, p->name[0], p->selections);
        else
          fprintf(stderr, " %c%s <%s>", OPTION_CHAR, p->name, p->selections);
      }
      else if (flag)
        fprintf(stderr, " %c%c", OPTION_CHAR, p->name[0]);
      else
        fprintf(stderr, " %c%s", OPTION_CHAR, p->name);
    }
    else if (p->selections)
      fprintf(stderr, " %s", p->selections);
    else
      fprintf(stderr, " ...");
  }
  fprintf(stderr, "\n");
}

