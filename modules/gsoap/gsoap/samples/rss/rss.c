/*
	rss.c

	Supports RSS 0.91, 0.92, and 2.0

	Server to retrieve RSS feeds and format the messages in HTML.

	Compile:
	$ soapcpp2 -c -CSL rss.h
	$ cc -o rss rss.c stdsoap2.c soapC.c

	Usage (client):
	$ rss [maxitems] URL
	Prints RSS content in text format.
	Examples:
	rss 10 http://newsrss.bbc.co.uk/rss/newsonline_world_edition/front_page/rss.xml
	rss 10 http:// < your_rss_file.xml

	Usage (server):
	Installed as CGI application prints Javascript code to view RSS feeds
	in Web pages. The Javascript code produced by this application is
	executed with the following example script embedded in the Web page,
	e.g. to display BBC news:

<script src="http://www.cs.fsu.edu/~engelen/rss.cgi/?rss=http%3A%2F%2Fnewsrss.bbc.co.uk%2Frss%2Fnewsonline_world_edition%2Ffront_page%2Frss.xml&max=10"></script>
<noscript><a href="http://newsrss.bbc.co.uk/rss/newsonline_world_edition/front_page/rss.xml">BBC News</a></noscript>

	To control the appearance with cascading style sheets:
	rss_box		the bounding div for the entire display 
	rss_table	the table with title, image, and items
	rss_title	the title of the feed and link style if displayed
	rss_image	the image of the feed
	rss_bar		the dividing bar between items
	rss_item	the title of the item
	rss_date	the date of the item
	rss_desc	the description of the item

	To enable HTTPS, compile with:
	$ cc -DWITH_OPENSSL -o rss rss.c stdsoap2.c soapC.c -lssl -lcrypto

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2008, Robert van Engelen, Genivia, Inc. All Rights Reserved.
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

#include "soapH.h"

#define STR(s) ((s)?(s):"")

/* Maximum number of HTTP redirects */
#define MAX_REDIRECTS 10

static void docwrite0(struct soap *soap, const char *s);
static void docwrite1(struct soap *soap, const char *s1, const char *s2, const char *s3);
static void docwrite2(struct soap *soap, const char *s1, const char *s2, const char *s3, const char *s4, const char *s5);
static void docerror(struct soap *soap, const char *s);
static void strwrite(struct soap *soop, const char *s);
static char *query_key(char *buf, size_t len, char **s);
static char *query_val(char *buf, size_t len, char **s);

int main(int argc, char **argv)
{ struct soap *soap = soap_new1(SOAP_C_UTFSTRING); /* preserve UTF8 string content */
  const char *endpoint = NULL;
  struct rss *rss;
  int maxitems = 10; /* max items to show */
  int show_date = 1; /* show date yes/no */
  int i;
  /* RSS has no encoding style */
  soap->encodingStyle = NULL;
  /* get URL of RSS feed or run as CGI app */
  if (argc >= 3)
  { maxitems = atoi(argv[1]);
    endpoint = argv[2];
  }
  else if (argc >= 2)
    endpoint = argv[1];
  else
  { char *query = getenv("QUERY_STRING"); /* CGI app: get query string */
    char *keybuf;
    char *valbuf;
    int len;
    if (query)
    { len = strlen(query);
      keybuf = (char*)soap_malloc(soap, len + 1);
      valbuf = (char*)soap_malloc(soap, len + 1);
      while (query)
      { char *key = query_key(keybuf, len, &query); /* decode next query string key */
        char *val = query_val(valbuf, len, &query); /* decode next query string value (if any) */
        /* get endpoint and max number of items to display */
        if (key && val)
        { if (!strcmp(key, "rss"))
            endpoint = soap_strdup(soap, val);
          else if (!strcmp(key, "max"))
            maxitems = atoi(val);
          else if (!strcmp(key, "date"))
            show_date = (*val == 'y');
        }
      }
    }
    if (!endpoint)
    { docerror(soap, "No RSS URL");
      exit(0);
    }
  }
  /* connect and parse HTTP header (max HTTP redirects) */
  for (i = 0; i < MAX_REDIRECTS; i++)
  { /* HTTP GET and parse HTTP header */
    if (soap_connect_command(soap, SOAP_GET, endpoint, NULL)
     || soap_begin_recv(soap))
    { if ((soap->error >= 301 && soap->error <= 303) || soap->error == 307)
        endpoint = soap_strdup(soap, soap->endpoint); /* HTTP redirects */
      else
      { soap_print_fault(soap, stderr);
        exit(soap->error);
      }
    }
    else
      break;
  }
  /* parse RSS */
  rss = soap_get_rss(soap, NULL, "rss", NULL);
  /* close connection */
  soap_end_recv(soap);
  soap_closesock(soap);
  if (argc < 2)
  { /* CGI application: produce Javascript */
    if (rss && (!strcmp(rss->version, "0.91") || !strcmp(rss->version, "0.92") || !strcmp(rss->version, "2.0")))
    { soap_begin_send(soap);
      soap->http_content = "application/javascript";
      soap_response(soap, SOAP_FILE); /* SOAP_FILE specifies http_content should be used */
      docwrite0(soap, "<div class='rss_box' align='center'>");
      docwrite0(soap, "<table cellpadding='2' width='100%' class='rss_table'>");
      if (rss->channel.title)
      { if (rss->channel.link)
	  docwrite2(soap, "<tr><th class='rss_title'><a href='", rss->channel.link, "' target='_blank'>", rss->channel.title, "</a></th></tr>");
        else
	  docwrite1(soap, "<tr><th class='rss_title'>", rss->channel.title, "</th></tr>");
      }
      if (rss->channel.image && rss->channel.image->url)
      { struct image *image = rss->channel.image;
        if (image->link)
          docwrite2(soap, "<tr><th class='rss_image'><a href='", image->link, "' target='_blank'><img src='", image->url, "' alt='Channel Image' border='0'/></a></th></tr>");
	else if (rss->channel.link)
          docwrite2(soap, "<tr><th class='rss_image'><a href='", rss->channel.link, "' target='_blank'><img src='", image->url, "' alt='Channel Image' border='0'/></a></th></tr>");
        else
          docwrite1(soap, "<tr><th class='rss_image'><img src='", image->url, "' alt='Channel Image' border='0'</th></tr>");
	if (!rss->channel.title && image->title)
          docwrite1(soap, "<tr><td class='rss_title'>", image->title, "</td></tr>");
	if (!rss->channel.description && image->description)
          docwrite1(soap, "<tr><td class='rss_desc'>", image->description, "</td></tr>");
      }
      if (rss->channel.description)
        docwrite1(soap, "<tr><th class='rss_desc'>", rss->channel.description, "</th></tr>");
      if (rss->channel.copyright)
        docwrite1(soap, "<tr><td class='rss_desc'>", rss->channel.copyright, "</td></tr>");
      if (rss->channel.__size < maxitems)
        maxitems = rss->channel.__size;
      for (i = 0; i < maxitems; i++)
      { struct item *item = &rss->channel.item[i];
        docwrite0(soap, "<tr><td class='rss_bar' bgcolor='#bbbbbb'></td></tr>");
	if (item->title)
        { if (item->link)
	    docwrite2(soap, "<tr><td class='rss_item' bgcolor='#dddddd'><a href='", item->link, "' target='_blank'>", item->title, "</a></td></tr>");
	  else
	    docwrite1(soap, "<tr><td class='rss_item' bgcolor='#dddddd'>", item->title, "</td></tr>");
	  if (show_date && item->pubDate)
	    docwrite1(soap, "<tr><td class='rss_date' bgcolor='#eeeeee'>Posted ", item->pubDate, "</td></tr>");
	  else if (show_date && item->dc__date)
	    docwrite1(soap, "<tr><td class='rss_date' bgcolor='#eeeeee'>Posted ", ctime(item->dc__date), "</td></tr>");
	  if (item->description)
	    docwrite1(soap, "<tr><td class='rss_desc' bgcolor='#eeeeee'>", item->description, "</td></tr>");
        }
      }
      docwrite0(soap, "</table>");
      docwrite0(soap, "</div>");
      soap_end_send(soap);
    }
    else
    { docerror(soap, "No RSS 0.91, 0.92, or 2.0 data");
      exit(0);
    }
  }
  else if (rss)
  { /* Interactive: produce text output */
    if (!strcmp(rss->version, "0.91") || !strcmp(rss->version, "0.92") || !strcmp(rss->version, "2.0"))
    { printf("Title: %s\n", STR(rss->channel.title));
      printf("Link: %s\n", STR(rss->channel.link));
      printf("Language: %s\n", STR(rss->channel.language));
      printf("Description: %s\n", STR(rss->channel.description));
      if (rss->channel.image)
      { printf("Image title: %s\n", STR(rss->channel.image->title));
        printf("Image url: %s\n", STR(rss->channel.image->url));
        printf("Image link: %s\n", STR(rss->channel.image->link));
        printf("Image dimensions: %d x %d\n", rss->channel.image->width, rss->channel.image->height);
        printf("Image description: %s\n", STR(rss->channel.image->description));
      }
      if (rss->channel.__size < maxitems)
        maxitems = rss->channel.__size;
      for (i = 0; i < maxitems; i++)
      { printf("\n%3d Title: %s\n", i+1, STR(rss->channel.item[i].title));
        printf("    Link: %s\n", STR(rss->channel.item[i].link));
        printf("    Description: %s\n", STR(rss->channel.item[i].description));
        if (rss->channel.item[i].pubDate)
          printf("    Posted: %s\n", rss->channel.item[i].pubDate);
        else if (rss->channel.item[i].dc__date)
          printf("    Posted: %s\n", ctime(rss->channel.item[i].dc__date));
      }
      printf("\nCopyright: %s\n", STR(rss->channel.copyright));
    }
    else
      fprintf(stderr, "RSS 0.91, 0.92, or 2.0 content expected\n");
  }
  else
    soap_print_fault(soap, stderr);
  soap_end(soap);
  soap_done(soap);
  soap_free(soap);
  return 0;
}

static void docwrite0(struct soap *soap, const char *s)
{ soap_send3(soap, "document.write(\"", s, "\");\n");
}

static void docwrite1(struct soap *soap, const char *s1, const char *s2, const char *s3)
{ soap_send2(soap, "document.write(\"", s1);
  strwrite(soap, s2);
  soap_send2(soap, s3, "\");\n");
}

static void docwrite2(struct soap *soap, const char *s1, const char *s2, const char *s3, const char *s4, const char *s5)
{ soap_send2(soap, "document.write(\"", s1);
  strwrite(soap, s2);
  soap_send(soap, s3);
  strwrite(soap, s4);
  soap_send2(soap, s5, "\");\n");
}

static void docerror(struct soap *soap, const char *s)
{ soap_begin_send(soap);
  soap->http_content = "application/javascript";
  soap_response(soap, SOAP_FILE);
  soap_send(soap, s);
  soap_end_send(soap);
}

static void strwrite(struct soap *soap, const char *s)
{ while (*s)
  { const char *t;
    for (t = s; *t; t++)
       if (*t == '\\' || *t == '"' || *t == '\n')
         break;
    soap_send_raw(soap, s, t - s);
    if (*t == '\\')
    { soap_send(soap, "\\\\");
      t++;
    }
    else if (*t == '"')
    { soap_send(soap, "\\\"");
      t++;
    }
    else if (*t == '\n')
    { t++;
      if (*t == '\n')
      { soap_send(soap, "<p/>"); /* two line breaks? Probably a new paragraph was intended */
        t++;
      }
      else
        soap_send(soap, " ");
    }
    s = t;
  }
}

static const char *decode_url(char *buf, size_t len, const char *val)
{ const char *s;
  char *t;
  if (!buf)
    return NULL;
  for (s = val; *s; s++)
    if (*s != ' ' && *s != '=')
      break;
  if (*s == '"')
  { t = buf;
    s++;
    while (*s && *s != '"' && --len)
      *t++ = *s++;
    *t = '\0';
    do s++;
    while (*s && *s != '&' && *s != '=');
  }
  else
  { t = buf;
    while (*s && *s != '&' && *s != '=' && --len)
    { switch (*s)
      { case '+':
          *t++ = ' ';
        case ' ':
          s++;
          break;
        case '%':
          *t++ = ((s[1] >= 'A' ? (s[1]&0x7) + 9 : s[1] - '0') << 4) + (s[2] >= 'A' ? (s[2]&0x7) + 9 : s[2] - '0');
          s += 3;
          break;
        default:
          *t++ = *s++;
      }
    }
    *t = '\0';
  }
  if (*s == '&')
    s++;
  return s;
}

static char *query_key(char *buf, size_t len, char **s)
{ char *t = *s;
  if (t && *t)
  { *s = (char*)decode_url(buf, len, t);
    return buf;
  }
  return *s = NULL;
}

static char *query_val(char *buf, size_t len, char **s)
{ char *t = *s;
  if (t && *t == '=')
  { *s = (char*)decode_url(buf, len, t);
    return buf;
  }
  return NULL;
}

/* RSS 0.91 doesn't use namespaces, but RSS 2.0 does, which means that soap->namespace=NULL when serializing RSS 0.91 feeds */
struct Namespace namespaces[] = {
{ "dc", "http://purl.org/dc/elements/1.1/" },
{ NULL, NULL } 
};
