/*
	rss.h

	Supports RSS 0.91, 0.92 and 2.0

	Retrieve RSS feeds and formats the messages in HTML.

	Use CSS to format the HTML layout.

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

//gsoap dc schema namespace: http://purl.org/dc/elements/1.1/

/// Container for mixed XML content (XML gSOAP built-in)
typedef char *XML;

/// <channel> element
struct channel
{
  char *title;          ///< channel title element
  char *link;           ///< channel link element
  char *language;
  char *copyright;
  XML description;	///< description may contain XHTML that is preserved in XML
  struct image *image;  ///< optional image element
  int __size;           ///< an array of items of size __size
  struct item *item;    ///< an array of item elements
  time_t *dc__date;	///< RSS 2.0 dc schema element (optional)
};

/// <item> element
struct item
{
  char *title;		///< item title element
  char *link;		///< item link element
  XML description;	///< description may contain XHTML that is preserved in XML
  char *pubDate;
  time_t *dc__date;	///< RSS 2.0 dc schema element
};

/// <image> element
struct image
{
  char *title;		///< image title element
  char *url;		///< image URL element
  char *link;		///< image link element
  int width  0:1 = 0;	///< optional, default value = 0
  int height 0:1 = 0;	///< optional, default value = 0
  XML description;	///< description may contain XHTML that is preserved in XML
};

/// <rss> element
struct rss
{
  @char *version = "2.0";	///< version attribute (optional, default="2.0")
  struct channel channel;	///< channel element
};
