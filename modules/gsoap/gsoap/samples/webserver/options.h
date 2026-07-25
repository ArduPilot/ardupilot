/*

options.h

Command line option handler for webserver.

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

	The option structure defines a program option as follows:
	name		the name of the option, which may include a period to
			define a one-character name for command-line options
			(single character followed by a period and the full
			name). The last option in an array MUST be NULL.
			When the name is empty (""), then this option is used to
			collect remaining command-line arguments, such as file
			names and so on.
	selections:	either must be NULL to define a boolean option. In this
				case selected=0 is false and selected=1 is true.
			or a space-delimited string of option values. In this
				case 'selected' is set to # word in list.
			or a single-word description for option values. In this
				case the argument is stored in 'value' field.
	selected:	word index in 'selections' list, or 0/1 for boolean
			option, or max length of string value a user can enter
			(see below).
	value:		the (default) string value of option

	Example:

	static struct option options[] =
	{ { "c.compress", NULL },	on/off option (-c turns it on)
  	  { "e.endpoint", "URL" }, 	this option requires an argument value
  	  { "name", "alphanum", 20, "demo" },
	  				this option requires an argument value.
					The default value will be "demo".
					The HTML slot will be 20 chars wide.
  	  { "action", "start finish" }, this option requires an argument "start"
	  				or "finish"
  	  { "", "file1 file2 ..." },	collect remaining arguments
  	  { NULL },			end of table
	};

	Functionality:

	int parse_options(int argc, char **argv, struct option *options)
	Parses options from the command line (argc, argv). When the command
	line options include the '-?' and '-help', a diagnostic 'Usage:'
	message is produced and the function returns 1 instead of a normal 0
	return value.

	void query_options(struct soap *soap, struct option *options)
	Parses options from the HTTP query string. This requires linking the
	httpget plugin. The function is normally called in the HTTP GET handler.
	
	int save_options(const char *file, const char *name, struct option *options)
	Saves an array of options to a file. The last option in the 'options'
	array MUST have a NULL name to delimit the array. Options are saved
	in XML format.

	int load_options(const char *file, const char *name, struct option *options)
	Parses options from a file saved with save_options(). The last option
	in the 'options' array MUST have a NULL name to delimit the array.

	int html_options(struct soap *soap, struct option *options)
	Produces an HTML table with options and form submit buttons to
	interactively change options through a Web page. The table is output
	via soap_send() which uses the current soap.socket or soap.sendfd
	descriptors. The table should be produced within a form, see also
	html_form_options().
	The HTML table uses the following picture files:
		checked.gif		check box (checked)
		unchecked.gif		check box (unchecked)
		selected.gif		radio button (selected)
		deselected.gif		radio button (deselected)

	int html_form_options(struct soap *soap, struct option *options)
	Produces an HTML form with action='' and method='get' (to send the
	request back to the current service. An HTML option table is produced
	within the form.
*/

struct option
{ const char *name;		/* name */
  const char *selections;	/* NULL (option does not require argument),
  				   or one-word description,
				   or selection of possible values (separated by spaces) */
  int selected;			/* >=0: current selection in selections list */
  char *value;			/* parsed value (when 'selections' is one name) */
};

extern struct option *copy_options(const struct option *options);
extern void free_options(struct option *options);
extern int parse_options(int argc, char **argv, struct option *options);
extern void query_options(struct soap *soap, struct option *options);
extern int save_options(const char *file, const char *name, struct option *options);
extern int load_options(const char *file, const char *name, struct option *options);
extern int html_options(struct soap *soap, struct option *options);
extern int html_form_options(struct soap *soap, struct option *options);

