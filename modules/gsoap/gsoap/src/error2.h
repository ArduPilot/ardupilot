/*
	error2.h

	Error handling.

gSOAP XML Web services tools
Copyright (C) 2000-2008, Robert van Engelen, Genivia Inc. All Rights Reserved.
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

extern char errbuf[];

#ifdef WIN32_WITHOUT_SOLARIS_FLEX
extern void soapcpp2error(const char*);
#else
extern void yyerror(const char*);
#endif

extern void lexerror(const char*);
extern void synerror(const char *);
extern void semerror(const char *);
extern void semwarn(const char *);
extern void compliancewarn(const char *);
extern void typerror(const char*);
extern void execerror(const char*);
extern void progerror(const char*, const char*, int);
extern int errstat(void);
