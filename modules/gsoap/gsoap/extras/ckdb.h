/*
	ckdb.h

	HTTP cookie database manager. See ckdb.c for more details.

	The contents of this file are subject to the gSOAP Public License
	Version 1.0 (the "License"); you may not use this file except in
	compliance with the License. You may obtain a copy of the License at
	http://www.cs.fsu.edu/~engelen/soaplicense.html Software distributed
	under the License is distributed on an "AS IS" basis, WITHOUT WARRANTY
	OF ANY KIND, either express or implied. See the License for the
	specific language governing rights and limitations under the License.

	The Initial Developer of the Original Code is Robert A. van Engelen.
	Copyright (C) 2000-2002 Robert A. van Engelen. All Rights Reserved.

*/

/* struct cookie must be a mirror image of struct soap_cookie in stdsoap2.h */
struct cookie
{
  struct cookie *next;
  char *name;
  char *value;
  char *domain;
  char *path;
  long expire;
  unsigned int version;
  short secure;
  [
  short session;	/* transient: do not (de)serialize */
  short env;		/* transient: do not (de)serialize */
  short modified;	/* transient: do not (de)serialize */
  ]
};
