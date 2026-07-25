/*

WS-Header.h

Combines WS-Routing, WS-Security, WS-Addressing

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2004-2005, Robert van Engelen, Genivia Inc. All Rights Reserved.
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
Copyright (C) 2000-2007, Robert van Engelen, Genivia Inc., All Rights Reserved.
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

#import "wsrp.h"
#import "wsa.h"
#import "wsse.h"

struct SOAP_ENV__Header
{
  mustUnderstand _wsrp__path		*wsrp__path	0; ///< WS-Routing
  mustUnderstand _wsse__Security	*wsse__Security	0; ///< WS-Security
  mustUnderstand _wsa__MessageID	 wsa__MessageID	0; ///< WS-Addressing
  mustUnderstand _wsa__RelatesTo	*wsa__RelatesTo	0; ///< WS-Addressing
  mustUnderstand _wsa__From		*wsa__From	0; ///< WS-Addressing
  mustUnderstand _wsa__ReplyTo		*wsa__ReplyTo	0; ///< WS-Addressing
  mustUnderstand _wsa__FaultTo		*wsa__FaultTo	0; ///< WS-Addressing
  mustUnderstand _wsa__To		 wsa__To	0; ///< WS-Addressing
  mustUnderstand _wsa__Action		 wsa__Action	0; ///< WS-Addressing
};
