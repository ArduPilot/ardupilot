/*
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

//gsoap ns service name:	Event Simple remote event handler
//gsoap ns service style:	rpc
//gsoap ns service encoding:	encoded
//gsoap ns service namespace:	http://www.cs.fsu.edu/~engelen/event.wsdl
//gsoap ns service location:	http://localhost:18000

//gsoap ns schema namespace:    urn:event

//gsoap ns schema type: event Set of four possible events
enum ns__event {
//gsoap ns schema type: event::EVENT_A first event
  EVENT_A,
//gsoap ns schema type: event::EVENT_B second event
  EVENT_B,
//gsoap ns schema type: event::EVENT_C third event
  EVENT_C,
//gsoap ns schema type: event::EVENT_Z final event
  EVENT_Z
};

//gsoap ns service method:              handle          Handles asynchronous events
//gsoap ns service method:              handle::event   EVENT_A, EVENT_B, EVENT_C, or EVENT_Z
//gsoap ns service method-action:       handle          "event"
int ns__handle(enum ns__event event, void);
