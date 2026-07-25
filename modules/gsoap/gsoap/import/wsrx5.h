/*
	wsrx5.h

	WS-ReliableMessaging definitions:
	SOAP Header definitions for WS-RM 1.0 2005
	WS-RM Operations for CreateSequence, CloseSequence, TerminateSequence
	WS-RM SequenceAcknowledgement server operation (RM dest for AcksTo)
	Note: changed TerminateSequenceResponse to TerminateSequence

	Imported by import/wsrm5.h

gSOAP XML Web services tools
Copyright (C) 2000-2012, Robert van Engelen, Genivia Inc., All Rights Reserved.
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
Copyright (C) 2000-2012, Robert van Engelen, Genivia Inc., All Rights Reserved.
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

struct SOAP_ENV__Header
{
  struct wsrm__SequenceType       *wsrm__Sequence                0;
  int                              __sizeAckRequested            0;
  struct wsrm__AckRequestedType   *wsrm__AckRequested            0;
  int                              __sizeSequenceAcknowledgement 0;
  struct _wsrm__SequenceAcknowledgement *wsrm__SequenceAcknowledgement 0;
  struct wsrm__SequenceFaultType  *wsrm__SequenceFault           0;
};

//gsoap wsrm service name: wsrm

//gsoap wsrm service method-header-part:     CreateSequence wsa5__MessageID
//gsoap wsrm service method-header-part:     CreateSequence wsa5__RelatesTo
//gsoap wsrm service method-header-part:     CreateSequence wsa5__From
//gsoap wsrm service method-header-part:     CreateSequence wsa5__ReplyTo
//gsoap wsrm service method-header-part:     CreateSequence wsa5__FaultTo
//gsoap wsrm service method-header-part:     CreateSequence wsa5__To
//gsoap wsrm service method-header-part:     CreateSequence wsa5__Action
//gsoap wsrm service method-action:          CreateSequence http://schemas.xmlsoap.org/ws/2005/02/rm/CreateSequence
//gsoap wsrm service method-output-action:   CreateSequence http://schemas.xmlsoap.org/ws/2005/02/rm/CreateSequenceResponse
int __wsrm__CreateSequence(
  struct wsrm__CreateSequenceType         *wsrm__CreateSequence,
  struct wsrm__CreateSequenceResponseType *wsrm__CreateSequenceResponse);

//gsoap wsrm service method-header-part:     CloseSequence wsa5__MessageID
//gsoap wsrm service method-header-part:     CloseSequence wsa5__RelatesTo
//gsoap wsrm service method-header-part:     CloseSequence wsa5__From
//gsoap wsrm service method-header-part:     CloseSequence wsa5__ReplyTo
//gsoap wsrm service method-header-part:     CloseSequence wsa5__FaultTo
//gsoap wsrm service method-header-part:     CloseSequence wsa5__To
//gsoap wsrm service method-header-part:     CloseSequence wsa5__Action
//gsoap wsrm service method-action:          CloseSequence http://schemas.xmlsoap.org/ws/2005/02/rm/CloseSequence
//gsoap wsrm service method-output-action:   CloseSequence http://schemas.xmlsoap.org/ws/2005/02/rm/CloseSequenceResponse
int __wsrm__CloseSequence(
  struct wsrm__CloseSequenceType         *wsrm__CloseSequence,
  struct wsrm__CloseSequenceResponseType *wsrm__CloseSequenceResponse);

//gsoap wsrm service method-header-part:     TerminateSequence wsa5__MessageID
//gsoap wsrm service method-header-part:     TerminateSequence wsa5__RelatesTo
//gsoap wsrm service method-header-part:     TerminateSequence wsa5__From
//gsoap wsrm service method-header-part:     TerminateSequence wsa5__ReplyTo
//gsoap wsrm service method-header-part:     TerminateSequence wsa5__FaultTo
//gsoap wsrm service method-header-part:     TerminateSequence wsa5__To
//gsoap wsrm service method-header-part:     TerminateSequence wsa5__Action
//gsoap wsrm service method-action:          TerminateSequence http://schemas.xmlsoap.org/ws/2005/02/rm/TerminateSequence
//gsoap wsrm service method-output-action:   TerminateSequence http://schemas.xmlsoap.org/ws/2005/02/rm/TerminateSequence
int __wsrm__TerminateSequence(
  struct wsrm__TerminateSequenceType         *wsrm__TerminateSequence,
  struct wsrm__TerminateSequenceResponseType *wsrm__TerminateSequence_);

//gsoap wsrm service method-header-part:     CreateSequenceResponse wsa5__MessageID
//gsoap wsrm service method-header-part:     CreateSequenceResponse wsa5__RelatesTo
//gsoap wsrm service method-header-part:     CreateSequenceResponse wsa5__From
//gsoap wsrm service method-header-part:     CreateSequenceResponse wsa5__ReplyTo
//gsoap wsrm service method-header-part:     CreateSequenceResponse wsa5__FaultTo
//gsoap wsrm service method-header-part:     CreateSequenceResponse wsa5__To
//gsoap wsrm service method-header-part:     CreateSequenceResponse wsa5__Action
//gsoap wsrm service method-action:          CreateSequenceResponse http://schemas.xmlsoap.org/ws/2005/02/rm/CreateSequenceResponse
int __wsrm__CreateSequenceResponse(struct wsrm__CreateSequenceResponseType *wsrm__CreateSequenceResponse, void);

//gsoap wsrm service method-header-part:     CloseSequenceResponse wsa5__MessageID
//gsoap wsrm service method-header-part:     CloseSequenceResponse wsa5__RelatesTo
//gsoap wsrm service method-header-part:     CloseSequenceResponse wsa5__From
//gsoap wsrm service method-header-part:     CloseSequenceResponse wsa5__ReplyTo
//gsoap wsrm service method-header-part:     CloseSequenceResponse wsa5__FaultTo
//gsoap wsrm service method-header-part:     CloseSequenceResponse wsa5__To
//gsoap wsrm service method-header-part:     CloseSequenceResponse wsa5__Action
//gsoap wsrm service method-action:          CloseSequenceResponse http://schemas.xmlsoap.org/ws/2005/02/rm/CloseSequenceResponse
int __wsrm__CloseSequenceResponse(struct wsrm__CloseSequenceResponseType *wsrm__CloseSequenceResponse, void);

//gsoap wsrm service method-header-part:     TerminateSequenceResponse wsa5__MessageID
//gsoap wsrm service method-header-part:     TerminateSequenceResponse wsa5__RelatesTo
//gsoap wsrm service method-header-part:     TerminateSequenceResponse wsa5__From
//gsoap wsrm service method-header-part:     TerminateSequenceResponse wsa5__ReplyTo
//gsoap wsrm service method-header-part:     TerminateSequenceResponse wsa5__FaultTo
//gsoap wsrm service method-header-part:     TerminateSequenceResponse wsa5__To
//gsoap wsrm service method-header-part:     TerminateSequenceResponse wsa5__Action
//gsoap wsrm service method-action:          TerminateSequenceResponse http://schemas.xmlsoap.org/ws/2005/02/rm/TerminateSequenceResponse
int __wsrm__TerminateSequenceResponse(struct wsrm__TerminateSequenceResponseType *wsrm__TerminateSequenceResponse, void);

//gsoap wsrm service method-header-part:     SequenceAcknowledgement wsa5__MessageID
//gsoap wsrm service method-header-part:     SequenceAcknowledgement wsa5__RelatesTo
//gsoap wsrm service method-header-part:     SequenceAcknowledgement wsa5__From
//gsoap wsrm service method-header-part:     SequenceAcknowledgement wsa5__ReplyTo
//gsoap wsrm service method-header-part:     SequenceAcknowledgement wsa5__FaultTo
//gsoap wsrm service method-header-part:     SequenceAcknowledgement wsa5__To
//gsoap wsrm service method-header-part:     SequenceAcknowledgement wsa5__Action
//gsoap wsrm service method-action:          SequenceAcknowledgement http://schemas.xmlsoap.org/ws/2005/02/rm/SequenceAcknowledgement
int __wsrm__SequenceAcknowledgement(void);

//gsoap wsrm service method-header-part:     AckRequested wsa5__MessageID
//gsoap wsrm service method-header-part:     AckRequested wsa5__RelatesTo
//gsoap wsrm service method-header-part:     AckRequested wsa5__From
//gsoap wsrm service method-header-part:     AckRequested wsa5__ReplyTo
//gsoap wsrm service method-header-part:     AckRequested wsa5__FaultTo
//gsoap wsrm service method-header-part:     AckRequested wsa5__To
//gsoap wsrm service method-header-part:     AckRequested wsa5__Action
//gsoap wsrm service method-action:          AckRequested http://schemas.xmlsoap.org/ws/2005/02/rm/AckRequested
int __wsrm__AckRequested(void);

//gsoap wsrm service method-header-part:     LastMessage wsa5__MessageID
//gsoap wsrm service method-header-part:     LastMessage wsa5__RelatesTo
//gsoap wsrm service method-header-part:     LastMessage wsa5__From
//gsoap wsrm service method-header-part:     LastMessage wsa5__ReplyTo
//gsoap wsrm service method-header-part:     LastMessage wsa5__FaultTo
//gsoap wsrm service method-header-part:     LastMessage wsa5__To
//gsoap wsrm service method-header-part:     LastMessage wsa5__Action
//gsoap wsrm service method-action:          LastMessage http://schemas.xmlsoap.org/ws/2005/02/rm/LastMessage
int __wsrm__LastMessage(void);

