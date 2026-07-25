/*
	wsp.h

	WS-Policy 1.2 and 1.5 binding schema

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2010, Robert van Engelen, Genivia Inc. All Rights Reserved.
This software is released under one of the following licenses:
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

//gsoap wsp schema documentation:	WS-Policy binding
//gsoap wsp schema namespace:		http://www.w3.org/ns/ws-policy
// 1.2 //gsoap wsp schema namespace:	http://schemas.xmlsoap.org/ws/2004/09/policy
//gsoap wsp schema elementForm:		qualified             
//gsoap wsp schema attributeForm:	unqualified           

#import "imports.h"
#import "wsu.h"

class wsp__Policy;
class wsp__Content;

extern class Service;
extern class Types;

class wsp__PolicyReference
{ public:
	@xsd__anyURI			URI;
	@xsd__string			Digest;
	@xsd__anyURI			DigestAlgorithm;
  private:
	wsp__Policy			*policyRef;
  public:
	int				traverse(wsdl__definitions&);
	void				policyPtr(wsp__Policy*);
	wsp__Policy			*policyPtr() const;
};

class wsp__Assertion
{ public:
	@bool				Optional = false;
	@bool				Ignorable = false;
	wsp__Content			*Policy;
};

#import "sp.h"
#import "wsrmp.h"
#import "wsam.h"

class wsp__Content
{ public:
	wsp__Policy			*Policy;
	wsp__PolicyReference		*PolicyReference;
	std::vector<wsp__Content*>	All;
	std::vector<wsp__Content*>	ExactlyOne;

	std::vector<sp__Parts>		sp__SignedParts;
	std::vector<sp__Parts>		sp__EncryptedParts;
	std::vector<sp__Parts>		sp__RequiredParts;

	sp__Elements			*sp__SignedElements;
	sp__Elements			*sp__EncryptedElements;
	sp__Elements			*sp__ContentEncryptedElements;
	sp__Elements			*sp__RequiredElements;

	sp__Token			*sp__UsernameToken;
	sp__Token			*sp__IssuedToken;
	sp__Token			*sp__X509Token;
	sp__Token			*sp__KerberosToken;
	sp__Token			*sp__SpnegoContextToken;
	sp__Token			*sp__SecurityContextToken;
	sp__Token			*sp__SecureConversationToken;
	sp__Token			*sp__SamlToken;
	sp__Token			*sp__RelToken;
	sp__Token			*sp__HttpsToken;
	sp__Token			*sp__KeyValueToken;

	wsp__Assertion			*sp__TransportBinding;
	wsp__Assertion			*sp__TransportToken;
	wsp__Assertion			*sp__AlgorithmSuite;
	wsp__Assertion			*sp__Layout;
	wsp__Assertion			*sp__SymmetricBinding;
	wsp__Assertion			*sp__AsymmetricBinding;
	wsp__Assertion			*sp__ProtectionToken;
	wsp__Assertion			*sp__InitiatorToken;
	wsp__Assertion			*sp__InitiatorSignatureToken;
	wsp__Assertion			*sp__InitiatorEncryptionToken;
	wsp__Assertion			*sp__RecipientToken;

	wsp__Assertion			*sp__SupportingTokens;
	wsp__Assertion			*sp__SignedSupportingTokens;
	wsp__Assertion			*sp__EndorsingSupportingTokens;
	wsp__Assertion			*sp__SignedEndorsingSupportingTokens;
	wsp__Assertion			*sp__SignedEncryptedSupportingTokens;
	wsp__Assertion			*sp__EncryptedSupportingTokens;
	wsp__Assertion			*sp__EndorsingEncryptedSupportingTokens;
	wsp__Assertion			*sp__SignedEndorsingEncryptedSupportingTokens;
	wsp__Assertion			*sp__Wss10;
	wsp__Assertion			*sp__Wss11;
	wsp__Assertion			*sp__Trust10;
	wsp__Assertion			*sp__Trust13;

	wsp__Content			*sp__BootstrapPolicy;

	xsd__string			wsaw__UsingAddressing;

	wsp__Assertion			*wsam__Addressing;

	wsrmp__RMAssertion		*wsrmp__RMAssertion_;
	wsrmp__RMAssertion		*wsrmp__DeliveryAssurance;
	xsd__string			wsrmp__AtLeastOnce;
	xsd__string			wsrmp__AtMostOnce;
	xsd__string			wsrmp__ExactlyOnce;
	xsd__string			wsrmp__InOrder;

	wsrmp5__RMAssertion		*wsrmp5__RMAssertion_;
	wsrmp5__RMAssertion		*wsrmp5__DeliveryAssurance;
	xsd__string			wsrmp5__AtLeastOnce;
	xsd__string			wsrmp5__AtMostOnce;
	xsd__string			wsrmp5__ExactlyOnce;
	xsd__string			wsrmp5__InOrder;

	wsp__Assertion			*sp__NoPassword;
	wsp__Assertion			*sp__HashPassword;

	wsp__Assertion			*sp__IncludeTimestamp;
	wsp__Assertion			*sp__EncryptBeforeSigning;
	wsp__Assertion			*sp__EncryptSignature;
	wsp__Assertion			*sp__ProtectTokens;
	wsp__Assertion			*sp__OnlySignEntireHeadersAndBody;

	xsd__string			sp__RequireDerivedKeys;
	xsd__string			sp__RequireImpliedDerivedKeys;
	xsd__string			sp__RequireExplicitDerivedKeys;

	xsd__string			sp__WssUsernameToken10;
	xsd__string			sp__WssUsernameToken11;

	xsd__string			sp__RequireExternalReference;
	xsd__string			sp__RequireInternalReference;

	xsd__string			sp__RequireKeyIdentifierReference;
	xsd__string			sp__RequireIssuerSerialReference;
	xsd__string			sp__RequireEmbeddedTokenReference;
	xsd__string			sp__RequireThumbprintReference;

	xsd__string			sp__WssX509V3Token10;
	xsd__string			sp__WssX509Pkcs7Token10;
	xsd__string			sp__WssX509PkiPathV1Token10;
	xsd__string			sp__WssX509V1Token11;
	xsd__string			sp__WssX509V3Token11;
	xsd__string			sp__WssX509Pkcs7Token11;
	xsd__string			sp__WssX509PkiPathV1Token11;

	xsd__string			sp__WssKerberosV5ApReqToken11;
	xsd__string			sp__WssGssKerberosV5ApReqToken11;

	xsd__string			sp__WssRelV10Token10;
	xsd__string			sp__WssRelV20Token10;
	xsd__string			sp__WssRelV10Token11;
	xsd__string			sp__WssRelV20Token11;

	xsd__string			sp__MustNotSendCancel;
	xsd__string			sp__MustNotSendAmend;
	xsd__string			sp__MustNotSendRenew;

	xsd__string			sp__MustSupportRefKeyIdentifier;
	xsd__string			sp__MustSupportRefIssuerSerial;
	xsd__string			sp__MustSupportRefExternalURI;
	xsd__string			sp__MustSupportRefEmbeddedToken;
	xsd__string			sp__MustSupportRefThumbprint;
	xsd__string			sp__MustSupportRefEncryptedKey;
	xsd__string			sp__RequireSignatureConfirmation;

	xsd__string			sp__MustSupportClientChallenge;
	xsd__string			sp__MustSupportServerChallenge;
	xsd__string			sp__RequireClientEntropy;
	xsd__string			sp__RequireServerEntropy;
	xsd__string			sp__MustSupportIssuedTokens;
	xsd__string			sp__RequireRequestSecurityTokenCollection;
	xsd__string			sp__RequireAppliesTo;

	xsd__string			sp__RequireExternalUriReference;
	xsd__string			sp__SC13SecurityContextToken;

	xsd__string			sp__Strict;
	xsd__string			sp__Lax;
	xsd__string			sp__LaxTsFirst;
	xsd__string			sp__LaxTsLast;

	xsd__string			sp__HttpBasicAuthentication;
	xsd__string			sp__HttpDigestAuthentication;
	xsd__string			sp__RequireClientCertificate;

	xsd__string			sp__Basic256;
	xsd__string			sp__Basic192;
	xsd__string			sp__Basic128;
	xsd__string			sp__TripleDes;
	xsd__string			sp__Basic256Rsa15;
	xsd__string			sp__Basic192Rsa15;
	xsd__string			sp__Basic128Rsa15;
	xsd__string			sp__TripleDesRsa15;
	xsd__string			sp__Basic256Sha256;
	xsd__string			sp__Basic192Sha256;
	xsd__string			sp__Basic128Sha256;
	xsd__string			sp__TripleDesSha256;
	xsd__string			sp__Basic256Sha256Rsa15;
	xsd__string			sp__Basic192Sha256Rsa15;
	xsd__string			sp__Basic128Sha256Rsa15;
	xsd__string			sp__TripleDesSha256Rsa15;

	xsd__string			sp__InclusiveC14N;
	xsd__string			sp__SOAPNormalization10;
	xsd__string			sp__STRTransform10;

	xsd__string			sp__Path10;
	xsd__string			sp__XPathFilter20;
	xsd__string			sp__AbsXPath;

	xsd__string			wsam__AnonymousResponses;
	xsd__string			wsam__NonAnonymousResponses;

	std::vector<_XML>		__any;
  public:
	int				traverse(wsdl__definitions&);
	void				generate(Service& service, Types& types, int indent) const;
};

class wsp__Policy : public wsp__Content
{ public:
	@xsd__anyURI			xml__base;
	@xsd__string			wsu__Id;
	@xsd__anyURI			TargetNamespace;
};

class wsp__Attachment
{ public:
	wsp__Policy			*Policy;
	wsp__PolicyReference		*PolicyReference;
};

class wsp__AppliesTo
{ public:
	_XML				__any;
};

class wsp__PolicyAttachment
{ public:
	wsp__AppliesTo			*AppliesTo;
	std::vector<wsp__Attachment>	Attachment;
};
