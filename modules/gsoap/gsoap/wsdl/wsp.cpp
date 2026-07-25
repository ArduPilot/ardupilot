/*
	wsp.cpp

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

#include "wsdlH.h"
#include "includes.h"
#include "types.h"
#include "service.h"

static wsp__Policy *search(const char *URI, wsdl__definitions& definitions);
static wsp__Policy *search(const char *URI, wsp__Policy *policy);
static wsp__Policy *search(const char *URI, wsp__Content *content);
static void gen_parts(const sp__Parts& parts, Types& types, const char *what, const char *name, int indent);

////////////////////////////////////////////////////////////////////////////////
//
//	wsp:OperatorContentType
//
////////////////////////////////////////////////////////////////////////////////

int wsp__Content::traverse(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << "  Analyzing wsp Policy" << std::endl;
  if (Policy)
    Policy->traverse(definitions);
  if (PolicyReference)
    PolicyReference->traverse(definitions);
  for (std::vector<wsp__Content*>::iterator i = All.begin(); i != All.end(); ++i)
  {
    if (*i)
      (*i)->traverse(definitions);
  }
  for (std::vector<wsp__Content*>::iterator j = ExactlyOne.begin(); j != ExactlyOne.end(); ++j)
  {
    if (*j)
      (*j)->traverse(definitions);
  }
  return SOAP_OK;
}

void wsp__Content::generate(Service& service, Types& types, int indent) const
{
  static const char stabs[] = "\t\t\t\t\t\t\t\t\t\t";
  const char *tabs;
  if (indent > 8)
    indent = 8;
  tabs = stabs + 9 - indent;
  // Recursive policies and references
  if (Policy)
    Policy->generate(service, types, indent);
  if (PolicyReference && PolicyReference->policyPtr())
    PolicyReference->policyPtr()->generate(service, types, indent);
  // WS-Policy All
  if (!All.empty())
  {
    fprintf(stream, "%s- All of the following:\n", tabs);
    for (std::vector<wsp__Content*>::const_iterator p = All.begin(); p != All.end(); ++p)
      if (*p)
        (*p)->generate(service, types, indent + 1);
  }
  // WS-Policy ExactlyOne
  if (!ExactlyOne.empty())
  {
    fprintf(stream, "%s- Exactly one of the following:\n", tabs);
    for (std::vector<wsp__Content*>::const_iterator p = ExactlyOne.begin(); p != ExactlyOne.end(); ++p)
      if (*p)
        (*p)->generate(service, types, indent + 1);
  }
  // WS-SecurityPolicy Parts (TODO: do we need vectors of these?)
  for (std::vector<sp__Parts>::const_iterator sp = sp__SignedParts.begin(); sp != sp__SignedParts.end(); ++sp)
    gen_parts(*sp, types, "sign", "[4.1.1] WS-Security Signed Parts", indent);
  for (std::vector<sp__Parts>::const_iterator ep = sp__EncryptedParts.begin(); ep != sp__EncryptedParts.end(); ++ep)
    gen_parts(*ep, types, "encrypt", "[4.2.1] Security Encrypted Parts", indent);
  for (std::vector<sp__Parts>::const_iterator rp = sp__RequiredParts.begin(); rp != sp__RequiredParts.end(); ++rp)
  {
    fprintf(stream, "%s- Required Header elements:", tabs);
    for (std::vector<sp__Header>::const_iterator h = (*rp).Header.begin(); h != (*rp).Header.end(); ++h)
      if ((*h).Name)
        fprintf(stream, " %s", types.aname(NULL, (*h).Namespace, (*h).Name));
      else if ((*h).Namespace)
        fprintf(stream, " %s", (*h).Namespace);
  }
  // WS-SecurityPolicy Elements
  sp__Elements *elts = NULL;
  const char *elts_name = NULL;
  if (sp__SignedElements)
  {
    elts = sp__SignedElements;
    elts_name = "[4.1.2] Signed";
  }
  if (sp__EncryptedElements)
  {
    elts = sp__EncryptedElements;
    elts_name = "[4.2.2] Encrypted";
  }
  if (sp__ContentEncryptedElements)
  {
    elts = sp__ContentEncryptedElements;
    elts_name = "[4.2.3] Content Encrypted";
  }
  if (sp__RequiredElements)
  {
    elts = sp__RequiredElements;
    elts_name = "[4.3.1] Required";
  }
  if (elts)
  {
    fprintf(stream, "%s- %s Elements requirements (XPath%s):\n%s  @verbatim\n", tabs, elts_name, elts->XPathVersion?elts->XPathVersion:"", tabs);
    for (std::vector<xsd__string>::const_iterator s = elts->XPath.begin(); s != elts->XPath.end(); ++s)
    {
      fprintf(stream, "%s  ", tabs);
      text(*s);
    }
    fprintf(stream, "%s  @endverbatim\n", tabs);
    service.add_import("wsse.h");
  }
  // WS-SecurityPolicy Tokens
  sp__Token *token = NULL;
  const char *token_name = NULL;
  if (sp__UsernameToken)
  {
    token = sp__UsernameToken;
    token_name = "[5.4.1] WS-Security Username";
  }
  else if (sp__IssuedToken)
  {
    token = sp__IssuedToken;
    token_name = "[5.4.2] WS-Trust Issued";
  }
  else if (sp__X509Token)
  {
    token = sp__X509Token;
    token_name = "[5.4.3] WS-Security X509";
  }
  else if (sp__KerberosToken)
  {
    token = sp__KerberosToken;
    token_name = "[5.4.4] WS-Security Kerberos";
  }
  else if (sp__SpnegoContextToken)
  {
    token = sp__SpnegoContextToken;
    token_name = "[5.4.5] WS-Trust n-leg RST/RSTR SPNEGO binary negotiation protocol (SpnegoContext)";
  }
  else if (sp__SecurityContextToken)
  {
    token = sp__SecurityContextToken;
    token_name = "[5.4.6] WS-SecureConversation SecurityContext";
  }
  else if (sp__SecureConversationToken)
  {
    token = sp__SecureConversationToken;
    token_name = "[5.4.7] WS-SecureConversation";
  }
  else if (sp__SamlToken)
  {
    token = sp__SamlToken;
    token_name = "[5.4.8] SAML";
  }
  else if (sp__RelToken)
  {
    token = sp__RelToken;
    token_name = "[5.4.9] WSS-REL";
  }
  else if (sp__HttpsToken)
  {
    token = sp__HttpsToken;
    token_name = "[5.4.10] HTTPS";
  }
  else if (sp__KeyValueToken)
  {
    token = sp__KeyValueToken;
    token_name = "[5.4.11] XML Signature";
  }
  if (token)
  {
    fprintf(stream, "%s- %s required:\n", tabs, token_name);
    if (token->IncludeToken)
      fprintf(stream, "%s  -# IncludeToken = %s\n", tabs, token->IncludeToken);
    if (token->Issuer && token->Issuer->Address)
      fprintf(stream, "%s  -# Issuer       = %s\n", tabs, token->Issuer->Address);
    if (token->IssuerName)
      fprintf(stream, "%s  -# Issuer Name  = %s\n", tabs, token->IssuerName);
    if (token->Policy)
      token->Policy->generate(service, types, indent + 1);
    // TODO: add wst:Claims?
    service.add_import("wsse.h");
  }
  // WS-SecurityPolicy
  if (sp__AlgorithmSuite)
  {
    fprintf(stream, "%s- [7.1] Security Binding Algorithm Suite requirements:\n", tabs);
    if (sp__AlgorithmSuite->Policy)
      sp__AlgorithmSuite->Policy->generate(service, types, indent + 1);
  }
  if (sp__Layout)
  {
    fprintf(stream, "%s- [7.2] WS-Security Header Layout requirements:\n", tabs);
    if (sp__Layout->Policy)
      sp__Layout->Policy->generate(service, types, indent + 1);
  }
  if (sp__TransportBinding)
  {
    fprintf(stream, "%s- [7.3] Transport Binding%s requirements:\n", tabs, sp__TransportBinding->Optional ? " (optional)" : sp__TransportBinding->Ignorable ? " (ignorable)" : "");
    if (sp__TransportBinding->Policy)
      sp__TransportBinding->Policy->generate(service, types, indent + 1);
  }
  if (sp__TransportToken)
  {
    fprintf(stream, "%s- Transport%s requirements:\n", tabs, sp__TransportToken->Optional ? " (optional)" : sp__TransportToken->Ignorable ? " (ignorable)" : "");
    if (sp__TransportToken->Policy)
      sp__TransportToken->Policy->generate(service, types, indent + 1);
  }
  if (sp__SymmetricBinding)
  {
    fprintf(stream, "%s- [7.4] WS-Security Symmetric Binding%s requirements:\n", tabs, sp__SymmetricBinding->Optional ? " (optional)" : sp__SymmetricBinding->Ignorable ? " (ignorable)" : "");
    if (sp__SymmetricBinding->Policy)
      sp__SymmetricBinding->Policy->generate(service, types, indent + 1);
    service.add_import("wsse.h");
  }
  if (sp__ProtectionToken)
  {
    fprintf(stream, "%s- Symmetric Protection%s requirements:\n", tabs, sp__ProtectionToken->Optional ? " (optional)" : sp__ProtectionToken->Ignorable ? " (ignorable)" : "");
    if (sp__ProtectionToken->Policy)
      sp__ProtectionToken->Policy->generate(service, types, indent + 1);
  }
  if (sp__AsymmetricBinding)
  {
    fprintf(stream, "%s- [7.5] WS-Security Asymmetric Binding%s (public key) requirements:\n", tabs, sp__AsymmetricBinding->Optional ? " (optional)" : sp__AsymmetricBinding->Ignorable ? " (ignorable)" : "");
    if (sp__AsymmetricBinding->Policy)
      sp__AsymmetricBinding->Policy->generate(service, types, indent + 1);
    service.add_import("wsse.h");
  }
  if (sp__InitiatorToken)
  {
    fprintf(stream, "%s- Initiator%s requirements:\n", tabs, sp__InitiatorToken->Optional ? " (optional)" : sp__InitiatorToken->Ignorable ? " (ignorable)" : "");
    if (sp__InitiatorToken->Policy)
      sp__InitiatorToken->Policy->generate(service, types, indent + 1);
  }
  if (sp__InitiatorSignatureToken)
  {
    fprintf(stream, "%s- Initiator Signature%s requirements:\n", tabs, sp__InitiatorSignatureToken->Optional ? " (optional)" : sp__InitiatorSignatureToken->Ignorable ? " (ignorable)" : "");
    if (sp__InitiatorSignatureToken->Policy)
      sp__InitiatorSignatureToken->Policy->generate(service, types, indent + 1);
  }
  if (sp__InitiatorEncryptionToken)
  {
    fprintf(stream, "%s- Initiator Encryption%s requirements:\n", tabs, sp__InitiatorEncryptionToken->Optional ? " (optional)" : sp__InitiatorEncryptionToken->Ignorable ? " (ignorable)" : "");
    if (sp__InitiatorEncryptionToken->Policy)
      sp__InitiatorEncryptionToken->Policy->generate(service, types, indent + 1);
  }
  if (sp__RecipientToken)
  {
    fprintf(stream, "%s- Recipient%s requirements:\n", tabs, sp__RecipientToken->Optional ? " (optional)" : sp__RecipientToken->Ignorable ? " (ignorable)" : "");
    if (sp__RecipientToken->Policy)
      sp__RecipientToken->Policy->generate(service, types, indent + 1);
  }
  if (sp__SupportingTokens)
  {
    fprintf(stream, "%s- [8.1] Supporting Tokens%s requirements:\n", tabs, sp__SupportingTokens->Optional ? " (optional)" : sp__SupportingTokens->Ignorable ? " (ignorable)" : "");
    if (sp__SupportingTokens->Policy)
      sp__SupportingTokens->Policy->generate(service, types, indent + 1);
  }
  if (sp__SignedSupportingTokens)
  {
    fprintf(stream, "%s- [8.2] Signed Supporting Tokens%s requirements:\n", tabs, sp__SignedSupportingTokens->Optional ? " (optional)" : sp__SignedSupportingTokens->Ignorable ? " (ignorable)" : "");
    if (sp__SignedSupportingTokens->Policy)
      sp__SignedSupportingTokens->Policy->generate(service, types, indent + 1);
  }
  if (sp__EndorsingSupportingTokens)
  {
    fprintf(stream, "%s- [8.3] Endorsing Supporting Tokens%s requirements:\n", tabs, sp__EndorsingSupportingTokens->Optional ? " (optional)" : sp__EndorsingSupportingTokens->Ignorable ? " (ignorable)" : "");
    if (sp__EndorsingSupportingTokens->Policy)
      sp__EndorsingSupportingTokens->Policy->generate(service, types, indent + 1);
  }
  if (sp__SignedEndorsingSupportingTokens)
  {
    fprintf(stream, "%s- [8.4] Signed Endorsing Supporting Tokens%s requirements:\n", tabs, sp__SignedEndorsingSupportingTokens->Optional ? " (optional)" : sp__SignedEndorsingSupportingTokens->Ignorable ? " (ignorable)" : "");
    if (sp__SignedEndorsingSupportingTokens->Policy)
      sp__SignedEndorsingSupportingTokens->Policy->generate(service, types, indent + 1);
  }
  if (sp__SignedEncryptedSupportingTokens)
  {
    fprintf(stream, "%s- [8.5] Signed Encrypted Supporting Tokens%s requirements:\n", tabs, sp__SignedEncryptedSupportingTokens->Optional ? " (optional)" : sp__SignedEncryptedSupportingTokens->Ignorable ? " (ignorable)" : "");
    if (sp__SignedEncryptedSupportingTokens->Policy)
      sp__SignedEncryptedSupportingTokens->Policy->generate(service, types, indent + 1);
  }
  if (sp__EncryptedSupportingTokens)
  {
    fprintf(stream, "%s- [8.6] Encrypted Supporting Tokens%s requirements:\n", tabs, sp__EncryptedSupportingTokens->Optional ? " (optional)" : sp__EncryptedSupportingTokens->Ignorable ? " (ignorable)" : "");
    if (sp__EncryptedSupportingTokens->Policy)
      sp__EncryptedSupportingTokens->Policy->generate(service, types, indent + 1);
  }
  if (sp__EndorsingEncryptedSupportingTokens)
  {
    fprintf(stream, "%s- [8.7] Endorsing Encrypted Supporting Tokens%s requirements:\n", tabs, sp__EndorsingEncryptedSupportingTokens->Optional ? " (optional)" : sp__EndorsingEncryptedSupportingTokens->Ignorable ? " (ignorable)" : "");
    if (sp__EndorsingEncryptedSupportingTokens->Policy)
      sp__EndorsingEncryptedSupportingTokens->Policy->generate(service, types, indent + 1);
  }
  if (sp__SignedEndorsingEncryptedSupportingTokens)
  {
    fprintf(stream, "%s- [8.8] Signed Endorsing Encrypted Supporting Tokens%s requirements:\n", tabs, sp__SignedEndorsingEncryptedSupportingTokens->Optional ? " (optional)" : sp__SignedEndorsingEncryptedSupportingTokens->Ignorable ? " (ignorable)" : "");
    if (sp__SignedEndorsingEncryptedSupportingTokens->Policy)
      sp__SignedEndorsingEncryptedSupportingTokens->Policy->generate(service, types, indent + 1);
  }
  // Wss10 or Wss11
  if (sp__Wss10)
  {
    fprintf(stream, "%s- [9.1] WSS: SOAP Message Security 1.0%s options:\n", tabs, sp__Wss10->Optional ? " (optional)" : sp__Wss10->Ignorable ? " (ignorable)" : "");
    if (sp__Wss10->Policy)
      sp__Wss10->Policy->generate(service, types, indent + 1);
    service.add_import("wsse.h");
  }
  else if (sp__Wss11)
  {
    fprintf(stream, "%s- [9.2] WSS: SOAP Message Security 1.1%s options:\n", tabs, sp__Wss11->Optional ? " (optional)" : sp__Wss11->Ignorable ? " (ignorable)" : "");
    if (sp__Wss11->Policy)
      sp__Wss11->Policy->generate(service, types, indent + 1);
    service.add_import("wsse.h");
  }
  if (sp__MustSupportRefKeyIdentifier)
    fprintf(stream, "%s- Key Identifier References\n", tabs);
  if (sp__MustSupportRefIssuerSerial)
    fprintf(stream, "%s- Issuer Serial References\n", tabs);
  if (sp__MustSupportRefExternalURI)
    fprintf(stream, "%s- External URI References\n", tabs);
  if (sp__MustSupportRefEmbeddedToken)
    fprintf(stream, "%s- Embedded Token References\n", tabs);
  if (sp__MustSupportRefThumbprint)
    fprintf(stream, "%s- Thumbprint References\n", tabs);
  if (sp__MustSupportRefEncryptedKey)
    fprintf(stream, "%s- EncryptedKey References\n", tabs);
  if (sp__RequireSignatureConfirmation)
    fprintf(stream, "%s- Signature Confirmation\n", tabs);
  // WS-SecureConversation
  if (sp__RequireDerivedKeys)
    fprintf(stream, "%s- Properties   = WS-SecureConversation RequireDerivedKeys\n", tabs);
  else if (sp__RequireImpliedDerivedKeys)
    fprintf(stream, "%s- Properties   = WS-SecureConversation RequireImpliedDerivedKeys\n", tabs);
  else if (sp__RequireExplicitDerivedKeys)
    fprintf(stream, "%s- Properties   = WS-SecureConversation RequireExplicitDerivedKeys\n", tabs);
  if (sp__MustNotSendCancel)
    fprintf(stream, "%s- WS-SecureConversation STS issuing the secure conversation token does not support SCT/Cancel RST messages", tabs);
  else if (sp__MustNotSendAmend)
    fprintf(stream, "%s- WS-SecureConversation STS issuing the secure conversation token does not support SCT/Amend RST messages", tabs);
  else if (sp__MustNotSendRenew)
    fprintf(stream, "%s- WS-SecureConversation STS issuing the secure conversation token does not support SCT/Renew RST messages", tabs);
  if (sp__RequireExternalUriReference)
    fprintf(stream, "%s- WS-SecureConversation external URI reference is required", tabs);
  if (sp__SC13SecurityContextToken)
    fprintf(stream, "%s- WS-SecureConversation Security Context Token should be used", tabs);
  // WS-Security passwords
  if (sp__NoPassword)
    fprintf(stream, "%s- No WS-Security password%s required\n", tabs, sp__NoPassword->Optional ? " (optional)" : sp__NoPassword->Ignorable ? " (ignorable)" : "");
  else if (sp__HashPassword)
  {
    fprintf(stream, "%s- Client-side WS-Security password%s should be set:\n\t@code\n\t#include \"plugin/wsseapi.h\"\n\tsoap_wsse_add_UsernameTokenDigest(soap, \"User\", \"<username>\", \"<password>\");\n\t@endcode\n", tabs, sp__HashPassword->Optional ? " (optional)" : sp__HashPassword->Ignorable ? " (ignorable)" : "");
    fprintf(stream, "%s- Server-side WS-Security password%s verified with:\n\t@code\n\t#include \"plugin/wsseapi.h\"\n\tconst char *username = soap_wsse_get_Username(soap);\n\t...\n\tif (soap_wsse_verify_Password(soap, \"<password>\")) ...<error>...\n\t@endcode\n", tabs, sp__HashPassword->Optional ? " (optional)" : sp__HashPassword->Ignorable ? " (ignorable)" : "");
    service.add_import("wsse.h");
  }
  if (sp__WssUsernameToken10)
  {
    fprintf(stream, "%s- Username token should be used as defined in UsernameTokenProfile1.0:\n", tabs);
    fprintf(stream, "%s  - Client-side WS-Security password should be set:\n\t@code\n\t#include \"plugin/wsseapi.h\"\n\tsoap_wsse_add_UsernameTokenDigest(soap, \"User\", \"<username>\", \"<password>\");\n\t@endcode\n", tabs);
    fprintf(stream, "%s  - Server-side WS-Security password verified with:\n\t@code\n\t#include \"plugin/wsseapi.h\"\n\tconst char *username = soap_wsse_get_Username(soap);\n\t...\n\tif (soap_wsse_verify_Password(soap, \"<password>\")) <error>\n\t@endcode\n", tabs);
    service.add_import("wsse.h");
  }
  else if (sp__WssUsernameToken11)
  {
    fprintf(stream, "%s- Username token should be used as defined in UsernameTokenProfile1.1:\n", tabs);
    fprintf(stream, "%s  - Client-side WS-Security plain-text password should be set:\n\t@code\n\t#include \"plugin/wsseapi.h\"\n\tsoap_wsse_add_UsernameToken(soap, \"User\", \"<username>\", \"<password>\");\n\t@endcode\n", tabs);
    fprintf(stream, "%s  - Client-side WS-Security digest password should be set:\n\t@code\n\t#include \"plugin/wsseapi.h\"\n\tsoap_wsse_add_UsernameTokenDigest(soap, \"User\", \"<username>\", \"<password>\");\n\t@endcode\n", tabs);
    fprintf(stream, "%s  - Server-side WS-Security password verified with:\n\t@code\n\t#include \"plugin/wsseapi.h\"\n\tconst char *username = soap_wsse_get_Username(soap);\n\t...\n\tif (soap_wsse_verify_Password(soap, \"<password>\")) ...\n\t@endcode\n", tabs);
    service.add_import("wsse.h");
  }
  // WS-Trust
  if (sp__RequireExternalReference)
    fprintf(stream, "%s- WS-Trust external reference is required when referencing this token\n", tabs);
  else if (sp__RequireInternalReference)
    fprintf(stream, "%s- WS-Trust internal reference is required when referencing this token\n", tabs);
  // WS-Trust 1.0 and 1.3
  if (sp__Trust10)
  {
    fprintf(stream, "%s- [10.1] WS-Trust 1.0%s options:\n", tabs, sp__Trust10->Optional ? " (optional)" : sp__Trust10->Ignorable ? " (ignorable)" : "");
    if (sp__Trust10->Policy)
      sp__Trust10->Policy->generate(service, types, indent + 1);
    service.add_import("wst.h");
  }
  else if (sp__Trust13)
  {
    fprintf(stream, "%s- [10.1] WS-Trust 1.3%s options:\n", tabs, sp__Trust13->Optional ? " (optional)" : sp__Trust13->Ignorable ? " (ignorable)" : "");
    if (sp__Trust13->Policy)
      sp__Trust13->Policy->generate(service, types, indent + 1);
    service.add_import("wst.h");
  }
  if (sp__MustSupportClientChallenge)
  {
    fprintf(stream, "%s- Client Challenge\n", tabs);
    service.add_import("wst.h");
  }
  if (sp__MustSupportServerChallenge)
  {
    fprintf(stream, "%s- Server Challenge\n", tabs);
    service.add_import("wst.h");
  }
  if (sp__RequireClientEntropy)
  {
    fprintf(stream, "%s- Client Entropy\n", tabs);
    service.add_import("wst.h");
  }
  if (sp__RequireServerEntropy)
  {
    fprintf(stream, "%s- Server Entropy\n", tabs);
    service.add_import("wst.h");
  }
  if (sp__MustSupportIssuedTokens)
  {
    fprintf(stream, "%s- Issued Tokens\n", tabs);
    service.add_import("wst.h");
  }
  if (sp__RequireRequestSecurityTokenCollection)
  {
    fprintf(stream, "%s- Collection\n", tabs);
    service.add_import("wst.h");
  }
  if (sp__RequireAppliesTo)
  {
    fprintf(stream, "%s-  STS requires the requestor to specify the scope for the issued token using wsp:AppliesTo in the RST\n", tabs);
    service.add_import("wst.h");
  }
  // WS-Security header layout
  if (sp__IncludeTimestamp)
  {
    fprintf(stream, "%s- WS-Security Timestamp%s should be set prior to send:\n\t@code\n\t#include \"plugin/wsseapi.h\"\n\tsoap_wsse_add_Timestamp(soap, \"Timestamp\", <seconds>);\n\t@endcode\n", tabs, sp__IncludeTimestamp->Optional ? " (optional)" : sp__IncludeTimestamp->Ignorable ? " (ignorable)" : "");
    fprintf(stream, "%s- WS-Security Timestamp%s presence and expiration verified post-receive with:\n\t@code\n\t#include \"plugin/wsseapi.h\"\n\tif (soap_wsse_verify_Timestamp(soap)) ...<error>...\n\t@endcode\n", tabs, sp__IncludeTimestamp->Optional ? " (optional)" : sp__IncludeTimestamp->Ignorable ? " (ignorable)" : "");
  }
  if (sp__EncryptBeforeSigning)
    fprintf(stream, "%s- WS-Security Encrypt Before Signing%s (gSOAP unsupported)\n", tabs, sp__EncryptBeforeSigning->Optional ? " (optional)" : sp__EncryptBeforeSigning->Ignorable ? " (ignorable)" : "");
  if (sp__EncryptSignature)
    fprintf(stream, "%s- WS-Security Encrypt Signature%s\n\t@code\n\t#include \"plugin/wsseapi.h\"\n\tsoap_wsse_add_EncryptedKey_encrypt_only(soap, <SOAP_MEC_ENV_ENC_xxx_CBC>, NULL, <cert>, NULL, <issuer>, <serial>, \"ds:Signature SOAP-ENV:Body\");\n\t@endcode\n", tabs, sp__EncryptSignature->Optional ? " (optional)" : sp__EncryptSignature->Ignorable ? " (ignorable)" : "");
  if (sp__ProtectTokens)
    fprintf(stream, "%s- WS-Security Token Protection%s required\n", tabs, sp__ProtectTokens->Optional ? " (optional)" : sp__ProtectTokens->Ignorable ? " (ignorable)" : "");
  if (sp__OnlySignEntireHeadersAndBody)
  {
    fprintf(stream, "%s- WS-Security Sign Entire Headers and Body%s:\n\t@code\n\t#include \"plugin/wsseapi.h\"\n\tsoap_wsse_set_wsu_id(soap, \"<ns:tagname1> <ns:tagname2> ...\"); // list each ns:tagname used in SOAP Header\n\tsoap_wsse_sign_body(soap, <algorithm>, <key>, <keylen>);\n\t@endcode\n", tabs, sp__OnlySignEntireHeadersAndBody->Optional ? " (optional)" : sp__OnlySignEntireHeadersAndBody->Ignorable ? " (ignorable)" : "");
  }
  if (sp__Strict)
    fprintf(stream, "%s- WS-Security headers 'declare before use' required (gSOAP default)\n", tabs);
  else if (sp__Lax)
    fprintf(stream, "%s- WS-Security headers may occur in any order (gSOAP allows this)\n", tabs);
  else if (sp__LaxTsFirst)
    fprintf(stream, "%s- WS-Security Timestamp must appear first (gSOAP default)\n", tabs);
  else if (sp__LaxTsLast)
    fprintf(stream, "%s- WS-Security Timestamp must appear last (requires changing the placement of the Timestamp header in SOAP_ENV__Header defined in import/wsse.h)\n", tabs);
  // HTTP authentication
  if (sp__HttpBasicAuthentication)
    fprintf(stream, "%s- HTTP/S Basic Authentication required:\n\t@code\n\tsoap->userid = \"<userid>\"; soap->passwd = \"<passwd>\";\nsoap_call_ns__method(...)\n\t@endcode\n", tabs);
  else if (sp__HttpDigestAuthentication)
    fprintf(stream, "%s- HTTP/S Digest Authentication required:\n%sSee plugin/httpda.c plugin for usage details\n", tabs, tabs);
  if (sp__RequireClientCertificate)
    fprintf(stream, "%s- HTTPS client must authenticate to server with a certificate:\n\t@code\n\tsoap_ssl_client_context(soap, <sslflags>, \"<certkeyfile>\", \"<certkeypw>\", ...)\n\t@endcode\n", tabs);
  //  Security token requirements
  if (sp__RequireKeyIdentifierReference)
    fprintf(stream, "%s- Key identifier reference is required\n", tabs);
  if (sp__RequireIssuerSerialReference)
    fprintf(stream, "%s- Issuer serial reference is required\n", tabs);
  if (sp__RequireEmbeddedTokenReference)
    fprintf(stream, "%s- An embedded token reference is required\n", tabs);
  if (sp__RequireThumbprintReference)
    fprintf(stream, "%s- A thumbprint reference is required\n", tabs);
  // Algorithm suite
  if (sp__Basic256)
    fprintf(stream, "%s- Basic256\n", tabs);
  else if (sp__Basic192)
    fprintf(stream, "%s- Basic192\n", tabs);
  else if (sp__Basic128)
    fprintf(stream, "%s- Basic128\n", tabs);
  else if (sp__TripleDes)
    fprintf(stream, "%s- TripleDes\n", tabs);
  else if (sp__Basic256Rsa15)
    fprintf(stream, "%s- Basic256Rsa15\n", tabs);
  else if (sp__Basic192Rsa15)
    fprintf(stream, "%s- Basic192Rsa15\n", tabs);
  else if (sp__Basic128Rsa15)
    fprintf(stream, "%s- Basic128Rsa15\n", tabs);
  else if (sp__TripleDesRsa15)
    fprintf(stream, "%s- TripleDesRsa15\n", tabs);
  else if (sp__Basic256Sha256)
    fprintf(stream, "%s- Basic256Sha256\n", tabs);
  else if (sp__Basic192Sha256)
    fprintf(stream, "%s- Basic192Sha256\n", tabs);
  else if (sp__Basic128Sha256)
    fprintf(stream, "%s- Basic128Sha256\n", tabs);
  else if (sp__TripleDesSha256)
    fprintf(stream, "%s- TripleDesSha256\n", tabs);
  else if (sp__Basic256Sha256Rsa15)
    fprintf(stream, "%s- Basic256Sha256Rsa15\n", tabs);
  else if (sp__Basic192Sha256Rsa15)
    fprintf(stream, "%s- Basic192Sha256Rsa15\n", tabs);
  else if (sp__Basic128Sha256Rsa15)
    fprintf(stream, "%s- Basic128Sha256Rsa15\n", tabs);
  else if (sp__TripleDesSha256Rsa15)
    fprintf(stream, "%s- TripleDesSha256Rsa15\n", tabs);
  if (sp__InclusiveC14N)
    fprintf(stream, "%s- InclusiveC14N\n", tabs);
  if (sp__SOAPNormalization10)
    fprintf(stream, "%s- SOAPNormalization10\n", tabs);
  if (sp__STRTransform10)
    fprintf(stream, "%s- STRTransform10\n", tabs);
  if (sp__Path10)
    fprintf(stream, "%s- Path10\n", tabs);
  else if (sp__XPathFilter20)
    fprintf(stream, "%s- XPathFilter20\n", tabs);
  else if (sp__AbsXPath)
    fprintf(stream, "%s- AbsXPath\n", tabs);
  // WSS
  if (sp__WssX509V3Token10)
    fprintf(stream, "%s- An X509 Version 3 token should be used as defined in X509TokenProfile1.0\n", tabs);
  else if (sp__WssX509Pkcs7Token10)
    fprintf(stream, "%s- An X509 PKCS7 token should be used as defined in X509TokenProfile1.0\n", tabs);
  else if (sp__WssX509PkiPathV1Token10)
    fprintf(stream, "%s- An X509 PKI Path Version 1 token should be used as defined in X509TokenProfile1.0\n", tabs);
  else if (sp__WssX509V1Token11)
    fprintf(stream, "%s- An X509 Version 1 token should be used as defined in X509TokenProfile1.1\n", tabs);
  else if (sp__WssX509V3Token11)
    fprintf(stream, "%s- An X509 Version 3 token should be used as defined in X509TokenProfile1.1\n", tabs);
  else if (sp__WssX509Pkcs7Token11)
    fprintf(stream, "%s- An X509 PKCS7 token should be used as defined in X509TokenProfile1.1\n", tabs);
  else if (sp__WssX509PkiPathV1Token11)
    fprintf(stream, "%s- An X509 PKI Path Version 1 token should be used as defined in X509TokenProfile1.1\n", tabs);
  if (sp__WssKerberosV5ApReqToken11)
    fprintf(stream, "%s- A Kerberos Version 5 AP-REQ X509 token should be used as defined in KerberosTokenProfile1.1\n", tabs);
  else if (sp__WssGssKerberosV5ApReqToken11)
    fprintf(stream, "%s- A GSS Kerberos Version 5 AP-REQ token should be used as defined in KerberosTokenProfile1.1\n", tabs);
  if (sp__WssRelV10Token10)
    fprintf(stream, "%s- A REL Version 1.0 token should be used as defined in RELTokenProfile1.0\n", tabs);
  else if (sp__WssRelV20Token10)
    fprintf(stream, "%s- A REL Version 2.0 token should be used as defined in RELTokenProfile1.0\n", tabs);
  else if (sp__WssRelV10Token11)
    fprintf(stream, "%s- A REL Version 1.0 token should be used as defined in RELTokenProfile1.1\n", tabs);
  else if (sp__WssRelV20Token11)
    fprintf(stream, "%s- A REL Version 2.0 token should be used as defined in RELTokenProfile1.1\n", tabs);
  if (sp__BootstrapPolicy)
  {
    fprintf(stream, "%s- SecureConversation BootstrapPolicy\n", tabs);
    sp__BootstrapPolicy->generate(service, types, indent + 1);
  }
  // WS-Addressing WSDL Policy
  if (wsaw__UsingAddressing)
  {
    fprintf(stream, "%s- WS-Addressing is used\n", tabs);
    service.add_import("wsa5.h");
  }
  // WS-Addressing Metadata Policy
  if (wsam__Addressing)
  {
    fprintf(stream, "%s- WS-Addressing%s is used\n", tabs, wsam__Addressing->Optional ? " (optional)" : wsam__Addressing->Ignorable ? " (ignorable)" : "");
    if (wsam__Addressing->Policy)
      wsam__Addressing->Policy->generate(service, types, indent + 1);
    service.add_import("wsa5.h");
  }
  if (wsam__AnonymousResponses)
    fprintf(stream, "%s- WS-Addressing Anonymous Responses\n", tabs);
  else if (wsam__NonAnonymousResponses)
    fprintf(stream, "%s- WS-Addressing NonAnonymous Responses\n", tabs);
  // WS-ReliableMessaging Policy 2007
  if (wsrmp__RMAssertion_)
  {
    fprintf(stream, "%s- WS-ReliableMessaging%s is used\n", tabs, wsrmp__RMAssertion_->Optional ? " (optional)" : wsrmp__RMAssertion_->Ignorable ? " (ignorable)" : "");
    if (wsrmp__RMAssertion_->InactivityTimeout && wsrmp__RMAssertion_->InactivityTimeout->Milliseconds)
      fprintf(stream, "%s  - Inactivity Timeout = %s (ms)\n", tabs, wsrmp__RMAssertion_->InactivityTimeout->Milliseconds);
    if (wsrmp__RMAssertion_->BaseRetransmissionInterval && wsrmp__RMAssertion_->BaseRetransmissionInterval->Milliseconds)
      fprintf(stream, "%s  - Base Retransmission Interval = %s (ms)\n", tabs, wsrmp__RMAssertion_->BaseRetransmissionInterval->Milliseconds);
    if (wsrmp__RMAssertion_->AcknowledgementInterval && wsrmp__RMAssertion_->AcknowledgementInterval->Milliseconds)
      fprintf(stream, "%s  - Acknowledgement Interval = %s (ms)\n", tabs, wsrmp__RMAssertion_->AcknowledgementInterval->Milliseconds);
    if (wsrmp__RMAssertion_->ExponentialBackoff)
      fprintf(stream, "%s  - ExponentialBackoff\n", tabs);
    if (wsrmp__RMAssertion_->Policy)
      wsrmp__RMAssertion_->Policy->generate(service, types, indent + 1);
    service.add_import("wsrm.h");
  }
  if (wsrmp__DeliveryAssurance)
  {
    fprintf(stream, "%s- WS-ReliableMessaging Delivery Assurance%s:\n", tabs, wsrmp__DeliveryAssurance->Optional ? " (optional)" : wsrmp__DeliveryAssurance->Ignorable ? " (ignorable)" : "");
    if (wsrmp__DeliveryAssurance->Policy)
      wsrmp__DeliveryAssurance->Policy->generate(service, types, indent + 1);
    service.add_import("wsrm.h");
  }
  if (wsrmp__AtLeastOnce)
    fprintf(stream, "%s- At Least Once\n", tabs);
  if (wsrmp__AtMostOnce)
    fprintf(stream, "%s- At Most Once\n", tabs);
  if (wsrmp__ExactlyOnce)
    fprintf(stream, "%s- Exactly Once\n", tabs);
  if (wsrmp__InOrder)
    fprintf(stream, "%s- In Order\n", tabs);
  // WS-ReliableMessaging Policy 2005
  if (wsrmp5__RMAssertion_)
  {
    fprintf(stream, "%s- WS-ReliableMessaging%s is used\n", tabs, wsrmp5__RMAssertion_->Optional ? " (optional)" : wsrmp5__RMAssertion_->Ignorable ? " (ignorable)" : "");
    if (wsrmp5__RMAssertion_->InactivityTimeout && wsrmp5__RMAssertion_->InactivityTimeout->Milliseconds)
      fprintf(stream, "%s  - Inactivity Timeout = %s (ms)\n", tabs, wsrmp5__RMAssertion_->InactivityTimeout->Milliseconds);
    if (wsrmp5__RMAssertion_->BaseRetransmissionInterval && wsrmp5__RMAssertion_->BaseRetransmissionInterval->Milliseconds)
      fprintf(stream, "%s  - Base Retransmission Interval = %s (ms)\n", tabs, wsrmp5__RMAssertion_->BaseRetransmissionInterval->Milliseconds);
    if (wsrmp5__RMAssertion_->AcknowledgementInterval && wsrmp5__RMAssertion_->AcknowledgementInterval->Milliseconds)
      fprintf(stream, "%s  - Acknowledgement Interval = %s (ms)\n", tabs, wsrmp5__RMAssertion_->AcknowledgementInterval->Milliseconds);
    if (wsrmp5__RMAssertion_->ExponentialBackoff)
      fprintf(stream, "%s  - ExponentialBackoff\n", tabs);
    if (wsrmp5__RMAssertion_->Policy)
      wsrmp5__RMAssertion_->Policy->generate(service, types, indent + 1);
    service.add_import("wsrm5.h");
  }
  if (wsrmp5__DeliveryAssurance)
  {
    fprintf(stream, "%s- WS-ReliableMessaging Delivery Assurance%s:\n", tabs, wsrmp5__DeliveryAssurance->Optional ? " (optional)" : wsrmp5__DeliveryAssurance->Ignorable ? " (ignorable)" : "");
    if (wsrmp5__DeliveryAssurance->Policy)
      wsrmp5__DeliveryAssurance->Policy->generate(service, types, indent + 1);
    service.add_import("wsrm5.h");
  }
  if (wsrmp5__AtLeastOnce)
    fprintf(stream, "%s- At Least Once\n", tabs);
  if (wsrmp5__AtMostOnce)
    fprintf(stream, "%s- At Most Once\n", tabs);
  if (wsrmp5__ExactlyOnce)
    fprintf(stream, "%s- Exactly Once\n", tabs);
  if (wsrmp5__InOrder)
    fprintf(stream, "%s- In Order\n", tabs);
  // All else
  for (std::vector<_XML>::const_iterator x = __any.begin(); x != __any.end(); ++x)
  {
    if (*x && *(*x))
    {
      fprintf(stream, "%s- Other policy requirements:\n\t@verbatim\n", tabs);
      text(*x);
      fprintf(stream, "\t@endverbatim\n");
    }
  }
}

static void gen_parts(const sp__Parts& parts, Types& types, const char *what, const char *name, int indent)
{
  static const char stabs[] = "\t\t\t\t\t\t\t\t\t\t";
  const char *tabs;
  if (indent > 8)
    indent = 8;
  tabs = stabs + 9 - indent;
  fprintf(stream, "%s- %s requirements:\n", tabs, name);
  if (parts.Body)
    fprintf(stream, "%s  -# Body:\n\t@code\n\t#include \"plugin/wsseapi.h\"\n\tsoap_wsse_%s_body(soap, <algorithm>, <key>, <keylen>);\n\t@endcode\n", tabs, what);
  if (!parts.Header.empty())
  {
    fprintf(stream, "%s  -# Header elements:\n\t@code\n\t#include \"plugin/wsseapi.h\"\n\tsoap_wsse_set_wsu_id(soap, \"", tabs);
    for (std::vector<sp__Header>::const_iterator h = parts.Header.begin(); h != parts.Header.end(); ++h)
    {
      if ((*h).Name)
        fprintf(stream, "%s ", types.aname(NULL, (*h).Namespace, (*h).Name));
      else if ((*h).Namespace)
        fprintf(stream, "%s: ", types.nsprefix(NULL, (*h).Namespace));
    }
    fprintf(stream, "\");\n\t@endcode\n");
  }
  if (parts.Attachments)
    fprintf(stream, "%s  -# Attachments as defined in SwAProfile1.1\n", tabs);
}

////////////////////////////////////////////////////////////////////////////////
//
//	wsp:PolicyReference
//
////////////////////////////////////////////////////////////////////////////////

int wsp__PolicyReference::traverse(wsdl__definitions& definitions)
{
  policyRef = NULL;
  if (!URI || !*URI)
  {
    std::cerr << "PolicyReference has no URI" << std::endl;
    return SOAP_OK;
  }
  if (*URI == '#')
  {
    policyRef = search(URI + 1, definitions);
    if (!policyRef)
    {
      std::cerr << "PolicyReference URI=\"" << URI << "\" not found" << std::endl;
      return SOAP_OK;
    }
  }
  return SOAP_OK;
}

void wsp__PolicyReference::policyPtr(wsp__Policy *Policy)
{
  policyRef = Policy;
}

wsp__Policy *wsp__PolicyReference::policyPtr() const
{
  return policyRef;
}

static wsp__Policy *search(const char *URI, wsdl__definitions& definitions)
{
  for (std::vector<wsp__Policy>::iterator p = definitions.wsp__Policy_.begin(); p != definitions.wsp__Policy_.end(); ++p)
  {
    wsp__Policy *policy = search(URI, &(*p));
    if (policy)
      return policy;
  }
  return NULL;
}

static wsp__Policy *search(const char *URI, wsp__Policy *policy)
{
  if (!policy)
    return NULL;
  if (policy->wsu__Id && !strcmp(URI, policy->wsu__Id))
    return policy;
  return search(URI, (wsp__Content*)policy);
}

static wsp__Policy *search(const char *URI, wsp__Content *content)
{
  wsp__Policy *policy;
  policy = search(URI, content->Policy);
  if (policy)
    return policy;
  for (std::vector<wsp__Content*>::iterator i = content->All.begin(); i != content->All.end(); ++i)
  {
    policy = search(URI, *i);
    if (policy)
      return policy;
  }
  for (std::vector<wsp__Content*>::iterator j = content->ExactlyOne.begin(); j != content->ExactlyOne.end(); ++j)
  {
    policy = search(URI, *j);
    if (policy)
      return policy;
  }
  return NULL;
}
