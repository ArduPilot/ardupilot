
SSL/TLS Client and Server Examples and Key Generation with gSOAP
================================================================

Using OpenSSL
-------------

The SSL-enabled applications are compiled with -DWITH_OPENSSL -DWITH_GZIP and
linked with -lgsoapssl++ (or -lgsoapssl) -lssl -lcrypto -lz

OpenSSL multithreaded applications require mutex locks for OpenSSL versions
prior to 1.1.0, see the CRYPTO_thread_setup() and CRYPTO_thread_cleanup() usage
in the examples and thread_setup.h and thread_setup.c/.cpp for these functions.

For OpenSSL versions 1.1.0 and greater, locking is not necessary and using
CRYPTO_thread_setup() and CRYPTO_thread_cleanup() has no effect.

The certificates of trusted certificate authorities (CA) are stored in
cacerts.pem and can be used to connect gSOAP clients to secure services. To do
so, use cacerts.pem instead of the demo cacert.pem with soap_ssl_client_context.

As a better alternative to calling soap_ssl_client_context(), trusted
certificate stores are automatically added with ssl_client_setup(), which works
for Unix/Linux and Windows platforms.  See further below.


Using GNUTLS
------------

The SSL-enabled applications are compiled with -DWITH_GNUTLS -DWITH_GZIP and
linked with stdsoap2.c[pp] -lgnutls -lgcrypt -lz

Note that stdsoap2.c[pp] MUST be compiled with -DWITH_GNUTLS to use GNUTLS,
because libgsoapssl is built with OpenSSL by default.

Note: GNUTLS does not support encrypted PEM keyfiles, so you cannot use the key
files generated with OpenSSL and use them in a GNUTLS-enabled application.

GNUTLS mutex locks are automatically enabled by the gSOAP engine stdsoap2.c[pp]
when pthreads are detected.


How to automatically load certificates from common locations on Unix/Linux
--------------------------------------------------------------------------

To automatically load certificates from common locations, you can use the
soap_ssl_client_setup() function defined in ssl_setup.h and ssl_setup.c.
To use soap_ssl_client_setup():

   #include "ssl_setup.h"
   #include "thread_setup.h"

   int main()
   {
     // OpenSSL versions prior to 1.1.0 require mutex locks
     if (CRYPTO_thread_setup())
       exit(EXIT_FAILURE);

     struct soap *soap = soap_new();

     if (soap_ssl_client_setup(soap, SOAP_SSL_DEFAULT, NULL, NULL, NULL, NULL))
     {
       soap_print_fault(soap, stderr);
       exit(EXIT_FAILURE);
     }

     ... // your code

     // OpenSSL versions prior to 1.1.0 require mutex locks
     CRYPTO_thread_cleanup();
   }

The arguments passed to soap_ssl_client_setup() are the same as
soap_ssl_client_context(), except that soap_ssl_client_setup() sets the CA path
parameter to common locations of certificates automatically.

When the gSOAP CURL plugin is used when -DWITH_CURL is defined,
soap_ssl_client_setup() uses the CURL certificate store.

Compile the source code with -DWITH_OPENSSL or -DWITH_GNUTLS and link with
OpenSSL or GNUTLS libraries.


How to automatically load certificates on Windows machines
----------------------------------------------------------

See the section above. The code is the same, but soap_ssl_client_setup() loads
certificates from the Windows system certificate store.

This also works with WinInet using the gSOAP WinInet plugin when -DWITH_WININET
is defined.


How to create self-signed certificates with OpenSSL
---------------------------------------------------

To generate a self-signed root certificate to sign client and server
certificates, first create a new private directory, say CA for "Certificate
Authority" in your home directory to store private keys and certificates.
Next, copy openssl.cnf, root.sh, and cert.sh to this directory.

Edit openssl.cnf and go to the [req_distinguished_name] section to add or
change the following items:

    [ req_distinguished_name ]
    countryName_default             = US
    stateOrProvinceName_default     = Your-State
    localityName_default            = Your-City
    0.organizationName_default      = Your-Company-Name
    emailAddress_default            = your-email@address

If you are going to use these settings often, we suggest to add the following
line to your .cshrc or .tcshrc when you're using csh or tcsh:

    setenv OPENSSL_CONF $HOME/CA/openssl.cnf

or add the folling line to .bashrc when you're using bash:

    export OPENSSL_CONF=$HOME/CA/openssl.cnf

To generate the root CA, execute:

    ./root.sh

When prompted, choose a PEM pass phrase to protect the CA's private key that
you are about to generate.  After entering your info, enter the pass phrase
again to self-sign the root CA.  You will also need the CA's PEM pass phrase
later when you sign certificates with the cert.sh script.

Now you have a new root.pem with the CA's private key.  Save the generated
root.pem keyfile and the CA's PEM pass phrase in a safe place.  Do not
distribute the root.pem!  The generated cacert.pem certificate of the CA can be
distributed and used by peers (web browsers and other client applications) to
authenticate all server certificates that are signed with it.

The root.pem and cacert.pem are valid for three years (1095 days as set in
openssl.cnf).

Next, generate the server.pem keyfile and sign it with the root CA by
executing:

    ./cert.sh server

When prompted, choose a PEM pass phrase to protect the server's private key
that you are about to generate.  The server's PEM pass phrase is used to lock
the private key of the server that is stored in the server.pem keyfile and will
therefore be needed by your server application to unlock the private key from
this file.  Enter your info and for the common name enter the server's host
name or simply localhost when testing your servers on your local machine.
Enter the root CA pass phrase when prompted to sign the server certificate.

Repeat this procedure for the client if the client must authenticate to a
server:

    ./cert.sh client

We strongly recommend to use a fresh PEM pass phrase to protect the client
private key.

The server.pem and client.pem keys are valid for one year.  Do not distribute
them, because they hold the private key.  The private key files are used
locally by the TLS/SSL application.  Only distribute the CA certificate
cacert.pem which is needed by peers to authenticate servers.

Server applications should use server.pem with soap_ssl_server_context():

    if (soap_ssl_server_context(soap,
      SOAP_SSL_DEFAULT,
      "server.pem", /* server keyfile (cert+key) */
      "XXXXXXXXXX", /* password to read the private key stored in keyfile */
      NULL,         /* no certificate to authenticate clients */
      NULL,         /* no capath to trusted certificates */
      NULL,         /* DH/RSA: use 2048 bit RSA (default with NULL) */
      NULL,         /* no random data to seed randomness */
      NULL          /* no SSL session cache */
    ))
    { 
      soap_print_fault(soap, stderr);
      exit(EXIT_FAILURE);
    }

Client applications should use cacert.pem to authenticate the server:
  
    if (soap_ssl_client_context(soap,
       SOAP_SSL_DEFAULT,
       NULL,          /* no keyfile */
       NULL,          /* no keyfile password */
       "cacert.pem",  /* self-signed certificate cacert.pem */
       NULL,          /* no capath to trusted certificates */
       NULL           /* no random data to seed randomness */
    ))
    {
       soap_print_fault(soap, stderr);
       exit(EXIT_FAILURE);
    }

To disable server authentication use SOAP_SSL_REQUIRE_SERVER_AUTHENTICATION and
use NULL for the certificate file.  Client applications may also use client.pem
as the key file with soap_ssl_client_context(), but this is only needed if the
client must authenticate to the server.  This assumes that the client and
server are tightly coupled and must mutually trust each other.

The server.pem and client.pem files actually hold both the private key and
certificate.

To print the contents of a PEM file:

    openssl x509 -text -in file.pem

To generate parameters for DH (Diffie Hellman) key exchange with OpenSSL, use:

    openssl dhparam -out dh2048.pem -2 2048

To summarize, the files you need are:

    openssl.cnf
    root.sh
    cert.sh

Files generated:

    cacert.pem      CA certificate for distribution and authentication
    root.pem        CA private key and certificate to sign client/server key files (do not distribute!)
    rootkey.pem     CA private key (do not distribute!)
    rootreq.pem     CA self-sign request
    root.srl        serial number

    server.pem      server private key and certificate (do not distribute!)
    servercert.pem  server certificate, signed by root CA, for distribution
    serverkey.pem   server private key (do not distribute!)
    serverreq.pem   sign request

    client.pem      client private key and certificate (do not distribute!)
    clientcert.pem  client certificate signed by root CA (public)
    clientkey.pem   client private key (do not distribute!)
    clientreq.pem   sign request

Files bundled with the gSOAP software:

    cacerts.pem     trusted certificates of common CAs
    cacerts.h       header file for cacerts.c: declares soap_ssl_client_cacerts()
    cacerts.c       trusted certificates of common CAs hardcoded, no cacerts.pem required


How to convert certificates to CER format for MS Windows
--------------------------------------------------------

To convert certificate cacert.pem to CER format:

    openssl x509 -in cacert.pem -outform der -out cacert.cer

Install the certificate on MS Windows by opening it and then select "Install
Certificate". Client applications running on MS Windows can now connect to the
server. The server authenticates to the client by means of the certificate.


How to create self-signed certificates with GNUTLS
--------------------------------------------------

We use the GNUTLS 'certtool' command (or the 'gnutls-certtool' command) to
create keys and certificates as follows.

First we generate a private key (for a client or server):

    certtool --generate-privkey --outfile privkey.pem

Make sure to use GNUTLS 'certtool', sometimes renamed to 'gnutls-certtool' to
avoid confusion with the system 'certtool' command.

Then we self-sign the certificate:

    certtool --generate-self-signed --load-privkey privkey.pem --outfile cert.pem

When prompted, the following options are recommended for the certificate:

* The certificate MUST NOT be marked as a CA certificate.
* The DNS name MUST match the FQDN that clients will use to access the server.
  Use the server domain name here. One name suffices.
* The certificate MUST be marked as usable for encryption.
* The certificate MUST be marked as usable for a TLS server (or client when
  appropriate).

The client.pem and server.pem keyfiles used in the soap_ssl_client_context()
and soap_ssl_server_context() API functions is a combination of the private key
and the certificate:

    cat privkey.pem cert.pem > server.pem

The client can use the cert.pem to authenticate the server. The private key
file and server.pem are for the server only and SHOULD NEVER be shared.

Note that the server.pem file generated above is NOT encrypted with a password,
so the password parameter of soap_ssl_server_context() is not used.  Neither is
the capath parameter used for the fact that GNUTLS does not search for loadable
certificates.

The PEM files produced by GNUTLS can be used with OpenSSL.

The PEM key files created with OpenSSL (such as server.pem and client.pem)
CANNOT be used with GNUTLS, because they contain encrypted private keys that
GNUTLS cannot read ("SSL/TLS error: Can't read key file").

We can also use GNUTLS 'certtool' to create a Certificate Authority (CA) to
sign client and server certificates as follows.

    certtool --generate-privkey --outfile cakey.pem

Then we self-sign the CA certificate:

    certtool --generate-self-signed --load-privkey cakey.pem --outfile cacert.pem

When prompted, the following options are recommended for the CA certificate:

* The CA certificate SHOULD be marked to belong to an authority.
* The CA certificate MUST be marked as a CA certificate.
* The CA certificate MUST be usable for signing other certificates.
* The CA certificate MUST be usable for signing Certificate Revocation Lists
  (CRLs).

Now we can create a server key and use the CA to sign the server's certificate:

    certtool --generate-privkey --outfile serverkey.pem
    certtool --generate-request --load-privkey serverkey.pem --outfile server.csr
    certtool --generate-certificate --load-request server.csr --load-ca-certificate cacert.pem --load-ca-privkey cakey.pem --outfile servercert.pem

Use the recommended options shown earlier for creating the certificate.

The client.pem and server.pem keyfiles used in the soap_ssl_client_context()
and soap_ssl_server_context() API functions is a combination of the private key
and the certificate:

    cat serverkey.pem servercert.pem > server.pem

The procedure above can be repeated to create a key and signed certificate for
clients and other servers. All clients and servers can be authenticated with
the CA certificate. The cacert.pem is to be used by all peers that require the
other party to authenticate (e.g. the client uses cacert.pem CA cert to
authenticate the server, who uses the server.pem keyfile).

To generate parameters for DH (Diffie Hellman) key exchange with GNUTLS, use:

    certtool --generate-dh-params --bits 2048 --outfile dh2048.pem

