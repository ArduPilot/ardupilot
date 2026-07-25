#!/bin/sh

# Create the self-signed root CA and root's certificate: root.pem cacert.pem

echo "* This utility creates a self-signed private root CA and public certificate:"
echo "*  root.pem   - private root CA (keep this secret)"
echo "*  cacert.pem - public CA certificate (shared)"
echo "* Distribute the cacert.pem to clients to authenticate your server"
echo "* Use root.pem to sign certificates using the cert.sh utility"
echo "*"
echo "* Enter a (new) secret pass phrase when requested"
echo "* Enter it again when prompted to self-sign the CA root certificate"
echo "* You also need the pass phrase later to sign the client and server key files"
echo "* Enter your company name as the Common Name (e.g. genivia.com) when requested"
echo "* The root CA will expire after three years (1095 days)"

openssl req -newkey rsa:1024 -sha1 -keyout rootkey.pem -out rootreq.pem

openssl x509 -req -in rootreq.pem -sha1 -extfile openssl.cnf -extensions v3_ca -signkey rootkey.pem -out cacert.pem -days 1095

cat cacert.pem rootkey.pem > root.pem

openssl x509 -subject -issuer -dates -noout -in root.pem
