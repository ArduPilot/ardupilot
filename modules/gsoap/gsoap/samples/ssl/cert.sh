#!/bin/sh

if [ "$#" = 1 ]
then

  echo "* This utility create certificate ${1}.pem signed by the root CA:"
  echo "*  ${1}cert.pem - public key (CA-signed certificate to be shared)"
  echo "*  ${1}key.pem  - private key (keep this secret)"
  echo "*  ${1}.pem     - private key bundled with certificates (keep secret)"
  echo "* Distribute the CA root cacert.pem (and/or ${1}cert.pem when needed)"
  echo "* Before using this utility, create a root CA root.pem using root.sh"
  echo "* Keep ${1}.pem key file secret: store locally with client/server app"
  echo "*"
  echo "* Enter a secret pass phrase when requested"
  echo "* The pass phrase is used to access ${1}.pem in your application"
  echo "* Enter the server's host name as the Common Name when requested"
  echo "* Enter the root CA pass phrase (Getting CA Private Key) to sign the key file"
  echo "* The key file will expire after three years or when the root CA expires"

  # Create a certificate and signing request

  openssl req -newkey rsa:1024 -sha1 -keyout ${1}key.pem -out ${1}req.pem

  # Sign the certificate with the root CA

  openssl x509 -req -in ${1}req.pem -sha1 -extfile openssl.cnf -extensions usr_cert -CA root.pem -CAkey root.pem -CAcreateserial -out ${1}cert.pem -days 1095

  # Bundle the CA certificate cacert with the certificate file

  cat ${1}cert.pem cacert.pem > ${1}tmp.pem
  mv -f ${1}tmp.pem ${1}cert.pem

  # Bundle certificates with the private key file

  cat ${1}key.pem ${1}cert.pem > ${1}.pem

  # Show what we got

  openssl x509 -subject -issuer -dates -noout -in ${1}.pem

else

  echo "Usage: cert.sh <certname>"
  exit 1

fi
