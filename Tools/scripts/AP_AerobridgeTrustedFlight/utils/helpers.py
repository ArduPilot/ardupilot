#!/usr/bin/env python3
'''
helper methods for Aerobridge Trusted Flight token/ certificate generation scrips
'''

from datetime import datetime, timedelta

from cryptography import x509
from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.hazmat.primitives.asymmetric.padding import PKCS1v15
from cryptography.x509.oid import NameOID

from .constants import EXPIRATION_IN_MINS
from .exceptions import ExpiredCertificateException

def create_csr(cname: str, is_ca: bool) -> [rsa.RSAPrivateKeyWithSerialization, x509.CertificateSigningRequest]:
    key = rsa.generate_private_key(public_exponent=65537, key_size=4096, backend=default_backend())

    csr = (x509.CertificateSigningRequestBuilder()
           .subject_name(x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, cname)]))
           .add_extension(x509.BasicConstraints(ca=is_ca, path_length=None), critical=True)
           .sign(key, hashes.SHA256(), backend=default_backend()))

    return key, csr


def sign_csr(csr: x509.CertificateSigningRequest, signing_key: rsa.RSAPrivateKeyWithSerialization,
             signer: x509.Certificate = None) -> x509.Certificate:
    # if signer is None, it's a self signed cert
    issuer = csr.subject if signer is None else signer.subject
    cert = (x509.CertificateBuilder()
            .subject_name(csr.subject)
            .issuer_name(issuer)
            .public_key(csr.public_key())
            .serial_number(x509.random_serial_number())
            .not_valid_before(datetime.utcnow())
            .not_valid_after(datetime.utcnow() + timedelta(minutes=EXPIRATION_IN_MINS))  # Certificate will be valid for 1 hours
            )

    for ext in csr.extensions:
        cert = cert.add_extension(ext.value, critical=ext.critical)

    return cert.sign(signing_key, hashes.SHA256(), backend=default_backend())


def create_signed_cert(cname: str, signing_key: rsa.RSAPrivateKeyWithSerialization, signer: x509.Certificate,
                       is_ca: bool) -> [rsa.RSAPrivateKeyWithSerialization, x509.Certificate]:
    key, csr = create_csr(cname=cname, is_ca=is_ca)
    cert = sign_csr(csr=csr, signing_key=signing_key, signer=signer)
    print(f'Signed certificate. CN: {cert.subject}')

    return key, cert


def get_private_key_from_file(filepath):
    data = open(filepath, 'rb').read()
    return serialization.load_pem_private_key(data=data, password=None, backend=default_backend())


def get_ca_chain_from_file(filepath):
    start = b'-----BEGIN CERTIFICATE-----'
    ca_chain = []
    data = open(filepath, 'rb').read()
    temp = data.split(start)
    for cert in temp[1:]:
        ca_chain.append(x509.load_pem_x509_certificate(data=start+cert, backend=default_backend()))
    return ca_chain


def get_certificate_from_file(filepath):
    # return first certificate
    return get_ca_chain_from_file(filepath)[0]


def validate_certificate(trusted_cert, cert_to_verify):
    public_key = trusted_cert.public_key()
    public_key.verify(
        cert_to_verify.signature,
        cert_to_verify.tbs_certificate_bytes,
        PKCS1v15(),
        cert_to_verify.signature_hash_algorithm
    )
    if cert_to_verify.not_valid_after <= datetime.utcnow():
        raise ExpiredCertificateException(f'Certificate is expired. CN: {cert_to_verify.subject}')
