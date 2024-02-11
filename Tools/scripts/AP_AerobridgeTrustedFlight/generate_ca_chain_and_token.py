#!/usr/bin/env python3

import os
import sys
from datetime import datetime, timedelta

import jwt
from cryptography import x509
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import rsa

from utils.constants import ROOT_CA_DIR, CA_CHAIN_FILE, TOKEN_FILE, EXPIRATION_IN_MINS, TOKEN_ISSUER
from utils.helpers import create_signed_cert, get_certificate_from_file, get_private_key_from_file


def create_ca_chain(chain_path, signing_key: rsa.RSAPrivateKeyWithSerialization, signer: x509.Certificate,
                    intermediate_ca_count: int) -> rsa.RSAPrivateKeyWithSerialization:
    ca_chain = []
    for i in range(intermediate_ca_count):
        signing_key, signer = create_signed_cert(f'intermediate{i + 1}.cname', signing_key, signer, True)
        ca_chain.append(signer)

    # leaf certificate CN should match with token issuer
    signing_key, signer = create_signed_cert(TOKEN_ISSUER, signing_key, signer, is_ca=False)

    with open(chain_path, 'wb') as f:
        # leaf certificate followed by signing certs
        f.write(signer.public_bytes(serialization.Encoding.PEM))

        for ca in ca_chain[::-1]:
            f.write(ca.public_bytes(serialization.Encoding.PEM))
        print(f'Certificate chain written to file: {f.name}')
    
    return signing_key


def create_token(key: rsa.RSAPrivateKeyWithSerialization, token_path: str) -> None:
    data = {
        'iss': TOKEN_ISSUER,
        'iat': datetime.utcnow(),
        'exp': datetime.utcnow() + timedelta(minutes=EXPIRATION_IN_MINS),  # token expiration in 1 hours 
        'key1': 'value1',
        'key2': 'value2'
    }
    encoded_token = jwt.encode(payload=data, key=key, algorithm='RS256')

    with open(token_path, 'wb') as f:
        f.write(encoded_token)
        print(f'Token written to file: {f.name}')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: generate_ca_chain_and_token.py <root_dir> <intermediate_ca_count>")
        sys.exit(1)

    root_dir = sys.argv[1]
    if not os.path.exists(root_dir):
        print(f"Directory `{root_dir}` not exists. Please provide valid directory path to find root certificate and key")
        sys.exit(1)

    ca_count = 0
    try:
        ca_count = int(sys.argv[2])
        if ca_count <= 0:
            print(f"Intermediate CA count must be a positive integer. Provided: `{ca_count}`")
            sys.exit(1)
    except ValueError:
        print(f"Intermediate CA count must be a positive integer. Provided: `{ca_count}`")
        sys.exit(1)

    # read previously generated root private key
    root_key = get_private_key_from_file(f'{root_dir}/{ROOT_CA_DIR}/private.pem')
    # read previously generated root certificate
    root_cert = get_certificate_from_file(f'{root_dir}/{ROOT_CA_DIR}/certificate.crt')

    leaf_key = create_ca_chain(f'{root_dir}/{CA_CHAIN_FILE}', root_key, root_cert, ca_count)
    create_token(leaf_key, f'{root_dir}/{TOKEN_FILE}')
