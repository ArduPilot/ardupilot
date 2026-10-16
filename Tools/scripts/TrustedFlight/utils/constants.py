EXPIRATION_IN_MINS = 15

JWT_TYPE = 'JWT'
# private alg value: monocypher's curve25519 + Blake2b, not standard RFC 8037 EdDSA (SHA-512)
JWT_ALG = 'EdDSA-Blake2b'
PUBLIC_KEY_FILE = 'key.pub'
TOKEN_FILE = 'token'
TOKEN_ISSUER = 'test.cname'
