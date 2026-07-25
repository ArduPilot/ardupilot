%PYTHON%\python genfile.py -f aes_ecb_128,aes_ecb_192,aes_ecb_256,aes_cbc_128,aes_cbc_192,aes_cbc_256,aes_ctr_128,aes_ctr_192,aes_ctr_256,aes_cfb_128,aes_cfb_192,aes_cfb_256 -o ref_aes -p ../source/testref

%PYTHON%\python genfile.py -f des_ecb_8,tdes_ecb_16,tdes_ecb_24,tdes_cbc_16,tdes_cbc_24 -o ref_des -p ../source/testref

%PYTHON%\python genfile.py -f sha_sha1_empty,sha_sha1_3,sha_sha1_56,sha_sha1_64,sha_sha1_128,sha_sha256_3,sha_sha256_56,sha_sha256_64,sha_sha256_128,sha_sha512_3,sha_sha512_56,sha_sha512_64,sha_sha512_128 -o ref_sha -p ../source/testref


%PYTHON%\python genfile.py -f hmac_hmac256_1,hmac_hmac256_2,hmac_hmac512_1,hmac_hmac512_2 -o ref_hmac -p ../source/testref