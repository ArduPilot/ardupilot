

call %PYTHON%\python -c "print 'DA39A3EE5E6B4B0D3255BFEF95601890AFD80709'.decode('hex')" >  sha_sha1_empty.enc

call %OPENSSL%\openssl dgst -sha1 -c -binary -out sha_sha1_3 plaintext_2
ren  sha_sha1_3 sha_sha1_3.enc

call %OPENSSL%\openssl dgst -sha1 -c -binary -out sha_sha1_56 plaintext_3
ren  sha_sha1_56 sha_sha1_56.enc

call %OPENSSL%\openssl dgst -sha1 -c -binary -out sha_sha1_64 plaintext_4
ren  sha_sha1_64 sha_sha1_64.enc

call %OPENSSL%\openssl dgst -sha1 -c -binary -out sha_sha1_128 plaintext_5
ren  sha_sha1_128 sha_sha1_128.enc


call %OPENSSL%\openssl dgst -sha256 -c -binary -out sha_sha256_3 plaintext_2
ren  sha_sha256_3 sha_sha256_3.enc

call %OPENSSL%\openssl dgst -sha256 -c -binary -out sha_sha256_56 plaintext_3
ren  sha_sha256_56 sha_sha256_56.enc

call %OPENSSL%\openssl dgst -sha256 -c -binary -out sha_sha256_64 plaintext_4
ren  sha_sha256_64 sha_sha256_64.enc

call %OPENSSL%\openssl dgst -sha256 -c -binary -out sha_sha256_128 plaintext_5
ren  sha_sha256_128 sha_sha256_128.enc


call %OPENSSL%\openssl dgst -sha512 -c -binary -out sha_sha512_3 plaintext_2
ren  sha_sha512_3 sha_sha512_3.enc


call %OPENSSL%\openssl dgst -sha512 -c -binary -out sha_sha512_56 plaintext_3
ren  sha_sha512_56 sha_sha512_56.enc

call %OPENSSL%\openssl dgst -sha512 -c -binary -out sha_sha512_64 plaintext_4
ren  sha_sha512_64 sha_sha512_64.enc

call %OPENSSL%\openssl dgst -sha512 -c -binary -out sha_sha512_128 plaintext_5
ren  sha_sha512_128 sha_sha512_128.enc


