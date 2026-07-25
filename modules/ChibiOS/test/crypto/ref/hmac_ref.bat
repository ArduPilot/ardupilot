

call %OPENSSL%\openssl dgst -sha256 -hmac "" -c -binary -out hmac_hmac256_1 plaintext_6
ren  hmac_hmac256_1 hmac_hmac256_1.enc

call %OPENSSL%\openssl dgst -sha256 -hmac "Jefe" -c -binary -out hmac_hmac256_2 plaintext_7
ren  hmac_hmac256_2 hmac_hmac256_2.enc

call %OPENSSL%\openssl dgst -sha512 -hmac "" -c -binary -out hmac_hmac512_1 plaintext_6
ren  hmac_hmac512_1 hmac_hmac512_1.enc

call %OPENSSL%\openssl dgst -sha512 -hmac "Jefe" -c -binary -out hmac_hmac512_2 plaintext_7
ren  hmac_hmac512_2 hmac_hmac512_2.enc
