
call %OPENSSL%\openssl enc -des-ecb -nopad -nosalt -K %K8% -e -in plaintext -out des_ecb_8.enc

REM TDES
call %OPENSSL%\openssl enc -des-ede-ecb -nopad -nosalt -K %K16% -e -in plaintext -out tdes_ecb_16.enc
call %OPENSSL%\openssl enc -des-ede3-ecb -nopad -nosalt -K %K24% -e -in plaintext -out tdes_ecb_24.enc
call %OPENSSL%\openssl enc -des-ede-cbc -nopad -nosalt -K %K16% -iv %DES_IV% -e -in plaintext -out tdes_cbc_16.enc
call %OPENSSL%\openssl enc -des-ede3-cbc -nopad -nosalt -K %K24% -iv %DES_IV% -e -in plaintext -out tdes_cbc_24.enc 