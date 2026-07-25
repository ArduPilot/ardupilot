

%OPENSSL%\openssl enc -aes-128-ecb -nosalt -nopad -K %K16% -in plaintext -out aes_ecb_128.enc
%OPENSSL%\openssl enc -aes-192-ecb -nopad -nosalt -K %K24% -in plaintext -out aes_ecb_192.enc
%OPENSSL%\openssl enc -aes-256-ecb -nopad -nosalt -K %K32% -in plaintext -out aes_ecb_256.enc

%OPENSSL%\openssl enc -aes-128-cbc -nopad -nosalt -K %K16% -iv %IV% -in plaintext -out aes_cbc_128.enc
%OPENSSL%\openssl enc -aes-192-cbc -nopad -nosalt -K %K24% -iv %IV% -in plaintext -out aes_cbc_192.enc
%OPENSSL%\openssl enc -aes-256-cbc -nopad -nosalt -K %K32% -iv %IV% -in plaintext -out aes_cbc_256.enc

%OPENSSL%\openssl enc -aes-128-ctr -nopad -nosalt -K %K16% -iv %IV% -in plaintext -out aes_ctr_128.enc
%OPENSSL%\openssl enc -aes-192-ctr -nopad -nosalt -K %K24% -iv %IV% -in plaintext -out aes_ctr_192.enc
%OPENSSL%\openssl enc -aes-256-ctr -nopad -nosalt -K %K32% -iv %IV% -in plaintext -out aes_ctr_256.enc

%OPENSSL%\openssl enc -aes-128-cfb -nopad -nosalt -K %K16% -iv %IV% -in plaintext -out aes_cfb_128.enc
%OPENSSL%\openssl enc -aes-192-cfb -nopad -nosalt -K %K24% -iv %IV% -in plaintext -out aes_cfb_192.enc
%OPENSSL%\openssl enc -aes-256-cfb -nopad -nosalt -K %K32% -iv %IV% -in plaintext -out aes_cfb_256.enc
