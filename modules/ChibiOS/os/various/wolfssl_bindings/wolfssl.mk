# List of the required lwIP files.
WOLFSSL = 	$(CHIBIOS)/ext/wolfssl

WOLFBINDSRC = \
        $(CHIBIOS)/os/various/wolfssl_bindings/wolfssl_chibios.c \
        $(CHIBIOS)/os/various/wolfssl_bindings/hwrng.c

WOLFCRYPTSRC = \
	$(WOLFSSL)/wolfcrypt/src/sha.c \
	$(WOLFSSL)/wolfcrypt/src/ge_low_mem.c \
	$(WOLFSSL)/wolfcrypt/src/compress.c \
	$(WOLFSSL)/wolfcrypt/src/chacha20_poly1305.c \
	$(WOLFSSL)/wolfcrypt/src/des3.c \
	$(WOLFSSL)/wolfcrypt/src/fe_low_mem.c \
	$(WOLFSSL)/wolfcrypt/src/hmac.c \
	$(WOLFSSL)/wolfcrypt/src/asm.c \
	$(WOLFSSL)/wolfcrypt/src/camellia.c \
	$(WOLFSSL)/wolfcrypt/src/ecc.c \
	$(WOLFSSL)/wolfcrypt/src/ecc_fp.c \
	$(WOLFSSL)/wolfcrypt/src/ripemd.c \
	$(WOLFSSL)/wolfcrypt/src/rsa.c \
	$(WOLFSSL)/wolfcrypt/src/wc_port.c \
	$(WOLFSSL)/wolfcrypt/src/arc4.c \
	$(WOLFSSL)/wolfcrypt/src/srp.c \
	$(WOLFSSL)/wolfcrypt/src/random.c \
	$(WOLFSSL)/wolfcrypt/src/idea.c \
	$(WOLFSSL)/wolfcrypt/src/blake2b.c \
	$(WOLFSSL)/wolfcrypt/src/error.c \
	$(WOLFSSL)/wolfcrypt/src/dh.c \
	$(WOLFSSL)/wolfcrypt/src/asn.c \
	$(WOLFSSL)/wolfcrypt/src/cmac.c \
	$(WOLFSSL)/wolfcrypt/src/signature.c \
	$(WOLFSSL)/wolfcrypt/src/pwdbased.c \
	$(WOLFSSL)/wolfcrypt/src/chacha.c \
	$(WOLFSSL)/wolfcrypt/src/md5.c \
	$(WOLFSSL)/wolfcrypt/src/aes.c \
	$(WOLFSSL)/wolfcrypt/src/wolfmath.c \
	$(WOLFSSL)/wolfcrypt/src/memory.c \
	$(WOLFSSL)/wolfcrypt/src/logging.c \
	$(WOLFSSL)/wolfcrypt/src/tfm.c \
	$(WOLFSSL)/wolfcrypt/src/coding.c \
	$(WOLFSSL)/wolfcrypt/src/rabbit.c \
	$(WOLFSSL)/wolfcrypt/src/pkcs12.c \
	$(WOLFSSL)/wolfcrypt/src/md2.c \
	$(WOLFSSL)/wolfcrypt/src/ge_operations.c \
	$(WOLFSSL)/wolfcrypt/src/sha512.c \
	$(WOLFSSL)/wolfcrypt/src/sha3.c \
	$(WOLFSSL)/wolfcrypt/src/port/nrf51.c \
	$(WOLFSSL)/wolfcrypt/src/port/pic32/pic32mz-crypt.c \
	$(WOLFSSL)/wolfcrypt/src/port/atmel/atmel.c \
	$(WOLFSSL)/wolfcrypt/src/port/nxp/ksdk_port.c \
	$(WOLFSSL)/wolfcrypt/src/port/ti/ti-des3.c \
	$(WOLFSSL)/wolfcrypt/src/port/ti/ti-ccm.c \
	$(WOLFSSL)/wolfcrypt/src/port/ti/ti-hash.c \
	$(WOLFSSL)/wolfcrypt/src/port/ti/ti-aes.c \
	$(WOLFSSL)/wolfcrypt/src/port/arm/armv8-aes.c \
	$(WOLFSSL)/wolfcrypt/src/port/arm/armv8-sha256.c \
	$(WOLFSSL)/wolfcrypt/src/port/xilinx/xil-aesgcm.c \
	$(WOLFSSL)/wolfcrypt/src/port/xilinx/xil-sha3.c \
	$(WOLFSSL)/wolfcrypt/src/hash.c \
	$(WOLFSSL)/wolfcrypt/src/curve25519.c \
	$(WOLFSSL)/wolfcrypt/src/integer.c \
	$(WOLFSSL)/wolfcrypt/src/wolfevent.c \
	$(WOLFSSL)/wolfcrypt/src/dsa.c \
	$(WOLFSSL)/wolfcrypt/src/pkcs7.c \
	$(WOLFSSL)/wolfcrypt/src/wc_encrypt.c \
	$(WOLFSSL)/wolfcrypt/src/cpuid.c \
	$(WOLFSSL)/wolfcrypt/src/sha256.c \
	$(WOLFSSL)/wolfcrypt/src/md4.c \
	$(WOLFSSL)/wolfcrypt/src/fe_operations.c \
	$(WOLFSSL)/wolfcrypt/src/ed25519.c \
	$(WOLFSSL)/wolfcrypt/src/poly1305.c \
	$(WOLFSSL)/wolfcrypt/src/hc128.c \

WOLFSSLSRC = \
	$(WOLFSSL)/src/internal.c \
	$(WOLFSSL)/src/tls.c \
	$(WOLFSSL)/src/keys.c \
	$(WOLFSSL)/src/crl.c \
	$(WOLFSSL)/src/ssl.c \
	$(WOLFSSL)/src/wolfio.c \
	$(WOLFSSL)/src/sniffer.c \
	$(WOLFSSL)/src/ocsp.c \
	$(WOLFSSL)/src/tls13.c


WOLFSRC = $(WOLFBINDSRC) $(WOLFCRYPTSRC) $(WOLFSSLSRC)

WOLFINC = \
        $(CHIBIOS)/os/various/wolfssl_bindings \
        $(WOLFSSL)/wolfcrypt/include \
        $(WOLFSSL)/wolfssl/include \
		$(WOLFSSL)

# Shared variables
ALLCSRC += $(WOLFSRC)
ALLINC  += $(WOLFINC)

