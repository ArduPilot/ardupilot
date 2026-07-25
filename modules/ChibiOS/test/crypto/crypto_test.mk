# List of all the ChibiOS OS Library test files.
TESTSRC += 	 ${CHIBIOS}/test/crypto/source/test/cry_test_root.c 			\
			 ${CHIBIOS}/test/crypto/source/testref/ref_aes.c 				\
			 ${CHIBIOS}/test/crypto/source/testref/ref_des.c 				\
			 ${CHIBIOS}/test/crypto/source/testref/ref_sha.c 				\
			 ${CHIBIOS}/test/crypto/source/testref/ref_gcm.c 				\
			 ${CHIBIOS}/test/crypto/source/testref/ref_hmac.c 				\
           	 ${CHIBIOS}/test/crypto/source/test/cry_test_sequence_001.c		\
           	 ${CHIBIOS}/test/crypto/source/test/cry_test_sequence_002.c		\
           	 ${CHIBIOS}/test/crypto/source/test/cry_test_sequence_003.c		\
           	 ${CHIBIOS}/test/crypto/source/test/cry_test_sequence_004.c		\
			 ${CHIBIOS}/test/crypto/source/test/cry_test_sequence_005.c		\
			 ${CHIBIOS}/test/crypto/source/test/cry_test_sequence_006.c		\
			 ${CHIBIOS}/test/crypto/source/test/cry_test_sequence_007.c		\
			 ${CHIBIOS}/test/crypto/source/test/cry_test_sequence_008.c		\
			 ${CHIBIOS}/test/crypto/source/test/cry_test_sequence_009.c
# Required include directories
TESTINC +=  ${CHIBIOS}/test/crypto/source/testref	\
			${CHIBIOS}/test/crypto/source/test

include $(CHIBIOS)/os/hal/lib/streams/streams.mk


