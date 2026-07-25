# We really don't want -Wno-address-of-packed-member here, as it's hiding potential 
# errors in the generator, but without it, we can't run the tests to completion. 
# particular issues are with pointers to arrays right now. TODO - fix the generator and remove this.
CFLAGS = -g -Wall -Werror -O0 -Wno-address-of-packed-member -Wno-unused-function
TESTPROTOCOL = ardupilotmega
ALLPROTOCOLS = minimal test common pixhawk ardupilotmega slugs ualberta

all:
	for p in ${ALLPROTOCOLS}; do make -f Makefile build TESTPROTOCOL=$$p; done

test:
	for p in ${ALLPROTOCOLS}; do make -f Makefile testprogs TESTPROTOCOL=$$p || exit 1; done

valgrindtest:
	for p in ${ALLPROTOCOLS}; do make -f Makefile valgrindprogs TESTPROTOCOL=$$p || exit 1; done

build: testmav2.0_${TESTPROTOCOL} testmav1.0_${TESTPROTOCOL} test_issues

testprogs: testmav2.0_${TESTPROTOCOL} testmav1.0_${TESTPROTOCOL}
	./testmav2.0_${TESTPROTOCOL}
	./testmav1.0_${TESTPROTOCOL}

valgrindprogs: testmav2.0_${TESTPROTOCOL} testmav1.0_${TESTPROTOCOL}
	valgrind -q ./testmav2.0_${TESTPROTOCOL}
	valgrind -q ./testmav1.0_${TESTPROTOCOL}

clean:
	rm -rf *.o *~ testmav1.0* testmav2.0* sha256_test test_issues

testmav1.0_${TESTPROTOCOL}: testmav.c $(COMMON)
	$(CC) $(CFLAGS) -I../../include_v1.0 -I../../include_v1.0/${TESTPROTOCOL} -o $@ testmav.c

testmav2.0_${TESTPROTOCOL}: testmav.c
	$(CC) $(CFLAGS) -I../../include_v2.0 -I../../include_v2.0/${TESTPROTOCOL} -o $@ testmav.c

testmav1.0_ardupilotmega: testmav.c
	$(CC) $(CFLAGS) -I../../include_v1.0 -I../../include_v1.0/ardupilotmega -o $@ testmav.c

testmav2.0_ardupilotmega: testmav.c
	$(CC) $(CFLAGS) -I../../include_v2.0 -I../../include_v2.0/ardupilotmega -o $@ testmav.c

testmav1.0_common: testmav.c
	$(CC) $(CFLAGS) -I../../include_v1.0 -I../../include_v1.0/common -o $@ testmav.c

testmav2.0_common: testmav.c
	$(CC) $(CFLAGS) -I../../include_v2.0 -I../../include_v2.0/common -o $@ testmav.c

testmav2.0_minimal: testmav.c
	$(CC) $(CFLAGS) -I../../include_v2.0 -I../../include_v2.0/minimal -o $@ testmav.c

sha256_test: sha256_test.c
	$(CC) $(CFLAGS) -I../../include_v2.0 -o $@ sha256_test.c

parse_tlog: parse_tlog.c
	$(CC) $(CFLAGS) -I../../include_v2.0 -I../../include_v2.0/ardupilotmega -o $@ parse_tlog.c

test_issues: test_issues.c
	$(CC) $(CFLAGS) -I../../include_v2.0 -I../../include_v2.0/all -o $@ test_issues.c
