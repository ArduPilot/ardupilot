RcAioPRU: RcAioPRU.p
	pasm -V3 -c RcAioPRU.p RcAioPRU_BBBMINI -DBBBMINI
	pasm -V3 -c RcAioPRU.p RcAioPRU_BBBLUE -DBBBLUE

test: RcAioPRUTest.c
	gcc -g -o RcAioPRUTest RcAioPRUTest.c

clean:
	rm RcAioPRU_bin.h RcAioPRUTest
