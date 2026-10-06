RcAioPRU: RcAioPRU.p
	pasm -V3 -c RcAioPRU.p RcAioPRU_BBBMINI -DBBBMINI
	pasm -V3 -c RcAioPRU.p RcAioPRU_BBBLUE -DBBBLUE
	pasm -V3 -c RcAioPRU.p RcAioPRU_POCKET -DPOCKET
	pasm -V3 -c RcAioPRUPB2.p RcAioPRU_POCKET2
	pasm -V3 -c RcAioPRU_PB2_DShot.p RcAioPRU_POCKET2_DShot -CPRUcode_DShot

debug: RcAioPRU.p
	pasm -V3 -c RcAioPRU.p RcAioPRU_BBBMINI -DBBBMINI -DDEBUG
	pasm -V3 -c RcAioPRU.p RcAioPRU_BBBLUE -DBBBLUE -DDEBUG
	pasm -V3 -c RcAioPRU.p RcAioPRU_POCKET -DPOCKET -DDEBUG
	pasm -V3 -c RcAioPRUPB2.p RcAioPRU_POCKET2 -DDEBUG
	pasm -V3 -c RcAioPRU_PB2_DShot.p RcAioPRU_POCKET2_DShot -DDEBUG -CPRUcode_DShot

test: RcAioPRUTest.c
	gcc -g -o RcAioPRUTest RcAioPRUTest.c

clean:
	rm RcAioPRU_BBBMINI_bin.h RcAioPRU_BBBLUE_bin.h RcAioPRU_POCKET_bin.h RcAioPRU_POCKET2_bin.h RcAioPRU_POCKET2_DShot_bin.h RcAioPRUTest
