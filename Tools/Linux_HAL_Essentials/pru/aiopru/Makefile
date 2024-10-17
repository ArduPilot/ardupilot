RcAioPRU: RcAioPRU.p
	pasm -V3 -c RcAioPRU.p RcAioPRU_BBBMINI -DBBBMINI
	pasm -V3 -c RcAioPRU.p RcAioPRU_BBBLUE -DBBBLUE
	pasm -V3 -c RcAioPRU.p RcAioPRU_POCKET -DPOCKET
	pasm -V3 -c RcAioPRU_DShot.p RcAioPRU_DShot_BBBMINI -DBBBMINI -CPRUcode_DShot
	pasm -V3 -c RcAioPRU_DShot.p RcAioPRU_DShot_BBBLUE -DBBBLUE -CPRUcode_DShot
	pasm -V3 -c RcAioPRU_DShot.p RcAioPRU_DShot_POCKET -DPOCKET -CPRUcode_DShot

debug: RcAioPRU.p
	pasm -V3 -c RcAioPRU.p RcAioPRU_BBBMINI -DBBBMINI -DDEBUG
	pasm -V3 -c RcAioPRU.p RcAioPRU_BBBLUE -DBBBLUE -DDEBUG
	pasm -V3 -c RcAioPRU.p RcAioPRU_POCKET -DPOCKET -DDEBUG
	pasm -V3 -c RcAioPRU_DShot.p RcAioPRU_DShot_BBBMINI -DBBBMINI -DDEBUG -CPRUcode_DShot
	pasm -V3 -c RcAioPRU_DShot.p RcAioPRU_DShot_BBBLUE -DBBBLUE -DDEBUG -CPRUcode_DShot
	pasm -V3 -c RcAioPRU_DShot.p RcAioPRU_DShot_POCKET -DPOCKET -DDEBUG -CPRUcode_DShot

test: RcAioPRUTest.c
	gcc -g -o RcAioPRUTest RcAioPRUTest.c

clean:
	rm RcAioPRU_BBBMINI_bin.h RcAioPRU_BBBLUE_bin.h RcAioPRU_POCKET_bin.h RcAioPRUTest RcAioPRU_DShot_BBBMINI_bin.h RcAioPRU_DShot_BBBLUE_bin.h RcAioPRU_DShot_POCKET_bin.h
