# this is meant to make existing build instructions work with waf

all:
	@cd ../../ && modules/waf/waf-light configure --board sitl --debug --disable-scripting
	@cd ../../ && modules/waf/waf-light --target tools/Replay
	@cp ../../build/sitl/tools/Replay Replay.elf
	@echo Built Replay.elf

clean:
	@cd ../../ && modules/waf/waf-light configure --board sitl clean

linux-debug: all
