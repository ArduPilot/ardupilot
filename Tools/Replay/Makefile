# this is meant to make existing build instructions work with waf

all:
	@cd ../../ && modules/waf/waf-light configure --board linux --debug
	@cd ../../ && modules/waf/waf-light --target tools/Replay
	@cp ../../build/linux/tools/Replay Replay.elf
	@echo Built Replay.elf

clean:
	@cd ../../ && modules/waf/waf-light configure --board linux clean

linux-debug: all
