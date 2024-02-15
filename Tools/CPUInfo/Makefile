# this is meant to make existing build instructions work with waf

all:
	@cd ../../ && modules/waf/waf-light configure --board linux --debug
	@cd ../../ && modules/waf/waf-light tool
	@cp ../../build/linux/tool/CPUInfo CPUInfo.elf
	@echo Built CPUInfo.elf

clean:
	@cd ../../ && modules/waf/waf-light configure --board linux clean

linux-debug: all
