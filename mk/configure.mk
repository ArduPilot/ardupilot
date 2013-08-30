configure:
	$(warning WARNING - A $(SKETCHBOOK)/config.mk file has been written)
	$(warning Please edit the file to match your system configuration, if you use a different board or port)
	@echo  > $(SKETCHBOOK)/config.mk
	@echo \# Select \'mega\' for the 1280 APM1, \'mega2560\' otherwise >> $(SKETCHBOOK)/config.mk
	@echo BOARD = mega2560     >> $(SKETCHBOOK)/config.mk
	@echo  >> $(SKETCHBOOK)/config.mk
	@echo \# HAL_BOARD determines default HAL target. >> $(SKETCHBOOK)/config.mk
	@echo HAL_BOARD ?= HAL_BOARD_APM2 >> $(SKETCHBOOK)/config.mk
	@echo  >> $(SKETCHBOOK)/config.mk
	@echo \# The communication port used to communicate with the APM. >> $(SKETCHBOOK)/config.mk
ifneq ($(findstring CYGWIN, $(SYSTYPE)),)
	@echo PORT = COM3 >> $(SKETCHBOOK)/config.mk
else
	@echo PORT = /dev/ttyACM0 >> $(SKETCHBOOK)/config.mk
endif
	@echo  >> $(SKETCHBOOK)/config.mk
	@echo  \# uncomment and fill in the path to Arduino if installed in an exotic location >> $(SKETCHBOOK)/config.mk
	@echo  \# ARDUINO = /path/to/Arduino  >> $(SKETCHBOOK)/config.mk
	@echo  >> $(SKETCHBOOK)/config.mk
	@echo \# PX4Firmware tree: fill in the path to PX4Firmware repository from github.com/diydrones: >> $(SKETCHBOOK)/config.mk
	@echo PX4_ROOT=../PX4Firmware >> $(SKETCHBOOK)/config.mk
	@echo  >> $(SKETCHBOOK)/config.mk
	@echo \# PX4NuttX tree: fill in the path to PX4NuttX repository from github.com/diydrones: >> $(SKETCHBOOK)/config.mk
	@echo NUTTX_SRC=../PX4NuttX/nuttx >> $(SKETCHBOOK)/config.mk
