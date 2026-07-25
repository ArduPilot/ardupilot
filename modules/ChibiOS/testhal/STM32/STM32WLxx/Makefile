##############################################################################
# STM32WLxx Test HAL multi-project makefile rules
#

all:
	@echo
	@echo === Test build for STM32WLxx ADC =====================================
	+@make --no-print-directory --directory ../multi/ADC -f ./make/stm32wl55jc_nucleo64.make all
	@echo ====================================================================
	@echo
	@echo === Test build for STM32WLxx DAC =====================================
	+@make --no-print-directory --directory ../multi/DAC -f ./make/stm32wl55jc_nucleo64.make all
	@echo ====================================================================
	@echo
	@echo === Test build for STM32WLxx EFL-MFS ===============================
	+@make --no-print-directory --directory ../multi/EFL-MFS -f ./make/stm32wl55jc_nucleo64.make all
	@echo ====================================================================
	@echo
	@echo === Test build for STM32WLxx I2S =====================================
	+@make --no-print-directory --directory ./I2S -f ./Makefile all
	@echo ====================================================================
	@echo
	@echo === Test build for STM32WLxx PAL =====================================
	+@make --no-print-directory --directory ../multi/PAL -f ./make/stm32wl55jc_nucleo64.make all
	@echo ====================================================================
	@echo
	@echo === Test build for STM32WLxx SPI =====================================
	+@make --no-print-directory --directory ./SPI -f ./Makefile all
	@echo ====================================================================
	@echo
	@echo === Test build for STM32WLxx SYSTICKv2 ===============================
	+@make --no-print-directory --directory ./SYSTICKv2 -f ./Makefile all
	@echo ====================================================================
	@echo
	@echo === Test build for STM32WLxx TRNG ====================================
	+@make --no-print-directory --directory ../multi/TRNG -f ./make/stm32wl55jc_nucleo64.make all
	@echo ====================================================================
	@echo
	@echo === Test build for STM32WLxx UART ====================================
	+@make --no-print-directory --directory ../multi/UART -f ./make/stm32wl55jc_nucleo64.make all
	@echo ====================================================================
	@echo
	@echo === Test build for STM32WLxx WDG =====================================
	+@make --no-print-directory --directory ./WDG -f ./Makefile all
	@echo ====================================================================
	@echo

clean:
	@echo
	+@make --no-print-directory --directory ../multi/ADC -f ./make/stm32wl55jc_nucleo64.make clean
	@echo
	+@make --no-print-directory --directory ../multi/DAC -f ./make/stm32wl55jc_nucleo64.make clean
	@echo
	+@make --no-print-directory --directory ../multi/EFL-MFS -f ./make/stm32wl55jc_nucleo64.make clean
	@echo
	+@make --no-print-directory --directory ./I2S -f ./Makefile clean
	@echo
	+@make --no-print-directory --directory ../multi/PAL -f ./make/stm32wl55jc_nucleo64.make clean
	@echo
	+@make --no-print-directory --directory ./SPI -f ./Makefile clean
	@echo
	+@make --no-print-directory --directory ./SYSTICKv2 -f ./Makefile clean
	@echo
	+@make --no-print-directory --directory ../multi/TRNG -f ./make/stm32wl55jc_nucleo64.make clean
	@echo
	+@make --no-print-directory --directory ../multi/UART -f ./make/stm32wl55jc_nucleo64.make clean
	@echo
	+@make --no-print-directory --directory ./WDG -f ./Makefile clean
	@echo

#
##############################################################################
