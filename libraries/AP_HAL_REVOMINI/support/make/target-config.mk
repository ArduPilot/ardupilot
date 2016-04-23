# Board-specific configuration values.  Flash and SRAM sizes in bytes.

ifeq ($(BOARD), revomini_MP32V1F4)
   MCU            := STM32F407VG
   PRODUCT_ID     := 0003
   ERROR_LED_PORT := GPIOB
   ERROR_LED_PIN  := 4
   DENSITY        := STM32_HIGH_DENSITY
   FLASH_SIZE     := 1048576
   SRAM_SIZE      := 131072
endif

# Memory target-specific configuration values

ifeq ($(MEMORY_TARGET), ram)
   LDSCRIPT := $(BOARD)/ram.ld
   VECT_BASE_ADDR := VECT_TAB_RAM
endif
ifeq ($(MEMORY_TARGET), flash)
   LDSCRIPT := $(BOARD)/flash.ld
   VECT_BASE_ADDR := VECT_TAB_FLASH
endif
ifeq ($(MEMORY_TARGET), jtag)
   LDSCRIPT := $(BOARD)/jtag.ld
   VECT_BASE_ADDR := VECT_TAB_BASE
endif
