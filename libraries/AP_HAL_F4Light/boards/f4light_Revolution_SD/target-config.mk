# Board-specific configuration values.  Flash and SRAM sizes in bytes.

   MCU            := STM32F405RG
   PRODUCT_ID     := 0003
   DENSITY        := STM32_HIGH_DENSITY
   FLASH_SIZE     := 1048576
   SRAM_SIZE      := 131072

    BOARD_TYPE     := 70
    BOARD_REV      := 4
    BOOTLOADER     := revo405_bl
    
# Memory target-specific configuration values

ifeq ($(MEMORY_TARGET), ram)
   LDSCRIPT := ram.ld
   VECT_BASE_ADDR := VECT_TAB_RAM
endif
ifeq ($(MEMORY_TARGET), flash)
   LDSCRIPT := flash.ld
   VECT_BASE_ADDR := VECT_TAB_FLASH
endif
ifeq ($(MEMORY_TARGET), jtag)
   LDSCRIPT := jtag.ld
   VECT_BASE_ADDR := VECT_TAB_BASE
endif
