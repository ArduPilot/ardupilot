to add your own board on any STM32F4XX you should compltete thease steps:


 *  give a name for your board - eg. yourboard_F4
 *  make directory  "yourboard_F4" in folder 'boards'
 *  copy all files from folder of most likely board (eg. revomini_Revolution) to this folder
 *  open board.c and adjust PIN_MAP[] array to your CPU usage
 *  change in "board.h" pin definitions. see pin number in PIN_MAP array
 *  check settings in board.h
 *  check settings in target-config.mk
 *  check PLL setup in system_stm32f4xx.c
 *  compile as "make f4light BOARD=yourboard_F4"
 
 