this is universal HAL for almost any F4 board, without any external OS

per-board Readme files are in boards/{board}/1_Readme.md


Warning!!!
EEPROM emulation in Flash cause periodic program hunging on time of sector erase! So to allow auto-save parameters
like MOT_THST_HOVER - MOT_HOVER_LEARN to be 2 you should defer parameter writing (Param BRD_EE_DEFER)



Timer usage:

1 RC-Output on some boards
2 RC-Output
3 RC-Output
4 soft_i2c0, PPM_IN on AirbotV2
5 micros()
6 event generation for WFE
7 scheduler
8 PPM_IN
9 soft_i2c1
10 soft_i2c2
11
12 PPM_IN
13 driver's io_completion
14 schedule tail timer
