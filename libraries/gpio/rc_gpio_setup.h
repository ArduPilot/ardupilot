/*******************************************************************************
* rc_gpio_setup.h
* just the one declaration for use by roboticscape.c
*******************************************************************************/

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif	

EXTERNC int configure_gpio_pins();
