/*
This is a collection of functions for operating GPIO and ADC
on the TI AM335x Sitara Processors. These operate with direct memory access
instead of userspace drivers. Therefore this code must be run as root, but
they execute roughly 2 orders of magnitude faster than userspace drivers
depending on the exact function.

Note that a device tree overlay is still necessary to configure the
pin multiplexer and enable clock signal to each subsystem.

Big thanks to Ethan Hayon for the GPIO code
*/

#ifndef MMAP_GPIO_ADC
#define MMAP_GPIO_ADC

#define HIGH 1
#define LOW  0 

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif	


// GPIO
EXTERNC int initialize_mmap_gpio();


// ADC
EXTERNC int initialize_mmap_adc();
EXTERNC int mmap_adc_read_raw(int ch);


#endif


