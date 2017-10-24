/*
This is a collection of functions for operating eQEP and PWM
on the TI AM335x Sitara Processors. These operate with direct memory access
instead of userspace drivers. Therefore this code must be run as root, but
they execute roughly 2 orders of magnitude faster than userspace drivers
depending on the exact function.

Note that a device tree overlay is still necessary to configure the
pin multiplexer and enable clock signal to each subsystem.

the pwm driver is still needed to set up pwm output. This can
be done through /sys/class/pwm or with simple_pwm.c
*/

#ifndef MMAP_PWMSS
#define MMAP_PWMSS
#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif	

// eQEP
EXTERNC int init_eqep(int ss);
EXTERNC int read_eqep(int ch);
EXTERNC int write_eqep(int ch, int val);


#endif



