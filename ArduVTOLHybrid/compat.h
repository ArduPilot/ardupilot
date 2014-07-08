
#ifndef __COMPAT_H__
#define __COMPAT_H__

#define HIGH 1
#define LOW 0

/* Forward declarations to avoid broken auto-prototyper (coughs on '::'?) */
static void run_cli(AP_HAL::UARTDriver *port);

#endif // __COMPAT_H__

