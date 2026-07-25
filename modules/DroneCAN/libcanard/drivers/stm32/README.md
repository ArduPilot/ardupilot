# Libcanard Driver for STM32 Microcontrollers

This is a compact and simple driver for STM32 microcontrollers.
It has no dependencies besides a tiny part of the standard C library,
and can be used with virtually any operating system or on bare metal
(so far it has been tested at least with ChibiOS).

In theory, the entire family of STM32 should be supported by this driver,
since they all share the same CAN controller IP known as bxCAN.
So far this driver has been tested at least with the following MCU:

* STM32F091
* STM32F105 - both CAN1 and CAN2
* STM32F446
* STM32F303 - Only CAN1 verified
* Please extend this list if you used it with other MCU.

## Features

* Proper handling of the TX queue prevents inner priority inversion.
* Dependency free, works with any OS and on bare metal.
* Compact, suitable for ROM and RAM limited applications (e.g. bootloaders).
* Does not use IRQ and critical sections at all.
* Non-blocking API.
* Supports both CAN1 and CAN2, but only one at a time via a compile time switch.
* Supports hardware acceptance filters.
* Supports proper CAN bus timing configuration in a user friendly way.

## Caveats

Some design trade-offs have been made in order to provide the above features.

* The RX FIFO is only 3 CAN frames deep,
since this is the depth provided by the CAN hardware.
In order to avoid frame loss due to RX overrun,
the following measures should be adopted:
  * Use hardware acceptance filters - the driver
provides a convenient API to configure them.
  * Read the queue at least every 3x minimum frame transmission intervals.
* The driver does not permit concurrent access from different threads of execution.
* The clocks of the CAN peripheral must be enabled by the application
before the driver is initialized.
The driver cannot do that because this logic is not uniform across the STM32 family.

Note that it is possible to invoke the driver's API functions from IRQ context,
provided that the application takes care about proper guarding with critical sections.
This approach allows the application to read the RX queue from an external CAN IRQ handler.

## How to Use

The driver is so simple its entire documentation is defined in the header file.
Please do endeavor to read it.

[Use code search to find real life usage examples using the keyword `canard_stm32`](https://github.com/search?q=canard_stm32&type=Code&utf8=%E2%9C%93).
