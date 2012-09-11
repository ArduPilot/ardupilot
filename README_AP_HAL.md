AP\_HAL Readme

Pat Hickey

Galois Inc

11 Sept 2011

--------
Overview
========

AP\_HAL is hardware abstraction layer for the ArduPilot project. The goal is
to separate AVR (via avr-libc) and Arduino (via the Arduino core and included
libraries like SPI and Wire) dependencies from the ArduPilot programs and
libraries.

This will make it possible to port the ArduPlane and ArduCopter programs
(and their associated libraries) to a new platform without changing any of
the ArduPlane or ArduCopter code, only implementing the AP\_HAL interface
for the new platform.

Currently, the AP\_HAL\_AVR library , found in /libraries/AP\_HAL\_AVR/, is an
implementation of AP\_HAL for the ArduPilot hardware platforms.

AP\_HAL\_AVR exports two HAL instances, `AP_HAL_AVR_APM1` and `AP_HAL_AVR_APM2`,
which provide the instances for APM1 and APM2 hardware.

-----------
Requirments
===========

AP\_HAL is designed to work with the Arduino 1.0.1 IDE with a special "Coreless"
modification, or with the ArduPilot Arduino.mk makefiles with a similar Coreless
extension. The Arduino.mk Makefile in the AP\_HAL development branch also
implements the coreless modification. You will not need to upgrade to a 
coreless patched IDE if you are just using Makefiles.

The Coreless modification to the Arduino IDE will be made available as a patch
and as compiled IDEs on the ArduPilot project's Downloads section.

Why Coreless Arduino
--------------------

todo

Consequences of Coreless Arduino
--------------------------------

* Need to provide an empty "Arduino.h" somewhere in the include path of each
  sketch. The line `#include <Arduino.h>` is added to each sketch by the
  Arduino IDE's sketch preprocessor, as well as by the Makefile build.

* Need to provide an entry point `int main(void)` for the application.
  Previously, a trivial implementation existed in the Arduino core:
    ```
    int main(void) {
        init(); /* Initialized the Arduino core functionality */
        setup();
        while(1) loop();
        return 0; /* Never reached */
    }
      
    ```

  Each program built with the coreless Arduino build will need to provide its
  own `main` function. The following implementation may be used in at the bottom
  of the sketch PDE/INO file:
    ```
    extern "C" {
        int main(void) {
            hal.init(NULL);
            setup();
            while(1) loop();
            return 0;
        }
    }
    ```
    The `extern "C"` wrapper is required because a PDE/INO file is compiled
    as C++.

* Global objects which were previously available in all Arduino sketches, such
  as Serial, Serial1, etc. no longer exist. This is by design - all of those
  hardware interfaces should be accessed through the HAL.

-------------------------
Using The AP\_HAL Library
=========================

AP\_HAL, found in /libraries/AP\_HAL/, is a library of purely virtual classes:
there is no concrete code in the AP\_HAL library, only interfaces. All code in
the core ArduPlane & ArduCopter programs, ArduPilot libraries, and example
sketches should depend only on the interfaces exposed by AP\_HAL.

The collection of classes in the AP\_HAL library exist in the AP\_HAL C++ 
namespace. The convention is for a program to instantiate a single instance
of the `AP_HAL::HAL` class, under a reference to the name `hal`.
    ```
    #include <AP_HAL.h>
    const AP_HAL::HAL& hal = specific_hal_implementation;
    ```
This instance should be made in a single object file.  All other object files,
including libraries (even those inside an AP\_HAL implementation, should use
the AP\_HAL interface by declaring an extern reference to `hal`.
    ```
    #include <AP_HAL.h>
    extern const AP_HAL::HAL& hal;
    ```

-----------------------------
Using The AP\_HAL\_AVR library
=============================

The AP\_HAL\_AVR library exports AP\_HAL::HAL instances for the APM1 and APM2. 
These instances are made of a number of concrete classes which implement the
AP\_HAL interfaces for the AVR platform. These implementations depend only on
avr-libc and not the Arduino core.

Some of the code in AP\_HAL\_AVR, such as the the GPIO class's pinMode, read,
and write, has been derived directly from the source code of the Arduino core
pinMode, digitalRead, and digitalWrite. 

When using the coreless Arduino IDE to build for AVR, you will need the
following three libraries included in the top level of your sketch:
    ```
    #include <AP_Common.h>
    #include <AP_HAL.h>
    #include <AP_HAL_AVR.h>
    ```
and then declare one of the following hal lines depending on your platform:
    ```
    const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
    ```
    or
    ```
    const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
    ```

-----------------------
AP\_HAL Library Contents
========================

The `AP_HAL` library is organized as follows:

AP\_HAL.h : exports all other headers for the library.

AP\_HAL\_Namespace.h : exposes the C++ namespace `AP_HAL`. The namespace
declaration declares each class by name (not implementation) and some useful
typedefs.

The AP\_HAL interface classes are each defined in a header file bearing their
name.


AP\_HAL::HAL
------------

The `AP_HAL::HAL` class is a container for the a complete set of device
drivers.  The class is defined in `/libraries/AP_HAL/HAL.h`.  It also has a
virtual (i.e. overridable) method to handle driver initialization.  Each device
driver is exposed as a pointer an AP\_HAL driver class, (e.g.  each serial
driver is exposed as a public `UARTDriver* uartN`).

The following drivers are public members of `AP_HAL::HAL`. (Each class is in
the `AP_HAL` namespace, left off for brevity.)

*  `UARTDriver* uart0` : Corresponds to ArduPilot FastSerial library override
    of the Arduino core Serial object
*  `UARTDriver* uart1` : Corresponds to Serial1 object (as above)
*  `UARTDriver* uart2` : Corresponds to Serial2 object (as above)
*  `UARTDriver* uart3` : Corresponds to Serial3 object (as above)
*  `I2CDriver*  i2c` : Corresponds to ArduPilot `/libraries/I2C` driver
*  `SPIDriver*  spi` : Corresponds to Arduino library SPI object
*  `AnalogIn*   analogin` : Corresponds to
   `/libraries/AP_AnalogSource/AP_AnalogSource_Arduino` driver
*  `Storage*    storage` : Corresponds to avr-libc's `<avr/eeprom.h>`
    driver
*  `Dataflash*  dataflash` : Corresponds to ArduPilot `/libraries/DataFlash`
    driver
*  `BetterStream* console` : New utility for warning and error reporting
*  `GPIO*       gpio` : Corresponds to Arduino core `pinMode`,
    `digitalRead`, and `digitalWrite` functionality
*  `RCInput*    rcin` : Corresponds to PPM input side of ArduPilot
    `/libraries/APM_RC` library
*  `RCOutput*   rcout` : Corresponds to PPM output side of ArduPilot
    `/libraries/APM_RC` library
*  `Scheduler*  scheduler` : Encompasses both Arduino core timing functions
    such as `millis` and `delay` and the ArduPilot
    `/library/AP_PeriodicProcess/` driver.

`AP_HAL` also has an unimplemented `virtual void init(void* opts) const` method.
This method should initialize each driver before the call to a sketch's `setup`
method.



AP\_HAL::UARTDriver
--------------------

The `AP_HAL::UARTDriver` class is the AP\_HAL replacment for ArduPilot's
`FastSerial` library. 

The `AP_HAL::UARTDriver` class is a pure virtual interface. The code is derived
directly from the `FastSerial` class. It provides the methods `begin()`,
`end()`, `flush()`, `is_initialized()`, `set_blocking_writes`, and
`tx_pending`.  The class hierchary for `AP_HAL::UARTDriver` is also derived
directly from the `FastSerial` class's hierarchy . `AP_HAL::UARTDriver` is a
`public AP_HAL::BetterStream`, which is a `public AP_HAL::Stream`, which is a
`public AP_HAL::Print`.

The `utility/` directory contains the definitions of the `AP_HAL::Print`,
`AP_HAL::Stream`, and `AP_HAL::BetterStream` classes, as well as default
implementations for the `AP_HAL::Print` class.

`AP_HAL::Print` and `AP_HAL::Stream` are derived directly from the classes of
the same name in the Arduino core. Some methods dealing with the Arduino core's
C++ `String`s, and the `Printable` class, have been left out.

The `AP_HAL::Print` class has default implementations of all of the print
methods. Each of these methods is implemented in terms of
`virtual size_t write(uint8_t char)`, which is expected to be implemented by
a class deriving from `AP_HAL::Print`. This is the same structure as used in
the Arduino core.

The `AP_HAL::Stream` class is based on the Arduino core's Stream, but reduced
to just the methods we actually use in ArduPilot - `int available()`,
`int read()`, and `int peek()`. These methods are all pure virtual

The `AP_HAL::BetterStream` class is a pure virtual version of the class by the
same name from Mike Smith's `FastSerial` library. It exposes the methods
`int txspace()`, `void print_P()`, `void println_P`, `void printf()`, and
`void printf_P()`.  As in FastSerial's BetterStream library, function names
postfixed with `_P` take a string in program space as their first argument.
To use the AVR program space types, the `AP_HAL::BetterStream` class depends on 
the `<avr/pgmspace.h>` and `AP_Common.h` header files. This is the only part of
the `AP_HAL` library which still depends on an AVR specific library. We will
find a way to make this dependency modular by isolating the parts of the 
`AP_Common` and `<avr/pgmspace.h>` headers which BetterStream depends upon
into a single header, and then conditionally defining those typedefs and
macros to innocuous implementations when compiling for other platforms.


AP\_HAL::I2CDriver
------------------

The `AP_HAL::I2CDriver` class is the AP\_HAL replacment for ArduPilot's
`I2C` library. (ArduPilot's `I2C` library is a replacment for the Arduino
provided `Wire` library. The `Wire` library is so bad I could cry.)

The `AP_HAL::I2CDriver` class is a pure virtual interface, found in
`/libraries/AP_HAL/I2CDriver.h.` The methods resemble those in defined by the
`I2C` class in  `/libraries/I2C/I2C.h`, but to ease future implementations, all
of the old Arduino `Wire` class compatibility methods have been dropped.

The `I2CDriver` interface supports the timeout features we require to assure
safe timing properties when polling the hardware I2C peripeheral. It also
only exposes whole-transaction interfaces to the user, to support more efficient
implementations in a threaded environment.


AP\_HAL::SPIDriver
------------------

The `AP_HAL::SPIDriver` class is pure virtual and can be found in
`/libraries/AP_HAL/SPIDriver.h`.

The `AP_HAL::SPIDriver` class is derived from, but only slightly resembles the
Arduino core's `SPI` library.  Methods we don't use have been removed, as has
the export of a global `SPI` object. The `begin` method has been swapped for a
more idiomatic `init` method, and `setClockDivider` has been replaced with a
more general `setSpeed` method.  In addition, it is not possible to operate the
SPI device as a slave.  The `uint8_t transfer(uint8_t data)` method remains in
common.

The `SPIDriver` interface (and the existing AVR implementation) will require
significant changes to support efficient implementations in a threaded
environment. Transfers will have to be possible in bulk, and somehow incorporate
the notion of the target device so that specialized hardware can manage the
slave device select lines. Currently, device select signals are sent using the
`AP_HAL::GPIO::write` method, which is the AP\_HAL analog of Arduino's
`digitalWrite`.


AP\_HAL::AnalogIn
-----------------

The `AP_HAL::AnalogIn` class is pure virtual and can be found in
`/libraries/AP_HAL/AnalogIn.h`. The pure virtual `AP_HAL::AnalogSource` class
is also defined in that class.

The `AP_HAL::AnalogSource` interface is based loosely on the `AP_AnalogSource`
interface in the existing AP\_AnalogSource library. At this time, an
`AP_HAL::AnalogSource` has a single method `float read()` which returns the
latest analog measurment. There are no methods for flow control - currently you
must assume the method will block until a measurment is ready; the timing is
entirely implementation defined.

The `AP_HAL::AnalogIn` interface does not have a close analog in the existing
libraries. Consider it to be, loosely, a factory for `AnalogSource` objects.
There are only two methods: the standard `void init(void*)` initializer and
a numbered interface to the available analog channels `AP_HAL::AnalogSource*
channel(int n)`. At this time the significance of each numbered channel is
determined by the implementation.

Extensions will be made to these interfaces as required to meet the needs of
other platforms. We will also have to consider making named channels, as
opposed to numbered channels, available from the  `AP_HAL::AnalogIn` interface


AP\_HAL::Storage
----------------

The `AP_HAL::Storage` class is pure virtual and can be found in
`/libraries/AP_HAL/Storage.h`. It is the AP\_HAL interface to take the place
of the AVR EEPROM, possibly with other nonvolatile storage.

The `AP_HAL::Storage` interface very closely resembles the avr-libc eeprom
interface. The use of `uint8_t*` projections into storage space are subject to
change - it seems to make more sense to use integer types to designate these
locations, as there is no valid dereference of a pointer value.

AP\_HAL::Dataflash
------------------

The `AP_HAL::Dataflash` class is pure virtual and can be found in
`/libraries/AP_HAL/Dataflash.h`. It is based on the existing ArduPilot
`DataFlash` library found in `/libraries/DataFlash`. The `AP_HAL::Dataflash`
interface is a cleaned up version of that library's interface. Public member
variables have been replaced with getter methods, unused public interfaces
have been removed, and all methods have had their names translated to lowercase
and underscores for style.

Similar to the problems with the SPI library, the existing `DataFlash` library
interface is oriented for byte-at-a-time writes to the hardware device. This
interface may have to be revised to support bulk transfers for efficient use
in a threaded environment.


AP\_HAL Console driver
----------------------

In the existing ArduPilot code, there is no unified way to send debugging
messages, warnings, and errors to the user. A dedicated Console driver, exposed
as an `AP_HAL::BetterStream`, is envisioned as a better way to communicate that
sort of information.

In the future, I envision extending this driver to support a transport layer
over a Mavlink connection.

Implementations may either leave the console unimplemented or alias it to one
of the UARTDriver implementations.

AP\_HAL::RCInput
--------------

The `AP_HAL::RCInput` class is pure virtual and can be found in
`/libraries/AP_HAL/RCInput.h`. The RCInput interface is based on the
input related methods of the existing ArduPilot `APM_RC` class. RCInput methods
were separated from RCOutput methods for clarity.

The methods `uint8_t valid()` and `uint16_t read(uint8_t)` carry over the exact
interface found in `APM_RC`, which is to specify the number of valid channels,
and then read each channel out by number.

Based on the most common use case, which is to read the valid flag and then all
input channels sequentially, a new interface `uint8_t read(uint16_t* periods,
uint8_t len)` makes the valid flag available as the return value, and writes
the periods as an array to the memory specified by the first argument.
This bulk read interface may be more efficient in a threaded environment.

AP\_HAL::RCOutput
--------------

The `AP_HAL::RCOutput` class is pure virtual and can be found in
`/libraries/AP_HAL/RCOutput.h`. The RCOutput interface is based on the
input related methods of the existing ArduPilot `APM_RC` class. RCOutput methods
were separated from RCInput methods for clarity.

The output methods from `APM_RC` were reproduced here faithfully, with minor
differences to naming. As an extension, the `read` and `write` methods are
overloaded to also have versions which take or give arrays of values, as the
most common use case is to read or write all channels sequentially, and bulk
reads and writes may be more efficient in a threaded environment.

AP\_HAL::Scheduler
--------------

The `AP_HAL::Scheduler` class is pure virtual and can be found in
`/libraries/AP_HAL/Scheduler.h`. 



Remaining ArduPilot AVR dependencies
====================================

The class `AP_Param` makes extensive use of AVR program space pointers and
accessors in its implementation. The definition of these types and accrssors
will have to be defined in a single external header file so they can be
replaced by innocuous types and accessors when compiling for other platforms


