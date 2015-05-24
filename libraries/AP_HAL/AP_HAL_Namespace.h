
#ifndef __AP_HAL_NAMESPACE_H__
#define __AP_HAL_NAMESPACE_H__


#include "string.h"
#include "utility/FastDelegate.h"

#if defined(__AVR__)
/*
  gcc on AVR doesn't allow for delegates in progmem. It gives a
  warning that the progmem area is uninitialised, and fills the area
  with zeros. This is a workaround.
 */
#define DELEGATE_FUNCTION_VOID_TYPEDEF(type)  typedef void (*type)(const void *)
#define AP_HAL_CLASSPROC_VOID(classptr, func) (void (*)(const void*))func
#else
#define DELEGATE_FUNCTION_VOID_TYPEDEF(type)  typedef fastdelegate::FastDelegate0<void> type
#define AP_HAL_CLASSPROC_VOID(classptr, func) fastdelegate::MakeDelegate(classptr, func)
#endif

// macros to hide the details of delegate functions using FastDelegate
#define AP_HAL_CLASSPROC(classptr, func) fastdelegate::MakeDelegate(classptr, func)
#define AP_HAL_MEMBERPROC(func) AP_HAL_CLASSPROC(this, func)

#define DELEGATE_FUNCTION0(rettype)          fastdelegate::FastDelegate0<rettype>
#define DELEGATE_FUNCTION1(rettype, args...) fastdelegate::FastDelegate0<rettype, args>
#define DELEGATE_FUNCTION2(rettype, args...) fastdelegate::FastDelegate0<rettype, args>

#define DELEGATE_FUNCTION(rettype, ...) fastdelegate::FastDelegate0<rettype, ## __VA_ARGS__>

#ifndef APM_BUILD_FUNCTOR
#define APM_BUILD_FUNCTOR 0
#endif

#if APM_BUILD_FUNCTOR
#include "utility/functor.h"

// Also add the hacks for the delegate implementation. Here it's just an alias
#define FUNCTOR_BIND_VOID(obj, func, rettype, ...) \
    FUNCTOR_BIND(obj, func, rettype, ## __VA_ARGS__)

#define FUNCTOR_TYPEDEF_VOID(name, rettype, ...) \
    FUNCTOR_TYPEDEF(name, rettype, ## __VA_ARGS__)

#else
#define FUNCTOR_TYPEDEF(name, rettype, ...) \
    typedef DELEGATE_FUNCTION(rettype, ## __VA_ARGS__) name

#define FUNCTOR_DECLARE(name, rettype, ...) \
    DELEGATE_FUNCTION(rettype, ## __VA_ARGS__) name

#define FUNCTOR_BIND(obj, func, rettype, ...) \
    AP_HAL_CLASSPROC(obj, func)

#define FUNCTOR_BIND_MEMBER(func, rettype, ...) \
    AP_HAL_MEMBERPROC(func)

#define FUNCTOR_BIND_VOID(obj, func, rettype, ...) \
    AP_HAL_CLASSPROC_VOID(obj, func)

#define FUNCTOR_TYPEDEF_VOID(name, rettype, ...) \
    DELEGATE_FUNCTION_VOID_TYPEDEF(name)

#endif

namespace AP_HAL {

    /* Toplevel pure virtual class Hal.*/
    class HAL;

    /* Toplevel class names for drivers: */
    class UARTDriver;
    class I2CDriver;

    class SPIDeviceDriver;
    class SPIDeviceManager;

    class AnalogSource;
    class AnalogIn;
    class Storage;
    class DigitalSource;
    class GPIO;
    class RCInput;
    class RCOutput;
    class Scheduler;
    class Semaphore;
    
    class Util;

    /* Utility Classes */
    class Print;
    class Stream;
    class BetterStream;

    /* Typdefs for function pointers (Procedure, Member Procedure) 

       For member functions we use the FastDelegate delegates class
       which allows us to encapculate a member function as a type
     */
    typedef void(*Proc)(void);
    FUNCTOR_TYPEDEF(MemberProc, void);

    /**
     * Global names for all of the existing SPI devices on all platforms.
     */

    enum SPIDevice {
        SPIDevice_Dataflash         = 0,
        SPIDevice_ADS7844           = 1,
        SPIDevice_MS5611            = 2,
        SPIDevice_MPU6000           = 3,
        SPIDevice_ADNS3080_SPI0     = 4,
        SPIDevice_ADNS3080_SPI3     = 5,
        SPIDevice_MPU9250           = 6,
        SPIDevice_L3GD20            = 7,
        SPIDevice_LSM303D           = 8,        
        SPIDevice_LSM9DS0_AM        = 9,
        SPIDevice_LSM9DS0_G         = 10,
        SPIDevice_Ublox             = 11
    };

}

#endif // __AP_HAL_NAMESPACE_H__
