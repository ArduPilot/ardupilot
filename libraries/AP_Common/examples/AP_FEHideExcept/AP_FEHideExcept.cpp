//
// Examples for AP_FEHideExcept
//

#include <AP_Common/AP_FEHideExcept.h>
#include <AP_HAL/AP_HAL.h>

#include <fenv.h>

#include <chrono>
#include <thread>

void print_supported_fpu_env();
void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void print_supported_fpu_env()
{
    // IEEE 754 exception macros
    hal.console->printf("IEEE 754 exception macros\n");
#ifdef FE_ALL_EXCEPT
    hal.console->printf("Have FE_ALL_EXCEPT         yes\n");
#else
    hal.console->printf("Have FE_ALL_EXCEPT         no\n");
#endif

#ifdef FE_INEXACT
    hal.console->printf("Have FE_INEXACT            yes\n");
#else
    hal.console->printf("Have FE_INEXACT            no\n");
#endif

#ifdef FE_DIVBYZERO
    hal.console->printf("Have FE_DIVBYZERO          yes\n");
#else
    hal.console->printf("Have FE_DIVBYZERO          no\n");
#endif

#ifdef FE_UNDERFLOW
    hal.console->printf("Have FE_UNDERFLOW          yes\n");
#else
    hal.console->printf("Have FE_UNDERFLOW          no\n");
#endif

#ifdef FE_OVERFLOW
    hal.console->printf("Have FE_OVERFLOW           yes\n");
#else
    hal.console->printf("Have FE_OVERFLOW           no\n");
#endif

#ifdef FE_INVALID
    hal.console->printf("Have FE_INVALID            yes\n");
#else
    hal.console->printf("Have FE_INVALID            no\n");
#endif

#ifdef FE_DOWNWARD
    hal.console->printf("Have FE_DOWNWARD           yes\n");
#else
    hal.console->printf("Have FE_DOWNWARD           no\n");
#endif

#ifdef FE_TONEAREST
    hal.console->printf("Have FE_TONEAREST          yes\n");
#else
    hal.console->printf("Have FE_TONEAREST          no\n");
#endif

#ifdef FE_TONEAREST
    hal.console->printf("Have FE_TOWARDZERO         yes\n");
#else
    hal.console->printf("Have FE_TOWARDZERO         no\n");
#endif

#ifdef FE_UPWARD
    hal.console->printf("Have FE_UPWARD             yes\n");
#else
    hal.console->printf("Have FE_UPWARD             no\n");
#endif

    // Intel specific exception macros
    hal.console->printf("\nIntel exception macros\n");
#ifdef FE_DENORMALOPERAND
    hal.console->printf("Have FE_DENORMALOPERAND    yes\n");
#else
    hal.console->printf("Have FE_DENORMALOPERAND    no\n");
#endif

    // ARM specific exception macros
    hal.console->printf("\nARM exception macros\n");
#ifdef FE_FLUSHTOZERO
    hal.console->printf("Have FE_FLUSHTOZERO        yes\n");
#else
    hal.console->printf("Have FE_FLUSHTOZERO        no\n");
#endif
}

void setup(void)
{
    hal.console->printf("AP_FEHideExcept Tests\n\n");

    print_supported_fpu_env();

    hal.console->printf("\n");
}

double zero = 0.0; 

void loop(void)
{

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // hide floating point exceptions
    {
        FEHideExcept hide_except;

        hal.console->printf("1.0/0.0 = %f\n", 1.0 / zero);

        if(fetestexcept(FE_DIVBYZERO)) {
            hal.console->printf("division by zero reported\n");
        } else {
            hal.console->printf("division by zero not reported\n");
        }
    }

    // floating point exceptions cleared and unhidden
    {
        // uncommenting the line below will cause an abort
        // hal.console->printf("1.0/0.0 = %f\n", 1.0 / zero);

        if(fetestexcept(FE_DIVBYZERO)) {
            hal.console->printf("division by zero reported\n");
        } else {
            hal.console->printf("division by zero not reported\n");
        }
    }
#endif  // CONFIG_HAL_BOARD

    // hal.scheduler->delay(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}


AP_HAL_MAIN();
