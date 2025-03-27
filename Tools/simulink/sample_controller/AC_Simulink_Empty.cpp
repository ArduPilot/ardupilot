

#include "AC_Simulink_Empty.h"
AC_Simulink_Empty::AC_Simulink_Empty() {}

void AC_Simulink_Empty::init() {
    SampleController_init();
}

void AC_Simulink_Empty::update() {
    SampleController_loop();
}

void AC_Simulink_Empty::reset() {

}