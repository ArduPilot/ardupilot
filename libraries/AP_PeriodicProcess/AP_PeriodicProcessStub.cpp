
#include "AP_PeriodicProcessStub.h"
AP_PeriodicProcessStub::AP_PeriodicProcessStub(int period) {}
void AP_PeriodicProcessStub::init( Arduino_Mega_ISR_Registry * isr_reg ){}
void AP_PeriodicProcessStub::register_process(void (*proc)(void) ) {}
void AP_PeriodicProcessStub::run(void) {}
