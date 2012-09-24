
#include "AP_PeriodicProcessStub.h"
AP_PeriodicProcessStub::AP_PeriodicProcessStub(uint8_t period) {
}
void AP_PeriodicProcessStub::init( Arduino_Mega_ISR_Registry * isr_reg ){
}
void AP_PeriodicProcessStub::register_process(ap_procedure proc) {
}
void AP_PeriodicProcessStub::set_failsafe(ap_procedure proc) {
}
bool AP_PeriodicProcessStub::queue_process(ap_procedure proc) {
	return true;
}
void AP_PeriodicProcessStub::suspend_timer(void) {
}
void AP_PeriodicProcessStub::resume_timer(void) {
}
void AP_PeriodicProcessStub::run(void) {
}
