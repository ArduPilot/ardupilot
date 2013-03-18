#ifndef SIM900Driver_h
#define SIM900Driver_h

#include "MobileDriver.h"

class SIM900Driver : public MobileDriver {
public:
	SIM900Driver(void) : MobileDriver(){}

protected:
	const MobileDriver::State* initialState(void) const;
	const MobileDriver::State* dataTransmissionState(void) const;
	const MobileDriver::State* resetState(void) const;
	const MobileDriver::State* receiveSMSState(void) const;
	const MobileDriver::State* matchInterrupts(void) const;
	const MobileDriver::State* matchURCs(void) const;
	bool isConnectionOpen(const MobileDriver::State* state) const;
};

#endif

