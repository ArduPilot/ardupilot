#ifndef APM_PerfMon_h
#define APM_PerfMon_h

// macros to make integrating into code easier
#define APM_PERFMON_REGISTER static int myFunc = APM_PerfMon::recordFunctionName(__func__); APM_PerfMon perfMon(myFunc);
#define APM_PERFMON_REGISTER_NAME(functionName) static int myFunc = APM_PerfMon::recordFunctionName(functionName); APM_PerfMon perfMon(myFunc);

#define PERFMON_MAX_FUNCTIONS 50
#define PERFMON_FUNCTION_NAME_LENGTH 20

__extension__ typedef int __guard __attribute__((mode (__DI__))); 

extern "C" int __cxa_guard_acquire(__guard *); 
extern "C" void __cxa_guard_release (__guard *); 
extern "C" void __cxa_guard_abort (__guard *);

#include "HardwareSerial.h"

class APM_PerfMon
{
    public:
        // static variables
        static int nextFuncNum;
		static char functionNames[PERFMON_MAX_FUNCTIONS][PERFMON_FUNCTION_NAME_LENGTH];
        static unsigned long time[PERFMON_MAX_FUNCTIONS];
		static unsigned long numCalls[PERFMON_MAX_FUNCTIONS];
		static unsigned long allStartTime;
		static unsigned long allEndTime;
		static APM_PerfMon* lastCreated;

		// static methods
        static int recordFunctionName(const char funcName[]);
        static void DisplayResults(HardwareSerial* aSerial);		
		static void ClearAll();
		static int strLen(char* str);
		
        // normal variables
        int _funcNum;
        unsigned long _startTime;
		APM_PerfMon* _parent;

		// normal methods
		APM_PerfMon(int funcNum);  // Constructor - records function start time
		~APM_PerfMon();  // Destructor - records function end time
		void stop();  // stops recording time spent in this function - meant to be called by a child.
		void start();  // restarts recording time spent in this function
};

#endif  // APM_PerfMon_h