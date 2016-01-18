#ifndef AP_PERFMON_H
#define AP_PERFMON_H

// macros to make integrating into code easier
#define AP_PERFMON_REGISTER static uint8_t myFunc = AP_PerfMon::recordFunctionName(__func__); AP_PerfMon perfMon(myFunc);
#define AP_PERFMON_REGISTER_NAME(functionName) static uint8_t myFunc = AP_PerfMon::recordFunctionName(functionName); AP_PerfMon perfMon(myFunc);

#define AP_PERFMON_REGISTER_FN(func_name) static uint8_t func_name ## _num = AP_PerfMon::recordFunctionName(#func_name);
#define AP_PERFMON_FUNCTION(func_name) AP_PerfMon perfMon(func_name ## _num);


#define PERFMON_MAX_FUNCTIONS 12
#define PERFMON_FUNCTION_NAME_LENGTH 10

#include <AP_HAL.h>
#include <AP_Math.h>

class AP_PerfMon
{
    public:
		// static methods
        static uint8_t recordFunctionName(const char funcName[]);
        static void DisplayResults();
		static void ClearAll();
        static void DisplayAndClear(uint32_t display_after_seconds);  // will display results after this many milliseconds.  should be called regularly

		// public methods
		AP_PerfMon(uint8_t funcNum);    // Constructor - records function start time
		~AP_PerfMon();                  // Destructor - records function end time
		void stop();                    // stops recording time spent in this function - meant to be called by a child.
		void start();                   // restarts recording time spent in this function

    private:
        // static variables
        static uint8_t nextFuncNum;
		static char functionNames[PERFMON_MAX_FUNCTIONS][PERFMON_FUNCTION_NAME_LENGTH];
        static uint32_t time[PERFMON_MAX_FUNCTIONS];
		static uint32_t numCalls[PERFMON_MAX_FUNCTIONS];
        static uint32_t maxTime[PERFMON_MAX_FUNCTIONS];
		static uint32_t allStartTime;
		static uint32_t allEndTime;
		static AP_PerfMon* lastCreated;
        static bool _enabled;

        // instance variables
        uint8_t _funcNum;
        unsigned long _startTime;
        unsigned long _time_this_iteration;
		AP_PerfMon* _parent;
};

#endif  // AP_PERFMON_H