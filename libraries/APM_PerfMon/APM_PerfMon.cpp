
extern "C" {
  // AVR LibC Includes
  #include "WConstants.h"
}

#include "APM_PerfMon.h"

// don't know why i need these
// see http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=410870
int __cxa_guard_acquire(__guard *g) {return !*(char *)(g);}; 
void __cxa_guard_release (__guard *g) {*(char *)g = 1;}; 
void __cxa_guard_abort (__guard *) {};

// static class variable definitions
int APM_PerfMon::nextFuncNum;
char APM_PerfMon::functionNames[PERFMON_MAX_FUNCTIONS][PERFMON_FUNCTION_NAME_LENGTH];
unsigned long APM_PerfMon::time[PERFMON_MAX_FUNCTIONS];
unsigned long APM_PerfMon::numCalls[PERFMON_MAX_FUNCTIONS];
unsigned long APM_PerfMon::allStartTime;
unsigned long APM_PerfMon::allEndTime;
APM_PerfMon* APM_PerfMon::lastCreated = NULL;

// constructor
APM_PerfMon::APM_PerfMon(int funcNum)
{
    // stop recording time from parent
	if( lastCreated != NULL )
	    lastCreated->stop();

    // check global start time
	if( allStartTime == 0 )
	    allStartTime = micros();
		
    _funcNum = funcNum;		// record which function we should assign time to
	_parent = lastCreated;  // add pointer to parent
	lastCreated = this;     // record myself as the last created instance
	
	numCalls[_funcNum]++;   // record that this function has been called
	start();  				// start recording time spent in this function
}

// destructor
APM_PerfMon::~APM_PerfMon()
{
    stop();   // stop recording time spent in this function
	lastCreated = _parent;  // make my parent the last created instance
	
	// restart recording time for parent
	if( lastCreated != NULL )
	    lastCreated->start();
}

// stop recording time
void APM_PerfMon::stop()
{
    time[_funcNum] += (micros()-_startTime);   
}

// stop recording time
void APM_PerfMon::start()
{
    _startTime = micros();  // restart recording time spent in this function  
}

// record function name in static list
int APM_PerfMon::recordFunctionName(const char funcName[])
{
    int nextNum = nextFuncNum++;
    int i;
    
    // clear existing function name (if any)
    functionNames[nextNum][0] = 0;
    
    // store function name
    for( i=0; i<PERFMON_FUNCTION_NAME_LENGTH-1 && funcName[i] != 0; i++ )
    {
        functionNames[nextNum][i] = funcName[i];      
    }
    functionNames[nextNum][i] = 0;
    
    return nextNum;
}

// ClearAll - clears all data from static members
void APM_PerfMon::ClearAll()
{
    int i;
	
	allStartTime = 0;
	allEndTime = 0;
	
	for(i=0; i<PERFMON_MAX_FUNCTIONS; i++)
	{
		time[i] = 0;  // reset times
		numCalls[i] = 0;  // reset num times called
	}
}

// ClearAll - clears all data from static members
void APM_PerfMon::DisplayResults(HardwareSerial* aSerial)
{
    int i,j,padding;
	float hz;
	float pct;
	unsigned long totalTime;
	unsigned long sumOfTime = 0;
	unsigned long unExplainedTime;
	
	if( allEndTime == 0 )
	    allEndTime = micros();
	
	totalTime = allEndTime - allStartTime;
	
	aSerial->print("PerfMon start:");
	aSerial->print(allStartTime/1000);
	aSerial->print(" (mils)   end:");
	aSerial->print(allEndTime/1000);
	aSerial->print(" (mils)  elapsed:");
	aSerial->print(totalTime/1000);
	aSerial->print(" (mils)");
	aSerial->println();
	aSerial->println("PerfMon:           \tcpu%\tmils\t#called\tHz");
	for( i=0; i<nextFuncNum; i++ )
	{
	    sumOfTime += time[i];
	    hz = numCalls[i]/(totalTime/1000000);
		pct = ((float)time[i] / (float)totalTime) * 100.0;
		padding = PERFMON_FUNCTION_NAME_LENGTH - strLen(functionNames[i]);
		
		aSerial->print(functionNames[i]);
		for(j=0;j<padding;j++)
		    aSerial->print(" ");
		aSerial->print("\t");
		aSerial->print(pct);
		aSerial->print("%\t");
		aSerial->print(time[i]/1000);
		aSerial->print("\t");
		aSerial->print(numCalls[i]);
		aSerial->print("\t");		
		aSerial->print(hz,0);
		aSerial->print("hz");
		aSerial->println();
	}
	// display unexplained time
	if( sumOfTime >= totalTime )
	    unExplainedTime = 0;
	else
	    unExplainedTime = totalTime - sumOfTime;
	pct = ((float)unExplainedTime / (float)totalTime) * 100.0;
	aSerial->print("unexplained        \t");
	aSerial->print(pct);
	aSerial->print("%\t");
	aSerial->print(unExplainedTime/1000);
	aSerial->println();

}

int APM_PerfMon::strLen(char* str)
{
    int i = 0;
    while( str[i] != 0 )
	    i++;
    return i;
}