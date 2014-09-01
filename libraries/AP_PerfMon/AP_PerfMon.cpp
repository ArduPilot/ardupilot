#include "AP_PerfMon.h"

extern const AP_HAL::HAL& hal;

// static class variable definitions
uint8_t AP_PerfMon::nextFuncNum;
char AP_PerfMon::functionNames[PERFMON_MAX_FUNCTIONS][PERFMON_FUNCTION_NAME_LENGTH];
uint32_t AP_PerfMon::time[PERFMON_MAX_FUNCTIONS];
uint32_t AP_PerfMon::maxTime[PERFMON_MAX_FUNCTIONS];
uint32_t AP_PerfMon::numCalls[PERFMON_MAX_FUNCTIONS];
uint32_t AP_PerfMon::allStartTime;
uint32_t AP_PerfMon::allEndTime;
AP_PerfMon* AP_PerfMon::lastCreated = NULL;
bool AP_PerfMon::_enabled = true;

// constructor
AP_PerfMon::AP_PerfMon(uint8_t funcNum) : _funcNum(funcNum), _time_this_iteration(0)
{
    // exit immediately if we are disabled
    if( !_enabled ) {
        return;
    }

    // check global start time
    if( allStartTime == 0 ) {
        allStartTime = hal.scheduler->micros();
    }

    // stop recording time from parent
    _parent = lastCreated;  // add pointer to parent
	if( _parent != NULL ) {
        _parent->stop();
    }

    // record myself as the last created instance
    lastCreated = this;

    numCalls[_funcNum]++;   // record that this function has been called
    start();  				// start recording time spent in this function
}

// destructor
AP_PerfMon::~AP_PerfMon()
{
    // exit immediately if we are disabled
    if( !_enabled ) {
        return;
    }

    stop();   // stop recording time spent in this function
    lastCreated = _parent;  // make my parent the last created instance

    // calculate max time spent in this function
    if( _time_this_iteration > maxTime[_funcNum] ) {
        maxTime[_funcNum] = _time_this_iteration;
    }

    // restart recording time for parent
    if( _parent != NULL )
        _parent->start();
}

// record function name in static list
uint8_t AP_PerfMon::recordFunctionName(const char funcName[])
{
    uint8_t nextNum = nextFuncNum++;
    uint8_t i;

    // clear existing function name (if any)
    functionNames[nextNum][0] = 0;

    // store function name
    for( i=0; i<PERFMON_FUNCTION_NAME_LENGTH-1 && funcName[i] != 0; i++ ) {
        functionNames[nextNum][i] = funcName[i];
    }
    functionNames[nextNum][i] = 0;

    return nextNum;
}

// stop recording time
void AP_PerfMon::start()
{
    _startTime = hal.scheduler->micros();  // start recording time spent in this function
}

// stop recording time
void AP_PerfMon::stop()
{
    uint32_t temp_time = hal.scheduler->micros()-_startTime;
    _time_this_iteration += temp_time;
    time[_funcNum] += temp_time;
}

// ClearAll - clears all data from static members
void AP_PerfMon::ClearAll()
{
    uint8_t i;
    AP_PerfMon *p = lastCreated;

    for(i=0; i<PERFMON_MAX_FUNCTIONS; i++) {
        time[i] = 0;        // reset times
        numCalls[i] = 0;    // reset num times called
        maxTime[i] = 0;     // reset maximum time
    }

    // reset start time to now
    allStartTime = hal.scheduler->micros();
    allEndTime = 0;

    // reset start times of any active counters
    while( p != NULL ) {
        p->_startTime = allStartTime;
        p = p->_parent;
    }
}

// DisplayResults - displays table of timing results
void AP_PerfMon::DisplayResults()
{
    uint8_t i,j,changed;
    float hz;
    float pct;
    uint32_t totalTime;
    uint32_t avgTime;
    uint32_t sumOfTime = 0;
    uint32_t unExplainedTime;
    uint8_t order[PERFMON_MAX_FUNCTIONS];

    // record end time
    if( allEndTime == 0 ) {
        allEndTime = hal.scheduler->micros();
    }

    // turn off any time recording
    if( lastCreated != NULL ) {
        lastCreated->stop();
    }
    _enabled = false;

    // reorder results
    for(i=0; i<nextFuncNum; i++) {
        order[i] = i;
    }
    changed=0;
    do{
        changed = 0;
        for(i=0; i<nextFuncNum-1; i++)
            if(time[order[i]]<time[order[i+1]]) {
                j = order[i];
                order[i] = order[i+1];
                order[i+1] = j;
                changed = 1;
            }
	}while(changed != 0);

    // calculate time elapsed
    totalTime = allEndTime - allStartTime;

    // ensure serial is blocking
    hal.console->set_blocking_writes(true);

    // print table of results
    hal.console->printf_P(PSTR("\nPerfMon elapsed:%lu(ms)\n"),(unsigned long)totalTime/1000);
    hal.console->printf_P(PSTR("Fn:\t\tcpu\ttot(ms)\tavg(ms)\tmax(ms)\t#calls\tHz\n"));
    for( i=0; i<nextFuncNum; i++ ) {
        j=order[i];
        sumOfTime += time[j];

        // calculate average execution time
        if( numCalls[j] > 0 ) {
            avgTime = time[j] / numCalls[j];
        }else{
            avgTime = 0;
        }

        hz = numCalls[j]/(totalTime/1000000);
        pct = ((float)time[j] / (float)totalTime) * 100.0;
        hal.console->printf_P(PSTR("%-10s\t%4.2f\t%lu\t%4.3f\t%4.3f\t%lu\t%4.1f\n"),
            functionNames[j],
            pct,
            (unsigned long)time[j]/1000,
            (float)avgTime/1000.0,
            (float)maxTime[j]/1000.0,
            numCalls[j],
            hz);
    }
    // display unexplained time
    if( sumOfTime >= totalTime ) {
        unExplainedTime = 0;
    } else {
        unExplainedTime = totalTime - sumOfTime;
    }
    pct = ((float)unExplainedTime / (float)totalTime) * 100.0;
    hal.console->printf_P(PSTR("unexpl:\t\t%4.2f\t%lu\n"),pct,(unsigned long)unExplainedTime/1000);

    // restore to blocking writes if necessary
    hal.console->set_blocking_writes(false);

    // turn back on any time recording
    if( lastCreated != NULL ) {
        lastCreated->start();
    }
    _enabled = true;
}

// DisplayAndClear - will display results after this many milliseconds.  should be called regularly
void AP_PerfMon::DisplayAndClear(uint32_t display_after_seconds)
{
    if( (hal.scheduler->micros() - allStartTime) > (uint32_t)(display_after_seconds * 1000000) ) {
        DisplayResults();
        ClearAll();
    }
}
