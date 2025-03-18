#include "Thread.h"

#include <AP_HAL/Scheduler.h>

namespace Linux {
class Task : public Thread {
public:

    Task(AP_HAL::Scheduler::Task &new_task) :
        Thread(nullptr),  // this passed-in task_t not called
        task{new_task}
         { }

protected:

    bool _run() override;

private:
    AP_HAL::Scheduler::Task &task;

};
}
