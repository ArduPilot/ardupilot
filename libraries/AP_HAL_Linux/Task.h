#include "Thread.h"

namespace Linux {
class Task : public Thread {
public:
    FUNCTOR_TYPEDEF(task_body_t, uint32_t);

    Task(task_t t_init, task_body_t t_body) :
        Thread(t_init),  // t_init not used from here
        _task_init{t_init},
        _task_body{t_body}
         { }

protected:

    bool _run() override;

private:
    task_t _task_init;
    task_body_t _task_body;

};
}
