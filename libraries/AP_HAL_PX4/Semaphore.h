#include <semaphore.h>
class Semaphore {
    sem_t _sem;
    
public:
    Semaphore(sem_t &semaphore ) : _sem(semaphore)
    {  }
    
    ~Semaphore() { 
	sem_post(&_sem);
    }
    
    int try_wait() { return sem_trywait(&_sem); }
};
