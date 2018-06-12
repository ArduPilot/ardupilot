/*
    (c) 2017 night_ghost@ykoctpa.ru
 
*/


#include <AP_HAL/AP_HAL.h>
#include "Semaphores.h"
#include "Scheduler.h"
#include "Util.h"

using namespace F4Light;

void *Util::malloc_type(size_t size, Memory_Type mem_type) { 
    void *ptr;
    switch(mem_type) {
    case AP_HAL::Util::MEM_FAST:
        ptr = sbrk_ccm(size);
        if(ptr != ((caddr_t)-1)) {
            memset(ptr,0,size); 
            return ptr;
        }
        // no break!
    case AP_HAL::Util::MEM_DMA_SAFE:
        ptr = malloc(size);
        if(ptr != NULL) {
            memset(ptr,0,size); 
            return ptr;
        }
        // no break!
        
    default:
        return NULL;
    }        
}
 
void  Util::free_type(void *ptr, size_t size, Memory_Type mem_type) { 
    switch(mem_type) {
    case AP_HAL::Util::MEM_DMA_SAFE:
        free(ptr); 
        break;

    case AP_HAL::Util::MEM_FAST:  // can't free CCM memory
        break;
    }

}