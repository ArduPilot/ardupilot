#include <stdio.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <stdint.h>
    
#define PRUSS_SHAREDRAM_BASE     0x4a312000
#define NUM_RING_ENTRIES 200

struct ring_buffer {
    volatile uint16_t ring_head;
    volatile uint16_t ring_tail;
    struct __attribute__((__packed__)) {
           uint16_t pin_value;
           uint16_t delta_t;
    } buffer[NUM_RING_ENTRIES];
};

static volatile struct ring_buffer *ring_buffer;
static FILE *logf;


void startup(void)
{
    logf = fopen("/tmp/pintiming.dat", "w");
}

void check_for_rcin(void)
{
    while (ring_buffer->ring_head != ring_buffer->ring_tail) {
        fprintf(logf,"%u %u\n",
                (unsigned int)ring_buffer->buffer[ring_buffer->ring_head].pin_value,
                (unsigned int)ring_buffer->buffer[ring_buffer->ring_head].delta_t);
            ring_buffer->ring_head = (ring_buffer->ring_head + 1) % NUM_RING_ENTRIES;
    }
}

void main(){
	
	int mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    ring_buffer = (volatile struct ring_buffer*) mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, PRUSS_SHAREDRAM_BASE);
    close(mem_fd);
    ring_buffer->ring_head = 0;
    startup();
    while(1){
        check_for_rcin();
    }
}
