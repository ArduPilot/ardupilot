// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

//Comment/uncomment the #includes statements depending on your BeagleBone version:
//#include "RcAioPRU_POCKET_bin.h"
//#include "RcAioPRU_BBBMINI_bin.h"
#include "RcAioPRU_BBBLUE_bin.h"

#define NUM_RING_ENTRIES 300
#define RCOUT_PRUSS_RAM_BASE 0x4a302000
#define RCOUT_PRUSS_CTRL_BASE 0x4a324000
#define RCOUT_PRUSS_IRAM_BASE 0x4a338000
#define RCIN_PRUSS_RAM_BASE   0x4a303000

#define ARRAY_SIZE(_arr) (sizeof(_arr) / sizeof(_arr[0]))

#define PWM_FREQ 50

struct ring_buffer {
	volatile uint16_t ring_head;
	volatile uint16_t ring_tail;
	struct {
		volatile uint32_t s1;
		volatile uint32_t s0;
	} buffer[NUM_RING_ENTRIES];
};

struct pwm {
	volatile uint32_t enable;
	volatile uint32_t ch1_hi_time;
	volatile uint32_t ch1_t_time;
	volatile uint32_t ch2_hi_time;
	volatile uint32_t ch2_t_time;
	volatile uint32_t ch3_hi_time;
	volatile uint32_t ch3_t_time;
	volatile uint32_t ch4_hi_time;
	volatile uint32_t ch4_t_time;
	volatile uint32_t ch5_hi_time;
	volatile uint32_t ch5_t_time;
	volatile uint32_t ch6_hi_time;
	volatile uint32_t ch6_t_time;
	volatile uint32_t ch7_hi_time;
	volatile uint32_t ch7_t_time;
	volatile uint32_t ch8_hi_time;
	volatile uint32_t ch8_t_time;
	volatile uint32_t ch9_hi_time;
	volatile uint32_t ch9_t_time;
	volatile uint32_t ch10_hi_time;
	volatile uint32_t ch10_t_time;
	volatile uint32_t ch11_hi_time;
	volatile uint32_t ch11_t_time;
	volatile uint32_t ch12_hi_time;
	volatile uint32_t ch12_t_time;
	volatile uint32_t time;
	volatile uint32_t max_cycle_time;
};

volatile struct ring_buffer *ring_buffer;
volatile struct pwm *pwm;

static const uint32_t TICK_PER_US = 200;
static const uint32_t TICK_PER_S = 200000000;
static const uint32_t TICK_DURATION_NS = 5;

int main (void)
{
   unsigned int ret, s0, s1, min_s0 = 0xffffffff, min_s1 = 0xffffffff, max_s0 = 0, max_s1 = 0;
   uint32_t mem_fd = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC);
   ring_buffer = (struct ring_buffer*) mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, RCIN_PRUSS_RAM_BASE);
   pwm = (struct pwm*) mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, RCOUT_PRUSS_RAM_BASE);
   uint32_t *iram = (uint32_t*)mmap(0, 0x2000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, RCOUT_PRUSS_IRAM_BASE);
   uint32_t *ctrl = (uint32_t*)mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, RCOUT_PRUSS_CTRL_BASE);
   uint64_t time_ns;
   close(mem_fd);

   // This loop checks that the IEP counter is really started. If not, the PRU is reset, the program is reload and PRU restarted
   // To report pwm->time and pwm->max_cycle_time, the PRU program must be compiled with -DDEBUG option, for example:
   // pasm -V3 -c RcAioPRU.p RcAioPRU_BBBLUE -DBBBLUE -DDEBUG
   // This is made for you by 'make debug' followed by 'make test'
   do {
      printf("The PRU will be reset\n");
      // Reset PRU 1
      *ctrl = 0;
      //You might uncomment this to identify more easily where the program ends in the IRAM
      //memset(iram, '\0', sizeof(PRUcode) + 128);
      // Load firmware
      memcpy(iram, PRUcode, sizeof(PRUcode));
      // Start PRU 1
      *ctrl |= 2;
      pwm->ch1_t_time = TICK_PER_S / PWM_FREQ;
      pwm->ch2_t_time = TICK_PER_S / PWM_FREQ;
      pwm->ch3_t_time = TICK_PER_S / PWM_FREQ;
      pwm->ch4_t_time = TICK_PER_S / PWM_FREQ;
      pwm->ch5_t_time = TICK_PER_S / PWM_FREQ;
      pwm->ch6_t_time = TICK_PER_S / PWM_FREQ;
      pwm->ch7_t_time = TICK_PER_S / PWM_FREQ;
      pwm->ch8_t_time = TICK_PER_S / PWM_FREQ;
      pwm->ch9_t_time = TICK_PER_S / PWM_FREQ;
      pwm->ch10_t_time = TICK_PER_S / PWM_FREQ;
      pwm->ch11_t_time = TICK_PER_S / PWM_FREQ;
      pwm->ch12_t_time = TICK_PER_S / PWM_FREQ;
      pwm->enable=0xffffffff;
      printf("IEP counter: 0x%08x\n", pwm->time);
   } while (pwm->time == 0xffffffff);

   while(1) {
      for(unsigned int a = 0; a < NUM_RING_ENTRIES; a++) {
         s0 = ring_buffer->buffer[a].s0;
         s1 = ring_buffer->buffer[a].s1;
         if(s0 > max_s0) {max_s0 = s0;}
         if(s1 > max_s1) {max_s1 = s1;}
         if(s0 < min_s0) {min_s0 = s0;}
         if(s1 < min_s1) {min_s1 = s1;}
      }
      s0 = ring_buffer->buffer[ring_buffer->ring_tail].s0;
      s1 = ring_buffer->buffer[ring_buffer->ring_tail].s1;
      time_ns = ((double)pwm->time) * ((double)TICK_DURATION_NS);
      printf("max ct: %3u cycles time: %11lluns head: %u tail: %3u s0: %7u s1: %7u s01: %7u jitter_s0: %uns jitter_s1: %uns\n", pwm->max_cycle_time, time_ns, ring_buffer->ring_head, ring_buffer->ring_tail, s0 * TICK_DURATION_NS, s1 * TICK_DURATION_NS, (s0+s1) * TICK_DURATION_NS, ((max_s0-min_s0) * TICK_DURATION_NS), ((max_s1-min_s1) * TICK_DURATION_NS));
      // uint32_t value = (uint32_t)((rand() % 1001 + 900) * TICK_PER_US);
      // pwm->ch1_hi_time = value;
      // pwm->ch2_hi_time = value;
      //pwm->ch1_hi_time = 1500 * TICK_PER_US;
      pwm->ch1_hi_time = (uint32_t)((rand() % 1001 + 900) * TICK_PER_US);
      pwm->ch2_hi_time = 1500 * TICK_PER_US;
      pwm->ch3_hi_time = (uint32_t)((rand() % 1001 + 900) * TICK_PER_US);
      pwm->ch4_hi_time = (uint32_t)((rand() % 1001 + 900) * TICK_PER_US);
      pwm->ch5_hi_time = (uint32_t)((rand() % 1001 + 900) * TICK_PER_US);
      pwm->ch6_hi_time = (uint32_t)((rand() % 1001 + 900) * TICK_PER_US);
      pwm->ch7_hi_time = (uint32_t)((rand() % 1001 + 900) * TICK_PER_US);
      pwm->ch8_hi_time = (uint32_t)((rand() % 1001 + 900) * TICK_PER_US);
      pwm->ch9_hi_time = (uint32_t)((rand() % 1001 + 900) * TICK_PER_US);
      pwm->ch10_hi_time = (uint32_t)((rand() % 1001 + 900) * TICK_PER_US);
      pwm->ch11_hi_time = (uint32_t)((rand() % 1001 + 900) * TICK_PER_US);
      pwm->ch12_hi_time = (uint32_t)((rand() % 1001 + 900) * TICK_PER_US);
      usleep(1000000);
      min_s0 = 0xffffffff;
      min_s1 = 0xffffffff;
      max_s0  = 0;
      max_s1  = 0;
   }
   return 0;
}
