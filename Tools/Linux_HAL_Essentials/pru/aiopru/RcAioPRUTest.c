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
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

#include "RcAioPRU_bin.h"

#define NUM_RING_ENTRIES 300
#define RCOUT_PRUSS_RAM_BASE 0x4a302000
#define RCOUT_PRUSS_CTRL_BASE 0x4a324000
#define RCOUT_PRUSS_IRAM_BASE 0x4a338000
#define RCIN_PRUSS_RAM_BASE   0x4a303000

struct ring_buffer {
	volatile uint16_t ring_head;
	volatile uint16_t ring_tail;
	struct {
		uint32_t s1;
		uint32_t s0;
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
	volatile uint32_t max_cycle_time;
};

volatile struct ring_buffer *ring_buffer;
volatile struct pwm *pwm;

int main (void)
{
   unsigned int ret, i, a, s0, s1, min_s0, min_s1, max_s0, max_s1;
   uint32_t mem_fd;
   uint32_t *iram;
   uint32_t *ctrl;

   mem_fd = open("/dev/mem", O_RDWR|O_SYNC);

   ring_buffer = (struct ring_buffer*) mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, RCIN_PRUSS_RAM_BASE);
   pwm = (struct pwm*) mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, RCOUT_PRUSS_RAM_BASE);
   iram = (uint32_t*)mmap(0, 0x2000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, RCOUT_PRUSS_IRAM_BASE);
   ctrl = (uint32_t*)mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, RCOUT_PRUSS_CTRL_BASE);

   close(mem_fd);

   // Reset PRU 1
   *ctrl = 0;
   usleep(5 * 1000);

   // Load firmware
   for(i = 0; i < ARRAY_SIZE(PRUcode); i++) {
      *(iram + i) = PRUcode[i];
   }

   usleep(5 * 1000);
   // Start PRU 1
   *ctrl = 3;

   i = 1;
   a = 0;
   min_s0 = 0xffffffff;
   min_s1 = 0xffffffff;
   max_s0  = 0;
   max_s1  = 0;

   pwm->ch1_t_time = 200 * 2200;
   pwm->ch2_t_time = 200 * 2300;
   pwm->ch3_t_time = 200 * 2400;
   pwm->ch4_t_time = 200 * 2500;
   pwm->ch5_t_time = 200 * 2600;
   pwm->ch6_t_time = 200 * 2700;
   pwm->ch7_t_time = 200 * 2800;
   pwm->ch8_t_time = 200 * 2900;
   pwm->ch9_t_time = 200 * 3000;
   pwm->ch10_t_time = 200 * 3100;
   pwm->ch11_t_time = 200 * 3200;
   pwm->ch12_t_time = 200 * 3300;
   pwm->enable=0xffffffff;

   while(1) {
      for(a = 0; a < NUM_RING_ENTRIES; a++) {
         s0 = ring_buffer->buffer[a].s0;
         s1 = ring_buffer->buffer[a].s1;
         if((s0 > max_s0) && (s0 != 1001)) {max_s0 = s0;}
         if((s1 > max_s1) && (s1 != 1001)) {max_s1 = s1;}
         if((s0 < min_s0) && (s0 != 1001)) {min_s0 = s0;}
         if((s1 < min_s1) && (s1 != 1001)) {min_s1 = s1;}
      }
      s0 = ring_buffer->buffer[ring_buffer->ring_tail].s0;
      s1 = ring_buffer->buffer[ring_buffer->ring_tail].s1;
      printf("max ct: %u ns  head: %u  tail: %3u   s0: %7u  s1: %7u  s01: %7u  jitter_s0 %u ns  jitter_s1 %u ns\n", pwm->max_cycle_time * 5, ring_buffer->ring_head, ring_buffer->ring_tail, s0/200, s1/200, (s0+s1)/200, ((max_s0-min_s0) * 5), ((max_s1-min_s1) * 5));
//      pwm->ch1_hi_time = (uint32_t)((rand() % 1001 + 900) * 200);
      pwm->ch2_hi_time = (uint32_t)((rand() % 1001 + 900) * 200);
      pwm->ch1_hi_time = 1000 * 200;
//      pwm->ch2_hi_time = 1500 * 200;
      pwm->ch3_hi_time = (uint32_t)((rand() % 1001 + 900) * 200);
      pwm->ch4_hi_time = (uint32_t)((rand() % 1001 + 900) * 200);
      pwm->ch5_hi_time = (uint32_t)((rand() % 1001 + 900) * 200);
      pwm->ch6_hi_time = (uint32_t)((rand() % 1001 + 900) * 200);
      pwm->ch7_hi_time = (uint32_t)((rand() % 1001 + 900) * 200);
      pwm->ch8_hi_time = (uint32_t)((rand() % 1001 + 900) * 200);
      pwm->ch9_hi_time = (uint32_t)((rand() % 1001 + 900) * 200);
      pwm->ch10_hi_time = (uint32_t)((rand() % 1001 + 900) * 200);
      pwm->ch11_hi_time = (uint32_t)((rand() % 1001 + 900) * 200);
      pwm->ch12_hi_time = (uint32_t)((rand() % 1001 + 900) * 200);

      usleep(50 * 1000);

      if(i < 100) {
         min_s0 = 0xffffffff;
         min_s1 = 0xffffffff;
         max_s0  = 0;
         max_s1  = 0;
         i++;
      }
      else if(((max_s1 - min_s1) * 5) > 1001000) {
         i = 500;
      }
   }
   return 0;
}
