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
#include "RCOutput_AioPRU.h"

#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE
#include "../../Tools/Linux_HAL_Essentials/pru/aiopru/RcAioPRU_BBBLUE_bin.h"
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET
#include "../../Tools/Linux_HAL_Essentials/pru/aiopru/RcAioPRU_POCKET_bin.h"
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET2
#include "../../Tools/Linux_HAL_Essentials/pru/aiopru/RcAioPRU_POCKET2_bin.h"
#else
#include "../../Tools/Linux_HAL_Essentials/pru/aiopru/RcAioPRU_BBBMINI_bin.h"
#endif

using namespace Linux;

static void catch_sigbus(int sig)
{
    AP_HAL::panic("RCOutputAioPRU.cpp:SIGBUS error generated");
}

#ifdef AP_HAL_LINUX_AIOPRU_OPTIMIZATION_LEVEL
#pragma GCC push_options
#pragma GCC optimize("O1")
#endif

void RCOutput_AioPRU::init()
{
   uint32_t mem_fd;
   uint32_t *iram;
   uint32_t *ctrl;

   signal(SIGBUS,catch_sigbus);

   mem_fd = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC);

   pwm = (struct pwm*) mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, RCOUT_PRUSS_RAM_BASE);
   iram = (uint32_t*)mmap(0, 0x2000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, RCOUT_PRUSS_IRAM_BASE);
   ctrl = (uint32_t*)mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, RCOUT_PRUSS_CTRL_BASE);

   close(mem_fd);

   // Reset PRU
   *ctrl = 0;

   // Loading firmware
   // Due for unknow condition in ARM64 + AM62X to load pru code should be used the wordwise method instead the memcpy
   // used in AM335X board to load the PRU code. The memcpy used with ARM64 causes the random length of the PRU code
   // loaded in the PRU causing segment fault.
#ifdef AP_HAL_LINUX_AIOPRU_LOAD_WORDWISE
   const size_t pru_firmware_words = sizeof(PRUcode) / sizeof(uint32_t);
   for (size_t i = 0; i < pru_firmware_words; ++i) {
      iram[i] = PRUcode[i];
   }
#else
   memcpy(iram, PRUcode, sizeof(PRUcode));
#endif

   // Start PRU
   *ctrl |= 2;

   // all outputs default to 50Hz, the top level vehicle code
   // overrides this when necessary
   set_freq(0xFFFFFFFF, 50);
}

void RCOutput_AioPRU::set_freq(uint32_t chmask, uint16_t freq_hz)
{
   uint8_t i;
   uint32_t tick = TICK_PER_S / freq_hz;

   for(i = 0; i < PWM_CHAN_COUNT; i++) {
      if(chmask & (1U << i)) {
         pwm->channel[i].time_t = tick;
      }
   }
}

uint16_t RCOutput_AioPRU::get_freq(uint8_t ch)
{
   uint16_t ret = 0;

   if(ch < PWM_CHAN_COUNT) {
      ret = TICK_PER_S / pwm->channel[ch].time_t;
   }

   return ret;
}

void RCOutput_AioPRU::enable_ch(uint8_t ch)
{
   if(ch < PWM_CHAN_COUNT) {
      pwm->channelenable |= 1U << ch;
   }
}

void RCOutput_AioPRU::disable_ch(uint8_t ch)
{
   if(ch < PWM_CHAN_COUNT) {
      pwm->channelenable &= !(1U << ch);
   }
}

void RCOutput_AioPRU::write(uint8_t ch, uint16_t period_us)
{
   if(ch < PWM_CHAN_COUNT) {
       if (corked) {
           pending_mask |= (1U << ch);
           pending[ch] = period_us;
       } else {
           pwm->channel[ch].time_high = TICK_PER_US * period_us;
       }
   }
}

uint16_t RCOutput_AioPRU::read(uint8_t ch)
{
   uint16_t ret = 0;

   if(ch < PWM_CHAN_COUNT) {
      ret = (pwm->channel[ch].time_high / TICK_PER_US);
   }

   return ret;
}

void RCOutput_AioPRU::read(uint16_t* period_us, uint8_t len)
{
   uint8_t i;

   if(len > PWM_CHAN_COUNT) {
      len = PWM_CHAN_COUNT;
   }

   for(i = 0; i < len; i++) {
      period_us[i] = pwm->channel[i].time_high / TICK_PER_US;
   }
}

void RCOutput_AioPRU::cork(void)
{
    corked = true;
}

void RCOutput_AioPRU::push(void)
{
    if (!corked) {
        return;
    }
    corked = false;
    for (uint8_t i=0; i<PWM_CHAN_COUNT; i++) {
        if (pending_mask & (1U<<i)) {
            write(i, pending[i]);
        }
    }
    pending_mask = 0;
}
