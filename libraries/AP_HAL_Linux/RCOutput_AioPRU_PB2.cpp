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

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET2

#include "RCOutput_AioPRU_PB2.h"
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>
#include <GCS_MAVLink/GCS.h>

#include "../../Tools/Linux_HAL_Essentials/pru/aiopru/RcAioPRU_POCKET2_DShot_bin.h"
#include "../../Tools/Linux_HAL_Essentials/pru/aiopru/RcAioPRU_POCKET2_bin.h"

using namespace Linux;
using namespace AP_HAL;

extern const AP_HAL::HAL& hal;

#ifdef AP_HAL_LINUX_AIOPRU_OPTIMIZATION_LEVEL
#pragma GCC push_options
#pragma GCC optimize("O1")
#endif

bool RCOutput_AioPRU_PB2::is_dshot_mode(enum RCOutput::output_mode mode)
{
    switch (mode) {
    case RCOutput::MODE_PWM_DSHOT150:
    case RCOutput::MODE_PWM_DSHOT300:
    case RCOutput::MODE_PWM_DSHOT600:
    case RCOutput::MODE_PWM_DSHOT1200:
        return true;
    default:
        return false;
    }
}

/*
  create a DSHOT 16 bit packet. Based on prepareDshotPacket from betaflight
  This code should be shared with the ChibiOS version.
 */
uint16_t RCOutput_AioPRU_PB2::create_dshot_packet(const uint16_t value, bool telem_request)
{
   uint16_t packet = (value << 1);

   if (telem_request) {
      // We don't actually support doing anything with these.
      packet |= 1;
   }

   // compute checksum
   uint16_t csum = 0;
   uint16_t csum_data = packet;
   for (uint8_t i = 0; i < 3; i++) {
      csum ^= csum_data;
      csum_data >>= 4;
   }
   csum &= 0xf;
   // append checksum
   packet = (packet << 4) | csum;

   return packet;
}

static void catch_sigbus(int sig)
{
   AP_HAL::panic("RCOutputAioPRU.cpp:SIGBUS error generated\n");
}

void RCOutput_AioPRU_PB2::init()
{
   uint32_t mem_fd;

   signal(SIGBUS,catch_sigbus);

   mem_fd = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC);

   rcoutput = (struct rcoutput*) mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, RCOUT_PB2_PRUSS_RAM_BASE);
   iram = (uint32_t*)mmap(0, 0x2000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, RCOUT_PB2_PRUSS_IRAM_BASE);
   ctrl = (uint32_t*)mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, RCOUT_PB2_PRUSS_CTRL_BASE);

   close(mem_fd);
   // Reset PRU
   *ctrl = 0;

   hal.scheduler->register_timer_process(FUNCTOR_BIND(this, &RCOutput_AioPRU_PB2::timer_tick, void));
}

void RCOutput_AioPRU_PB2::set_freq(uint32_t chmask, uint16_t freq_hz)
{
   if(is_dshot_mode(_mode))
      return;

   uint8_t i;
   uint32_t tick = TICK_PER_S / freq_hz;

   for(i = 0; i < RC_CHAN_COUNT; i++) {
      if(chmask & (1U << i)) {
         rcoutput->pwm_out[i].time_t = tick;
      }
   }
}

uint16_t RCOutput_AioPRU_PB2::get_freq(uint8_t ch)
{
   if(is_dshot_mode(_mode))
      return 600;

   uint16_t ret = 0;

   if(ch < RC_CHAN_COUNT) {
      ret = TICK_PER_S /rcoutput->pwm_out[ch].time_t;
   }

   return ret;
}

void RCOutput_AioPRU_PB2::enable_ch(uint8_t ch)
{
   if(ch < RC_CHAN_COUNT) {
      rcoutput->channelenable |= 1U << ch;
   }
}

void RCOutput_AioPRU_PB2::disable_ch(uint8_t ch)
{
   if(ch < RC_CHAN_COUNT) {
      rcoutput->channelenable &= !(1U << ch);
   }
}

void RCOutput_AioPRU_PB2::write(uint8_t ch, uint16_t period_us)
{
   pending_mask |= (1U << ch);
   period[ch] = period_us;

   if (!corked) {
      push_internal();
   }
}

uint16_t RCOutput_AioPRU_PB2::read(uint8_t ch)
{
   uint16_t ret = 0;
   if (ch < RC_CHAN_COUNT) {
      ret = period[ch];
   }
   return ret;
}

void RCOutput_AioPRU_PB2::read(uint16_t* period_us, uint8_t len)
{
   if(len > (RC_CHAN_COUNT)) {
      len = (RC_CHAN_COUNT);
   }
   memcpy(period_us, period, sizeof(uint16_t) * len);
}

void RCOutput_AioPRU_PB2::cork(void)
{
    corked = true;
}

void RCOutput_AioPRU_PB2::push(void)
{
    if (!corked) {
        return;
    }
    corked = false;

    push_internal();
}

void RCOutput_AioPRU_PB2::set_output_mode(uint32_t mask, const enum output_mode mode)
{
   if(is_dshot_protocol(mode))
   {
      // Loading firmware
      // Due for unknow condition in ARM64 + AM62X to load pru code should be used the wordwise method instead the memcpy
      // used in AM335X board to load the PRU code. The memcpy used with ARM64 causes the random length of the PRU code
      // loaded in the PRU causing segment fault.
      gcs().send_text(MAV_SEVERITY_INFO, "ESC Mode: DShot 600 - %d", mode);
#ifdef AP_HAL_LINUX_AIOPRU_LOAD_WORDWISE
         const size_t pru_firmware_words = sizeof(PRUcode_DShot) / sizeof(uint32_t);
         for (size_t i = 0; i < pru_firmware_words; ++i) {
            iram[i] = PRUcode_DShot[i];
         }
#else
         memcpy(iram, PRUcode_DShot, sizeof(PRUcode_DShot));
#endif
      _mode = output_mode::MODE_PWM_DSHOT600;
   } else {
      gcs().send_text(MAV_SEVERITY_INFO, "ESC Mode: PWM");
#ifdef AP_HAL_LINUX_AIOPRU_LOAD_WORDWISE
      const size_t pru_firmware_words = sizeof(PRUcode) / sizeof(uint32_t);
      for (size_t i = 0; i < pru_firmware_words; ++i) {
         iram[i] = PRUcode[i];
      }
#else
      memcpy(iram, PRUcode, sizeof(PRUcode));
#endif
      _mode = output_mode::MODE_PWM_NORMAL;
   }

   // Starting the PRU
   *ctrl |= 2;
  
}

void RCOutput_AioPRU_PB2::timer_tick()
{
   //Setting failsafe
   rcoutput->failsafe = 0xffffffff;
}

void RCOutput_AioPRU_PB2::push_internal()
{
   if(!is_dshot_mode(_mode)) {
      for (uint8_t i = 0; i < (RC_CHAN_COUNT); i++) {
         if (pending_mask & (1U << i)) {
            rcoutput->pwm_out[i].time_high = TICK_PER_US * period[i];
         }
      }
      return;
   } else {
      for (uint8_t i = 0; i < (RC_CHAN_COUNT); i++) {
         if (pending_mask & (1U << i)) {
            uint16_t pwm = period[i];
            pwm = constrain_int16(pwm, 1000, 2000);
            uint16_t value = 2 * (pwm - 1000);
            if (value != 0) {
               // dshot values are from 48 to 2047. Zero means off.
               value += 47;
            }
            rcoutput->dshot_out[i].ch_frame = create_dshot_packet(value, true);
            rcoutput->dshot_out[i].bitpos = 0;
         }
      }
   }
   pending_mask = 0;
}
#endif //HAL_BOARD_SUBTYPE_LINUX_POCKET2