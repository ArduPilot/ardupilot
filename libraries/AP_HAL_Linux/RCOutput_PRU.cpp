#include "RCOutput_PRU.h"

#include <dirent.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>

using namespace Linux;

#define PWM_CHAN_COUNT 12

static const uint8_t chan_pru_map[]= {10,8,11,9,7,6,5,4,3,2,1,0};                //chan_pru_map[CHANNEL_NUM] = PRU_REG_R30/31_NUM;

static void catch_sigbus(int sig)
{
    AP_HAL::panic("RCOutput.cpp:SIGBUS error generated");
}
void RCOutput_PRU::init()
{
    uint32_t mem_fd;
    signal(SIGBUS,catch_sigbus);
    mem_fd = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC);
    sharedMem_cmd = (struct pwm_cmd *) mmap(0, 0x1000, PROT_READ|PROT_WRITE,
                                            MAP_SHARED, mem_fd, RCOUT_PRUSS_SHAREDRAM_BASE);
    close(mem_fd);

    // all outputs default to 50Hz, the top level vehicle code
    // overrides this when necessary
    set_freq(0xFFFFFFFF, 50);
}

void RCOutput_PRU::set_freq(uint32_t chmask, uint16_t freq_hz)            //LSB corresponds to CHAN_1
{
    uint8_t i;
    unsigned long tick=TICK_PER_S/(unsigned long)freq_hz;

    for (i=0;i<PWM_CHAN_COUNT;i++) {
        if (chmask & (1U<<i)) {
            sharedMem_cmd->periodhi[chan_pru_map[i]][0]=tick;
        }
    }
}

uint16_t RCOutput_PRU::get_freq(uint8_t ch)
{
    return TICK_PER_S/sharedMem_cmd->periodhi[chan_pru_map[ch]][0];
}

void RCOutput_PRU::enable_ch(uint8_t ch)
{
    sharedMem_cmd->enmask |= 1U<<chan_pru_map[ch];
}

void RCOutput_PRU::disable_ch(uint8_t ch)
{
    sharedMem_cmd->enmask &= !(1U<<chan_pru_map[ch]);
}

void RCOutput_PRU::write(uint8_t ch, uint16_t period_us)
{
    if (corked) {
        pending[ch] = period_us;
        pending_mask |= (1U << ch);
    } else {
        sharedMem_cmd->periodhi[chan_pru_map[ch]][1] = TICK_PER_US*period_us;
    }
}

uint16_t RCOutput_PRU::read(uint8_t ch)
{
    return (sharedMem_cmd->hilo_read[chan_pru_map[ch]][1]/TICK_PER_US);
}

void RCOutput_PRU::read(uint16_t* period_us, uint8_t len)
{
    uint8_t i;
    if(len>PWM_CHAN_COUNT){
        len = PWM_CHAN_COUNT;
    }
    for(i=0;i<len;i++){
        period_us[i] = sharedMem_cmd->hilo_read[chan_pru_map[i]][1]/TICK_PER_US;
    }
}

void RCOutput_PRU::cork(void)
{
    corked = true;
}

void RCOutput_PRU::push(void)
{
    if (!corked) {
        return;
    }
    corked = false;
    for (uint8_t i=0; i<ARRAY_SIZE(pending); i++) {
        if (pending_mask & (1U << i)) {
            write(i, pending[i]);
        }
    }
    pending_mask = 0;
}
