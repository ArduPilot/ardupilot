
#include <AP_HAL/AP_HAL.h>

#include "RCOutput_ZYNQ.h"

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

using namespace Linux;

#define PWM_CHAN_COUNT 8
#define RCOUT_ZYNQ_PWM_BASE	 0x43c00000
#define PWM_CMD_CONFIG	         0	/* full configuration in one go */
#define PWM_CMD_ENABLE	         1	/* enable a pwm */
#define PWM_CMD_DISABLE	         2	/* disable a pwm */
#define PWM_CMD_MODIFY	         3	/* modify a pwm */
#define PWM_CMD_SET	         4	/* set a pwm output explicitly */
#define PWM_CMD_CLR	         5	/* clr a pwm output explicitly */
#define PWM_CMD_TEST	         6	/* various crap */


static void catch_sigbus(int sig)
{
    AP_HAL::panic("RCOutput.cpp:SIGBUS error generated\n");
}
void RCOutput_ZYNQ::init()
{
    uint32_t mem_fd;
    signal(SIGBUS,catch_sigbus);
    mem_fd = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC);
    sharedMem_cmd = (struct pwm_cmd *) mmap(0, 0x1000, PROT_READ|PROT_WRITE, 
                                            MAP_SHARED, mem_fd, RCOUT_ZYNQ_PWM_BASE);
    close(mem_fd);

    // all outputs default to 50Hz, the top level vehicle code
    // overrides this when necessary
    set_freq(0xFFFFFFFF, 50);
}

void RCOutput_ZYNQ::set_freq(uint32_t chmask, uint16_t freq_hz)            //LSB corresponds to CHAN_1
{
    uint8_t i;
    unsigned long tick=TICK_PER_S/(unsigned long)freq_hz;

    for (i=0;i<PWM_CHAN_COUNT;i++) {
        if (chmask & (1U<<i)) {
            sharedMem_cmd->periodhi[i].period=tick;
        }
    }
}

uint16_t RCOutput_ZYNQ::get_freq(uint8_t ch)
{
    if (ch >= PWM_CHAN_COUNT) {
        return 0;
    }

    return TICK_PER_S/sharedMem_cmd->periodhi[ch].period;
}

void RCOutput_ZYNQ::enable_ch(uint8_t ch)
{
    // sharedMem_cmd->enmask |= 1U<<chan_pru_map[ch];
}

void RCOutput_ZYNQ::disable_ch(uint8_t ch)
{
    // sharedMem_cmd->enmask &= !(1U<<chan_pru_map[ch]);
}

void RCOutput_ZYNQ::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= PWM_CHAN_COUNT) {
        return;
    }

    if (corked) {
        pending[ch] = period_us;
        pending_mask |= (1U << ch);
    } else {
        sharedMem_cmd->periodhi[ch].hi = TICK_PER_US*period_us;
    }
}

uint16_t RCOutput_ZYNQ::read(uint8_t ch)
{
    if (ch >= PWM_CHAN_COUNT) {
        return 0;
    }

    return sharedMem_cmd->periodhi[ch].hi/TICK_PER_US;
}

void RCOutput_ZYNQ::read(uint16_t* period_us, uint8_t len)
{
    uint8_t i;
    if(len>PWM_CHAN_COUNT){
        len = PWM_CHAN_COUNT;
    }
    for(i=0;i<len;i++){
        period_us[i] = sharedMem_cmd->periodhi[i].hi/TICK_PER_US;
    }
}

void RCOutput_ZYNQ::cork(void)
{
    corked = true;
}

void RCOutput_ZYNQ::push(void)
{
    if (!corked) {
        return;
    }
    corked = false;
    for (uint8_t i=0; i<MAX_ZYNQ_PWMS; i++) {
        if (pending_mask & (1U << i)) {
            write(i, pending[i]);
        }
    }
    pending_mask = 0;
}
