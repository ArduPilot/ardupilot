
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "RCOutput_ZYNQ.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <sys/mman.h>
#include <signal.h>
using namespace Linux;

#define PWM_CHAN_COUNT 8	// FIXME

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
static void catch_sigbus(int sig)
{
    hal.scheduler->panic("RCOutput.cpp:SIGBUS error gernerated\n");
}
void LinuxRCOutput_ZYNQ::init(void* machtnicht)
{
    uint32_t mem_fd;
    signal(SIGBUS,catch_sigbus);
    mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    sharedMem_cmd = (struct pwm_cmd *) mmap(0, 0x1000, PROT_READ|PROT_WRITE, 
                                            MAP_SHARED, mem_fd, RCOUT_ZYNQ_PWM_BASE);
    close(mem_fd);

    // all outputs default to 50Hz, the top level vehicle code
    // overrides this when necessary
    set_freq(0xFFFFFFFF, 50);
}

void LinuxRCOutput_ZYNQ::set_freq(uint32_t chmask, uint16_t freq_hz)            //LSB corresponds to CHAN_1
{
    uint8_t i;
    unsigned long tick=TICK_PER_S/(unsigned long)freq_hz;

    for (i=0;i<PWM_CHAN_COUNT;i++) {
        if (chmask & (1U<<i)) {
            sharedMem_cmd->periodhi[i].period=tick;
        }
    }
}

uint16_t LinuxRCOutput_ZYNQ::get_freq(uint8_t ch)
{
    return TICK_PER_S/sharedMem_cmd->periodhi[ch].period;;
}

void LinuxRCOutput_ZYNQ::enable_ch(uint8_t ch)
{
    // sharedMem_cmd->enmask |= 1U<<chan_pru_map[ch];
}

void LinuxRCOutput_ZYNQ::disable_ch(uint8_t ch)
{
    // sharedMem_cmd->enmask &= !(1U<<chan_pru_map[ch]);
}

void LinuxRCOutput_ZYNQ::write(uint8_t ch, uint16_t period_us)
{
    sharedMem_cmd->periodhi[ch].hi = TICK_PER_US*period_us;
}

void LinuxRCOutput_ZYNQ::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
    uint8_t i;
    if(len>PWM_CHAN_COUNT){
        len = PWM_CHAN_COUNT;
    }
    for(i=0;i<len;i++){
        write(ch+i,period_us[i]);
    }
}

uint16_t LinuxRCOutput_ZYNQ::read(uint8_t ch)
{
    return (sharedMem_cmd->periodhi[ch].hi/TICK_PER_US);
}

void LinuxRCOutput_ZYNQ::read(uint16_t* period_us, uint8_t len)
{
    uint8_t i;
    if(len>PWM_CHAN_COUNT){
        len = PWM_CHAN_COUNT;
    }
    for(i=0;i<len;i++){
        period_us[i] = sharedMem_cmd->periodhi[i].hi/TICK_PER_US;
    }
}

#endif
