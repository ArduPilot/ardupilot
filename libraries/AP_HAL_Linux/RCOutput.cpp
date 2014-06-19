
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_ERLE

#include "RCOutput.h"
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
using namespace Linux;


#define PWM_CHAN_COUNT 12

int chan_pru_map[]= {10,8,11,9,7,6,5,4,3,2,1,0};                //chan_pru_map[CHANNEL_NUM] = PRU_REG_R30/31_NUM;
int pru_chan_map[]= {11,10,9,8,7,6,5,4,1,3,0,2};                //pru_chan_map[PRU_REG_R30/31_NUM] = CHANNEL_NUM;

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

void LinuxRCOutput::init(void* machtnicht)
{
    int mem_fd;
    mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    sharedMem_cmd = (struct pwm_cmd *) mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, PRUSS_SHAREDRAM_BASE);
    close(mem_fd);
    sharedMem_cmd->cmd = PWM_CMD_CONFIG;
    sharedMem_cmd->u.cfg.enmask = 0xFFF;
    for(int i=0;i<PWM_CHAN_COUNT;i++){
        sharedMem_cmd->u.cfg.hilo[i][0] = TICK_PER_US*1000;
        sharedMem_cmd->u.cfg.hilo[i][1] = (TICK_PER_S/490)-TICK_PER_US*1000;
    }
    sharedMem_cmd->magic = PWM_CMD_MAGIC;
}

void LinuxRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)            //LSB corresponds to CHAN_1
{
    int i;
    unsigned long tick=200000000/(unsigned long)freq_hz;
    for(i=0;i<12;i++){
        if(chmask&(1<<i)){
            period[chan_pru_map[i]]=tick;
        }
    }
}

uint16_t LinuxRCOutput::get_freq(uint8_t ch)
{
    return TICK_PER_S/period[chan_pru_map[ch]];
}

void LinuxRCOutput::enable_ch(uint8_t ch)
{
    int i;
    while(sharedMem_cmd->magic != PWM_REPLY_MAGIC && i < 5){
        usleep(2);
        i++;
    }
    if(i == 5){
        hal.console->println("RCOutput: PWM Write Failed!");
        return;
    }

    sharedMem_cmd->cmd = PWM_CMD_ENABLE;
    sharedMem_cmd->pwm_nr = chan_pru_map[ch];
    sharedMem_cmd->magic = PWM_CMD_MAGIC;
}

void LinuxRCOutput::disable_ch(uint8_t ch)
{
    int i;
    while(sharedMem_cmd->magic != PWM_REPLY_MAGIC && i < 5){
        usleep(2);
        i++;
    }
    if(i == 5){
        hal.console->println("RCOutput: PWM Write Failed!");
        return;
    }

    sharedMem_cmd->cmd = PWM_CMD_DISABLE;
    sharedMem_cmd->pwm_nr = chan_pru_map[ch];
    sharedMem_cmd->magic = PWM_CMD_MAGIC;
}

void LinuxRCOutput::write(uint8_t ch, uint16_t period_us)
{
    int i;
    pwm_hi[chan_pru_map[ch]]=period_us*TICK_PER_US;

    while(sharedMem_cmd->magic != PWM_REPLY_MAGIC && i < 5){
        usleep(2);
        i++;
    }
    if(i == 5){
        hal.console->println("RCOutput: PWM Write Failed!");
        return;
    }

    sharedMem_cmd->cmd = PWM_CMD_MODIFY;
    sharedMem_cmd->pwm_nr = chan_pru_map[ch];
    sharedMem_cmd->u.hilo[0] = pwm_hi[chan_pru_map[ch]];
    sharedMem_cmd->u.hilo[1] = period[chan_pru_map[ch]] - pwm_hi[chan_pru_map[ch]];
    sharedMem_cmd->magic = PWM_CMD_MAGIC;
}

void LinuxRCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
    int i;
    for(i=0;i<len;i++){
        write(ch+i,period_us[i]);
    }
}

uint16_t LinuxRCOutput::read(uint8_t ch) 
{
    return (pwm_hi[chan_pru_map[ch]]/TICK_PER_US);
}

void LinuxRCOutput::read(uint16_t* period_us, uint8_t len)
{
    int i;
    for(i=0;i<len;i++){
        period_us[i] = pwm_hi[chan_pru_map[i]]/TICK_PER_US;
    }
}

#endif
