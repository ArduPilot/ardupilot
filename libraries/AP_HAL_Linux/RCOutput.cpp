
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_ERLE

#include "RCOutput.h"

using namespace std;

#define PWM_DIR "/sys/class/pwm"
#define PWM_DIR_EXPORT "/sys/class/pwm/export"
#define PWM_CHAN_COUNT 12

int chan_pru_map[]= {10,8,11,9,7,6,5,4,3,2,1,0};                //chan_pru_map[CHANNEL_NUM] = PRU_REG_R30/31_NUM;
int pru_chan_map[]= {11,10,9,8,7,6,5,4,1,3,0,2};                //pru_chan_map[PRU_REG_R30/31_NUM] = CHANNEL_NUM;

void LinuxRCOutput::init()
{
    DIR* _pwm_dir = opendir(PWM_DIR);
    int _pwm_fd;
    char chan[2];
    struct dirent *dir;
    int pru_load_success=0;
    int exported_count=0;
    int i,pru_r30;

     if (_pwm_dir){
            while ((dir = readdir(_pwm_dir)) != NULL){
            if(strcmp(dir->d_name,"pwmchip0")==0){
                pru_load_success = 1;
                closedir(_pwm_dir);
                break;
            }
        }
        if(!pru_load_success){
            perror("PRU PWM LOAD failed: PRU might not be enabled");
        }

    }
    else{
        perror("PWM DIR not found");
    }
    
    _pwm_fd = open(PWM_DIR_EXPORT,O_WRONLY);

    if(_pwm_fd < 0){
         perror("PWM_EXPORT not found");
    }

    for(i=0;i<PWM_CHAN_COUNT;i++){
        sprintf(chan, "%d",chan_pru_map[i]);
        write(_pwm_fd,&chan,sizeof(chan));
    }

    close(_pwm_fd);
    _pwm_dir = opendir(PWM_DIR);

    while ((dir = readdir(_pwm_dir)) != NULL){
        pru_r30=strtol(dir->d_name+3,NULL,10);
        if(pru_r30>0 && pru_r30<12){
            write1(pru_chan_map[pru_r30],10);                //writing absolute minimum to duty_ns necessary for pwm_sysfs to start working 
            //printf("CHANNEL %d",pru_chan_map[pru_r30]);
            //printf(" exported\n");
            exported_count++;
        }
        else if(strcmp(dir->d_name,"pwm0")==0){
            write1(pru_chan_map[pru_r30],10);
            //printf("CHANNEL %d",pru_chan_map[0]);
            //printf(" exported \n");
            exported_count++;
        }
    }
    if(exported_count != PWM_CHAN_COUNT){
        printf("WARNING: All channels not exported\n");
    }
    closedir(_pwm_dir);
}

void LinuxRCOutput::set_freq(uint32_t chmask, uint32_t freq_hz)            //LSB corresponds to CHAN_1
{
    int _fd, ret,i;
    unsigned long period_ns=1000000000/freq_hz;
    char period_sysfs_path[20],buffer[15];
    sprintf(buffer,"%lu",(unsigned long)period_ns);
   
    for(i=0;i<12;i++){
        if(chmask&(1<<i)){
            sprintf(period_sysfs_path,"%s/pwm%d%s",PWM_DIR,chan_pru_map[i],"/period_ns");
               _fd=open(period_sysfs_path,O_WRONLY);
            if(_fd < 0){
                perror("Period/Frequency file open failed");
            }
    
            ret = write(_fd,&buffer,sizeof(buffer));
            if(ret < 0){
                perror("Period/Frequency write failed");
            }
        }
    }
}

uint16_t LinuxRCOutput::get_freq(uint8_t ch) 
{
    int _fd, ret;
    int freq_hz;
    char period_sysfs_path[20],buffer[15];
    sprintf(period_sysfs_path,"%s/pwm%d%s",PWM_DIR,chan_pru_map[ch],"/period_ns");
    
    _fd=open(period_sysfs_path,O_RDONLY);
    if(_fd < 0){
        perror("period_ns: file open failed");
    }

    ret = read(_fd,&buffer,sizeof(buffer));
    if(ret < 0){
        perror("period_ns: read failed");
    }
    freq_hz=1000000000/(unsigned long)strtol(buffer,NULL,10);

    return freq_hz;
}

void LinuxRCOutput::enable_ch(uint8_t ch)
{
    int _fd, ret;
    char run_sysfs_path[20],buffer[]="1";
    sprintf(run_sysfs_path,"%s/pwm%d%s",PWM_DIR,chan_pru_map[ch],"/run");
    
    _fd=open(run_sysfs_path,O_RDONLY);
    if(_fd < 0){
        perror("run: file open failed");
    }

    ret = read(_fd,&buffer,sizeof(buffer));
    if(ret < 0){
        perror("run: read failed");
    }
}

void LinuxRCOutput::disable_ch(uint8_t ch)
{
    int _fd, ret;
    char run_sysfs_path[20],buffer[]="0";
    sprintf(run_sysfs_path,"%s/pwm%d%s",PWM_DIR,chan_pru_map[ch],"/run");
    
    _fd=open(run_sysfs_path,O_WRONLY);
    if(_fd < 0){
        perror("run: file open failed");
    }

    ret = write(_fd,&buffer,sizeof(buffer));
    if(ret < 0){
        perror("run: read failed");
    }
}

void LinuxRCOutput::write(uint8_t ch, uint16_t period_us)
{
    int _fd, ret;
    char duty_sysfs_path[20],buffer[15];
    sprintf(buffer,"%lu",(unsigned long)period_us*1000);
    sprintf(duty_sysfs_path,"%s/pwm%d%s",PWM_DIR,chan_pru_map[ch],"/duty_ns");
    
    _fd=open(duty_sysfs_path,O_WRONLY);
    if(_fd < 0){
        perror("duty_ns: file open failed");
    }

    ret = write(_fd,&buffer,sizeof(buffer));
    if(ret < 0){
        perror("duty_ns: read failed");
    }
}

void LinuxRCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
    int _fd, ret,i;
    char duty_sysfs_path[25],buffer[15];
    for(i=0;i<len;i++){
        sprintf(buffer,"%lu",(unsigned long)period_us[i]*1000);
        sprintf(duty_sysfs_path,"%s/pwm%d%s",PWM_DIR,chan_pru_map[ch+i],"/duty_ns");
    
        _fd=open(duty_sysfs_path,O_WRONLY);
        if(_fd < 0){
            perror("duty_ns: file open failed");
        }

        ret = write(_fd,&buffer,sizeof(buffer));
        if(ret < 0){
            perror("duty_ns: read failed");
        }
        close(_fd);
    }
}

uint16_t LinuxRCOutput::read(uint8_t ch) {
    int _fd, ret;
    char duty_sysfs_path[25],buffer[15];
    
    sprintf(duty_sysfs_path,"%s/pwm%d%s",PWM_DIR,chan_pru_map[ch],"/duty_ns");
    
    _fd=open(duty_sysfs_path,O_RDONLY);
    if(_fd < 0){
        perror("duty_ns: file open failed");
    }

    ret = read(_fd,&buffer,sizeof(buffer));
    if(ret < 0){
        perror("duty_ns: read failed");
    }
    close(_fd);        
    return strtol(buffer,NULL,10)/1000;
    
}

void LinuxRCOutput::read(uint16_t* period_us, uint8_t len)
{
    int _fd, ret ,i;
    char duty_sysfs_path[25],buffer[15];
    for(i=0;i<len;i++){
        sprintf(duty_sysfs_path,"%s/pwm%d%s",PWM_DIR,chan_pru_map[i],"/duty_ns");
    
        _fd=open(duty_sysfs_path,O_RDONLY);
        if(_fd < 0){
            perror("duty_ns: file open failed");
        }

        ret = read(_fd,&buffer,sizeof(buffer));
        if(ret < 0){
            perror("duty_ns: read failed");
        }
        period_us[i]=strtol(buffer,NULL,10)/1000;
        close(_fd);
    }
}

