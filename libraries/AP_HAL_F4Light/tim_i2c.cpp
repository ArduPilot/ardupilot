/*

    software I2C driver via timer interrupts   (c) Night_Ghost@ykoctpa.ru
    
    each wave divided to 4 points
    
        ___
SCL ___/   \__
p    0 1 2 3

points 0 & 2 are when f_sda is high, these points to set(0) or read (2) data. Covered by 1st switch
points 1 & 3 are when f_sda is low,  these points to change SCL state

state variable is relative to SCL, changed forcely only in point 2
naming: A_0L - Address   bit 0   need to set low  - point3 - will be low at data stage (data point 0)
        R_AH - Read data bit ACK need to set high - point1 - will be high at data stage (data point 2) 

in START/STOP lines are exchanged:


start: f_sda set high so it starts form SCL stage
      ___
SCL      \_
      _
SDA    \___
p      1 2 



stop:
    __   ___
SCL   \_/
           _
SDA x \___/

p   2 0 1 0   -- SDA stage of STOP2 not required
     \ \ \ \- SCL stage of STOP2 - set SDA high
      \ \ \- SDA stage of STOP - set SCL high
       \ \- SCL stage of STOP - set SCL low and SDA low
        \- here we found that STOP required and changed state, and set SDA low
        
        
restart: 
     __   _____
SCL    \_/     \_
        ___
SDA  x /   \_____
p    2 0 1 0 1 0 
      \ \ \ \ \ \- SCL stage of address bit 0
       \ \ \ \ \- SDA stage of restart2, here we do nothing
        \ \ \ \- SCL stage of restart2, here we set SDA low - restart formed
         \ \ \- SDA stage of restart, set SCL high
          \ \- here 1st SCL state of RESTART and we begins restart forming by setting SDA high and SCL low
           \- here we found that restart required 
         
sometimes low state of SCL is twice less than required period but low side is much forcer than upper

it requires 189 interrupts to receive 2 bytes (with adressing and restart) and takes 9673 cycles (plus 24*189=4536 cycles for interrupt itself), 
 or ~75 cycles per one interrupt of 168+168 available (0.5MHz), or ~22% of CPU
 
 
it requires 16326 cycles to receive 6 bytes (with addressing and restart)
*/

#pragma GCC optimize ("O2")

#include <AP_HAL/AP_HAL.h>
#include "tim_i2c.h"
#include <stdio.h>
#include <i2c.h>

// Software I2C driver
// Can be configured to use any suitable pins

using namespace F4Light;

extern const AP_HAL::HAL& hal;

#define SCL_H         {scl_port->BSRRL = scl_pin; }
#define SCL_L         {scl_port->BSRRH = scl_pin; }

#define SDA_H         {sda_port->BSRRL = sda_pin; }
#define SDA_L         {sda_port->BSRRH = sda_pin; }

#define SCL_read      ((scl_port->IDR & scl_pin)!=0)
#define SDA_read      ((sda_port->IDR & sda_pin)!=0)

#define I2C_yield(x)     hal_yield(x)

#define SI2C_BIT_TIME 8


#ifdef SI2C_DEBUG
 Soft_I2C::SI2C_State Soft_I2C::log[SI2C_LOG_SIZE];
 uint16_t             Soft_I2C::log_ptr;
#endif

#ifdef SI2C_PROF
    uint64_t Soft_I2C::full_time IN_CCM;
#endif

static void delay_10us(){
    hal_delay_microseconds(10);
}

Soft_I2C::Soft_I2C() 
: _scl_dev(NULL)
, _sda_dev(NULL)
{ // empty constructor for 1st initialization. real initializations in init_hw()
}


// prepare but don't touch pins
void Soft_I2C::init_hw( const gpio_dev *scl_dev, uint8_t scl_bit, const gpio_dev *sda_dev, uint8_t sda_bit, const timer_dev * tim)
{
    _scl_dev=scl_dev;
    _scl_bit=scl_bit;
    _sda_dev=sda_dev;
    _sda_bit=sda_bit;
    

    sda_port = sda_dev->GPIOx;
    sda_pin  = 1<<sda_bit;

    scl_port = scl_dev->GPIOx;
    scl_pin  = 1<<scl_bit;

    SCL_H; // passive in high state
    SDA_H;

    gpio_set_mode(_scl_dev, _scl_bit, GPIO_OUTPUT_OD_PU);
    gpio_set_mode(_sda_dev, _sda_bit, GPIO_OUTPUT_OD_PU);
        
    gpio_set_speed(_scl_dev, _scl_bit, GPIO_speed_2MHz); // low speed to prevent glitches
    gpio_set_speed(_sda_dev, _sda_bit, GPIO_speed_2MHz);

    
    _timer = tim;
}


// start using
void Soft_I2C::init() { // nothing to do

}


void Soft_I2C::tick() { // ISR

#ifdef SI2C_PROF
    uint32_t t = stopwatch_getticks();
    int_count++;
#endif

    if(wait_scl) {
        if(!SCL_read) return; // wait SCL
        wait_scl = false;
    }
    
    f_sda = !f_sda;

#ifdef SI2C_DEBUG
    SI2C_State &sp = log[log_ptr]; // remember last operation
    
    sp.time  = hal_micros();
    sp.state = state;
    sp.f_sda = f_sda;
    sp.sda   = SDA_read;

    log_ptr++;
    if(log_ptr>=SI2C_LOG_SIZE) log_ptr=0;
#endif

    if(f_sda) { // time to write/read data
        State s = state;
        state = (State) (state+1);  // change to next state
        
        switch(s){ 
        case RESTART:
            SCL_H;
            break;
        case RESTART2:
            data = (_addr << 1) | I2C_Direction_Receiver;
            state = A_0L;       // will send address at next tick
            break;            

        case START:
            SCL_L;
            break;

        case STOP:
            SCL_H;
            break;
        
        case  A_AH: // ACK for address - read on high level of SCL
            if(SDA_read){ // nack;
                if(was_restart) result = I2C_NO_REGISTER;
                else            result = I2C_NO_DEVICE;
                
                done    = true;
                timer_pause(_timer);
            } else { // ack - will send data
                if(send_len) { // send first
                    data = *send++;
                    --send_len;
                    state=W_0L;
                } else { // just receive, all sent before
                    state=R_0L;
                }
            }
            break;
            
        case  W_AH: // ACK for write byte - read on high level of SCL
            if(SDA_read){ // nack;
                 result = I2C_ERR_WRITE;
                 done    = true;
                 timer_pause(_timer);
            } else {
                if(send_len == 0) { // all data sent
                    if(recv_len) {
                        state=RESTART; // restart to read
                    } else {
                        state=STOP; // last byte sent
                    }
                } else { 
                    data = *send++; 
                    send_len--; 
                    state=W_0L; // beginning of next byte
                }    
            }            
            break;


        case  R_AL: // do ACK for read byte
            *recv++ = data;
            recv_len--;

            if(recv_len) {
                SDA_L; // ACK
            } else {
                SDA_H; // NACK on last byte
            }
            
            break;

        case R_AH: // only after R_AL
            if(recv_len) {
                state=R_0L; // will receive next byte
            } else {
                state = STOP;
            }
            break;            

            
        // send address - change SDA on low SCL
        case    A_0L:
        case    A_1L:
        case    A_2L:
        case    A_3L:
        case    A_4L:
        case    A_5L:
        case    A_6L:
        case    A_7L:
        // byte write
        case    W_0L:
        case    W_1L:
        case    W_2L:
        case    W_3L:
        case    W_4L:
        case    W_5L:
        case    W_6L:
        case    W_7L:
            if (data & 0x80) { SDA_H; }
            else             { SDA_L; }
            data <<=1;
            break;
            
        // byte read  - when SCL is high
        case    R_0H:
        case    R_1H:
        case    R_2H:
        case    R_3H:
        case    R_4H:
        case    R_5H:
        case    R_6H:
        case    R_7H:
            data <<= 1;
            if (SDA_read) data |= 0x01;
            break;
        
        default: 
            break;
        }
#ifdef SI2C_PROF
        full_time += stopwatch_getticks() - t;
#endif
        return; // data part is over
    }
    

    // SCL part
    switch(state){
    default:     // something went wrong
    case DUMMY:
        f_sda=true; // never change state
#ifdef SI2C_PROF
        full_time += stopwatch_getticks() - t;
#endif
        return;
        
    case RESTART:
        SCL_L; 
        delay_ns100(1);
        SDA_H;  //      release SDA
        break;

    case RESTART2:
    case START:  
        SDA_L;
        break;
            
    // address 
    case    A_0L: // high to low - end of state in the middle of byte
    case    A_1L:
    case    A_2L:
    case    A_3L:
    case    A_4L:
    case    A_5L:
    case    A_6L:
    case    A_7L:
    // byte write
    case    W_0L:
    case    W_1L:
    case    W_2L:
    case    W_3L:
    case    W_4L:
    case    W_5L:
    case    W_6L:
    case    W_7L:
    // byte read
    case    R_1L:
    case    R_2L:
    case    R_3L:
    case    R_4L:
    case    R_5L:
    case    R_6L:
    case    R_7L:
    case    R_AL: // on read we control SDA line at ATC bit
        SCL_L;
        break;

    case    R_0L: // start of read - give SDA control to slave
        SCL_L;
        delay_ns100(1);
        SDA_H; // release SDA to slave
        break;
    
    case    A_AL: // will be ack bit. on address and write we should give SDA to slave
    case    W_AL:
        SCL_L;
        delay_ns100(1);
        SDA_H; // release SDA to slave
        break;
        
    case    A_0H: // low to high - strobe. just set SCL 
    case    A_1H:
    case    A_2H:
    case    A_3H:
    case    A_4H:
    case    A_5H:
    case    A_6H:
    case    A_7H:
    case    A_AH:

    case    W_0H: 
    case    W_1H:
    case    W_2H:
    case    W_3H:
    case    W_4H:
    case    W_5H:
    case    W_6H:
    case    W_7H:
    case    W_AH:

    case    R_0H: 
    case    R_1H:
    case    R_2H:
    case    R_3H:
    case    R_4H:
    case    R_5H:
    case    R_6H:
    case    R_7H:
    case    R_AH:
        SCL_H;
        if (!SCL_read) wait_scl=true;
        break;
        
    case STOP:
        SCL_L;        
        delay_ns100(1);
        SDA_L; //       prepare to get it high
        break;

    case STOP2:
        SDA_H;
        done = true;
        timer_pause(_timer);
        result = I2C_OK;
        break;
    }

#ifdef SI2C_PROF
    full_time += stopwatch_getticks() - t;
#endif
}

bool Soft_I2C::_start(void)
{
    while(_timer->state->busy)  {
        hal_yield(0);
    }


    SDA_H;            // just in case
    SCL_H;
    
    if (!SCL_read)       return false; // bus busy
    if (!SDA_read)       return false; // bus busy

    state = DUMMY;// to skip interrupt on init
    f_sda = true;  // will be SCL phase

    _timer->state->busy = true;

#define SI2C_PERIOD 2 // time between interrups in uS

// timers are per bus so re-init timer before use
    uint32_t freq = configTimeBase(_timer, 0, 10000);       //10MHz
    Revo_handler h = { .mp = FUNCTOR_BIND_MEMBER(&Soft_I2C::tick, void) };
    timer_attach_interrupt(_timer, TIMER_UPDATE_INTERRUPT, h.h, TIMER_I2C_INT_PRIORITY); // high priority
    timer_set_reload(_timer, SI2C_PERIOD * freq / 1000000);         // period to generate 2uS requests - 500kHz interrupts /4 = 125kHz I2C. 
                                                                    //  I hope that there will be a time between interrupts :)
    bit_time = SI2C_PERIOD*4;

    state  = START;
    result = I2C_OK;
    wait_scl=false;
    f_sda  = true;   // we did START touching sda
    done   = false;
    was_restart = false;

#ifdef SI2C_DEBUG
    memset(log, 0, sizeof(log)); 
    log_ptr=0;
#endif
#ifdef SI2C_PROF
    full_time = 0;
    int_count = 0;
#endif

    data = _addr << 1 | I2C_Direction_Transmitter; // generate address to send

    timer_resume(_timer); // all another in interrupt
    timer_generate_update(_timer);

    return true;
}



uint8_t Soft_I2C::wait_done(){
    uint32_t t = hal_micros();

    uint32_t timeout = SI2C_BIT_TIME*9*(send_len+recv_len+1); // time to full transfer

    while(!done) {
        uint32_t dt = hal_micros() - t;
        
        if(dt > timeout*16) {           // 16 times of full transfer
            timer_pause(_timer);
            if(state==STOP2) break; // all fine
            
            if(SDA_read) { // low SDA first
                if(SCL_read) { // at low SCL
                    SCL_L;
                    hal_delay_microseconds(2);
                }
                SDA_L;
                hal_delay_microseconds(2);
            }
            SCL_H;
            hal_delay_microseconds(2);
            SDA_H;
            if(state>=STOP) break; // data received
            result = I2C_ERR_TIMEOUT;
            break;
        }
        hal_yield(timeout);
        timer_resume(_timer); // just for case
    }

    timer_detach_interrupt(_timer, TIMER_UPDATE_INTERRUPT);

    _timer->state->busy = false;

#if 0 // to set breakpoint on error

    if(result<I2C_ERROR || result == I2C_ERR_TIMEOUT)   return result;
    
    hal_delay_microseconds(2);
    printf("\ni2c error on timer %lx\n",(uint32_t)_timer);
#endif
    return result;    
}

uint8_t  Soft_I2C::writeBuffer( uint8_t addr, uint8_t len, const uint8_t *buf)
{

    recv_len = 0;
    send_len = len;
    send     = buf;
    _addr    = addr;
    
    if (!_start()) {
        return I2C_ERROR; // bus busy
    }

    return wait_done();
}


uint8_t Soft_I2C::read( uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{

    recv_len = len;
    recv     = buf;
    send_len = 1;
    send     = &reg;
    _addr    = addr;
    
    if (!_start()) {
        return I2C_ERROR;
    }

    return wait_done();
}


uint8_t Soft_I2C::transfer(uint8_t  addr, uint8_t  _send_len, const uint8_t *_send, uint8_t len, uint8_t *buf){

    recv_len= len;
    recv    = buf;
    send_len= _send_len;
    send    = _send;
    _addr   = addr;
    
    if (!_start()) {
        return I2C_ERROR;
    }

    return wait_done();
}


#define MAX_I2C_TIME 300 // 300ms before device turn off

bool Soft_I2C::bus_reset(void) {

    uint32_t t=systick_uptime();

again:
    /* Wait for any clock stretching to finish */
    while (!SCL_read) {// device can output 1 so check clock first
        hal_yield(0); // пока ожидаем - пусть другие работают
        
        if(systick_uptime()-t > MAX_I2C_TIME) return false;
    }

    delay_10us();       // 50kHz

    while (!SDA_read) {
        /* Wait for any clock stretching to finish */
        while (!SCL_read) {
            SCL_H; // may be another thread causes LOW
            hal_yield(0); // while we wait - let others work

            if(systick_uptime()-t > MAX_I2C_TIME) return false;
        }
            
        delay_10us();   // 50kHz

        /* Pull low */
        SCL_L;
        delay_10us();

        /* Release high again */
        SCL_H;
        delay_10us();
        SDA_H;
    }

    /* Generate start then stop condition */
    SDA_L;
    delay_10us();
    SCL_L;
    delay_10us();
    SCL_H;
    delay_10us();
    SDA_H;
    
    {
        uint32_t rtime = stopwatch_getticks();
        uint32_t dt    = us_ticks * 50; // 50uS

        while ((stopwatch_getticks() - rtime) < dt) {
            if (!SCL_read)  goto again; // any SCL activity after STOP
        }
    }
    return true;
}

