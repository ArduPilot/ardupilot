/* -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*

(c) 2017 night_ghost@ykoctpa.ru
 
 * I2CDriver.cpp --- AP_HAL_F4Light I2C driver.
 *
 */
#pragma GCC optimize ("O2")

#include <AP_HAL/AP_HAL.h>
#include <AP_Param_Helper/AP_Param_Helper.h>

#include "I2CDevice.h"
#include <i2c.h>

using namespace F4Light;

extern const AP_HAL::HAL& hal;

F4Light::Semaphore I2CDevice::_semaphores[3]; // 2 HW and 1 SW

const timer_dev * I2CDevice::_timers[3] = { // one timer per bus for all devices
    TIMER4, // for bus 0 so not will be used on AirbotV2 boards when it used for PPM_IN
    TIMER10,
    TIMER9,
};

bool I2CDevice::lateInitDone=false;


I2CDevice * I2CDevice::devices[MAX_I2C_DEVICES]; // links to all created devices
uint8_t I2CDevice::dev_count; // number of devices

#ifdef I2C_DEBUG
 I2C_State I2CDevice::log[I2C_LOG_SIZE] IN_CCM;
 uint8_t   I2CDevice::log_ptr=0;
#endif



void I2CDevice::lateInit() {
    lateInitDone=true;
}


I2CDevice::I2CDevice(uint8_t bus, uint8_t address)
        : _bus(bus)
        , _address(address)
        , _retries(5)
        , _lockup_count(0)
        , _initialized(false)
        , _slow(false)
        , _failed(false)
        , need_reset(false)
        , _dev(NULL)
{


    // store link to created devices
    if(dev_count<MAX_I2C_DEVICES){
        devices[dev_count++] = this; // links to all created devices
    }                    
}

I2CDevice::~I2CDevice() { 
    for(int i=0;i<dev_count;i++){
        if(devices[i] == this){
            devices[i] = NULL;
        }
    }
}


void I2CDevice::init(){
    if(!lateInitDone) {
        ((HAL_F4Light&) hal).lateInit();
    }

    if(need_reset) _do_bus_reset();

    if(_failed) return;

    if(_initialized) return;
    
    const i2c_dev *dev=NULL;

    switch(_bus) {
    case 0:         // this is always internal bus
#if defined(I2C1_SDA) && defined(I2C1_SCL) && !defined(BOARD_I2C1_DISABLE)
	    _offs =0;
 #if defined(BOARD_I2C_BUS_SLOW) && BOARD_I2C_BUS_SLOW==0
            _slow=true;
 #endif

 #if defined(BOARD_SOFT_I2C) || defined(BOARD_SOFT_I2C1)
            if(s_i2c==NULL) s_i2c = new Soft_I2C;
            { // isolate p_sda & p_scl
                const stm32_pin_info &p_sda = PIN_MAP[_I2C1->sda_pin];
                const stm32_pin_info &p_scl = PIN_MAP[_I2C1->scl_pin];
                s_i2c->init_hw( 
                    p_scl.gpio_device, p_scl.gpio_bit,
                    p_sda.gpio_device, p_sda.gpio_bit,
                    _timers[_bus]
                );
            }
 #else
	    dev = _I2C1;
 #endif
	    break;
#else
        return;
#endif

    case 1:     // flexi port - I2C2
#if defined(I2C2_SDA) && defined(I2C2_SCL) && !defined( BOARD_I2C2_DISABLE) &&  !defined(BOARD_HAS_UART3) // in this case I2C on FlexiPort will be bus 2

	    _offs = 2;
 #if defined(BOARD_I2C_BUS_SLOW) && BOARD_I2C_BUS_SLOW==1
            _slow=true;
 #endif

 #if defined(BOARD_SOFT_I2C) || defined(BOARD_SOFT_I2C2)
            if(s_i2c==NULL) s_i2c = new Soft_I2C;
            { // isolate p_sda & p_scl
                const stm32_pin_info &p_sda = PIN_MAP[_I2C2->sda_pin];
                const stm32_pin_info &p_scl = PIN_MAP[_I2C2->scl_pin];
                s_i2c->init_hw( 
                    p_scl.gpio_device, p_scl.gpio_bit,
                    p_sda.gpio_device, p_sda.gpio_bit,
                    _timers[_bus]
                );
            }
 #else
	    dev = _I2C2;
 #endif
	    break;
#else
            return; // not initialized so always returns false
#endif

    case 2:         // this bus can use only soft I2C driver
#if defined(BOARD_I2C_BUS_SLOW) && BOARD_I2C_BUS_SLOW==2
            _slow=true;
#endif

#ifdef BOARD_I2C_FLEXI
            if(hal_param_helper->_flexi_i2c){ // move external I2C to flexi port
 #if defined(BOARD_SOFT_I2C) || defined(BOARD_SOFT_I2C3)
                if(s_i2c==NULL) s_i2c = new Soft_I2C;
                { // isolate p_sda & p_scl
                    const stm32_pin_info &p_sda = PIN_MAP[_I2C2->sda_pin];
                    const stm32_pin_info &p_scl = PIN_MAP[_I2C2->scl_pin];
                    s_i2c->init_hw( 
                        p_scl.gpio_device, p_scl.gpio_bit,
                        p_sda.gpio_device, p_sda.gpio_bit,
                        _timers[_bus]
                    );
                }
 #else
                dev = _I2C2;
 #endif
            } else 
#endif
            { //                         external I2C on Input port
#if defined(BOARD_SOFT_SCL) && defined(BOARD_SOFT_SDA)
                if(s_i2c==NULL) s_i2c = new Soft_I2C;
                s_i2c->init_hw( 
                    PIN_MAP[BOARD_SOFT_SCL].gpio_device,     PIN_MAP[BOARD_SOFT_SCL].gpio_bit,
                    PIN_MAP[BOARD_SOFT_SDA].gpio_device,     PIN_MAP[BOARD_SOFT_SDA].gpio_bit,
                    _timers[_bus]
                );
#endif
            }        
            break;

// TODO 
#if defined(I2C3_SDA) && defined(I2C3_SCL)
#endif

    default:
            return;
    }
    _dev = dev; // remember

    
    if(_dev) {
        i2c_init(_dev, _offs, _slow?I2C_250KHz_SPEED:I2C_400KHz_SPEED);

    }else if (s_i2c){
        s_i2c->init( );

        if(_slow) {
            s_i2c->set_low_speed(true);
        }
    }
    else { // neither hardware nor software initalization was successful
        return;
    }
    _initialized=true;
}



void I2CDevice::register_completion_callback(Handler h) { 
    if(h && _completion_cb) {// IOC from last call still not called - some error occured so bus reset needed
        _completion_cb=0;
        _do_bus_reset();
    }
    
    _completion_cb=h;
}
    



bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len)
{

    uint16_t retries=_retries;
    
again:


    uint32_t ret=0;
    uint8_t last_op=0;

    if(!_initialized) {
        init();
        if(!_initialized) return false;
    }


    if(!_dev){ // no hardware so use soft I2C
            
        if(recv_len) memset(recv, 0, recv_len); // for DEBUG
            
        if(recv_len==0){ // only write
            ret=s_i2c->writeBuffer( _address, send_len, send );            
        }else if(send_len==1){ // only read - send byte is address
            ret=s_i2c->read(_address, *send, recv_len, recv);                
        } else {
            ret=s_i2c->transfer(_address, send_len, send, recv_len, recv);
        }
            
        if(ret == I2C_NO_DEVICE) 
            return false;

        if(ret == I2C_OK) 
            return true;

        if((_retries-retries) > 0) { // don't reset and count for fixed at 2nd try errors
            _lockup_count ++;          
            last_error = ret;  
              
            if(!s_i2c->bus_reset()) return false;    
        }

        _dev->state->busy = false;

        if(retries--) goto again;

        return false;
    } // software I2C

// Hardware
#ifdef I2C_DEBUG
    {
     I2C_State &sp = log[log_ptr]; // remember last operation
     sp.st_sr1      = _dev->I2Cx->SR1;
     sp.st_sr2      = _dev->I2Cx->SR2;
     }
#endif

    // 1st wait for bus free
    uint32_t t=Scheduler::_micros();
    while(_dev->state->busy){
        hal_yield(0);
        if(Scheduler::_micros() - t > 5000) {
//            grab_count++;
            break;
        }
    }
    _dev->state->busy=true;

    if(recv_len==0) { // only write
        last_op=1;
        ret = i2c_write(_address, send, send_len);
    } else {
        last_op=0;
        ret = i2c_read( _address, send, send_len, recv, recv_len);
    }


#ifdef I2C_DEBUG
     I2C_State &sp = log[log_ptr]; // remember last operation
     
     sp.time     = Scheduler::_micros();
     sp.bus      =_bus;
     sp.addr     =_address;
     sp.send_len = send_len;
     sp.recv_len = recv_len;
     sp.ret      = ret;
     sp.sr1      = _dev->I2Cx->SR1;
     sp.sr2      = _dev->I2Cx->SR2;
     sp.cr1      = _dev->I2Cx->CR1;
     sp.state    = _state;
     sp.pos      = I2C_FINISH;
     if(log_ptr<I2C_LOG_SIZE-1) log_ptr++;
     else                       log_ptr=0;
#endif

    
    
    if(ret == I2C_PENDING) return true; // transfer with callback

    if(ret == I2C_OK) {
        _dev->state->busy=false;
        return true;
    }
    
// something went wrong and completion callback never will be called, so release bus semaphore
    if(_completion_cb)  {
        _completion_cb = 0;     // to prevent 2nd bus reset
        register_completion_callback((Handler)0);
    }

    if(ret == I2C_ERR_STOP || ret == I2C_STOP_BERR || ret == I2C_STOP_BUSY) { // bus or another errors on Stop, or bus busy after Stop.
                                                                            //   Data is good but bus reset required
        need_reset = true;
        _initialized=false; // will be reinitialized at next transfer

        _dev->I2Cx->CR1 |= I2C_CR1_SWRST; // set for some time
        
        // we not count such errors as _lockup_count
    
        Revo_handler h = { .mp=FUNCTOR_BIND_MEMBER(&I2CDevice::do_bus_reset, void) }; // schedule reset as io_task
        Scheduler::_register_io_process(h.h, IO_ONCE); 
        
        _dev->state->busy=false;
        return true; // data is OK
    } 
    
    if(ret != I2C_NO_DEVICE) { // for all errors except NO_DEVICE do bus reset

        if(ret == I2C_BUS_BUSY) {
            _dev->I2Cx->CR1 |= I2C_CR1_SWRST;           // set SoftReset for some time 
            hal_yield(0);
            _dev->I2Cx->CR1 &= (uint16_t)(~I2C_CR1_SWRST); // clear SoftReset flag            
            _dev->I2Cx->CR1 |= I2C_CR1_PE; // enable
        }

        if((_retries-retries) > 0 || ret==I2C_BUS_ERR){ // not reset bus or log error on 1st try, except ArbitrationLost error
            last_error = ret;   // remember
            last_error_state = _state; // remember to show
            if(last_op) last_error+=50; // to distinguish read and write errors

            _lockup_count ++;  
            _initialized=false; // will be reinitialized at next transfer
        
            _do_bus_reset();
        
            if(_failed) {
                _dev->state->busy=false;
                return false;
            }
        }
    }
    _dev->state->busy=false;

    if(retries--) goto again;

    return false;
}


void I2CDevice::do_bus_reset(){ // public - with semaphores
    if(_semaphores[_bus].take(HAL_SEMAPHORE_BLOCK_FOREVER)){
        _do_bus_reset();
        _semaphores[_bus].give();
    }
}

void I2CDevice::_do_bus_reset(){ // private
    _dev->state->busy=true;
    _dev->I2Cx->CR1 &= (uint16_t)(~I2C_CR1_SWRST); // clear soft reset flag

    if(!need_reset) return; // already done
    
    i2c_deinit(_dev); // disable I2C hardware
    if(!i2c_bus_reset(_dev)) {
        _failed = true;         // can't do it in limited time
    }
    need_reset = false; // done
    _dev->state->busy=false;
}


bool I2CDevice::read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                 uint32_t recv_len, uint8_t times){

    while(times--) {
	bool ret = read_registers(first_reg, recv, recv_len);
	if(!ret) return false;
	recv += recv_len;
    }

    return true;
}


enum I2C_state {
    I2C_want_SB=0,
    I2C_want_ADDR,   // 1
    I2C_want_TXE,    // 2
    I2C_want_RX_SB,  // 3
    I2C_want_RX_ADDR,// 4
    I2C_want_RXNE,   // 5
    I2C_done         // 6
} ;


/*
    moved from low layer to be properly integrated to multitask

*/



/* Send a buffer to the i2c port */
uint32_t I2CDevice::i2c_write(uint8_t addr, const uint8_t *tx_buff, uint8_t len) {

    uint32_t ret = wait_stop_done(true);
    if(ret!=I2C_OK) return ret;

    _addr=addr;
    _tx_buff=tx_buff;
    _tx_len=len;
    _rx_len=0; // only write

            
    i2c_set_isr_handler(_dev, Scheduler::get_handler(FUNCTOR_BIND_MEMBER(&I2CDevice::isr_ev, void)));

    _state = I2C_want_SB;
    _error = I2C_ERR_TIMEOUT;

    // Bus got!  enable Acknowledge for our operation
    _dev->I2Cx->CR1 |= I2C_CR1_ACK; 
    _dev->I2Cx->CR1 &= ~I2C_NACKPosition_Next; 

    // need to wait until  transfer complete 
    uint32_t t = hal_micros();
    uint32_t timeout = i2c_bit_time * 9 * (len+1) * 8 + 100; // time to transfer all data *8 plus 100uS

    _task = Scheduler::get_current_task();// if function called from task - store it and pause

    EnterCriticalSection;
    // Send START condition
    _dev->I2Cx->CR1 |= I2C_CR1_START;
     _dev->I2Cx->CR2 |= I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;    // Enable interrupts

     if(_task) Scheduler::task_pause(timeout);
    LeaveCriticalSection;

    if(_completion_cb) return I2C_PENDING;
        
    while (hal_micros() - t < timeout && _error==I2C_ERR_TIMEOUT) {        
        hal_yield(0);
    }

    if(_error==I2C_ERR_TIMEOUT) finish_transfer();                        

    return _error;    
}

uint32_t I2CDevice::i2c_read(uint8_t addr, const uint8_t *tx_buff, uint8_t txlen, uint8_t *rx_buff, uint8_t rxlen)
{
    uint32_t ret = wait_stop_done(false); // wait for bus release from previous transfer and force it if needed
    if(ret!=I2C_OK) return ret;

    _addr=addr;
    _tx_buff=tx_buff;
    _tx_len=txlen;
    _rx_buff=rx_buff;
    _rx_len=rxlen;

            
    i2c_set_isr_handler(_dev, Scheduler::get_handler(FUNCTOR_BIND_MEMBER(&I2CDevice::isr_ev, void)));

    _state = I2C_want_SB;
    _error = I2C_ERR_TIMEOUT;

    _dev->I2Cx->CR1 &= ~I2C_NACKPosition_Next;  // I2C_NACKPosition_Current
    _dev->I2Cx->CR1 |= I2C_CR1_ACK;             // enable Acknowledge for our operation

    uint32_t t = hal_micros();
    uint32_t timeout = i2c_bit_time * 9 * (txlen+rxlen) * 8 + 100; // time to transfer all data *8 plus 100uS
    _task = Scheduler::get_current_task(); // if function called from task - store it and pause

    EnterCriticalSection;
#ifdef I2C_DEBUG
    {
     I2C_State &sp = log[log_ptr]; // remember last operation
     
     sp.time     = Scheduler::_micros();
     sp.cr1      = _dev->I2Cx->CR1;
     sp.sr1      = _dev->I2Cx->SR1;
     sp.sr2      = _dev->I2Cx->SR2;
     sp.state    = _state;
     sp.pos      = I2C_START;
     if(log_ptr<I2C_LOG_SIZE-1) log_ptr++;
     else                       log_ptr=0;
    }
#endif

    _dev->I2Cx->CR1 |= I2C_CR1_START;    // Send START condition
     if(_task) Scheduler::task_pause(timeout);
     _dev->I2Cx->CR2 |= I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;    // Enable interrupts

#ifdef I2C_DEBUG
     I2C_State &sp = log[log_ptr]; // remember last operation
     
     sp.time     = Scheduler::_micros();
     sp.cr1      = _dev->I2Cx->CR1;
     sp.sr1      = _dev->I2Cx->SR1;
     sp.sr2      = _dev->I2Cx->SR2;
     sp.state    = _state;
     sp.pos      = I2C_START;
     if(log_ptr<I2C_LOG_SIZE-1) log_ptr++;
     else                       log_ptr=0;
#endif
    LeaveCriticalSection;

    if(_completion_cb) return I2C_PENDING;
            
    // need to wait until DMA transfer complete
    while (hal_micros() - t < timeout && _error==I2C_ERR_TIMEOUT) {    
        hal_yield(0);
    }

    if(_error==I2C_ERR_TIMEOUT) finish_transfer();                        

    return _error;
}

void I2CDevice::isr_ev(){
    bool err;

    // get err parameter 
    asm volatile("MOV     %0, r1\n\t"  : "=rm" (err));

    uint32_t sr1itflags = _dev->I2Cx->SR1;
    uint32_t itsources  = _dev->I2Cx->CR2;

    if(err){

        /* I2C Bus error interrupt occurred ----------------------------------------*/
        if(((sr1itflags & I2C_BIT_BERR) != RESET) && ((itsources & I2C_IE_ERR) != RESET)) {    /* Clear BERR flag */
          _dev->I2Cx->SR1 = (uint16_t)(~I2C_BIT_BERR); // Errata 2.4.6
        }
  
        /* I2C Arbitration Loss error interrupt occurred ---------------------------*/
        if(((sr1itflags & I2C_BIT_ARLO) != RESET) && ((itsources & I2C_IE_ERR) != RESET)) {
          _error = I2C_BUS_ERR;
    
          /* Clear ARLO flag */
          _dev->I2Cx->SR1 = (uint16_t)(~I2C_BIT_ARLO); // reset them
        }
  
        /* I2C Acknowledge failure error interrupt occurred ------------------------*/
        if(((sr1itflags & I2C_BIT_AF) != RESET) && ((itsources & I2C_IE_ERR) != RESET))  {
            /* Clear AF flag */
            _dev->I2Cx->SR1 = (uint16_t)(~I2C_BIT_AF); // reset it

            if(_state == I2C_want_ADDR) { // address transfer
                _error = I2C_NO_DEVICE;
            } else if(_state == I2C_want_RX_ADDR) { // restart
                _error = I2C_ERR_REGISTER;
            } else {
                _error = I2C_ERROR;
            }
    
            _dev->I2Cx->CR1 |= I2C_CR1_STOP;          /* Generate Stop */      
        }

#ifdef I2C_DEBUG
     I2C_State &sp = log[log_ptr]; // remember last operation
     
     sp.time     = Scheduler::_micros();
     sp.sr1      = sr1itflags;
     sp.sr2      = _dev->I2Cx->SR2;
     sp.cr1      = _dev->I2Cx->CR1;
     sp.state    = _state;
     sp.pos      = I2C_ERR;
     if(log_ptr<I2C_LOG_SIZE-1) log_ptr++;
     else                       log_ptr=0;
#endif

        if(_error) { // смысла ждать больше нет
            finish_transfer();
        }    
    }else{

        /* SB Set ----------------------------------------------------------------*/
        if(((sr1itflags & I2C_BIT_SB & I2C_BIT_MASK) != RESET) && ((itsources & I2C_IE_EVT) != RESET))    {
            // Send address for write
            if(_tx_len){
                i2c_send_address(_dev, _addr<<1, I2C_Direction_Transmitter);
                _state = I2C_want_ADDR;
            } else {
                i2c_send_address(_dev, _addr<<1, I2C_Direction_Receiver);
                _state = I2C_want_RX_ADDR;
            }

            _dev->I2Cx->CR1 &= (uint16_t)(~I2C_CR1_STOP);    /* clear STOP condition - just to touch CR1*/
        }
        /* ADDR Set --------------------------------------------------------------*/
        else if(((sr1itflags & I2C_BIT_ADDR & I2C_BIT_MASK) != RESET) && ((itsources & I2C_IE_EVT) != RESET))    {
            /* Clear ADDR register by reading SR1 then SR2 register (SR1 has already been read) */
        
            if(_tx_len) { // transmit
                // all flags set before
                _state = I2C_want_TXE;
            }else {      // receive
                _dev->I2Cx->CR2 |= I2C_CR2_ITBUFEN; // enable RXNE interrupt
                if(_rx_len == 1) {                 // Disable Acknowledge for 1-byte transfer
                    _dev->I2Cx->CR1 &= ~I2C_CR1_ACK;
                } else {
                    _dev->I2Cx->CR1 |= I2C_CR1_ACK;
                }
                _state = I2C_want_RXNE;
            }        
        }
    
        uint32_t sr2itflags   = _dev->I2Cx->SR2; // read SR2 - ADDR is cleared
    
        if((itsources & I2C_IE_BUF) != RESET ){ // data io

            if((sr1itflags & I2C_BIT_TXE & I2C_BIT_MASK) != RESET) {// TXE set 
                if((sr2itflags & (I2C_BIT_TRA) & I2C_BIT_MASK) != RESET) {    // I2C in mode Transmitter

                    if(_tx_len) {
                        _dev->I2Cx->DR = *_tx_buff++; // 1 byte
                        _tx_len--;
                    } else { // tx is over and last byte is sent
                        _dev->I2Cx->CR2 &= ~I2C_CR2_ITBUFEN; // disable TXE interrupt
                    }        
                }
            } 
            if((sr1itflags & I2C_BIT_RXNE & I2C_BIT_MASK) != RESET)   {       // RXNE set
                if(_rx_len && !_tx_len) {
                    *_rx_buff++ = (uint8_t)(_dev->I2Cx->DR);
                    _rx_len -= 1; // 1 byte done
	        
                    if(_rx_len == 1) { // last second byte
                        _dev->I2Cx->CR1 &= ~I2C_CR1_ACK;     // Disable Acknowledgement - send NACK for last byte 
                        _dev->I2Cx->CR1 |= I2C_CR1_STOP;     // Send STOP
                    } else if(_rx_len==0) {
                        _error = I2C_OK;
                        _state = I2C_done;

                        finish_transfer();
                    }
                } else { // fake byte after enable ITBUF
                    (void)_dev->I2Cx->DR;
                }
            }
        }
        if((sr1itflags & I2C_BIT_BTF & I2C_BIT_MASK) != RESET) {// BTF set 
            if((sr2itflags & (I2C_BIT_TRA) & I2C_BIT_MASK) != RESET) {    // I2C in mode Transmitter
                // BTF on transmit
                if(_rx_len) {
                    // wait a little - some devices requires time for internal operations
                    delay_ns100(3);
                    
                    // Send START condition a second time
                    _dev->I2Cx->CR1 |= I2C_CR1_START;
                    _state = I2C_want_RX_SB;
                    _dev->I2Cx->CR2 &= ~I2C_CR2_ITBUFEN; // disable TXE interrupt - just for case, should be disabled
                } else {   
                    _dev->I2Cx->CR1 |= I2C_CR1_STOP;     // Send STOP condition
                    _error = I2C_OK;                    // TX is done
                    _state = I2C_done;
                    finish_transfer();
                }
                
            } else { // BTF on receive??
                // 
            }
        }

#ifdef I2C_DEBUG
     I2C_State &sp = log[log_ptr]; // remember last operation
     
     sp.time     = Scheduler::_micros();
     sp.sr1      = sr1itflags;
     sp.sr2      = sr2itflags;
     sp.cr1      = _dev->I2Cx->CR1;
     sp.state    = _state;
     sp.pos      = I2C_ISR;
     if(log_ptr<I2C_LOG_SIZE-1) log_ptr++;
     else                       log_ptr=0;
#endif

    }
}


void I2CDevice::finish_transfer(){

    _dev->I2Cx->CR2 &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);    // Disable interrupts
    i2c_clear_isr_handler(_dev);
    _dev->I2Cx->CR1 &= ~(I2C_CR1_STOP | I2C_CR1_START); // clear CR1 - just for case

    Handler h;
    if( (h=_completion_cb) ){    // io completion
        _completion_cb=0; // only once and before call because handler can set it itself
        
        revo_call_handler(h, (uint32_t)_dev);
    }
    
    if(_task){ // resume paused task
        Scheduler::task_resume(_task);
        _task=NULL;
    }    
}

uint32_t I2CDevice::wait_stop_done(bool is_write){
    uint32_t sr1;
    uint32_t t;

    uint8_t ret;
    uint8_t i;
    for(i=0; i<10; i++){

        if( (_dev->I2Cx->CR1 & I2C_CR1_PE) ) {
            ret=I2C_OK;
        } else {
            ret=91;
        }
       // Wait to make sure that STOP control bit has been cleared - bus released
        t = hal_micros();
        while (_dev->I2Cx->CR1 & I2C_CR1_STOP ){
            if((sr1=_dev->I2Cx->SR1) & I2C_BIT_BERR & I2C_BIT_MASK) _dev->I2Cx->SR1 = (uint16_t)(~I2C_BIT_BERR); // Errata 2.4.6

            if(sr1 & I2C_BIT_ARLO & I2C_BIT_MASK) { // arbitration lost or bus error
                _dev->I2Cx->SR1 = (uint16_t)(~I2C_BIT_ARLO); // reset them
                ret= I2C_STOP_BERR; // bus error on STOP
                break;
            }
            if(sr1 & I2C_BIT_TIMEOUT & I2C_BIT_MASK) { // bus timeout
                _dev->I2Cx->SR1 = (uint16_t)(~I2C_BIT_TIMEOUT); // reset it
                ret= I2C_ERR_STOP_TIMEOUT;                             // STOP generated by hardware
                break;
            }

            if (hal_micros() - t > I2C_SMALL_TIMEOUT) {
                ret=I2C_ERR_STOP;
                break;
            }
            hal_yield(0);
        }

        /* wait while the bus is busy */
        t = hal_micros();
        while ((_dev->I2Cx->SR2 & (I2C_BIT_BUSY) & I2C_BIT_MASK) != 0) {
            if (hal_micros() - t > I2C_SMALL_TIMEOUT) {
                ret=2; // bus busy
                break;
            }
	
            hal_yield(0); 
        }


        if(ret==I2C_OK) return ret;
        
        _dev->I2Cx->CR1 |= I2C_CR1_SWRST;           // set SoftReset for some time 
        hal_yield(0);
        _dev->I2Cx->CR1 &= (uint16_t)(~I2C_CR1_SWRST); // clear SoftReset flag                    
        _dev->I2Cx->CR1 |= I2C_CR1_PE; // enable

        if(i>1){
            last_error = ret;   // remember
            if(is_write) last_error+=50;

            _lockup_count ++;  
            _initialized=false; // will be reinitialized in init()
        
            need_reset=true;
            init();
            
            if(!_initialized) return ret;
        
        }

    }

    return I2C_OK;
}



/*
    errata 2.4.6
Spurious Bus Error detection in Master mode
Description
In Master mode, a bus error can be detected by mistake, so the BERR flag can be wrongly
raised in the status register. This will generate a spurious Bus Error interrupt if the interrupt
is enabled. A bus error detection has no effect on the transfer in Master mode, therefore the
I2C transfer can continue normally.
Workaround
If a bus error interrupt is generated in Master mode, the BERR flag must be cleared by
software. No other action is required and the on-going transfer can be handled normally

*/
