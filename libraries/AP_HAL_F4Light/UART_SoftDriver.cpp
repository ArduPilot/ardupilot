/*

    (c) 2017 night_ghost@ykoctpa.ru
 
based on: ST appnote
 
 * SerialDriver.cpp --- AP_HAL_F4Light SoftSerial driver.
 *
 */

#pragma GCC optimize ("O2")

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_F4LIGHT && defined(BOARD_SOFTSERIAL_RX) && defined(BOARD_SOFTSERIAL_TX)
#include "UART_SoftDriver.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>

#include <gpio_hal.h>


using namespace F4Light;


// hardware RX and software TX

void SerialDriver::begin(uint32_t baud) {
    gpio_write_bit(tx_pp.gpio_device, tx_pp.gpio_bit, _inverse?LOW:HIGH);
    gpio_set_mode(tx_pp.gpio_device, tx_pp.gpio_bit, GPIO_OUTPUT_PP);
    gpio_set_mode(rx_pp.gpio_device, rx_pp.gpio_bit, GPIO_INPUT_PU);
    

    timer_pause(timer);
    uint32_t prescaler;
    
    if (baud > 2400) {
        bitPeriod = (uint16_t)((uint32_t)(CYCLES_PER_MICROSECOND * 1000000) / baud);
        prescaler=1;
    } else {
        bitPeriod = (uint16_t)(((uint32_t)(CYCLES_PER_MICROSECOND * 1000000) / 16) / baud);
        prescaler=16;
    }

    timer_set_prescaler(timer, prescaler-1);
    
    timer_set_reload(timer, bitPeriod/2); // for TX needs
        
    transmitBufferRead = transmitBufferWrite = 0;
    txBitCount = 8; // 1st interrupt will generate STOP
    txSkip=true;

    // Set rx State machine start state, attach the bit interrupt and mask it until start bit is received
    receiveBufferRead = receiveBufferWrite = 0;
    rxBitCount = 9;

    rxSetCapture(); // wait for start bit
    timer_attach_interrupt(timer, channel,                Scheduler::get_handler(FUNCTOR_BIND_MEMBER(&SerialDriver::rxNextBit,void)), SOFT_UART_INT_PRIORITY);
    timer_attach_interrupt(timer, TIMER_UPDATE_INTERRUPT, Scheduler::get_handler(FUNCTOR_BIND_MEMBER(&SerialDriver::txNextBit,void)), SOFT_UART_INT_PRIORITY); // also enables interrupt, so 1st interrupt will be ASAP
    // there will be interrupt after enabling TX interrupts, it will be disabled in handler
        
    timer_generate_update(timer);     // Load the timer values and start it
    timer_resume(timer);
    
    _initialized = true;
}


void SerialDriver::rxSetCapture(){
    timer_ic_set_mode(timer, channel, TIM_ICSelection_DirectTI | TIM_ICPSC_DIV1, 3);
    timer_cc_set_pol(timer,  channel, _inverse?TIMER_POLARITY_RISING:TIMER_POLARITY_FALLING);
}

void SerialDriver::rxSetCompare(){
    timer_set_mode(timer, channel, TIMER_OUTPUT_COMPARE); // for RX needs, capture mode by hands
}


void SerialDriver::end() {
    timer_pause(timer);
    gpio_write_bit(tx_pp.gpio_device, tx_pp.gpio_bit, 1);
    _initialized = false;

}

void SerialDriver::flush() {
    receiveBufferRead = receiveBufferWrite = 0;
}


bool SerialDriver::tx_pending() {
    if(!_initialized) return 0;
    
    return (transmitBufferWrite + SS_MAX_TX_BUFF - transmitBufferRead) % SS_MAX_TX_BUFF;
}


uint32_t SerialDriver::available() {

    return (receiveBufferWrite + SS_MAX_RX_BUFF - receiveBufferRead) % SS_MAX_RX_BUFF;
}

uint32_t SerialDriver::txspace() {
    return SS_MAX_TX_BUFF - tx_pending();
}

int16_t SerialDriver::read() {
    if (!_initialized)
        return -1;
  
    // Wait if buffer is empty
    if(receiveBufferRead == receiveBufferWrite) return -1; // no data
  
    uint8_t inData = receiveBuffer[receiveBufferRead];
  
    receiveBufferRead = (receiveBufferRead + 1) % SS_MAX_RX_BUFF;

    return inData;
}

size_t SerialDriver::write(uint8_t c) {
    if (!_initialized) return 0;

    // Blocks if buffer full
    uint16_t n_try=3;
    do { // wait for free space
        if( ((transmitBufferWrite + 1) % SS_MAX_TX_BUFF) == transmitBufferRead ){
            Scheduler::yield(); // while we wait - let others work
            if(! _blocking) n_try--;    
        } else break; // got it!
    } while(n_try);

    // Save new data in buffer and bump the write pointer
    transmitBuffer[transmitBufferWrite] = c;

    transmitBufferWrite = (transmitBufferWrite == SS_MAX_TX_BUFF) ? 0 : transmitBufferWrite + 1;


    // Check if transmit timer interrupt enabled and if not unmask it
    // transmit timer interrupt will get masked by transmit ISR when buffer becomes empty
    if (!activeTX) {
        activeTX=true;
        
        // Set state to 10 (send start bit) and re-enable transmit interrupt
        txBitCount = 10;

        txEnableInterrupts(); // enable
    }

    return 1;
}

size_t SerialDriver::write(const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--) {
        n += write(*buffer++);
    }
    return n;
}


#define bitRead(value, bit)            (((value) >> (bit)) & 0x01)

// Transmits next bit. Called by timer update interrupt
void SerialDriver::txNextBit(void /* TIM_TypeDef *tim */) { // ISR

    txSkip= !txSkip;
    
    if(txSkip) return; // one bit per 2 periods


    // State 0 through 7 - transmit bits
    if (txBitCount <= 7) {
        if (bitRead(transmitBuffer[transmitBufferRead], txBitCount) == (_inverse?0:1)) {
            gpio_write_bit(tx_pp.gpio_device, tx_pp.gpio_bit,HIGH); 
        } else {
            gpio_write_bit(tx_pp.gpio_device, tx_pp.gpio_bit,LOW);
        }

        // Bump the bit/state counter to state 8
        txBitCount++; 

#if DEBUG_DELAY && defined(DEBUG_PIN1)
        GPIO::_write(DEBUG_PIN1,1);
        GPIO::_write(DEBUG_PIN1,0);
#endif

    // State 8 - Send the stop bit and reset state to state -1
    //          Shutdown timer interrupt if buffer empty
    } else if (txBitCount == 8) {

        // Send the stop bit
        gpio_write_bit(tx_pp.gpio_device, tx_pp.gpio_bit, _inverse?LOW:HIGH); 

        transmitBufferRead = (transmitBufferRead == SS_MAX_TX_BUFF ) ? 0 : transmitBufferRead + 1;

        if (transmitBufferRead != transmitBufferWrite) { // we have data do transmit
            txBitCount = 10;
        } else {
            // Buffer empty so shutdown timer until write() puts data in
            txDisableInterrupts();
            activeTX=false;
        }

    // Send  start bit for new byte
    } else if (txBitCount >= 10) {
        gpio_write_bit(tx_pp.gpio_device, tx_pp.gpio_bit, _inverse?HIGH:LOW);

        txBitCount = 0;                    
    }
}



// Receive next bit. Called by timer channel interrupt
void SerialDriver::rxNextBit(void /* TIM_TypeDef *tim */) { // ISR

    if(!activeRX) { // capture start bit

        // Test if this is really the start bit and not a spurious edge
        if (rxBitCount == 9) {  

            uint16_t pos = timer_get_capture(timer, channel);

            rxSetCompare(); // turn to compare mode
            
            timer_set_compare(timer, channel, pos); // captured value
    
            // Set state/bit to first bit
            rxSkip=false; // next half bit will OK
            activeRX=true;
        }
    } else { // compare match twice per bit;
        rxSkip= !rxSkip;

        if(!rxSkip) return; // not the middle of bit
        
// now in middle of bit
        uint8_t d = gpio_read_bit( rx_pp.gpio_device, rx_pp.gpio_bit);
        
        if (rxBitCount == 9) {   // check start bit again
            if ( d == _inverse?HIGH:LOW) { // start OK
                rxBitCount = 0;
            } else { // false start
                activeRX=false;
                rxSetCapture(); // turn back to capture mode
            }
        } else if (rxBitCount < 8) { // get bits
            receiveByte >>= 1;  
            
            
            if ( d == _inverse?LOW:HIGH) 
              receiveByte |= 0x80;

#if DEBUG_DELAY
            GPIO::_write(DEBUG_PIN,1);
            GPIO::_write(DEBUG_PIN,0);
#endif
    
      
            rxBitCount++;  

        // State 8 - Save incoming byte and update buffer
        } else if (rxBitCount == 8) {
            if ( d == _inverse?LOW:HIGH) { // stop OK - save byte
                // Finish out stop bit while we...  
            
                if (gpio_read_bit( rx_pp.gpio_device, rx_pp.gpio_bit) == _inverse?LOW:HIGH) // valid STOP
                    receiveBuffer[receiveBufferWrite] = receiveByte;

                //  Calculate location in buffer for next incoming byte        
                 uint8_t next = (receiveBufferWrite + 1) % SS_MAX_RX_BUFF; // FYI - With this logic we effectively only have an (SS_MAX_RX_BUFF - 1) buffer size

                //  Test if buffer full
                //  If the buffer isn't full update the tail pointer to point to next location
                 if (next != receiveBufferRead) {
                    receiveBufferWrite = next;
                 } 
#ifdef SS_DEBUG
                 else {
                    bufferOverflow = true;  //  Else if it is now full set the buffer overflow flag 

      
#if DEBUG_DELAY && defined(DEBUG_PIN1)
                  overFlowTail = receiveBufferWrite;
                  overFlowHead = receiveBufferRead;

                  GPIO::_write(DEBUG_PIN1, 1);
                  GPIO::_write(DEBUG_PIN1, 0);
#endif
                }
#endif
            }
            // Set for state 9 to receive next byte
            rxBitCount = 9;
            activeRX=false;
            rxSetCapture(); // turn back to capture mode
        }
    }
}

#endif 
