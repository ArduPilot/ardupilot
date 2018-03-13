/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
(c) 2017 night_ghost@ykoctpa.ru
 
*/


#pragma GCC optimize ("O2")

#include <AP_HAL/AP_HAL.h>


#include <spi.h>
#include "Semaphores.h"

#include <inttypes.h>
#include <vector>
#include <AP_HAL/HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "Scheduler.h"
#include <spi.h>
#include <boards.h>

#include "SPIDevice.h"
#include "GPIO.h"


using namespace F4Light;


extern const SPIDesc spi_device_table[];    // different SPI tables per board subtype

extern const uint8_t F4Light_SPI_DEVICE_NUM_DEVICES; 

static const spi_pins board_spi_pins[] = {
    { // 0
        BOARD_SPI1_SCK_PIN,
        BOARD_SPI1_MISO_PIN,
        BOARD_SPI1_MOSI_PIN
    },
    { // 1
        BOARD_SPI2_SCK_PIN,
        BOARD_SPI2_MISO_PIN,
        BOARD_SPI2_MOSI_PIN
    },
    { //2
        BOARD_SPI3_SCK_PIN,
        BOARD_SPI3_MISO_PIN,
        BOARD_SPI3_MOSI_PIN
    }
};


F4Light::Semaphore SPIDevice::_semaphores[MAX_BUS_NUM] IN_CCM; // per bus data
void *              SPIDevice::owner[MAX_BUS_NUM] IN_CCM;
uint8_t *           SPIDevice::buffer[MAX_BUS_NUM] IN_CCM;      // DMA buffers in RAM

#ifdef DEBUG_SPI 
spi_trans SPIDevice::spi_trans_array[SPI_LOG_SIZE]  IN_CCM;
uint8_t   SPIDevice::spi_trans_ptr=0;
#endif


#ifdef BOARD_SOFTWARE_SPI

// as fast as possible
#define SCK_H       {sck_port->BSRRL = sck_pin; } 
#define SCK_L       {sck_port->BSRRH = sck_pin; }

#define MOSI_H      {mosi_port->BSRRL = mosi_pin; }
#define MOSI_L      {mosi_port->BSRRH = mosi_pin; }

#define MISO_read   ((miso_port->IDR & miso_pin)!=0)


static void dly_spi() {
    delay_ns100(dly_time);
};


uint8_t SPIDevice::_transfer_s(uint8_t bt) {

    for(int ii = 0; ii < 8; ++ii) {
        if (bt & 0x80) {
            MOSI_H;
        } else {
            MOSI_L;
        }
        SCK_L;

        dly_spi();
        SCK_H;
         
        bt <<= 1;
        if (MISO_read) {
            bt |= 1;
        }
        dly_spi();
    }

    return bt;
}

void SPIDevice::spi_soft_set_speed(){
    uint16_t rate;

    switch(_speed) {
    case SPI_36MHZ:
    case SPI_18MHZ:
    case SPI_9MHZ:
    case SPI_4_5MHZ:
        rate = 1;
        break;
    case SPI_2_25MHZ:
        rate = 2;
        break;
    case SPI_1_125MHZ:
        rate = 4; // 400 ns delay
        break;
    case SPI_562_500KHZ:
        rate = 8;
        break;
    case SPI_281_250KHZ:
        rate = 16;
        break;
    case SPI_140_625KHZ:
    default:
        rate = 32;
        break;
    }
    dly_time = rate; 
}

#endif

void SPIDevice::register_completion_callback(Handler h) { 
    if(_completion_cb){ // IOC from last call still not called - timeout
        // TODO: ???
    }
    _completion_cb = h; 
}


uint8_t SPIDevice::transfer(uint8_t out){
#ifdef BOARD_SOFTWARE_SPI
    if(_desc.mode == SPI_TRANSFER_SOFT) {
        return _transfer_s(out);
    } else 
#endif
    {
        return _transfer(out);
    }
    
}


uint8_t SPIDevice::_transfer(uint8_t data) {
    (void)_desc.dev->SPIx->DR; // read fake data out

    //wait for TXE before send
    while (!spi_is_tx_empty(_desc.dev)) {    // should wait transfer finished
        if(!spi_is_busy(_desc.dev) ) break;
    }

    //write 1byte
    _desc.dev->SPIx->DR = data;

    //wait for read byte
    while (!spi_is_rx_nonempty(_desc.dev)) {    // should wait transfer finished
        if(!spi_is_busy(_desc.dev) ) break;
    }
    
    return (uint8_t)(_desc.dev->SPIx->DR); // we got a byte so transfer complete
}

void SPIDevice::send(uint8_t out) {
    //wait for TXE before send
    while (!spi_is_tx_empty(_desc.dev)) {    // should wait transfer finished
        if(!spi_is_busy(_desc.dev) ) break;
    }
    //write 1byte
    spi_tx_reg(_desc.dev, out); //    _desc.dev->SPIx->DR = data;
}


bool SPIDevice::transfer(const uint8_t *out, uint32_t send_len, uint8_t *recv, uint32_t recv_len){
    
    uint8_t err=1;

// differrent devices on bus requires different modes
    if(owner[_desc.bus-1] != this) { // bus was in use by another driver so SPI hardware need reinit
        _initialized=false;
    }

    if(!_initialized){
        init();
        if(!_initialized) return false;
        owner[_desc.bus-1] = this; // Got it!
    }

#ifdef BOARD_SOFTWARE_SPI

// not deleted for case of needs for external SPI on arbitrary pins

    if(_desc.mode == SPI_TRANSFER_SOFT) {
        spi_soft_set_speed();

        _cs_assert();


        if (out != NULL && send_len) {
            for (uint16_t i = 0; i < send_len; i++) {
                _transfer_s(out[i]);
            }    
        } 
    
        if(recv !=NULL && recv_len) {
            for (uint16_t i = 0; i < recv_len; i++) {
                recv[i] = _transfer_s(0xff);
            }
        }

        _cs_release();

        if(_completion_cb) {
            revo_call_handler(_completion_cb, (uint32_t)&_desc);
            _completion_cb=0;
        }
    } else 
#endif
    {
    
        uint32_t t = hal_micros();
        while(_desc.dev->state->busy){ //       wait for previous transfer finished
            if(hal_micros() - t > 5000){
            // TODO increment grab counter
                 break; // SPI transfer can't be so long so let grab the bus
            }
            hal_yield(0);
        }

        
        spi_set_speed(_desc.dev, determine_baud_rate(_speed));
        
        _desc.dev->state->busy = true; // we got bus
        _cs_assert();

        _send_address = out;    //      remember transfer params for ISR
        _send_len     = send_len;
        _dummy_len    = send_len;
        _recv_address = recv;
        _recv_len     = recv_len;


#define MIN_DMA_BYTES 1 // 4 // write to 2-byte register not uses DMA, 4-byte command to DataFlash will
//#define MIN_DMA_BYTES 32 // for debug

        switch(_desc.mode){
            
        case SPI_TRANSFER_DMA: // DMA
            if(send_len + recv_len >= MIN_DMA_BYTES) {
                get_dma_ready();
                uint8_t nb = _desc.bus-1;

                bool can_dma = false;
                _desc.dev->state->len=0;

                // alloc buffer if recv needed - to not in ISR
                if(recv_len && !ADDRESS_IN_RAM(recv)){   //not in CCM
                    if(buffer[nb] == NULL){             
                        buffer[nb] = (uint8_t *)malloc(SPI_BUFFER_SIZE); // allocate only on 1st use
                    }
                }     

                _isr_mode = SPI_ISR_NONE;

                if(send_len){
                
//[ for debug
//                    if(send_len>1){
//]
                    if(ADDRESS_IN_RAM(out)){   // not in CCM
                        can_dma=true;
                    } else {
                        if(send_len<=SPI_BUFFER_SIZE){                        
                            if(buffer[nb] == NULL){             // allocate only on 1st use
                                buffer[nb] = (uint8_t *)malloc(SPI_BUFFER_SIZE);
                            }
                            if(buffer[nb]){
                                memmove(buffer[nb],out,send_len);
                                out = buffer[nb];
                                _send_len=0;
                                can_dma=true;
                            }
                        }
                    } 
//[ for debug
//                    }
//]
                    
                    if(can_dma){
                        setup_dma_transfer(out, NULL, send_len);
                    } else {
                        _isr_mode = SPI_ISR_SEND_DMA; // try DMA on receive
                        setup_isr_transfer();
                    }
                } else { // send_len == 0 so there will no RXNE interrupts so 
                    //  we need setup DMA RX transfer here
                    uint16_t len = _recv_len;
                    if(ADDRESS_IN_RAM(recv)){   //not in CCM
                        _desc.dev->state->len=0;
                        can_dma=true;
                    }else if(len<=SPI_BUFFER_SIZE && buffer[nb]){
                        _desc.dev->state->len=len;
                        _desc.dev->state->dst=recv;
                        recv = buffer[nb];
                        can_dma=true;
                    } 
                    if(can_dma){
                        setup_dma_transfer(NULL, recv, len);
                        _recv_len=0;
                    } else {
                        _isr_mode = SPI_ISR_RECEIVE;  // just receive
                        setup_isr_transfer();
                    }
                }
            
                err = do_transfer(can_dma, send_len + recv_len);
                break;
            }
            // no break!

        case SPI_TRANSFER_INTR: // interrupts
            _isr_mode = _send_len ? SPI_ISR_SEND : SPI_ISR_RECEIVE;
            setup_isr_transfer();
            err=do_transfer(false, send_len + recv_len);
            break;

        case SPI_TRANSFER_POLL: // polling
            err = spimaster_transfer(_desc.dev, out, send_len, recv, recv_len);
            _cs_release();
            _desc.dev->state->busy=false;
            if(_completion_cb) {
                revo_call_handler(_completion_cb, (uint32_t)&_desc);
                _completion_cb=0;
            }
            break;


        default: 
            break;
        }
    }



    return err==0;

}




// not used anywhere

bool SPIDevice::transfer_fullduplex(const uint8_t *out, uint8_t *recv, uint32_t len) {


    if(owner[_desc.bus-1] != this) { // bus was in use by another driver so need reinit
        _initialized=false;
    }
    
    if(!_initialized) {
        init();
        if(!_initialized) return false;
        owner[_desc.bus-1] = this; // Got it!
    }


#ifdef BOARD_SOFTWARE_SPI
    if(_desc.mode == SPI_TRANSFER_SOFT) {
        _cs_assert();
        spi_soft_set_speed();

        if (out != NULL && recv !=NULL && len) {
            for (uint16_t i = 0; i < len; i++) {
                recv[i] = _transfer_s(out[i]);
            }    
        } 
        return true;
    } else 
#endif
    {
        spi_set_speed(_desc.dev, determine_baud_rate(_speed)); 
        _cs_assert();
        
        switch(_desc.mode){
                
        case SPI_TRANSFER_DMA:
            if((out==NULL || ADDRESS_IN_RAM(out)) && (recv==NULL || ADDRESS_IN_RAM(recv)) ) {
                setup_dma_transfer(out, recv, len);
                return do_transfer(true, len)==0;
            }    
    
        // no break;
        case SPI_TRANSFER_INTR: // interrupts
            _isr_mode = SPI_ISR_RXTX;
            setup_isr_transfer();
            return do_transfer(false, len)==0;

        case SPI_TRANSFER_POLL: // polling
        default:
            if (out != NULL && recv !=NULL && len) {
                for (uint16_t i = 0; i < len; i++) {
                    recv[i] = _transfer(out[i]);
                }    
            }
            _cs_release();
            return true;
        } 
    }
    return false;
}


AP_HAL::OwnPtr<F4Light::SPIDevice>
SPIDeviceManager::_get_device(const char *name)
{
    const SPIDesc *desc = nullptr;
    
    /* Find the bus description in the table */
    for (uint8_t i = 0; i < F4Light_SPI_DEVICE_NUM_DEVICES; i++) {
        if (!strcmp(spi_device_table[i].name, name)) {
            desc = &spi_device_table[i];
            break;
        }
    }
 
    if (!desc) {
        AP_HAL::panic("SPI: invalid device name");
    }

    return AP_HAL::OwnPtr<F4Light::SPIDevice>(new SPIDevice(*desc));
}


SPIDevice::SPIDevice(const SPIDesc &device_desc)
    : _desc(device_desc)
    , _initialized(false)
    , _completion_cb(0)

{
    if(_desc.cs_pin < BOARD_NR_GPIO_PINS) {
        _cs = GPIO::get_channel(_desc.cs_pin);
        if (!_cs) {
            AP_HAL::panic("SPI: wrong CS pin");
        }
    } else {
        _cs = NULL; // caller itself controls CS
    }        
}        

const spi_pins* SPIDevice::dev_to_spi_pins(const spi_dev *dev) {
    if (     dev->SPIx == SPI1)
        return &board_spi_pins[0];
    else if (dev->SPIx == SPI2)
        return &board_spi_pins[1];
    else if (dev->SPIx == SPI3)
        return &board_spi_pins[2];
    else {
        assert_param(0);
        return NULL;
    }
}

spi_baud_rate SPIDevice::determine_baud_rate(SPIFrequency freq)
{

    spi_baud_rate rate;

    switch(freq) {
    case SPI_36MHZ:
    rate = SPI_BAUD_PCLK_DIV_2;
	byte_time = 1; // time in 0.25uS units
	break;
    case SPI_18MHZ:
	rate = SPI_BAUD_PCLK_DIV_4;
	byte_time = 2;
	break;
    case SPI_9MHZ:
	rate = SPI_BAUD_PCLK_DIV_8;
	byte_time = 4;
	break;
    case SPI_4_5MHZ:
	rate = SPI_BAUD_PCLK_DIV_16;
	byte_time = 8;
	break;
    case SPI_2_25MHZ:
	rate = SPI_BAUD_PCLK_DIV_32;
	byte_time = 16;
	break;
    case SPI_1_125MHZ:
	rate = SPI_BAUD_PCLK_DIV_64;
	byte_time = 32;
	break;
    case SPI_562_500KHZ:
	rate = SPI_BAUD_PCLK_DIV_128;
	byte_time = 64;
	break;
    case SPI_281_250KHZ:
	rate = SPI_BAUD_PCLK_DIV_256;
	byte_time = 128;
	break;
    case SPI_140_625KHZ:
	rate = SPI_BAUD_PCLK_DIV_256;
	byte_time = 255;
	break;
    default:
	rate = SPI_BAUD_PCLK_DIV_32;
	byte_time = 16;
	break;
    }
    return rate;
}


void SPIDevice::get_dma_ready(){
    const Spi_DMA &dp = _desc.dev->dma;

//  check for DMA not busy before use
    uint32_t t = hal_micros();
    while(dma_is_stream_enabled(dp.stream_rx) || dma_is_stream_enabled(dp.stream_tx) ) {   // wait for previous transfer termination
        if(hal_micros() - t > 1000) break; // DMA transfer can't be more than 1ms
        hal_yield(0);
    } 

    spi_disable_irq(_desc.dev, SPI_RXNE_TXE_INTERRUPTS); // just for case
}


// DMA dummy workplace 
static uint32_t rw_workbyte[] = { 0xffff }; // not in stack!

void  SPIDevice::setup_dma_transfer(const uint8_t *out, const uint8_t *recv, uint32_t btr){
    DMA_InitType DMA_InitStructure;

    const Spi_DMA &dp = _desc.dev->dma;
    uint32_t memory_inc;

    dma_init(dp.stream_rx); dma_init(dp.stream_tx);

    dma_clear_isr_bits(dp.stream_rx); dma_clear_isr_bits(dp.stream_tx);


    DMA_InitStructure.DMA_PeripheralBaseAddr    = (uint32_t)(&(_desc.dev->SPIx->DR));
    DMA_InitStructure.DMA_BufferSize            = btr;

    DMA_InitStructure.DMA_FIFO_flags = DMA_FIFOThreshold_Full | DMA_FIFOMode_Disable; // TODO use FIFO on large transfers

  // receive stream
    if(recv) {
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)recv;
        memory_inc                            = DMA_CR_MINC;
    } else {
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rw_workbyte;
        memory_inc                            = 0;
    }

    DMA_InitStructure.DMA_flags      = DMA_CR_MSIZE_8BITS | DMA_CR_PSIZE_8BITS |
                                       DMA_CR_MBURST0     | DMA_CR_PBURST0 |
                                       DMA_CR_DIR_P2M     |  memory_inc |
                                       dp.channel | _desc.prio;
    dma_init_transfer(dp.stream_rx, &DMA_InitStructure);

  // transmit stream
    if(out) {
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)out;
        memory_inc                            = DMA_CR_MINC;
    } else {
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rw_workbyte;
        memory_inc                            = 0;
    }
    DMA_InitStructure.DMA_flags      = DMA_CR_MSIZE_8BITS | DMA_CR_PSIZE_8BITS |
                                       DMA_CR_MBURST0     | DMA_CR_PBURST0 |
                                       DMA_CR_DIR_M2P     | memory_inc |
                                       dp.channel | _desc.prio;
    dma_init_transfer(dp.stream_tx, &DMA_InitStructure);

    dma_enable(dp.stream_rx); dma_enable(dp.stream_tx); // run them both!

    // we attach interrupt on RX channel's TransferComplete, so at ISR time bus will be free
    dma_attach_interrupt(dp.stream_rx, Scheduler::get_handler(FUNCTOR_BIND_MEMBER(&SPIDevice::dma_isr, void)), DMA_CR_TCIE);
}

void SPIDevice::start_dma_transfer(){
    spi_enable_dma_req(_desc.dev, SPI_DMAreq_Rx | SPI_DMAreq_Tx); /* Enable SPI TX/RX request */
}


void SPIDevice::disable_dma(){
    const Spi_DMA &dp = _desc.dev->dma;

    dma_disable(dp.stream_rx); dma_disable(dp.stream_tx);
    dma_detach_interrupt(dp.stream_rx); // we attach interrupt each request

    // Disable SPI RX/TX request 
    spi_disable_dma_req(_desc.dev, SPI_DMAreq_Rx | SPI_DMAreq_Tx);

    dma_clear_isr_bits(dp.stream_rx); dma_clear_isr_bits(dp.stream_tx);
}

void SPIDevice::dma_isr(){
    disable_dma();
    
    if(_desc.dev->state->len) {
        memmove(_desc.dev->state->dst, &buffer[_desc.bus-1][0], _desc.dev->state->len);
        _desc.dev->state->len=0; // once
    }

    _send_len = 0;  // send done

// we attach interrupt on RX channel's TransferComplete, so at ISR bus will be free

    if(_recv_len) {  //   now we should program DMA for receive or turn on ISR mode receiving if can't DMA
        (void)_desc.dev->SPIx->DR; // read fake data out
        // now we should set up DMA transfer

        spi_wait_busy(_desc.dev); // just for case - RX transfer is finished
        
        delay_ns100(3); // small delay between TX and RX, to give the chip time to think over domestic affairs
                        // for slow devices which need a time between address and data 
                        // 200ns uses setup_dma_transfer() itself

        bool can_dma = false;
        uint8_t *recv = _recv_address;
        uint16_t len  = _recv_len;
        uint8_t nb = _desc.bus-1;
            
        if(ADDRESS_IN_RAM(recv)){   //not in CCM
            _desc.dev->state->len=0;
            can_dma=true;                
        }else if(len<=SPI_BUFFER_SIZE && buffer[nb]){
            _desc.dev->state->len=len;
            _desc.dev->state->dst=recv;
            recv = buffer[nb];
            can_dma=true;
        } 
        if(can_dma){
            _isr_mode = SPI_ISR_NONE;
            spi_disable_irq(_desc.dev, SPI_RXNE_INTERRUPT); // disable RXNE interrupt, TXE already disabled
            setup_dma_transfer(NULL, recv, len);
            _recv_len = 0;      // receive by DMA
            start_dma_transfer();
        } else {
            _isr_mode = SPI_ISR_RECEIVE;  // just receive
            setup_isr_transfer();        
            spi_enable_irq(_desc.dev, SPI_TXE_INTERRUPT); // enable - will be interrupt just immediate
        }            
    } else { // all done
        isr_transfer_finish();                // releases SPI bus 
    }
}


void SPIDevice::init(){
    if(_cs) {
        _cs->mode(OUTPUT);
        _cs_release();    // do not hold the SPI bus initially
    }

    _completion_cb=0;
    
    const spi_pins *pins = dev_to_spi_pins(_desc.dev);

    if (!pins || pins->sck > BOARD_NR_GPIO_PINS || pins->mosi > BOARD_NR_GPIO_PINS || pins->miso > BOARD_NR_GPIO_PINS) {
        return;
    }

#ifdef BOARD_SOFTWARE_SPI
    if(_desc.mode == SPI_TRANSFER_SOFT) { //software
    
        { // isolate p
            const stm32_pin_info &p = PIN_MAP[pins->sck];
        
            const gpio_dev *sck_dev  = p.gpio_device;
                  uint8_t   sck_bit  = p.gpio_bit;

            gpio_set_mode(sck_dev,  sck_bit, GPIO_OUTPUT_PP);
            gpio_set_speed(sck_dev, sck_bit, GPIO_speed_100MHz);
            gpio_write_bit(sck_dev, sck_bit, 1); // passive SCK high
            

            sck_port = sck_dev->GPIOx;
            sck_pin  = 1<<sck_bit;
        }

        { // isolate p
            const stm32_pin_info &p = PIN_MAP[pins->mosi];

            const gpio_dev *mosi_dev = p.gpio_device;
                  uint8_t   mosi_bit = p.gpio_bit;
            gpio_set_mode(mosi_dev,  mosi_bit, GPIO_OUTPUT_PP);
            gpio_set_speed(mosi_dev, mosi_bit, GPIO_speed_100MHz);
            gpio_write_bit(mosi_dev, mosi_bit, 1); // passive MOSI high

            mosi_port = mosi_dev->GPIOx;
            mosi_pin  = 1<<mosi_bit;
        }

        { // isolate p
            const stm32_pin_info &p = PIN_MAP[pins->miso];

            const gpio_dev *miso_dev = p.gpio_device; 
                  uint8_t   miso_bit = p.gpio_bit;
            gpio_set_mode(miso_dev,  miso_bit, GPIO_INPUT_PU);
            gpio_set_speed(miso_dev, miso_bit, GPIO_speed_100MHz);

            miso_port = miso_dev->GPIOx;
            miso_pin  = 1<<miso_bit;
        }

    } else 
#endif
    { /// hardware
        spi_init(_desc.dev); // disable device

        const stm32_pin_info &miso = PIN_MAP[pins->miso];
        spi_gpio_master_cfg(_desc.dev,
                        miso.gpio_device, PIN_MAP[pins->sck].gpio_bit,
                        miso.gpio_bit,    PIN_MAP[pins->mosi].gpio_bit);


        spi_master_enable(_desc.dev, determine_baud_rate(_desc.lowspeed), _desc.sm, MSBFIRST);          
    }
    _initialized=true;

}


bool SPIDevice::set_speed(AP_HAL::Device::Speed speed)
{
//* this requires for 1-byte transfers
    if(owner[_desc.bus-1] != this) { // bus was in use by another driver so need reinit
        _initialized=false;
    }
    
    if(!_initialized) {
        init();
        if(!_initialized) return false;
        owner[_desc.bus-1] = this; // Got it!
    }
//*/

    switch (speed) {
    case AP_HAL::Device::SPEED_HIGH:
        _speed = _desc.highspeed;
        break;
    case AP_HAL::Device::SPEED_LOW:
    default:
        _speed = _desc.lowspeed;
        break;
    }

    return true;
}


// start transfer and wait until it finished
uint8_t  SPIDevice::do_transfer(bool is_DMA, uint32_t nbytes)
{

#ifdef DEBUG_SPI 
//    spi_trans_ptr=0; // each transfer from start

    spi_trans &p = spi_trans_array[spi_trans_ptr];

    p.time = hal_micros();
    p.dev      = _desc.dev;
    p.send_len = _send_len;
    p.recv_len = _recv_len;
    p.dummy_len=_dummy_len;
    p.data     = *_send_address;
    p.cr2 = 0;
    p.sr1 = 0;
    p.mode = _isr_mode;
    p.act = 0xff;
        
    spi_trans_ptr++;
    if(spi_trans_ptr>=SPI_LOG_SIZE) spi_trans_ptr=0;
#endif

    if(_completion_cb) {// we should call it after completion via interrupt    
        if(is_DMA) {
            start_dma_transfer();
        } else {
            spi_enable_irq(_desc.dev, SPI_RXNE_TXE_INTERRUPTS );// enable both interrupts - will be interrupt just immediate
        }
        return 0;                                               // all another in ISR
    }

    // no callback - need to wait 
    uint32_t timeout = nbytes * 16; // time to transfer all data - 16uS per byte
    
    uint32_t t=hal_micros();
    
    _task = Scheduler::get_current_task();

    EnterCriticalSection; // we are in multitask so if task switch occures between enable and pause then all transfer occures before task will be paused
                          //  so task will be paused after transfer and never be resumed - so will cause timeout

    if(_task) Scheduler::task_pause(timeout);

    if(is_DMA) {             // Enable SPI TX/RX request 
        start_dma_transfer();
    } else {
        spi_enable_irq(_desc.dev, SPI_RXNE_TXE_INTERRUPTS); // enable both interrupts - will be interrupt just immediate
    }
    LeaveCriticalSection;
            
    while (hal_micros()-t < timeout && _desc.dev->state->busy) {
        hal_yield(0); // пока ждем пусть другие работают.
    }

    _task = NULL; // already resumed
    if(_desc.dev->state->busy) { // timeout, so there was no ISR, so
#ifdef DEBUG_SPI 
        p.sr1=0x80;
#endif
        if(is_DMA) disable_dma(); 
        isr_transfer_finish();     //   disable interrupts
    } 

    return (_send_len == 0 && _recv_len == 0)? 0 : 1;
}


void SPIDevice::setup_isr_transfer() {
    spi_attach_interrupt(_desc.dev, Scheduler::get_handler(FUNCTOR_BIND_MEMBER(&SPIDevice::spi_isr, void)) );    

   (void)_desc.dev->SPIx->DR; // read fake data out
}



uint16_t  SPIDevice::send_strobe(const uint8_t *buffer, uint16_t len){ // send in ISR and strobe each byte by CS
    while(_desc.dev->state->busy) hal_yield(0); // wait for previous transfer finished
    
    _send_address = buffer;
    _send_len = len;
    _recv_len = 0;
    _isr_mode = SPI_ISR_STROBE;

    spi_attach_interrupt(_desc.dev, Scheduler::get_handler(FUNCTOR_BIND_MEMBER(&SPIDevice::spi_isr, void)) );    

    _cs->_write(0);

    _desc.dev->state->busy=true;

   (void)_desc.dev->SPIx->DR; // read fake data out
//[ write out 1st byte to do RXNE interrupt
    _send_len--;       //        write 1st byte to start transfer
    uint8_t b = *_send_address++;
    _desc.dev->SPIx->DR = b;
//]

    (void) do_transfer(false, len);
    
    return _send_len;
}

// gives received bytes to callback and returns when callback returns true but not linger than timeout (uS)
// so it works like wait for needed byte in ISR - but without wait
uint8_t SPIDevice::wait_for(uint8_t out, spi_WaitFunc cb, uint32_t dly){ // wait for needed byte in ISR
    _send_len = out;
    _isr_mode = SPI_ISR_COMPARE;
    _recv_len = 0;  // we shouldn't receive after wait
    _compare_cb = cb;    

    spi_attach_interrupt(_desc.dev, Scheduler::get_handler(FUNCTOR_BIND_MEMBER(&SPIDevice::spi_isr, void)) );    

    uint32_t t = hal_micros();
    _desc.dev->state->busy=true;

    // need to wait until transfer complete 
    _task = Scheduler::get_current_task();

    (void)_desc.dev->SPIx->DR; // read fake data out
    _desc.dev->SPIx->DR = out; // start transfer and clear flag

    EnterCriticalSection;      // prevent from task switch
    if(_task)  Scheduler::task_pause(dly); // if function called from task - store it and pause

    spi_enable_irq(_desc.dev, SPI_RXNE_TXE_INTERRUPTS); // enable both - will be interrupt on next line
    LeaveCriticalSection;

    while (hal_micros() - t < dly && _desc.dev->state->busy) {
        hal_yield(0);
    }

    _task = NULL; // already resumed
    if(_desc.dev->state->busy) isr_transfer_finish(); // timeout
    return  _recv_data;
}

// releases SPI bus after transfer finished, call callback and resume task
void SPIDevice::isr_transfer_finish(){
    spi_disable_irq(_desc.dev, SPI_RXNE_TXE_INTERRUPTS); // just for case

    spi_detach_interrupt(_desc.dev);

    while (spi_is_rx_nonempty(_desc.dev)) {    // read out fake data for TX only transfers
        (void)spi_rx_reg(_desc.dev);
    }

    _desc.dev->state->busy=false; // reset 

    if(_task){ // resume paused task
        Scheduler::task_resume(_task); // task will be resumed having very high priority & force 
                                              // context switch just after return from ISR so task will get a tick
        _task=NULL;
    }

    spi_wait_busy(_desc.dev);          // just for case - we already after last RXNE
    if(_isr_mode != SPI_ISR_COMPARE) {
        _cs_release(); // free bus
    }

    Handler h;
    if((h=_completion_cb)) {
        _completion_cb=0; // only once and BEFORE call itself because IOC can do new transfer

        revo_call_handler(h, (uint32_t)&_desc);
    }
}

void SPIDevice::spi_isr(){
#ifdef DEBUG_SPI 
    spi_trans &p = spi_trans_array[spi_trans_ptr];

    p.time = hal_micros();
    p.dev      = _desc.dev;
    p.send_len = _send_len;
    p.recv_len = _recv_len;
    p.dummy_len=_dummy_len;
    p.cr2 = _desc.dev->SPIx->CR2;
    p.sr1 = _desc.dev->SPIx->SR;
    p.mode = _isr_mode;
    p.act = 0;
        
    spi_trans_ptr++;
    if(spi_trans_ptr>=SPI_LOG_SIZE) spi_trans_ptr=0;
#endif


    if(spi_is_tx_empty(_desc.dev)  && spi_is_irq_enabled(_desc.dev, SPI_TXE_INTERRUPT)) {
#ifdef DEBUG_SPI 
        p.act |= 1;
#endif
        switch(_isr_mode) {
        case SPI_ISR_NONE:
            (void)_desc.dev->SPIx->DR;          // reset RXNE  
            spi_disable_irq(_desc.dev, SPI_TXE_INTERRUPT);  // disable TXE interrupt
            _isr_mode=SPI_ISR_FINISH; // should  releases SPI bus after last RXNE is set
            break;
        
        case SPI_ISR_SEND:
        case SPI_ISR_SEND_DMA:
            if(_send_len) {
                _send_len--;
                _desc.dev->SPIx->DR = *_send_address++;
            } else { // all sent
                // now we in 1 byte till bus release, so wait for RXNE
            
                spi_disable_irq(_desc.dev, SPI_TXE_INTERRUPT);  // disable TXE interrupt
            
                if(_recv_len) {
                    if(_isr_mode==SPI_ISR_SEND) {
                        _isr_mode=SPI_ISR_WAIT_RX;              // switch receive mode after receiving of fake RX byte
                    } else if(_isr_mode==SPI_ISR_SEND_DMA) {
                        _isr_mode=SPI_ISR_WAIT_RX_DMA;          // will switch to DMA receive mode after receiving of fake RX byte
                    } else {
                        _isr_mode=SPI_ISR_FINISH; // should release SPI bus after last RXNE is set
                    }
                } else {
                    _isr_mode=SPI_ISR_FINISH; // should release SPI bus after last RXNE is set            
                }            
            }
            break;

        case SPI_ISR_RXTX:
            _desc.dev->SPIx->DR = *_send_address++;
            break;

        case SPI_ISR_RECEIVE:
            if(_recv_len > 1) { // not on last byte
                _desc.dev->SPIx->DR = 0xFF; // dummy byte
            } else {
                spi_disable_irq(_desc.dev, SPI_TXE_INTERRUPT);  // disable TXE interrupt
            }
            break;
        
        case SPI_ISR_COMPARE:
        case SPI_ISR_STROBE: 
        default:
            spi_disable_irq(_desc.dev, SPI_TXE_INTERRUPT);  // disable unneeded TXE interrupt
            break;            
        }
    }

    if(spi_is_rx_nonempty(_desc.dev) /* && spi_is_irq_enabled(_desc.dev, SPI_RXNE_INTERRUPT) */) {
#ifdef DEBUG_SPI 
        p.act |= 2;
#endif

        switch(_isr_mode) {

        case SPI_ISR_STROBE: {
                (void)_desc.dev->SPIx->DR; // read fake data out
                if(_send_len){
                    spi_wait_busy(_desc.dev); 
                    delay_ns100(1);
                    _cs->_write(1);
                    delay_ns100(1);
                    _send_len--;
                    uint8_t b = *_send_address++;
                    _cs->_write(0);
                    delay_ns100(1);
                    _desc.dev->SPIx->DR = b;            
                } else {
                    isr_transfer_finish();                // releases SPI bus after transfer complete
                }
            } 
            break;

        case SPI_ISR_SEND:
        case SPI_ISR_SEND_DMA:
            (void)_desc.dev->SPIx->DR; // read fake data out
            _dummy_len--;               // and count them
            break;

        case SPI_ISR_RECEIVE:
            if(_recv_len){
                _recv_len--;
                *_recv_address++ = _desc.dev->SPIx->DR;
            }
            if(_recv_len==0) {      // last byte received
                isr_transfer_finish();  // releases SPI bus after last RXNE is set
            }
            break;
        
        case SPI_ISR_RXTX:
            *_recv_address++ = _desc.dev->SPIx->DR;
            _send_len--;
            if(!_send_len) { // readed the last byte
                isr_transfer_finish();                // releases SPI bus            
            }
            break;

        case SPI_ISR_COMPARE:
            _recv_data = _desc.dev->SPIx->DR;

#ifdef DEBUG_SPI 
            p.data = _recv_data;
            p.cb   = _compare_cb;
#endif

            if(_compare_cb(_recv_data) ) {   // ok
                if(_recv_len){
                    _isr_mode = SPI_ISR_RECEIVE; // we should receive after wait ?
                } else {
                    _send_len=0;                 // mark transfer finished
                    isr_transfer_finish();
                }
            }  else { // do nothing - just skip byte
                _desc.dev->SPIx->DR = _send_len; // data to send in len, only when we need next byte
            }
            break;

        case SPI_ISR_WAIT_RX_DMA:  // we just got last RXNE of transfer
            (void)_desc.dev->SPIx->DR; // read fake data out
            if(_dummy_len){
                _dummy_len--; 
            }
            if(_dummy_len==0) {  // this was a last dummy byte,  now we should program DMA for receive or turn on ISR mode receiving if can't DMA
                // now we should set up DMA transfer

                bool can_dma = false;
                uint8_t *recv = _recv_address;
                uint16_t len  = _recv_len;
                uint8_t nb = _desc.bus-1;
            
                delay_ns100(5); // small delay between TX and RX, to give the chip time to think over domestic affairs
            
                if(ADDRESS_IN_RAM(recv)){   //not in CCM
                    _desc.dev->state->len=0;
                    can_dma=true;                
                }else if(len<=SPI_BUFFER_SIZE && buffer[nb]){
                    _desc.dev->state->len=len;
                    _desc.dev->state->dst=recv;
                    recv = buffer[nb];
                    can_dma=true;
                } 
                if(can_dma){
#ifdef DEBUG_SPI 
                    p.act |= 10;
#endif
                    spi_disable_irq(_desc.dev, SPI_RXNE_INTERRUPT); // disable RXNE interrupt, TXE already disabled
                    setup_dma_transfer(NULL, recv, len);
                    start_dma_transfer();
                    _recv_len = 0; // all done
                    _isr_mode = SPI_ISR_NONE;
                    break;
                }
                
            }
            // no break! we can't receive via DMA so setup receive in ISR

        case SPI_ISR_WAIT_RX:           //  turn on ISR mode receiving 
            (void)_desc.dev->SPIx->DR;          // read fake data out
            if(_dummy_len){
                _dummy_len--;
            }
            if(_dummy_len==0) {  // this was a last dummy byte,  now we should receive            
                _isr_mode = SPI_ISR_RECEIVE;                // we should receive 
                _desc.dev->SPIx->DR = 0xFF; // write dummy byte for 1st transfer

                _dummy_len = _recv_len-1; // set number of bytes to be sent
                if(_dummy_len) {        // if we should send additional bytes
                    spi_enable_irq(_desc.dev, SPI_TXE_INTERRUPT);  // enable TXE interrupt
                }
            }
            break;

        case SPI_ISR_FINISH:
            (void)_desc.dev->SPIx->DR;          // read fake data out
            if(_dummy_len){
                _dummy_len--;   
            }
            if(_dummy_len) break; // not last byte
        
        case SPI_ISR_NONE:
        default:
            isr_transfer_finish();                // releases SPI bus
            break;
        }
    }
}

