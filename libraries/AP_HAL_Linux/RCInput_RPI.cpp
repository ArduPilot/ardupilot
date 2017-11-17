#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO ||        \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 ||   \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH ||           \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DARK ||         \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "GPIO.h"
#include "RCInput_RPI.h"
#include "Util_RPI.h"

#ifdef DEBUG
#define debug(fmt, args ...) do { fprintf(stderr,"[RCInput_RPI]: %s:%d: " fmt, __FUNCTION__, __LINE__, ## args); } while (0)
#else
#define debug(fmt, args ...)
#endif

//Parametres
#define RCIN_RPI_BUFFER_LENGTH   8
#define RCIN_RPI_SAMPLE_FREQ     500
#define RCIN_RPI_DMA_CHANNEL     0
#define RCIN_RPI_MAX_COUNTER     1300
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH
#define PPM_INPUT_RPI RPI_GPIO_5
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
#define PPM_INPUT_RPI NAVIO_GPIO_PPM_IN
#define PAGE_SIZE           (4*1024)
#else
#define PPM_INPUT_RPI RPI_GPIO_4
#endif
#define RCIN_RPI_MAX_SIZE_LINE   50

//Memory Addresses
#define RCIN_RPI_RPI1_DMA_BASE 0x20007000
#define RCIN_RPI_RPI1_CLK_BASE 0x20101000
#define RCIN_RPI_RPI1_PCM_BASE 0x20203000

#define RCIN_RPI_RPI2_DMA_BASE 0x3F007000
#define RCIN_RPI_RPI2_CLK_BASE 0x3F101000
#define RCIN_RPI_RPI2_PCM_BASE 0x3F203000

#define RCIN_RPI_GPIO_LEV0_ADDR  0x7e200034
#define RCIN_RPI_DMA_LEN         0x1000
#define RCIN_RPI_CLK_LEN         0xA8
#define RCIN_RPI_PCM_LEN         0x24
#define RCIN_RPI_TIMER_BASE      0x7e003004

#define RCIN_RPI_DMA_SRC_INC     (1<<8)
#define RCIN_RPI_DMA_DEST_INC    (1<<4) 
#define RCIN_RPI_DMA_NO_WIDE_BURSTS  (1<<26)
#define RCIN_RPI_DMA_WAIT_RESP   (1<<3)
#define RCIN_RPI_DMA_D_DREQ      (1<<6)
#define RCIN_RPI_DMA_PER_MAP(x)  ((x)<<16)
#define RCIN_RPI_DMA_END         (1<<1)
#define RCIN_RPI_DMA_RESET       (1<<31)
#define RCIN_RPI_DMA_INT         (1<<2)

#define RCIN_RPI_DMA_CS          (0x00/4)
#define RCIN_RPI_DMA_CONBLK_AD   (0x04/4)
#define RCIN_RPI_DMA_DEBUG       (0x20/4)

#define RCIN_RPI_PCM_CS_A        (0x00/4)
#define RCIN_RPI_PCM_FIFO_A      (0x04/4)
#define RCIN_RPI_PCM_MODE_A      (0x08/4)
#define RCIN_RPI_PCM_RXC_A       (0x0c/4)
#define RCIN_RPI_PCM_TXC_A       (0x10/4)
#define RCIN_RPI_PCM_DREQ_A      (0x14/4)
#define RCIN_RPI_PCM_INTEN_A     (0x18/4)
#define RCIN_RPI_PCM_INT_STC_A   (0x1c/4)
#define RCIN_RPI_PCM_GRAY        (0x20/4)

#define RCIN_RPI_PCMCLK_CNTL     38
#define RCIN_RPI_PCMCLK_DIV      39


extern const AP_HAL::HAL& hal;

using namespace Linux;


volatile uint32_t *RCInput_RPI::pcm_reg;
volatile uint32_t *RCInput_RPI::clk_reg;
volatile uint32_t *RCInput_RPI::dma_reg;

Memory_table::Memory_table()
{
    _page_count = 0;
}

// Init Memory table
Memory_table::Memory_table(uint32_t page_count, int version)
{
    uint32_t i;
    int fdMem, file;
    // Cache coherent adresses depends on RPI's version
    uint32_t bus = version == 1 ? 0x40000000 : 0xC0000000;
    uint64_t pageInfo;
    void *offset;

    _virt_pages = (void **)malloc(page_count * sizeof(void *));
    _phys_pages = (void **)malloc(page_count * sizeof(void *));
    _page_count = page_count;

    if ((fdMem = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0) {
        fprintf(stderr, "Failed to open /dev/mem\n");
        exit(-1);
    }

    if ((file = open("/proc/self/pagemap", O_RDWR | O_SYNC | O_CLOEXEC)) < 0) {
        fprintf(stderr, "Failed to open /proc/self/pagemap\n");
        exit(-1);
    }

    // Magic to determine the physical address for this page:
    offset = mmap(0, _page_count * PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS | MAP_NORESERVE | MAP_LOCKED, -1, 0);
    lseek(file, ((uintptr_t)offset) / PAGE_SIZE * 8, SEEK_SET);

    // Get list of available cache coherent physical addresses
    for (i = 0; i < _page_count; i++) {
        _virt_pages[i] = mmap(0, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS | MAP_NORESERVE | MAP_LOCKED, -1, 0);
        if (::read(file, &pageInfo, 8) < 8) {
            fprintf(stderr, "Failed to read pagemap\n");
            exit(-1);
        }
        _phys_pages[i] = (void *)((uintptr_t)(pageInfo * PAGE_SIZE) | bus);
    }

    // Map physical addresses to virtual memory
    for (i = 0; i < _page_count; i++) {
        munmap(_virt_pages[i], PAGE_SIZE);
        _virt_pages[i] = mmap(_virt_pages[i], PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED | MAP_NORESERVE | MAP_LOCKED, fdMem, ((uintptr_t)_phys_pages[i] & (version == 1 ? 0xFFFFFFFF : ~bus)));
        memset(_virt_pages[i], 0xee, PAGE_SIZE);
    }
    close(file);
    close(fdMem);
}

Memory_table::~Memory_table()
{
    free(_virt_pages);
    free(_phys_pages);
}

// This function returns physical address with help of pointer, which is offset
// from the beginning of the buffer.
void *Memory_table::get_page(void **const pages, uint32_t addr) const
{
    if (addr >= PAGE_SIZE * _page_count) {
        return nullptr;
    }
    return (uint8_t *)pages[(uint32_t)addr / 4096] + addr % 4096;
}

//Get virtual address from the corresponding physical address from memory_table.
void *Memory_table::get_virt_addr(const uint32_t phys_addr) const
{
    // FIXME: Can't the address be calculated directly?
    // FIXME: if the address room  in _phys_pages is not fragmented one may avoid
    //        a complete loop ..
    uint32_t i = 0;
    for (; i < _page_count; i++) {
        if ((uintptr_t)_phys_pages[i] == (((uintptr_t)phys_addr) & 0xFFFFF000)) {
            return (void *)((uintptr_t)_virt_pages[i] + (phys_addr & 0xFFF));
        }
    }
    return nullptr;
}

// This function returns offset from the beginning of the buffer using virtual
// address and memory_table.
uint32_t Memory_table::get_offset(void ** const pages, const uint32_t addr) const
{
    uint32_t i = 0;
    for (; i < _page_count; i++) {
        if ((uintptr_t) pages[i] == (addr & 0xFFFFF000) ) {
            return (i*PAGE_SIZE + (addr & 0xFFF));
        }
    }
    return -1;
}

// How many bytes are available for reading in circle buffer?
uint32_t Memory_table::bytes_available(const uint32_t read_addr, const uint32_t write_addr) const
{
    if (write_addr > read_addr) {
        return (write_addr - read_addr);
    } else {
        return _page_count * PAGE_SIZE - (read_addr - write_addr);
    }
}

uint32_t Memory_table::get_page_count() const
{
    return _page_count;
}

// Physical addresses of peripheral depends on Raspberry Pi's version
void RCInput_RPI::set_physical_addresses(int version)
{
    if (version == 1) {
        dma_base = RCIN_RPI_RPI1_DMA_BASE;
        clk_base = RCIN_RPI_RPI1_CLK_BASE;
        pcm_base = RCIN_RPI_RPI1_PCM_BASE;
    } else if (version == 2) {
        dma_base = RCIN_RPI_RPI2_DMA_BASE;
        clk_base = RCIN_RPI_RPI2_CLK_BASE;
        pcm_base = RCIN_RPI_RPI2_PCM_BASE;
    }
}

// Map peripheral to virtual memory
void *RCInput_RPI::map_peripheral(uint32_t base, uint32_t len)
{
    int fd = open("/dev/mem", O_RDWR | O_CLOEXEC);
    void *vaddr;

    if (fd < 0) {
        printf("Failed to open /dev/mem: %m\n");
        return nullptr;
    }
    vaddr = mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, base);
    if (vaddr == MAP_FAILED) {
        printf("rpio-pwm: Failed to map peripheral at 0x%08x: %m\n", base);
    }

    close(fd);
    return vaddr;
}

// Method to init DMA control block
void RCInput_RPI::init_dma_cb(dma_cb_t **cbp, uint32_t mode, uint32_t source, uint32_t dest, uint32_t length, uint32_t stride, uint32_t next_cb)
{
    (*cbp)->info = mode;
    (*cbp)->src = source;
    (*cbp)->dst = dest;
    (*cbp)->length = length;
    (*cbp)->next = next_cb;
    (*cbp)->stride = stride;
}

void RCInput_RPI::stop_dma()
{
    dma_reg[RCIN_RPI_DMA_CS | RCIN_RPI_DMA_CHANNEL << 8] = 0;
}

/* We need to be sure that the DMA is stopped upon termination */
void RCInput_RPI::termination_handler(int signum)
{
    stop_dma();
    AP_HAL::panic("Interrupted: %s", strsignal(signum));
}

// This function is used to init DMA control blocks (setting sampling GPIO
// register, destination adresses, synchronization)
void RCInput_RPI::init_ctrl_data()
{
    uint32_t phys_fifo_addr;
    uint32_t dest = 0;
    uint32_t cbp = 0;
    dma_cb_t *cbp_curr;
    // Set fifo addr (for delay)
    phys_fifo_addr = ((pcm_base + 0x04) & 0x00FFFFFF) | 0x7e000000;

    // Init dma control blocks.
    /*We are transferring 1 byte of GPIO register. Every 56th iteration we are 
      sampling TIMER register, which length is 8 bytes. So, for every 56 samples of GPIO we need 
      56 * 1 + 8 = 64 bytes of buffer. Value 56 was selected specially to have a 64-byte "block" 
      TIMER - GPIO. So, we have integer count of such "blocks" at one virtual page. (4096 / 64 = 64 
      "blocks" per page. As minimum, we must have 2 virtual pages of buffer (to have integer count of 
      vitual pages for control blocks): for every 56 iterations (64 bytes of buffer) we need 56 control blocks for GPIO
      sampling, 56 control blocks for setting frequency and 1 control block for sampling timer, so,
      we need 56 + 56 + 1 = 113 control blocks. For integer value, we need 113 pages of control blocks.
      Each control block length is 32 bytes. In 113 pages we will have (113 * 4096 / 32) = 113 * 128 control
      blocks. 113 * 128 control blocks = 64 * 128 bytes of buffer = 2 pages of buffer.
      So, for 56 * 64 * 2 iteration we init DMA for sampling GPIO
      and timer to (64 * 64 * 2) = 8192 bytes = 2 pages of buffer.
    */

    for (uint32_t i = 0; i < 56 * 128 * RCIN_RPI_BUFFER_LENGTH; i++) {
        //Transfer timer every 56th sample
        if (i % 56 == 0) {
            cbp_curr = (dma_cb_t *)con_blocks->get_page(con_blocks->_virt_pages, cbp);

            init_dma_cb(&cbp_curr, RCIN_RPI_DMA_NO_WIDE_BURSTS | RCIN_RPI_DMA_WAIT_RESP | RCIN_RPI_DMA_DEST_INC | RCIN_RPI_DMA_SRC_INC, RCIN_RPI_TIMER_BASE,
                        (uintptr_t)circle_buffer->get_page(circle_buffer->_phys_pages, dest),
                        8,
                        0,
                        (uintptr_t)con_blocks->get_page(con_blocks->_phys_pages,
                                                        cbp + sizeof(dma_cb_t)));

            dest += 8;
            cbp += sizeof(dma_cb_t);
        }

        // Transfer GPIO (1 byte)
        cbp_curr = (dma_cb_t *)con_blocks->get_page(con_blocks->_virt_pages, cbp);
        init_dma_cb(&cbp_curr, RCIN_RPI_DMA_NO_WIDE_BURSTS | RCIN_RPI_DMA_WAIT_RESP, RCIN_RPI_GPIO_LEV0_ADDR,
                    (uintptr_t)circle_buffer->get_page(circle_buffer->_phys_pages, dest),
                    1,
                    0,
                    (uintptr_t)con_blocks->get_page(con_blocks->_phys_pages,
                                                    cbp + sizeof(dma_cb_t)));

        dest += 1;
        cbp += sizeof(dma_cb_t);

        // Delay (for setting sampling frequency)
        /* DMA is waiting data request signal (DREQ) from PCM. PCM is set for 1 MhZ freqency, so,
	       each sample of GPIO is limited by writing to PCA queue.
	    */
        cbp_curr = (dma_cb_t *)con_blocks->get_page(con_blocks->_virt_pages, cbp);
        init_dma_cb(&cbp_curr, RCIN_RPI_DMA_NO_WIDE_BURSTS | RCIN_RPI_DMA_WAIT_RESP | RCIN_RPI_DMA_D_DREQ | RCIN_RPI_DMA_PER_MAP(2),
                    RCIN_RPI_TIMER_BASE, phys_fifo_addr,
                    4,
                    0,
                    (uintptr_t)con_blocks->get_page(con_blocks->_phys_pages,
                                                    cbp + sizeof(dma_cb_t)));

        cbp += sizeof(dma_cb_t);
    }
    //Make last control block point to the first (to make circle)
    cbp -= sizeof(dma_cb_t);
    ((dma_cb_t *)con_blocks->get_page(con_blocks->_virt_pages, cbp))->next = (uintptr_t)con_blocks->get_page(con_blocks->_phys_pages, 0);
}

/*Initialise PCM
  See BCM2835 documentation:
  http://www.raspberrypi.org/wp-content/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
 */
void RCInput_RPI::init_PCM()
{
    pcm_reg[RCIN_RPI_PCM_CS_A] = 1;                                          // Disable Rx+Tx, Enable PCM block
    hal.scheduler->delay_microseconds(100);
    clk_reg[RCIN_RPI_PCMCLK_CNTL] = 0x5A000006;                              // Source=PLLD (500MHz)
    hal.scheduler->delay_microseconds(100);
    clk_reg[RCIN_RPI_PCMCLK_DIV] = 0x5A000000 | ((50000/RCIN_RPI_SAMPLE_FREQ)<<12);   // Set pcm div. If we need to configure DMA frequency.
    hal.scheduler->delay_microseconds(100);
    clk_reg[RCIN_RPI_PCMCLK_CNTL] = 0x5A000016;                              // Source=PLLD and enable
    hal.scheduler->delay_microseconds(100);
    pcm_reg[RCIN_RPI_PCM_TXC_A] = 0<<31 | 1<<30 | 0<<20 | 0<<16;             // 1 channel, 8 bits
    hal.scheduler->delay_microseconds(100);
    pcm_reg[RCIN_RPI_PCM_MODE_A] = (10 - 1) << 10;                           //PCM mode
    hal.scheduler->delay_microseconds(100);
    pcm_reg[RCIN_RPI_PCM_CS_A] |= 1<<4 | 1<<3;                               // Clear FIFOs
    hal.scheduler->delay_microseconds(100);
    pcm_reg[RCIN_RPI_PCM_DREQ_A] = 64<<24 | 64<<8;                           // DMA Req when one slot is free?
    hal.scheduler->delay_microseconds(100);
    pcm_reg[RCIN_RPI_PCM_CS_A] |= 1<<9;                                      // Enable DMA
    hal.scheduler->delay_microseconds(100);
    pcm_reg[RCIN_RPI_PCM_CS_A] |= 1<<2;                                      // Enable Tx
    hal.scheduler->delay_microseconds(100);
}

/*Initialise DMA
  See BCM2835 documentation:
  http://www.raspberrypi.org/wp-content/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
 */
void RCInput_RPI::init_DMA()
{
    dma_reg[RCIN_RPI_DMA_CS | RCIN_RPI_DMA_CHANNEL << 8] = RCIN_RPI_DMA_RESET;                 //Reset DMA
    hal.scheduler->delay_microseconds(100);
    dma_reg[RCIN_RPI_DMA_CS | RCIN_RPI_DMA_CHANNEL << 8] = RCIN_RPI_DMA_INT | RCIN_RPI_DMA_END;
    dma_reg[RCIN_RPI_DMA_CONBLK_AD | RCIN_RPI_DMA_CHANNEL << 8] = reinterpret_cast<uintptr_t>(con_blocks->get_page(con_blocks->_phys_pages, 0));//Set first control block address
    dma_reg[RCIN_RPI_DMA_DEBUG | RCIN_RPI_DMA_CHANNEL << 8] = 7;                      // clear debug error flags
    dma_reg[RCIN_RPI_DMA_CS | RCIN_RPI_DMA_CHANNEL << 8] = 0x10880001;                // go, mid priority, wait for outstanding writes    
}


// We must stop DMA when the process is killed
void RCInput_RPI::set_sigaction()
{
    struct sigaction sa, sa_old;

    memset(&sa_old, 0, sizeof(sa));
    memset(&sa, 0, sizeof(sa));

    /* Ignore signals */
    sa.sa_handler = SIG_IGN;
    sigaction(SIGWINCH, &sa, nullptr);
    sigaction(SIGTTOU, &sa, nullptr);
    sigaction(SIGTTIN, &sa, nullptr);

    /*
     * Catch all other signals to ensure DMA is disabled - some of them may
     * already be handled elsewhere in cases we consider normal termination.
     * In those cases the teardown() method must be called.
     */
    for (int i = 0; i < NSIG; i++) {
        sigaction(i, nullptr, &sa_old);

        if (sa_old.sa_handler == nullptr) {
            sa.sa_handler = RCInput_RPI::termination_handler;
            sigaction(i, &sa, nullptr);
        }
    }
}

// Initial setup of variables
RCInput_RPI::RCInput_RPI():
    circle_buffer{nullptr},
    con_blocks{nullptr},
    prev_tick(0),
    delta_time(0),
    curr_tick_inc(1000/RCIN_RPI_SAMPLE_FREQ),
    curr_pointer(0),
    curr_channel(0),
    width_s0(0),
    curr_signal(0),
    last_signal(228),
    state(RCIN_RPI_INITIAL_STATE)
{
}

RCInput_RPI::~RCInput_RPI()
{
    delete circle_buffer;
    delete con_blocks;
}

void RCInput_RPI::teardown()
{
    stop_dma();
}

//Initializing necessary registers
void RCInput_RPI::init_registers()
{
    dma_reg = (uint32_t *)map_peripheral(dma_base, RCIN_RPI_DMA_LEN);
    pcm_reg = (uint32_t *)map_peripheral(pcm_base, RCIN_RPI_PCM_LEN);
    clk_reg = (uint32_t *)map_peripheral(clk_base, RCIN_RPI_CLK_LEN);
}

void RCInput_RPI::init()
{
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2
    int version = 2;
#else
    int version = UtilRPI::from(hal.util)->get_rpi_version();
#endif
    set_physical_addresses(version);
    circle_buffer = new Memory_table(RCIN_RPI_BUFFER_LENGTH * 2, version);
    con_blocks = new Memory_table(RCIN_RPI_BUFFER_LENGTH * 113, version);

    init_registers();

    // Enable PPM input
    enable_pin = hal.gpio->channel(PPM_INPUT_RPI);
    enable_pin->mode(HAL_GPIO_INPUT);

    // Configuration
    set_sigaction();
    init_ctrl_data();
    init_PCM();
    init_DMA();

    // Wait a bit to let DMA fill queues and come to stable sampling
    hal.scheduler->delay(300);

    // Reading first sample
    curr_tick = *((uint64_t *)circle_buffer->get_page(circle_buffer->_virt_pages, curr_pointer));
    prev_tick = curr_tick;
    curr_pointer += 8;
    curr_signal = *((uint8_t *)circle_buffer->get_page(circle_buffer->_virt_pages, curr_pointer)) & 0x10 ? 1 : 0;
    last_signal = curr_signal;
    curr_pointer++;

    _initialized = true;
}

// Processing signal
void RCInput_RPI::_timer_tick()
{
    uint32_t counter = 0;

    if (!_initialized) {
        return;
    }

    // Now we are getting address in which DMAC is writing at current moment
    dma_cb_t *ad = (dma_cb_t *)con_blocks->get_virt_addr(dma_reg[RCIN_RPI_DMA_CONBLK_AD | RCIN_RPI_DMA_CHANNEL << 8]);
    if (!ad) {
        debug("DMA sampling stopped, restarting...\n");
        init_ctrl_data();
        init_PCM();
        init_DMA();
        return;
    }

    for (int j = 1; j >= -1; j--) {
        void *x = circle_buffer->get_virt_addr((ad + j)->dst);
        if (x != nullptr) {
            counter = circle_buffer->bytes_available(curr_pointer,
                                                     circle_buffer->get_offset(circle_buffer->_virt_pages, (uintptr_t)x));
            break;
        }
    }

    if (counter == 0) {
        return;
    }

    // How many bytes have DMA transferred (and we can process)?
    // We can't stay in method for a long time, because it may lead to delays
    if (counter > RCIN_RPI_MAX_COUNTER) {
        debug("%5d sample(s) dropped\n", (counter - RCIN_RPI_MAX_COUNTER) / 0x8);
        counter = RCIN_RPI_MAX_COUNTER;
    }

    // Processing ready bytes
    for (; counter > 0x40; counter--) {
        // Is it timer sample?
        if (curr_pointer % (64) == 0) {
            curr_tick = *((uint64_t *)circle_buffer->get_page(circle_buffer->_virt_pages, curr_pointer));
            curr_pointer += 8;
            counter -= 8;
        }

        // Reading required bit
        curr_signal = *((uint8_t *)circle_buffer->get_page(circle_buffer->_virt_pages, curr_pointer)) & 0x10 ? 1 : 0;

        // If the signal changed
        if (curr_signal != last_signal) {
            delta_time = curr_tick - prev_tick;
            prev_tick = curr_tick;
            switch (state) {
            case RCIN_RPI_INITIAL_STATE:
                state = RCIN_RPI_ZERO_STATE;
                break;
            case RCIN_RPI_ZERO_STATE:
                if (curr_signal == 0) {
                    width_s0 = (uint16_t)delta_time;
                    state = RCIN_RPI_ONE_STATE;
                }
                break;
            case RCIN_RPI_ONE_STATE:
                if (curr_signal == 1) {
                    width_s1 = (uint16_t)delta_time;
                    state = RCIN_RPI_ZERO_STATE;
                    _process_rc_pulse(width_s0, width_s1);
                }
                break;
            }
        }

        last_signal = curr_signal;
        curr_pointer++;
        if (curr_pointer >= circle_buffer->get_page_count() * PAGE_SIZE) {
            curr_pointer = 0;
        }
        curr_tick += curr_tick_inc;
    }
}

#endif
