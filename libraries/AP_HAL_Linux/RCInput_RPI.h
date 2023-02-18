#pragma once

#include "AP_HAL_Linux.h"
#include "RCInput.h"
#include <signal.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <assert.h>
#include <queue>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH
#define RCIN_RPI_CHN_NUM 8
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_OBAL_V1
#define RCIN_RPI_CHN_NUM 8
#else
#define RCIN_RPI_CHN_NUM 1
#endif

namespace Linux {

enum state_t{
    RCIN_RPI_INITIAL_STATE = -1,
    RCIN_RPI_ZERO_STATE = 0,
    RCIN_RPI_ONE_STATE = 1
};

//Memory table structure
typedef struct {
    void **virt_pages;
    void **phys_pages;
    uint32_t page_count;
} memory_table_t;


//DMA control block structure
typedef struct {
  uint32_t info, src, dst, length,
    stride, next, pad[2];
} dma_cb_t;

class Memory_table {
// Allow RCInput_RPI access to private members of Memory_table
friend class RCInput_RPI;

private:
    void** _virt_pages;
    void** _phys_pages;
    uint32_t _page_count;

public:
    Memory_table();
    Memory_table(uint32_t, int);
    ~Memory_table();

    //Get virtual address from the corresponding physical address from memory_table.
    void* get_virt_addr(const uint32_t phys_addr) const;

    // This function returns physical address with help of pointer, which is offset from the beginning of the buffer.
    void* get_page(void **pages, const uint32_t addr) const;

    // This function returns offset from the beginning of the buffer using (virtual) address in 'pages' and memory_table.
    uint32_t get_offset(void **pages, const uint64_t addr) const;

    //How many bytes are available for reading in circle buffer?
    uint32_t bytes_available(const uint32_t read_addr, const uint32_t write_addr) const;

    uint32_t get_page_count() const;
};


class RCInput_RPI : public RCInput
{
public:
    void init() override;
    void _timer_tick(void) override;
    RCInput_RPI();
    ~RCInput_RPI();

private:

    //Physical adresses of peripherals. Are different on different Raspberries.
    uint32_t dma_base;
    uint32_t clk_base;
    uint32_t pcm_base;

    //registers
    static volatile uint32_t *pcm_reg;
    static volatile uint32_t *clk_reg;
    static volatile uint32_t *dma_reg;

    Memory_table *circle_buffer;
    Memory_table *con_blocks;

    uint64_t curr_tick;

    uint32_t curr_tick_inc;
    uint32_t curr_pointer;
    uint32_t curr_channel;

    struct RcChannel {
        RcChannel() :
            prev_tick(0), delta_time(0),
            width_s0(0), width_s1(0),
            curr_signal(0), last_signal(0),
            enable_pin(0), state(RCIN_RPI_INITIAL_STATE)
        {}

        uint64_t prev_tick;
        uint64_t delta_time;

        uint16_t width_s0;
        uint16_t width_s1;

        uint8_t curr_signal;
        uint8_t last_signal;

        state_t state;

        AP_HAL::DigitalSource *enable_pin;
    } rc_channels[RCIN_RPI_CHN_NUM];

    bool _initialized = false;
    int _version =0;
    
    void init_dma_cb(dma_cb_t** cbp, uint32_t mode, uint32_t source, uint32_t dest, uint32_t length, uint32_t stride, uint32_t next_cb);
    void* map_peripheral(uint32_t base, uint32_t len);
    void init_registers();
    void init_ctrl_data();
    void init_PCM();
    void init_DMA();
    void init_buffer();
    static void stop_dma();
    static void termination_handler(int signum);
    void set_sigaction();
    void set_physical_addresses();
    void teardown() override;
};

}
