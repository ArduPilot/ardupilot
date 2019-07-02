/*
  bouncebuffer code for supporting ChibiOS drivers that use DMA, but may be passed memory that
  is not DMA safe. 

  This API assumes that there will be only one user of a bouncebuffer at a time
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

struct bouncebuffer_t {
    uint8_t *dma_buf;
    uint8_t *orig_buf;
    uint32_t size;
    bool busy;
    bool is_sdcard;
};

/*
  initialise a bouncebuffer
 */
void bouncebuffer_init(struct bouncebuffer_t **bouncebuffer, uint32_t prealloc_bytes, bool sdcard);

/*
  setup for reading from a device into memory, allocating a bouncebuffer if needed
 */
void bouncebuffer_setup_read(struct bouncebuffer_t *bouncebuffer, uint8_t **buf, uint32_t size);
    
/*
  finish a read operation
 */
void bouncebuffer_finish_read(struct bouncebuffer_t *bouncebuffer, const uint8_t *buf, uint32_t size);
    
/*
  setup for reading from memory to a device, allocating a bouncebuffer if needed
 */
void bouncebuffer_setup_write(struct bouncebuffer_t *bouncebuffer, const uint8_t **buf, uint32_t size);
    
/*
  finish a write operation
 */
void bouncebuffer_finish_write(struct bouncebuffer_t *bouncebuffer, const uint8_t *buf);

#ifdef __cplusplus
}
#endif
