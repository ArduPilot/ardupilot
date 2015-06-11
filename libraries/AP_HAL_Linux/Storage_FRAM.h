#ifndef __AP_HAL_LINUX_STORAGE_FRAM_H__
#define __AP_HAL_LINUX_STORAGE_FRAM_H__

#include <AP_HAL.h>
#include "AP_HAL_Linux_Namespace.h"

#define OPCODE_WREN   0b0110        /* Write Enable Latch */
#define OPCODE_WRDI   0b0100        /* Reset Write Enable Latch */
#define OPCODE_RDSR   0b0101        /* Read Status Register */
#define OPCODE_WRSR   0b0001        /* Write Status Register */
#define OPCODE_READ   0b0011        /* Read Memory */
#define OPCODE_WRITE  0b0010        /* Write Memory */
#define OPCODE_RDID   0b10011111    /* Read Device ID */

class Linux::LinuxStorage_FRAM : public Linux::LinuxStorage 
{
public:
    LinuxStorage_FRAM();
    void _timer_tick(void);

private:
    uint32_t fptr;
   
    int32_t write(uint16_t fd, uint8_t *Buff, uint16_t NumBytes);
    int32_t read(uint16_t fd, uint8_t *Buff, uint16_t NumBytes);
    uint32_t lseek(uint16_t fd,uint32_t offset,uint16_t whence);
    int8_t open();

    int8_t _register_write( uint8_t* src, uint16_t addr, uint16_t len );
    int8_t _register_read( uint16_t addr, uint8_t opcode );
    int8_t _write_enable(bool enable);
    int8_t transaction(uint8_t* tx, uint8_t* rx, uint16_t len);

    void _storage_create(void);
    void _storage_open(void);

    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;
};
#endif // __AP_HAL_LINUX_STORAGE_H__
