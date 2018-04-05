
namespace OSDns {// OSD interface emulates UART

    void osd_begin(AP_HAL::OwnPtr<F4Light::SPIDevice> spi);
    void osd_loop();

    int16_t osd_available();

    int16_t osd_getc();
    void osd_dequeue();

    uint32_t osd_txspace();
    void osd_putc(uint8_t c); 

    void max_do_transfer(const uint8_t *buffer, uint16_t len);
    void update_max_buffer(const uint8_t *buffer, uint16_t len);

    inline uint32_t millis(){ return AP_HAL::millis(); }
    
    class BetterStream;
}
