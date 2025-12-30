#pragma once

#include "AP_HAL_RP.h"
#include <string.h>

namespace RP {

#define CACHED_SECTORS 3

struct SectorCache {
    u8 buf[512];
    u32 block;
    u32 page;
    u32 sector;
    u8 prio;
    SectorCache():
    buf{0}, block{0xFFFFFFFF}, page{0xFFFFFFFF}, sector{0xFFFFFFFF}, prio{0xFF}
    {}
};

class NAND_Chip {
private:
    char m_manufacturer[13];
    char m_model[21];
    uint8_t m_manufacturerId;
    uint32_t m_pageSize;
    uint32_t m_spareSize;
    uint32_t m_pageCount;
    uint32_t m_blockCount;
    uint32_t m_maxBbBlock;
    uint32_t m_maxProgTime;
    uint32_t m_maxEraseTime;
    uint32_t m_maxReadTime;
    uint32_t m_totalSize;
public:
    NAND_Chip(): m_manufacturer{0}, m_model{0}, m_manufacturerId{0},
        m_pageSize{0}, m_spareSize{0}, m_pageCount{0}, m_blockCount{0}, 
        m_maxBbBlock{0}, m_maxProgTime{0}, m_maxEraseTime{0}, m_maxReadTime{0}, m_totalSize{0} 
    {}

    const char *manufacturer() const
    { return static_cast<const char *>(m_manufacturer); }

    const char *model() const
    { return static_cast<const char *>(m_manufacturer); }

    const uint8_t manufacturerId() const
    { return static_cast<const uint8_t>(m_manufacturerId); }

    const uint32_t pageSize() const
    { return static_cast<const uint32_t>(m_pageSize); }

    const uint32_t spareSize() const
    { return static_cast<const uint32_t>(m_spareSize); }

    const uint32_t pageCount() const
    { return static_cast<const uint32_t>(m_pageCount); }

    const uint32_t blockCount() const
    { return static_cast<const uint32_t>(m_blockCount); }

    const uint32_t maxBbBlock() const
    { return static_cast<const uint32_t>(m_maxBbBlock); }

    const uint32_t maxProgTime() const
    { return static_cast<const uint32_t>(m_maxProgTime); }

    const uint32_t maxEraseTime() const
    { return static_cast<const uint32_t>(m_maxEraseTime); }

    const uint32_t maxReadTime() const
    { return static_cast<const uint32_t>(m_maxReadTime); }

    const uint32_t totalSize() const
    { return static_cast<const uint32_t>(m_totalSize); }

    void manufacturer(const char *v)
    { strncpy(m_manufacturer, v, 12); }

    void model(const char *v)
    { strncpy(m_model, v, 20); }

    void manufacturerId(uint8_t v)
    { m_manufacturerId = v; }

    void pageSize(uint32_t v)
    { m_pageSize = v; }

    void spareSize(uint32_t v)
    { m_spareSize = v; }

    void pageCount(uint32_t v)
    { m_pageCount = v; }

    void blockCount(uint32_t v)
    { m_blockCount = v; }

    void maxBbBlock(uint32_t v)
    { m_maxBbBlock = v; }

    void maxProgTime(uint32_t v)
    { m_maxProgTime = v; }

    void maxEraseTime(uint32_t v)
    { m_maxEraseTime = v; }

    void maxReadTime(uint32_t v)
    { m_maxReadTime = v; }

    void totalSize(uint32_t v)
    { m_totalSize = v; }
};

class NAND_PIO_Driver {
private:
    NAND_Chip m_chip;
    uint8_t m_smId;
    uint8_t m_offset;
    bool m_ready;

    uint8_t m_dmaTxChannel, m_dmaRxChannel;

    uint32_t m_cachedBlock;
    uint32_t m_cachedPage;

    SectorCache m_secCashed[CACHED_SECTORS];

    uint8_t spi_write(uint8_t data = 0);
    bool burst_spi_write(uint16_t len, const uint8_t *src);
    bool burst_spi_read(u16 len, u8 *dst);

    //void spi_read(uint8_t* buffer, size_t len);
    //void select_chip();
    //void deselect_chip();

    bool check_read_id();
    void set_feature(uint8_t featureRegister, uint8_t data);
    u8 get_feature(u8 featureRegister);
    bool check_feature(u8 mask, u8 value, u8 featureRegister);

    void read_page(uint32_t block, uint32_t page, bool getFeatureWait = true);
    u32 read_from_cache(u32 block, u32 start, u16 length, u8 *buf);
    void erase_block(u32 block, bool getFeatureWait = true);

    void write_enable();
    void write_disable();

    u16 program_load(u32 block, u32 start, u16 length, const u8 *buf);
    void program_execute(u32 block, u32 page, bool getFeatureWait = true);

    void wait_feature(bool getFeatureWait);

    void invalidate_caches();

public:
    NAND_PIO_Driver();
    bool init(pin_size_t ioBase, pin_size_t sckPin, pin_size_t csPin);
    bool read_page(uint32_t block, uint32_t page, uint8_t *data);
    bool write_page(uint32_t block, uint32_t page, const uint8_t *data);
    bool erase_block(uint32_t block);
};

} // namespace RP

inline u32 DECODE_U4(const u8 *buf) {
    u32 result;
    memcpy(&result, buf, 4); // memcpy needed because of 4-byte-alignment
    return result;
}
inline i32 DECODE_I4(const u8 *buf) {
    i32 result;
    memcpy(&result, buf, 4);
    return result;
}
inline f32 DECODE_R4(const u8 *buf) {
    f32 result;
    memcpy(&result, buf, 4);
    return result;
}
inline i64 DECODE_I8(const u8 *buf) {
    i64 result;
    memcpy(&result, buf, 8);
    return result;
}
inline f64 DECODE_R8(const u8 *buf) {
    f64 result;
    memcpy(&result, buf, 8);
    return result;
}
