#pragma once

#include "AP_HAL_RP.h"
#include <string.h>

namespace RP {

#define CACHED_SECTORS 3

struct SectorCache {
    uint8_t buf[512];
    uint32_t block;
    uint32_t page;
    uint32_t sector;
    uint8_t prio;
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
    uint32_t m_maxProgTime;
    uint32_t m_maxEraseTime;
    uint32_t m_maxReadTime;
    uint32_t m_totalSize;
public:
    NAND_Chip(): m_manufacturer{0}, m_model{0}, m_manufacturerId{0},
        m_pageSize{0}, m_spareSize{0}, m_pageCount{0}, m_blockCount{0}, 
        m_maxProgTime{0}, m_maxEraseTime{0}, m_maxReadTime{0}, m_totalSize{0}
    {}

    const char *manufacturer() const
    { return static_cast<const char *>(m_manufacturer); }

    const char *model() const
    { return static_cast<const char *>(m_manufacturer); }

    uint8_t manufacturerId() const
    { return m_manufacturerId; }

    uint32_t pageSize() const
    { return m_pageSize; }

    uint32_t spareSize() const
    { return m_spareSize; }

    uint32_t pageCount() const
    { return m_pageCount; }

    uint32_t blockCount() const
    { return m_blockCount; }

    uint32_t maxProgTime() const
    { return m_maxProgTime; }

    uint32_t maxEraseTime() const
    { return m_maxEraseTime; }

    uint32_t maxReadTime() const
    { return m_maxReadTime; }

    uint32_t totalSize() const
    { return m_totalSize; }

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
    HAL_Semaphore m_sem;

    uint8_t m_dmaTxChannel, m_dmaRxChannel;

    uint32_t m_cachedBlock;
    uint32_t m_cachedPage;

    SectorCache m_secCashed[CACHED_SECTORS];

    uint8_t spi_write(uint8_t data = 0);
    bool burst_spi_write(uint16_t len, const uint8_t *src);
    bool burst_spi_read(uint16_t len, uint8_t *dst);

    bool check_read_id();
    void set_feature(uint8_t featureRegister, uint8_t data);
    uint8_t get_feature(uint8_t featureRegister);
    bool check_feature(uint8_t mask, uint8_t value, uint8_t featureRegister);

    uint8_t get_status();

    void read_page(uint32_t block, uint32_t page, bool getFeatureWait = true);
    uint32_t read_from_cache(uint32_t block, uint32_t start, uint16_t length, uint8_t *buf);

    uint16_t program_load(uint32_t block, uint32_t start, uint16_t length, const uint8_t *buf);
    void program_execute(uint32_t block, uint32_t page, bool getFeatureWait = true);

    void wait_feature(bool getFeatureWait);

    void invalidate_caches();

public:
    NAND_PIO_Driver();
    bool init(uint8_t ioBase, uint8_t sckPin, uint8_t csPin);
    bool read_page(uint32_t block, uint32_t page, uint8_t *data);
    bool write_page(uint32_t block, uint32_t page, const uint8_t *data);
    void erase_block(uint32_t block, bool getFeatureWait = true);

    void write_enable();
    void write_disable();

    bool is_busy();

    AP_HAL::Semaphore* get_semaphore() {
        return &m_sem;
    }
    uint32_t get_manufacturer_id() {
        return static_cast<uint32_t>(m_chip.manufacturerId());
    }
    uint32_t get_page_size() {
        return static_cast<uint32_t>(m_chip.pageSize());
    }
    uint32_t get_block_size() {
        return static_cast<uint32_t>(m_chip.pageSize() * m_chip.pageCount());
    }
    uint32_t get_block_count() {
        return static_cast<uint32_t>(m_chip.blockCount());
    }
};

} // namespace RP

inline uint32_t DECODE_U4(const uint8_t *buf) {
    uint32_t result;
    memcpy(&result, buf, 4); // memcpy needed because of 4-byte-alignment
    return result;
}
inline int32_t DECODE_I4(const uint8_t *buf) {
    int32_t result;
    memcpy(&result, buf, 4);
    return result;
}
inline float DECODE_R4(const uint8_t *buf) {
    float result;
    memcpy(&result, buf, 4);
    return result;
}
inline int64_t DECODE_I8(const uint8_t *buf) {
    int64_t result;
    memcpy(&result, buf, 8);
    return result;
}
inline double DECODE_R8(const uint8_t *buf) {
    double result;
    memcpy(&result, buf, 8);
    return result;
}
