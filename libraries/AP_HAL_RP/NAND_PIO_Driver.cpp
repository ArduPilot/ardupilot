#include "NAND_PIO_Driver.h"
#include "extended_spi.pio.h"
#include <string.h>

using namespace RP;

enum FlashCmd {
    FLASH_CMD_RESET = 0xFF,
    FLASH_CMD_GET_FEATURE = 0x0F,
    FLASH_CMD_SET_FEATURE = 0x1F,
    FLASH_CMD_READ_ID = 0x9F,
    FLASH_CMD_PAGE_READ = 0x13,
    FLASH_CMD_READ_FROM_CACHE_X1 = 0x03,
    FLASH_CMD_READ_FROM_CACHE_X2 = 0x3B,
    FLASH_CMD_READ_FROM_CACHE_DUAL_IO = 0xBB,
    FLASH_CMD_WRITE_ENABLE = 0x06,
    FLASH_CMD_WRITE_DISABLE = 0x04,
    FLASH_CMD_BLOCK_ERASE = 0xD8,
    FLASH_CMD_PROGRAM_EXECUTE = 0x10,
    FLASH_CMD_PROGRAM_LOAD_X1 = 0x02,
    FLASH_CMD_PROGRAM_LOAD_RANDOM_DATA_X1 = 0x84,
};

//----------------------- NAND_PIO_Driver ----------------------------- 

NAND_PIO_Driver::NAND_PIO_Driver():
    m_chip{}, m_smId{0}, m_ready{false}, m_dmaTxChannel{0}, m_dmaRxChannel{0},
    m_cachedBlock{0xFFFFFFFF}, m_cachedPage{0xFFFFFFFF}, m_secCashed{}
{}

bool NAND_PIO_Driver::init(pin_size_t ioBase, pin_size_t sckPin, pin_size_t csPin)
{
    // set up SPI
    gpio_init(csPin);
    gpio_set_dir(csPin, GPIO_OUT);
    gpio_put(csPin, true);
    gpio_set_slew_rate(csPin, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(sckPin, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(ioBase, GPIO_SLEW_RATE_FAST);
    m_smId = pio_claim_unused_sm(PIO_EXT_SPI_BB, true);
    DEV_PRINTF("Got state machine Id %d\n", m_smId);
    m_offset = pio_add_program(PIO_EXT_SPI_BB, &extended_spi_program);
    pio_spi_init(PIO_EXT_SPI_BB, m_smId, m_offset, 8, 1, sckPin, ioBase, ioBase + 1);

    // set up DMA channels for RX/TX
    m_dmaTxChannel = dma_claim_unused_channel(true);
    m_dmaRxChannel = dma_claim_unused_channel(true);
    dma_channel_config flashDmaTxConfig = dma_channel_get_default_config(m_dmaTxChannel);
    dma_channel_config flashDmaRxConfig = dma_channel_get_default_config(m_dmaRxChannel);
    channel_config_set_read_increment(&flashDmaTxConfig, true);
    channel_config_set_write_increment(&flashDmaTxConfig, false);
    channel_config_set_read_increment(&flashDmaRxConfig, false);
    channel_config_set_write_increment(&flashDmaRxConfig, true);
    channel_config_set_dreq(&flashDmaTxConfig, pio_get_dreq(PIO_EXT_SPI_BB, m_smId, true));
    channel_config_set_dreq(&flashDmaRxConfig, pio_get_dreq(PIO_EXT_SPI_BB, m_smId, false));
    channel_config_set_transfer_data_size(&flashDmaTxConfig, DMA_SIZE_8);
    channel_config_set_transfer_data_size(&flashDmaRxConfig, DMA_SIZE_8);
    dma_channel_set_config(m_dmaTxChannel, &flashDmaTxConfig, false);
    dma_channel_set_config(m_dmaRxChannel, &flashDmaRxConfig, false);
    dma_channel_set_write_addr(m_dmaTxChannel, &PIO_EXT_SPI_BB->txf[m_smId], false);
    dma_channel_set_read_addr(m_dmaRxChannel, &PIO_EXT_SPI_BB->rxf[m_smId], false);

    // find flash chip
    for (int i = 0; i < 50; i++) {
        if (check_read_id()) break;
        if (i == 49) return false;
        sleep_ms(2);
    }

    for (int i = 0; i < 50; i++) {
        if (check_read_id()) break;
        if (i == 49) return false;
        sleep_ms(2);
    }
    DEV_PRINTF("Found flash chip");

    // prepare flash chip
    set_feature(0xA0, 0x00); // disable block lock
    set_feature(0xB0, 0x50); // ECC enabled, access OTP/Parameter/UID
    set_feature(0xD0, 0x00); // default, just make sure we have selected die 0
    DEV_PRINTF("Disabled block lock, flash chip is functional");

    // read parameter page
    read_page(0, 1);
    u8 *buf = (u8 *)malloc(2176);
    if (buf == nullptr) return false;
    read_from_cache(0, 0, 256, buf);

    m_chip.manufacturer((char *)&buf[32]);
    m_chip.model((char *)&buf[44]);
    m_chip.manufacturerId(buf[64]);
    m_chip.pageSize(DECODE_U4(&buf[80]));
    m_chip.spareSize(DECODE_U2(&buf[84]));
    m_chip.pageCount(DECODE_U4(&buf[92]));
    m_chip.blockCount(DECODE_U4(&buf[96]));
    m_chip.maxBbBlock(m_chip.blockCount() - 40);
    m_chip.maxProgTime(DECODE_U2(&buf[133]));
    m_chip.maxEraseTime(DECODE_U2(&buf[135]));
    m_chip.maxReadTime(DECODE_U2(&buf[137]));

    if (m_chip.pageSize() != 2048) {
        DEV_PRINTF("Page size refused %d\n", m_chip.pageSize());
        free(buf);
        return false;
    }

    m_chip.totalSize(m_chip.pageSize() * m_chip.pageCount() * m_chip.blockCount());

    set_feature(0xB0, 0x10);
    invalidate_caches();

    // Accept Micron devices up to 4 Gb nominal size (512 MiB)
    if (m_chip.manufacturerId() != 0x2C || !m_chip.totalSize() || m_chip.totalSize() > 512 * 1024 * 1024) {
        DEV_PRINTF("Manufacturer ID %d, totalSize %d\n", m_chip.manufacturerId(), m_chip.totalSize());
        free(buf);
        return false;
    }

    m_ready = true;

    free(buf);
    return true;
}

uint8_t NAND_PIO_Driver::spi_write(uint8_t data)
{
    pio_sm_clear_fifos(PIO_EXT_SPI_BB, m_smId);
    pio_sm_put_blocking(PIO_EXT_SPI_BB, m_smId, ((uint32_t)data) << 24);
    return pio_sm_get_blocking(PIO_EXT_SPI_BB, m_smId);
}

bool NAND_PIO_Driver::check_read_id()
{
    gpio_put(PIN_FLASH_CS, false);
    spi_write(FLASH_CMD_READ_ID);
    spi_write(); // dummy byte
    uint8_t read0 = spi_write();
    uint8_t read1 = spi_write();
    gpio_put(PIN_FLASH_CS, true);
    return read0 == 0x2c && read1 == 0x24;
}

void NAND_PIO_Driver::set_feature(uint8_t featureRegister, uint8_t data)
{
    gpio_put(PIN_FLASH_CS, false);
    singleSpiTransfer(FLASH_CMD_SET_FEATURE);
    singleSpiTransfer(featureRegister);
    singleSpiTransfer(data);
    gpio_put(PIN_FLASH_CS, true);
}

u8 NAND_PIO_Driver::get_feature(u8 featureRegister)
{
    gpio_put(PIN_FLASH_CS, false);
    spi_write(FLASH_CMD_GET_FEATURE);
    spi_write(featureRegister);
    u8 ret = spi_write();
    gpio_put(PIN_FLASH_CS, true);
    return ret;
}

bool NAND_PIO_Driver::check_feature(u8 mask, u8 value, u8 featureRegister)
{
    u8 read = get_feature();
    return (read & mask) == value;
}

bool NAND_PIO_Driver::burst_spi_write(uint16_t len, const uint8_t *src)
{
    if (src == nullptr) return false;
    uint8_t *dummy = (uint8_t *)malloc(len);
    if (dummy == nullptr) return false;
    pio_sm_clear_fifos(PIO_EXT_SPI_BB, m_smId);
    dma_channel_abort(m_dmaTxChannel);
    dma_channel_abort(m_dmaRxChannel);
    while (dma_channel_is_busy(m_dmaTxChannel) || dma_channel_is_busy(m_dmaRxChannel) || dma_hw->abort & (1u << m_dmaTxChannel | 1u << m_dmaRxChannel)) {
        // wait until busy is deasserted and abort is cleared (12.6.8.3 and E5)
        tight_loop_contents();
    }

    dma_channel_set_transfer_count(m_dmaRxChannel, len, false);
    dma_channel_set_write_addr(m_dmaRxChannel, dummy, false);
    dma_channel_set_transfer_count(m_dmaTxChannel, len, false);
    dma_channel_set_read_addr(m_dmaTxChannel, src, false);
    dma_start_channel_mask((1u << m_dmaTxChannel) | (1u << m_dmaRxChannel));

    while (dma_channel_is_busy(m_dmaRxChannel) || dma_channel_is_busy(m_dmaTxChannel)) {
        tight_loop_contents();
    }
    free(dummy);
    return true;
}

void NAND_PIO_Driver::read_page(uint32_t block, uint32_t page, bool getFeatureWait)
{
    if (m_cachedBlock == block && m_cachedPage == page) return;
    gpio_put(PIN_FLASH_CS, false);
    uint32_t addr = (page & 0x3F) | (block << 6);
    uint8_t buf[4] = {FLASH_CMD_PAGE_READ, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr};
    burst_spi_write(4, buf);
    gpio_put(PIN_FLASH_CS, true);
    m_cachedBlock = block;
    wait_feature(getFeatureWait);
}

u32 NAND_PIO_Driver::read_from_cache(u32 block, u32 start, u16 length, u8 *buf)
{
    if (start + (u32)length > 2176) return 0;
    start |= (block & 0b1) << 12;
    const u32 lenBackup = length;
    gpio_put(PIN_FLASH_CS, false);
    u8 req[4] = {FLASH_CMD_READ_FROM_CACHE_X1, (u8)(start >> 8), (u8)start, 0};
    burst_spi_write(4, req);
    burst_spi_read(length, buf);
    gpio_put(PIN_FLASH_CS, true);
    return lenBackup;
}

bool NAND_PIO_Driver::burst_spi_read(u16 len, u8 *dst)
{
    if (dst == nullptr) return false;
    u8 *dummy = (u8 *)malloc(len);
    if (dummy == nullptr) return false;
    pio_sm_clear_fifos(PIO_EXT_SPI_BB, m_smId);
    dma_channel_abort(m_dmaTxChannel);
    dma_channel_abort(m_dmaRxChannel);
    while (dma_channel_is_busy(m_dmaTxChannel) || dma_channel_is_busy(m_dmaRxChannel) || dma_hw->abort & (1u << m_dmaTxChannel | 1u << m_dmaRxChannel)) {
        // wait until busy is deasserted and abort is cleared (12.6.8.3 and E5)
        tight_loop_contents();
    }

    dma_channel_set_write_addr(m_dmaRxChannel, dst, false);
    dma_channel_set_transfer_count(m_dmaRxChannel, len, false);
    dma_channel_set_read_addr(m_dmaTxChannel, dummy, false);
    dma_channel_set_transfer_count(m_dmaTxChannel, len, false);
    dma_start_channel_mask((1u << m_dmaTxChannel) | (1u << m_dmaRxChannel));

    while (dma_channel_is_busy(m_dmaTxChannel) || dma_channel_is_busy(m_dmaRxChannel)) {
        tight_loop_contents();
    }
    free(dummy);
    return true;
}

void NAND_PIO_Driver::erase_block(u32 block, bool getFeatureWait)
{
    write_enable();
    gpio_put(PIN_FLASH_CS, false);
    u32 addr = block << 6;
    u8 buf[4] = {FLASH_CMD_BLOCK_ERASE, (u8)(addr >> 16), (u8)(addr >> 8), (u8)(addr)};
    burst_spi_write(4, buf);
    gpio_put(PIN_FLASH_CS, true);
    wait_feature(getFeatureWait);
}

void NAND_PIO_Driver::write_enable()
{
    gpio_put(PIN_FLASH_CS, false);
    spi_write(FLASH_CMD_WRITE_ENABLE);
    gpio_put(PIN_FLASH_CS, true);
}

void NAND_PIO_Driver::write_disable()
{
    gpio_put(PIN_FLASH_CS, false);
    spi_write(FLASH_CMD_WRITE_DISABLE);
    gpio_put(PIN_FLASH_CS, true);
}

u16 NAND_PIO_Driver::program_load(u32 block, u32 start, u16 length, const u8 *buf)
{
    if (start + (u32)length > 2176) return 0;
    write_enable();
    start |= (block & 0b1) << 12;
    const u16 lenBackup = length;
    gpio_put(PIN_FLASH_CS, false);
    u8 req[3] = {FLASH_CMD_PROGRAM_LOAD_X1, (u8)(start >> 8), (u8)start};
    burst_spi_write(3, req);
    burst_spi_write(length, buf);
    gpio_put(PIN_FLASH_CS, true);
    m_cachedBlock = 0xFFFFFFFF;
    m_cachedPage = 0xFFFFFFFF;
    return lenBackup;
}

void NAND_PIO_Driver::program_execute(u32 block, u32 page, bool getFeatureWait)
{
    write_enable();
    gpio_put(PIN_FLASH_CS, false);
    u32 addr = (page & 0x3F) | ((u32)block << 6);
    u8 buf[4] = {FLASH_CMD_PROGRAM_EXECUTE, (u8)(addr >> 16), (u8)(addr >> 8), (u8)addr};
    burst_spi_write(4, buf);
    gpio_put(PIN_FLASH_CS, true);
    invalidate_caches();
    wait_feature(getFeatureWait);
}

void NAND_PIO_Driver::wait_feature(bool getFeatureWait)
{
    if (getFeatureWait) {
        while (check_feature(0b1, 0b1)) {
            tight_loop_contents();
        }
    }
}

void NAND_PIO_Driver::invalidate_caches()
{
    m_cachedBlock = 0xFFFFFFFF;
    m_cachedPage = 0xFFFFFFFF;
    for (auto &sc : m_secCaches)
    {
        sc.block = 0xFFFFFFF;
        sc.page = 0xFFFFFFFF;
        sc.sector = 0xFFFFFFFF;
        sc.prio = 0xFF;
    }
}

bool NAND_PIO_Driver::read_page(uint32_t block, uint32_t page, uint8_t *data)
{
    if (data == nullptr) return false;
    if (block >= 2048 || page >= 64) return false;
    read_page(block, page);
    for (size_t i = 0; i < 2; i++) {
        read_from_cache(block, 1024 * i, 1024, (uint8_t *)(data + 1024 * i));
    }
    return true;
}

bool NAND_PIO_Driver::write_page(uint32_t block, uint32_t page, const uint8_t *data)
{
    if (data == nullptr) return false;
    if (block >= 2048 || page >= 64) return false;

    for (size_t i = 0; i < 2; i++) {
        program_load(block, 1024 * i, 1024, (const uint8_t *)(data + 1024 * i));
    }
    program_execute(block, page);
    return true;
}

bool NAND_PIO_Driver::erase_block(uint32_t block)
{
    erase_block(block);
    return true;
}
