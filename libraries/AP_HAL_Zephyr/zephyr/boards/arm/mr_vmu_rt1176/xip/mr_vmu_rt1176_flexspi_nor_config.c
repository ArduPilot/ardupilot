/*
 * Copyright (c) 2019, MADMACHINE LIMITED
 * Copyright 2024 NXP
 * Copyright 2026 ArduPilot authors
 *
 * The FlexSPI NOR configuration block field values in this file are derived
 * from Zephyr's boards/nxp/vmu_rt1170/flexspi_nor_config.c (see the detailed
 * provenance note below), and this file therefore keeps that file's licence.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* mr_vmu_rt1176 FlexSPI NOR Configuration Block (FCB): the structure the BootROM
 * reads at offset 0 to bring the external flash up before any code runs. */

#if defined(CONFIG_NXP_IMXRT_BOOT_HEADER) && defined(CONFIG_BOOT_FLEXSPI_NOR)

#include <flexspi_nor_config.h>

__attribute__((section(".boot_hdr.conf"), used))
const struct flexspi_nor_config_t mr_vmu_rt1176_flexspi_nor_config = {
	.mem_config = {
		.tag = FLEXSPI_CFG_BLK_TAG,
		.version = FLEXSPI_CFG_BLK_VERSION,
		.read_sample_clk_src =
			FLEXSPI_READ_SAMPLE_CLK_LOOPBACK_INTERNALLY,
		.cs_hold_time = 1u,
		.cs_setup_time = 1u,
		.sflash_pad_type = SERIAL_FLASH_1_PAD,
		.serial_clk_freq = FLEXSPI_SERIAL_CLK_80MHZ,
		.sflash_a1_size = 64u * 1024u * 1024u,
		/* SOURCE OF THE WRITE-SIDE SEQUENCES (1/3/5/8/9/11): copied from NXP's own
		 * reference FCB for this part, not derived, because a wrong write sequence
		 * bricks the boot flash. */
		.lookup_table = {
			/* 0: Read */
			[4 * NOR_CMD_LUT_SEQ_IDX_READ + 0] = FLEXSPI_LUT_SEQ(
					CMD_SDR, FLEXSPI_1PAD,
					0x03, RADDR_SDR,
					FLEXSPI_1PAD, 0x18),
			[4 * NOR_CMD_LUT_SEQ_IDX_READ + 1] = FLEXSPI_LUT_SEQ(
					READ_SDR, FLEXSPI_1PAD,
					0x04, STOP,
					FLEXSPI_1PAD, 0),

			/* 1: Read Status Register (WIP is bit 0, busy when 1 -
			 * matches the default busy_offset/busy_bit_polarity of 0) */
			[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS + 0] = FLEXSPI_LUT_SEQ(
					CMD_SDR, FLEXSPI_1PAD,
					0x05, READ_SDR,
					FLEXSPI_1PAD, 0x04),

			/* 3: Write Enable */
			[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE + 0] = FLEXSPI_LUT_SEQ(
					CMD_SDR, FLEXSPI_1PAD,
					0x06, STOP,
					FLEXSPI_1PAD, 0),

			/* 5: Erase Sector (4KB) */
			[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 0] = FLEXSPI_LUT_SEQ(
					CMD_SDR, FLEXSPI_1PAD,
					0x20, RADDR_SDR,
					FLEXSPI_1PAD, 0x18),

			/* 8: Erase Block (64KB) */
			[4 * NOR_CMD_LUT_SEQ_IDX_ERASEBLOCK + 0] = FLEXSPI_LUT_SEQ(
					CMD_SDR, FLEXSPI_1PAD,
					0xD8, RADDR_SDR,
					FLEXSPI_1PAD, 0x18),

			/* 9: Page Program (256B) */
			[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 0] = FLEXSPI_LUT_SEQ(
					CMD_SDR, FLEXSPI_1PAD,
					0x02, RADDR_SDR,
					FLEXSPI_1PAD, 0x18),
			[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 1] = FLEXSPI_LUT_SEQ(
					WRITE_SDR, FLEXSPI_1PAD,
					0x04, STOP,
					FLEXSPI_1PAD, 0),

			/* 11: Chip Erase */
			[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 0] = FLEXSPI_LUT_SEQ(
					CMD_SDR, FLEXSPI_1PAD,
					0x60, STOP,
					FLEXSPI_1PAD, 0),
		},
	},
	.page_size = 256u,
	.sector_size = 4u * 1024u,
	.block_size = 64u * 1024u,
	.is_uniform_block_size = false,
};

#endif /* defined(CONFIG_NXP_IMXRT_BOOT_HEADER) && defined(CONFIG_BOOT_FLEXSPI_NOR) */
