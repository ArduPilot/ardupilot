/*
 * Block device emulated in RAM
 *
 * Copyright (c) 2022, The littlefs authors.
 * Copyright (c) 2017, Arm Limited. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "bd/lfs_rambd.h"

int lfs_rambd_create(const struct lfs_config *cfg,
        const struct lfs_rambd_config *bdcfg) {
    LFS_RAMBD_TRACE("lfs_rambd_create(%p {.context=%p, "
                ".read=%p, .prog=%p, .erase=%p, .sync=%p}, "
                "%p {.read_size=%"PRIu32", .prog_size=%"PRIu32", "
                ".erase_size=%"PRIu32", .erase_count=%"PRIu32", "
                ".buffer=%p})",
            (void*)cfg, cfg->context,
            (void*)(uintptr_t)cfg->read, (void*)(uintptr_t)cfg->prog,
            (void*)(uintptr_t)cfg->erase, (void*)(uintptr_t)cfg->sync,
            (void*)bdcfg,
            bdcfg->read_size, bdcfg->prog_size, bdcfg->erase_size,
            bdcfg->erase_count, bdcfg->buffer);
    lfs_rambd_t *bd = cfg->context;
    bd->cfg = bdcfg;

    // allocate buffer?
    if (bd->cfg->buffer) {
        bd->buffer = bd->cfg->buffer;
    } else {
        bd->buffer = lfs_malloc(bd->cfg->erase_size * bd->cfg->erase_count);
        if (!bd->buffer) {
            LFS_RAMBD_TRACE("lfs_rambd_create -> %d", LFS_ERR_NOMEM);
            return LFS_ERR_NOMEM;
        }
    }

    // zero for reproducibility
    memset(bd->buffer, 0, bd->cfg->erase_size * bd->cfg->erase_count);

    LFS_RAMBD_TRACE("lfs_rambd_create -> %d", 0);
    return 0;
}

int lfs_rambd_destroy(const struct lfs_config *cfg) {
    LFS_RAMBD_TRACE("lfs_rambd_destroy(%p)", (void*)cfg);
    // clean up memory
    lfs_rambd_t *bd = cfg->context;
    if (!bd->cfg->buffer) {
        lfs_free(bd->buffer);
    }
    LFS_RAMBD_TRACE("lfs_rambd_destroy -> %d", 0);
    return 0;
}

int lfs_rambd_read(const struct lfs_config *cfg, lfs_block_t block,
        lfs_off_t off, void *buffer, lfs_size_t size) {
    LFS_RAMBD_TRACE("lfs_rambd_read(%p, "
                "0x%"PRIx32", %"PRIu32", %p, %"PRIu32")",
            (void*)cfg, block, off, buffer, size);
    lfs_rambd_t *bd = cfg->context;

    // check if read is valid
    LFS_ASSERT(block < bd->cfg->erase_count);
    LFS_ASSERT(off  % bd->cfg->read_size == 0);
    LFS_ASSERT(size % bd->cfg->read_size == 0);
    LFS_ASSERT(off+size <= bd->cfg->erase_size);

    // read data
    memcpy(buffer, &bd->buffer[block*bd->cfg->erase_size + off], size);

    LFS_RAMBD_TRACE("lfs_rambd_read -> %d", 0);
    return 0;
}

int lfs_rambd_prog(const struct lfs_config *cfg, lfs_block_t block,
        lfs_off_t off, const void *buffer, lfs_size_t size) {
    LFS_RAMBD_TRACE("lfs_rambd_prog(%p, "
                "0x%"PRIx32", %"PRIu32", %p, %"PRIu32")",
            (void*)cfg, block, off, buffer, size);
    lfs_rambd_t *bd = cfg->context;

    // check if write is valid
    LFS_ASSERT(block < bd->cfg->erase_count);
    LFS_ASSERT(off  % bd->cfg->prog_size == 0);
    LFS_ASSERT(size % bd->cfg->prog_size == 0);
    LFS_ASSERT(off+size <= bd->cfg->erase_size);

    // program data
    memcpy(&bd->buffer[block*bd->cfg->erase_size + off], buffer, size);

    LFS_RAMBD_TRACE("lfs_rambd_prog -> %d", 0);
    return 0;
}

int lfs_rambd_erase(const struct lfs_config *cfg, lfs_block_t block) {
    LFS_RAMBD_TRACE("lfs_rambd_erase(%p, 0x%"PRIx32" (%"PRIu32"))",
            (void*)cfg, block, ((lfs_rambd_t*)cfg->context)->cfg->erase_size);
    lfs_rambd_t *bd = cfg->context;

    // check if erase is valid
    LFS_ASSERT(block < bd->cfg->erase_count);

    // erase is a noop
    (void)block;

    LFS_RAMBD_TRACE("lfs_rambd_erase -> %d", 0);
    return 0;
}

int lfs_rambd_sync(const struct lfs_config *cfg) {
    LFS_RAMBD_TRACE("lfs_rambd_sync(%p)", (void*)cfg);

    // sync is a noop
    (void)cfg;

    LFS_RAMBD_TRACE("lfs_rambd_sync -> %d", 0);
    return 0;
}
