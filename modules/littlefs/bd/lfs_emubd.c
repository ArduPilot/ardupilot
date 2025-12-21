/*
 * Emulating block device, wraps filebd and rambd while providing a bunch
 * of hooks for testing littlefs in various conditions.
 *
 * Copyright (c) 2022, The littlefs authors.
 * Copyright (c) 2017, Arm Limited. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 199309L
#endif

#include "bd/lfs_emubd.h"

#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>

#ifdef _WIN32
#include <windows.h>
#endif


// access to lazily-allocated/copy-on-write blocks
//
// Note we can only modify a block if we have exclusive access to it (rc == 1)
//

static lfs_emubd_block_t *lfs_emubd_incblock(lfs_emubd_block_t *block) {
    if (block) {
        block->rc += 1;
    }
    return block;
}

static void lfs_emubd_decblock(lfs_emubd_block_t *block) {
    if (block) {
        block->rc -= 1;
        if (block->rc == 0) {
            free(block);
        }
    }
}

static lfs_emubd_block_t *lfs_emubd_mutblock(
        const struct lfs_config *cfg,
        lfs_emubd_block_t **block) {
    lfs_emubd_t *bd = cfg->context;
    lfs_emubd_block_t *block_ = *block;
    if (block_ && block_->rc == 1) {
        // rc == 1? can modify
        return block_;

    } else if (block_) {
        // rc > 1? need to create a copy
        lfs_emubd_block_t *nblock = malloc(
                sizeof(lfs_emubd_block_t) + bd->cfg->erase_size);
        if (!nblock) {
            return NULL;
        }

        memcpy(nblock, block_,
                sizeof(lfs_emubd_block_t) + bd->cfg->erase_size);
        nblock->rc = 1;

        lfs_emubd_decblock(block_);
        *block = nblock;
        return nblock;

    } else {
        // no block? need to allocate
        lfs_emubd_block_t *nblock = malloc(
                sizeof(lfs_emubd_block_t) + bd->cfg->erase_size);
        if (!nblock) {
            return NULL;
        }

        nblock->rc = 1;
        nblock->wear = 0;

        // zero for consistency
        memset(nblock->data,
                (bd->cfg->erase_value != -1) ? bd->cfg->erase_value : 0,
                bd->cfg->erase_size);

        *block = nblock;
        return nblock;
    }
}


// emubd create/destroy

int lfs_emubd_create(const struct lfs_config *cfg,
        const struct lfs_emubd_config *bdcfg) {
    LFS_EMUBD_TRACE("lfs_emubd_create(%p {.context=%p, "
                ".read=%p, .prog=%p, .erase=%p, .sync=%p}, "
                "%p {.read_size=%"PRIu32", .prog_size=%"PRIu32", "
                ".erase_size=%"PRIu32", .erase_count=%"PRIu32", "
                ".erase_value=%"PRId32", .erase_cycles=%"PRIu32", "
                ".badblock_behavior=%"PRIu8", .power_cycles=%"PRIu32", "
                ".powerloss_behavior=%"PRIu8", .powerloss_cb=%p, "
                ".powerloss_data=%p, .track_branches=%d})",
            (void*)cfg, cfg->context,
            (void*)(uintptr_t)cfg->read, (void*)(uintptr_t)cfg->prog,
            (void*)(uintptr_t)cfg->erase, (void*)(uintptr_t)cfg->sync,
            (void*)bdcfg,
            bdcfg->read_size, bdcfg->prog_size, bdcfg->erase_size,
            bdcfg->erase_count, bdcfg->erase_value, bdcfg->erase_cycles,
            bdcfg->badblock_behavior, bdcfg->power_cycles,
            bdcfg->powerloss_behavior, (void*)(uintptr_t)bdcfg->powerloss_cb,
            bdcfg->powerloss_data, bdcfg->track_branches);
    lfs_emubd_t *bd = cfg->context;
    bd->cfg = bdcfg;

    // allocate our block array, all blocks start as uninitialized
    bd->blocks = malloc(bd->cfg->erase_count * sizeof(lfs_emubd_block_t*));
    if (!bd->blocks) {
        LFS_EMUBD_TRACE("lfs_emubd_create -> %d", LFS_ERR_NOMEM);
        return LFS_ERR_NOMEM;
    }
    memset(bd->blocks, 0, bd->cfg->erase_count * sizeof(lfs_emubd_block_t*));

    // setup testing things
    bd->readed = 0;
    bd->proged = 0;
    bd->erased = 0;
    bd->power_cycles = bd->cfg->power_cycles;
    bd->ooo_block = -1;
    bd->ooo_data = NULL;
    bd->disk = NULL;

    if (bd->cfg->disk_path) {
        bd->disk = malloc(sizeof(lfs_emubd_disk_t));
        if (!bd->disk) {
            LFS_EMUBD_TRACE("lfs_emubd_create -> %d", LFS_ERR_NOMEM);
            return LFS_ERR_NOMEM;
        }
        bd->disk->rc = 1;
        bd->disk->scratch = NULL;

        #ifdef _WIN32
        bd->disk->fd = open(bd->cfg->disk_path,
                O_RDWR | O_CREAT | O_BINARY, 0666);
        #else
        bd->disk->fd = open(bd->cfg->disk_path,
                O_RDWR | O_CREAT, 0666);
        #endif
        if (bd->disk->fd < 0) {
            int err = -errno;
            LFS_EMUBD_TRACE("lfs_emubd_create -> %d", err);
            return err;
        }

        // if we're emulating erase values, we can keep a block around in
        // memory of just the erase state to speed up emulated erases
        if (bd->cfg->erase_value != -1) {
            bd->disk->scratch = malloc(bd->cfg->erase_size);
            if (!bd->disk->scratch) {
                LFS_EMUBD_TRACE("lfs_emubd_create -> %d", LFS_ERR_NOMEM);
                return LFS_ERR_NOMEM;
            }
            memset(bd->disk->scratch,
                    bd->cfg->erase_value,
                    bd->cfg->erase_size);

            // go ahead and erase all of the disk, otherwise the file will not
            // match our internal representation
            for (size_t i = 0; i < bd->cfg->erase_count; i++) {
                ssize_t res = write(bd->disk->fd,
                        bd->disk->scratch,
                        bd->cfg->erase_size);
                if (res < 0) {
                    int err = -errno;
                    LFS_EMUBD_TRACE("lfs_emubd_create -> %d", err);
                    return err;
                }
            }
        }
    }

    LFS_EMUBD_TRACE("lfs_emubd_create -> %d", 0);
    return 0;
}

int lfs_emubd_destroy(const struct lfs_config *cfg) {
    LFS_EMUBD_TRACE("lfs_emubd_destroy(%p)", (void*)cfg);
    lfs_emubd_t *bd = cfg->context;

    // decrement reference counts
    for (lfs_block_t i = 0; i < bd->cfg->erase_count; i++) {
        lfs_emubd_decblock(bd->blocks[i]);
    }
    free(bd->blocks);

    // clean up other resources 
    lfs_emubd_decblock(bd->ooo_data);
    if (bd->disk) {
        bd->disk->rc -= 1;
        if (bd->disk->rc == 0) {
            close(bd->disk->fd);
            free(bd->disk->scratch);
            free(bd->disk);
        }
    }

    LFS_EMUBD_TRACE("lfs_emubd_destroy -> %d", 0);
    return 0;
}


// powerloss hook
static int lfs_emubd_powerloss(const struct lfs_config *cfg) {
    lfs_emubd_t *bd = cfg->context;

    // emulate out-of-order writes?
    lfs_emubd_block_t *ooo_data = NULL;
    if (bd->cfg->powerloss_behavior == LFS_EMUBD_POWERLOSS_OOO
            && bd->ooo_block != -1) {
        // since writes between syncs are allowed to be out-of-order, it
        // shouldn't hurt to restore the first write on powerloss, right?
        ooo_data = bd->blocks[bd->ooo_block];
        bd->blocks[bd->ooo_block] = lfs_emubd_incblock(bd->ooo_data);

        // mirror to disk file?
        if (bd->disk
                && (bd->blocks[bd->ooo_block]
                    || bd->cfg->erase_value != -1)) {
            off_t res1 = lseek(bd->disk->fd,
                    (off_t)bd->ooo_block*bd->cfg->erase_size,
                    SEEK_SET);
            if (res1 < 0) {
                return -errno;
            }

            ssize_t res2 = write(bd->disk->fd,
                    (bd->blocks[bd->ooo_block])
                        ? bd->blocks[bd->ooo_block]->data
                        : bd->disk->scratch,
                    bd->cfg->erase_size);
            if (res2 < 0) {
                return -errno;
            }
        }
    }

    // simulate power loss
    bd->cfg->powerloss_cb(bd->cfg->powerloss_data);

    // if we continue, undo out-of-order write emulation
    if (bd->cfg->powerloss_behavior == LFS_EMUBD_POWERLOSS_OOO
            && bd->ooo_block != -1) {
        lfs_emubd_decblock(bd->blocks[bd->ooo_block]);
        bd->blocks[bd->ooo_block] = ooo_data;

        // mirror to disk file?
        if (bd->disk
                && (bd->blocks[bd->ooo_block]
                    || bd->cfg->erase_value != -1)) {
            off_t res1 = lseek(bd->disk->fd,
                    (off_t)bd->ooo_block*bd->cfg->erase_size,
                    SEEK_SET);
            if (res1 < 0) {
                return -errno;
            }

            ssize_t res2 = write(bd->disk->fd,
                    (bd->blocks[bd->ooo_block])
                        ? bd->blocks[bd->ooo_block]->data
                        : bd->disk->scratch,
                    bd->cfg->erase_size);
            if (res2 < 0) {
                return -errno;
            }
        }
    }

    return 0;
}


// block device API

int lfs_emubd_read(const struct lfs_config *cfg, lfs_block_t block,
        lfs_off_t off, void *buffer, lfs_size_t size) {
    LFS_EMUBD_TRACE("lfs_emubd_read(%p, "
                "0x%"PRIx32", %"PRIu32", %p, %"PRIu32")",
            (void*)cfg, block, off, buffer, size);
    lfs_emubd_t *bd = cfg->context;

    // check if read is valid
    LFS_ASSERT(block < bd->cfg->erase_count);
    LFS_ASSERT(off  % bd->cfg->read_size == 0);
    LFS_ASSERT(size % bd->cfg->read_size == 0);
    LFS_ASSERT(off+size <= bd->cfg->erase_size);

    // get the block
    const lfs_emubd_block_t *b = bd->blocks[block];
    if (b) {
        // block bad?
        if (bd->cfg->erase_cycles && b->wear >= bd->cfg->erase_cycles &&
                bd->cfg->badblock_behavior == LFS_EMUBD_BADBLOCK_READERROR) {
            LFS_EMUBD_TRACE("lfs_emubd_read -> %d", LFS_ERR_CORRUPT);
            return LFS_ERR_CORRUPT;
        }

        // read data
        memcpy(buffer, &b->data[off], size);
    } else {
        // zero for consistency
        memset(buffer,
                (bd->cfg->erase_value != -1) ? bd->cfg->erase_value : 0,
                size);
    }   

    // track reads
    bd->readed += size;
    if (bd->cfg->read_sleep) {
        int err = nanosleep(&(struct timespec){
                .tv_sec=bd->cfg->read_sleep/1000000000,
                .tv_nsec=bd->cfg->read_sleep%1000000000},
            NULL);
        if (err) {
            err = -errno;
            LFS_EMUBD_TRACE("lfs_emubd_read -> %d", err);
            return err;
        }
    }

    LFS_EMUBD_TRACE("lfs_emubd_read -> %d", 0);
    return 0;
}

int lfs_emubd_prog(const struct lfs_config *cfg, lfs_block_t block,
        lfs_off_t off, const void *buffer, lfs_size_t size) {
    LFS_EMUBD_TRACE("lfs_emubd_prog(%p, "
                "0x%"PRIx32", %"PRIu32", %p, %"PRIu32")",
            (void*)cfg, block, off, buffer, size);
    lfs_emubd_t *bd = cfg->context;

    // check if write is valid
    LFS_ASSERT(block < bd->cfg->erase_count);
    LFS_ASSERT(off  % bd->cfg->prog_size == 0);
    LFS_ASSERT(size % bd->cfg->prog_size == 0);
    LFS_ASSERT(off+size <= bd->cfg->erase_size);

    // get the block
    lfs_emubd_block_t *b = lfs_emubd_mutblock(cfg, &bd->blocks[block]);
    if (!b) {
        LFS_EMUBD_TRACE("lfs_emubd_prog -> %d", LFS_ERR_NOMEM);
        return LFS_ERR_NOMEM;
    }

    // block bad?
    if (bd->cfg->erase_cycles && b->wear >= bd->cfg->erase_cycles) {
        if (bd->cfg->badblock_behavior ==
                LFS_EMUBD_BADBLOCK_PROGERROR) {
            LFS_EMUBD_TRACE("lfs_emubd_prog -> %d", LFS_ERR_CORRUPT);
            return LFS_ERR_CORRUPT;
        } else if (bd->cfg->badblock_behavior ==
                LFS_EMUBD_BADBLOCK_PROGNOOP ||
                bd->cfg->badblock_behavior ==
                LFS_EMUBD_BADBLOCK_ERASENOOP) {
            LFS_EMUBD_TRACE("lfs_emubd_prog -> %d", 0);
            return 0;
        }
    }

    // were we erased properly?
    if (bd->cfg->erase_value != -1) {
        for (lfs_off_t i = 0; i < size; i++) {
            LFS_ASSERT(b->data[off+i] == bd->cfg->erase_value);
        }
    }

    // prog data
    memcpy(&b->data[off], buffer, size);

    // mirror to disk file?
    if (bd->disk) {
        off_t res1 = lseek(bd->disk->fd,
                (off_t)block*bd->cfg->erase_size + (off_t)off,
                SEEK_SET);
        if (res1 < 0) {
            int err = -errno;
            LFS_EMUBD_TRACE("lfs_emubd_prog -> %d", err);
            return err;
        }

        ssize_t res2 = write(bd->disk->fd, buffer, size);
        if (res2 < 0) {
            int err = -errno;
            LFS_EMUBD_TRACE("lfs_emubd_prog -> %d", err);
            return err;
        }
    }

    // track progs
    bd->proged += size;
    if (bd->cfg->prog_sleep) {
        int err = nanosleep(&(struct timespec){
                .tv_sec=bd->cfg->prog_sleep/1000000000,
                .tv_nsec=bd->cfg->prog_sleep%1000000000},
            NULL);
        if (err) {
            err = -errno;
            LFS_EMUBD_TRACE("lfs_emubd_prog -> %d", err);
            return err;
        }
    }

    // lose power?
    if (bd->power_cycles > 0) {
        bd->power_cycles -= 1;
        if (bd->power_cycles == 0) {
            int err = lfs_emubd_powerloss(cfg);
            if (err) {
                LFS_EMUBD_TRACE("lfs_emubd_prog -> %d", err);
                return err;
            }
        }
    }

    LFS_EMUBD_TRACE("lfs_emubd_prog -> %d", 0);
    return 0;
}

int lfs_emubd_erase(const struct lfs_config *cfg, lfs_block_t block) {
    LFS_EMUBD_TRACE("lfs_emubd_erase(%p, 0x%"PRIx32" (%"PRIu32"))",
            (void*)cfg, block, ((lfs_emubd_t*)cfg->context)->cfg->erase_size);
    lfs_emubd_t *bd = cfg->context;

    // check if erase is valid
    LFS_ASSERT(block < bd->cfg->erase_count);

    // emulate out-of-order writes? save first write
    if (bd->cfg->powerloss_behavior == LFS_EMUBD_POWERLOSS_OOO
            && bd->ooo_block == -1) {
        bd->ooo_block = block;
        bd->ooo_data = lfs_emubd_incblock(bd->blocks[block]);
    }

    // get the block
    lfs_emubd_block_t *b = lfs_emubd_mutblock(cfg, &bd->blocks[block]);
    if (!b) {
        LFS_EMUBD_TRACE("lfs_emubd_erase -> %d", LFS_ERR_NOMEM);
        return LFS_ERR_NOMEM;
    }

    // block bad?
    if (bd->cfg->erase_cycles) {
        if (b->wear >= bd->cfg->erase_cycles) {
            if (bd->cfg->badblock_behavior ==
                    LFS_EMUBD_BADBLOCK_ERASEERROR) {
                LFS_EMUBD_TRACE("lfs_emubd_erase -> %d", LFS_ERR_CORRUPT);
                return LFS_ERR_CORRUPT;
            } else if (bd->cfg->badblock_behavior ==
                    LFS_EMUBD_BADBLOCK_ERASENOOP) {
                LFS_EMUBD_TRACE("lfs_emubd_erase -> %d", 0);
                return 0;
            }
        } else {
            // mark wear
            b->wear += 1;
        }
    }

    // emulate an erase value?
    if (bd->cfg->erase_value != -1) {
        memset(b->data, bd->cfg->erase_value, bd->cfg->erase_size);

        // mirror to disk file?
        if (bd->disk) {
            off_t res1 = lseek(bd->disk->fd,
                    (off_t)block*bd->cfg->erase_size,
                    SEEK_SET);
            if (res1 < 0) {
                int err = -errno;
                LFS_EMUBD_TRACE("lfs_emubd_erase -> %d", err);
                return err;
            }

            ssize_t res2 = write(bd->disk->fd,
                    bd->disk->scratch,
                    bd->cfg->erase_size);
            if (res2 < 0) {
                int err = -errno;
                LFS_EMUBD_TRACE("lfs_emubd_erase -> %d", err);
                return err;
            }
        }
    }

    // track erases
    bd->erased += bd->cfg->erase_size;
    if (bd->cfg->erase_sleep) {
        int err = nanosleep(&(struct timespec){
                .tv_sec=bd->cfg->erase_sleep/1000000000,
                .tv_nsec=bd->cfg->erase_sleep%1000000000},
            NULL);
        if (err) {
            err = -errno;
            LFS_EMUBD_TRACE("lfs_emubd_erase -> %d", err);
            return err;
        }
    }

    // lose power?
    if (bd->power_cycles > 0) {
        bd->power_cycles -= 1;
        if (bd->power_cycles == 0) {
            int err = lfs_emubd_powerloss(cfg);
            if (err) {
                LFS_EMUBD_TRACE("lfs_emubd_erase -> %d", err);
                return err;
            }
        }
    }

    LFS_EMUBD_TRACE("lfs_emubd_erase -> %d", 0);
    return 0;
}

int lfs_emubd_sync(const struct lfs_config *cfg) {
    LFS_EMUBD_TRACE("lfs_emubd_sync(%p)", (void*)cfg);
    lfs_emubd_t *bd = cfg->context;

    // emulate out-of-order writes? reset first write, writes
    // cannot be out-of-order across sync
    if (bd->cfg->powerloss_behavior == LFS_EMUBD_POWERLOSS_OOO) {
        lfs_emubd_decblock(bd->ooo_data);
        bd->ooo_block = -1;
        bd->ooo_data = NULL;
    }

    LFS_EMUBD_TRACE("lfs_emubd_sync -> %d", 0);
    return 0;
}


/// Additional extended API for driving test features ///

static int lfs_emubd_crc_(const struct lfs_config *cfg,
        lfs_block_t block, uint32_t *crc) {
    lfs_emubd_t *bd = cfg->context;

    // check if crc is valid
    LFS_ASSERT(block < cfg->block_count);

    // crc the block
    uint32_t crc_ = 0xffffffff;
    const lfs_emubd_block_t *b = bd->blocks[block];
    if (b) {
        crc_ = lfs_crc(crc_, b->data, cfg->block_size);
    } else {
        uint8_t erase_value = (bd->cfg->erase_value != -1)
                ? bd->cfg->erase_value
                : 0;
        for (lfs_size_t i = 0; i < cfg->block_size; i++) {
            crc_ = lfs_crc(crc_, &erase_value, 1);
        }
    }
    *crc = 0xffffffff ^ crc_;

    return 0;
}

int lfs_emubd_crc(const struct lfs_config *cfg,
        lfs_block_t block, uint32_t *crc) {
    LFS_EMUBD_TRACE("lfs_emubd_crc(%p, %"PRIu32", %p)",
            (void*)cfg, block, crc);
    int err = lfs_emubd_crc_(cfg, block, crc);
    LFS_EMUBD_TRACE("lfs_emubd_crc -> %d", err);
    return err;
}

int lfs_emubd_bdcrc(const struct lfs_config *cfg, uint32_t *crc) {
    LFS_EMUBD_TRACE("lfs_emubd_bdcrc(%p, %p)", (void*)cfg, crc);

    uint32_t crc_ = 0xffffffff;
    for (lfs_block_t i = 0; i < cfg->block_count; i++) {
        uint32_t i_crc;
        int err = lfs_emubd_crc_(cfg, i, &i_crc);
        if (err) {
            LFS_EMUBD_TRACE("lfs_emubd_bdcrc -> %d", err);
            return err;
        }

        crc_ = lfs_crc(crc_, &i_crc, sizeof(uint32_t));
    }
    *crc = 0xffffffff ^ crc_;

    LFS_EMUBD_TRACE("lfs_emubd_bdcrc -> %d", 0);
    return 0;
}

lfs_emubd_sio_t lfs_emubd_readed(const struct lfs_config *cfg) {
    LFS_EMUBD_TRACE("lfs_emubd_readed(%p)", (void*)cfg);
    lfs_emubd_t *bd = cfg->context;
    LFS_EMUBD_TRACE("lfs_emubd_readed -> %"PRIu64, bd->readed);
    return bd->readed;
}

lfs_emubd_sio_t lfs_emubd_proged(const struct lfs_config *cfg) {
    LFS_EMUBD_TRACE("lfs_emubd_proged(%p)", (void*)cfg);
    lfs_emubd_t *bd = cfg->context;
    LFS_EMUBD_TRACE("lfs_emubd_proged -> %"PRIu64, bd->proged);
    return bd->proged;
}

lfs_emubd_sio_t lfs_emubd_erased(const struct lfs_config *cfg) {
    LFS_EMUBD_TRACE("lfs_emubd_erased(%p)", (void*)cfg);
    lfs_emubd_t *bd = cfg->context;
    LFS_EMUBD_TRACE("lfs_emubd_erased -> %"PRIu64, bd->erased);
    return bd->erased;
}

int lfs_emubd_setreaded(const struct lfs_config *cfg, lfs_emubd_io_t readed) {
    LFS_EMUBD_TRACE("lfs_emubd_setreaded(%p, %"PRIu64")", (void*)cfg, readed);
    lfs_emubd_t *bd = cfg->context;
    bd->readed = readed;
    LFS_EMUBD_TRACE("lfs_emubd_setreaded -> %d", 0);
    return 0;
}

int lfs_emubd_setproged(const struct lfs_config *cfg, lfs_emubd_io_t proged) {
    LFS_EMUBD_TRACE("lfs_emubd_setproged(%p, %"PRIu64")", (void*)cfg, proged);
    lfs_emubd_t *bd = cfg->context;
    bd->proged = proged;
    LFS_EMUBD_TRACE("lfs_emubd_setproged -> %d", 0);
    return 0;
}

int lfs_emubd_seterased(const struct lfs_config *cfg, lfs_emubd_io_t erased) {
    LFS_EMUBD_TRACE("lfs_emubd_seterased(%p, %"PRIu64")", (void*)cfg, erased);
    lfs_emubd_t *bd = cfg->context;
    bd->erased = erased;
    LFS_EMUBD_TRACE("lfs_emubd_seterased -> %d", 0);
    return 0;
}

lfs_emubd_swear_t lfs_emubd_wear(const struct lfs_config *cfg,
        lfs_block_t block) {
    LFS_EMUBD_TRACE("lfs_emubd_wear(%p, %"PRIu32")", (void*)cfg, block);
    lfs_emubd_t *bd = cfg->context;

    // check if block is valid
    LFS_ASSERT(block < bd->cfg->erase_count);

    // get the wear
    lfs_emubd_wear_t wear;
    const lfs_emubd_block_t *b = bd->blocks[block];
    if (b) {
        wear = b->wear;
    } else {
        wear = 0;
    }

    LFS_EMUBD_TRACE("lfs_emubd_wear -> %"PRIi32, wear);
    return wear;
}

int lfs_emubd_setwear(const struct lfs_config *cfg,
        lfs_block_t block, lfs_emubd_wear_t wear) {
    LFS_EMUBD_TRACE("lfs_emubd_setwear(%p, %"PRIu32", %"PRIi32")",
            (void*)cfg, block, wear);
    lfs_emubd_t *bd = cfg->context;

    // check if block is valid
    LFS_ASSERT(block < bd->cfg->erase_count);

    // set the wear
    lfs_emubd_block_t *b = lfs_emubd_mutblock(cfg, &bd->blocks[block]);
    if (!b) {
        LFS_EMUBD_TRACE("lfs_emubd_setwear -> %d", LFS_ERR_NOMEM);
        return LFS_ERR_NOMEM;
    }
    b->wear = wear;

    LFS_EMUBD_TRACE("lfs_emubd_setwear -> %d", 0);
    return 0;
}

lfs_emubd_spowercycles_t lfs_emubd_powercycles(
        const struct lfs_config *cfg) {
    LFS_EMUBD_TRACE("lfs_emubd_powercycles(%p)", (void*)cfg);
    lfs_emubd_t *bd = cfg->context;

    LFS_EMUBD_TRACE("lfs_emubd_powercycles -> %"PRIi32, bd->power_cycles);
    return bd->power_cycles;
}

int lfs_emubd_setpowercycles(const struct lfs_config *cfg,
        lfs_emubd_powercycles_t power_cycles) {
    LFS_EMUBD_TRACE("lfs_emubd_setpowercycles(%p, %"PRIi32")",
            (void*)cfg, power_cycles);
    lfs_emubd_t *bd = cfg->context;

    bd->power_cycles = power_cycles;

    LFS_EMUBD_TRACE("lfs_emubd_powercycles -> %d", 0);
    return 0;
}

int lfs_emubd_copy(const struct lfs_config *cfg, lfs_emubd_t *copy) {
    LFS_EMUBD_TRACE("lfs_emubd_copy(%p, %p)", (void*)cfg, (void*)copy);
    lfs_emubd_t *bd = cfg->context;

    // lazily copy over our block array
    copy->blocks = malloc(bd->cfg->erase_count * sizeof(lfs_emubd_block_t*));
    if (!copy->blocks) {
        LFS_EMUBD_TRACE("lfs_emubd_copy -> %d", LFS_ERR_NOMEM);
        return LFS_ERR_NOMEM;
    }

    for (size_t i = 0; i < bd->cfg->erase_count; i++) {
        copy->blocks[i] = lfs_emubd_incblock(bd->blocks[i]);
    }

    // other state
    copy->readed = bd->readed;
    copy->proged = bd->proged;
    copy->erased = bd->erased;
    copy->power_cycles = bd->power_cycles;
    copy->ooo_block = bd->ooo_block;
    copy->ooo_data = lfs_emubd_incblock(bd->ooo_data);
    copy->disk = bd->disk;
    if (copy->disk) {
        copy->disk->rc += 1;
    }
    copy->cfg = bd->cfg;

    LFS_EMUBD_TRACE("lfs_emubd_copy -> %d", 0);
    return 0;
}

