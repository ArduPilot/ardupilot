/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/* This is an, automatically generated, implementation file that can be
   manually edited, it is not re-generated if already present.*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

static BYTE translate_oflag(int oflag) {

  switch (oflag & VO_SUPPORTED_FLAGS_MASK) {
  case VO_RDONLY:                                   /* r */
    return FA_READ;
  case VO_RDWR:                                     /* r+ */
    return FA_READ | FA_WRITE;
  case VO_CREAT | VO_APPEND | VO_WRONLY:            /* a */
    return FA_OPEN_APPEND | FA_WRITE;
  case VO_CREAT | VO_APPEND | VO_RDWR:              /* a+ */
    return FA_OPEN_APPEND | FA_WRITE | FA_READ;
  case VO_CREAT | VO_WRONLY:                        /* w */
    return FA_CREATE_ALWAYS | FA_WRITE;
  case VO_CREAT | VO_RDWR:                          /* w+ */
    return FA_CREATE_ALWAYS | FA_WRITE | FA_READ;
  case VO_CREAT | VO_EXCL | VO_WRONLY:              /* wx */
    return  FA_CREATE_NEW | FA_WRITE;
  case VO_CREAT | VO_EXCL | VO_RDWR:                /* w+x */
    return  FA_CREATE_NEW | FA_WRITE | FA_READ;
  default:
    break;
  }

  return (BYTE)0;
}

static vfs_mode_t translate_mode(BYTE fattrib) {
  vfs_mode_t mode = VFS_MODE_S_IRUSR;

  if ((fattrib & AM_RDO) == 0) {
    mode |= VFS_MODE_S_IWUSR;
  }

  if ((fattrib & AM_DIR) == 0) {
    mode |= VFS_MODE_S_IFREG;
  }
  else {
    mode |= VFS_MODE_S_IFDIR;
  }

  return mode;
}

static msg_t translate_error(FRESULT res) {
  msg_t msg;

  switch (res) {
  case FR_OK:
    msg = CH_RET_SUCCESS;
    break;
  case FR_TIMEOUT:
    msg = CH_RET_TIMEOUT;
    break;
  case FR_NOT_ENOUGH_CORE:
    msg = CH_RET_ENOMEM;
    break;
  case FR_TOO_MANY_OPEN_FILES:
    msg = CH_RET_ENFILE;
    break;
  case FR_DISK_ERR:
  case FR_NOT_READY:
  case FR_INVALID_DRIVE:
  case FR_NO_FILESYSTEM:
    msg = CH_RET_EIO;
    break;
  case FR_NO_FILE:
  case FR_NO_PATH:
    msg = CH_RET_ENOENT;
    break;
  case FR_INVALID_NAME:
    msg = CH_RET_ENAMETOOLONG;
    break;
  case FR_DENIED:
  case FR_WRITE_PROTECTED:
    msg = CH_RET_EACCES;
    break;
  case FR_EXIST:
    msg = CH_RET_EEXIST;
    break;
  case FR_IS_DIRECTORY:
    msg = CH_RET_EISDIR;
    break;
  case FR_NOT_DIRECTORY:
    msg = CH_RET_ENOTDIR;
    break;
  default:
    msg = CH_RET_INNER_ERROR;
    break;
  }

  return msg;
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief       Module initialization.
 *
 * @init
 */
void __drv_fatfs_init(void) {

    /* Initializing pools.*/
    chPoolObjectInit(&vfs_fatfs_driver_static.dir_nodes_pool,
                     sizeof (vfs_fatfs_dir_node_c),
                     chCoreAllocAlignedI);
    chPoolObjectInit(&vfs_fatfs_driver_static.file_nodes_pool,
                     sizeof (vfs_fatfs_file_node_c),
                     chCoreAllocAlignedI);
    chPoolObjectInit(&vfs_fatfs_driver_static.info_nodes_pool,
                     sizeof (FILINFO),
                     chCoreAllocAlignedI);
    chPoolObjectInit(&vfs_fatfs_driver_static.fs_nodes_pool,
                     sizeof (FATFS),
                     NULL);

    /* Preloading pools.*/
    chPoolLoadArray(&vfs_fatfs_driver_static.dir_nodes_pool,
                    &vfs_fatfs_driver_static.dir_nodes[0],
                    DRV_CFG_FATFS_DIR_NODES_NUM);
    chPoolLoadArray(&vfs_fatfs_driver_static.file_nodes_pool,
                    &vfs_fatfs_driver_static.file_nodes[0],
                    DRV_CFG_FATFS_FILE_NODES_NUM);
    chPoolLoadArray(&vfs_fatfs_driver_static.fs_nodes_pool,
                    &__nocache_vfs_fatfs_driver_static.fs[0],
                    DRV_CFG_FATFS_FS_NUM);
}

/**
 * @brief       Mounts a FatFS volume.
 *
 * @param[in]     name          Name to be assigned to the volume, see FatFS @p
 *                              f_mount() documentation because there are
 *                              several options.
 * @param[in]     mountnow      Immediate mount option.
 * @return                      The operation result.
 *
 * @api
 */
msg_t ffdrvMount(const char *name, bool mountnow) {
  FATFS *fs;
  FRESULT res;

  fs = f_getfs(name);
  if (fs == NULL) {
    fs = chPoolAlloc(&vfs_fatfs_driver_static.fs_nodes_pool);
    if (fs == NULL) {
      return CH_RET_ENOMEM;
    }
  }

  res = f_mount(fs, name, (BYTE)(mountnow ? 1 : 0));
  if (res != FR_OK) {
    chPoolFree(&vfs_fatfs_driver_static.fs_nodes_pool, (void *)fs);
  }

  return translate_error(res);
}

/**
 * @brief       Unmounts a FatFS volume.
 *
 * @param[in]     name          Name of the volume to be unmounted.
 * @return                      The operation result.
 *
 * @api
 */
msg_t ffdrvUnmount(const char *name) {
  FATFS *fs;
  FRESULT res;

  fs = f_getfs(name);
  if (fs == NULL) {
    return CH_RET_EINVAL;
  }

  res = f_unmount(name);

  chPoolFree(&vfs_fatfs_driver_static.fs_nodes_pool, (void *)fs);

  return translate_error(res);
}

/*===========================================================================*/
/* Module class "vfs_fatfs_dir_node_c" methods.                              */
/*===========================================================================*/

/* TODO automatic forward declarations of all method implementations.*/
static msg_t __ffdir_next_impl(void *ip, vfs_direntry_info_t *dip);

/**
 * @name        Methods implementations of vfs_fatfs_dir_node_c
 * @{
 */
/**
 * @memberof    vfs_fatfs_dir_node_c
 * @protected
 *
 * @brief       Implementation of object creation.
 * @note        This function is meant to be used by derived classes.
 *
 * @param[out]    ip            Pointer to a @p vfs_fatfs_dir_node_c instance
 *                              to be initialized.
 * @param[in]     vmt           VMT pointer for the new object.
 * @param[in]     driver        Pointer to the controlling driver.
 * @param[in]     mode          Node mode flags.
 * @return                      A new reference to the object.
 */
static void *__ffdir_objinit_impl(void *ip, const void *vmt,
                                  vfs_driver_c *driver, vfs_mode_t mode) {
  vfs_fatfs_dir_node_c *self = (vfs_fatfs_dir_node_c *)ip;

  /* Initialization code.*/
  self = __vfsdir_objinit_impl(ip, vmt, (vfs_driver_c *)driver, mode);

  return self;
}

/**
 * @memberof    vfs_fatfs_dir_node_c
 * @protected
 *
 * @brief       Implementation of object finalization.
 * @note        This function is meant to be used by derived classes.
 *
 * @param[in,out] ip            Pointer to a @p vfs_fatfs_dir_node_c instance
 *                              to be disposed.
 */
static void __ffdir_dispose_impl(void *ip) {
  vfs_fatfs_dir_node_c *self = (vfs_fatfs_dir_node_c *)ip;

  /* Closing directory.*/
  (void) f_closedir(&self->dir);

  /* Finalization of the ancestors-defined parts.*/
  __vfsdir_dispose_impl(ip);

  /* Last because it corrupts the object.*/
  chPoolFree(&vfs_fatfs_driver_static.dir_nodes_pool, ip);
}

/**
 * @memberof    vfs_fatfs_dir_node_c
 * @protected
 *
 * @brief       Override of method @p vfsNodeStat().
 *
 * @param[in,out] ip            Pointer to a @p vfs_fatfs_dir_node_c instance.
 * @param[out]    sp            Pointer to a @p vfs_stat_t structure.
 * @return                      The operation result.
 */
static msg_t __ffdir_stat_impl(void *ip, vfs_stat_t *sp) {

  return __vfsnode_stat_impl(ip, sp);
}

/**
 * @memberof    vfs_fatfs_dir_node_c
 * @protected
 *
 * @brief       Override of method @p vfsDirReadFirst().
 *
 * @param[in,out] ip            Pointer to a @p vfs_fatfs_dir_node_c instance.
 * @param[out]    dip           Pointer to a @p vfs_direntry_info_t structure.
 * @return                      The operation result.
 */
static msg_t __ffdir_first_impl(void *ip, vfs_direntry_info_t *dip) {
  vfs_fatfs_dir_node_c *self = (vfs_fatfs_dir_node_c *)ip;
  msg_t ret;
  FRESULT res;

  res = f_rewinddir(&self->dir);
  if (res == FR_OK) {
    ret = __ffdir_next_impl(ip, dip);
  }
  else {
    ret = translate_error(res);
  }

  return ret;
}

/**
 * @memberof    vfs_fatfs_dir_node_c
 * @protected
 *
 * @brief       Override of method @p vfsDirReadNext().
 *
 * @param[in,out] ip            Pointer to a @p vfs_fatfs_dir_node_c instance.
 * @param[out]    dip           Pointer to a @p vfs_direntry_info_t structure.
 * @return                      The operation result.
 */
static msg_t __ffdir_next_impl(void *ip, vfs_direntry_info_t *dip) {
  vfs_fatfs_dir_node_c *self = (vfs_fatfs_dir_node_c *)ip;
  msg_t ret;

  do {
    FRESULT res;
    FILINFO *fip;

    fip = (FILINFO *)chPoolAlloc(&vfs_fatfs_driver_static.info_nodes_pool);
    if (fip != NULL) {

      res = f_readdir(&self->dir, fip);
      if (res == FR_OK) {
        if (fip->fname[0] == '\0') {
          ret = (msg_t)0;
        }
        else {
          dip->mode = translate_mode(fip->fattrib);
          dip->size = (vfs_offset_t)fip->fsize;
          strncpy(dip->name, fip->fname, VFS_CFG_NAMELEN_MAX);
          dip->name[VFS_CFG_NAMELEN_MAX] = '\0';
          ret = (msg_t)1;
        }
      }
      else {
        ret = translate_error(res);
      }

      chPoolFree(&vfs_fatfs_driver_static.info_nodes_pool, (void *)fip);
    }
    else {
      ret = CH_RET_ENOMEM;
      break;
    }
  }
  while (false);

  return ret;
}
/** @} */

/**
 * @brief       VMT structure of VFS fatfs directory node class.
 * @note        It is public because accessed by the inlined constructor.
 */
static const struct vfs_fatfs_dir_node_vmt __vfs_fatfs_dir_node_vmt = {
  .dispose                  = __ffdir_dispose_impl,
  .addref                   = __ro_addref_impl,
  .release                  = __ro_release_impl,
  .stat                     = __ffdir_stat_impl,
  .first                    = __ffdir_first_impl,
  .next                     = __ffdir_next_impl
};

/**
 * @name        Default constructor of vfs_fatfs_dir_node_c
 * @{
 */
/**
 * @memberof    vfs_fatfs_dir_node_c
 *
 * @brief       Default initialization function of @p vfs_fatfs_dir_node_c.
 *
 * @param[out]    self          Pointer to a @p vfs_fatfs_dir_node_c instance
 *                              to be initialized.
 * @param[in]     driver        Pointer to the controlling driver.
 * @param[in]     mode          Node mode flags.
 * @return                      Pointer to the initialized object.
 *
 * @objinit
 */
static vfs_fatfs_dir_node_c *ffdirObjectInit(vfs_fatfs_dir_node_c *self,
                                             vfs_driver_c *driver,
                                             vfs_mode_t mode) {

  return __ffdir_objinit_impl(self, &__vfs_fatfs_dir_node_vmt, driver, mode);
}
/** @} */

/*===========================================================================*/
/* Module class "vfs_fatfs_file_node_c" methods.                             */
/*===========================================================================*/

/**
 * @name        Interfaces implementation of vfs_fatfs_file_node_c
 * @{
 */
/**
 * @memberof    vfs_fatfs_file_node_c
 * @private
 *
 * @brief       Implementation of interface method @p stmWrite().
 *
 * @param[in,out] ip            Pointer to the @p sequential_stream_i class
 *                              interface.
 * @param[in]     bp            Pointer to the data buffer.
 * @param[in]     n             The maximum amount of data to be transferred.
 * @return                      The number of bytes transferred. The returned
 *                              value can be less than the specified number of
 *                              bytes if an end-of-file condition has been met.
 */
static size_t __fffile_stm_write_impl(void *ip, const uint8_t *bp, size_t n) {
  vfs_fatfs_file_node_c *self = oopIfGetOwner(vfs_fatfs_file_node_c, ip);
  msg_t msg;

  msg = vfsFileWrite((void *)self, bp, n);
  if (CH_RET_IS_ERROR(msg)) {

    return (size_t)0;
  }

  return (size_t)msg;
}

/**
 * @memberof    vfs_fatfs_file_node_c
 * @private
 *
 * @brief       Implementation of interface method @p stmRead().
 *
 * @param[in,out] ip            Pointer to the @p sequential_stream_i class
 *                              interface.
 * @param[out]    bp            Pointer to the data buffer.
 * @param[in]     n             The maximum amount of data to be transferred.
 * @return                      The number of bytes transferred. The returned
 *                              value can be less than the specified number of
 *                              bytes if an end-of-file condition has been met.
 */
static size_t __fffile_stm_read_impl(void *ip, uint8_t *bp, size_t n) {
  vfs_fatfs_file_node_c *self = oopIfGetOwner(vfs_fatfs_file_node_c, ip);
  msg_t msg;

  msg = vfsFileRead((void *)self, bp, n);
  if (CH_RET_IS_ERROR(msg)) {

    return (size_t)0;
  }

  return (size_t)msg;
}

/**
 * @memberof    vfs_fatfs_file_node_c
 * @private
 *
 * @brief       Implementation of interface method @p stmPut().
 *
 * @param[in,out] ip            Pointer to the @p sequential_stream_i class
 *                              interface.
 * @param[in]     b             The byte value to be written to the stream.
 * @return                      The operation status.
 */
static int __fffile_stm_put_impl(void *ip, uint8_t b) {
  vfs_fatfs_file_node_c *self = oopIfGetOwner(vfs_fatfs_file_node_c, ip);
  msg_t msg;

  msg = vfsFileWrite((void *)self, &b, (size_t)1);
  if (CH_RET_IS_ERROR(msg)) {

    return STM_TIMEOUT;
  }

  return msg;
}

/**
 * @memberof    vfs_fatfs_file_node_c
 * @private
 *
 * @brief       Implementation of interface method @p stmGet().
 *
 * @param[in,out] ip            Pointer to the @p sequential_stream_i class
 *                              interface.
 * @return                      A byte value from the stream.
 */
static int __fffile_stm_get_impl(void *ip) {
  vfs_fatfs_file_node_c *self = oopIfGetOwner(vfs_fatfs_file_node_c, ip);
  msg_t msg;
  uint8_t b;

  msg = vfsFileRead((void *)self, &b, (size_t)1);
  if (CH_RET_IS_ERROR(msg)) {

    return STM_TIMEOUT;
  }

  return (msg_t)b;
}
/** @} */

/**
 * @name        Methods implementations of vfs_fatfs_file_node_c
 * @{
 */
/**
 * @memberof    vfs_fatfs_file_node_c
 * @protected
 *
 * @brief       Implementation of object creation.
 * @note        This function is meant to be used by derived classes.
 *
 * @param[out]    ip            Pointer to a @p vfs_fatfs_file_node_c instance
 *                              to be initialized.
 * @param[in]     vmt           VMT pointer for the new object.
 * @param[in]     driver        Pointer to the controlling driver.
 * @param[in]     mode          Node mode flags.
 * @return                      A new reference to the object.
 */
static void *__fffile_objinit_impl(void *ip, const void *vmt,
                                   vfs_driver_c *driver, vfs_mode_t mode) {
  vfs_fatfs_file_node_c *self = (vfs_fatfs_file_node_c *)ip;

  /* Initialization of interface sequential_stream_i.*/
  {
    static const struct sequential_stream_vmt fffile_stm_vmt = {
      .instance_offset      = offsetof(vfs_fatfs_file_node_c, stm),
      .write                = __fffile_stm_write_impl,
      .read                 = __fffile_stm_read_impl,
      .put                  = __fffile_stm_put_impl,
      .get                  = __fffile_stm_get_impl
    };
    oopIfObjectInit(&self->stm, &fffile_stm_vmt);
  }

  /* Initialization code.*/
  self = __vfsfile_objinit_impl(ip, vmt, (vfs_driver_c *)driver, mode);

  return self;
}

/**
 * @memberof    vfs_fatfs_file_node_c
 * @protected
 *
 * @brief       Implementation of object finalization.
 * @note        This function is meant to be used by derived classes.
 *
 * @param[in,out] ip            Pointer to a @p vfs_fatfs_file_node_c instance
 *                              to be disposed.
 */
static void __fffile_dispose_impl(void *ip) {
  vfs_fatfs_file_node_c *self = (vfs_fatfs_file_node_c *)ip;

  /* Closing file.*/
  (void) f_close(&self->file);

  /* Finalization of the ancestors-defined parts.*/
  __vfsfile_dispose_impl(ip);

  /* Last because it corrupts the object.*/
  chPoolFree(&vfs_fatfs_driver_static.file_nodes_pool, ip);
}

/**
 * @memberof    vfs_fatfs_file_node_c
 * @protected
 *
 * @brief       Override of method @p vfsNodeStat().
 *
 * @param[in,out] ip            Pointer to a @p vfs_fatfs_file_node_c instance.
 * @param[out]    sp            Pointer to a @p vfs_stat_t structure.
 * @return                      The operation result.
 */
static msg_t __fffile_stat_impl(void *ip, vfs_stat_t *sp) {
  vfs_fatfs_file_node_c *self = (vfs_fatfs_file_node_c *)ip;

  sp->mode = self->mode;
  sp->size = (vfs_offset_t)self->file.obj.objsize;

  return CH_RET_SUCCESS;
}

/**
 * @memberof    vfs_fatfs_file_node_c
 * @protected
 *
 * @brief       Override of method @p vfsFileRead().
 *
 * @param[in,out] ip            Pointer to a @p vfs_fatfs_file_node_c instance.
 * @param[out]    buf           Pointer to the data buffer.
 * @param[in]     n             Maximum amount of data to be transferred.
 * @return                      The transferred number of bytes or an error.
 */
static ssize_t __fffile_read_impl(void *ip, uint8_t *buf, size_t n) {
  vfs_fatfs_file_node_c *self = (vfs_fatfs_file_node_c *)ip;
  FRESULT res;
  UINT br;

  res = f_read(&self->file, (void *)buf, (UINT)n, &br);
  if (res != FR_OK) {

    return translate_error(res);
  }

  return (ssize_t)br;
}

/**
 * @memberof    vfs_fatfs_file_node_c
 * @protected
 *
 * @brief       Override of method @p vfsFileWrite().
 *
 * @param[in,out] ip            Pointer to a @p vfs_fatfs_file_node_c instance.
 * @param[in]     buf           Pointer to the data buffer.
 * @param[in]     n             Maximum amount of data to be transferred.
 * @return                      The transferred number of bytes or an error.
 */
static ssize_t __fffile_write_impl(void *ip, const uint8_t *buf, size_t n) {
  vfs_fatfs_file_node_c *self = (vfs_fatfs_file_node_c *)ip;
  FRESULT res;
  UINT bw;

  res = f_write(&self->file, (const void *)buf, (UINT)n, &bw);
  if (res != FR_OK) {

    return translate_error(res);
  }

  return (ssize_t)bw;
}

/**
 * @memberof    vfs_fatfs_file_node_c
 * @protected
 *
 * @brief       Override of method @p vfsFileSetPosition().
 *
 * @param[in,out] ip            Pointer to a @p vfs_fatfs_file_node_c instance.
 * @param[in]     offset        Offset to be applied.
 * @param[in]     whence        Seek mode to be used.
 * @return                      The operation result.
 */
static msg_t __fffile_setpos_impl(void *ip, vfs_offset_t offset,
                                  vfs_seekmode_t whence) {
  vfs_fatfs_file_node_c *self = (vfs_fatfs_file_node_c *)ip;
  vfs_offset_t finaloff;

  chDbgCheck((whence == SEEK_SET) ||
             (whence == SEEK_CUR) ||
             (whence == SEEK_END));

  switch (whence) {
  case VFS_SEEK_CUR:
    finaloff = offset + (vfs_offset_t)self->file.fptr;
    break;
  case VFS_SEEK_END:
    finaloff = offset + (vfs_offset_t)self->file.obj.objsize;
    break;
  case VFS_SEEK_SET:
  default:
    finaloff = offset;
    break;
  }

  if (finaloff < 0) {
    return CH_RET_EOVERFLOW;
  }

  return translate_error(f_lseek(&self->file, (FSIZE_t)finaloff));
}

/**
 * @memberof    vfs_fatfs_file_node_c
 * @protected
 *
 * @brief       Override of method @p vfsFileGetPosition().
 *
 * @param[in,out] ip            Pointer to a @p vfs_fatfs_file_node_c instance.
 * @return                      The current file position.
 */
static vfs_offset_t __fffile_getpos_impl(void *ip) {
  vfs_fatfs_file_node_c *self = (vfs_fatfs_file_node_c *)ip;

  return (vfs_offset_t)f_tell(&self->file);
}

/**
 * @memberof    vfs_fatfs_file_node_c
 * @protected
 *
 * @brief       Override of method @p vfsFileGetStream().
 *
 * @param[in,out] ip            Pointer to a @p vfs_fatfs_file_node_c instance.
 * @return                      Pointer to the HAL stream interface.
 */
static sequential_stream_i *__fffile_getstream_impl(void *ip) {
  vfs_fatfs_file_node_c *self = (vfs_fatfs_file_node_c *)ip;

  return &self->stm;
}
/** @} */

/**
 * @brief       VMT structure of VFS fatfs file node class.
 * @note        It is public because accessed by the inlined constructor.
 */
static const struct vfs_fatfs_file_node_vmt __vfs_fatfs_file_node_vmt = {
  .dispose                  = __fffile_dispose_impl,
  .addref                   = __ro_addref_impl,
  .release                  = __ro_release_impl,
  .stat                     = __fffile_stat_impl,
  .read                     = __fffile_read_impl,
  .write                    = __fffile_write_impl,
  .setpos                   = __fffile_setpos_impl,
  .getpos                   = __fffile_getpos_impl,
  .getstream                = __fffile_getstream_impl
};

/**
 * @name        Default constructor of vfs_fatfs_file_node_c
 * @{
 */
/**
 * @memberof    vfs_fatfs_file_node_c
 *
 * @brief       Default initialization function of @p vfs_fatfs_file_node_c.
 *
 * @param[out]    self          Pointer to a @p vfs_fatfs_file_node_c instance
 *                              to be initialized.
 * @param[in]     driver        Pointer to the controlling driver.
 * @param[in]     mode          Node mode flags.
 * @return                      Pointer to the initialized object.
 *
 * @objinit
 */
static vfs_fatfs_file_node_c *fffileObjectInit(vfs_fatfs_file_node_c *self,
                                               vfs_driver_c *driver,
                                               vfs_mode_t mode) {

  return __fffile_objinit_impl(self, &__vfs_fatfs_file_node_vmt, driver, mode);
}
/** @} */

/*===========================================================================*/
/* Module class "vfs_fatfs_driver_c" methods.                                */
/*===========================================================================*/

/**
 * @name        Methods implementations of vfs_fatfs_driver_c
 * @{
 */
/**
 * @memberof    vfs_fatfs_driver_c
 * @protected
 *
 * @brief       Implementation of object creation.
 * @note        This function is meant to be used by derived classes.
 *
 * @param[out]    ip            Pointer to a @p vfs_fatfs_driver_c instance to
 *                              be initialized.
 * @param[in]     vmt           VMT pointer for the new object.
 * @return                      A new reference to the object.
 */
void *__ffdrv_objinit_impl(void *ip, const void *vmt) {
  vfs_fatfs_driver_c *self = (vfs_fatfs_driver_c *)ip;

  /* Initialization of the ancestors-defined parts.*/
  __vfsdrv_objinit_impl(self, vmt);

  /* No initialization code.*/

  return self;
}

/**
 * @memberof    vfs_fatfs_driver_c
 * @protected
 *
 * @brief       Implementation of object finalization.
 * @note        This function is meant to be used by derived classes.
 *
 * @param[in,out] ip            Pointer to a @p vfs_fatfs_driver_c instance to
 *                              be disposed.
 */
void __ffdrv_dispose_impl(void *ip) {
  vfs_fatfs_driver_c *self = (vfs_fatfs_driver_c *)ip;

  /* No finalization code.*/
  (void)self;

  /* Finalization of the ancestors-defined parts.*/
  __vfsdrv_dispose_impl(self);
}

/**
 * @memberof    vfs_fatfs_driver_c
 * @protected
 *
 * @brief       Override of method @p vfsDrvChangeCurrentDirectory().
 *
 * @param[in,out] ip            Pointer to a @p vfs_fatfs_driver_c instance.
 * @param[in]     path          Path of the new current directory.
 * @return                      The operation result.
 */
msg_t __ffdrv_setcwd_impl(void *ip, const char *path) {
#if FF_FS_RPATH >= 1

  (void)ip;

  return translate_error(f_chdir((const TCHAR *)path));
#else

  (void)ip;

  if (strcmp(path, "/") != 0) {
    return CH_RET_ENOENT;
  }

  return CH_RET_SUCCESS;
#endif
}

/**
 * @memberof    vfs_fatfs_driver_c
 * @protected
 *
 * @brief       Override of method @p vfsDrvGetCurrentDirectory().
 *
 * @param[in,out] ip            Pointer to a @p vfs_fatfs_driver_c instance.
 * @param[out]    buf           Buffer for the path string.
 * @param[in]     size          Size of the buffer.
 * @return                      The operation result.
 */
msg_t __ffdrv_getcwd_impl(void *ip, char *buf, size_t size) {
#if FF_FS_RPATH >= 2

  (void)ip;

  return translate_error(f_getcwd((TCHAR *)buf, (UINT)size));
#else

  (void)ip;

  if (size < 2) {
    return CH_RET_ERANGE;
  }

  buf[0] = '/';
  buf[1] = '\0';

  return CH_RET_SUCCESS;
#endif
}

/**
 * @memberof    vfs_fatfs_driver_c
 * @protected
 *
 * @brief       Override of method @p vfsDrvStat().
 *
 * @param[in,out] ip            Pointer to a @p vfs_fatfs_driver_c instance.
 * @param[in]     path          Absolute path of the node to be examined.
 * @param[out]    sp            Pointer to a @p vfs_stat_t structure.
 * @return                      The operation result.
 */
msg_t __ffdrv_stat_impl(void *ip, const char *path, vfs_stat_t *sp) {
  msg_t ret;

  (void)ip;

  do {
    FRESULT res;
    FILINFO *fip;

    fip = (FILINFO *)chPoolAlloc(&vfs_fatfs_driver_static.info_nodes_pool);
    if (fip != NULL) {

      res = f_stat((const TCHAR *)path, fip);
      if (res == FR_OK) {

        sp->mode = translate_mode(fip->fattrib);
        sp->size = (vfs_offset_t)fip->fsize;

        ret = CH_RET_SUCCESS;
      }
      else {
        ret = translate_error(res);
      }

      chPoolFree(&vfs_fatfs_driver_static.info_nodes_pool, (void *)fip);
    }
    else {
      ret = CH_RET_ENOMEM;
      break;
    }
  }
  while (false);

  return ret;
}

/**
 * @memberof    vfs_fatfs_driver_c
 * @protected
 *
 * @brief       Override of method @p vfsDrvOpenDirectory().
 *
 * @param[in,out] ip            Pointer to a @p vfs_fatfs_driver_c instance.
 * @param[in]     path          Absolute path of the directory to be opened.
 * @param[out]    vdnpp         Pointer to the pointer to the instantiated @p
 *                              vfs_directory_node_c object.
 * @return                      The operation result.
 */
msg_t __ffdrv_opendir_impl(void *ip, const char *path,
                           vfs_directory_node_c **vdnpp) {
  vfs_fatfs_driver_c *self = (vfs_fatfs_driver_c *)ip;
  msg_t ret;

  do {
    vfs_fatfs_dir_node_c *ffdnp;
    FRESULT res;

    ffdnp = chPoolAlloc(&vfs_fatfs_driver_static.dir_nodes_pool);
    if (ffdnp != NULL) {

      res = f_opendir(&ffdnp->dir, (const TCHAR *)path);
      if (res == FR_OK) {

        /* Node object initialization.*/
        (void) ffdirObjectInit(ffdnp,
                               (vfs_driver_c *)self,
                               translate_mode(ffdnp->dir.obj.attr));

        *vdnpp = (vfs_directory_node_c *)ffdnp;
        ret = CH_RET_SUCCESS;
        break;
      }

      chPoolFree(&vfs_fatfs_driver_static.dir_nodes_pool, (void *)ffdnp);
    }
    else {
      ret = CH_RET_ENOMEM;
      break;
    }

    ret = translate_error(res);
  }
  while (false);

  return ret;
}

/**
 * @memberof    vfs_fatfs_driver_c
 * @protected
 *
 * @brief       Override of method @p vfsDrvOpenFile().
 *
 * @param[in,out] ip            Pointer to a @p vfs_fatfs_driver_c instance.
 * @param[in]     path          Absolute path of the directory to be opened.
 * @param[in]     flags         File open flags.
 * @param[out]    vfnpp         Pointer to the pointer to the instantiated @p
 *                              vfs_file_node_c object.
 * @return                      The operation result.
 */
msg_t __ffdrv_openfile_impl(void *ip, const char *path, int flags,
                            vfs_file_node_c **vfnpp) {
  vfs_fatfs_driver_c *self = (vfs_fatfs_driver_c *)ip;
  msg_t ret;

  do {
    vfs_fatfs_file_node_c *fffnp;
    FRESULT res;
    BYTE mode;

    mode = translate_oflag(flags);
    if (mode == (BYTE)0) {
      ret = CH_RET_EINVAL;
      break;
    }

    fffnp = chPoolAlloc(&vfs_fatfs_driver_static.file_nodes_pool);
    if (fffnp != NULL) {

      res = f_open(&fffnp->file, (const TCHAR *)path, mode);
      if (res == FR_OK) {

        /* Node object initialization.*/
        (void) fffileObjectInit(fffnp,
                                (vfs_driver_c *)self,
                                translate_mode(fffnp->file.obj.attr));

        *vfnpp = (vfs_file_node_c *)fffnp;
        ret = CH_RET_SUCCESS;
        break;
      }

      chPoolFree(&vfs_fatfs_driver_static.file_nodes_pool, (void *)fffnp);
    }
    else {
      ret = CH_RET_ENOMEM;
      break;
    }

    ret = translate_error(res);
  }
  while (false);

  return ret;
}

/**
 * @memberof    vfs_fatfs_driver_c
 * @protected
 *
 * @brief       Override of method @p vfsDrvUnlink().
 *
 * @param[in,out] ip            Pointer to a @p vfs_fatfs_driver_c instance.
 * @param[in]     path          Path of the file to be unlinked.
 * @return                      The operation result.
 */
msg_t __ffdrv_unlink_impl(void *ip, const char *path) {

  (void)ip;

  return translate_error(f_unlink((const TCHAR *)path));
}

/**
 * @memberof    vfs_fatfs_driver_c
 * @protected
 *
 * @brief       Override of method @p vfsDrvRename().
 *
 * @param[in,out] ip            Pointer to a @p vfs_fatfs_driver_c instance.
 * @param[in]     oldpath       Path of the node to be renamed.
 * @param[in]     newpath       New path of the renamed node.
 * @return                      The operation result.
 */
msg_t __ffdrv_rename_impl(void *ip, const char *oldpath, const char *newpath) {

  (void)ip;

  return translate_error(f_rename((const TCHAR *)oldpath,
                                  (const TCHAR *)newpath));
}

/**
 * @memberof    vfs_fatfs_driver_c
 * @protected
 *
 * @brief       Override of method @p vfsDrvMkdir().
 *
 * @param[in,out] ip            Pointer to a @p vfs_fatfs_driver_c instance.
 * @param[in]     path          Path of the directory to be created.
 * @param[in]     mode          Mode flags for the directory.
 * @return                      The operation result.
 */
msg_t __ffdrv_mkdir_impl(void *ip, const char *path, vfs_mode_t mode) {

  (void)ip;
  (void)mode; /* Not handled by FatFS.*/

  return translate_error(f_mkdir((const TCHAR *)path));
}

/**
 * @memberof    vfs_fatfs_driver_c
 * @protected
 *
 * @brief       Override of method @p vfsDrvRmdir().
 *
 * @param[in,out] ip            Pointer to a @p vfs_fatfs_driver_c instance.
 * @param[in]     path          Path of the directory to be removed.
 * @return                      The operation result.
 */
msg_t __ffdrv_rmdir_impl(void *ip, const char *path) {

  (void)ip;

  return translate_error(f_rmdir((const TCHAR *)path));
}
/** @} */

/**
 * @brief       VMT structure of VFS fatfs driver class.
 * @note        It is public because accessed by the inlined constructor.
 */
const struct vfs_fatfs_driver_vmt __vfs_fatfs_driver_vmt = {
  .dispose                  = __ffdrv_dispose_impl,
  .setcwd                   = __ffdrv_setcwd_impl,
  .getcwd                   = __ffdrv_getcwd_impl,
  .stat                     = __ffdrv_stat_impl,
  .opendir                  = __ffdrv_opendir_impl,
  .openfile                 = __ffdrv_openfile_impl,
  .unlink                   = __ffdrv_unlink_impl,
  .rename                   = __ffdrv_rename_impl,
  .mkdir                    = __ffdrv_mkdir_impl,
  .rmdir                    = __ffdrv_rmdir_impl
};

