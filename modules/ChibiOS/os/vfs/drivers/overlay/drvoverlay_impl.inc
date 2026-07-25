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

static msg_t match_driver(vfs_overlay_driver_c *odp,
                          const char **pathp,
                          vfs_driver_c **vdpp) {
  unsigned i;

  i = 0U;
  while (i < odp->next_driver) {
    size_t n;

    n = vfs_path_match_element(*pathp, odp->names[i], VFS_CFG_NAMELEN_MAX + 1);
    if (n < VFS_CFG_NAMELEN_MAX + 1) {
      *pathp += n;
      *vdpp = odp->drivers[i];
      return CH_RET_SUCCESS;
    }

    i++;
  }

  return CH_RET_ENOENT;
}

static const char *get_current_directory(vfs_overlay_driver_c *drvp) {
  const char *cwd = drvp->path_cwd;

  /* In case it has not yet been defined using root.*/
  if (cwd == NULL) {
    return "/";
  }

  return cwd;
}

static msg_t build_absolute_path(vfs_overlay_driver_c *drvp,
                                 char *buf,
                                 const char *path) {
  msg_t ret;

  do {

    /* Initial buffer state, empty string.*/
    *buf = '\0';

    /* Relative paths handling.*/
    if (!vfs_path_is_separator(*path)) {
      if (vfs_path_append(buf,
                      get_current_directory(drvp),
                      VFS_CFG_PATHLEN_MAX + 1) == (size_t)0) {
        ret = CH_RET_ENAMETOOLONG;
        break;
      }
    }

    /* Adding the specified path.*/
    if (vfs_path_append(buf, path, VFS_CFG_PATHLEN_MAX + 1) == (size_t)0) {
      ret = CH_RET_ENAMETOOLONG;
      break;
    }

    /* Normalization of the absolute path.*/
    if (vfs_path_normalize(buf, buf, VFS_CFG_PATHLEN_MAX + 1) == (size_t)0) {
      ret = CH_RET_ENAMETOOLONG;
      break;
    }

    ret = CH_RET_SUCCESS;

  } while (false);

  return ret;
}

static msg_t open_absolute_dir(vfs_overlay_driver_c *drvp,
                               char *path,
                               vfs_directory_node_c **vdnpp) {
  msg_t ret;

  do {
    const char *scanpath;

    /* Making sure there is a final separator.*/
    if (vfs_path_add_separator(path, VFS_CFG_PATHLEN_MAX + 1) == (size_t)0) {
      ret = CH_RET_ENAMETOOLONG;
      break;
    }

    /* Initial separator is expected, skipping it.*/
    scanpath = path + 1;

    /* If it is the root.*/
    if (*scanpath == '\0') {
      vfs_overlay_dir_node_c *self;

      /* Creating a root directory node.*/
      self = chPoolAlloc(&vfs_overlay_driver_static.dir_nodes_pool);
      if (self != NULL) {

        /* Node object initialization.*/
        (void) ovldirObjectInit(self, drvp, VFS_MODE_S_IFDIR | VFS_MODE_S_IRUSR);

        /* Trying to obtain a root node from the overlaid driver, it
           could fail, in that case the pointer stays at NULL.*/
        if (drvp->overlaid_drv != NULL) {
          (void) vfsDrvOpenDirectory((void *)drvp->overlaid_drv,
                                     drvp->path_prefix == NULL ? "/" : drvp->path_prefix,
                                     &self->overlaid_root);
        }
        *vdnpp = (vfs_directory_node_c *)self;

        ret = CH_RET_SUCCESS;
        break;
      }
      else {
        ret = CH_RET_ENOMEM;
        break;
      }
    }
    else { /* Not the root.*/
      vfs_driver_c *dp;

      /* Searching for a match among registered overlays.*/
      ret = match_driver(drvp, &scanpath, &dp);
      if (!CH_RET_IS_ERROR(ret)) {
        /* Delegating node creation to a registered driver.*/
        ret = vfsDrvOpenDirectory((void *)dp,
                                  *scanpath == '\0' ? "/" : scanpath,
                                  vdnpp);
      }
      else {
        size_t path_offset;

        /* Is there an overlaid driver? if so we need to pass request
           processing there.*/
        if (drvp->overlaid_drv != NULL) {

          /* Processing the prefix, if defined.*/
          if (drvp->path_prefix != NULL) {
            if ((path_offset = vfs_path_prepend(path,
                                                drvp->path_prefix,
                                                VFS_CFG_PATHLEN_MAX + 1)) == (size_t)0) {
              ret = CH_RET_ENAMETOOLONG;
              break;
            }
          }
          else {
            path_offset = 0U;
          }

          /* Passing the combined path to the overlaid driver.*/
          ret = vfsDrvOpenDirectory((void *)drvp->overlaid_drv,
                                    path,
                                    vdnpp);
          CH_BREAK_ON_ERROR(ret);

          ret = (msg_t)path_offset;
        }
      }
    }
  }
  while (false);

  return ret;
}

static msg_t open_absolute_file(vfs_overlay_driver_c *drvp,
                                char *path,
                                int oflag,
                                vfs_file_node_c **vfnpp) {
  msg_t ret;

  do {
    const char *scanpath;

    /* Initial separator is expected, skipping it.*/
    scanpath = path + 1;

    /* If it is the root.*/
    if (*scanpath == '\0') {

      /* Always not found, root is not a file.*/
      ret = CH_RET_EISDIR;
    }
    else {
      vfs_driver_c *dp;

      /* Searching for a match among registered overlays.*/
      ret = match_driver(drvp, &scanpath, &dp);
      if (!CH_RET_IS_ERROR(ret)) {
        /* Delegating node creation to a registered driver, making sure it
           does not receive an empty path.*/
        ret = vfsDrvOpenFile((void *)dp, *scanpath == '\0' ? "/" : scanpath, oflag, vfnpp);
      }
      else {
        /* Is there an overlaid driver? if so we need to pass request
           processing there.*/
        if (drvp->overlaid_drv != NULL) {

          /* Processing the prefix, if defined.*/
          if (drvp->path_prefix != NULL) {
            if (vfs_path_prepend(path,
                                 drvp->path_prefix,
                                 VFS_CFG_PATHLEN_MAX + 1) == (size_t)0) {
              ret = CH_RET_ENAMETOOLONG;
              break;
            }
          }

          /* Passing the combined path to the overlaid driver.*/
          ret = vfsDrvOpenFile((void *)drvp->overlaid_drv, path, oflag, vfnpp);
        }
      }
    }
  }
  while (false);

  return ret;
}

static msg_t drv_overlaid_path_call(vfs_overlay_driver_c *drvp,
                                    char *buf,
                                    msg_t (*fn)(void *ip, const char *path)) {
  msg_t ret;

  do {
    /* Is there an overlaid driver? if so we need to pass request
       processing there.*/
    if (drvp->overlaid_drv != NULL) {

      /* Processing the prefix, if defined.*/
      if (drvp->path_prefix != NULL) {
        if (vfs_path_prepend(buf,
                             drvp->path_prefix,
                             VFS_CFG_PATHLEN_MAX + 1) == (size_t)0) {
          ret = CH_RET_ENAMETOOLONG;
          break;
        }
      }

      /* Passing the combined path to the overlaid driver.*/
      ret = fn((void *)drvp->overlaid_drv, buf);
    }
    else {
      ret = CH_RET_ENOENT;
    }
  } while (false);

  return ret;
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief       Module initialization.
 *
 * @init
 */
void __drv_overlay_init(void) {

  /* Initializing pools.*/
  chPoolObjectInit(&vfs_overlay_driver_static.dir_nodes_pool,
                   sizeof (vfs_overlay_dir_node_c),
                   chCoreAllocAlignedI);

  /* Preloading pools.*/
  chPoolLoadArray(&vfs_overlay_driver_static.dir_nodes_pool,
                  &vfs_overlay_driver_static.dir_nodes[0],
                  DRV_CFG_OVERLAY_DIR_NODES_NUM);
}

/*===========================================================================*/
/* Module class "vfs_overlay_dir_node_c" methods.                            */
/*===========================================================================*/

/**
 * @name        Methods implementations of vfs_overlay_dir_node_c
 * @{
 */
/**
 * @memberof    vfs_overlay_dir_node_c
 * @protected
 *
 * @brief       Implementation of object creation.
 * @note        This function is meant to be used by derived classes.
 *
 * @param[out]    ip            Pointer to a @p vfs_overlay_dir_node_c instance
 *                              to be initialized.
 * @param[in]     vmt           VMT pointer for the new object.
 * @param[in]     driver        Pointer to the controlling driver.
 * @param[in]     mode          Node mode flags.
 * @return                      A new reference to the object.
 */
void *__ovldir_objinit_impl(void *ip, const void *vmt,
                            vfs_overlay_driver_c *driver, vfs_mode_t mode) {
  vfs_overlay_dir_node_c *self = (vfs_overlay_dir_node_c *)ip;

  /* Initialization code.*/
  self = __vfsdir_objinit_impl(ip, vmt, (vfs_driver_c *)driver, mode);

  self->index         = 0U;
  self->overlaid_root = NULL;

  return self;
}

/**
 * @memberof    vfs_overlay_dir_node_c
 * @protected
 *
 * @brief       Implementation of object finalization.
 * @note        This function is meant to be used by derived classes.
 *
 * @param[in,out] ip            Pointer to a @p vfs_overlay_dir_node_c instance
 *                              to be disposed.
 */
void __ovldir_dispose_impl(void *ip) {
  vfs_overlay_dir_node_c *self = (vfs_overlay_dir_node_c *)ip;

  /* Releasing overridden driver object.*/
  if (self->overlaid_root != NULL) {
    (void) __ro_release_impl((void *)self->overlaid_root);
  }

  /* Finalization of the ancestors-defined parts.*/
  __vfsdir_dispose_impl(self);

  /* Last because it corrupts the object.*/
  chPoolFree(&vfs_overlay_driver_static.dir_nodes_pool, ip);
}

/**
 * @memberof    vfs_overlay_dir_node_c
 * @protected
 *
 * @brief       Override of method @p vfsNodeStat().
 *
 * @param[in,out] ip            Pointer to a @p vfs_overlay_dir_node_c
 *                              instance.
 * @param[out]    sp            Pointer to a @p vfs_stat_t structure.
 * @return                      The operation result.
 */
msg_t __ovldir_stat_impl(void *ip, vfs_stat_t *sp) {
  vfs_overlay_dir_node_c *self = (vfs_overlay_dir_node_c *)ip;

  return __vfsnode_stat_impl(self, sp); /* TODO */
}

/**
 * @memberof    vfs_overlay_dir_node_c
 * @protected
 *
 * @brief       Override of method @p vfsDirReadFirst().
 *
 * @param[in,out] ip            Pointer to a @p vfs_overlay_dir_node_c
 *                              instance.
 * @param[out]    dip           Pointer to a @p vfs_direntry_info_t structure.
 * @return                      The operation result.
 */
msg_t __ovldir_first_impl(void *ip, vfs_direntry_info_t *dip) {
  vfs_overlay_dir_node_c *self = (vfs_overlay_dir_node_c *)ip;

  self->index = 0U;

  return __ovldir_next_impl(self, dip);
}

/**
 * @memberof    vfs_overlay_dir_node_c
 * @protected
 *
 * @brief       Override of method @p vfsDirReadNext().
 *
 * @param[in,out] ip            Pointer to a @p vfs_overlay_dir_node_c
 *                              instance.
 * @param[out]    dip           Pointer to a @p vfs_direntry_info_t structure.
 * @return                      The operation result.
 */
msg_t __ovldir_next_impl(void *ip, vfs_direntry_info_t *dip) {
  vfs_overlay_dir_node_c *self = (vfs_overlay_dir_node_c *)ip;
  vfs_overlay_driver_c *drvp = (vfs_overlay_driver_c *)self->driver;

  if (self->index < drvp->next_driver) {
    dip->mode = VFS_MODE_S_IFDIR | VFS_MODE_S_IRUSR;
    dip->size = (vfs_offset_t)0;
    strcpy(dip->name, drvp->names[self->index]);

    self->index++;

    return (msg_t)1;
  }
  if (self->overlaid_root != NULL) {
    if (self->index == drvp->next_driver) {

      self->index++;

      return vfsDirReadFirst((void *)self->overlaid_root, dip);
    }
    if (self->index > drvp->next_driver) {

      return vfsDirReadNext((void *)self->overlaid_root, dip);
    }
  }

  return (msg_t)0;
}
/** @} */

/**
 * @brief       VMT structure of VFS overlay directory node class.
 * @note        It is public because accessed by the inlined constructor.
 */
const struct vfs_overlay_dir_node_vmt __vfs_overlay_dir_node_vmt = {
  .dispose                  = __ovldir_dispose_impl,
  .addref                   = __ro_addref_impl,
  .release                  = __ro_release_impl,
  .stat                     = __ovldir_stat_impl,
  .first                    = __ovldir_first_impl,
  .next                     = __ovldir_next_impl
};

/*===========================================================================*/
/* Module class "vfs_overlay_driver_c" methods.                              */
/*===========================================================================*/

/**
 * @name        Methods implementations of vfs_overlay_driver_c
 * @{
 */
/**
 * @memberof    vfs_overlay_driver_c
 * @protected
 *
 * @brief       Implementation of object creation.
 * @note        This function is meant to be used by derived classes.
 *
 * @param[out]    ip            Pointer to a @p vfs_overlay_driver_c instance
 *                              to be initialized.
 * @param[in]     vmt           VMT pointer for the new object.
 * @param[in]     overlaid_drv  Pointer to a driver to be overlaid or @p NULL.
 * @param[in]     path_prefix   Prefix to be added to the paths or @p NULL, it
 *                              must be a normalized absolute path.
 * @return                      A new reference to the object.
 */
void *__ovldrv_objinit_impl(void *ip, const void *vmt,
                            vfs_driver_c *overlaid_drv,
                            const char *path_prefix) {
  vfs_overlay_driver_c *self = (vfs_overlay_driver_c *)ip;

  /* Initialization of the ancestors-defined parts.*/
  __vfsdrv_objinit_impl(self, vmt);

  /* Initialization code.*/
  self->overlaid_drv = overlaid_drv;
  self->path_prefix  = path_prefix;
  self->path_cwd     = NULL;
  self->next_driver  = 0U;

  return self;
}

/**
 * @memberof    vfs_overlay_driver_c
 * @protected
 *
 * @brief       Implementation of object finalization.
 * @note        This function is meant to be used by derived classes.
 *
 * @param[in,out] ip            Pointer to a @p vfs_overlay_driver_c instance
 *                              to be disposed.
 */
void __ovldrv_dispose_impl(void *ip) {
  vfs_overlay_driver_c *self = (vfs_overlay_driver_c *)ip;

  /* Finalization code.*/
  unsigned i;

  /* Releasing all registered drivers.*/
  i = 0U;
  while (i < self->next_driver) {
    roRelease(self->drivers[i]);
    i++;
  }

  /* Finalization of the ancestors-defined parts.*/
  __vfsdrv_dispose_impl(self);
}

/**
 * @memberof    vfs_overlay_driver_c
 * @protected
 *
 * @brief       Override of method @p vfsDrvChangeCurrentDirectory().
 *
 * @param[in,out] ip            Pointer to a @p vfs_overlay_driver_c instance.
 * @param[in]     path          Path of the new current directory.
 * @return                      The operation result.
 */
msg_t __ovldrv_setcwd_impl(void *ip, const char *path) {
  vfs_overlay_driver_c *self = (vfs_overlay_driver_c *)ip;
  msg_t ret;

  do {
    vfs_directory_node_c *vdnp;
    size_t path_offset;

    ret = build_absolute_path(self, self->buf, path);
    CH_BREAK_ON_ERROR(ret);

    /* Trying to access the directory in order to validate the
       combined path. Note, it can modify the path in the buffer.*/
    ret = open_absolute_dir(self, self->buf, &vdnp);
    CH_BREAK_ON_ERROR(ret);
    roRelease((void *)vdnp);
    path_offset = (size_t)ret;

    /* One-time allocation of the CWD buffer, this memory is allocated, once,
       only if the application uses a CWD, it is never released.*/
    if (self->path_cwd == NULL) {
      self->path_cwd = chCoreAlloc(VFS_CFG_PATHLEN_MAX + 1);
      if (self->path_cwd == NULL) {
        ret = CH_RET_ENOMEM;
        break;
      }
    }

    /* Copying the validated path into the CWD buffer.*/
    strcpy(self->path_cwd, self->buf + path_offset);

  } while (false);

  return ret;
}

/**
 * @memberof    vfs_overlay_driver_c
 * @protected
 *
 * @brief       Override of method @p vfsDrvGetCurrentDirectory().
 *
 * @param[in,out] ip            Pointer to a @p vfs_overlay_driver_c instance.
 * @param[out]    buf           Buffer for the path string.
 * @param[in]     size          Size of the buffer.
 * @return                      The operation result.
 */
msg_t __ovldrv_getcwd_impl(void *ip, char *buf, size_t size) {
  vfs_overlay_driver_c *self = (vfs_overlay_driver_c *)ip;

  *buf = '\0';
  if (vfs_path_append(buf, get_current_directory(self), size) == (size_t)0) {
    return CH_RET_ERANGE;
  }

  return CH_RET_SUCCESS;
}

/**
 * @memberof    vfs_overlay_driver_c
 * @protected
 *
 * @brief       Override of method @p vfsDrvStat().
 *
 * @param[in,out] ip            Pointer to a @p vfs_overlay_driver_c instance.
 * @param[in]     path          Absolute path of the node to be examined.
 * @param[out]    sp            Pointer to a @p vfs_stat_t structure.
 * @return                      The operation result.
 */
msg_t __ovldrv_stat_impl(void *ip, const char *path, vfs_stat_t *sp) {
  vfs_overlay_driver_c *self = (vfs_overlay_driver_c *)ip;
  msg_t ret;

  do {
    const char *scanpath;

    /* Building the absolute path based on current directory.*/
    ret = build_absolute_path(self, self->buf, path);
    CH_BREAK_ON_ERROR(ret);

    /* Skipping the root separator.*/
    scanpath = self->buf + 1;

    /* If it is not root checking among mounted drivers.*/
    if (*scanpath != '\0') {
      vfs_driver_c *dp;

      /* Searching for a match among registered overlays.*/
      ret = match_driver(self, &scanpath, &dp);
      if (!CH_RET_IS_ERROR(ret)) {
        /* Delegating information request to a registered driver.*/
        ret = vfsDrvStat((void *)dp, scanpath, sp);
        break;
      }
    }

    /* Is there an overlaid driver? if so we need to pass request
       processing there.*/
    if (self->overlaid_drv != NULL) {

      /* Processing the prefix, if defined.*/
      if (self->path_prefix != NULL) {
        if (vfs_path_prepend(self->buf,
                             self->path_prefix,
                             VFS_CFG_PATHLEN_MAX + 1) == (size_t)0) {
          ret = CH_RET_ENAMETOOLONG;
          break;
        }
      }

      /* Passing the combined path to the overlaid driver.*/
      ret = vfsDrvStat((void *)self->overlaid_drv, self->buf, sp);
    }
    else {
      /* This is the root directory.*/
      sp->size = 0;
      sp->mode = VFS_MODE_S_IFDIR;
      ret = CH_RET_SUCCESS;
    }
  } while (false);

  return ret;
}

/**
 * @memberof    vfs_overlay_driver_c
 * @protected
 *
 * @brief       Override of method @p vfsDrvOpenDirectory().
 *
 * @param[in,out] ip            Pointer to a @p vfs_overlay_driver_c instance.
 * @param[in]     path          Absolute path of the directory to be opened.
 * @param[out]    vdnpp         Pointer to the pointer to the instantiated @p
 *                              vfs_directory_node_c object.
 * @return                      The operation result.
 */
msg_t __ovldrv_opendir_impl(void *ip, const char *path,
                            vfs_directory_node_c **vdnpp) {
  vfs_overlay_driver_c *self = (vfs_overlay_driver_c *)ip;
  msg_t ret;

  do {
    /* Building the absolute path based on current directory.*/
    ret = build_absolute_path(self, self->buf, path);
    CH_BREAK_ON_ERROR(ret);

    ret = open_absolute_dir(self, self->buf, vdnpp);
    CH_BREAK_ON_ERROR(ret);

    /* Required because the offset returned by open_absolute_dir().*/
    ret = CH_RET_SUCCESS;
  } while (false);

  return ret;
}

/**
 * @memberof    vfs_overlay_driver_c
 * @protected
 *
 * @brief       Override of method @p vfsDrvOpenFile().
 *
 * @param[in,out] ip            Pointer to a @p vfs_overlay_driver_c instance.
 * @param[in]     path          Absolute path of the directory to be opened.
 * @param[in]     flags         File open flags.
 * @param[out]    vfnpp         Pointer to the pointer to the instantiated @p
 *                              vfs_file_node_c object.
 * @return                      The operation result.
 */
msg_t __ovldrv_openfile_impl(void *ip, const char *path, int flags,
                             vfs_file_node_c **vfnpp) {
  vfs_overlay_driver_c *self = (vfs_overlay_driver_c *)ip;
  msg_t ret;

  do {
    /* Building the absolute path based on current directory.*/
    ret = build_absolute_path(self, self->buf, path);
    CH_BREAK_ON_ERROR(ret);

    ret = open_absolute_file(self, self->buf, flags, vfnpp);
  } while (false);

  return ret;
}

/**
 * @memberof    vfs_overlay_driver_c
 * @protected
 *
 * @brief       Override of method @p vfsDrvUnlink().
 *
 * @param[in,out] ip            Pointer to a @p vfs_overlay_driver_c instance.
 * @param[in]     path          Path of the file to be unlinked.
 * @return                      The operation result.
 */
msg_t __ovldrv_unlink_impl(void *ip, const char *path) {
  vfs_overlay_driver_c *self = (vfs_overlay_driver_c *)ip;
  msg_t ret;

  do {
    const char *scanpath;

    /* Building the absolute path based on current directory.*/
    ret = build_absolute_path(self, self->buf, path);
    CH_BREAK_ON_ERROR(ret);

    /* Skipping the root separator.*/
    scanpath = self->buf + 1;

    /* If it is the root.*/
    if (*scanpath == '\0') {
      ret = CH_RET_EISDIR;
    }
    else { /* Not the root.*/
      vfs_driver_c *dp;

      /* Searching for a match among registered overlays.*/
      ret = match_driver(self, &scanpath, &dp);
      if (!CH_RET_IS_ERROR(ret)) {
        /* Delegating file deletion to a registered driver.*/
        ret = vfsDrvUnlink((void *)dp, scanpath);
      }
      else {
        /* Passing the request to the overlaid driver, if any.*/
        /* TODO remove the dirty trick.*/
        ret = drv_overlaid_path_call(self, self->buf,
                                     self->overlaid_drv->vmt->unlink);
      }
    }
  } while (false);

  return ret;
}

/**
 * @memberof    vfs_overlay_driver_c
 * @protected
 *
 * @brief       Override of method @p vfsDrvRename().
 *
 * @param[in,out] ip            Pointer to a @p vfs_overlay_driver_c instance.
 * @param[in]     oldpath       Path of the node to be renamed.
 * @param[in]     newpath       New path of the renamed node.
 * @return                      The operation result.
 */
msg_t __ovldrv_rename_impl(void *ip, const char *oldpath, const char *newpath) {
  vfs_overlay_driver_c *self = (vfs_overlay_driver_c *)ip;
  msg_t ret;
  vfs_shared_buffer_t *shbuf;

  /* Taking an extra path buffer from the pool.*/
  shbuf = vfs_buffer_take_immediate();
  if (shbuf == NULL) {
    return CH_RET_ENOMEM;
  }

  do {
    msg_t oldret, newret;
    vfs_driver_c *olddp, *newdp;
    const char *op, *np;

    /* Building the absolute paths based on current directory.*/
    ret = build_absolute_path(self, self->buf, oldpath);
    CH_BREAK_ON_ERROR(ret);
    ret = build_absolute_path(self, shbuf->buf, newpath);
    CH_BREAK_ON_ERROR(ret);

    /* Skipping root separators.*/
    op = self->buf + 1;
    np = shbuf->buf + 1;

    /* Searching for a match among registered drivers.*/
    oldret = match_driver(self, &op, &olddp);
    newret = match_driver(self, &np, &newdp);

    /* There are various combinations to consider.*/
    if (!CH_RET_IS_ERROR(oldret) && !CH_RET_IS_ERROR(newret)) {
       /* If paths both refer to registered drivers then must refer to
          the same driver, we cannot do a rename across drivers.*/
      if (olddp == newdp) {
        /* Delegating node renaming to the registered driver.*/
        ret = vfsDrvRename((void *)olddp, op, np);
      }
      else {
        /* Mixed, not allowing it.*/
        ret = CH_RET_EXDEV;
      }
    }
    else if (CH_RET_IS_ERROR(oldret) && CH_RET_IS_ERROR(newret)) {
      /* If both paths refer to the overlaid driver then passing down the
         request.*/
      if (self->overlaid_drv != NULL) {

        /* Processing the prefix, if defined.*/
        if (self->path_prefix != NULL) {
          if (vfs_path_prepend(self->buf,
                               self->path_prefix,
                               VFS_CFG_PATHLEN_MAX + 1) == (size_t)0) {
            ret = CH_RET_ENAMETOOLONG;
            break;
          }
          if (vfs_path_prepend(shbuf->buf,
                               self->path_prefix,
                               VFS_CFG_PATHLEN_MAX + 1) == (size_t)0) {
            ret = CH_RET_ENAMETOOLONG;
            break;
          }
        }

        /* Passing the combined path to the overlaid driver.*/
        ret = vfsDrvRename((void *)self->overlaid_drv,
                           self->buf,
                           shbuf->buf);
      }
    }
    else {
      /* Mixed, not allowing it.*/
      ret = CH_RET_EXDEV;
    }
  } while (false);

  /* Buffers returned, note, in reverse order.*/
  vfs_buffer_release(shbuf);

  return ret;
}

/**
 * @memberof    vfs_overlay_driver_c
 * @protected
 *
 * @brief       Override of method @p vfsDrvMkdir().
 *
 * @param[in,out] ip            Pointer to a @p vfs_overlay_driver_c instance.
 * @param[in]     path          Path of the directory to be created.
 * @param[in]     mode          Mode flags for the directory.
 * @return                      The operation result.
 */
msg_t __ovldrv_mkdir_impl(void *ip, const char *path, vfs_mode_t mode) {
  vfs_overlay_driver_c *self = (vfs_overlay_driver_c *)ip;
  msg_t ret;

  do {
    const char *scanpath;

    /* Building the absolute path based on current directory.*/
    ret = build_absolute_path(self, self->buf, path);
    CH_BREAK_ON_ERROR(ret);

    /* Skipping the root separator.*/
    scanpath = self->buf + 1;

    /* If it is the root.*/
    if (*scanpath == '\0') {
      ret = CH_RET_EEXIST;
    }
    else { /* Not the root.*/
      vfs_driver_c *dp;

      /* Searching for a match among registered overlays.*/
      ret = match_driver(self, &scanpath, &dp);
      if (!CH_RET_IS_ERROR(ret)) {
        /* Delegating directory creation to a registered driver.*/
        ret = vfsDrvMkdir((void *)dp, scanpath, mode);
      }
      else {
        /* Is there an overlaid driver? if so we need to pass request
           processing there.*/
        if (self->overlaid_drv != NULL) {

          /* Processing the prefix, if defined.*/
          if (self->path_prefix != NULL) {
            if (vfs_path_prepend(self->buf,
                                 self->path_prefix,
                                 VFS_CFG_PATHLEN_MAX + 1) == (size_t)0) {
              ret = CH_RET_ENAMETOOLONG;
              break;
            }
          }

          /* Passing the combined path to the overlaid driver.*/
          ret = vfsDrvMkdir((void *)self->overlaid_drv, self->buf, mode);
        }
        else {
          ret = CH_RET_ENOENT;
        }
      }
    }
  } while (false);

  return ret;
}

/**
 * @memberof    vfs_overlay_driver_c
 * @protected
 *
 * @brief       Override of method @p vfsDrvRmdir().
 *
 * @param[in,out] ip            Pointer to a @p vfs_overlay_driver_c instance.
 * @param[in]     path          Path of the directory to be removed.
 * @return                      The operation result.
 */
msg_t __ovldrv_rmdir_impl(void *ip, const char *path) {
  vfs_overlay_driver_c *self = (vfs_overlay_driver_c *)ip;
  msg_t ret;

  do {
    const char *scanpath;

    /* Building the absolute path based on current directory.*/
    ret = build_absolute_path(self, self->buf, path);
    CH_BREAK_ON_ERROR(ret);

    /* Skipping the root separator.*/
    scanpath = self->buf + 1;

    /* If it is the root.*/
    if (*scanpath == '\0') {
      ret = CH_RET_EACCES;
    }
    else { /* Not the root.*/
      vfs_driver_c *dp;

      /* Searching for a match among registered overlays.*/
      ret = match_driver(self, &scanpath, &dp);
      if (!CH_RET_IS_ERROR(ret)) {
        /* Delegating directory deletion to a registered driver.*/
        ret = vfsDrvRmdir((void *)dp, scanpath);
      }
      else {
        /* Passing the request to the overlaid driver, if any.*/
        /* TODO remove the dirty trick.*/
        ret = drv_overlaid_path_call(self, self->buf,
                                     self->overlaid_drv->vmt->rmdir);
      }
    }
  } while (false);

  return ret;
}
/** @} */

/**
 * @brief       VMT structure of VFS overlay driver class.
 * @note        It is public because accessed by the inlined constructor.
 */
const struct vfs_overlay_driver_vmt __vfs_overlay_driver_vmt = {
  .dispose                  = __ovldrv_dispose_impl,
  .setcwd                   = __ovldrv_setcwd_impl,
  .getcwd                   = __ovldrv_getcwd_impl,
  .stat                     = __ovldrv_stat_impl,
  .opendir                  = __ovldrv_opendir_impl,
  .openfile                 = __ovldrv_openfile_impl,
  .unlink                   = __ovldrv_unlink_impl,
  .rename                   = __ovldrv_rename_impl,
  .mkdir                    = __ovldrv_mkdir_impl,
  .rmdir                    = __ovldrv_rmdir_impl
};

/**
 * @name        Regular methods of vfs_overlay_driver_c
 * @{
 */
/**
 * @memberof    vfs_overlay_driver_c
 * @public
 *
 * @brief       Registers a VFS driver as an overlay.
 * @note        The overlay becomes the owner of the registered driver, the
 *              reference is released when dre driver is unregistered or when
 *              the whole overlay object is disposed.
 *
 * @param[in,out] ip            Pointer to a @p vfs_overlay_driver_c instance.
 * @param[in]     vdp           Driver object reference to be registered.
 * @param[in]     name          Name for the new overlay directory, must be
 *                              alphanumeric.
 * @return                      The operation result.
 *
 * @api
 */
msg_t ovldrvRegisterDriver(void *ip, vfs_driver_c *vdp, const char *name) {
  vfs_overlay_driver_c *self = (vfs_overlay_driver_c *)ip;
  msg_t ret;

  chSysLock();

  if (self->next_driver >= DRV_CFG_OVERLAY_DRV_MAX) {
    ret = CH_RET_ENOMEM;
  }
  else {
    self->names[self->next_driver]   = name;
    self->drivers[self->next_driver] = vdp;
    self->next_driver++;
    ret = CH_RET_SUCCESS;
  }

  chSysUnlock();

  return ret;
}

/**
 * @memberof    vfs_overlay_driver_c
 * @public
 *
 * @brief       Unregisters a VFS driver.
 * @note        The reference to the registered driver is released.
 *
 * @param[in,out] ip            Pointer to a @p vfs_overlay_driver_c instance.
 * @param[in]     name          Name of the overlay directory to be
 *                              unregistered.
 * @return                      The operation result.
 *
 * @api
 */
msg_t ovldrvUnregisterDriver(void *ip, const char *name) {
  vfs_overlay_driver_c *self = (vfs_overlay_driver_c *)ip;
  unsigned i;

  chSysLock();

  i = 0U;
  while (i < self->next_driver) {

    /* Is name matching?*/
    if (strcmp(self->names[i], name) == 0) {
      vfs_driver_c *vdp = self->drivers[i];

      /* Found match, move following entries down, if any.*/
      while (i < self->next_driver - 1) {
        self->drivers[i] = self->drivers[i + 1];
        self->names[i] = self->names[i + 1];
        i++;
      }
      self->next_driver--;

      chSysUnlock();

      /* Releasing the unregistered object.*/
      roRelease(vdp);

      return CH_RET_SUCCESS;
    }

    i++;
  }

  chSysUnlock();

  return CH_RET_ENOENT;
}
/** @} */

