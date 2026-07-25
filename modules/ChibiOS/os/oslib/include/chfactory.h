/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    oslib/include/chfactory.h
 * @brief   ChibiOS objects factory structures and macros.
 *
 * @addtogroup oslib_objects_factory
 * @{
 */

#ifndef CHFACTORY_H
#define CHFACTORY_H

#if (CH_CFG_USE_FACTORY == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Maximum length for object names.
 * @details If the specified length is zero then the name is stored by
 *          pointer but this could have unintended side effects.
 */
#if !defined(CH_CFG_FACTORY_MAX_NAMES_LENGTH) || defined(__DOXYGEN__)
#define CH_CFG_FACTORY_MAX_NAMES_LENGTH     8
#endif

/**
 * @brief   Enables the registry of generic objects.
 */
#if !defined(CH_CFG_FACTORY_OBJECTS_REGISTRY) || defined(__DOXYGEN__)
#define CH_CFG_FACTORY_OBJECTS_REGISTRY     TRUE
#endif

/**
 * @brief   Enables factory for generic buffers.
 */
#if !defined(CH_CFG_FACTORY_GENERIC_BUFFERS) || defined(__DOXYGEN__)
#define CH_CFG_FACTORY_GENERIC_BUFFERS      TRUE
#endif

/**
 * @brief   Enables factory for semaphores.
 */
#if !defined(CH_CFG_FACTORY_SEMAPHORES) || defined(__DOXYGEN__)
#define CH_CFG_FACTORY_SEMAPHORES           TRUE
#endif

/**
 * @brief   Enables factory for mailboxes.
 */
#if !defined(CH_CFG_FACTORY_MAILBOXES) || defined(__DOXYGEN__)
#define CH_CFG_FACTORY_MAILBOXES            TRUE
#endif

/**
 * @brief   Enables factory for objects FIFOs.
 */
#if !defined(CH_CFG_FACTORY_OBJ_FIFOS) || defined(__DOXYGEN__)
#define CH_CFG_FACTORY_OBJ_FIFOS            TRUE
#endif

/**
 * @brief   Enables factory for Pipes.
 */
#if !defined(CH_CFG_FACTORY_PIPES) || defined(__DOXYGEN__)
#define CH_CFG_FACTORY_PIPES                TRUE
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if (CH_CFG_FACTORY_SEMAPHORES == TRUE) && (CH_CFG_USE_SEMAPHORES == FALSE)
/*lint -save -e767 [20.5] Valid because the #undef.*/
#undef CH_CFG_FACTORY_SEMAPHORES
#define CH_CFG_FACTORY_SEMAPHORES           FALSE
/*lint -restore*/
#endif

#if (CH_CFG_FACTORY_MAILBOXES == TRUE) && (CH_CFG_USE_MAILBOXES == FALSE)
/*lint -save -e767 [20.5] Valid because the #undef.*/
#undef CH_CFG_FACTORY_MAILBOXES
#define CH_CFG_FACTORY_MAILBOXES            FALSE
/*lint -restore*/
#endif

#if (CH_CFG_FACTORY_OBJ_FIFOS == TRUE) && (CH_CFG_USE_OBJ_FIFOS == FALSE)
/*lint -save -e767 [20.5] Valid because the #undef.*/
#undef CH_CFG_FACTORY_OBJ_FIFOS
#define CH_CFG_FACTORY_OBJ_FIFOS            FALSE
/*lint -restore*/
#endif

#if (CH_CFG_FACTORY_PIPES == TRUE) && (CH_CFG_USE_PIPES == FALSE)
/*lint -save -e767 [20.5] Valid because the #undef.*/
#undef CH_CFG_FACTORY_PIPES
#define CH_CFG_FACTORY_PIPES                FALSE
/*lint -restore*/
#endif

#define CH_FACTORY_REQUIRES_POOLS                                           \
  ((CH_CFG_FACTORY_OBJECTS_REGISTRY == TRUE) ||                             \
   (CH_CFG_FACTORY_SEMAPHORES == TRUE))

#define CH_FACTORY_REQUIRES_HEAP                                            \
  ((CH_CFG_FACTORY_GENERIC_BUFFERS == TRUE) ||                              \
   (CH_CFG_FACTORY_MAILBOXES == TRUE) ||                                    \
   (CH_CFG_FACTORY_OBJ_FIFOS == TRUE) ||                                    \
   (CH_CFG_FACTORY_PIPES == TRUE))

#if (CH_CFG_FACTORY_MAX_NAMES_LENGTH < 0) ||                                \
    (CH_CFG_FACTORY_MAX_NAMES_LENGTH > 32)
#error "invalid CH_CFG_FACTORY_MAX_NAMES_LENGTH value"
#endif

#if (CH_CFG_USE_MUTEXES == FALSE) && (CH_CFG_USE_SEMAPHORES == FALSE)
#error "CH_CFG_USE_FACTORY requires CH_CFG_USE_MUTEXES and/or CH_CFG_USE_SEMAPHORES"
#endif

#if CH_CFG_USE_MEMCORE == FALSE
#error "CH_CFG_USE_FACTORY requires CH_CFG_USE_MEMCORE"
#endif

#if CH_FACTORY_REQUIRES_POOLS && (CH_CFG_USE_MEMPOOLS == FALSE)
#error "CH_CFG_USE_MEMPOOLS is required"
#endif

#if CH_FACTORY_REQUIRES_HEAP && (CH_CFG_USE_HEAP == FALSE)
#error "CH_CFG_USE_HEAP is required"
#endif

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a dynamic object list element.
 */
typedef struct ch_dyn_element {
  /**
   * @brief   Next dynamic object in the list.
   */
  struct ch_dyn_element *next;
  /**
   * @brief   Number of references to this object.
   */
  ucnt_t                refs;
#if (CH_CFG_FACTORY_MAX_NAMES_LENGTH > 0) || defined(__DOXYGEN__)
  char                  name[CH_CFG_FACTORY_MAX_NAMES_LENGTH];
#else
  const char            *name;
#endif
} dyn_element_t;

/**
 * @brief   Type of a dynamic object list.
 */
typedef struct ch_dyn_list {
    dyn_element_t       *next;
} dyn_list_t;

#if (CH_CFG_FACTORY_OBJECTS_REGISTRY == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Type of a registered object.
 */
typedef struct ch_registered_static_object {
  /**
   * @brief   List element of the registered object.
   */
  dyn_element_t         element;
  /**
   * @brief   Pointer to the object.
   * @note    The type of the object is not stored in anyway.
   */
  void                  *objp;
} registered_object_t;
#endif

#if (CH_CFG_FACTORY_GENERIC_BUFFERS == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Type of a dynamic buffer object.
 */
typedef struct ch_dyn_object {
  /**
   * @brief   List element of the dynamic buffer object.
   */
  dyn_element_t         element;
} dyn_buffer_t;
#endif

#if (CH_CFG_FACTORY_SEMAPHORES == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Type of a dynamic semaphore.
 */
typedef struct ch_dyn_semaphore {
  /**
   * @brief   List element of the dynamic semaphore.
   */
  dyn_element_t         element;
  /**
   * @brief   The semaphore.
   */
  semaphore_t           sem;
} dyn_semaphore_t;
#endif

#if (CH_CFG_FACTORY_MAILBOXES == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Type of a dynamic buffer object.
 */
typedef struct ch_dyn_mailbox {
  /**
   * @brief   List element of the dynamic buffer object.
   */
  dyn_element_t         element;
  /**
   * @brief   The mailbox.
   */
  mailbox_t             mbx;
} dyn_mailbox_t;
#endif

#if (CH_CFG_FACTORY_OBJ_FIFOS == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Type of a dynamic buffer object.
 */
typedef struct ch_dyn_objects_fifo {
  /**
   * @brief   List element of the dynamic buffer object.
   */
  dyn_element_t         element;
  /**
   * @brief   The objects FIFO.
   */
  objects_fifo_t        fifo;
} dyn_objects_fifo_t;
#endif

#if (CH_CFG_FACTORY_PIPES == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Type of a dynamic pipe object.
 */
typedef struct ch_dyn_pipe {
  /**
   * @brief   List element of the dynamic pipe object.
   */
  dyn_element_t         element;
  /**
   * @brief   The pipe.
   */
  pipe_t                pipe;
} dyn_pipe_t;
#endif

/**
 * @brief   Type of the factory main object.
 */
typedef struct ch_objects_factory {
  /**
   * @brief   Factory access mutex or semaphore.
   */
#if (CH_CFG_USE_MUTEXES == TRUE) || defined(__DOXYGEN__)
  mutex_t               mtx;
#else
  semaphore_t           sem;
#endif
#if (CH_CFG_FACTORY_OBJECTS_REGISTRY == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   List of the registered objects.
   */
  dyn_list_t            obj_list;
  /**
   * @brief   Pool of the available registered objects.
   */
  memory_pool_t         obj_pool;
#endif /* CH_CFG_FACTORY_OBJECTS_REGISTRY = TRUE */
#if (CH_CFG_FACTORY_GENERIC_BUFFERS == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   List of the allocated buffer objects.
   */
  dyn_list_t            buf_list;
#endif /* CH_CFG_FACTORY_GENERIC_BUFFERS = TRUE */
#if (CH_CFG_FACTORY_SEMAPHORES == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   List of the allocated semaphores.
   */
  dyn_list_t            sem_list;
  /**
   * @brief   Pool of the available semaphores.
   */
  memory_pool_t         sem_pool;
#endif /* CH_CFG_FACTORY_SEMAPHORES = TRUE */
#if (CH_CFG_FACTORY_MAILBOXES == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   List of the allocated buffer objects.
   */
  dyn_list_t            mbx_list;
#endif /* CH_CFG_FACTORY_MAILBOXES = TRUE */
#if (CH_CFG_FACTORY_OBJ_FIFOS == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   List of the allocated "objects FIFO" objects.
   */
  dyn_list_t            fifo_list;
#endif /* CH_CFG_FACTORY_OBJ_FIFOS = TRUE */
#if (CH_CFG_FACTORY_PIPES == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   List of the allocated pipe objects.
   */
  dyn_list_t            pipe_list;
#endif /* CH_CFG_FACTORY_PIPES = TRUE */
} objects_factory_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
extern objects_factory_t ch_factory;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void __factory_init(void);
  dyn_element_t *chFactoryDuplicateReference(dyn_element_t *dep);
#if (CH_CFG_FACTORY_OBJECTS_REGISTRY == TRUE) || defined(__DOXYGEN__)
  registered_object_t *chFactoryRegisterObject(const char *name,
                                               void *objp);
  registered_object_t *chFactoryFindObject(const char *name);
  registered_object_t *chFactoryFindObjectByPointer(void *objp);
  void chFactoryReleaseObject(registered_object_t *rop);
#endif
#if (CH_CFG_FACTORY_GENERIC_BUFFERS == TRUE) || defined(__DOXYGEN__)
  dyn_buffer_t *chFactoryCreateBuffer(const char *name, size_t size);
  dyn_buffer_t *chFactoryFindBuffer(const char *name);
  void chFactoryReleaseBuffer(dyn_buffer_t *dbp);
#endif
#if (CH_CFG_FACTORY_SEMAPHORES == TRUE) || defined(__DOXYGEN__)
  dyn_semaphore_t *chFactoryCreateSemaphore(const char *name, cnt_t n);
  dyn_semaphore_t *chFactoryFindSemaphore(const char *name);
  void chFactoryReleaseSemaphore(dyn_semaphore_t *dsp);
#endif
#if (CH_CFG_FACTORY_MAILBOXES == TRUE) || defined(__DOXYGEN__)
  dyn_mailbox_t *chFactoryCreateMailbox(const char *name, size_t n);
  dyn_mailbox_t *chFactoryFindMailbox(const char *name);
  void chFactoryReleaseMailbox(dyn_mailbox_t *dmp);
#endif
#if (CH_CFG_FACTORY_OBJ_FIFOS == TRUE) || defined(__DOXYGEN__)
  dyn_objects_fifo_t *chFactoryCreateObjectsFIFO(const char *name,
                                                 size_t objsize,
                                                 size_t objn,
                                                 unsigned objalign);
  dyn_objects_fifo_t *chFactoryFindObjectsFIFO(const char *name);
  void chFactoryReleaseObjectsFIFO(dyn_objects_fifo_t *dofp);
#endif
#if (CH_CFG_FACTORY_PIPES == TRUE) || defined(__DOXYGEN__)
  dyn_pipe_t *chFactoryCreatePipe(const char *name, size_t size);
  dyn_pipe_t *chFactoryFindPipe(const char *name);
  void chFactoryReleasePipe(dyn_pipe_t *dpp);
#endif
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#if (CH_CFG_FACTORY_OBJECTS_REGISTRY == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Returns the pointer to the inner registered object.
 *
 * @param[in] rop       registered object reference
 * @return              The pointer to the registered object.
 *
 * @api
 */
static inline void *chFactoryGetObject(registered_object_t *rop) {

  return rop->objp;
}
#endif /* CH_CFG_FACTORY_OBJECTS_REGISTRY == TRUE */

#if (CH_CFG_FACTORY_GENERIC_BUFFERS == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Returns the size of a generic dynamic buffer object.
 *
 * @param[in] dbp       dynamic buffer object reference
 * @return              The size of the buffer object in bytes.
 *
 * @api
 */
static inline size_t chFactoryGetBufferSize(dyn_buffer_t *dbp) {

  return chHeapGetSize(dbp) - sizeof (dyn_element_t);
}

/**
 * @brief   Returns the pointer to the inner buffer.
 *
 * @param[in] dbp       dynamic buffer object reference
 * @return              The pointer to the dynamic buffer.
 *
 * @api
 */
static inline uint8_t *chFactoryGetBuffer(dyn_buffer_t *dbp) {

  return (uint8_t *)(dbp + 1);
}
#endif /* CH_CFG_FACTORY_GENERIC_BUFFERS == TRUE */

#if (CH_CFG_FACTORY_SEMAPHORES == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Returns the pointer to the inner semaphore.
 *
 * @param[in] dsp       dynamic semaphore object reference
 * @return              The pointer to the semaphore.
 *
 * @api
 */
static inline semaphore_t *chFactoryGetSemaphore(dyn_semaphore_t *dsp) {

  return &dsp->sem;
}
#endif /* CH_CFG_FACTORY_SEMAPHORES == TRUE */

#if (CH_CFG_FACTORY_MAILBOXES == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Returns the pointer to the inner mailbox.
 *
 * @param[in] dmp       dynamic mailbox object reference
 * @return              The pointer to the mailbox.
 *
 * @api
 */
static inline mailbox_t *chFactoryGetMailbox(dyn_mailbox_t *dmp) {

  return &dmp->mbx;
}
#endif /* CH_CFG_FACTORY_MAILBOXES == TRUE */

#if (CH_CFG_FACTORY_OBJ_FIFOS == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Returns the pointer to the inner objects FIFO.
 *
 * @param[in] dofp      dynamic "objects FIFO" object reference
 * @return              The pointer to the objects FIFO.
 *
 * @api
 */
static inline objects_fifo_t *chFactoryGetObjectsFIFO(dyn_objects_fifo_t *dofp) {

  return &dofp->fifo;
}
#endif /* CH_CFG_FACTORY_OBJ_FIFOS == TRUE */

#if (CH_CFG_FACTORY_PIPES == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Returns the pointer to the inner pipe.
 *
 * @param[in] dpp       dynamic pipe object reference
 * @return              The pointer to the pipe.
 *
 * @api
 */
static inline pipe_t *chFactoryGetPipe(dyn_pipe_t *dpp) {

  return &dpp->pipe;
}
#endif /* CH_CFG_FACTORY_PIPES == TRUE */

#endif /* CH_CFG_USE_FACTORY == TRUE */

#endif /* CHFACTORY_H */

/** @} */
