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
 * @file    rt/include/chregistry.h
 * @brief   Threads registry macros and structures.
 *
 * @addtogroup registry
 * @{
 */

#ifndef CHREGISTRY_H
#define CHREGISTRY_H

#if (CH_CFG_USE_REGISTRY == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   ChibiOS/RT memory signature record.
 */
typedef struct {
  char      identifier[4];          /**< @brief Always set to "main".       */
  uint8_t   zero;                   /**< @brief Must be zero.               */
  uint8_t   size;                   /**< @brief Size of this structure.     */
  uint16_t  version;                /**< @brief Encoded ChibiOS/RT version. */
  uint8_t   ptrsize;                /**< @brief Size of a pointer.          */
  uint8_t   timesize;               /**< @brief Size of a @p systime_t.     */
  uint8_t   threadsize;             /**< @brief Size of a @p thread_t.      */
  uint8_t   off_prio;               /**< @brief Offset of @p prio field.    */
  uint8_t   off_ctx;                /**< @brief Offset of @p ctx field.     */
  uint8_t   off_newer;              /**< @brief Offset of @p newer field.   */
  uint8_t   off_older;              /**< @brief Offset of @p older field.   */
  uint8_t   off_name;               /**< @brief Offset of @p name field.    */
  uint8_t   off_stklimit;           /**< @brief Offset of @p stklimit field.*/
  uint8_t   off_state;              /**< @brief Offset of @p state field.   */
  uint8_t   off_flags;              /**< @brief Offset of @p flags field.   */
  uint8_t   off_refs;               /**< @brief Offset of @p refs field.    */
  uint8_t   off_preempt;            /**< @brief Offset of @p ticks field.   */
  uint8_t   off_time;               /**< @brief Offset of @p time field.    */
  uint8_t   off_reserved[4];
  uint8_t   intctxsize;             /**< @brief Size of a @p port_intctx.   */
  uint8_t   intervalsize;           /**< @brief Size of a @p sysinterval_t. */
  uint8_t   instancesnum;           /**< @brief Number of instances.        */
  uint8_t   off_sys_state;          /**< @brief Offset of @p state field.   */
  uint8_t   off_sys_instances;      /**< @brief Offset of @p instances array
                                                field.                      */
  uint8_t   off_sys_reglist;        /**< @brief Offset of @p reglist field. */
  uint8_t   off_sys_rfcu;           /**< @brief Offset of @p rfcu field.    */
  uint8_t   off_sys_reserved[4];
  uint8_t   off_inst_rlist_current; /**< @brief Offset of @p rlist.current
                                                field.                      */
  uint8_t   off_inst_rlist;         /**< @brief Offset of @p rlist field.   */
  uint8_t   off_inst_vtlist;        /**< @brief Offset of @p vtlist field.  */
  uint8_t   off_inst_reglist;       /**< @brief Offset of @p reglist field. */
  uint8_t   off_inst_core_id;       /**< @brief Offset of @p core_id field. */
  uint8_t   off_inst_rfcu;          /**< @brief Offset of @p rfcu field.    */
} chdebug_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Access to the registry list header.
 */
#if (CH_CFG_SMP_MODE == TRUE) || defined(__DOXYGEN__)
#define REG_HEADER(oip) (&ch_system.reglist.queue)
#else
#define REG_HEADER(oip) (&(oip)->reglist.queue)
#endif

/**
 * @brief   Removes a thread from the registry list.
 * @note    This macro is not meant for use in application code.
 *
 * @param[in] tp        thread to remove from the registry
 */
#define REG_REMOVE(tp) (void) ch_queue_dequeue(&(tp)->rqueue)

/**
 * @brief   Adds a thread to the registry list.
 * @note    This macro is not meant for use in application code.
 *
 * @param[in] oip       pointer to the OS instance
 * @param[in] tp        thread to add to the registry
 */
#define REG_INSERT(oip, tp) ch_queue_insert(REG_HEADER(oip), &(tp)->rqueue)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  extern ROMCONST chdebug_t ch_debug;
  thread_t *chRegFirstThread(void);
  thread_t *chRegNextThread(thread_t *tp);
  thread_t *chRegFindThreadByName(const char *name);
  thread_t *chRegFindThreadByPointer(thread_t *tp);
  thread_t *chRegFindThreadByWorkingArea(stkalign_t *wa);
#ifdef __cplusplus
}
#endif

#endif /* CH_CFG_USE_REGISTRY == TRUE */

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Initializes a registry.
 * @note    Internal use only.
 *
 * @param[out] rp       pointer to a @p registry_t structure
 *
 * @init
 */
static inline void __reg_object_init(registry_t *rp) {

  ch_queue_init(&rp->queue);
}

/**
 * @brief   Sets the current thread name.
 * @pre     This function only stores the pointer to the name if the option
 *          @p CH_CFG_USE_REGISTRY is enabled else no action is performed.
 *
 * @param[in] name      thread name as a zero terminated string
 *
 * @api
 */
static inline void chRegSetThreadName(const char *name) {

#if CH_CFG_USE_REGISTRY == TRUE
  __sch_get_currthread()->name = name;
#else
  (void)name;
#endif
}

/**
 * @brief   Returns the name of the specified thread.
 * @pre     This function only returns the pointer to the name if the option
 *          @p CH_CFG_USE_REGISTRY is enabled else @p NULL is returned.
 *
 * @param[in] tp        pointer to the thread
 *
 * @return              Thread name as a zero terminated string.
 * @retval NULL         if the thread name has not been set.
 *
 */
static inline const char *chRegGetThreadNameX(thread_t *tp) {

#if CH_CFG_USE_REGISTRY == TRUE
  return tp->name;
#else
  (void)tp;
  return NULL;
#endif
}

/**
 * @brief   Changes the name of the specified thread.
 * @pre     This function only stores the pointer to the name if the option
 *          @p CH_CFG_USE_REGISTRY is enabled else no action is performed.
 *
 * @param[in] tp        pointer to the thread
 * @param[in] name      thread name as a zero terminated string
 *
 * @xclass
 */
static inline void chRegSetThreadNameX(thread_t *tp, const char *name) {

#if CH_CFG_USE_REGISTRY == TRUE
  tp->name = name;
#else
  (void)tp;
  (void)name;
#endif
}

#endif /* CHREGISTRY_H */

/** @} */
