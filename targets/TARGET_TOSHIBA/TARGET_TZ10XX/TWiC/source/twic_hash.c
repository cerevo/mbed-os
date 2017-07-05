/**
 * @file twic_hash.c
 * @brief a source file for TZ10xx TWiC for Bluetooth 4.0/4.1 Smart
 * @version V1.2.0
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */


#include "tz1ut_list.h"
#include "tz1sm_hal.h"
#include "twic_service.h"
#include "twic_hash.h"

TWIC_HASH_DEF(srvif);

twicEntry_t *twicHashFind1stHandle(const uint16_t handle);
twicEntry_t *twicHashFind2ndHandle(const uint16_t handle);

void twicHashCreate(twicEntry_t *(* const entry_hash)[],
                    const uint16_t entry_hash_size)
{
  memset((void *)(*entry_hash), 0, sizeof(twicEntry_t *) * entry_hash_size);
  TWIC_HASH(srvif) = entry_hash;
  TWIC_HASH_SIZE(srvif) = entry_hash_size;
  return;
}

static TZ1K_INLINE uint16_t twicHashValue(const uint16_t value)
{
  return (value % TWIC_HASH_SIZE(srvif));
}

static TZ1K_INLINE void
twicHashInsertHandle(const uint16_t idx, twicEntry_t * const entity)
{
  twicEntry_t *(*_hash)[] = TWIC_HASH(srvif);
  twicEntry_t *p;
  
  p = (*_hash)[idx];
  if (NULL == p) {
    (*_hash)[idx] = entity; entity->next = NULL; entity->prev = NULL;
  } else {
    while (p->next) p = p->next;
    p->next = entity; entity->prev = p; entity->next = NULL;
  }

  return;
}

uint8_t twicHashIsEmpty(void)
{
  twicEntry_t *(*_hash)[] = TWIC_HASH(srvif);
  twicEntry_t *p;
  uint16_t idx;
  
  for (idx = 0; idx < TWIC_HASH_SIZE(srvif); idx++)
    for (p = (*_hash)[idx]; (p);) return 0;

  return 1;
}

void twicHashInsert1stHandle(twicEntry_t * const entity, const uint16_t handle)
{
  twicEntry_t *conn = entity->instance->signal;
  
  //if (twicHashFind1stHandle(handle))
  //twicLog("handle %d has the synonym value.\r\n", handle);
  conn->handle = handle;
  entity->handle = handle;
  twicPrintf("Adr = 0x%x\r\n", entity);
  twicHashInsertHandle(twicHashValue(handle), entity);

  return;
}

void twicHashInsert2ndHandle(twicEntry_t * const entity, const uint16_t handle)
{
  //if (twicHashFind2ndHandle(handle))
  //twicLog("handle %d has the synonym value.\r\n", handle);
  entity->u.attr.b_handle = handle;
  twicHashInsertHandle(twicHashValue(handle), entity);

  return;
}

twicEntry_t *twicHashFind1stHandle(const uint16_t handle)
{
  twicEntry_t *(*_hash)[] = TWIC_HASH(srvif);
  twicEntry_t *p = NULL;
  uint16_t value = handle;
  
  for (p = (*_hash)[twicHashValue(value)]; (p); p = p->next)
    if (TWIC_ENTRY_TYPE_MNG == p->type && p->handle == handle)
      return p;
  
  return p;
}

twicEntry_t *twicHashFind2ndHandle(const uint16_t handle)
{
  twicEntry_t *(*_hash)[] = TWIC_HASH(srvif);
  twicEntry_t *p = NULL;
  uint16_t value = handle;

  for (p = (*_hash)[twicHashValue(value)]; (p); p = p->next)
    if (TWIC_ENTRY_TYPE_MNG != p->type && p->u.attr.b_handle == handle)
      return p;
  
  return p;
}

void twicHashRemove1stHandle(const uint16_t handle)
{
  twicEntry_t *(*_hash)[] = TWIC_HASH(srvif);
  twicEntry_t *p, *q;
  uint16_t value = handle;
  uint16_t idx = twicHashValue(value);
  
  for (p = (*_hash)[idx]; (p);) {
    if (TWIC_ENTRY_TYPE_MNG == p->type && p->handle == handle) {
      q = p->next; p->next = NULL;
      if (p->prev) { p->prev->next = q; p->prev = NULL; }
      else (*_hash)[idx] = q;
      p = q;
    } else p = p->next;
  }

  return;
}

void twicHashRemove2ndHandle(const uint16_t handle)
{
  twicEntry_t *(*_hash)[] = TWIC_HASH(srvif);
  twicEntry_t *p, *q;
  uint16_t value = handle;
  uint16_t idx = twicHashValue(value);
  
  for (p = (*_hash)[idx]; (p);) {
    if (TWIC_ENTRY_TYPE_MNG != p->type && p->u.attr.b_handle == handle) {
      q = p->next; p->next = NULL;
      if (p->prev) { p->prev->next = q; p->prev = NULL; }
      else (*_hash)[idx] = q;
      p = q;
    } else p = p->next;
  }
  
  return;
}

void twicHashCleanupInterface(const uint8_t interface)
{
  twicEntry_t *(*_hash)[] = TWIC_HASH(srvif);
  twicEntry_t *p, *q;
  uint16_t idx;
  
  for (idx = 0; idx < TWIC_HASH_SIZE(srvif); idx++) {
    for (p = (*_hash)[idx]; (p);) {
      if (p->interface == interface) {      
        q = p->next; p->next = NULL;
        if (p->prev) { p->prev->next = q; p->prev = NULL; }
        else (*_hash)[idx] = q;
        p = q;
      } else p = p->next;
    }
  }

  return;
}

