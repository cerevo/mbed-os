/**
 * @file tz1tl_list.h
 * @brief a header file for TZ10xx TWiC for Bluetooth 4.0/4.1 Smart
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

#ifndef _TZ1UT_LIST_H_
#define _TZ1UT_LIST_H_

#include "TZ10xx.h"

typedef TZ1K_PACKED_HDR struct tz1utListHead {
  struct tz1utListHead *next;
  struct tz1utListHead *prev;
} TZ1K_PACKED_FTR tz1utListHead_t;

#define TZ1UT_LIST_INIT(name) { &(tz1utList_##name), &(tz1utList_##name) }
#define TZ1UT_LIST_DEF(name)                                     \
  tz1utListHead_t tz1utList_##name = TZ1UT_LIST_INIT(name)
#define TZ1UT_LIST_EXTERN(name) extern tz1utListHead_t tz1utList_##name
#define TZ1UT_LIST(name) &(tz1utList_##name)


static TZ1K_INLINE void tz1utListInit(tz1utListHead_t *list)
{
  list->next = list;
  list->prev = list;
}

static TZ1K_INLINE void tz1utListInsert(
  tz1utListHead_t *in, tz1utListHead_t *prev, tz1utListHead_t *next)
{
  next->prev = in;
  in->next = next;
  in->prev = prev;
  prev->next = in;
}

static TZ1K_INLINE void tz1utListAdd(tz1utListHead_t *in, tz1utListHead_t *head)
{
  tz1utListInit(in);
  tz1utListInsert(in, head, head->next);
}

static TZ1K_INLINE void
tz1utListAddTail(tz1utListHead_t *in, tz1utListHead_t *head)
{
  tz1utListInit(in);
  tz1utListInsert(in, head->next, head);
}

static TZ1K_INLINE void
tz1utListRemove(tz1utListHead_t *prev, tz1utListHead_t *next)
{
  next->prev = prev;
  prev->next = next;
}

static TZ1K_INLINE void tz1utListDel(tz1utListHead_t *entry)
{
  tz1utListRemove(entry->prev, entry->next);
}

static TZ1K_INLINE unsigned char tz1utListIsEmpty(tz1utListHead_t *entry)
{
  tz1utListHead_t *next = entry->next;
  return (next == entry && next == entry->prev) ? 1 : 0;
}

#define tz1utListoffsetof(packed, element)       \
  ((unsigned long) &((packed *)0)->element)

#define tz1utListContainerOf(ptr, packed, element)       \
  (packed *)((char *)((tz1utListHead_t *)ptr) -          \
             tz1utListoffsetof(packed, element))

#define tz1utListEach(packed, pos, head, element)                        \
  for (pos = tz1utListContainerOf((head)->next, packed, element);        \
       &pos->element != (head);                                         \
       pos = tz1utListContainerOf(pos->element.next, packed, element))

#endif /* _TZ1UT_LIST_H_ */
