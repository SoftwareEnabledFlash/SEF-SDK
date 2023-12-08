/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * dlist.h
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * This software is licensed under the 3-Clause BSD License.
 *
 * Copyright (C) 2018-2023 - KIOXIA Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

#ifndef SEF_SRC_UTILS_DLIST_H
#define SEF_SRC_UTILS_DLIST_H

#include <stdbool.h>

#ifndef cointainer_of
#include <stddef.h>

#define container_of(ptr, type, member)                    \
    ({                                                     \
        const typeof(((type *)0)->member) *__mptr = (ptr); \
        (type *)((char *)__mptr - offsetof(type, member)); \
    })
#endif

/**
 * @brief circularly double linked list entry structure
 **/
typedef struct TmaDListEntry
{
    struct TmaDListEntry *next; /**< next entry in the list (or 1st entry if this is the head of a list) */
    struct TmaDListEntry *prev; /**< previous entry in the list (or last entry if this is the head of a list) */
} TmaDListEntry;

/**
 * @brief  list head structure
 **/
typedef struct TmaDList
{
    struct TmaDListEntry head;
} TmaDList;

/**
 * @brief insert a node after a given node/list
 *
 * @param iter Node to insert after or a list to insert at the head
 * @param node Node to insert into the list iter lives in.
 **/
void utl_DListInsertAfter(TmaDListEntry *iter, TmaDListEntry *node);
#define utl_DListPushHead(L, N) utl_DListInsertAfter(&(L)->head, N)

#define utl_DListGetHead(L) (utl_DListIsEmpty(L) ? NULL : (L)->head.next)

#define utl_DListGetHeadAs(L, T, M) \
    (utl_DListIsEmpty(L) ? NULL : container_of((L)->head.next, T, M))

#define utl_DListNextAs(L, N, T, M)                                               \
    ((N) ? ((N)->M.next == (&(L)->head) ? NULL : container_of((N)->M.next, T, M)) \
         : utl_DListGetHeadAs((L), T, M))

#define utl_DListPrevAs(L, N, T, M)                                               \
    ((N) ? ((N)->M.prev == (&(L)->head) ? NULL : container_of((N)->M.prev, T, M)) \
         : utl_DListGetTailAs((L), T, M))

/**
 * @brief insert a node before a given node/list
 *
 * @param iter Node to insert before or a list to insert at the head
 * @param node Node to insert into the list iter lives in.
 **/
void utl_DListInsertBefore(TmaDListEntry *ip, TmaDListEntry *node);
#define utl_DListPushTail(L, N) utl_DListInsertBefore(&(L)->head, N)

#define utl_DListGetTail(L) (utl_DListIsEmpty(L) ? NULL : (L)->head.prev)

#define utl_DListGetTailAs(L, T, M) \
    (utl_DListIsEmpty(L) ? NULL : container_of((L)->head.prev, T, M))

/**
 * @brief Removes a node from a list
 *
 * @param node Node to remove from a TmaDList
 * @return TmaDListEntry* Pointer to the node removed as a convience
 **/
TmaDListEntry *utl_DListRemove(TmaDListEntry *node);

/**
 * @brief Init's a list to be an empty list
 *
 * Run time version of UTL_INIT_DLIST()
 *
 * @param head entry to init as a head entry
 **/
void utl_DListInit(TmaDList *list);

/**
 * @brief Initializes an entry
 *
 * @param node to initialize as an entry
 **/
void utl_DListInitEntry(TmaDListEntry *node);

/**
 * @brief Given a list, return true is it's empty
 *
 * @param head Head of a TmaDList
 * @return true if the list is empty
 * @return false if the list is not empty
 **/
bool utl_DListIsEmpty(TmaDList *list);

/**
 * @brief Given a list entry, returns true if it's in a list
 *
 * @param entry Entry to test if it's in a list or not
 * @return true if the entry is in a list
 * @return false if th entry is not in a list
 */
bool utl_DListIsInList(TmaDListEntry *entry);

/**
 * @brief Pops the head entry off a list off and returns it
 *
 * @param list TmaDList to pop an entry from
 * @return TmaDListEntry* Head entry in a list or null if empty
 **/
TmaDListEntry *utl_DListPopHead(TmaDList *list);
#define utl_DListPopHeadAs(L, T, M) \
    (utl_DListIsEmpty(L) ? NULL : container_of(utl_DListRemove((L)->head.next), T, M))

/**
 * @brief Pops the tail entry off a list off and returns it
 *
 * @param list TmaDList to pop an entry from
 * @return TmaDListEntry* Tail entry in a list or null if empty
 **/
TmaDListEntry *utl_DListPopTail(TmaDList *list);
#define utl_DListPopTailAs(L, T, M) \
    (utl_DListIsEmpty(L) ? NULL : container_of(utl_DListRemove((L)->prev), T, M))

/**
 * @brief Pos all the entries from src and appends on dst
 *
 * @param dst Destination list for source entries
 * @param src Source list to move to dst
 */
void utl_DListAppendList(TmaDList *dst, TmaDList *src);

/**
 * @brief Used to initialize a statically allocated TMADList head structure.
 *
 * static version of utl_DListInit()
 *
 * @parm L pointer to a TmaDList
 */
#define UTL_INIT_DLIST(L)          \
    {                              \
        {                          \
            &(L)->head, &(L)->head \
        }                          \
    }

/**
 * @brief Used to declare a statically allocated TmaDList
 *
 * @param N name of of the the static head to declare
 */
#define UTL_DECLARE_DLIST(N) TmaDList N = UTL_INIT_DLIST(&(N))

#endif
