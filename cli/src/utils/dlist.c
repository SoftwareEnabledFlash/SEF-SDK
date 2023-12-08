/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * dlist.c
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
#include "dlist.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>

// LCOV_EXCL_START

void utl_DListInsertAfter(TmaDListEntry *iter, TmaDListEntry *node)
{
    assert(node->next == NULL);
    assert(node->prev == NULL);
    assert(iter->next != NULL);
    assert(iter->prev != NULL);
    node->next = iter->next;
    node->prev = iter;
    iter->next->prev = node;
    iter->next = node;
}

void utl_DListInsertBefore(TmaDListEntry *iter, TmaDListEntry *node)
{
    assert(node->next == NULL);
    assert(node->prev == NULL);
    assert(iter->next != NULL);
    assert(iter->prev != NULL);
    node->next = iter;
    node->prev = iter->prev;
    iter->prev->next = node;
    iter->prev = node;
}

TmaDListEntry *utl_DListRemove(TmaDListEntry *node)
{
    node->next->prev = node->prev;
    node->prev->next = node->next;
    node->next = NULL;
    node->prev = NULL;
    return node;
}

void utl_DListInit(TmaDList *list)
{
    list->head.next = &list->head;
    list->head.prev = &list->head;
}

void utl_DListInitEntry(TmaDListEntry *node)
{
    node->next = NULL;
    node->prev = NULL;
}

bool utl_DListIsEmpty(TmaDList *list)
{
    return list->head.next == &list->head /*|| list->head.next == NULL*/;
}

bool utl_DListIsInList(TmaDListEntry *entry)
{
    return entry->next != NULL;
}

TmaDListEntry *utl_DListPopHead(TmaDList *list)
{
    if (!utl_DListIsEmpty(list))
    {
        return utl_DListRemove(list->head.next);
    }
    return NULL;
}

TmaDListEntry *utl_DListPopTail(TmaDList *list)
{
    if (!utl_DListIsEmpty(list))
    {
        return utl_DListRemove(list->head.prev);
    }
    return NULL;
}

void utl_DListAppendList(TmaDList *dst, TmaDList *src)
{
    dst->head.prev->next = src->head.next;
    src->head.next->prev = dst->head.prev;
    dst->head.prev = src->head.prev;
    src->head.prev->next = &dst->head;
    utl_DListInit(src);
}