/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef-slist.c
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
#include "sef-slist.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>    // NULL
#include <stdint.h>    // uintptr_t

void SefSlistInitialize(struct sefSlist* head)
{
    atomic_store(&head->val, 0);
}

struct sefSlistNode* SefSlistPopAll(struct sefSlist* list)
{
    return (struct sefSlistNode*)atomic_exchange(&list->val, (uintptr_t)NULL);
}

bool SefSlistPush(struct sefSlist* list, struct sefSlistNode* node)
{
    uintptr_t new_head = (uintptr_t)node;
    uintptr_t old_head = atomic_load(&list->val);
    node->next = (struct sefSlistNode*)old_head;
    while (!atomic_compare_exchange_weak(&list->val, &old_head, new_head))
    {
        node->next = (struct sefSlistNode*)old_head;
    }

    return old_head == (uintptr_t)NULL;
}

struct sefSlistNode* SefSlistReverse(struct sefSlistNode* list)
{
    struct sefSlistNode* rlist = NULL;

    while (list)
    {
        struct sefSlistNode* next = list->next;
        list->next = rlist;
        rlist = list;
        list = next;
    }

    return rlist;
}
