/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef-slist.h
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

#ifdef __cplusplus
#include <atomic>
using std::atomic_uintptr_t;
#else /* not __cplusplus */
#ifdef __STDC_NO_ATOMICS__
#error C11 atomics are required
#endif /* __STDC_NO_ATOMICS__ */

#include <stdatomic.h>
#endif /* __cplusplus */

#include <stdbool.h>
#include <stddef.h>    // offset of
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#ifndef cointainer_of
#define container_of(ptr, type, member)                   \
    ({                                                    \
        const typeof(((type*)0)->member)* __mptr = (ptr); \
        (type*)((char*)__mptr - offsetof(type, member));  \
    })
#endif

struct sefSlist
{
    _Atomic(struct sefSlistNode*) head;
};

struct sefSlistNode
{
    struct sefSlistNode* next;
};

//!\brief Initializes a list head so it's ready to use
void SefSlistInitialize(struct sefSlist* head);

//!\brief Returns the current head and sets the current head to NULL
struct sefSlistNode* SefSlistPopAll(struct sefSlist* list);

//!\brief Returns the current head and sets the current head to NULL
struct sefSlistNode* SefSlistPopAllIf(struct sefSlist* list, struct sefSlistNode* head);

//!\brief Entry pushed to head and next set to old head, returns list prev-empty state
bool SefSlistPush(struct sefSlist* list, struct sefSlistNode* node);

struct sefSlistNode* SefSlistGetHead(struct sefSlist* list);

//!\brief Reverse a popped list
struct sefSlistNode* SefSlistReverse(struct sefSlistNode* list);

#define SefSlistNodeAs(_N, _T, _M)            \
    ({                                        \
        __typeof__(_N) _n = (_N);             \
        _n ? container_of(_n, _T, _M) : NULL; \
    })

#define SefSlistPopAllAs(l, t, m) SefSlistNodeAs(SefSlistPopAll(l), t, m)

// note: not safe to use on a shared list
#define SefSlistNextAs(n, t, m) (n ? SefSlistNodeAs((n)->next, t, m) : NULL)
