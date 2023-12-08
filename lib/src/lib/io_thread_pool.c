/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * io_thread_pool.c
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

#include "io_thread_pool.h"
#include <stdatomic.h>
#include <stdlib.h>

struct io_thread_pool *io_thread_pool_create(int num_threads, int queue_size)
{
    int ret = 0;

    struct io_thread_pool *pool = calloc(1, sizeof(*pool) + num_threads * sizeof(struct io_thread *));
    if (!pool)
    {
        return NULL;
    }

    int i;
    for (i = 0; i < num_threads; ++i)
    {
        ret = io_thread_create(queue_size, &pool->pool[i]);
        if (ret)
        {
            break;
        }
    }

    pool->count = num_threads;

    if (ret)
    {
        for (i--; i >= 0; --i)
        {
            io_thread_delete(pool->pool[i]);
        }
        free(pool);
        pool = 0;
    }

    return pool;
}

int io_thread_pool_delete(struct io_thread_pool *pool)
{
    int i;

    if (!pool)
    {
        return -EINVAL;
    }

    for (i = 0; i < pool->count; ++i)
    {
        io_thread_delete(pool->pool[i]);
    }
    free(pool);
    return 0;
}

int io_thread_pool_submit(struct io_thread_pool *pool, struct io_request *request)
{
    struct io_thread *io_thread;

    if (!pool)
    {
        return -EINVAL;
    }

    io_thread = io_thread_get();

    if (!io_thread)
    {
        atomic_uint_fast64_t *index = (atomic_uint_fast64_t *)&pool->index;
        uint64_t cur = atomic_fetch_add(index, 1) % pool->count;
        io_thread = pool->pool[cur];
    }

    return io_thread_submit(io_thread, request);
}
