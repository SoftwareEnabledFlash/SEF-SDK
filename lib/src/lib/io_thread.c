/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * io_thread.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include "io_thread.h"
#include "liburing.h"
#include "ulog.h"

static pthread_key_t pthread_key;

pthread_once_t once_control = PTHREAD_ONCE_INIT;

#define MAX_UCSX_QUEUE_COUNT 2

static unsigned int io_uring_get_sqes(struct io_uring *ring, struct io_uring_sqe **sqes, unsigned int count)
{
    if (io_uring_sq_space_left(ring) >= count)
    {
        for (int i = 0; i < count; i++)
        {
            sqes[i] = io_uring_get_sqe(ring);
        }
        return count;
    }
    return 0;
}

int io_thread_submit(struct io_thread *io_thread, struct io_request *request)
{
    if (!io_thread)
    {
        return -EINVAL;
    }

    if (io_thread->shutdown)
    {
        return -ESHUTDOWN;
    }
    atomic_fetch_add(&io_thread->queued, 1);
    bool was_empty = SefSlistPush(&io_thread->queue, &request->link);
    if (was_empty)
    {
        eventfd_write(io_thread->wfd, 1);
    }
    return 0;
}

static void io_thread_cleanup(void *param)
{
    struct io_thread *io_thread = param;

    io_uring_queue_exit(io_thread->ring);
    close(io_thread->efd);
    close(io_thread->wfd);
}

static void init_once(void)
{
    pthread_key_create(&pthread_key, io_thread_cleanup);
}

static void *io_thread_func(void *param)
{
    struct io_thread *self = param;
    pthread_setspecific(pthread_key, self);
    struct sefSlistNode *list = NULL;
    struct io_uring *ring = self->ring;
    uint32_t outstanding = 0, completed = 0;
    uint64_t submitted = 0;
    uint64_t received = 0;
    int num_queued = 0;

    int block_with_queued = 0;
    int block_with_outstanding = 0;
    int new_items_in_queue = 0;
    int woke_with_outstanding = 0;
    int woke_with_incoming = 0;
    int total_loops = 0;

    while (!self->shutdown || (outstanding || atomic_load(&self->queue.head)))
    {
        struct io_request *request;

        total_loops++;

        if (!list)
        {
            list = SefSlistPopAll(&self->queue);
            if (list)
            {
                list = SefSlistReverse(list);
                new_items_in_queue++;
            }
        }

        if (!list && (num_queued == 0 || completed == 0))
        {
            if (outstanding)
            {
                block_with_outstanding++;
            }
            if (num_queued)
            {
                block_with_queued++;
            }
            fd_set fds;
            struct timeval tv;

            /* Watch stdin (fd 0) to see when it has input. */

            FD_ZERO(&fds);
            FD_SET(self->efd, &fds);
            FD_SET(self->wfd, &fds);

            /* Wait up to five seconds. */

            tv.tv_sec = 2;
            tv.tv_usec = 0;

            int ret =
                select((self->efd > self->wfd ? self->efd : self->wfd) + 1, &fds, NULL, NULL, &tv);
            if (ret > 0)
            {
                if (FD_ISSET(self->efd, &fds))
                {
                    eventfd_read(self->efd, NULL);
                }
                if (FD_ISSET(self->wfd, &fds))
                {
                    eventfd_read(self->wfd, NULL);
                }
            }
            else if (ret < 0)
            {
                ULOG_ERROR("select error = %d\n", ret);
            }
            if (outstanding)
            {
                woke_with_outstanding++;
            }
            if (atomic_load(&self->queue.head))
            {
                woke_with_incoming++;
            }
        }

        while ((request = (struct io_request *)list) && outstanding < *ring->cq.kring_entries)
        {
            struct io_uring_sqe *sqes[MAX_UCSX_QUEUE_COUNT];
            int count = request->fused ? 2 : 1;
            int i;

            int num_sqes = io_uring_get_sqes(ring, sqes, count);

            if (num_sqes < count)
            {
                ULOG_DEBUG("no sqes %d %d\n", num_sqes, count);
                break;
            }

            num_queued += count;

            for (i = 0; i < count; ++i)
            {
                request->prep(sqes[i], request);
                io_uring_sqe_set_data(sqes[i], request);
                request = request->fused;
            }

            list = list->next;
        }

        submitted += num_queued;

        if (num_queued)
        {
            int count = io_uring_submit(ring);
            if (count != num_queued)
            {
                ULOG_NOTICE("Short submit %d %d\n", count, num_queued);
                fprintf(stderr, "Short submit %d %d\n", count, num_queued);
            }
            if (count > 0)
            {
                num_queued -= count;
                outstanding += count;
                atomic_fetch_add(&self->submitted, count);
            }
            else
            {
                ULOG_NOTICE("uring submit error %s\n", strerror(count));
            }
        }

        completed = 0;

        if (outstanding)
        {
            do
            {
                struct io_uring_cqe *cqe;
                int ret = io_uring_peek_cqe(ring, &cqe);
                if (ret)
                {
                    if (ret != -EAGAIN)
                    {
                        ULOG_ERROR("read failed: %s\n", strerror(ret));
                    }
                    break;
                }
                else
                {
                    struct io_request *request = io_uring_cqe_get_data(cqe);
                    int res = cqe->res;
                    uint64_t result = *(uint64_t *)cqe->big_cqe;
                    io_uring_cqe_seen(ring, cqe);
                    atomic_fetch_add(&self->completed, 1);
                    request->complete(request, res, result);
                    received++;
                    completed++;
                }
            } while (--outstanding);
        }
    }

    return NULL;
}

int io_thread_create(int queue_size, struct io_thread **out)
{
    int ret;

    pthread_once(&once_control, init_once);

    struct io_thread *io_thread = calloc(1, sizeof(struct io_thread) + sizeof(struct io_uring));
    if (!io_thread)
    {
        return -ENOMEM;
    }

    io_thread->wfd = -1;
    io_thread->efd = eventfd(0, 0);
    if (io_thread->efd == -1)
    {
        ret = -errno;
        goto error;
    }
    io_thread->wfd = eventfd(0, 0);
    if (io_thread->wfd == -1)
    {
        ret = -errno;
        goto error;
    }
    io_thread->ring = (struct io_uring *)(io_thread + 1);
    ret = io_uring_queue_init(queue_size, io_thread->ring, IORING_SETUP_SQE128 | IORING_SETUP_CQE32);
    if (ret)
    {
        goto error;
    }

    io_uring_register_eventfd(io_thread->ring, io_thread->efd);

    SefSlistInitialize(&io_thread->queue);

    ret = pthread_create(&io_thread->thread, NULL, io_thread_func, io_thread);
    if (ret)
    {
        goto error;
    }

    *out = io_thread;

    return 0;

error:
    close(io_thread->efd);
    close(io_thread->wfd);
    free(io_thread);
    return ret;
}

int io_thread_delete(struct io_thread *io_thread)
{
    void *ret = 0;

    if (!io_thread)
    {
        return -EINVAL;
    }

    io_thread->shutdown = true;
    eventfd_write(io_thread->wfd, 1);
    pthread_join(io_thread->thread, &ret);
    free(io_thread);
    return (intptr_t)ret;
}

struct io_thread *io_thread_get()
{
    return pthread_getspecific(pthread_key);
}

int io_thread_is()
{
    return (io_thread_get() != 0);
}
