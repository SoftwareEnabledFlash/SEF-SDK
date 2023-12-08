/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * buddy.c
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
#include "buddy.h"

#include <stdatomic.h>

#define DIV_ROUND_UP(x, y) ((x) / (y) + ((x) % (y) ? 1 : 0))

/*** Conversion utilities ***/

static inline uint32_t page_to_num(const BuddyAllocator *buddy, Page *page)
{
    return (uint32_t)(page - buddy->pages);
}

static inline uint32_t addr_to_num(const BuddyAllocator *buddy, void *buf)
{
    return (buf - buddy->buffer) / buddy->page_size;
}

static inline Page *num_to_page(const BuddyAllocator *buddy, uint32_t page_num)
{
    return &buddy->pages[page_num];
}

inline static uint32_t get_buddy_base(uint32_t page_num, uint32_t order)
{
    uint32_t buddy_page = page_num;
    buddy_page >>= order;
    buddy_page <<= order;
    return buddy_page;
}

inline static uint32_t get_buddy_num(uint32_t page_num, uint32_t order)
{
    return get_buddy_base(page_num, order) ^ (1 << order);
}

inline static uint32_t req_order_from_count(uint32_t num_pages)
{
    uint32_t o = 0;
    uint32_t np = num_pages;
    while (np >>= 1)
    {
        ++o;
    }
    return o + (num_pages == (1 << o) ? 0 : 1);
}

inline static uint32_t order_from_count(uint32_t num_pages)
{
    uint32_t r = 0;
    while (num_pages >>= 1)
    {
        ++r;
    }
    return r;
}

inline static uint32_t max_order(const BuddyAllocator *buddy, uint32_t page_num)
{
    int i;
    for (i = 0; i < buddy->order; ++i)
    {
        if (page_num & (1 << i))
        {
            break;
        }
    }
    return i;
}

inline static uint32_t order_from_buf(BuddyAllocator *buddy, void *buffer, uint32_t num_pages)
{
    uint32_t page_num = addr_to_num(buddy, buffer);
    uint32_t order = max_order(buddy, page_num);
    return ((1 << order) <= num_pages) ? order : order_from_count(num_pages);
}

/*** Methods for adding and removing items from an Order ***/

static inline uint32_t bits_required(const BuddyAllocator *buddy, uint32_t order)
{
    return ((1 << (buddy->order - order)) >> 1) ?: 1;
}

static inline uint32_t bytes_required(const BuddyAllocator *buddy, uint32_t order)
{
    return DIV_ROUND_UP(bits_required(buddy, order), 8) ?: 1;
}

inline static bool flip_bit(BuddyAllocator *buddy, int order, uint32_t page_num)
{
    uint32_t shift = order + 1;
    uint32_t byte_offset = page_num >> (shift + 3);
    uint32_t bit_offset = (page_num >> shift) & 0x7;
    uint8_t prev_value;
    prev_value = atomic_fetch_xor(&buddy->orders[order].bitmap[byte_offset], 1 << bit_offset);
    return prev_value & (1 << bit_offset);
}

inline static void remove_from_list(Page *page)
{
    *page->prev = page->next;
    if (page->next)
    {
        page->next->prev = page->prev;
    }
    page->next = 0;
    page->prev = 0;
}

inline static void remove_free_list(BuddyAllocator *buddy, int order, uint32_t page_num)
{
    Page *page = &buddy->pages[page_num];
    remove_from_list(page);
    --buddy->orders[order].free_count;
}

inline static Page *pop_free_list_locked(BuddyAllocator *buddy, int order)
{
    Page *page = buddy->orders[order].free_list;
    if (page)
    {
        remove_free_list(buddy, order, page_to_num(buddy, page));
        flip_bit(buddy, order, page_to_num(buddy, page));
    }
    return page;
}

inline static Page *pop_free_list(BuddyAllocator *buddy, int order)
{
    Page *page;
    pthread_mutex_lock(&buddy->orders[order].mutex);
    page = pop_free_list_locked(buddy, order);
    pthread_mutex_unlock(&buddy->orders[order].mutex);
    return page;
}

inline static bool push_free_list_locked(BuddyAllocator *buddy, int order, int page_num)
{
    Page *page = &buddy->pages[page_num];
    if (!flip_bit(buddy, order, page_num))
    {
        Order *bitmap = &buddy->orders[order];
        page->next = bitmap->free_list;
        page->prev = &bitmap->free_list;
        if (page->next)
        {
            page->next->prev = &page->next;
        }
        page->order = order;
        bitmap->free_list = page;
        ++bitmap->free_count;
        return true;
    }
    return false;
}

inline static bool push_free_list(BuddyAllocator *buddy, int order, int page_num)
{
    bool ret;
    pthread_mutex_lock(&buddy->orders[order].mutex);
    ret = push_free_list_locked(buddy, order, page_num);
    pthread_mutex_unlock(&buddy->orders[order].mutex);
    return ret;
}

// LCOV_EXCL_START
/*** Display methods ***/

static void print_list(const BuddyAllocator *buddy, int order)
{
    Page *p = buddy->orders[order].free_list;
    uint32_t bitmap_size = bytes_required(buddy, order);
    int i;
    printf("\t%d: ", order);
    while (p)
    {
        printf("%u ", page_to_num(buddy, p));
        p = p->next;
    }
    printf(" | ");
    for (i = 0; i < bitmap_size; ++i)
    {
        printf("%02hhx ", buddy->orders[order].bitmap[i]);
    }
    printf("\n");
}

static __attribute__((used)) void print_lists(BuddyAllocator *buddy)
{
    int order = buddy->order;
    printf("free list: \n");
    for (; order >= 0; --order)
    {
        print_list(buddy, order);
    }
}

// LCOV_EXCL_STOP

/*** Other utility methods ***/

static bool is_valid_page(BuddyAllocator *buddy, void *buffer, int order)
{
    uint64_t heap_size = (1 << buddy->order) * buddy->page_size;
    void *heap_start = buddy->buffer;
    uint64_t buf_size = (1 << order) * buddy->page_size;
    if (buffer < heap_start || buffer + buf_size > heap_start + heap_size)
    {
        return false;
    }
    if ((buffer - heap_start) % buddy->page_size)
    {
        return false;
    }
    return true;
}

static void split_down(BuddyAllocator *buddy, uint32_t page_num, int high, int low)
{
    while (--high >= low)
    {
        uint32_t buddy_page = get_buddy_num(page_num, high);
        push_free_list(buddy, high, buddy_page);
    }
}

static void join_up(BuddyAllocator *buddy, uint32_t page_num, int order)
{
    while (pthread_mutex_lock(&buddy->orders[order].mutex),
           !push_free_list_locked(buddy, order, page_num))
    {
        if (order >= buddy->order)
        {
            break;
        }
        uint32_t buddy_page = get_buddy_num(page_num, order);
        remove_free_list(buddy, order, buddy_page);
        pthread_mutex_unlock(&buddy->orders[order].mutex);
        ++order;
        page_num = get_buddy_base(page_num, order);
    }
    pthread_mutex_unlock(&buddy->orders[order].mutex);
}

/*** API methods ***/

int buddy_init(BuddyAllocator *buddy, uint32_t order, uint32_t page_size)
{
    uint64_t heap_size = page_size << order;
    uint64_t num_pages = 1 << order;
    int i;

    buddy->page_size = page_size;
    buddy->order = order;
    buddy->free_pages = num_pages;

    buddy->buffer = aligned_alloc(page_size, heap_size);
    if (!buddy->buffer)
    {
        return -ENOMEM;
    }

    buddy->pages = calloc(num_pages, sizeof(Page));
    for (i = 0; i < num_pages; ++i)
    {
        buddy->pages[i].num = i;
    }

    buddy->orders = calloc(order + 1, sizeof(Order));

    for (i = 0; i <= order; ++i)
    {
        uint32_t bitmap_size = bytes_required(buddy, i);
        // todo: is 2* req?
        buddy->orders[i].bitmap = calloc(2 * bitmap_size, sizeof(atomic_uchar));
        buddy->orders[i].order = i;
        pthread_mutex_init(&buddy->orders[i].mutex, NULL);
    }

    buddy->orders[order].free_count = 0;
    push_free_list_locked(buddy, order, 0);

    return 0;
}

int buddy_destroy(BuddyAllocator *buddy)
{
    int i;

    if (buddy_check(buddy, false))
    {
        printf("buddy is corrupt\n");
    }

    if (buddy_avail_pages(buddy) != (1 << buddy->order))
    {
        printf("Outstanding pages\n");
        return -EBUSY;
    }

    for (i = 0; i <= buddy->order; ++i)
    {
        free(buddy->orders[i].bitmap);
    }

    free(buddy->orders);
    free(buddy->pages);
    free(buddy->buffer);

    return 0;
}

void *buddy_alloc_order(BuddyAllocator *buddy, uint32_t order)
{
    void *buffer = 0;
    int i;

    if (order > buddy->order)
    {
        return 0;
    }

    for (i = order; i <= buddy->order; ++i)
    {
        Page *page = pop_free_list(buddy, i);
        if (page)
        {
            uint32_t page_num = page_to_num(buddy, page);
            buffer = buddy->buffer + (page_num * buddy->page_size);
            page->order = order;
            split_down(buddy, page_num, i, order);
            break;
        }
    }

    // printf("alloc %d %d\n", addr_to_num(buddy, buffer), order);
    //    print_lists(buddy);

    return buffer;
}

void *buddy_alloc_pages(BuddyAllocator *buddy, uint32_t num_pages)
{
    uint32_t order = req_order_from_count(num_pages);
    void *buffer = buddy_alloc_order(buddy, order);
    if (buffer)
    {
        uint32_t remainder = (1 << order) - num_pages;
        if (remainder)
        {
            buddy_free_pages(buddy, buffer + (num_pages * buddy->page_size), remainder);
        }
    }
    return buffer;
}

void *buddy_alloc_pages_avail(BuddyAllocator *buddy, uint32_t *num_pages)
{
    void *buffer = buddy_alloc_pages(buddy, *num_pages);
    if (!buffer)
    {
        uint32_t order = req_order_from_count(*num_pages);
        if (order)
        {
            do
            {
                buffer = buddy_alloc_order(buddy, --order);
            } while (!buffer && order);

            if (buffer)
            {
                *num_pages = 1 << order;
            }
        }
    }

    return buffer;
}

int buddy_free_order(BuddyAllocator *buddy, void *buffer, uint32_t order)
{
    Page *page;
    uint32_t page_num;

    if (order > buddy->order)
    {
        return -EINVAL;
    }

    if (!is_valid_page(buddy, buffer, order))
    {
        return -EINVAL;
    }

    page_num = addr_to_num(buddy, buffer);
    page = num_to_page(buddy, page_num);

    if (page->order != order)
    {
        // printf("order mismatch\n");
    }

    join_up(buddy, page_num, order);

    return 0;
}

int buddy_free_pages(BuddyAllocator *buddy, void *buffer, uint32_t num_pages)
{
    int ret = 0;
    //    printf("free pages %d %d\n", addr_to_num(buddy, buffer), num_pages);
    while (num_pages)
    {
        uint32_t order = order_from_buf(buddy, buffer, num_pages);
        if ((ret = buddy_free_order(buddy, buffer, order)))
        {
            printf("bad free %p\n", buffer);
            break;
        }
        num_pages -= (1 << order);
        buffer += ((1 << order) * buddy->page_size);
    }

    //    print_lists(buddy);
    return ret;
}

int buddy_free(BuddyAllocator *buddy, void *buffer)
{
    Page *page;
    uint32_t page_num;

    if (!is_valid_page(buddy, buffer, 0))
    {
        return -EINVAL;
    }

    page_num = addr_to_num(buddy, buffer);
    page = num_to_page(buddy, page_num);

    return buddy_free_order(buddy, buffer, page->order);
}

uint32_t buddy_avail_pages(const BuddyAllocator *buddy)
{
    uint32_t count = 0;
    int i;
    for (i = 0; i <= buddy->order; ++i)
    {
        count += buddy->orders[i].free_count << i;
    }
    return count;
}

uint32_t buddy_max_avail_pages(const BuddyAllocator *buddy)
{
    int i;
    for (i = buddy->order; i >= 0; --i)
    {
        if (buddy->orders[i].free_list)
        {
            return 1 << i;
        }
    }
    return 0;
}

int buddy_check(const BuddyAllocator *buddy, bool print)
{
    return 0;
}
