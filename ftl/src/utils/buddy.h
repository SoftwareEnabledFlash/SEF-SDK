/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * buddy.h
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

#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Page
{
    struct Page *next;
    struct Page **prev;
    uint32_t order;
    uint32_t num;
} Page;

typedef struct Order
{
    uint32_t order;
    uint32_t free_count;
    atomic_uchar *bitmap;
    Page *free_list;
    pthread_mutex_t mutex;
} Order;

typedef struct BuddyAllocator
{
    uint32_t order;
    uint32_t page_size;
    void *buffer;
    Page *pages;
    Order *orders;
    uint32_t free_pages;
} BuddyAllocator;

/**
 *  @brief      Allocate a pool of 2 ^ order * page_size and
 *              create a BuddyAllocator using it
 *
 *  @param      buddy       The buddy allocator to initialize
 *  @param      order       The number of pages to create, defined as 2 ^ order
 *  @param      page_size   The size of the pages that can be allocated
 *
 *  @retval     0        The allocator was created successfully
 *  @retval    -ENOMEM   There was insufficient memory to create the allocator
 *
 */
int buddy_init(BuddyAllocator *buddy, uint32_t order, uint32_t page_size);

/**
 *  @brief      Release the memory associated with a previously created allocator
 *
 *  @param      buddy       The buddy allocator to destroy
 *
 *  @retval     0           The allocator was destroyed successfully
 *  @retval     -EBUSY      The allocator still had outstanding pages
 *
 */
int buddy_destroy(BuddyAllocator *buddy);

/**
 *  @brief      Allocate a 2 ^ order * pages from the allocator
 *
 *  @param      buddy       The buddy allocator to allocate from
 *  @param      order       The number of pages to allocate, defined as 2 ^ order
 *
 *  @return     A pointer to the allocated memory, or an error code
 *
 *  @retval    -ENOMEM      There was insufficient memory to allocator the desired
 *
 */
void *buddy_alloc_order(BuddyAllocator *buddy, uint32_t order);

/**
 *  @brief      Allocate pages from the allocator
 *
 *  @param      buddy       The buddy allocator to allocate from
 *  @param      page_size   The number pages to allocated
 *
 *  @return     A pointer to the allocated memory, or an error code
 *
 *  @retval    -ENOMEM      There was insufficient memory to allocator the desired
 *
 */
void *buddy_alloc_pages(BuddyAllocator *buddy, uint32_t pages);

/**
 *  @brief      Allocate the largest amount of pages available up to num_pages
 *
 *  @param      buddy       The buddy allocator to allocate from
 *  @param      num_pages   A pointer to the number of pages desired, contains
 *                          the number actually allocated if allocation is
 *                          successful
 *
 *  @return     A pointer to the allocated memory, or an error code
 *
 *  @retval    -ENOMEM      There was insufficient memory to allocator the desired
 *
 */
void *buddy_alloc_pages_avail(BuddyAllocator *buddy, uint32_t *num_pages);

/**
 *  @brief      Return 2 ^ order pages to an allocator
 *
 *  @param      buddy       The buddy allocator to return pages to
 *  @param      order       The number of pages to return, defined as 2 ^ order
 *
 *  @retval     0        The pages were successfully returned
 *  @retval    -EINVAL   The order was too large for the allocator, or the pages
 *                       were not pages previously allocated by the allocator
 *
 */
int buddy_free_order(BuddyAllocator *buddy, void *, uint32_t order);

/**
 *  @brief      Return pages to an allocator
 *
 *  @param      buddy       The buddy allocator to return pages to
 *  @param      pages       The number of pages to return
 *
 *  @retval     0        The pages were successfully returned
 *  @retval    -EINVAL   The pages were not pages previously allocated by the allocator
 *
 */
int buddy_free_pages(BuddyAllocator *buddy, void *, uint32_t pages);

/**
 *  @brief      Return pages to an allocator in whatever the order the
 *              pages were originally allocated at
 *
 *  @param      buddy       The buddy allocator to return pages to
 *  @param      pages       The pages to return
 *
 *  @retval     0        The pages were successfully returned
 *  @retval    -EINVAL   The pages were not pages previously allocated by the allocator
 *
 */
int buddy_free(BuddyAllocator *buddy, void *pages);

/**
 *  @brief      The number of pages currently available in the allocator
 *              NOTE: not thread safe
 *
 *  @param      buddy   The buddy allocator to get the number of pages from
 *
 *  @return     The number of pages available
 *
 */
uint32_t buddy_avail_pages(const BuddyAllocator *buddy);

/**
 *  @brief      Return the largest number of pages that can be allocated with
 *              buddy_alloc_order
 *
 *  @param      buddy   The buddy allocator to get the number of pages from
 *
 *  @return     The largest number of pages that can be allocated with
 *              buddy_alloc_order
 *
 */
uint32_t buddy_max_avail_pages(const BuddyAllocator *buddy);

/**
 *  @brief      Run a diagnostic on the buddy allocator
 *              NOTE: not implemented
 *
 *  @param      buddy       The buddy allocator to check
 *  @param      print       Whether to print out defects as found
 *
 *  @retval     0        Always returns 0
 *
 */
int buddy_check(const BuddyAllocator *buddy, bool print);

#ifdef __cplusplus
}
#endif
