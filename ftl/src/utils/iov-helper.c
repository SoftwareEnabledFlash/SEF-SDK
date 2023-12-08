/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * iov-helper.c
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
#include "iov-helper.h"

#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/uio.h>

#include "../config.h"
#include "../sef-utils.h"

ssize_t lenIov(struct iovec *iov, int iovcnt)
{
    ssize_t len = 0;
    while (iovcnt--)
    {
        len += iov++->iov_len;
    }
    return len;
}

void memsetIov(struct iovec *iov, int iovcnt, uint32_t iovOffset, char value, int count)
{
    int srcIndex = 0;
    size_t thisTime;

    // skipping the oivOffset values
    while (iovOffset)
    {
        if (iov[srcIndex].iov_len <= iovOffset)
        {
            iovOffset -= iov[srcIndex].iov_len;
            srcIndex++;
            assert(srcIndex < iovcnt);
            continue;
        }

        thisTime = iov[srcIndex].iov_len - iovOffset;
        if (thisTime > count)
        {
            thisTime = count;
        }
        memset(iov[srcIndex].iov_base + iovOffset, value, thisTime);
        count -= thisTime;
        srcIndex++;
        break;
    }

    // set the value of the iov
    while (count)
    {
        assert(srcIndex < iovcnt);
        thisTime = iov[srcIndex].iov_len;
        if (thisTime > count)
        {
            thisTime = count;
        }
        memset(iov[srcIndex].iov_base, value, thisTime);
        count -= thisTime;
        srcIndex++;
    }
}

void copyIov(const struct iovec *srcIov,
             int srcIovcnt,
             size_t srcIovOffset,
             struct iovec **destIov,
             uint16_t *destIovcnt)
{
    int srcIndex = 0;
    int dstIndex = 0;
    *destIov = (struct iovec *)SUmalloc(srcIovcnt * sizeof(struct iovec));    // worst case...
    assert(*destIov);
    (*destIov)[0].iov_base = srcIov[0].iov_base;

    // skipping the oivOffset values
    while (srcIovOffset)
    {
        if (srcIov[srcIndex].iov_len <= srcIovOffset)
        {
            srcIovOffset -= srcIov[srcIndex].iov_len;
            srcIndex++;
            assert(srcIndex < srcIovcnt);
            continue;
        }

        (*destIov)[0].iov_base = (char *)srcIov[srcIndex].iov_base + srcIovOffset;
        (*destIov)[0].iov_len = srcIov[srcIndex].iov_len - srcIovOffset;
        srcIndex++;
        dstIndex++;
        break;
    }

    // copy the value of src to dest iov
    while (srcIndex < srcIovcnt)
    {
        (*destIov)[dstIndex] = srcIov[srcIndex];
        srcIndex++;
        dstIndex++;
    }

    *destIovcnt = dstIndex;
}
