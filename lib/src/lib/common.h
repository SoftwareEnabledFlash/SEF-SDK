/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * common.h
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

#ifndef __COMMON_H__
#define __COMMON_H__

#include <assert.h>
#if defined(UNIT_TEST) && defined(ENV_COVERAGE)
#include <gcov.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef UNIT_TEST
#define STATIC
#ifdef ENV_COVERAGE
#if defined(__GNUC__) && __GNUC__ > 10
#define SEF_GCOV_FLUSH \
    __gcov_dump();     \
    __gcov_reset();
#else
#define SEF_GCOV_FLUSH __gcov_flush()
#endif    // __GNUC__
#else
#define SEF_GCOV_FLUSH
#endif    // ENV_COVERAGE
#else
#define STATIC static
#define SEF_GCOV_FLUSH
#endif    // UNIT_TEST

#ifdef NDEBUG
#define SEF_ASSERT(expr) (__ASSERT_VOID_CAST(0))
#else /* Not NDEBUG */
#define SEF_ASSERT(expr)                                                 \
    do                                                                   \
    {                                                                    \
        if (expr)                                                        \
        {                                                                \
            __ASSERT_VOID_CAST(0);                                       \
        }                                                                \
        else                                                             \
        {                                                                \
            SEF_GCOV_FLUSH;                                              \
            __assert_fail(#expr, __FILE__, __LINE__, __ASSERT_FUNCTION); \
        }                                                                \
    } while (0)
#endif /* NDEBUG */

#ifdef __cplusplus
}
#endif

#define sizeof_member(a, m)   sizeof(((a *)0)->m)
#define struct_size(p, m, n)  (sizeof(*(p)) + sizeof(*(p)->m) * (n))
#define trailing_struct(p, o) ((void *)(((char *)(p)) + (o)))

//!< Use NVMe fused, requires a driver that supports fused an IOSQE_IO_FUSE.
#define CC_USE_FUSED 0

#define CC_EBUSY_LOG_TIME_LIMIT_SEC 30    //!< Time limit on EBUSY retry log msg
#define CC_ACR_VALIDATION           1     //!< Enable ACR validation
#define CC_ARR_VALIDATION           1    //!< Enable ARR validation, also enables LD2 specific fixes
#define CC_DUMP_NLC                 0    //!< Enable dumping the NLC request sent to a device
#define CC_IGNORE_EXTRA_CLOSE       1    //!< LD2 can create multiple closes

//!< LD2 doesn't support FUA on NLC Controls if FUA or flush is used when the io commit flag is set
#define CC_FLUSH_SYNC_COPY 1

#define CC_EMU_VALIDATION 1    //!< Validate emulator is supported

#endif /* __COMMON_H__ */
