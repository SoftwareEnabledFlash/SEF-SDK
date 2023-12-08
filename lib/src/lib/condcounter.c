/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * condcounter.c
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
#undef NDEBUG
#include <assert.h>
#if !defined(_WIN32)
#include <stdatomic.h>
#else
#include "stdatomic.h"
#endif

#include "condcounter.h"

bool _condcounter_cond_add8(volatile condCounter8_t* object, int8_t cond, int8_t operand)
{
    int8_t oldVal = *object;
    int8_t newVal;

    do
    {
        if ((oldVal & 0x80) != (cond << 7))
        {
            return false;
        }
        newVal = oldVal + operand;
        newVal &= 0x7f;
        newVal |= cond << 7;
    } while (!atomic_compare_exchange_strong(object, &oldVal, newVal));

    return true;
}

bool _condcounter_cond_add16(volatile condCounter16_t* object, int16_t cond, int16_t operand)
{
    int16_t oldVal = *object;
    int16_t newVal;

    do
    {
        if ((oldVal & 0xc000) != (cond << 14))
        {
            return false;
        }
        newVal = oldVal + operand;
        newVal &= 0x3fff;
        newVal |= cond << 14;
    } while (!atomic_compare_exchange_strong(object, &oldVal, newVal));

    return true;
}

bool _condcounter_cond_add32(volatile condCounter32_t* object, int32_t cond, int32_t operand)
{
    int32_t oldVal = *object;
    int32_t newVal;

    do
    {
        if ((oldVal & 0xff000000) != (cond << 24))
        {
            return false;
        }
        newVal = oldVal + operand;
        newVal &= 0x00ffffff;
        newVal |= cond << 24;
    } while (!atomic_compare_exchange_strong(object, &oldVal, newVal));

    return true;
}

bool _condcounter_cond_add64(volatile condCounter64_t* object, int64_t cond, int64_t operand)
{
    int64_t oldVal = *object;
    int64_t newVal;

    do
    {
        if ((oldVal & 0xffff000000000000) != (cond << 48))
        {
            return false;
        }
        newVal = oldVal + operand;
        newVal &= 0x0000ffffffffffff;
        newVal |= cond << 48;
    } while (!atomic_compare_exchange_strong(object, &oldVal, newVal));

    return true;
}

int8_t _condcounter_sub8(volatile condCounter8_t* object, int8_t operand)
{
    int8_t oldVal = *object;
    int8_t newVal;
    int8_t cond;

    do
    {
        cond = oldVal & 0x80;
        newVal = oldVal - operand;
        newVal &= 0x7f;
        newVal |= cond;
    } while (!atomic_compare_exchange_strong(object, &oldVal, newVal));

    return (oldVal & 0x7f);
}

int16_t _condcounter_sub16(volatile condCounter16_t* object, int16_t operand)
{
    int16_t oldVal = *object;
    int16_t newVal;
    int16_t cond;

    do
    {
        cond = oldVal & 0xc000;
        newVal = oldVal - operand;
        newVal &= 0x3fff;
        newVal |= cond;
    } while (!atomic_compare_exchange_strong(object, &oldVal, newVal));

    return (oldVal & 0x3fff);
}

int32_t _condcounter_sub32(volatile condCounter32_t* object, int32_t operand)
{
    int32_t oldVal = *object;
    int32_t newVal;
    int32_t cond;

    do
    {
        cond = oldVal & 0xff000000;
        newVal = oldVal - operand;
        newVal &= 0x00ffffff;
        newVal |= cond;
    } while (!atomic_compare_exchange_strong(object, &oldVal, newVal));

    return (oldVal & 0x00ffffff);
}

int64_t _condcounter_sub64(volatile condCounter64_t* object, int64_t operand)
{
    int64_t oldVal = *object;
    int64_t newVal;
    int64_t cond;

    do
    {
        cond = oldVal & 0xffff000000000000;
        newVal = oldVal - operand;
        newVal &= 0x0000ffffffffffff;
        newVal |= cond;
    } while (!atomic_compare_exchange_strong(object, &oldVal, newVal));

    return (oldVal & 0x0000ffffffffffff);
}

int8_t _condcounter_condition_get8(volatile condCounter8_t* object)
{
    int8_t oldVal = *object;

    return (oldVal & 0x80) >> 7;
}

int16_t _condcounter_condition_get16(volatile condCounter16_t* object)
{
    int16_t oldVal = *object;

    return (oldVal & 0xc000) >> 14;
}

int32_t _condcounter_condition_get32(volatile condCounter32_t* object)
{
    int32_t oldVal = *object;

    return (oldVal & 0xff000000) >> 24;
}

int64_t _condcounter_condition_get64(volatile condCounter64_t* object)
{
    int64_t oldVal = *object;

    return (oldVal & 0xffff000000000000) >> 48;
}

int8_t _condcounter_counter_get8(volatile condCounter8_t* object)
{
    int8_t oldVal = *object;

    return (oldVal & 0x7f);
}

int16_t _condcounter_counter_get16(volatile condCounter16_t* object)
{
    int32_t oldVal = *object;

    return (oldVal & 0x3fff);
}

int32_t _condcounter_counter_get32(volatile condCounter32_t* object)
{
    int32_t oldVal = *object;

    return (oldVal & 0x00ffffff);
}

int64_t _condcounter_counter_get64(volatile condCounter64_t* object)
{
    int64_t oldVal = *object;

    return (oldVal & 0x0000ffffffffffff);
}

int8_t _condcounter_condition_exchange8(volatile condCounter8_t* object, int8_t desired)
{
    int8_t oldVal = *object;
    int8_t newVal;

    do
    {
        newVal = oldVal & 0x7f;
        newVal |= desired << 7;
    } while (!atomic_compare_exchange_strong(object, &oldVal, newVal));

    return (oldVal & 0x80) >> 7;
}

int16_t _condcounter_condition_exchange16(volatile condCounter16_t* object, int16_t desired)
{
    int16_t oldVal = *object;
    int16_t newVal;

    do
    {
        newVal = oldVal & 0x3fff;
        newVal |= desired << 14;
    } while (!atomic_compare_exchange_strong(object, &oldVal, newVal));

    return (oldVal & 0xc000) >> 14;
}

int32_t _condcounter_condition_exchange32(volatile condCounter32_t* object, int32_t desired)
{
    int32_t oldVal = *object;
    int32_t newVal;

    do
    {
        newVal = oldVal & 0x00ffffff;
        newVal |= desired << 24;
    } while (!atomic_compare_exchange_strong(object, &oldVal, newVal));

    return (oldVal & 0xff000000) >> 24;
}

int64_t _condcounter_condition_exchange64(volatile condCounter64_t* object, int64_t desired)
{
    int64_t oldVal = *object;
    int64_t newVal;

    do
    {
        newVal = oldVal & 0x0000ffffffffffff;
        newVal |= desired << 48;
    } while (!atomic_compare_exchange_strong(object, &oldVal, newVal));

    return (oldVal & 0xffff000000000000) >> 48;
}

bool _condcounter_compare_exchange8(volatile condCounter8_t* object, int8_t* expected, int8_t desired)
{
    int8_t oldVal = *object;
    int8_t newVal;
    int8_t condExpected;
    int8_t cond;

    while (1)
    {
        cond = oldVal & 0x80;
        condExpected = *expected & 0x7f;
        condExpected |= cond;
        newVal = desired & 0x7f;
        newVal |= cond;

        if (atomic_compare_exchange_strong(object, &condExpected, newVal))
        {
            return true;
        }

        if ((condExpected & 0x80) == cond)
        {
            *expected = condExpected & 0x7f;
            return false;
        }
    }
}

bool _condcounter_compare_exchange16(volatile condCounter16_t* object, int16_t* expected, int16_t desired)
{
    int16_t oldVal = *object;
    int16_t newVal;
    int16_t condExpected;
    int16_t cond;

    while (1)
    {
        cond = oldVal & 0xc000;
        condExpected = *expected & 0x3fff;
        condExpected |= cond;
        newVal = desired & 0x3fff;
        newVal |= cond;

        if (atomic_compare_exchange_strong(object, &condExpected, newVal))
        {
            return true;
        }

        if ((condExpected & 0xc000) == cond)
        {
            *expected = condExpected & 0x3fff;
            return false;
        }
    }
}

bool _condcounter_compare_exchange32(volatile condCounter32_t* object, int32_t* expected, int32_t desired)
{
    int32_t oldVal = *object;
    int32_t newVal;
    int32_t condExpected;
    int32_t cond;

    while (1)
    {
        cond = oldVal & 0xff000000;
        condExpected = *expected & 0x00ffffff;
        condExpected |= cond;
        newVal = desired & 0x00ffffff;
        newVal |= cond;

        if (atomic_compare_exchange_strong(object, &condExpected, newVal))
        {
            return true;
        }

        if ((condExpected & 0xff000000) == cond)
        {
            *expected = condExpected & 0x00ffffff;
            return false;
        }
    }
}

bool _condcounter_compare_exchange64(volatile condCounter64_t* object, int64_t* expected, int64_t desired)
{
    int64_t oldVal = *object;
    int64_t newVal;
    int64_t condExpected;
    int64_t cond;

    while (1)
    {
        cond = oldVal & 0xffff000000000000;
        condExpected = *expected & 0x0000ffffffffffff;
        condExpected |= cond;
        newVal = desired & 0x0000ffffffffffff;
        newVal |= cond;

        if (atomic_compare_exchange_strong(object, &condExpected, newVal))
        {
            return true;
        }

        if ((condExpected & 0xffff000000000000) == cond)
        {
            *expected = condExpected & 0x0000ffffffffffff;
            return false;
        }
    }
}
