/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * condcounter.h
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
#include <stdbool.h>
#include <stdint.h>

#if defined(_WIN32)
#include "stdatomic.h"
#endif

//
// concounter : conditional atomic operations on a counter.
//
// Most significant bits of an atomic counter are used to store a condition variable, which controls if the atomic operation
// will be performed. Useful for both conditional flags and states for state machines.
//
// The number of bits used for the condition variable varies by size of the atomic and reduces the counter's range.
//
// 8-bit atomics reserve one bit and have a range of -63 to 63
// 16-bit atomics reserve two bits and have a range of -8,192 to 8,191
// 32-bit atomics reserve a byte and have a range of -8,388,608 to 8,388,607
// 64-bit atomics reserve two bytes and have a range of -140,737,488,355,328 to 140,737,488,355,327
//
// NOTE: Not all counter operations defined yet, and not all defined are implemented in the .c file...
// if you require an implementation or additional operation, use the exiting ones as examples. They
// are all very similar...

typedef _Atomic(int8_t) condCounter8_t;
typedef _Atomic(int16_t) condCounter16_t;
typedef _Atomic(int32_t) condCounter32_t;
typedef _Atomic(int64_t) condCounter64_t;

#define condcounter_cond_add(object, cond, operand)                                                      \
    ((sizeof(*(object)) == 1)                                                                            \
         ? _condcounter_cond_add8((volatile condCounter8_t*)(object), (int8_t)(cond), (int8_t)(operand)) \
         : ((sizeof(*(object)) == 2)                                                                     \
                ? _condcounter_cond_add16((volatile condCounter16_t*)(object), (int16_t)(cond),          \
                                          (int16_t)(operand))                                            \
                : ((sizeof(*(object)) == 4)                                                              \
                       ? _condcounter_cond_add32((volatile condCounter32_t*)(object),                    \
                                                 (int32_t)(cond), (int32_t)(operand))                    \
                       : _condcounter_cond_add64((volatile condCounter64_t*)(object),                    \
                                                 (int64_t)(cond), (int64_t)(operand)))))

extern bool _condcounter_cond_add8(volatile condCounter8_t* object, int8_t cond, int8_t operand);
extern bool _condcounter_cond_add16(volatile condCounter16_t* object, int16_t cond, int16_t operand);
extern bool _condcounter_cond_add32(volatile condCounter32_t* object, int32_t cond, int32_t operand);
extern bool _condcounter_cond_add64(volatile condCounter64_t* object, int64_t cond, int64_t operand);

#define condcounter_sub(object, operand)                                                             \
    ((sizeof(*(object)) == 1)                                                                        \
         ? _condcounter_sub8((volatile condCounter8_t*)(object), (int8_t)(operand))                  \
         : ((sizeof(*(object)) == 2)                                                                 \
                ? _condcounter_sub16((volatile condCounter16_t*)(object), (int16_t)(operand))        \
                : ((sizeof(*(object)) == 4)                                                          \
                       ? _condcounter_sub32((volatile condCounter32_t*)(object), (int32_t)(operand)) \
                       : _condcounter_sub64((volatile condCounter64_t*)(object), (int64_t)(operand)))))

extern int8_t _condcounter_sub8(volatile condCounter8_t* object, int8_t operand);
extern int16_t _condcounter_sub16(volatile condCounter16_t* object, int16_t operand);
extern int32_t _condcounter_sub32(volatile condCounter32_t* object, int32_t operand);
extern int64_t _condcounter_sub64(volatile condCounter64_t* object, int64_t operand);

#define condcounter_condition_get(object)                                                  \
    ((sizeof(*(object)) == 1)                                                              \
         ? _condcounter_condition_get8((volatile condCounter8_t*)(object))                 \
         : ((sizeof(*(object)) == 2)                                                       \
                ? _condcounter_condition_get16((volatile condCounter16_t*)(object))        \
                : ((sizeof(*(object)) == 4)                                                \
                       ? _condcounter_condition_get32((volatile condCounter32_t*)(object)) \
                       : _condcounter_condition_get64((volatile condCounter64_t*)(object)))))

extern int8_t _condcounter_condition_get8(volatile condCounter8_t* object);
extern int16_t _condcounter_condition_get16(volatile condCounter16_t* object);
extern int32_t _condcounter_condition_get32(volatile condCounter32_t* object);
extern int64_t _condcounter_condition_get64(volatile condCounter64_t* object);

#define condcounter_counter_get(object)                                                  \
    ((sizeof(*(object)) == 1)                                                            \
         ? _condcounter_counter_get8((volatile condCounter8_t*)(object))                 \
         : ((sizeof(*(object)) == 2)                                                     \
                ? _condcounter_counter_get16((volatile condCounter16_t*)(object))        \
                : ((sizeof(*(object)) == 4)                                              \
                       ? _condcounter_counter_get32((volatile condCounter32_t*)(object)) \
                       : _condcounter_counter_get64((volatile condCounter64_t*)(object)))))

extern int8_t _condcounter_counter_get8(volatile condCounter8_t* object);
extern int16_t _condcounter_counter_get16(volatile condCounter16_t* object);
extern int32_t _condcounter_counter_get32(volatile condCounter32_t* object);
extern int64_t _condcounter_counter_get64(volatile condCounter64_t* object);

#define condcounter_condition_exchange(object, desired)                                            \
    ((sizeof(*(object)) == 1)                                                                      \
         ? _condcounter_condition_exchange8((volatile condCounter8_t*)(object), (int8_t)(desired)) \
         : ((sizeof(*(object)) == 2)                                                               \
                ? _condcounter_condition_exchange16((volatile condCounter16_t*)(object),           \
                                                    (int16_t)(desired))                            \
                : ((sizeof(*(object)) == 4)                                                        \
                       ? _condcounter_condition_exchange32((volatile condCounter32_t*)(object),    \
                                                           (int32_t)(desired))                     \
                       : _condcounter_condition_exchange64((volatile condCounter64_t*)(object),    \
                                                           (int64_t)(desired)))))

extern int8_t _condcounter_condition_exchange8(volatile condCounter8_t* object, int8_t desired);
extern int16_t _condcounter_condition_exchange16(volatile condCounter16_t* object, int16_t desired);
extern int32_t _condcounter_condition_exchange32(volatile condCounter32_t* object, int32_t desired);
extern int64_t _condcounter_condition_exchange64(volatile condCounter64_t* object, int64_t desired);

#define condcounter_compare_exchange(object, expected, desired)                                    \
    (sizeof(*(object)) == 1                                                                        \
         ? _condcounter_compare_exchange8((volatile condCounter8_t*)(object), (int8_t*)(expected), \
                                          (int8_t)(desired))                                       \
         : (sizeof(*(object)) == 2                                                                 \
                ? _condcounter_compare_exchange16((volatile condCounter16_t*)(object),             \
                                                  (int16_t*)(expected), (int16_t)(desired))        \
                : (sizeof(*(object)) == 4                                                          \
                       ? _condcounter_compare_exchange32((volatile condCounter32_t*)(object),      \
                                                         (int32_t*)(expected), (int32_t)(desired)) \
                       : _condcounter_compare_exchange64((volatile condCounter64_t*)(object),      \
                                                         (int64_t*)(expected), (int64_t)(desired)))))

extern bool _condcounter_compare_exchange8(volatile condCounter8_t* object,
                                           int8_t* expected,
                                           int8_t desired);
extern bool _condcounter_compare_exchange16(volatile condCounter16_t* object,
                                            int16_t* expected,
                                            int16_t desired);
extern bool _condcounter_compare_exchange32(volatile condCounter32_t* object,
                                            int32_t* expected,
                                            int32_t desired);
extern bool _condcounter_compare_exchange64(volatile condCounter64_t* object,
                                            int64_t* expected,
                                            int64_t desired);
