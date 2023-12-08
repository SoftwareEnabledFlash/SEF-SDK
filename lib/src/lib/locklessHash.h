/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * locklessHash.h
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
#ifndef LOCKLESSHASH_H
#define LOCKLESSHASH_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// lockless hash table with 32 bit unsigned key and a pointer value.
// NOTE: there are two reserved keys and one value that cannot be supplied by the user:
//    key = 0 is used to indicate an empty hash table entry internally
//    key = 0xffffffff is used to indicate that a table is resizing and no new
//          elements will be inserted in it.
//    value - 0 (NULL) is used to indicate an empty hash table entry

typedef void* LHThandle;

LHThandle LHTcreate(void);
void LHTdestroy(LHThandle handle);

void* LHTget(LHThandle handle, uint32_t lookupKey);
void* LHTput(LHThandle handle, uint32_t lookupKey, void* value);
void* LHTputIfEmpty(LHThandle handle, uint32_t lookupKey, void* value);
void* LHTdelete(LHThandle handle, uint32_t lookupKey);
void LHTdump(LHThandle handle, int printData);
void LHTforeach(LHThandle handle, void (*func)(void*, void*), void* param);

#ifdef __cplusplus
}
#endif
#endif
