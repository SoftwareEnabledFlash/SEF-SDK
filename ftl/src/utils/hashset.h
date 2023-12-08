/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * hashset.h
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
#ifndef HASHSET_H
#define HASHSET_H

#include <stdint.h>

typedef struct HashSet_ *HashSet;

/**
 *  @brief      Initializes the hashSet and allocates needed memory
 *
 *  @param      hashSet        hashSet handle to be used for access the hashSet
 */
void HashSetInit(HashSet *hashSet);

/**
 *  @brief      cleans up the hashSet and de-allocates the used memory
 *
 *  @param      hashSet        hashSet handle to be used for access the hashSet
 */
void HashSetCleanup(HashSet hashSet);

/**
 *  @brief      Adds a new element to the hashSet
 *
 *  @param      hashSet        hashSet handle to be used for access the hashSet
 *  @param      element        the element to be stored
 */
void HashSetAdd(HashSet hashSet, uint64_t element);

/**
 *  @brief      Returns the number of elements stored in the hashSet
 *
 *  @param      hashSet        hashSet handle to be used for access the hashSet
 *
 *  @return     Returns the number of elements stored in the hashSet
 */
int HashSetCount(HashSet hashSet);

/**
 *  @brief      Returns if the element is new or already inserted
 *
 *  @param      hashSet        hashSet handle to be used for access the hashSet
 *  @param      element        the element to be stored
 *
 * @retval  1    New element
 * @retval  0    Previously inserted element
 */
int HashSetIsNew(HashSet hashSet, uint64_t element);

void HashSetGetElems(HashSet hashSet, uint64_t *buff, int bufferSize);

/**
 *  @brief      This function is used as a helper to iterate over the elements inserted in the hash set
 *
 *  @param      hashSet         hashSet handle to be used for access the hashSet
 *  @param      data            A void pointer to be passed to the called function
 *  @param      callback        The function called for each data stored
 */
void HashSetForEach(HashSet hashSet, void *data, void *callback(uint64_t element, void *data));

#endif /* HASHSET_H */
