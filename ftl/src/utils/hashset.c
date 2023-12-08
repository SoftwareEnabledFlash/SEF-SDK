/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * hashset.c
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
#include "hashset.h"

#include <stdint.h>
#include <stdlib.h>

#include "../config.h"
#include "../sef-utils.h"

#define TABLESIZE 257

struct HashNode
{
    uint64_t data;
    struct HashNode *next;
};

struct HashSet_
{
    struct HashNode *table[TABLESIZE];
    int count;
};

void HashSetInit(HashSet *hashSet)
{
    int i;
    *hashSet = SUmalloc(sizeof(struct HashSet_));

    (*hashSet)->count = 0;
    for (i = 0; i < TABLESIZE; i++)
    {
        (*hashSet)->table[i] = NULL;
    }
}

void HashSetCleanup(HashSet hashSet)
{
    int i;
    struct HashNode *node, *nextNode;

    // empty the table
    for (i = 0; i < TABLESIZE; i++)
    {
        node = hashSet->table[i];

        while (node != NULL)
        {
            nextNode = node->next;

            SUfree(node);

            node = nextNode;
        }
    }

    SUfree(hashSet);
}

void CreateHashNode(struct HashNode **node, uint64_t element)
{
    *node = SUmalloc(sizeof(struct HashNode));

    (*node)->data = element;
    (*node)->next = NULL;
}

void HashSetAdd(HashSet hashSet, uint64_t element)
{
    int cell;
    struct HashNode *node, *newNode;

    if (!HashSetIsNew(hashSet, element))
    {
        return;
    }

    cell = element % TABLESIZE;
    if (hashSet->table[cell] == NULL)
    {
        CreateHashNode(&newNode, element);

        hashSet->table[cell] = newNode;
        hashSet->count++;
        return;
    }

    // insert at end of chain
    node = hashSet->table[cell];
    while (node->next != NULL)
    {
        node = node->next;
    }

    CreateHashNode(&newNode, element);
    node->next = newNode;
    hashSet->count++;
}

int HashSetCount(HashSet hashSet)
{
    return hashSet->count;
}

int HashSetIsNew(HashSet hashSet, uint64_t element)
{
    int cell;
    struct HashNode *node;

    cell = element % TABLESIZE;
    if (hashSet->table[cell] == NULL)
    {
        return 1;
    }

    node = hashSet->table[cell];
    while (node != NULL)
    {
        if (node->data == element)
        {
            return 0;
        }

        node = node->next;
    }

    return 1;
}

void HashSetGetElems(HashSet hashSet, uint64_t *buff, int bufferSize)
{
    int i, addedNodes, maximumNodes;
    struct HashNode *node;

    maximumNodes = bufferSize / sizeof(uint64_t);
    addedNodes = 0;
    for (i = 0; i < TABLESIZE; i++)
    {
        node = hashSet->table[i];

        while (node != NULL)
        {
            if (addedNodes >= maximumNodes)
            {
                return;
            }

            buff[addedNodes] = node->data;
            addedNodes++;

            node = node->next;
        }
    }
}

void HashSetForEach(HashSet hashSet, void *data, void *callback(uint64_t element, void *data))
{
    int i;
    struct HashNode *node;

    for (i = 0; i < TABLESIZE; i++)
    {
        node = hashSet->table[i];

        while (node != NULL)
        {
            callback(node->data, data);
            node = node->next;
        }
    }
}
